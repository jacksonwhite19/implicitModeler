"""In-memory ordinary append executor using the qualified production kernels.

Non-append transactions fail closed to the existing correction/remesh provider;
this module never interprets a failed nominal proposal as an accepted step.
"""
import json,time,subprocess
from pathlib import Path
from collections import Counter
import numpy as np
from incremental_layer_validation import Topology,Quality,ExactVolume,key,t
from run_geometry_family import AnalyticProvider
from sdf_cfd_mesher_v4.advancing_front.execution_profile import ExecutionProfile,MemoryTransaction
from sdf_cfd_mesher_v4.advancing_front.mixed_front import cycle,face_key,geometry_inputs,rebuild_dependencies
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
from native_proof_segments import LoadedSegment,SegmentProofRows
from compact_proof_rows import CompactProofRows


class MemorySegment:
    """Immutable compact proof delta; same row semantics as disk segments."""
    rows=LoadedSegment.rows

    def __init__(self,rows,faces,indices):
        pairs=np.frombuffer(rows.pairs,dtype=np.uint64).reshape(-1,2)
        used=np.unique(pairs)
        self.faces=[faces[int(i)] for i in used]
        self.pairs=np.searchsorted(used,pairs).astype(np.uint64)
        self.codes=np.frombuffer(rows.codes,dtype=np.uint64)
        self.payloads=rows.payloads
        self.mapping=np.asarray([indices[k] for k in self.faces],dtype=np.uint64)
        methods,bad=rows.summary()
        if bad:raise ValueError('InvalidMemoryProofSegment')
        self.descriptor=dict(count=len(rows),methods=dict(methods))


def native_append(before, cells, points, outer, changed):
    """Update native incidence only for replacement-free appended cells."""
    n=len(before['wall']['vertices'])
    def source(i):return f'layer:{i//n}:source:{i%n}'
    after=dict(before)
    after['_production_unsealed']=True
    after.update(vertices=dict(before['vertices']),cells=dict(before['cells']),faces=dict(before['faces']),
                 columns=dict(before['columns']),prism_front=dict(before['prism_front']),versions=dict(before['versions']))
    touched=set()
    for c in changed:
        k=str(c['id'])
        if k in after['cells']:raise ValueError('AppendReplacesCommittedCell')
        vertices=[source(v) for v in c['vertices']]
        faces=[list(cycle(tuple(source(v) for v in f))) for f in c['faces']]
        after['cells'][k]=dict(kind=c['kind'],source=str(c['column']),layer=c['layer'],vertices=vertices,faces=sorted(faces))
        for v,i in zip(vertices,c['vertices']):
            value=points[i].tolist()
            if v in before['vertices'] and before['vertices'][v]!=value:raise ValueError('CommittedCoordinateMutation')
            after['vertices'][v]=value
        for f in faces:
            fk=face_key(f);touched.add(fk)
            old=after['faces'].get(fk)
            row=dict(vertices=list(min(cycle(tuple(f)),cycle(tuple(reversed(f))))),owners={} if old is None else dict(old['owners']))
            row['owners'][k]=f;after['faces'][fk]=row
    after['outer']=sorted(face_key(tuple(source(v) for v in f)) for f in outer)
    ownership={}
    for f in after['outer']:
        for owner in after['faces'][f]['owners']:
            ownership.setdefault(after['cells'][owner]['source'],[]).append(f)
    for c in changed:
        column=str(c['column']);k=str(c['id']);tops=[f for f in ownership.get(column,[]) if k in after['faces'][f]['owners'] and len(after['faces'][f]['vertices'])==3]
        if len(tops)!=1:raise ValueError('ActiveColumnHasNoUniqueExposedPrismTop')
        after['columns'][column]=dict(status='ActivePrismColumn',last_prism_layer=c['layer'],exposed_prism_face=tops[0],outer_ownership=ownership[column])
        after['prism_front'][column]=tops[0]
    # Match the existing version convention; unchanged geometry is never hashed.
    for family,keys in [('cell',[str(c['id']) for c in changed]),('face',touched)]:
        table=family+'s'
        for k in keys:
            dep=family+':'+k;old=before['versions'].get(dep,dict(topology=0,geometry=0))
            after['versions'][dep]=dict(topology=old['topology']+int(before[table].get(k)!=after[table][k]),geometry=old['geometry']+1)
    return after


def update_degree(state,points,boundary,witness,additions):
    """Keep the existing runner's rejected-nominal correction handoff."""
    try:return state.update(points,boundary,witness,additions)
    except ValueError as exc:
        if str(exc) in ('IncrementalHistoricalDegreeFailure','IncrementalNewDegreeFailure','InvalidBootstrapDegree'):
            raise ValueError('GlobalDegreeFailure') from exc
        raise


def archive_dependencies(run, before, state, topology, touched, additions, n):
    """Prepare receipts only for the existing archive/correction consumer.

    Rolling acceptance binds live mesh and numerical caches independently. It
    has no receipt renderer and discards these tables at commit. Do not build
    them, or invoke the archive schedule, on that branch. All nonrolling work
    and receipt inputs remain unchanged.
    """
    if run.rolling:
        return {}, {}
    def source(v):return f'layer:{v//n}:source:{v%n}'
    changed_face_keys={face_key(tuple(source(v) for v in topology.owners[f][0][1])) for f in touched}
    inputs=run.inputs.copy()
    inputs.update(geometry_inputs(dict(state,cells={str(c['id']):state['cells'][str(c['id'])] for c in additions},
                                      faces={k:state['faces'][k] for k in changed_face_keys})))
    vertex_incidence=run.vertex_incidence.copy()
    for k in changed_face_keys-before['faces'].keys():
        for v in state['faces'][k]['vertices']:
            vertex_incidence[v]=vertex_incidence.get(v,set())|{k}
    schedule=(run.provider.fast_dependency_schedule() if hasattr(run.provider,'fast_dependency_schedule')
              else run.provider.schedule.targets.tolist())
    rebuild_dependencies(before,state,schedule,dependency_inputs=inputs,vertex_incidence=vertex_incidence)
    return inputs, vertex_incidence


import stage_observations as obs

class FastLayer:
    rolling = False  # Compatibility for existing explicitly reconstructed owners.
    def __init__(self,state,spec,folder,profile=ExecutionProfile.ProductionFast,*,bound_provider=None):
        self.rolling = False
        self.profile=ExecutionProfile(profile)
        if self.profile is not ExecutionProfile.ProductionFast:
            raise ValueError('UseMixedFrontRunnerForDurableProfiles')
        if bound_provider is None:
            self.provider=AnalyticProvider(spec,folder)
        else:
            from accepted_fast_binding import AcceptedFastProvider
            if type(bound_provider) is not AcceptedFastProvider:
                raise ValueError('UnsupportedFastGeometryBinding')
            bound_provider.verify_snapshot(state,spec,folder)
            self.provider=bound_provider
        self.folder=Path(folder);self.spec=spec;self.tx=MemoryTransaction(state)
        if any(state['authority'].get(k)!=v for k,v in self.provider.authority.items()):raise ValueError('ChangedScientificImplementation')
        fronts,levels=self.provider.geometry(state)
        import generalize_local_remesh as g
        p,c,o=g.s.construct(self.provider.x,self.provider.tri,fronts,levels)
        self.wall=[tuple(reversed(f)) for f in self.provider.tri]
        owners,check=t.ownership(c,o,self.wall)
        if not check['passed']:raise ValueError('InvalidBootstrapOwnership')
        self.points=p;self.outer=o;self.topology=Topology(c,owners)
        self.quality=Quality.receipts(state,[r[0][1] for r in owners.values()]);self.quality.bind_masks(self.topology,o)
        self.volume=ExactVolume(p,c,o,[tuple(f) for f in self.provider.tri],self.quality.exact)
        from incremental_degree import DegreeState
        self.degree_state=DegreeState(p,self.wall+list(o),self.volume.witness)
        self.proofs=None;self.face_keys=None
        self.inputs=geometry_inputs(state)
        self.vertex_incidence={}
        for k,row in state['faces'].items():
            for v in row['vertices']:self.vertex_incidence.setdefault(v,set()).add(k)
        if hasattr(self.provider,'attach_session'):self.provider.attach_session(self.tx)

    def binding_state(self):
        """Current evidence for the unchanged general correction/remesh path.

        This is a rare conservative handoff. Ordinary commits do not construct
        these complete receipt bindings or run a full scientific hash.
        """
        if self.rolling:raise ValueError('RollingFastProdHasNoHistoricalReceiptRenderer')
        if self.proofs is None:return self.tx.current
        from live_proof_receipts import LiveProofReceipts
        state=dict(self.tx.current);state.pop('_production_unsealed',None);n=len(self.provider.x)
        # Preserve the existing adapter's exact receipt shape; proofs remain
        # streamed from compact segments rather than expanded into a dict.
        inputs=geometry_inputs(state);quality={}
        for i,row in self.quality.cells.items():
            c=state['cells'][str(i)];deps=['cell:'+str(i)]+['face:'+face_key(f) for f in c['faces']]+['authority:quality']
            quality['cell:'+str(i)]=dict(cell_type=c['kind'],inputs={k:inputs[k] for k in deps},value=row,exact=self.quality.exact[i])
        def source(v):return f'layer:{v//n}:source:{v%n}'
        for f,row in self.quality.faces.items():
            perimeter=self.topology.owners[f][0][1]
            fk=face_key(tuple(source(v) for v in perimeter));deps=['face:'+fk,'authority:quality']+['cell:'+str(i) for i in row['owners']]
            quality['face:'+fk]=dict(inputs={k:inputs[k] for k in deps},value=row)
        for i,row in self.quality.prospective.items():
            deps=['cell:'+str(i),'authority:quality']+['face:'+face_key(f) for f in state['cells'][str(i)]['faces']]
            quality['final_incidence:'+str(i)]=dict(inputs={k:inputs[k] for k in deps},value=row,
                receipt_class='FinalIncidenceDeterminant',final_core_quality=False)
        state['quality']=quality;state['certificate']=LiveProofReceipts(self.proofs,self.face_keys,inputs)
        return state

    @obs.observed('prism.step','prism')
    def step(self):
        start=time.perf_counter();timings={};before=self.tx.current;generation=self.tx.generation
        if self.rolling:self.live_acceptance.check(self)
        def mark(name,tick):timings[name]=time.perf_counter()-tick
        if before['events'] or any(c['kind']!='prism' for c in before['cells'].values()):
            raise ValueError('RequiresQualifiedMixedContinuationProvider')
        tick=time.perf_counter();fronts,levels=self.provider.geometry(before,1);layer=len(fronts)
        # This fast transaction handles a complete ordinary exposed cohort.
        # Other topology is handed back uncommitted, never approximated.
        if set(before['prism_front'])!=set(before['columns']) or np.any(levels!=layer-1):
            raise ValueError('RequiresQualifiedLocalCohortProvider')
        points=np.concatenate([self.provider.x]+fronts);n=len(self.provider.x)
        additions=[t.cell(np.r_[f+(layer-1)*n,f+layer*n],i,layer) for i,f in enumerate(self.provider.tri)]
        outer=[tuple(map(int,f+layer*n)) for f in self.provider.tri]
        mark('geometry',tick);tick=time.perf_counter()
        topology,touched=self.topology.update(additions);topology.boundary(outer,self.wall)
        state=native_append(before,topology.cells,points,outer,additions)
        mark('topology',tick);tick=time.perf_counter()
        from rolling_fast_state import _background_active,_background_pending
        terminal_owner=_background_active.get(self) if self.rolling else None
        terminal_pending=terminal_owner is not None and terminal_owner in _background_pending
        quality,counts=self.quality.update(points,topology,{c['id'] for c in additions},touched,outer,
            retain_geometry=terminal_pending,
            completed_front=self.provider.completed_front if self.rolling else False)
        mark('hard_quality',tick);tick=time.perf_counter()
        if hasattr(self.provider,'validate_fast_target'):
            sdf=self.provider.validate_fast_target(before,fronts[-1],layer)
        else:
            sdf=self.provider.geometry_source.sdf_record(self.spec,fronts[-1],self.provider.schedule.targets[layer-1])
        if not sdf['admitted']:raise ValueError('RequiresQualifiedOrdinaryCorrection')
        mark('sdf',tick);tick=time.perf_counter()
        volume=self.volume.update(points,additions,(),self.quality.exact,quality.exact,outer,[tuple(f) for f in self.provider.tri])
        mark('exact_volume_aggregate',tick);tick=time.perf_counter()
        degree_state,degree_metrics=update_degree(self.degree_state,points,self.wall+list(outer),volume.witness,additions)
        degree=degree_state.result();mark('incremental_degree',tick)
        tick=time.perf_counter();faces=[topology.owners[k][0][1] for k in sorted(topology.owners)]
        def source(v):return f'layer:{v//n}:source:{v%n}'
        keys=[face_key(tuple(source(v) for v in f)) for f in faces];indices={k:i for i,k in enumerate(keys)}
        if len(indices)!=len(keys):raise ValueError('DuplicateCertificateFace')
        if self.rolling:
            from rolling_fast_state import inherit
            inherit(before,state,additions)
            segments = None
        elif self.proofs is None:
            base=before['certificate']
            if hasattr(self.provider,'load_fast_segments'):
                segments=self.provider.load_fast_segments(indices)
            else:
                segments=[LoadedSegment(s,base.proof_objects,indices,True) for s in base.proof_segments]
            if sum(s.descriptor['count'] for s in segments)!=len(base):raise ValueError('IncompleteBaseProofPopulation')
        else:
            import copy
            segments=[]
            for old in self.proofs.segments:
                s=copy.copy(old);s.mapping=np.asarray([indices[k] for k in s.faces],dtype=np.uint64)
                if len(s.mapping)>1 and np.any(s.mapping[1:]<=s.mapping[:-1]):raise ValueError('ProofSegmentOrderChanged')
                segments.append(s)
            segments.append(MemorySegment(self.proofs.fresh,self.face_keys,indices))
        from native_mixed_face_certificate import certify
        fresh=certify(points,faces,[i for k,i in indices.items() if k not in before['faces']])
        if not fresh['passed']:raise ValueError('ContinuousGeometryFailure')
        if self.rolling:
            from rolling_fast_state import prepare
            rolling_quality,rolling_acceptance=prepare(self,before,state,points,topology,quality,volume,degree_state,outer,additions,
                fresh,faces,[i for k,i in indices.items() if k not in before['faces']])
            proofs=None
        else:
            proofs=SegmentProofRows(segments,fresh['proofs']);methods,bad=proofs.summary()
            if bad:raise ValueError('RetainedProofFailure')
        mark('continuous_certificate',tick);tick=time.perf_counter()
        ids=sorted({v for f in outer for v in f});lookup={v:i+1 for i,v in enumerate(ids)}
        path=self.folder/'CORE_INGEST_INPUT.json'
        path.write_text(json.dumps(dict(coordinates=points[ids].tolist(),triangles=[[lookup[v] for v in f] for f in outer],outer_hash=identity(outer))))
        def wsl(p):p=p.resolve();return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')
        from owned_runtime import V4
        subprocess.run(['wsl','--','python3',wsl(V4/'tools/cavity_core_ingestion_smoke.py'),wsl(path)],check=True,capture_output=True)
        if not json.loads((self.folder/'CORE_INGESTION.json').read_text())['passed']:raise ValueError('CoreIngestionFailure')
        mark('core_ingestion',tick);tick=time.perf_counter()
        inputs,vertex_incidence=archive_dependencies(self,before,state,topology,touched,additions,n)
        mark('dependencies',tick)
        from sdf_cfd_mesher_v4.advancing_front.termination_policy import LocalTerminationPolicy,IMMEDIATE_CHECKS
        context=dict(base=identity(dict(generation=generation,authority=before['authority'])),
                     region=identity(state['prism_front']),layer=layer,topology=identity(sorted(touched)),schedule=before['authority']['schedule'])
        ref=dict(context=context,proof=identity(dict(context=context,new_exact=fresh['proof_hash'],quality={k:v for k,v in counts.items() if k!='seconds'},degree=degree['passed'])))
        evidence=dict(context=context,support=dict(ref,status='clean'),immediate=dict(ref,status='valid',seed={}),
                      warnings=[],attempts=[],witnesses=[],accepted=dict(ref,stage='NominalAdvance',checks={k:True for k in IMMEDIATE_CHECKS}))
        decision=LocalTerminationPolicy.start(evidence).decision
        if decision['next_action']=='RollbackRemesh':raise ValueError('UnexpectedNominalRemeshDecision')
        # Candidate is private throughout. All metric/cache changes publish only
        # after every gate passes; exceptions retain the old generation/caches.
        prepared=None
        if hasattr(self.provider,'prepare_commit'):
            tick=time.perf_counter()
            prepared=self.provider.prepare_commit(before,state,generation,layer)
            mark('parent_rebinding',tick)
        tick=time.perf_counter()
        self.tx.commit(state,generation,lambda candidate:candidate is state and degree['passed'] and
                       (not hasattr(self.provider,'verify_prepared') or self.provider.verify_prepared(prepared,candidate,generation)))
        if prepared is not None:mark('private_commit',tick)
        self.topology=topology;self.quality=quality;self.volume=volume;self.points=points;self.outer=outer
        self.proofs=proofs;self.face_keys=None if self.rolling else keys
        self.degree_state=degree_state
        self.inputs=inputs;self.vertex_incidence=vertex_incidence
        if hasattr(self.provider,'finish_commit'):
            self.provider.finish_commit(prepared)
            self.spec=self.provider.spec
        if self.rolling:
            self.quality=rolling_quality;self.live_acceptance=rolling_acceptance
            if terminal_pending:
                from rolling_fast_state import _activate_producer_background
                _activate_producer_background(self,terminal_owner)
            # State-local mesh inputs are not a historical receipt archive.
            # Ordinary rolling mode never invokes the correction renderer.
            self.inputs={};self.vertex_incidence={}
        return dict(layer=layer,seconds=time.perf_counter()-start,timings=timings,counts=counts,
            relations=self.live_acceptance.count if self.rolling else len(proofs),new_relations=len(fresh['proofs']),cells=len(topology.cells),sdf_points=len(fronts[-1]),
            degree_passed=degree['passed'],exact_volume_balance=volume.total==volume.boundary_total,
            degree_metrics=degree_metrics,
            durability=('Process-local accepted mesh; no restart or archived scientific seal' if self.rolling
                        else 'Unsealed in-memory candidate; final seal required'),profile=self.profile.value,decision=decision)
