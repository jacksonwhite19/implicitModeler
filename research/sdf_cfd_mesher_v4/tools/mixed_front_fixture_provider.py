"""Qualified local geometry provider for the native mixed-front rolling loop.

Uses existing SDF fixture, cavity, transition and exact-validation machinery.
No reference remesh output is read during construction.
"""
import json,time,subprocess
from validation_row_stream import validation_digest
from pathlib import Path
import generalize_local_remesh as g
from exercise_mixed_remesh_transactions import mesh as template_mesh
from sdf_cfd_mesher_v4.advancing_front.mixed_front import from_mesh,scientific_hash,rebuild_dependencies
from sdf_cfd_mesher_v4.advancing_front.mixed_evidence import attach_validation
from sdf_cfd_mesher_v4.advancing_front.termination_policy import REMESH_CHECKS,IMMEDIATE_CHECKS
from sdf_cfd_mesher_v4.common.authority import identity,file_hash


def outer_failure_columns(mesh,certificate):
    """Outer face indices are mutable; source columns belong to owning cells."""
    owners={}
    for cell in mesh['cells']:
        for face in cell['faces']:owners.setdefault(tuple(sorted(face)),set()).add(int(cell['column']))
    columns=set()
    for failure in certificate['failures']:
        for index in failure['pair']:
            face=tuple(sorted(mesh['outer'][index]));bound=owners.get(face,set())
            if len(bound)!=1:raise ValueError('UnboundOuterFailureAncestry')
            columns.update(bound)
    return sorted(columns)


def release_rejected_proofs(proposal):
    """Retire an invalid candidate's expanded proofs after policy evaluation.

    Replacement validation reuses only committed proofs. Keep the rejected
    candidate's coverage/hash and failed witnesses; its full passing population
    is no longer a scientific input to discovery or the transaction.
    """
    checked=proposal['check']
    if checked.get('passed') is True:return None
    certificate=checked.get('certificate',{})
    if not certificate.get('proofs'):return None
    record=dict(proof_hash=certificate['proof_hash'],required_pairs=certificate['required_pairs'],
                retired_expanded_records=len(certificate['proofs']),failed_witnesses=len(certificate.get('failures',[])))
    del certificate['proofs']
    return record


class Provider:
    def __init__(self,spec,folder,geometry=None):
        self.spec=spec;self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.geometry_source=geometry or g.fixtures
        self.x,self.tri=self.geometry_source.build(spec);self.schedule=g.q.load_schedule()
        self.authority=dict(schedule=g.q.array_hash(self.schedule.targets),quality=file_hash(Path(g.t.__file__)),certificate=file_hash(Path(g.s.cert.__file__)))
        self.records=[];self.times={};self.checked={};self.meshes={};self.numerical={}
        self.reuse_certificates=False
        self.previous_certificate_state=None

    def pack(self,fronts,levels,previous=None,eligible=None):
        mesh=template_mesh(self.x,self.tri,fronts,levels)
        # Only fully exposed prism tops remain eligible. Transition ownership
        # belongs to the outer interface, not the prism extrusion population.
        tops={tuple(sorted(f)) for f in mesh['outer']}
        last={}
        for c in mesh['cells']:
            if c['kind']=='prism' and tuple(sorted(c['vertices'][3:])) in tops:
                last[c['column']]=c['layer']
        if eligible is None and previous is not None:
            eligible={int(k) for k in previous['prism_front']}
        if eligible is not None:last={k:v for k,v in last.items() if k in eligible}
        state=from_mesh(mesh,self.authority,active_columns=last,previous=previous)
        self.meshes[identity(state['cells'])]=mesh
        return state

    def geometry(self,state,extra=0):
        depth=[0]*len(self.x)
        for key in state['vertices']:
            _,layer,_,source=key.split(':');depth[int(source)]=max(depth[int(source)],int(layer))
        maximum=max(depth)
        fronts=[self.geometry_source.front(self.spec,v) for v in self.schedule.targets[:maximum+extra]]
        for key,p in state['vertices'].items():
            _,layer,_,source=key.split(':')
            if int(layer):fronts[int(layer)-1][int(source)]=p
        return fronts,g.np.array(depth)

    def initialize(self):
        fronts=[];scan=[]
        for k,target in enumerate(self.schedule.targets,1):
            fronts.append(self.geometry_source.front(self.spec,target))
            _,_,_,check,_=g.validation(self.x,self.tri,fronts,[k]*len(self.x),False)
            sdf=self.geometry_source.sdf_record(self.spec,fronts[-1],target)
            scan.append(dict(layer=k,quality=check['quality']['passed'],sdf=sdf))
            if not check['quality']['passed'] or not sdf['admitted']:break
        _,_,_,checked,_=g.validation(self.x,self.tri,fronts[:-1],[k-1]*len(self.x))
        if not checked['passed']:raise ValueError('SupportBlocked')
        state=self.pack(fronts[:-1],[k-1]*len(self.x))
        self.bind(state,checked)
        rebuild_dependencies(None,state,self.schedule.targets.tolist())
        g.save(self.folder/'INITIAL_SCAN.json',scan)
        return state

    def bind(self,state,checked,previous=None):
        from validation_row_stream import normalized_validation
        checked=normalized_validation(checked)
        return attach_validation(state,self.meshes[identity(state['cells'])],checked,previous,
            receipt_directory=self.folder/'checkpoints'/'objects')

    def propose(self,state):
        from terminal_capacity import prevent_terminal_advance
        prevent_terminal_advance(state)
        self.previous_certificate_state=state if self.reuse_certificates else None
        start=time.perf_counter();fronts,levels=self.geometry(state,1);k=len(fronts)
        if state['events']:
            # Existing qualified bounded continuation selector; its accepted
            # candidate is committed by the same rolling runner on this step.
            events=list(state['events'].values())
            seed=set(events[-1]['cavity_manifest']['seed']['vertices'])
            continuation=g.continue_local(self.spec,self.x,self.tri,fronts[:-1],levels,seed,geometry=self.geometry_source,complete_validator=getattr(self,"continuation_validator",None),front_provider=getattr(self,"continuation_front",None),eligible=state["prism_front"])
            if not continuation['passed']:raise ValueError('NeighborContinuationFailure')
            chosen=next(a for a in reversed(continuation['attempts']) if a.get('passed'))
            if 'front' in continuation:fronts[-1]=g.np.asarray(continuation['front'])
            if 'fronts' in continuation:fronts=[g.np.asarray(a) for a in continuation['fronts']];k=continuation['layer']
            trial=g.np.asarray(continuation['levels']) if 'levels' in continuation else levels.copy()
            trial[self.tri[chosen['column']]]=k
            checked=chosen['result'];self.records.append(dict(continuation=continuation))
        else:
            trial=levels.copy()
            for source in state['prism_front']:
                trial[self.tri[int(source)]]=k
            _,_,_,checked,_=g.validation(self.x,self.tri,fronts,trial,False)
            sdf=self.geometry_source.sdf_record(self.spec,fronts[-1],self.schedule.targets[k-1])
            if checked['quality']['passed'] and sdf['admitted']:
                if not self.reuse_certificates:
                    _,_,_,checked,_=g.validation(self.x,self.tri,fronts,trial,True)
        candidate=self.pack(fronts,trial,state)
        if self.reuse_certificates and not checked['passed'] and checked['quality']['passed'] and not state['events'] and sdf['admitted']:
            from reuse_mixed_certificate import certify
            m=self.meshes[identity(candidate['cells'])];p=g.np.asarray(m['points']);c=m['cells'];o=m['outer']
            owners,_=g.t.ownership(c,o,[tuple(reversed(f)) for f in self.tri]);faces=[owners[key][0][1] for key in sorted(owners)]
            cert=certify(p,faces,candidate,state);volume=g.volume.audit(p,c,o,[tuple(map(int,f)) for f in self.tri])
            checked.update(certificate=cert,volume=volume,passed=cert['passed'] and volume['exact_volume_balance'] and volume['positive_cells'] and volume['interior_degree']['passed'])
        self.times['proposal']=time.perf_counter()-start
        return dict(state=candidate,fronts=fronts,levels=trial,check=checked,layer=k)

    def evidence(self,state,proposal):
        context=dict(base=scientific_hash(state),region=identity(state['prism_front']),layer=proposal['layer'],
                     topology=identity(state['faces']),schedule=self.authority['schedule'])
        checked=proposal['check'];ref=dict(context=context,proof=validation_digest(checked))
        valid=checked['passed'];seed={};witnesses=[]
        if not valid:
            m=self.meshes[identity(proposal['state']['cells'])]
            exact=g.s.cert.certify(g.np.array(m['points']),m['outer'],range(len(m['outer'])))
            vertices,columns=g.failure_vertices(self.tri,checked['quality'])
            columns=sorted(set(columns)|set(outer_failure_columns(m,exact)))
            vertices.update(int(v) for column in columns for v in self.tri[column])
            seed=dict(vertices=sorted(vertices),columns=columns,cells=[proposal['layer']*1000000+c for c in columns],faces=[])
            # This existing witness is specifically the exact sphere-radius
            # capacity proof. Other SDF backends must not inherit its inputs.
            capacity=self.spec.get('kind')=='spheres' and self.spec.get('cavity') and any(g.t.F(float(r))-g.t.F(float(self.schedule.targets[proposal['layer']-1]))<=0 for r in self.spec['radii'])
            if not exact['failures'] and not capacity:raise ValueError('OrdinaryCorrectionProviderRequired')
            witnesses=[dict(ref,scope='immediate',qualified=True,kind='QualifiedCapacityLimit' if capacity else 'ExactCollision',seed=seed)]
        evidence=dict(context=context,support=dict(ref,status='clean'),immediate=dict(ref,status='valid' if valid else 'invalid',seed=seed),
                      warnings=[] if valid else [ref['proof']],attempts=[],witnesses=witnesses,
                      accepted=dict(ref,stage='NominalAdvance',checks={k:True for k in IMMEDIATE_CHECKS}) if valid else None)
        return evidence

    def candidate_check(self,points,cells,outer):
        from coupled_shell_validation import check
        return check(points,cells,outer)

    def discover(self,before,proposal,decision):
        start=time.perf_counter()
        rss_before=g.q.psutil.Process().memory_info().rss
        retired=release_rejected_proofs(proposal)
        if retired is not None:
            retired.update(rss_before=rss_before,rss_after=g.q.psutil.Process().memory_info().rss,
                           scope='Rejected proposal only; evidence hash and failed witnesses retained; no committed proof retired')
            g.save(self.folder/f'REJECTED_PROOF_LIFETIME_{proposal["layer"]}_{retired["proof_hash"]}.json',retired)
        levels,expansion=g.discover(self.x,self.tri,proposal['fronts'],set(decision['seed']['vertices']),proposal['layer'],base_levels=proposal['levels'],candidate_check=self.candidate_check)
        g.save(self.folder/f'DISCOVERY_{proposal["layer"]}.json',dict(levels=None if levels is None else levels.tolist(),expansion=expansion,seed=decision['seed']))
        if levels is None:raise ValueError('CavityDiscoveryFailure')
        state=self.pack(proposal['fronts'],levels,before)
        removed=sorted(k for k,c in before['cells'].items() if state['cells'].get(k)!=c)
        result=dict(removed_cells=removed,levels=levels.tolist(),expansion=expansion,seed=decision['seed'],boundary=identity(before['outer']))
        self.times['cavity']=time.perf_counter()-start
        return result

    def remesh(self,before,private,proposal,cavity):
        requested=proposal.get('levels',[len(proposal['fronts'])]*len(self.x))
        eligible={int(i) for i in before['prism_front'] if all(cavity['levels'][v]==requested[v] for v in self.tri[int(i)])}
        return self.pack(proposal['fronts'],cavity['levels'],before,eligible)

    def correct(self,before,proposal,decision):
        if not proposal['check']['passed']:raise ValueError('OrdinaryCorrectionProviderRequired')
        return proposal['state']

    def numerical_validation(self,state,proposal,cavity):
        from coupled_shell_validation import check
        start=time.perf_counter();fronts,levels=self.geometry(state)
        p,c,o,checked,times=g.validation(self.x,self.tri,fronts,levels,False)
        if not (checked['quality']['passed'] and checked['ownership']['passed'] and checked['vertex_links']['passed']):
            raise ValueError('NumericalMixedValidationFailure')
        coupled=check(p,c,o,True)
        if not coupled['passed']:raise ValueError('CompleteNeighborMixedValidationFailure')
        if coupled.get('scope')!='AllPrismUnchanged':
            checked['complete_neighbor']=coupled
            g.save(self.folder/f'COUPLED_{proposal["layer"]}.json',coupled)
        key=scientific_hash(state)
        self.numerical[key]=(p,c,o,checked,times,fronts,levels)
        self.times['numerical_validation']=time.perf_counter()-start
        return dict(state=key,passed=True,proof=validation_digest(checked))

    def validate(self,state,proposal,cavity):
        self.numerical_validation(state,proposal,cavity)
        return self.certify_validation(state,proposal,cavity)

    def assigned_targets(self,state,layer,source_indices):
        schedules=state.get('local_schedule',{}).get('actual_targets',{})
        if str(layer) in schedules:return g.np.asarray(schedules[str(layer)])[source_indices]
        return self.schedule.targets[layer-1]

    def certify_validation(self,state,proposal,cavity):
        start=time.perf_counter();fronts,levels=self.geometry(state)
        key=scientific_hash(state)
        if key not in self.numerical:raise ValueError('MissingBoundNumericalValidation')
        p,c,o,checked,times,fronts,levels=self.numerical.pop(key)
        from coupled_shell_validation import qualify_closures
        qualify_closures(self.folder,proposal['layer'],p,c,checked)
        owners,_=g.t.ownership(c,o,[tuple(reversed(f)) for f in self.tri])
        faces=[owners[k][0][1] for k in sorted(owners)]
        tick=time.perf_counter()
        reuse_proposal=self.reuse_certificates and scientific_hash(proposal['state'])==key and proposal['check'].get('passed') and 'certificate' in proposal['check']
        if reuse_proposal:cert=proposal['check']['certificate']
        elif self.reuse_certificates:
            from reuse_mixed_certificate import certify
            cert=certify(p,faces,state,self.previous_certificate_state)
        else:cert=g.s.cert.certify(p,faces,range(len(faces)))
        times['exact_certificate']=time.perf_counter()-tick
        tick=time.perf_counter()
        # The same scientific-state equality already required for proof reuse
        # also binds the candidate's exact volume/degree audit. A changed state
        # or a candidate without that complete audit still runs the validator.
        volume=(proposal['check']['volume'] if reuse_proposal and 'volume' in proposal['check']
                else g.volume.audit(p,c,o,[tuple(map(int,f)) for f in self.tri]))
        times['volume_accounting']=time.perf_counter()-tick
        checked.update(certificate=cert,volume=volume,passed=cert['passed'] and volume['interior_degree']['passed'] and volume['exact_volume_balance'] and volume['positive_cells'])
        if not checked['passed']:raise ValueError('ExactMixedValidationFailure')
        ids=sorted({v for f in o for v in f});lookup={v:i+1 for i,v in enumerate(ids)}
        g.save(self.folder/'CORE_INGEST_INPUT.json',dict(coordinates=p[ids].tolist(),triangles=[[lookup[v] for v in f] for f in o],outer_hash=identity(o)))
        def wsl(path):return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\','/')
        subprocess.run(['wsl','--','python3',wsl(g.s.ROOT/'tools/cavity_core_ingestion_smoke.py'),wsl((self.folder/'CORE_INGEST_INPUT.json').resolve())],check=True,capture_output=True)
        core=json.loads((self.folder/'CORE_INGESTION.json').read_text())
        assigned=[]
        for layer in range(1,len(fronts)+1):
            used=sorted({v for cell in c for v in cell['vertices'] if v//len(self.x)==layer})
            if used:assigned.append(self.geometry_source.sdf_record(self.spec,p[used],self.assigned_targets(state,layer,g.np.asarray(used)%len(self.x))))
        from terminal_capacity import range_witness,terminal_state,remesh_admitted
        terminal=None
        witness=(range_witness(self.spec,float(self.schedule.targets[len(fronts)]),sorted(map(int,state['columns'])))
                 if cavity and len(fronts)<len(self.schedule.targets) and self.geometry_source.__name__=='geometry_family_authority' else None)
        if witness is not None:
            terminal=terminal_state(state,self.spec,float(self.schedule.targets[len(fronts)]),self.schedule.targets,witness)['local_schedule']['terminal_capacity']
            continuation=dict(passed=False,status='GlobalTerminalCapacityProven',projection_attempts=0)
        else:
            continuation=g.continue_local(self.spec,self.x,self.tri,fronts,levels,set(cavity['seed']['vertices']),geometry=self.geometry_source,complete_validator=getattr(self,"continuation_validator",None),front_provider=getattr(self,"continuation_front",None),eligible=state["prism_front"]) if cavity else {'passed':True}
        checks=dict(hard_quality=checked['quality']['passed'],exact_validity=all(r['jacobian_sign']>0 and r['volume_sign']>0 for r in checked['quality']['exact']),
                    ownership=checked['ownership']['passed'],continuous=checked['certificate']['passed'],
                    sdf_target=all(r['admitted'] for r in assigned),volume_conformity=checked['volume']['exact_volume_balance'] and checked['volume']['positive_cells'] and checked['volume']['interior_degree']['passed'],
                    outer_interface=checked['vertex_links']['passed'] and checked['ownership']['passed'],core_compatibility=core['passed'],
                    neighbor_continuation=continuation['passed'],wall_first_layer=state['wall']['vertices']=={str(i):v.tolist() for i,v in enumerate(self.x)},
                    outside_preserved=True) # Enforced by the journal before this callback.
        if not remesh_admitted(dict(checks=checks,terminal_capacity=terminal)):
            g.save(self.folder/f'REJECTED_ACCEPTANCE_{proposal["layer"]}.json',dict(checks=checks,sdf=assigned,continuation=continuation,state=key))
            raise ValueError('MixedRemeshAcceptanceFailed')
        if terminal is not None:
            updated=terminal_state(state,self.spec,terminal['target'],self.schedule.targets,witness)
            state.update(updated)
        self.checked[scientific_hash(state)]=checked
        self.times['validation']=time.perf_counter()-start
        self.times['validation_stages']=times
        return dict(state=scientific_hash(state),passed=True,proof=validation_digest(checked),checks=checks,terminal_capacity=terminal)

    def rebuild(self,before,state,receipt):
        start=time.perf_counter()
        bound=self.bind(state,self.checked[receipt['state']],before)
        coupled=self.checked[receipt['state']].get('complete_neighbor')
        if coupled is not None:
            from sdf_cfd_mesher_v4.advancing_front.complete_neighbor_receipt import bind_complete_neighbor
            bind_complete_neighbor(state,coupled)
        dependencies=rebuild_dependencies(before,state,self.schedule.targets.tolist())
        from receipt_retirement_archive import archive
        self.records.append(dict(receipts=archive(self.folder,bound),dependencies=dependencies))
        self.times['rebuild']=time.perf_counter()-start
        return state
