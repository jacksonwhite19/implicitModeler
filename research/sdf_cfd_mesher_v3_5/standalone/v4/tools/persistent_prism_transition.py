"""Layer-generic changed-geometry certification from CommittedState.

This module has no oracle reader and no legacy whole-state constructor.
Numerical mesh incidence is a global certification query, not a checkpoint.
"""
from collections import ChainMap, Counter
from fractions import Fraction
import json, time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front import cycle, face_key, geometry_inputs


def append_components(before, additions, points, n):
    """Keep old maps behind references; write only changed geometry/ownership."""
    def source(v): return f'layer:{v//n}:source:{v%n}'
    changes = {k: {} for k in ('vertices', 'cells', 'faces', 'columns', 'prism_front', 'versions')}
    for c in additions:
        k = str(c['id'])
        if k in before['cells']: raise ValueError('AppendReplacesCommittedCell')
        vs = [source(v) for v in c['vertices']]
        fs = [list(cycle(tuple(source(v) for v in f))) for f in c['faces']]
        changes['cells'][k] = dict(kind=c['kind'], source=str(c['column']), layer=c['layer'], vertices=vs, faces=sorted(fs))
        for v, i in zip(vs, c['vertices']):
            value = points[i].tolist()
            if v in before['vertices']:
                if before['vertices'][v] != value: raise ValueError('CommittedCoordinateMutation')
            else: changes['vertices'][v] = value
        for f in fs:
            fk = face_key(f)
            if fk not in changes['faces']:
                old = before['faces'].get(fk)
                changes['faces'][fk] = dict(vertices=list(min(cycle(f), cycle(tuple(reversed(f))))),
                                           owners={} if old is None else dict(old['owners']))
            changes['faces'][fk]['owners'][k] = f
        top = face_key(vs[3:]); column = str(c['column'])
        changes['columns'][column] = dict(status='ActivePrismColumn', last_prism_layer=c['layer'],
            exposed_prism_face=top, outer_ownership=[top])
        changes['prism_front'][column] = top
    for family in ('cell', 'face'):
        for k, row in changes[family+'s'].items():
            dep = family + ':' + k
            old = before['versions'].get(dep, dict(topology=0, geometry=0))
            changes['versions'][dep] = dict(topology=old['topology'] + int(before[family+'s'].get(k) != row),
                                           geometry=old['geometry'] + 1)
    state = {k: before[k] for k in before}
    state.update({k: ChainMap(v, before[k]) for k, v in changes.items()})
    state['outer'] = sorted(changes['prism_front'].values())
    inputs = geometry_inputs(dict(state, cells=changes['cells'], faces=changes['faces']))
    return state, changes, inputs


def produce_with_index(state, adapter, spec, output, packed, version_binding=None):
    from path_preserving_deduplication import durable_json, file_hash
    from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
    from geometry_family_authority import Source, sdf_record
    import local_termination_topology as t
    from final_shell_incidence import check as incidence_check
    from native_mixed_face_certificate import certify
    from native_proof_segments import persist_segment
    from incremental_layer_validation import ExactVolume
    import cavity_volume_audit as volume
    from composable_state import plan_transition
    plan = plan_transition(state, adapter)
    target = plan["target_layer"]; lower = plan["base_layer"]
    started = time.perf_counter(); phases = {}
    def mark(name, tick):
        phases[name] = time.perf_counter()-tick
        print(name, phases[name], flush=True)
        durable_json(output/'GENERATION_PHASES.json', phases)
    admission = json.loads((output/'ADMISSION.json').read_text())
    e = admission['evidence']
    if e['binding'] != adapter.binding or identity(e) != admission['evidence_identity']:
        raise ValueError('StaleAdmission')
    if identity(admission['decision']) != admission['decision_identity'] or not admission['decision']['generation_attempt_authorized']:
        raise ValueError('AdmissionNotAuthorized')
    for name, key in [('DIRECT.json','screen'), ('LOCALIZATION.json','localization')]:
        if file_hash(output/name) != e['authorities'][key]: raise ValueError('AdmissionEvidenceMutation')
    tick=time.perf_counter()
    prediction=predict_next_front(adapter.front.coordinates,adapter.schedule.targets[target-1],Source(spec),iteration_cap=256)
    high=prediction.coordinates
    assigned=sdf_record(spec,high,adapter.schedule.targets[target-1])
    if not assigned['admitted']: raise ValueError('GeneratedNextFrontSDFRejected')
    mark('projection',tick)
    with np.load(output/'SCREEN_ARRAYS.npz',allow_pickle=False) as forecast:
        provisional=forecast['provisional_high']
    delta=high-provisional
    tick=time.perf_counter(); n=len(high);tri=adapter.topology.triangles
    points=np.concatenate((*adapter.committed_prefix,high))
    additions=[t.cell(np.r_[f+lower*n,f+target*n],i,target) for i,f in enumerate(tri)]
    # Geometry is globally queried for incidence, spatial coverage and degree.
    # No old proof or quality population is loaded into this mesh view.
    from persistent_numeric_incidence import CellSequence,advance
    oldcells=CellSequence(packed);cells=oldcells+additions
    outer=[tuple(map(int,f+target*n)) for f in tri]
    owners,topology=advance(packed,additions,outer,[tuple(reversed(f)) for f in tri])
    links=t.vertex_links(outer)
    if not topology['passed'] or not links['passed']:raise ValueError('CandidateTopologyRejected')
    mark('geometry_and_global_incidence',tick)
    tick=time.perf_counter();active={c['id'] for c in additions}
    local={tuple(sorted(f)):owners[tuple(sorted(f))] for c in additions for f in c['faces']}
    needed={i for rows in local.values() for i,_ in rows}
    q=t.quality(points,cells.selected(needed),local,1,active_cells=active)
    prior={i for i in needed if i not in active}
    from prism_hotpath_v2 import completed_incidence
    incidence=completed_incidence(points,cells.selected(active|prior),outer,owners,active_cells=active|prior)
    det={r['id']:r['determinant'] for r in incidence['cells']}
    q['cells']=[dict(r,determinant=det[r['id']]) for r in q['cells']]
    q['failures']=[r for r in q['failures'] if r['kind']!='cell_quality']
    q['failures'] += [dict(kind='cell_quality',**r) for r in q['cells'] if not(r['volume']>0 and r['determinant']>=.0011 and r['aspect']<=1000)]
    q['passed']=not q['failures']
    durable_json(output/'NEW_QUALITY.json',q)
    durable_json(output/'AFFECTED_INCIDENCE.json',incidence)
    if not q['passed'] or not incidence['passed']:raise ValueError('CandidateHardQualityRejected')
    mark('new_quality_and_completed_incidence',tick)
    tick=time.perf_counter()
    import composable_transition
    before=state
    if version_binding is not None:
        from collections.abc import Mapping
        class Absent(Mapping):
            def __init__(self,source,keys):self.source=source;self.absent=keys
            def __getitem__(self,key):
                if key in self.absent:raise KeyError(key)
                return self.source[key]
            def __contains__(self,key):return key not in self.absent and key in self.source
            def __iter__(self):return iter(self.source)
            def __len__(self):return len(self.source)
        new_cells={str(c['id']) for c in additions}
        for segment in packed.cell_segments:
            ids=np.asarray(segment[:,0]);queries=np.array(sorted(map(int,new_cells)),dtype=np.int64)
            positions=np.searchsorted(ids,queries);valid=positions<len(ids)
            if np.any(ids[positions[valid]]==queries[valid]):raise ValueError('AppendReplacesCommittedCell')
        new_faces={face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in owners.delta[key][0][1])) for key in owners.created_keys}
        missing={'cells':new_cells,'faces':new_faces,'versions':{'cell:'+k for k in new_cells}|{'face:'+k for k in new_faces}}
        before={k:Absent(state[k],missing[k]) if k in missing else state[k] for k in state}
    after,changes,inputs=composable_transition.append_components(before,additions,points,n)
    if version_binding is not None and set(changes['versions'])!=({'cell:'+k for k in changes['cells']}|{'face:'+k for k in changes['faces']}):
        raise ValueError('ChangedVersionClosure')
    durable_json(output/'GEOMETRY_DELTA.json',changes)
    durable_json(output/'CHANGED_INPUTS.json',inputs)
    mark('geometry_delta',tick)
    tick=time.perf_counter()
    faces=[rows[0][1] for key,rows in owners.ordered_items()]
    keys=[face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in f)) for f in faces]
    newface={face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in owners.delta[key][0][1])) for key in owners.created_keys}
    if len(newface)!=len(owners.created_keys) or not newface<=changes['faces'].keys():raise ValueError('NewFaceIdentityMismatch')
    changed=[i for i,k in enumerate(keys) if k in newface]
    fresh=certify(points,faces,changed)
    if not fresh['passed']:raise ValueError('CandidateFreshContinuousRejected')
    objects=output/'objects';objects.mkdir(exist_ok=True)
    descriptor=persist_segment(fresh['proofs'],keys,objects)
    durable_json(output/'FRESH_CERTIFICATE.json',dict(
        **{k:v for k,v in fresh.items() if k!='proofs'},segment=descriptor,
        prior_component=state.components['certificate'],
        prior_population=len(state['certificate']),prior_payloads_read=0,prior_payloads_recomputed=0,
        combined_pairs=len(state['certificate'])+fresh['required_pairs'],unique_faces=len(faces),
        prior_geometry_preserved=True,changed_owner_faces=len(changes['faces'])-len(newface),
        note='Fresh realized-geometry proofs; complete acceptance still requires binding, volume and state assembly'))
    mark('fresh_continuous_proofs',tick)
    tick=time.perf_counter()
    total=sum((Fraction(r['signed_volume']) for r in q['exact']),Fraction())
    layerboundary=[tuple(reversed(tuple(map(int,f+lower*n)))) for f in tri]+outer
    boundary=sum((ExactVolume.contribution(points,f) for f in layerboundary),Fraction())
    if total!=boundary:raise ValueError('ExactAddedVolumeBalance')
    witnesses=[ExactVolume.sample(points,c) for c in cells]
    degree=volume.degree(points,[tuple(reversed(f)) for f in tri]+outer,witnesses)
    durable_json(output/'VOLUME_DELTA.json',dict(new_signed_volume=str(total),new_boundary_volume=str(boundary),
        exact_balance=True,prior_balance='Reused from authenticated committed base; geometry and internal cells unchanged',
        global_degree=degree))
    if not degree['passed']:raise ValueError('GlobalDegreeFailure')
    mark('global_degree_and_added_volume',tick)
    np.savez_compressed(output/'GENERATED_NEXT.npz',coordinates=high,topology=tri,
        new_connectivity=np.asarray([c['vertices'] for c in additions],dtype=np.int64))
    result=dict(transition_plan=plan,scope='Independently generated next-layer geometry and changed numerical evidence; not yet a published state',
        coordinates=array_hash(high),topology=array_hash(tri),new_connectivity=array_hash(np.asarray([c['vertices'] for c in additions],dtype=np.int64)),
        counts=dict(new_vertices=n,new_cells=len(additions),new_faces=len(newface),changed_owner_faces=len(changes['faces'])-len(newface),
            cells_reused=len(oldcells),combined_cells=len(cells),faces=len(faces),new_cell_quality=len(q['cells']),new_face_quality=len(q['faces']),
            completed_prior_determinants=len(prior),new_exact_cells=len(q['exact']),historical_exact_recomputed=0),
        forecast_coordinate_exact=bool(np.array_equal(high,provisional)),forecast_max_abs_difference=float(np.max(abs(delta))),
        sdf=assigned,topology_audit=topology,vertex_links=links,hard_failures=q['failures'],
        prior_receipt_payload_reads=0,prior_quality_rows=0,oracle_unlocked=False,
        phases=phases,seconds=time.perf_counter()-started)
    from persistent_numeric_incidence import write_segment
    parent_ref=packed.reference
    numeric_root=write_segment(output/'numeric_delta',owners.delta,state.content_root,parent_ref,
        cells=additions,topology=topology)
    from prism_source_lookup import write_lookup
    lookup=write_lookup(output/'numeric_lookup',((k,owners.delta[k]) for k in sorted(owners.delta)),n,numeric_root,state.content_root)
    durable_json(output/'PERSISTENT_GEOMETRY.json',dict(schema='V4PersistentGeometryBinding/1',
        parent=parent_ref,delta=dict(directory=str((output/'numeric_delta').resolve()),root=numeric_root,content=state.content_root,lookup=lookup),
        topology=topology,counts=result['counts'],degree=identity(degree),volume=identity(dict(total=str(total),boundary=str(boundary))),
        geometry_delta=file_hash(output/'GEOMETRY_DELTA.json'),version_closure=version_binding))
    durable_json(output/'GENERATION.json',result)
    return result


def produce(state,adapter,spec,output):
    from persistent_numeric_incidence import open_geometry
    reference=getattr(state,'numeric_incidence',None) or state.body['plan'].get('persistent_geometry')
    if reference is None:raise ValueError('MissingAuthenticatedNumericIncidence')
    from contextlib import ExitStack
    import immutable_state_access as access
    from sdf_cfd_mesher_v4.common.authority import identity
    with ExitStack() as held:
        binding=getattr(state,'numeric_version_closure',None)
        if binding is not None:
            held.enter_context(access.guard(binding['path']))
            summary=access.read(binding['path'])
            if identity(summary)!=binding['root'] or summary['parent_content']!=state.content_root or summary['numeric_root']!=reference['root'] or not summary['closed']:
                raise ValueError('WrongVersionClosure')
        elif reference.get('version_closure') is not None:
            binding=dict(inherited_committed_root=state.root)
        packed=held.enter_context(open_geometry(reference,state.content_root))
        return produce_with_index(state,adapter,spec,output,packed,binding)
