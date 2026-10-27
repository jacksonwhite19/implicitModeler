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
    # Keep unrelated inherited components lazy. In particular, constructing a
    # geometry delta does not consume the old atlas or forecast population.
    state = ChainMap({k: ChainMap(v, before[k]) for k, v in changes.items()}, before)
    state['outer'] = sorted(changes['prism_front'].values())
    inputs = geometry_inputs(dict(vertices=state['vertices'], cells=changes['cells'],
        faces=changes['faces'], wall=before['wall'], authority=before['authority']))
    return state, changes, inputs


def produce(state, adapter, spec, output):
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
    def index(v):
        a=v.split(':');return int(a[1])*n+int(a[3])
    oldcells=[]
    for key in sorted(state['cells'],key=int):
        c=state['cells'][key]
        ids=[index(v) for v in c['vertices']]
        oldcells.append(t.cell(ids,int(c['source']),int(c['layer'])))
    cells=oldcells+additions
    outer=[tuple(map(int,f+target*n)) for f in tri]
    owners,topology=t.ownership(cells,outer,[tuple(reversed(f)) for f in tri])
    links=t.vertex_links(outer)
    if not topology['passed'] or not links['passed']:raise ValueError('CandidateTopologyRejected')
    mark('geometry_and_global_incidence',tick)
    tick=time.perf_counter();active={c['id'] for c in additions}
    local={tuple(sorted(f)):owners[tuple(sorted(f))] for c in additions for f in c['faces']}
    needed={i for rows in local.values() for i,_ in rows}
    q=t.quality(points,[c for c in cells if c['id'] in needed],local,1,active_cells=active)
    prior={i for i in needed if i not in active}
    incidence=incidence_check(points,cells,outer,active_cells=active|prior)
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
    after,changes,inputs=append_components(state,additions,points,n)
    durable_json(output/'GEOMETRY_DELTA.json',changes)
    durable_json(output/'CHANGED_INPUTS.json',inputs)
    mark('geometry_delta',tick)
    tick=time.perf_counter()
    faces=[owners[k][0][1] for k in sorted(owners)]
    keys=[face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in f)) for f in faces]
    newface={k for k in changes['faces'] if k not in state['faces']}
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
    durable_json(output/'GENERATION.json',result)
    return result
