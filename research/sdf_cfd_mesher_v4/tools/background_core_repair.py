"""Generic live-core swaps then coordinated placement before final acceptance.

No fixture dispatch or retained mesh input. Current raw failures select complete
local closures. Existing V4 predicates admit edits; the caller owns final joined
acceptance. Backend-trusted mode retains its explicit initial embedding premise.
"""
import time,json
from collections import Counter
import numpy as np
from background_topology_repair import CoreTopology,key,cavity_geometry
from background_local_repair import star_index,verify_edits
from coordinated_local_repair import CoordinatedRepair,work_budget
from refined_local_repair import record_key,raw_failures
from prism_hotpath_v2 import batch_quality

from background_completion_controls import CORE_SCHEMA as SCHEMA,CORE_FIELDS as FIELDS,validate_core as validate
PROPOSAL_REJECTIONS={'InvalidCoreRemovalScope','DegenerateReplacement','InvalidReplacementOrientation','ChangedVertexSet','DuplicateReplacement','ChangedOrientedCavityBoundary','InvalidOrientedCavityIncidence','NonmanifoldCavityBoundary','UnexpectedNewBoundaryFace','ReplacementMustRetainActualOwners'}
def evidence_sink(folder,held=None):
    """Existing durable JSON owner; confirmed private edit prefix, not restart."""
    from pathlib import Path
    from path_preserving_deduplication import durable_json
    from immutable_state_access import guard
    from sdf_cfd_mesher_v4.common.authority import identity,file_hash
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    receipts=[]
    def retain(event,payload):
        sequence=len(receipts)+1;path=folder/('%06d.json'%sequence)
        body=dict(schema='V4CoreRepairDiagnosticEvent/1',sequence=sequence,event=event,payload=payload,
            provisional=True,scientific_authority=False,restart_supported=False,
            scope='Durable confirmed private edit prefix only; no accepted mesh or complete hard-killed state.')
        if path.exists():raise ValueError('ExistingCoreRepairEvidence')
        authored=identity(body);durable_json(path,body)
        def check():
            if identity(json.loads(path.read_text(encoding='utf-8')))!=authored:raise ValueError('ChangedCoreRepairEvidence')
            return file_hash(path)
        if held is None:
            with guard(path):digest=check()
        else:
            held.enter_context(guard(path));digest=check()
        receipts.append(dict(file=path.name,sha256=digest,event=event))
    retain.receipts=receipts
    return retain

def atoms(q):
    return raw_failures(dict(cells=[r for r in q['failures'] if r['kind']=='cell_quality'],
        exact=[dict(id=r['id'],**r['detail']) for r in q['failures'] if r['kind']=='exact_cell'],faces=[r for r in q['failures'] if r['kind']=='face_quality']))
def affected_rows(reports,ids,faces):
    return [r for r in reports.values() if (r['kind']=='face_quality' and key(r['vertices']) in faces) or (r['kind']!='face_quality' and r['id'] in ids)]
def replace_rows(reports,ids,faces,rows):
    for r in affected_rows(reports,ids,faces):reports.pop(record_key(r),None)
    reports.update((record_key(r),r) for r in rows)
def boundary_passed(points,star,owners,far):
    from capsule_core_control_audit import boundary_skewness
    import local_termination_topology as t
    faces=[f for c in star for f in c['faces'] if key(f) in far]
    if not faces:return True
    byid={c['id']:c for c in star};centres={i:t.geometry(points,byid[i])['centre'] for i in {owners[key(f)][0][0] for f in faces}}
    values=boundary_skewness(points[np.asarray(faces)],np.asarray([centres[owners[key(f)][0][0]] for f in faces]))
    return bool(np.isfinite(values).all() and np.all(values<=3.63636))
def construct(handoff,points,cells,owners,core_ids,fixed,far_faces,controls,*,_evidence=None):
    from rolling_fast_state import BackgroundInputs
    from sdf_cfd_mesher_v4.common.authority import identity,array_hash
    if type(handoff) is not BackgroundInputs:raise ValueError('ProtectedLiveGenericRepairHandoffRequired')
    handoff.require(handoff.view);controls=validate(controls.copy());control_identity=identity(controls)
    result=_repair(points,cells,owners,core_ids,fixed,far_faces,controls,_evidence=_evidence)
    if identity(controls)!=control_identity:raise ValueError('ChangedGenericCoreRepairControls')
    handoff.require(handoff.view)
    return result
def _repair(points,cells,owners,core_ids,fixed,far_faces,controls,*,_evidence=None):
    """Pure owned numerical consumer; never a handoff or mesh publisher."""
    from sdf_cfd_mesher_v4.common.authority import array_hash
    controls=validate(controls.copy());started=time.perf_counter();original=points.copy();fixed=set(fixed)
    initial=batch_quality(points,cells,owners,0,active_cells=set(core_ids),mixed_cells=True,face_batch_size=4096,compact=True)
    if any(r['kind']=='exact_cell' or r['kind']=='cell_quality' and not r['volume']>0 for r in initial['failures']):raise ValueError('BackendCoreNotPositiveForRelativeRepair')
    reports={record_key(r):r for r in initial['failures']};initial_atoms=atoms(initial)
    state=CoreTopology(points,cells,owners,core_ids,far_faces);topology=[];trials=Counter();seen=set();coordinate_records=[];witnesses=[]
    def emit(event,payload):
        if _evidence is not None:_evidence(event,payload)
    if _evidence is not None:
        ordered=[state.byid[i] for i in sorted(state.core_ids)]
        emit('initial',dict(points_sha256=array_hash(original),core_ids_sha256=array_hash(np.asarray([c['id'] for c in ordered],dtype='<i8')),
            core_rows_sha256=array_hash(np.asarray([c['vertices'] for c in ordered],dtype='<i8')),
            fixed_points_sha256=array_hash(original[sorted(fixed)]),controls=controls,initial_raw_records=len(reports)))
    search_started=time.perf_counter();deadline=search_started+controls['seconds_per_sweep']
    def budget():
        if time.perf_counter()>=deadline:
            detail=dict(search_seconds=time.perf_counter()-search_started,topology_edits=len(topology),coordinate_groups=len(coordinate_records),modified_vertices=len({r['vertex'] for r in witnesses}),remaining_records=len(reports),remaining_by_kind=dict(Counter(r['kind'] for r in reports.values())))
            emit('time_limit',detail)
            raise ValueError('GenericCoreRepairTimeLimit',detail)
    print(json.dumps(dict(stage='generic_core_repair_start',initial_records=len(reports),core_cells=len(core_ids),search_seconds_limit=controls['seconds_per_sweep'])),flush=True)
    # Only strictly reducing elementary edits commit. Coordinates stay fixed
    # throughout this stage, so unchanged face-owner contexts can be skipped.
    for sweep in range(len(initial_atoms)+1):
        accepted=0;seeds=sorted({i for r in reports.values() for i in (r['owners'] if r['kind']=='face_quality' else [r['id']])}&state.core_ids)
        for seed in seeds:
            budget()
            if seed not in state.core_ids or not any(seed in (r['owners'] if r['kind']=='face_quality' else [r['id']]) for r in reports.values()):continue
            passing=[]
            for order,proposal in enumerate(state.inventory(seed)):
                budget();stamp=state.stamp(proposal)
                if stamp in seen:continue
                seen.add(stamp)
                try:
                    view=state.view(proposal)
                except ValueError as exc:
                    if not exc.args or exc.args[0] not in PROPOSAL_REJECTIONS:raise
                    trials[str(exc)]+=1;continue
                q=state.quality(view)
                removed=affected_rows(reports,set(proposal['remove']),view['touched'])
                if q['passed'] and removed:passing.append((order,proposal,view,q,len(removed)))
                else:trials['raw_quality_or_no_reduction']+=1
            if not passing:continue
            chosen=min(passing,key=lambda row:(-row[4],-row[3]['summary']['minimum_determinant'],-row[3]['summary']['minimum_weight'],row[0]))
            _,proposal,view,q,_=chosen;before=cavity_geometry(points,view['removed']);after=cavity_geometry(points,view['added'])
            if before['boundary_chain']!=after['boundary_chain']:raise ValueError('ChangedAdmittedCoreCavityBoundary')
            budget()
            old_count=len(reports);replace_rows(reports,set(proposal['remove']),view['touched'],[]);state.commit(view)
            topology.append(dict(operation=proposal['operation'],removed=proposal['remove'],added=[dict(id=c['id'],vertices=c['vertices']) for c in view['added']],
                before_records=old_count,after_records=len(reports),new_records=0,before_geometry=before,after_geometry=after,
                unchanged_coordinates=True,scope='Exact closed cavity replacement relative to initial backend embedding.'))
            emit('topology',topology[-1])
            accepted+=1
            if len(topology)%10==0:print(json.dumps(dict(stage='generic_core_topology',edits=len(topology),remaining=len(reports),seconds=time.perf_counter()-search_started)),flush=True)
        if not accepted:break
    else:raise ValueError('CoreTopologyStrictReductionBound')
    topology_seconds=time.perf_counter()-search_started
    # Full incidence is enumerated once per current residual sweep. Each local
    # solve receives complete stars and actual one-hop geometry, never the whole
    # million-cell model or blanket transitive support enrichment.
    while reports:
        budget();before_count=len(reports);groups=set()
        for r in reports.values():
            ids=r['owners'] if r['kind']=='face_quality' else [r['id']]
            variables={v for i in set(ids)&state.core_ids for v in state.byid[i]['vertices']}-fixed
            if variables:groups.add(tuple(sorted(variables)))
        if not groups:break
        # Witness cell/link order must match final_core used by exact replay.
        wanted={v for row in groups for v in row};stars=star_index((state.byid[i] for i in sorted(state.core_ids)),wanted)
        for row in sorted(groups,key=lambda row:(len(row),row)):
            budget();union={c['id']:c for v in row for c in stars[v]};faces={key(f) for c in union.values() for f in c['faces']}
            selected=affected_rows(reports,set(union),faces)
            if not selected:continue
            local_owners={f:state.owners[f] for f in faces};needed={i for uses in local_owners.values() for i,_ in uses}
            local_cells=[state.byid[i] for i in sorted(needed)];local_fixed=set(range(len(points)))-set(row)
            group=CoordinatedRepair(points,local_cells,local_owners,set(union),local_fixed,selected)
            with work_budget(budget):
                record=group.group_trial(list(row),stars,maxiter=controls['maxiter'],ftol=controls['ftol'],max_depth=controls['preservation_max_depth'],max_attempts=controls['preservation_steps_per_vertex']*len(row),_geometric_start=True)
            budget()
            if record['adopted']:
                before_q=batch_quality(points,local_cells,local_owners,0,active_cells=set(union),mixed_cells=True,face_batch_size=4096,compact=True)
                after_q=batch_quality(group.points,local_cells,local_owners,0,active_cells=set(union),mixed_cells=True,face_batch_size=4096,compact=True)
                if atoms(after_q)-atoms(before_q):raise ValueError('NewCoreCoordinateFailure')
                if group.points[sorted(local_fixed)].tobytes()!=points[sorted(local_fixed)].tobytes():raise ValueError('ChangedUnplannedCoreCoordinate')
                if not boundary_passed(group.points,list(union.values()),local_owners,state.far):
                    record.update(adopted=False,reason='PhysicalBoundaryQualityRejection')
                    coordinate_records.append(record);continue
                # Complete sphere-cone witnesses are replayed on this exact
                # stage connectivity before any later cavity replacement.
                for v in row:points[v]=group.points[v]
                replace_rows(reports,set(union),faces,after_q['failures']);witnesses.extend(group.edits)
                record.update(affected_cells=sorted(union),new_atoms=0,remaining_records=len(reports),witnesses=group.edits)
                emit('coordinate',record)
            coordinate_records.append(record)
            if len(coordinate_records)%10==0:print(json.dumps(dict(stage='generic_core_coordinates',groups=len(coordinate_records),remaining=len(reports),seconds=time.perf_counter()-search_started)),flush=True)
        if len(reports)>=before_count:break
    final_core=[state.byid[i] for i in sorted(state.core_ids)]
    emit('before_witness_replay',dict(topology_edits=len(topology),coordinate_groups=len(coordinate_records),witnesses=len(witnesses),
        remaining_raw_records=len(reports),remaining_by_kind=dict(Counter(r['kind'] for r in reports.values()))))
    try:relative=verify_edits(original,points,final_core,fixed,witnesses)
    except Exception as error:
        emit('witness_replay_failure',dict(error=repr(error),witnesses=witnesses,
            original_points_sha256=array_hash(original),candidate_points_sha256=array_hash(points),
            final_core_ids_sha256=array_hash(np.asarray([c['id'] for c in final_core],dtype='<i8')),
            remaining_raw_records=len(reports),final_acceptance_passed=False))
        raise
    # Preserve the existing exact verifier's fixed-connectivity domain. Later
    # replacements operate with frozen points and carry exact cavity proofs;
    # those are composed with this verified coordinate stage, never substituted
    # for replay of a witness against a different tetrahedron population.
    coordinate_core_ids_sha256=array_hash(np.asarray([c['id'] for c in final_core],dtype='<i8'))
    coordinate_core_rows_sha256=array_hash(np.asarray([c['vertices'] for c in final_core],dtype='<i8'))
    emit('coordinate_stage_verified',dict(relative_preservation=relative,core_ids_sha256=coordinate_core_ids_sha256,
        core_rows_sha256=coordinate_core_rows_sha256,points_sha256=array_hash(points)))
    frozen_sha=array_hash(points);expanded_seen=set();expanded_trials=Counter();expanded_edits=[];expanded_started=time.perf_counter()
    for sweep in range(len(reports)+1):
        budget();accepted=0
        seeds=sorted({i for r in reports.values() for i in (r['owners'] if r['kind']=='face_quality' else [r['id']])}&state.core_ids)
        for seed in seeds:
            budget()
            if seed not in state.core_ids or not any(seed in (r['owners'] if r['kind']=='face_quality' else [r['id']]) for r in reports.values()):continue
            for proposal in state.expanded_inventory(seed,budget):
                budget()
                if 'generation_rejection' in proposal:
                    expanded_trials[proposal['generation_rejection']]+=1;continue
                stamp=state.stamp(proposal)
                if stamp in expanded_seen:continue
                expanded_seen.add(stamp)
                try:view=state.view(proposal)
                except ValueError as error:
                    if not error.args or error.args[0] not in PROPOSAL_REJECTIONS:raise
                    expanded_trials[str(error)]+=1;continue
                removed=affected_rows(reports,set(proposal['remove']),view['touched'])
                if not removed:continue
                q=state.quality(view)
                if not q['passed']:
                    expanded_trials['raw_quality_or_no_reduction']+=1;continue
                before=cavity_geometry(points,view['removed']);after=cavity_geometry(points,view['added'])
                if before['boundary_chain']!=after['boundary_chain']:raise ValueError('ChangedAdmittedCoreCavityBoundary')
                budget();old_count=len(reports);replace_rows(reports,set(proposal['remove']),view['touched'],[]);state.commit(view)
                record=dict(operation=proposal['operation'],removed=proposal['remove'],added=[dict(id=c['id'],vertices=c['vertices']) for c in view['added']],
                    before_records=old_count,after_records=len(reports),new_records=0,before_geometry=before,after_geometry=after,
                    unchanged_coordinates=True,phase='after_verified_coordinates',scope='Frozen-point exact closed cavity replacement after fixed-connectivity coordinate replay.')
                topology.append(record);expanded_edits.append(record);emit('topology',record);accepted+=1
                print(json.dumps(dict(stage='generic_core_expanded_cavity',edits=len(expanded_edits),remaining=len(reports),seconds=time.perf_counter()-search_started)),flush=True)
                break
        if not accepted:break
    else:raise ValueError('ExpandedCavityStrictReductionBound')
    if array_hash(points)!=frozen_sha:raise ValueError('ChangedFrozenCavityCoordinates')
    final_core=[state.byid[i] for i in sorted(state.core_ids)]
    if expanded_edits:
        relative=dict(relative,coordinate_stage_core_ids_sha256=coordinate_core_ids_sha256,
            coordinate_stage_core_rows_sha256=coordinate_core_rows_sha256,post_coordinate_cavity_edits=len(expanded_edits),
            final_core_ids_sha256=array_hash(np.asarray([c['id'] for c in final_core],dtype='<i8')),
            scope='Exact coordinate witness replay on the declared stage connectivity, followed by frozen-point independently certified closed cavity replacements; relative original embedded-filling premise retained. Not a coordinate replay on final changed connectivity.')
    if points[sorted(fixed)].tobytes()!=original[sorted(fixed)].tobytes():raise ValueError('ChangedPhysicalCoreRepairCoordinates')
    report=dict(schema=SCHEMA,initial_raw_records=len(initial['failures']),remaining_raw_records=len(reports),remaining=list(reports.values()),
        topology_edits=len(topology),topology=topology,topology_attempt_contexts=len(seen),rejected_topology=dict(trials),coordinate_groups=coordinate_records,
        expanded_cavity_edits=len(expanded_edits),expanded_attempt_contexts=len(expanded_seen),expanded_rejections=dict(expanded_trials),
        expanded_cavity_seconds=time.perf_counter()-expanded_started,
        coordinate_stage_core_ids_sha256=coordinate_core_ids_sha256,coordinate_stage_core_rows_sha256=coordinate_core_rows_sha256,
        proposal_methods=['geometry_altitude_volume_complete_star_kernel','larger_closed_edge_and_one_hop_delaunay_cavities'],
        witnesses=witnesses,relative_preservation=relative,modified_vertices=len({r['vertex'] for r in witnesses}),
        original_core_cells=len(core_ids),final_core_cells=len(final_core),final_core_ids=[c['id'] for c in final_core],
        topology_seconds=topology_seconds,search_seconds=time.perf_counter()-search_started,seconds=time.perf_counter()-started,
        final_acceptance_required=True,source_geometry_scope='Unchanged occupied cavities/interior stars; original source-domain and backend embedding premise retained. No independent whole-core SDF or global filling claim.',
        controls=controls,physical_boundary_coordinates_unchanged=True,old_prism_cells_unchanged=True,
        scope='Live generic raw cell/face construction; exact relative topology and coordinate proofs, not complete-mesh acceptance or restart.')
    emit('completed_relative_repair',dict(relative_preservation=relative,remaining_raw_records=len(reports),
        final_acceptance_required=True,topology_edits=len(topology),modified_vertices=report['modified_vertices']))
    if _evidence is not None:report['diagnostic_events']=list(getattr(_evidence,'receipts',[]))
    return points,list(state.byid.values()),final_core,state.owners,report
