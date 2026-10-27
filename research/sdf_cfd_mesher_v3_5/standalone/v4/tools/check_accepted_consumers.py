"""Bounded consumer verification. Never calls a layer executor."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import sys

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
OUT = V4/'results/accepted_front_consumers'
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(path, value):
    from path_preserving_deduplication import durable_json
    durable_json(path, value)


def baseline():
    from sdf_cfd_mesher_v4.common.authority import file_hash
    previous = read(V4/'results/sf2_continuation_binding/BASELINE.json')
    status = subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'], cwd=ROOT).decode('utf-8').splitlines()
    # This checker is the only file added before baseline capture.
    status = [s for s in status if s != '?? research/sdf_cfd_mesher_v4/tools/check_accepted_consumers.py']
    if len(status) != 155 or status != previous['status']:
        raise ValueError('UnexpectedInitialWorktree')
    paths = {r['path'] for r in previous['files']}
    paths.update(str(p.relative_to(ROOT)).replace('\\','/') for p in [
        V4/'tools/production_forecast_admission.py', V4/'tools/composable_admission.py',
        V4/'tools/critical_prism_transition.py', V4/'src/sdf_cfd_mesher_v4/forecast/engine.py'])
    files = [dict(path=p,bytes=(ROOT/p).stat().st_size,sha256=file_hash(ROOT/p)) for p in sorted(paths)]
    OUT.mkdir(exist_ok=False)
    save(OUT/'BASELINE.json', dict(start='2026-09-30T10:26:41.5588148-05:00',
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        status=status, files=files, available_c=153377820672, available_z=847650447360))


def verify(destination, access=False):
    import dataclasses
    import copy
    from contextlib import ExitStack
    from unittest.mock import patch
    import numpy as np
    import psutil
    from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash,identity
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY,PRIOR,access_controls
    from accepted_trajectory_contract import AcceptedTrajectoryOwner,verify_record,plan_transition,require_transition_authorization,bind_forecast
    from production_forecast_admission import ProductionForecastAdapter,BoundProductionForecastCache
    from sealed_front_connection import load_case,qualified_dispatch,predict_bound,inspect_front,indexed_session
    from composable_state import CommittedState
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    destination.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter();phases={}
    record=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
    expected='b2cb76a6c7384d2e60cc1dddf9682474ee72822cc6e1945611613a77b1b29265'
    case=read(PRIOR/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
    old,selected=existing_state_class().load(STUDY/'planner/selected_state')
    mapping=exact_vertex_map(old.wall,wall)
    with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz') as data:
        first=data['front'];refs=data['face_refs'];curves=data['curve_edges'];corners=data['corner_refs']
    kernel,_=verify_kernel_sources();phases['inputs']=time.perf_counter()-started
    def denied(*a,**kw):raise AssertionError('ScalarPredictorCalledForDeclaredTrajectory')
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
        obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
        authority=bind_sf2_policy(bind_canonical(obj)).authority
        tick=time.perf_counter()
        owner=AcceptedTrajectoryOwner(authority,record,expected,wall,first,tri,refs,mapping,selected,parent=case['committed'])
        phases['live_all_vertex_directions']=time.perf_counter()-tick
        if owner.state.vectors.tobytes()!=old.vectors[:,mapping].tobytes():raise ValueError('LiveVectorDifference')
        normal_pairs=owner.state.pairs
        heights=np.einsum('ij,ij->i',owner.state.normals[normal_pairs[:,1]],(first-wall)[normal_pairs[:,0]])
        # Compare to existing implementation on the historical input, not a copied update.
        direct=old.positions(selected)
        bound=owner.positions()
        comparisons=[]
        for layer in range(2,21):
            expected_points=direct[layer,mapping];actual=bound[layer]
            differences=int(np.count_nonzero(actual.view('u8')!=expected_points.view('u8')))
            if differences:raise ValueError('AcceptedPositionDifference:'+str(layer))
            comparisons.append(dict(layer=layer,vertices=len(wall),bit_differences=differences,
                maximum_absolute_difference_m=float(np.max(abs(actual-expected_points))),coordinates=array_hash(actual),
                increment=owner.schedule.row_record(layer),historical_only=True))
        rejections={}
        def reject(name,call,want):
            try:call()
            except ValueError as error:
                if str(error)!=want:raise
                rejections[name]=str(error);return
            raise AssertionError('ExpectedRejection:'+name)
        common=dict(geometry=case['geometry_identity'],parent=case['committed'],layer=1,units='m')
        for name,value,want in [('parent','stale','StaleContinuationParent'),('geometry','wrong','ContinuationGeometryMismatch'),
            ('units','mm','ContinuationUnitsMismatch'),('layer',2,'ContinuationSchedulePositionMismatch'),('layer',20,'NoPendingIncrement')]:
            reject(name+str(value),lambda n=name,v=value:verify_record(record,expected,**dict(common,**{n:v})),want)
        altered=copy.deepcopy(record);altered['later_scalar_quantity']='normal_height_m'
        reject('altered_record',lambda:verify_record(altered,expected,**common),'AlteredContinuationDiagnostic')
        reject('incompatible_quantity',lambda:verify_record(altered,identity(altered),**common),'UnsupportedContinuationScalarMeaning')
        changed=copy.deepcopy(record);changed['generation_authorized']=True
        reject('authorization_promotion',lambda:verify_record(changed,identity(changed),**common),'DiagnosticAuthorizationRequired')
        reject('scalar_target',lambda:owner.predict(first,.00036025794570401554),'StaleDeclaredTrajectoryTarget')
        target=owner.schedule.targets[1]
        reject('wrong_target_parent',lambda:owner.predict(first,dataclasses.replace(target,parent='stale')),'StaleDeclaredTrajectoryTarget')
        reject('wrong_quantity',lambda:owner.predict(first,dataclasses.replace(target,quantity='normal_height_m')),'IncompatibleTargetQuantity')
        reject('wrong_position',lambda:owner.predict(first,owner.schedule.targets[2]),'DeclaredTargetParentCoordinatesMismatch')
        from accepted_trajectory_contract import AcceptedSchedule
        changed=mapping.copy();changed[:2]=changed[:2][::-1]
        reject('mapping',lambda:AcceptedSchedule(record,selected,changed),'ContinuationMappingMismatch')
        changed_schedule=selected.copy();changed_schedule[0,1]=np.nextafter(changed_schedule[0,1],np.inf)
        reject('schedule',lambda:AcceptedSchedule(record,changed_schedule,mapping),'SelectedScheduleIdentityMismatch')
        original=owner.state.vectors;owner.state.vectors=original.copy();owner.state.vectors[0,0,0]=np.nextafter(original[0,0,0],np.inf)
        reject('vectors',owner.check,'AlteredAcceptedTrajectoryBinding');owner.state.vectors=original
        reject('target_coordinates',lambda:owner.validate_target(first,target),'DeclaredTargetCoordinatesMismatch')
        tick=time.perf_counter()
        with qualified_dispatch(),CommittedState(read(PRIOR/'WING_REFERENCE.json')) as base:
            adapter=bind_forecast(base,case,owner)
            plan=plan_transition(base,adapter)
            if plan['generation_authorized'] is not False:raise ValueError('UnexpectedGenerationGrant')
            # Existing generation boundary, without produce/cell construction.
            generated,validation=predict_bound(adapter,adapter.trajectory_spec,first,target,iteration_cap=256)
            import sdf_cfd_mesher_v4.forecast.engine as engine
            import sdf_cfd_mesher_v4.advancing_front.predict as predictor
            with patch.object(engine,'predict_next_front',denied),patch.object(predictor,'predict_next_front',denied):
                forecast=BoundProductionForecastCache(adapter).forecast(adapter)
            if forecast.vertex['coordinates'][0].tobytes()!=generated.coordinates.tobytes():raise ValueError('ForecastGenerationDifference')
            if forecast.vertex['coordinates'].tobytes()!=bound[2:].tobytes():raise ValueError('ForecastAcceptedPositionDifference')
            reject('generation_authorization',lambda:require_transition_authorization(adapter),'DiagnosticContinuationGenerationUnauthorized')
            import composable_admission
            import critical_prism_transition
            # These guards must fire before a plan, output file, scientific screen or cell.
            reject('actual_admission_guard',lambda:composable_admission.admit(base,adapter,adapter.trajectory_spec,destination/'not_created'),
                   'DiagnosticContinuationGenerationUnauthorized')
            reject('actual_generation_guard',lambda:critical_prism_transition.produce_with_index(base,adapter,adapter.trajectory_spec,destination/'not_created',None),
                   'DiagnosticContinuationGenerationUnauthorized')
            adapter.binding['committed']='stale'
            reject('adapter_parent',lambda:owner.check_adapter(adapter,adapter.trajectory_spec),'StaleContinuationParent')
            adapter.binding['committed']=case['committed']
            rejected_owner=adapter.sdf;adapter.sdf=object()
            reject('different_forecast_owner',lambda:owner.check_adapter(adapter,adapter.trajectory_spec),'DifferentForecastGenerationTrajectoryOwner')
            adapter.sdf=rejected_owner
            original_topology=adapter.topology
            adapter.topology=dataclasses.replace(adapter.topology,triangles=adapter.topology.triangles[:,::-1])
            reject('topology_ownership',lambda:owner.check_adapter(adapter,adapter.trajectory_spec),'TrajectorySourceOwnershipMismatch')
            adapter.topology=original_topology
            inspected=inspect_front(base,wall,tri,surface,request,base.expected)
            with indexed_session(base,inspected,read(PRIOR/'WING_INDICES.json')) as (numeric,pairs):
                face=base['prism_front']['0'];affected=pairs.affected({'face:'+face:'unchanged_probe'})
                access_result=dict(native=base.scientific,content=base.content_root,root=base.root,
                    persistent_faces=len(numeric),affected=affected,deep_restart=False)
            base.check()
            consumer=dict(binding=adapter.binding,binding_identity=adapter.identity,owner=owner.source_identity,
                schedule=owner.schedule.sha256,plan=plan,forecast_scientific_identity=forecast.scientific_hash(),
                forecast_layers=forecast.layers.tolist(),hard_events=forecast.hard_events(),
                fields=sorted(forecast.vertex),edge_fields=sorted(forecast.edge),
                generation_target_validation=validation,forecast_generation_bit_differences=0,
                no_scalar_predictor_called=True,no_stage_advanced=True)
        phases['consumers_and_authenticated_access']=time.perf_counter()-tick
        regions={}
        for ref in np.unique(refs):
            ids=np.unique(tri[refs==ref]);regions[str(int(ref))]=dict(vertices=len(ids),bit_differences=int(np.count_nonzero(bound[2,ids].view('u8')!=direct[2,mapping[ids]].view('u8'))))
        for name,ids in [('curves',np.unique(curves[:,:2]).astype(int)),('corners',np.flatnonzero(corners))]:
            regions[name]=dict(vertices=len(ids),bit_differences=int(np.count_nonzero(bound[2,ids].view('u8')!=direct[2,mapping[ids]].view('u8'))))
        result=dict(classification='V4AcceptedFrontTrajectoryConsumerConnectionVerifiedDiagnosticOnly',
            geometry=obj.identity,kernel_source=kernel,record_identity=expected,owner=owner.source_identity,
            vertices=len(wall),triangles=len(tri),live_vectors_bit_differences=0,comparisons=comparisons,
            incident_normal_equations=len(normal_pairs),first_normal_height_residual_max_m=float(np.max(abs(heights-owner.state.first_height))),
            regions=regions,consumer=consumer,access=access_result,rejections=rejections,
            original_request_sha256=request['request_file_sha256'],generation_authorized=False,
            layers_advanced=0,new_cells=0,publication=False,
            qualification='Target arithmetic/ownership connection only; no trajectory, cell, collision, or transition pass.',
            known_counterexample_sha256=file_hash(V4/'results/sf2_continuation_binding/target_check/SCALAR_COUNTEREXAMPLE.json'))
    # Close the live source before capsule controls. No deep recovery or enrollment.
    if access:
        tick=time.perf_counter();access_controls(destination);phases['legacy_access_capsule_controls']=time.perf_counter()-tick
    result.update(phases=phases,seconds=time.perf_counter()-started,
        peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
        peak_scope='Main verification process only; no claim of combined host-process peak.')
    save(destination/'CHECK.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('consumer','comparisons')},indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--baseline',action='store_true')
    parser.add_argument('--output',type=Path);parser.add_argument('--access-controls',action='store_true')
    args=parser.parse_args()
    if args.baseline: baseline()
    elif args.output:verify(args.output,args.access_controls)
