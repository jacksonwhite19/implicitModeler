"""Configure existing optimized executors; never supply historical fronts."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
OUT = V4/'results/authorized_continuation'
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]


def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))

def continuation_inputs(control):
    """Select explicit fresh initialization inputs, retaining legacy defaults."""
    from check_sf2_continuation import PRIOR
    from path_preserving_deduplication import file_hash
    spec=control.get('fresh_inputs') if control is not None else None
    if spec is None:
        return (read(PRIOR/'CASES.json')['wing'],
                read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json'),
                V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',read(PRIOR/'WING_INDICES.json'))
    if set(spec)!=set(('case','diagnostic','candidate','indices')):raise ValueError('IncompleteFreshContinuationInputs')
    paths={}
    for name,row in spec.items():
        p=Path(row['path']).resolve()
        if not p.is_relative_to((V4/'results').resolve()) or file_hash(p)!=row['sha256']:
            raise ValueError('ChangedFreshContinuationInput',name)
        paths[name]=p
    return read(paths['case']),read(paths['diagnostic']),paths['candidate'],read(paths['indices'])

def resource_origin(control):
    """Current fresh case record, or the unchanged legacy resource origin."""
    from path_preserving_deduplication import file_hash
    spec=control.get('resource_origin') if control is not None else None
    if spec is None:
        if control is not None and control.get('fresh_inputs'):raise ValueError('MissingFreshResourceOrigin')
        return read(V4/'results/live_sf2_first_layer/PREFLIGHT_A.json')['resources']
    p=Path(spec['path']).resolve()
    if not p.is_relative_to((V4/'results').resolve()) or file_hash(p)!=spec['sha256']:raise ValueError('ChangedFreshResourceOrigin')
    pre=read(p);auth=control['authorization_record']
    if (pre.get('passed') is not True or pre['surface']['surface_identity']!=auth['surface']
        or pre['surface']['geometry_identity']!=auth['geometry']
        or pre['request_sha256']!=auth['original_request_sha256']
        or pre['height_hex']!=float(auth['first_full_normal_height_m']).hex()):raise ValueError('WrongFreshResourceOrigin')
    if pre['resources']!=control.get('reviewed_resources'):raise ValueError('ChangedReviewedFirstLayerResourcePolicy')
    return pre['resources']


def baseline():
    from sdf_cfd_mesher_v4.common.authority import file_hash
    from path_preserving_deduplication import durable_json
    old=read(V4/'results/wing_continuation_layers2to5/BASELINE.json')
    status=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()
    status=[s for s in status if s!='?? research/sdf_cfd_mesher_v4/tools/run_authorized_continuation.py']
    if status!=old['status']:raise ValueError('UnrelatedWorktreeChanged')
    paths={r['path'] for r in old['files']}
    paths.update(str(p.relative_to(ROOT)).replace('\\','/') for p in (V4/'results/wing_continuation_layers2to5').glob('*') if p.is_file())
    paths.update('research/sdf_cfd_mesher_v4/tools/'+n for n in ('composable_publication.py','critical_assembly.py','composable_assembly.py'))
    files=[dict(path=p,bytes=(ROOT/p).stat().st_size,sha256=file_hash(ROOT/p)) for p in sorted(paths)]
    OUT.mkdir(exist_ok=False)
    durable_json(OUT/'BASELINE.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        utc=datetime.now(timezone.utc).isoformat(),status=status,files=files,initial_C=153302806528,initial_Z=847650447360))


from production_prism_stack import (execution_scope, continuation_scope, execution_resources,
    recovered_execution_access, current_recovered_access, committed_index_session,
    validate_execution_parent)


def execution_access_controls(out,continuation):
    """Check the actual selected parent, not an obsolete native L1 fixture.

    Legacy diagnostics retain their original controls. Explicit current-front
    execution uses the same guarded CommittedState without deep initialization,
    forecast or generation. Per-layer indexed checks remain in the executor.
    """
    if continuation is None:
        from check_sf2_continuation import access_controls
        return access_controls(out)
    from unittest.mock import patch
    from composable_state import CommittedState
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from production_forecast_admission import BoundProductionForecastCache
    from path_preserving_deduplication import durable_json
    from sealed_front_connection import qualified_dispatch
    def denied(*args,**kwargs):raise AssertionError('NoRestartForecastOrGenerationDuringGuardedOpen')
    records=[]
    with qualified_dispatch(),patch.object(NativeCheckpointStore,'restart',denied),patch.object(BoundProductionForecastCache,'forecast',denied):
        for attempt in range(2):
            with CommittedState(continuation['parent']['reference']) as base:
                validate_execution_parent(base,continuation,continuation['layers'][0])
                base.check()
                records.append(dict(reference=base.reference,scientific=base.scientific,
                    layer=base.layer,proof_count=len(base['certificate']),quality_count=len(base['quality'])))
    if records[0]!=records[1]:raise ValueError('GuardedReopenDifference')
    durable_json(out/'CURRENT_PARENT_ACCESS.json',dict(passed=True,opens=records,reopen_equal=True,
        native_restart_during_reopen=False,no_forecast=True,no_generation=True,
        scope='Selected authenticated current parent; per-layer persistent index, geometry and pair sessions remain required.'))


from production_prism_stack import close_preparation_jobs, publish_replayed


def execute(resume_candidate=None, resume_manifest=None, resume_manifest_hash=None, *, round_trip=None, continuation=None):
    import time, traceback, shutil
    from contextlib import ExitStack
    import numpy as np
    import psutil
    from sdf_cfd_mesher_v4.common.authority import identity, file_hash, array_hash
    from path_preserving_deduplication import durable_json
    from check_sf2_continuation import existing_state_class, exact_vertex_map, STUDY, PRIOR, access_controls
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from accepted_production_contract import bind_production, plan_transition, validate_authorization
    from sealed_front_connection import load_case, indexed_session, predict_bound
    from production_prism_stack import session,ExecutionProfile
    from composable_state import CommittedState
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    save=lambda name,value:durable_json(OUT/name,value)
    if round_trip is not None and continuation is not None:raise ValueError('ConflictingExecutionControls')
    if (OUT/'START.json').exists():raise ValueError('PreservePriorAttempt')
    start=time.perf_counter();stage='preflight';layer=2;last=1;phases={};measurements=[]
    resources=execution_resources(resource_origin(continuation),
        continuation.get('resource_policy') if continuation is not None else None)
    auth=read(V4/'results/wing_continuation_layers2to5/AUTHORIZATION.json') if continuation is None else continuation['authorization_record']
    auth_id=identity(auth) if continuation is None else continuation['authorization']
    inside=continuation.get('compare_inside_publisher',False) if continuation is not None else False
    if type(inside) is not bool:raise ValueError('InvalidReplayedPublicationMode')
    profile=ExecutionProfile(continuation.get('execution_profile','qualification') if continuation is not None else 'qualification')
    if round_trip is not None and profile is not ExecutionProfile.QUALIFICATION:
        raise ValueError('RoundTripRequiresQualificationProfile')
    save('START.json',dict(utc=datetime.now(timezone.utc).isoformat(),resources=resources,
        basis='Reuse existing conservative first-layer operational allowance for this incremental campaign; no whole-run timeout or invented memory threshold.',
        authorization=auth_id))
    def resource_check():
        size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
        free=shutil.disk_usage(ROOT).free
        save('RESOURCES.json',dict(output_bytes=size,available_c=free,peak_main_working_set=psutil.Process().memory_info().peak_wset,
             elapsed_s=time.perf_counter()-start,stage=stage,layer=layer))
        limit=resources['output_limit_bytes']
        if (limit is not None and size>limit) or free<resources['minimum_free_bytes']:raise ValueError('OperationalResourceLimit')
    def measure_phase(folder,label):
        # Operational observation only; concurrent index writers may still run.
        # Never authenticate mutable outputs by this resource snapshot.
        from finish_retained_l5 import live_allocation
        paths=[p for p in folder.rglob('*') if p.is_file()]
        logical=0
        for p in paths:
            try:logical+=p.stat().st_size
            except FileNotFoundError:pass
        measurements.append(dict(layer=layer,stage=label,seconds=time.perf_counter()-start,
            logical_bytes=logical,allocation=live_allocation(paths),
            free_c=shutil.disk_usage(ROOT).free,peak_main_working_set=psutil.Process().memory_info().peak_wset))
        save('MEASUREMENTS.json',measurements)
    try:
        if continuation is not None:
            selected=continuation_scope(continuation,auth,auth_id)
            layer=selected[0];last=layer-1
        resource_check()
        controls=OUT/'controls';controls.mkdir()
        tick=time.perf_counter();execution_access_controls(controls,continuation);phases['capsule_and_guarded_controls']=time.perf_counter()-tick
        case,diagnostic,first_path,initial_indices=continuation_inputs(continuation)
        wall,tri,surface,request=load_case(case)
        if continuation is None and auth_id!='bd2e3209707fd7f11d3082a069f2490a6903421e73ca16727f375683b4d96772':
            raise ValueError('OriginalContinuationAuthorizationMismatch')
        state,selected=existing_state_class().load(STUDY/'planner/selected_state')
        mapping=exact_vertex_map(state.wall,wall)
        with np.load(first_path,allow_pickle=False) as z:
            first,refs=z['front'],z['face_refs']
        kernel,_=verify_kernel_sources()
        selection=continuation['layers'] if continuation is not None else (None if round_trip is None else round_trip['layers'])
        layers=execution_scope(auth,selection)
        reference=read(PRIOR/'WING_REFERENCE.json') if round_trip is None else round_trip['parent']['reference']
        if continuation is not None:
            layers=continuation_scope(continuation,auth,auth_id)
            reference=continuation['parent']['reference'];last=layers[0]-1;layer=layers[0]
            save('CONTINUATION_EXECUTION.json',continuation)
        if round_trip is not None:
            if (round_trip.get('schema')!='V4AuthorizedTransitionRoundTrip/1'
                or round_trip['authorization']!=auth_id or len(layers)!=1
                or round_trip['parent'].get('passed') is not True
                or round_trip['reference_result'].get('passed') is not True):
                raise ValueError('InvalidTransitionRoundTripControl')
            save('ROUND_TRIP.json',round_trip)
            last=layers[0]-1
        with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
            obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
            authority=bind_sf2_policy(bind_canonical(obj)).authority
            tick=time.perf_counter()
            original=AcceptedTrajectoryOwner(authority,diagnostic,identity(diagnostic),wall,first,tri,refs,mapping,selected,parent=case['committed'])
            validate_authorization(auth,auth_id,original)
            phases['live_owner']=time.perf_counter()-tick
            save('PREFLIGHT.json',dict(geometry=obj.identity,kernel=kernel,owner=original.source_identity,schedule=original.schedule.sha256,
                authorization=auth_id,authorization_record=auth,original_request=request['request_file_sha256'],
                implementation={p.name:file_hash(p) for p in (V4/'tools').glob('*.py') if p.name in (
                    'accepted_production_contract.py','accepted_trajectory_contract.py','composable_admission.py','production_readonly_admission.py',
                    'critical_prism_transition.py','critical_prepare.py','critical_assembly.py','composable_state.py','production_forecast_admission.py','sealed_front_connection.py','production_prism_stack.py')}))
            with session(profile) as stack:
                for layer in layers:
                    folder=OUT/f'L{layer}';folder.mkdir()
                    with CommittedState(reference) as base,ExitStack() as sessions:
                        stage='parent_binding';tick=time.perf_counter()
                        if round_trip is not None and (base.scientific!=round_trip['parent']['scientific']
                            or base.layer!=layers[0]-1):raise ValueError('WrongRoundTripParent')
                        validate_execution_parent(base,continuation,layer)
                        recovered=sessions.enter_context(recovered_execution_access(base,continuation,auth))
                        if recovered is not None:durable_json(folder/'RECOVERED_ACCESS.json',recovered)
                        adapter,inspected=bind_production(base,case,original,auth,auth_id)
                        owner=adapter.geometry_binding
                        if base.layer!=layer-1:raise ValueError('NonSequentialCommittedParent')
                        if base.kind=='native':
                            numeric,pairs=sessions.enter_context(indexed_session(base,inspected,initial_indices))
                        else:
                            numeric,pairs=sessions.enter_context(committed_index_session(base,adapter,continuation))
                        plan=plan_transition(base,adapter)
                        durable_json(folder/'CONNECTION.json',dict(plan=plan,parent=base.scientific,root=base.root,
                            typed_increment=adapter.schedule.row_record(layer),authorization=auth_id,numeric_faces=len(numeric),
                            original_owner=original.source_identity,production_owner=owner.source_identity))
                        # Direct preserved arithmetic versus the actual generation boundary.
                        target=adapter.schedule.targets[layer-1]
                        predicted,validation=predict_bound(adapter,adapter.trajectory_spec,adapter.front.coordinates,target,iteration_cap=256)
                        expected=original.positions()[layer]
                        if predicted.coordinates.tobytes()!=expected.tobytes():raise ValueError('DeclaredGenerationArithmeticMismatch')
                        durable_json(folder/'TARGET.json',dict(validation=validation,vertices=len(expected),
                            bit_differences=0,coordinates=array_hash(expected),new_cells=0))
                        if not validation['admitted']:raise ValueError('ContinuationTrajectoryPremiseRejected')
                        phases[f'L{layer}_binding']=time.perf_counter()-tick
                        def observe(name,event,seconds):
                            nonlocal stage,last
                            stage=name
                            if event=='before':resource_check();print(layer,name,flush=True)
                            else:
                                phases[f'L{layer}_{name}']=seconds
                                save('PHASES.json',phases);measure_phase(folder,name)
                                if name=='guarded_reopen':
                                    reopened=read(folder/'REOPEN.json');last=layer
                                    save('LAST_QUALIFIED.json',dict(layer=last,reference=reopened['reference']))
                        def comparison(plan):
                            if round_trip is not None and plan['scientific']!=round_trip['reference_result']['scientific']:
                                raise ValueError('RetainedScientificIdentityMismatch')
                        retained=((resume_candidate,resume_manifest,resume_manifest_hash)
                            if resume_candidate is not None and layer==auth['authorized_layers'][0] else None)
                        result=stack.step(base,adapter,folder,pairs,observe=observe,
                            prepare_check=comparison if round_trip is not None else None,retained=retained)
                        reference,scientific=result['reference'],result['scientific']
                    last=layer;save('LAST_QUALIFIED.json',dict(layer=last,reference=reference))

                    save('PHASES.json',phases)
                    resource_check()
        resource_check()
        save('RESULT.json',dict(passed=True,last_qualified_layer=last,phases=phases,seconds=time.perf_counter()-start))
    except Exception as error:
        save('STOP.json',dict(code=str(error),exception=type(error).__name__,stage=stage,attempted_layer=layer,
            last_qualified_layer=last,traceback=traceback.format_exc(),phases=phases,seconds=time.perf_counter()-start,
            peak_main_working_set=psutil.Process().memory_info().peak_wset))
        print('STOP',stage,str(error),flush=True)
        raise
    finally:resource_check()


if __name__=='__main__':
    raise SystemExit('Historical driver retired. Use research/sdf_cfd_mesher_v4/mesh.py with an explicit case and restart.')
    p=argparse.ArgumentParser();p.add_argument('--baseline',action='store_true');p.add_argument('--execute',action='store_true')
    p.add_argument('--output',type=Path)
    p.add_argument('--resume-candidate',type=Path)
    p.add_argument('--resume-manifest',type=Path)
    p.add_argument('--resume-manifest-hash')
    args=p.parse_args()
    if args.output is not None:
        OUT=args.output.resolve()
        if not OUT.is_relative_to(V4/'results'):raise ValueError('OutputOutsideV4Results')
        OUT.mkdir(parents=True,exist_ok=False)
    if args.baseline:baseline()
    elif args.execute:
        if bool(args.resume_candidate)!=bool(args.resume_manifest and args.resume_manifest_hash):
            raise ValueError('CompleteRetainedManifestBindingRequired')
        execute(args.resume_candidate,args.resume_manifest,args.resume_manifest_hash)
