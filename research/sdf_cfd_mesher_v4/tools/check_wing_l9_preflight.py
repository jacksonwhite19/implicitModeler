"""Prepare one successor control and inspect its existing production binding.

This is a read-only prerequisite check. No cells, forecast, proofs, indices or
checkpoints are generated here. Original authorization and states stay intact.
"""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import json
import subprocess
import sys
import time
import traceback

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
OUT = V4/'results/wing_l9_preflight'
ACCESS = V4/'results/storage_cleanup_mesh_readiness'
from path_preserving_deduplication import durable_json, file_hash
from record_receipt_resume import disk, status
from sdf_cfd_mesher_v4.common.authority import identity


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def baseline():
    if OUT.exists():
        raise ValueError('PreserveL9PreflightEvidence')
    prior = read(ACCESS/'BASELINE.json')
    for p,h in prior['protected'].items():
        if file_hash(p) != h:
            raise ValueError('ProtectedPreflightSourceChanged', p)
    old_path = V4/'results/wing_continuation_layers6to8/AUTHORIZATION.json'
    old = read(old_path)
    if identity(old) != '022ba294325d1e26e85634609fee42fed307f38df87a55f881d83001a8c58ce1':
        raise ValueError('ChangedPredecessorAuthorization')
    source = read(ACCESS/'BASELINE.json')['source']
    if source['scientific'] != 'e64b53451463bfe764fd2fb5726b5779658419def6eaf37bb2c63de4605be599':
        raise ValueError('WrongQualifiedStartingFront')
    parent = dict(source, layer=8)
    diagnostic = read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
    value = diagnostic['nominal_increment_hex'][8]
    auth = deepcopy(old)
    auth.update(authorized_layers=[9], predecessor=old, predecessor_identity=identity(old),
        approved_parent=parent, authorization='User: Go, following readiness and next bounded mesh-step discussion. Scope: one next layer L9 only; no L10.',
        increments=[dict(layer=9,quantity='path_length_m',binary64_hex=value,value_m=float.fromhex(value))],
        restrictions=['No L10','Stop at first required failure','No tuning, scientific retry or gate relaxation',
                      'This record is authorization, not admission or qualification evidence'],
        production_admission_passed=False)
    rec = read(ACCESS/'RECOVERY.json')['record']
    live = read(ACCESS/'REOPEN.json')
    if live['scientific'] != source['scientific'] or live['reference'] != rec['reference']:
        raise ValueError('UnboundCurrentAccess')
    resources = read(V4/'results/live_sf2_first_layer/PREFLIGHT_A.json')['resources']
    control = dict(schema='V4AuthorizedContinuationExecution/1',layers=[9],resume=True,
        parent=dict(live,layer=8),authorization_record=auth,authorization=identity(auth),
        compare_inside_publisher=True,
        recovered_access=dict(directory=str(ACCESS/'access'),identity=identity(rec),reference=rec['reference']),
        resource_policy=dict(total_storage_policy='monitor_without_output_cap',output_limit_bytes=None,
                             minimum_free_bytes=resources['minimum_free_bytes']))
    pins = {str(p):file_hash(p) for p in [old_path,ACCESS/'RECOVERY.json',ACCESS/'REOPEN.json',
        ACCESS/'SOURCE_CONNECTION.json',ACCESS/'SOURCE_REVIEW.json',ACCESS/'RECOVERY_REVIEW.json',
        V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json']}
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),status=status(),
        protected=prior['protected'],binaries=prior['binaries'],inputs=pins,disk=disk(),
        scope='One L9 preflight. Stop before generation if existing production binding fails.'))
    durable_json(OUT/'AUTHORIZATION.json',auth)
    durable_json(OUT/'CONTROL.json',control)
    print('L9',float.fromhex(value),value,'authorization',identity(auth),flush=True)


def check():
    import numpy as np
    import psutil
    import immutable_state_access as access
    from native_reader_source_access import qualified_reader_access
    from capsule_reuse_dispatch import qualified_dispatch
    from composable_state import CommittedState
    from current_front_dependencies import install_reader
    from run_authorized_continuation import continuation_scope, recovered_execution_access
    from accepted_production_contract import validate_authorization, bind_production
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from check_sf2_continuation import existing_state_class, exact_vertex_map, STUDY, PRIOR
    from sealed_front_connection import load_case
    from run_live_sf2_surface import CanonicalKernel, bind_canonical, bind_sf2_policy, verify_kernel_sources
    if (OUT/'CHECK.json').exists():
        raise ValueError('PreserveL9PreflightResult')
    start=time.perf_counter();stage='source_and_input_integrity';phases={}
    b=read(OUT/'BASELINE.json')
    try:
        with ExitStack() as held:
            for p,h in {**b['inputs'],**b['protected'],**b['binaries']}.items():
                held.enter_context(access.guard(p))
                if file_hash(p)!=h:
                    raise ValueError('ChangedPreflightDependency',p)
            for name in ('BASELINE.json','CONTROL.json','AUTHORIZATION.json'):
                held.enter_context(access.guard(OUT/name))
            c=read(OUT/'CONTROL.json');auth=read(OUT/'AUTHORIZATION.json')
            layers=continuation_scope(c,auth,identity(auth));next_layer=layers[0]
            if disk()['C:\\']['available'] < c['resource_policy']['minimum_free_bytes']:
                raise ValueError('OperationalResourceLimit')
            approved=read(ACCESS/'RECOVERY_REVIEW.json')
            held.enter_context(qualified_reader_access(ACCESS/'SOURCE_CONNECTION.json',approved['source_connection_identity'],
                ACCESS/'SOURCE_REVIEW.json',approved['source_review_sha256']))
            install_reader()
            case=read(PRIOR/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
            diagnostic=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
            state,selected=existing_state_class().load(STUDY/'planner/selected_state')
            mapping=exact_vertex_map(state.wall,wall)
            with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',allow_pickle=False) as z:
                first,refs=z['front'],z['face_refs']
            kernel,_=verify_kernel_sources()
            stage='live_geometry_and_schedule'
            with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
                obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
                authority=bind_sf2_policy(bind_canonical(obj)).authority
                owner=AcceptedTrajectoryOwner(authority,diagnostic,identity(diagnostic),wall,first,tri,refs,mapping,selected,parent=case['committed'])
                validate_authorization(auth,identity(auth),owner)
                phases['geometry_schedule']=dict(geometry=obj.identity,kernel=kernel,owner=owner.source_identity,
                    schedule=owner.schedule.sha256,increment=owner.schedule.row_record(next_layer),authorization=identity(auth),
                    requested_layers=layers)
                stage='authenticated_parent'
                with qualified_dispatch(),CommittedState(c['parent']['reference']) as base:
                    base.check()
                    if base.scientific!=c['parent']['scientific'] or base.layer!=next_layer-1:
                        raise ValueError('WrongActualCurrentFrontParent')
                    phases['parent']=dict(scientific=base.scientific,reference=base.reference,layer=base.layer)
                    stage='production_parent_authorization_binding'
                    with recovered_execution_access(base,c,auth) as evidence:
                        phases['recovered_access']=evidence
                        adapter,inspected=bind_production(base,case,owner,auth,identity(auth))
                        phases['binding']=dict(identity=adapter.identity,increment=adapter.schedule.row_record(next_layer))
                        stage='trajectory_premises'
                        target=adapter.schedule.targets[next_layer-1]
                        prediction=adapter.geometry_binding.predict(adapter.front.coordinates,target,iteration_cap=256)
                        validation=adapter.geometry_binding.validate_target(prediction.coordinates,target)
                        phases['trajectory']=validation
                        if not validation['admitted']:
                            raise ValueError('ContinuationTrajectoryPremiseRejected')
            result=dict(passed=True,stage=stage,phases=phases)
    except Exception as exc:
        result=dict(passed=False,stage=stage,code=str(exc),exception=type(exc).__name__,traceback=traceback.format_exc(),phases=phases)
    result.update(seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset,
        disk=disk(),no_forecast=True,no_generation=True,last_qualified_layer=b.get('request',{}).get('from_layer',8))
    durable_json(OUT/'CHECK.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('phases','traceback')}),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    if sys.argv[1]=='baseline':baseline()
    elif sys.argv[1]=='check':sys.exit(check())
    else:raise ValueError('UnsupportedReadOnlyPreflightMode')
