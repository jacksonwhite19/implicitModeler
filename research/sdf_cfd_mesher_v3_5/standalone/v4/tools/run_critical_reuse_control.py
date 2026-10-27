"""One normal critical-stack control; configure existing executors only.

No cell, certificate, format or publisher implementation. No qualification or
replay campaign. The existing normal production gates/readbacks remain required.
"""
import argparse
from contextlib import ExitStack, contextmanager
from pathlib import Path
import subprocess
import sys
import time

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk,status
from sdf_cfd_mesher_v4.common.authority import identity
from run_successor_benchmark import execution_sources,execution_binaries,retained_execution_inputs,FLAGS

OUT=V4/'results/critical_reuse_production';RUN=OUT/'run';MONITOR=OUT/'monitor'
REFERENCE=V4/'results/fresh_successor_benchmark/current_parent_run/L7'
OWN=('tools/run_critical_reuse_control.py','tests/test_critical_reuse_control.py',
     'tools/capsule_reuse_dispatch.py','tools/sealed_front_connection.py','tests/test_capsule_reuse_dispatch.py',
     'CURRENT_TASK.md','README.md','CRITICAL_REUSE_PRODUCTION_REPORT.md','results/critical_reuse_control/',
     'results/critical_reuse_source_compatible/','results/critical_reuse_production/')
ALIGNED=False


def configure_output(output=None):
    """Explicit new result path; the original control remains immutable."""
    global OUT,RUN,MONITOR,OWN,ALIGNED
    if output is None:return
    path=Path(output).resolve()
    if not path.is_relative_to((V4/'results').resolve()) or path==(V4/'results').resolve():
        raise ValueError('CriticalOutputOutsideResults')
    OUT=path;RUN=path/'run';MONITOR=path/'monitor';ALIGNED=True
    OWN=(*OWN,str(path.relative_to(V4)).replace('\\','/')+'/',
         'GENERIC_CRITICAL_PRODUCTION_REPORT.md',
         'tools/collect_generic_critical_production.py','tests/test_generic_critical_collection.py')
    if path.name=='geometry_binding_production':
        OWN=(*OWN,'results/geometry_binding_efficiency/','GEOMETRY_BINDING_EFFICIENCY_REPORT.md',
             'tools/profile_geometry_binding.py','tests/test_lazy_geometry_binding.py',
             'tools/composable_transition.py','tools/critical_prism_transition.py',
             'tools/prism_hotpath_v2.py','tools/critical_assembly.py')


def read(p):
    import json
    return json.loads(Path(p).read_text(encoding='utf-8'))


def unrelated():
    prefix='research/sdf_cfd_mesher_v4/'
    return [r for r in status() if not any(r[3:]==prefix+p or p.endswith('/') and r[3:].startswith(prefix+p) for p in OWN)]


def baseline():
    if (OUT/'BASELINE.json').exists() or RUN.exists():raise ValueError('PreserveCriticalProductionControl')
    prior=read(OUT/'INITIAL.json') if ALIGNED else read(V4/'results/capsule_fast_reuse/BASELINE.json')
    prefix='research/sdf_cfd_mesher_v4/'
    original=[r for r in prior['status'] if not any(r[3:]==prefix+p or p.endswith('/') and r[3:].startswith(prefix+p) for p in OWN)]
    if unrelated()!=original:raise ValueError('ChangedUnrelatedControlStatus')
    protected={p:h for p,h in prior['protected'].items() if not ALIGNED or Path(p).resolve()!=Path(__file__).resolve()}
    if any(file_hash(p)!=h for p,h in protected.items()):raise ValueError('ChangedProtectedControlInput')
    control=read(V4/'results/fresh_successor_benchmark/CONTROL.json')
    control=dict(control,layers=[7],reuse_validated_assembly=True,
                 scope='One normal production L6->L7 control; no fresh replay or deep qualification campaign')
    from run_authorized_continuation import continuation_scope
    continuation_scope(control,control['authorization_record'],control['authorization'])
    resources=read(V4/'results/live_sf2_first_layer/PREFLIGHT_A.json')['resources']
    from run_authorized_continuation import execution_resources
    resources=execution_resources(resources,control['resource_policy'])
    if disk()['C:\\']['available']<resources['minimum_free_bytes']:raise ValueError('OperationalResourceLimit')
    OUT.mkdir(exist_ok=ALIGNED);RUN.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),status=unrelated(),
        protected=protected,disk=disk(),alignment_benchmark=ALIGNED))
    durable_json(OUT/'CONTROL.json',control)
    durable_json(RUN/'START.json',dict(resources=resources,scope=control['scope'],authorization=control['authorization']))
    inputs=retained_execution_inputs(read(V4/'results/fresh_successor_benchmark/REVIEW_REQUEST.json')['inputs'])
    for p in [REFERENCE/n for n in ('CANDIDATE_PLAN.json','REOPEN.json','GENERATION.json','FRESH_CERTIFICATE.json')]:
        inputs[str(p)]=file_hash(p)
    durable_json(OUT/'REVIEW_REQUEST.json',dict(runtime=execution_sources([Path(__file__)]),binary=execution_binaries(),inputs=inputs,
        baseline=file_hash(OUT/'BASELINE.json'),control=file_hash(OUT/'CONTROL.json'),start=file_hash(RUN/'START.json'),
        flags=FLAGS,scope=control['scope']))
    print('Prepared control; independent source review required',disk(),flush=True)


def verified(held):
    import immutable_state_access as access
    for name in ('REVIEW.json','REVIEW_REQUEST.json','BASELINE.json','CONTROL.json'):
        held.enter_context(access.guard(OUT/name))
    review=read(OUT/'REVIEW.json');q=read(OUT/'REVIEW_REQUEST.json')
    if (review.get('passed') is not True or not review.get('reviewer') or
        review.get('request_sha256')!=file_hash(OUT/'REVIEW_REQUEST.json') or review.get('runtime')!=q['runtime'] or
        q['baseline']!=file_hash(OUT/'BASELINE.json') or q['control']!=file_hash(OUT/'CONTROL.json') or
        q['start']!=file_hash(RUN/'START.json') or q['flags']!=FLAGS):raise ValueError('IndependentCriticalControlReviewRequired')
    held.enter_context(access.guard(RUN/'START.json'))
    for group in ('runtime','binary','inputs'):
        for p,h in q[group].items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedCriticalControlDependency',p)
    c=read(OUT/'CONTROL.json')
    from run_authorized_continuation import continuation_scope
    if continuation_scope(c,c['authorization_record'],c['authorization'])!=[7] or c.get('reuse_validated_assembly') is not True:
        raise ValueError('WrongCriticalControlScope')
    return c


def compare_plan(actual,expected,dependency_evidence=None):
    if (actual['content'].get('components')!=actual['components'] or
        expected['content'].get('components')!=expected['components']):
        return dict(passed=False,reason='DetachedContentComponents')
    comparisons={k:expected['components'].get(k)==v for k,v in actual['components'].items()}
    if dependency_evidence is not None:
        # Container identity still changes. Permit only separately proven typed
        # dependency meaning; every other component must match byte-exactly.
        if dependency_evidence.get('passed') is not True:raise ValueError('UnverifiedTypedDependencyComparison')
        if (set(actual['components'].get('forecast',{}))!={'replace'} or
            set(expected['components'].get('forecast',{}))!={'replace'} or
            actual['components']['forecast']['replace']!=dependency_evidence.get('actual_semantic') or
            expected['components']['forecast']['replace']!=dependency_evidence.get('reference_semantic')):
            return dict(passed=False,reason='UnboundForecastDescriptor')
        comparisons['forecast']=dependency_evidence['passed']
        adjusted=dict(actual['content'],components=dict(actual['components'],forecast=expected['components']['forecast']))
        content_equal=adjusted==expected['content']
        science_equal=identity(actual['content'])==actual['scientific'] and identity(expected['content'])==expected['scientific']
    else:
        content_equal=actual['content']==expected['content']
        science_equal=identity(actual['content'])==actual['scientific']==expected['scientific']
    passed=(set(actual['components'])==set(expected['components']) and all(comparisons.values()) and
        content_equal and science_equal and
        identity(expected['content'])==expected['scientific'])
    return dict(passed=passed,comparisons=comparisons,scientific=actual['scientific'],reference=expected['scientific'],
                container_identity_equal=actual['scientific']==expected['scientific'],dependency_evidence=dependency_evidence,
                scope='Fresh production versus retained qualified scientific components, not a fresh replay')


def compare_dependencies(base,adapter,folder,expected,accepted):
    """Existing authenticated shared reader and exact typed-schedule comparison.

    Only implementation-bound target owner changes. The parent, authorization,
    quantities, per-source increments, full suffix and row population stay exact.
    """
    from current_front_dependencies import authenticated_snapshot,_encoded,_validated
    import immutable_state_access as access
    from accepted_production_contract import dependency_schedule
    contract=read(accepted.verify(REFERENCE/'CONTINUATION_CONTRACT.json'))
    original=adapter.geometry_binding.original
    owner=adapter.geometry_binding
    reference_owner=identity(dict(root_owner=original.source_identity,authorization=owner.authorization_identity,
        parent=base.scientific,layer=base.layer,implementation=contract['implementation']))
    schedule=dependency_schedule(adapter)
    reference_schedule=[dict(r,target=dict(r['target'],owner=reference_owner)) for r in schedule]
    if (contract['authorization']!=owner.authorization or contract['identity']!=owner.authorization_identity or
        contract['root_owner']!=original.source_identity or contract['diagnostic']!=identity(owner.record) or
        contract['parent']!=base.scientific or contract['plan']['target']!=reference_schedule[base.layer]['target']):
        raise ValueError('UnboundReferenceDependencyOwner')
    path=accepted.verify(REFERENCE/'NEW_FORECAST_DEPENDENCIES.json')
    with authenticated_snapshot(folder/'NEW_FORECAST_DEPENDENCIES.json') as a,authenticated_snapshot(path) as b:
        if b['semantic']!=expected['components']['forecast']['replace']:raise ValueError('ReferenceDependencyDigestMismatch')
        def rows(p):
            body=access.read(p)
            if not _encoded(body):return body,None
            return body['rows'],_validated(body,access.read(body['shared']['path']))
        av,aschedules=rows(folder/'NEW_FORECAST_DEPENDENCIES.json');bv,bschedules=rows(path)
        if set(av)!=set(bv):raise ValueError('ChangedDependencyInventory')
        for bundles,wanted in ((aschedules,schedule[base.layer+1:]),(bschedules,reference_schedule[base.layer+1:])):
            if bundles is not None and (len(bundles)!=1 or any(v!=wanted for v in bundles.values())):
                raise ValueError('ChangedTypedDependencySchedule')
        for key in av:
            x=av[key];y=bv[key];layer=x['current_layer']
            if (set(x)!=({'region','current_layer','schedule'} if aschedules is not None else {'region','current_layer','remaining_schedule'}) or
                set(y)!=({'region','current_layer','schedule'} if bschedules is not None else {'region','current_layer','remaining_schedule'}) or
                x['region']!=y['region'] or layer!=y['current_layer'] or layer!=base.layer+1 or
                (aschedules is None and x['remaining_schedule']!=schedule[layer:]) or
                (bschedules is None and y['remaining_schedule']!=reference_schedule[layer:])):
                raise ValueError('ChangedTypedDependencyMeaning',key)
        return dict(passed=True,rows=len(av),actual_semantic=a['semantic'],reference_semantic=b['semantic'],
            actual_owner=owner.source_identity,reference_owner=reference_owner,parent=base.scientific,
            actual_implementation=file_hash(__import__('accepted_production_contract').__file__),
            reference_implementation=contract['implementation'],schedule=owner.schedule.sha256,
            scope='All rows and complete typed suffix exact; only validated implementation-bound target owner differs')


def authenticated_comparator(state,expected,parent):
    state.check()
    if (state.layer!=7 or state.parent.reference!=parent or state.plan['parent']!=parent or
        state.scientific!=expected['scientific'] or not compare_plan(state.plan,expected)['passed']):
        raise ValueError('UnboundQualifiedCriticalComparator')
    return expected


@contextmanager
def observed_json_writes(records):
    # The transition imports this function locally when called, not as a module
    # attribute. Observe its existing dependency; do not replace serialization.
    from unittest.mock import patch
    import path_preserving_deduplication as writer
    original=writer.durable_json
    def observed(path,value):
        tick=time.perf_counter();result=original(path,value)
        records.append(dict(name=Path(path).name,seconds=time.perf_counter()-tick,bytes=Path(path).stat().st_size))
        return result
    with patch.object(writer,'durable_json',observed):yield


def publish_compared(base,adapter,folder,pairs,publisher,expected,accepted=None):
    from production_prism_stack import publish_checked
    def check(actual):
        evidence=compare_dependencies(base,adapter,folder,expected,accepted) if ALIGNED else None
        if evidence is not None and evidence['actual_semantic']!=actual['components']['forecast']['replace']:
            raise ValueError('ActualDependencyDigestMismatch')
        result=compare_plan(actual,expected,evidence)
        durable_json(folder/'REFERENCE_COMPARISON.json',result)
        if not result['passed']:raise ValueError('CriticalControlScientificMismatch')
    return publish_checked(base,adapter,folder,pairs,publisher,check)


def worker():
    import os,traceback
    import numpy as np
    import psutil
    from sealed_front_connection import load_case,predict_bound
    from production_prism_stack import session,ExecutionProfile
    from composable_state import CommittedState
    from accepted_production_contract import bind_production,validate_authorization
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY,PRIOR
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    from production_prism_stack import recovered_execution_access,validate_execution_parent,committed_index_session
    from profile_remaining_prism import memory_sample
    from unittest.mock import patch
    started=time.perf_counter();stage='preflight';phases={};observed_writes=[]
    def save(n,v):durable_json(RUN/n,v)
    def timed(name,call):
        nonlocal stage
        stage=name;save('STAGE.json',dict(stage=stage,elapsed=time.perf_counter()-started));print(stage,flush=True)
        tick=time.perf_counter()
        with memory_sample(RUN,name):result=call()
        phases[name]=time.perf_counter()-tick;save('PHASES.json',phases)
        return result
    if (RUN/'RESULT.json').exists() or (RUN/'STOP.json').exists():raise ValueError('PreserveCriticalControlRun')
    try:
        with ExitStack() as held:
            c=verified(held)
            for k,v in FLAGS.items():
                if k in os.environ and os.environ[k]!=v:raise ValueError('ConflictingCriticalStorageControl',k)
            os.environ.update(FLAGS)
            case=read(PRIOR/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
            diagnostic=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
            historical,selected=existing_state_class().load(STUDY/'planner/selected_state')
            mapping=exact_vertex_map(historical.wall,wall)
            with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',allow_pickle=False) as z:first,refs=z['front'],z['face_refs']
            kernel,_=verify_kernel_sources()
            host=held.enter_context(CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60))
            obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
            authority=bind_sf2_policy(bind_canonical(obj)).authority
            original=timed('live_owner',lambda:AcceptedTrajectoryOwner(authority,diagnostic,identity(diagnostic),wall,first,tri,refs,mapping,selected,parent=case['committed']))
            validate_authorization(c['authorization_record'],c['authorization'],original)
            stack=held.enter_context(session(ExecutionProfile.PRODUCTION))
            base=timed('parent_open',lambda:held.enter_context(CommittedState(c['parent']['reference'])))
            validate_execution_parent(base,c,7)
            held.enter_context(recovered_execution_access(base,c,c['authorization_record']))
            accepted=timed('reference_open',lambda:held.enter_context(CommittedState(read(REFERENCE/'REOPEN.json')['reference'])))
            expected=authenticated_comparator(accepted,read(REFERENCE/'CANDIDATE_PLAN.json'),base.reference)
            adapter,inspected=timed('binding',lambda:bind_production(base,case,original,c['authorization_record'],c['authorization']))
            _,pairs=held.enter_context(committed_index_session(base,adapter,c))
            target=adapter.schedule.targets[6]
            predicted,valid=predict_bound(adapter,adapter.trajectory_spec,adapter.front.coordinates,target,iteration_cap=256)
            if predicted.coordinates.tobytes()!=original.positions()[7].tobytes() or not valid['admitted']:
                raise ValueError('CriticalControlDeclaredTargetMismatch')
            save('BINDING.json',dict(geometry=obj.identity,kernel=kernel,parent=base.scientific,authorization=c['authorization'],
                original_owner=original.source_identity,production_owner=adapter.geometry_binding.source_identity,
                schedule=original.schedule.sha256,increment=adapter.schedule.row_record(7),target=valid))
            measurements={}
            def observe(name,event,seconds):
                nonlocal stage
                stage=name
                if event=='before':
                    save('STAGE.json',dict(stage=stage,elapsed=time.perf_counter()-started));print(stage,flush=True)
                    measurement=ExitStack();held.enter_context(measurement)
                    measurement.enter_context(memory_sample(RUN,name));measurements[name]=measurement
                    if name=='generation':measurement.enter_context(observed_json_writes(observed_writes))
                else:
                    measurements.pop(name).close()
                    phases[name]=seconds;save('PHASES.json',phases)
            def check(actual):
                evidence=compare_dependencies(base,adapter,RUN,expected,accepted) if ALIGNED else None
                if evidence is not None and evidence['actual_semantic']!=actual['components']['forecast']['replace']:
                    raise ValueError('ActualDependencyDigestMismatch')
                result=compare_plan(actual,expected,evidence)
                durable_json(RUN/'REFERENCE_COMPARISON.json',result)
                if not result['passed']:raise ValueError('CriticalControlScientificMismatch')
                accepted.check()
            result=stack.step(base,adapter,RUN,pairs,observe=observe,prepare_check=check)
            save('GENERATION_WRITES.json',observed_writes)
            reference,scientific=result['reference'],result['scientific']
            import assembly_quality_handoff as reuse
            save('REUSE.json',dict(reuse._active.get()['stats']))
            base.check();accepted.check()
        save('RESULT.json',dict(passed=True,layer=7,scientific=scientific,reference=reference,phases=phases,
            admission_to_publication=sum(phases[k] for k in ('admission','generation','assembly','publication')),
            seconds=time.perf_counter()-started,peak_main_working_set=psutil.Process().memory_info().peak_wset,
            no_fresh_replay=True,no_deep_qualification_campaign=True,last_canonical_layer_unchanged=8))
    except Exception as error:
        save('STOP.json',dict(code=str(error),exception=type(error).__name__,stage=stage,phases=phases,
            traceback=traceback.format_exc(),seconds=time.perf_counter()-started))
        raise


def reopen():
    from composable_state import CommittedState
    from profile_remaining_prism import memory_sample
    started=time.perf_counter()
    with ExitStack() as held:
        verified(held);result=read(RUN/'RESULT.json')
        if result.get('passed') is not True or (RUN/'STOP.json').exists():raise ValueError('IncompleteCriticalControl')
        with memory_sample(RUN,'reopen'),CommittedState(result['reference']) as state:
            state.check()
            comparison=read(RUN/'REFERENCE_COMPARISON.json')
            if (state.layer!=7 or state.scientific!=result['scientific'] or comparison['passed'] is not True or
                comparison['scientific']!=state.scientific or comparison['reference']!=read(REFERENCE/'REOPEN.json')['scientific']):
                raise ValueError('CriticalControlReopenMismatch')
            durable_json(RUN/'REOPEN.json',dict(passed=True,layer=7,scientific=state.scientific,reference=state.reference,
                cells=len(state['cells']),proofs=len(state['certificate']),quality=len(state['quality']),
                seconds=time.perf_counter()-started,stats=dict(state.stats)))


def monitor():
    import finish_retained_l5 as existing
    with ExitStack() as held:verified(held)
    existing.CAMPAIGN=RUN;existing.OUT=MONITOR;MONITOR.mkdir(exist_ok=False)
    command=[sys.executable,'-B',__file__,'worker']
    if ALIGNED:command+=['--output',str(OUT)]
    return existing.monitor(worker_command=command,enforce_output_cap=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('baseline','worker','monitor','reopen'))
    p.add_argument('--output',type=Path);a=p.parse_args();configure_output(a.output)
    result=globals()[a.mode]()
    if a.mode=='monitor':sys.exit(result)
