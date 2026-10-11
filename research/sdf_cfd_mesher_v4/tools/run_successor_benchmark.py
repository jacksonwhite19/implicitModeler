"""Configure the shared optimized successor executor; measure marginal output.

No geometry, proof, transaction or publication implementation lives here.
The original L6-L8 authorization and sealed L6 remain unchanged.
"""
import argparse
import ast
import hashlib
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys
import time
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from benchmark_retained_l6 import read,totals
from record_receipt_resume import disk,status
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity

OUT=V4/'results/fresh_successor_benchmark'
RUN=OUT/'current_parent_run'
MONITOR=OUT/'monitor_current_parent'
PARENT=V4/'results/retained_l6_workflow/actual_prepare'
AUTH=V4/'results/wing_continuation_layers6to8/AUTHORIZATION.json'
OWN=['research/sdf_cfd_mesher_v4/'+p for p in ('CURRENT_TASK.md','README.md',
    'FRESH_SUCCESSOR_BENCHMARK_REPORT.md','tools/run_authorized_continuation.py',
    'tools/run_successor_benchmark.py','tests/test_successor_execution.py',
    'tests/test_recovered_parent_authorization.py',
    'results/fresh_successor_benchmark/')]
from capsule_reuse_dispatch import PRODUCTION_CONTROLS as FLAGS


def execution_sources(seed):
    """Conservative local import closure, including deferred imports and natives.

    Parse source without executing installers. Keep existing external pins.
    Local literal native source paths are pinned alongside importing Python.
    """
    roots=(V4/'tools',V4/'src')
    pending=[Path(p).resolve() for p in seed];found=set()
    while pending:
        p=pending.pop()
        if p in found:continue
        if not p.is_file():raise ValueError('MissingSuccessorExecutionSource',str(p))
        found.add(p)
        if p.suffix!='.py':continue
        tree=ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p))
        for node in ast.walk(tree):
            modules=[]
            if isinstance(node,ast.Import):modules=[a.name for a in node.names]
            elif isinstance(node,ast.ImportFrom):
                name=node.module or ''
                if node.level:
                    parent=p.parent
                    for _ in range(node.level-1):parent=parent.parent
                    candidate=parent.joinpath(*name.split('.')) if name else parent
                    for q in (candidate.with_suffix('.py'),candidate/'__init__.py'):
                        if q.is_file():pending.append(q.resolve())
                    for alias in node.names:
                        q=candidate/(alias.name+'.py')
                        if q.is_file():pending.append(q.resolve())
                else:modules=[name]+[name+'.'+a.name for a in node.names]
            for name in modules:
                for root in roots:
                    candidate=root.joinpath(*name.split('.'))
                    for q in (candidate.with_suffix('.py'),candidate/'__init__.py'):
                        if q.is_file():pending.append(q.resolve())
            if isinstance(node,ast.Constant) and isinstance(node.value,str) and node.value.endswith(('.rs','.py')):
                # with_name()/tools-relative native source and rooted repo sources.
                for root in (p.parent,V4/'tools',V4,V4.parent/'sdf_cfd_mesher_v3',ROOT):
                    q=root/node.value
                    if q.is_file():pending.append(q.resolve())
    return {str(p):file_hash(p) for p in sorted(found)}


def execution_binaries():
    """Pin existing source-addressed DLLs; never invoke a build for preflight."""
    from sdf_cfd_mesher_v4.certificate.efficient import SOURCE,ROOT as certificate_root
    from optimized_mixed_certificate import NATIVE
    from canonical_spline_authority import DLL
    source_bytes=SOURCE.read_bytes()
    paths=[ROOT/'target/debug/canonical_kernel_host.exe',DLL,
        certificate_root/'results/task4b/native'/('proof_'+hashlib.sha256(source_bytes).hexdigest()+'.dll'),
        V4/'results/production_hotpath_optimization/native'/
            (hashlib.sha256(NATIVE.read_bytes()+source_bytes).hexdigest()+'.dll')]
    if any(not p.is_file() for p in paths):raise ValueError('MissingExistingSuccessorBinary',paths)
    return {str(p.resolve()):file_hash(p) for p in paths}


def retained_execution_inputs(seed):
    """Hold the actual case and recovered trajectory inputs, not proof closure."""
    pins=dict(seed)
    case_path=V4/'results/sealed_front_connection/CASES.json'
    diagnostic_path=V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json'
    for p in (case_path,diagnostic_path):pins[str(p)]=file_hash(p)
    pins.update(read(case_path)['wing']['sources'])
    for row in read(diagnostic_path)['sources']:
        pins[str((ROOT/row['path']).resolve())]=row['sha256']
    for p,h in pins.items():
        if file_hash(p)!=h:raise ValueError('ChangedRetainedExecutionInput',p)
    return pins


def unrelated():
    return [r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in OWN)]


def request():
    """Pin the final tested caller source without replacing an earlier request."""
    path=OUT/'REVIEW_REQUEST.json'
    if path.exists():raise ValueError('PreserveSuccessorReviewRequest')
    before=read(OUT/'REVIEW_REQUEST_BEFORE_REVIEW_FIX.json')
    before['runtime']=execution_sources(before['runtime'])
    before['binary']=execution_binaries()
    before['inputs']=retained_execution_inputs(before['inputs'])
    if before['baseline_sha256']!=file_hash(OUT/'BASELINE.json') or before['control_sha256']!=file_hash(OUT/'CONTROL.json'):
        raise ValueError('ChangedSuccessorControl')
    durable_json(path,before)


def baseline():
    if OUT.exists():raise ValueError('PreserveSuccessorBenchmark')
    prior=read(V4/'results/preparation_snapshot_profile/BASELINE.json')
    if unrelated()!=prior['unrelated']:raise ValueError('ChangedUnrelatedSuccessorStatus')
    for kind in ('protected','inputs'):
        if any(file_hash(p)!=h for p,h in prior[kind].items()):raise ValueError('ChangedProtectedSuccessorInput',kind)
    runtime=read(V4/'results/retained_l6_workflow/AUDIT_REQUEST.json')['runtime']
    runtime.update({str(V4/'tools'/n):file_hash(V4/'tools'/n) for n in
        ('run_authorized_continuation.py','run_successor_benchmark.py','recursive_replay_retention.py')})
    runtime=execution_sources(runtime)
    names=[AUTH,PARENT/'REOPEN.json',PARENT/'REFERENCE.json',PARENT/'CANDIDATE_PLAN.json',
        PARENT/'CANDIDATE_ACCEPTANCE.json',V4/'results/component_epoch_recovery/RECOVERY.json',
        V4/'results/component_epoch_recovery/access/CURRENT.json',
        V4/'results/live_sf2_first_layer/PREFLIGHT_A.json',
        V4/'results/smooth_wing_recovery/recovered_sf2.rhai']
    recovery=read(V4/'results/component_epoch_recovery/RECOVERY.json')['record']
    names += [V4/'results/component_epoch_recovery/access'/('RECOVERY_'+identity(recovery)+'.json')]
    original=read(AUTH);parent=dict(read(PARENT/'REOPEN.json'),layer=6)
    if identity(original)!='022ba294325d1e26e85634609fee42fed307f38df87a55f881d83001a8c58ce1':
        raise ValueError('WrongSuccessorAuthorization')
    if parent['scientific']!='61831ef2187d1adf083bd6a81858cba9ae811b1e1c47db093405db6356a2fade':
        raise ValueError('WrongQualifiedSuccessorParent')
    resources=read(V4/'results/live_sf2_first_layer/PREFLIGHT_A.json')['resources']
    control=dict(schema='V4AuthorizedContinuationExecution/1',layers=[7,8],resume=True,parent=parent,
        authorization_record=original,authorization=identity(original),compare_inside_publisher=True,
        resource_policy=dict(total_storage_policy='monitor_without_output_cap',output_limit_bytes=None,
                             minimum_free_bytes=resources['minimum_free_bytes']),
        recovered_access=dict(directory=str(V4/'results/component_epoch_recovery/access'),
            identity=identity(recovery),reference=recovery['reference']))
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),unrelated=unrelated(),
        protected=prior['protected'],inputs=prior['inputs'],disk=disk(),
        scope='Fresh L7 then L8 through the existing shared optimized executor; unchanged authorization, schedule and science.'))
    durable_json(OUT/'CONTROL.json',control)
    durable_json(OUT/'REVIEW_REQUEST.json',dict(runtime=runtime,inputs={str(p):file_hash(p) for p in names},
        binary=execution_binaries(),baseline_sha256=file_hash(OUT/'BASELINE.json'),
        control_sha256=file_hash(OUT/'CONTROL.json'),flags=FLAGS))
    print('Prepared reviewed successor control',control['layers'],parent['scientific'],disk(),flush=True)


def verified(held):
    import immutable_state_access as access
    for p in (OUT/'REVIEW_REQUEST.json',OUT/'REVIEW.json',OUT/'BASELINE.json',OUT/'CONTROL.json'):
        held.enter_context(access.guard(p))
    q=read(OUT/'REVIEW_REQUEST.json');review=read(OUT/'REVIEW.json')
    if (review.get('passed') is not True or not review.get('reviewer')
        or review['request_sha256']!=file_hash(OUT/'REVIEW_REQUEST.json') or review['runtime']!=q['runtime']
        or q['baseline_sha256']!=file_hash(OUT/'BASELINE.json') or q['control_sha256']!=file_hash(OUT/'CONTROL.json')
        or q['flags']!=FLAGS):raise ValueError('UnreviewedSuccessorExecution')
    for p,h in {**q['runtime'],**q['inputs'],**q['binary']}.items():
        held.enter_context(access.guard(p))
        if file_hash(p)!=h:raise ValueError('ChangedSuccessorDependency',p)
    import run_authorized_continuation as runner
    control=read(OUT/'CONTROL.json');runner.continuation_scope(control,read(AUTH),identity(read(AUTH)))
    return control


def preflight():
    from composable_state import CommittedState
    from sealed_front_connection import qualified_dispatch
    with ExitStack() as held:
        c=verified(held)
        with qualified_dispatch(),CommittedState(c['parent']['reference']) as base:
            base.check()
            if base.layer!=6 or base.scientific!=c['parent']['scientific']:raise ValueError('WrongActualBenchmarkParent')
            durable_json(OUT/'PREFLIGHT_CURRENT_PARENT.json',dict(passed=True,scientific=base.scientific,layer=base.layer,
                reference=base.reference,proof_count=len(base['certificate']),quality_count=len(base['quality']),disk=disk()))


def worker():
    import os
    import run_authorized_continuation as runner
    with ExitStack() as held:
        c=verified(held)
        for k,v in FLAGS.items():
            if k in os.environ and os.environ[k]!=v:raise ValueError('ConflictingSuccessorStorageControl',k)
        os.environ.update(FLAGS)
        runner.OUT=RUN;runner.OUT.mkdir(exist_ok=False)
        runner.execute(continuation=c)


def monitor():
    import finish_retained_l5 as m
    with ExitStack() as held:verified(held)
    m.OUT=MONITOR;m.OUT.mkdir(exist_ok=False)
    return m.monitor(worker_command=[sys.executable,'-B',__file__,'worker'],enforce_output_cap=False,
        extra_storage_roots=[V4/'results/wing_continuation_layers6to8',
            V4/'results/current_front_transition_round_trip',V4/'results/retained_l6_workflow',RUN])


def completed_evidence(result,stop,monitor,records,last,layers):
    """No aggregate success without each qualified/replayed/reopened child."""
    if (not result or result.get('passed') is not True or stop is not None
        or type(monitor.get('exit_code')) is not int or monitor['exit_code']!=0
        or type(monitor.get('worker_exit_code')) is not int or monitor['worker_exit_code']!=0
        or monitor.get('resource_stopped') is not False
        or result.get('last_qualified_layer')!=layers[-1] or last!=layers[-1]):return False
    for layer in layers:
        rows=records.get(f'L{layer}',{}).get('evidence',{})
        if not all(n in rows for n in ('ADMISSION.json','GENERATION.json','ASSEMBLY.json',
            'FRESH_CERTIFICATE.json','VOLUME_DELTA.json','REPLAY.json','PUBLICATION.json','REOPEN.json',
            'CANDIDATE_PLAN.json','replay/CANDIDATE_PLAN.json')):return False
        replay=rows['REPLAY.json']['value'];opened=rows['REOPEN.json']['value']
        if (replay.get('passed') is not True or not replay.get('comparisons')
            or not all(v is True for v in replay['comparisons'].values())
            or replay.get('content')!=replay.get('replay_content')
            or opened.get('passed') is not True or opened.get('scientific')!=replay.get('content')):return False
        generation=rows['GENERATION.json']['value'];certificate=rows['FRESH_CERTIFICATE.json']['value']
        if (generation.get('hard_failures')!=[] or generation.get('topology_audit',{}).get('passed') is not True
            or certificate.get('passed') is not True or certificate.get('failures')!=[]):return False
        decision=rows['ADMISSION.json']['value'].get('decision',{})
        volume=rows['VOLUME_DELTA.json']['value'];published=rows['PUBLICATION.json']['value']
        primary=rows['CANDIDATE_PLAN.json']['value'];other=rows['replay/CANDIDATE_PLAN.json']['value']
        if (decision.get('generation_attempt_authorized') is not True or decision.get('next_layer')!=layer
            or decision.get('publication_authorized') is not False
            or generation.get('transition_plan',{}).get('target_layer')!=layer
            or volume.get('exact_balance') is not True or volume.get('global_degree',{}).get('passed') is not True
            or published.get('root')!=opened.get('reference',{}).get('root')
            or published.get('content_root')!=opened.get('scientific')
            or primary.get('components')!=other.get('components')
            or set(replay['comparisons'])!=set(primary.get('components',{}))
            or primary.get('content')!=other.get('content')
            or identity(primary.get('content'))!=primary.get('scientific')
            or primary.get('scientific')!=other.get('scientific')
            or primary.get('scientific')!=replay.get('content')):return False
    return True


def compact_volume(value):
    """Report full witness outcomes without copying the authenticated rows.

    The original file stays intact and its SHA is recorded by collect(). This
    does not change a production result, proof, reader or acceptance decision.
    """
    from collections import Counter
    degree=value['global_degree']
    rows=degree['values']
    return dict(value,global_degree={**{k:v for k,v in degree.items() if k!='values'},
        'witness_count':len(rows),
        'degree_counts':dict(Counter('unresolved' if r is None else str(r['degree']) for r in rows)),
        'rows_retained_in':'VOLUME_DELTA.json'})


def collect():
    if (OUT/'EVIDENCE.json').exists():raise ValueError('PreserveSuccessorEvidence')
    b=read(OUT/'BASELINE.json')
    if unrelated()!=b['unrelated']:raise ValueError('ChangedSuccessorUnrelatedStatus')
    for kind in ('protected','inputs'):
        if any(file_hash(p)!=h for p,h in b[kind].items()):raise ValueError('ChangedSuccessorProtectedInput',kind)
    m=read(MONITOR/'MONITOR_FINAL.json');run=RUN
    result=read(run/'RESULT.json') if (run/'RESULT.json').exists() else None
    stop=read(run/'STOP.json') if (run/'STOP.json').exists() else None
    records={}
    for folder in sorted(run.glob('L*')):
        if not folder.is_dir():continue
        record={n:dict(value=read(folder/n),sha256=file_hash(folder/n)) for n in
            ('CONNECTION.json','TARGET.json','ADMISSION.json','GENERATION.json','ASSEMBLY.json',
             'FRESH_CERTIFICATE.json','VOLUME_DELTA.json','REPLAY.json','PUBLICATION.json','REOPEN.json') if (folder/n).exists()}
        if 'VOLUME_DELTA.json' in record:
            record['VOLUME_DELTA.json']['value']=compact_volume(record['VOLUME_DELTA.json']['value'])
        for name in ('CANDIDATE_PLAN.json','replay/CANDIDATE_PLAN.json'):
            if (folder/name).exists():record[name]=dict(value=read(folder/name),sha256=file_hash(folder/name))
        records[folder.name]=dict(evidence=record,allocation=totals([p for p in folder.rglob('*') if p.is_file()]))
    paths=[p for p in OUT.rglob('*') if p.is_file()]
    last=read(run/'LAST_QUALIFIED.json')['layer'] if (run/'LAST_QUALIFIED.json').exists() else 6
    durable_json(OUT/'EVIDENCE.json',dict(result=result,stop=stop,layers=records,
        passed=completed_evidence(result,stop,m,records,last,read(OUT/'CONTROL.json')['layers']),
        last_qualified_layer=last,
        phases=read(run/'PHASES.json') if (run/'PHASES.json').exists() else {},
        measurements=read(run/'MEASUREMENTS.json') if (run/'MEASUREMENTS.json').exists() else [],
        monitor=dict(exit_code=m['exit_code'],worker_exit_code=m['worker_exit_code'],resource_stopped=m['resource_stopped'],
            seconds=m['seconds'],peaks=m['peaks'],storage=m['storage']),
        final_disk=disk(),new_output=totals(paths),preservation=dict(exact_status_entries=len(b['unrelated']),
            protected_hashes=len(b['protected']),input_pins=len(b['inputs'])),elapsed=time.time()-b['start']))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','request','preflight','worker','monitor','collect']);a=p.parse_args()
    if a.mode=='monitor':sys.exit(monitor())
    globals()[a.mode]()
