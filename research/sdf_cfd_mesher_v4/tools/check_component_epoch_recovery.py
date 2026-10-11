"""Configure existing metadata-only recovery and read-only retention planning.

No front executor or cleanup operation is reachable from this control.
"""
from pathlib import Path
import json
import subprocess
import sys
import time

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
OUT=V4/'results/component_epoch_recovery'
from path_preserving_deduplication import durable_json,file_hash,metadata
from record_receipt_resume import disk,status
from check_current_front_encoding import allocation

OWN=['research/sdf_cfd_mesher_v4/'+p for p in ('CURRENT_TASK.md','README.md',
     'COMPONENT_EPOCH_RECOVERY_REPORT.md','tools/check_component_epoch_recovery.py',
     'tools/recursive_epoch_recovery.py','tools/recursive_epoch_recovery.md',
     'tests/test_component_epoch_recovery.py','results/component_epoch_recovery/')]


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def unrelated(rows,own=OWN):
    return [r for r in rows if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in own)]
def totals(paths):return {k:v for k,v in allocation(paths).items() if k!='records'}


def baseline():
    if OUT.exists():raise ValueError('PreserveComponentRecoveryEvidence')
    previous=read(V4/'results/l5_recovery_preflight/BASELINE.json')
    expected=read(V4/'results/retained_l6_storage_plan/BASELINE.json')['unrelated']
    if unrelated(status())!=expected:raise ValueError('ExactUnrelatedStatusChanged')
    protected=previous['protected']
    if any(file_hash(p)!=h for p,h in protected.items()):raise ValueError('ProtectedSourceChanged')
    OUT.mkdir()
    pointer=V4/'results/current_front_transition_round_trip/A/L5/REOPEN.json'
    durable_json(OUT/'BASELINE.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        utc=time.time(),unrelated=expected,protected=protected,disk=disk(),
        source=read(pointer),source_record_sha256=file_hash(pointer),
        intentional_source_changes=['recursive_epoch_recovery.py'],
        retained_inventory_sha256=file_hash(V4/'results/single_quality_storage/BASELINE.json')))


def worker():
    import traceback,psutil
    import recursive_epoch_recovery as recovery
    start=time.perf_counter();b=read(OUT/'BASELINE.json')
    try:
        validate_worker_binding(b,read(OUT/'REVIEW.json'))
        source=b['source']
        result=recovery.recover_components(source['reference'],source['scientific'],OUT/'access',
            progress=lambda s:print(s,flush=True))
        durable_json(OUT/'RECOVERY.json',dict(passed=True,record=result,
            seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset))
    except BaseException as exc:
        durable_json(OUT/'FAILURE.json',dict(error=repr(exc),traceback=traceback.format_exc(),
            seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset))
        raise


def validate_worker_binding(b,review):
    """Qualified input and independently reviewed current implementation pins."""
    pointer=V4/'results/current_front_transition_round_trip/A/L5/REOPEN.json'
    if file_hash(pointer)!=b['source_record_sha256'] or read(pointer)!=b['source']:
        raise ValueError('ChangedQualifiedL5Source')
    if b['source'].get('passed') is not True or b['source']['scientific']!='905511e4d6a2edcf0ca00dadd1f4ee8522bdb39634edfc1e83928eddf7074019':
        raise ValueError('WrongQualifiedL5Input')
    required=(Path(__file__).resolve(),V4/'tools/recursive_epoch_recovery.py',V4/'tools/composable_state.py')
    if review.get('passed') is not True or review.get('baseline_sha256')!=file_hash(OUT/'BASELINE.json'):
        raise ValueError('IndependentReviewedBindingRequired')
    for path in required:
        if review['implementation_hashes'].get(str(path))!=file_hash(path):raise ValueError('ReviewedImplementationChanged')
    for path,sha in b['protected'].items():
        if Path(path).name not in b['intentional_source_changes'] and file_hash(path)!=sha:
            raise ValueError('ProtectedSourceChanged',path)


def monitor():
    """Existing resource values; access-only task, not a production cap-fit claim."""
    import psutil
    from finish_retained_l5 import live_allocation
    if (OUT/'WORKER.log').exists():raise ValueError('PreservePriorRecoveryAttempt')
    controls=read(V4/'results/wing_continuation_layers6to8/MONITOR.json')['resources']
    if disk()['C:\\']['available']<controls['minimum_free_bytes']:raise ValueError('ResourceStopBeforeRecovery')
    start=time.perf_counter();peak=0
    with (OUT/'WORKER.log').open('xb') as log:
        proc=subprocess.Popen([sys.executable,'-B',__file__,'worker'],stdout=log,stderr=subprocess.STDOUT)
        while True:
            try:peak=max(peak,psutil.Process(proc.pid).memory_info().peak_wset)
            except psutil.NoSuchProcess:pass
            files=[p for p in OUT.rglob('*') if p.is_file()]
            measured=live_allocation(files);free=disk()['C:\\']['available']
            row=dict(seconds=time.perf_counter()-start,peak_working_set=peak,allocation=measured,free=free)
            if free<controls['minimum_free_bytes'] or measured['allocated']>controls['output_limit_bytes']:
                proc.terminate();proc.wait();durable_json(OUT/'RESOURCE_STOP.json',dict(controls=controls,measurement=row))
                return 1
            code=proc.poll()
            if code is not None:
                durable_json(OUT/'MONITOR.json',dict(controls=controls,measurement=row,returncode=code,
                    scope='Metadata access recovery only. Original production campaign remains above its cap; no fit claim.'))
                return code
            time.sleep(controls['monitor_seconds'])


def opened():
    from composable_state import CommittedState
    from current_front_dependencies import install_reader
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure
    import psutil
    destination=OUT/(sys.argv[2] if len(sys.argv)>2 else 'OPEN.json')
    if destination.parent!=OUT or destination.exists():raise ValueError('PreserveFreshOpenEvidence')
    install_reader();start=time.perf_counter();rec=discover(OUT/'access')
    with authenticated_closure(rec['reference']) as (state,closure,nodes):
        if state.layer!=5 or state.content_root!=rec['content']:raise ValueError('RecoveredL5Mismatch')
        # Actual installed compact forecast reader; no forecast/predictor runs.
        view=state['forecast'];rows=len(view);first=next(iter(view));sample=view[first]
        state.check()
        durable_json(destination,dict(passed=True,layer=state.layer,scientific=state.scientific,
            reference=state.reference,closure=closure,nodes=nodes,forecast_rows=rows,
            forecast_sample_key=first,forecast_sample_identity=__import__('sdf_cfd_mesher_v4.common.authority',fromlist=['identity']).identity(sample),
            stats=dict(state.stats),seconds=time.perf_counter()-start,
            peak_working_set=psutil.Process().memory_info().peak_wset,no_forecast_or_generation=True))


def planning():
    """Only existing successful retention manifests; no apply/unlink/relink."""
    from current_front_dependencies import install_reader
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure,plan
    import psutil
    if (OUT/'PLAN.json').exists():raise ValueError('PreserveReclamationPlan')
    install_reader();start=time.perf_counter();rec=discover(OUT/'access')
    old=read(V4/'results/l5_recovery_preflight/PREFLIGHT.json')
    records=read(V4/'results/single_quality_storage/BASELINE.json')['census']['records']
    folders=[V4/'results/wing_receipt_resume/A'/f'L{k}' for k in range(2,6)]
    folders+=[V4/'results/current_front_transition_round_trip/A/L5',V4/'results/wing_continuation_layers6to8/A/L6']
    completed=[];candidates=[];protected=[]
    with authenticated_closure(rec['reference']) as (state,closure,nodes):
        if state.layer!=5 or state.scientific!=rec['content']:raise ValueError('PlanParentMismatch')
        prior_results=[V4/'results/wing_storage_connection'/name/'RESULT.json' for name in ('L2_active_closure','L3','L4')]
        prior_results += [V4/'results/retained_l5_finish/retention_recovery/retention/RESULT.json',
                          V4/'results/current_front_transition_round_trip/A/L5/replay_retention/RESULT.json']
        for path in prior_results:
            result=read(path);compact_path=path.parent/'COMPACT.json';compact=read(compact_path)
            record=read(path.parent/'PLAN.json')
            if result.get('passed') is not True or file_hash(compact_path)!=record['compact_sha256']:
                raise ValueError('ChangedPriorRetentionEvidence')
            absent=[];known=0
            for row in compact['discarded']:
                if Path(row['path']).exists():raise ValueError('PriorRetiredPathReappeared',row['path'])
                canonical=row['proof']['canonical'];sha=row['sha256']
                if canonical in closure:
                    if closure[canonical]['sha256']!=sha:raise ValueError('RetainedCanonicalIdentityMismatch')
                    known+=1
                absent.append(row['path'])
            completed.append(dict(result=str(path),sha256=file_hash(path),compact_sha256=file_hash(compact_path),
                retired_files=len(absent),historical_allocation_reclaimed=result['allocation_reclaimed'],
                recovered_closure_canonicals=known,current_reclamation_credit=0))
        for folder in folders:
            replay=folder/'replay'
            if not (folder/'REOPEN.json').exists():
                protected.append(dict(folder=str(folder),reason='NoSuccessfulPublicationAndGuardedReopen',new_eligible_bytes=0));continue
            # Already-completed retention removed private proof/array exports.
            if not any((replay/name).exists() for name in ('GENERATED_NEXT.npz','QUALITY_DELTA.json','CANDIDATE_HOT.json')):
                protected.append(dict(folder=str(folder),reason='PrivateReplayExportsAlreadyRetired',new_eligible_bytes=0));continue
            result=plan(folder,OUT/'retention_plans'/folder.parent.parent.name/folder.name,rec['reference'])
            candidates.append(dict(folder=str(folder),actions=result['actions']))
        active=[Path(r['path']) for r in records if r['path'] in closure]
        outside=[Path(r['path']) for r in records if r['path'] not in closure]
        allocated=totals([Path(r['path']) for r in records])['unique_allocation_bytes']
        actions=[a for group in candidates for a in group['actions']]
        eligible=sum(a['cluster_bytes'] for a in actions)
        cap=old['resources']['output_limit_bytes'];lower=allocated+2*old['resources']['single_quality_allocated_bytes']
        durable_json(OUT/'PLAN.json',dict(authenticated_L5=state.scientific,reference=rec['reference'],
            declared_campaign=totals(active),not_declared_campaign=totals(outside),campaign_allocated=allocated,
            completed_retention=completed,remaining_candidates=candidates,protected=protected,
            newly_eligible_allocated_bytes=eligible,reclaimed_this_task=0,
            output_limit_bytes=cap,two_independent_quality_lower_bound=lower,
            lower_bound_deficit_after_eligible=max(0,lower-eligible-cap),
            peak_upper_bound_established=False,production_resume_allowed=False,
            no_cleanup=True,no_layer_advance=True,
            scope='Uses existing successful-only retention predicate. Outside closure is not disposable; previous reclamation cannot be credited twice. Failed L6 and sealed historical states stay intact.',
            seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset))


def finish():
    if (OUT/'FINAL.json').exists():raise ValueError('PreserveFinalEvidence')
    b=read(OUT/'BASELINE.json')
    if unrelated(status())!=b['unrelated']:raise ValueError('ExactUnrelatedStatusChanged')
    preserved={};changed={}
    for path,sha in b['protected'].items():
        actual=file_hash(path)
        if Path(path).name in b['intentional_source_changes']:changed[path]=dict(before=sha,after=actual)
        elif actual!=sha:raise ValueError('ProtectedSourceChanged',path)
        else:preserved[path]=sha
    geometry=[]
    prior=read(V4/'results/current_front_dependency_encoding/BASELINE.json')
    for row in prior['protected']:
        if row['path'].replace('\\','/').startswith('src/sdf/aerospace/'):
            if file_hash(ROOT/row['path'])!=row['sha256']:raise ValueError('ProtectedKernelGeometryChanged')
            geometry.append(row)
    rows=read(V4/'results/single_quality_storage/BASELINE.json')['census']['records']
    fields=('device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams')
    for row in rows:
        now=metadata(row['path'])
        if any(now[k]!=row[k] for k in fields):raise ValueError('RetainedScientificFileChanged',row['path'])
    if file_hash(V4/'results/current_front_transition_round_trip/A/L5/REOPEN.json')!=b['source_record_sha256']:
        raise ValueError('SealedL5ReferenceChanged')
    files=[p for p in OUT.rglob('*') if p.is_file()]+[ROOT/p for p in OWN if not p.endswith('/')]
    durable_json(OUT/'FINAL.json',dict(disk=disk(),unrelated_status_exact=True,unrelated_entries=len(b['unrelated']),
        retained_campaign_files_preserved=len(rows),protected_sources_preserved=preserved,
        retained_kernel_geometry_hashes=geometry,
        intentional_source_changes=changed,task_allocation_before_final=totals(files),
        hashes={str(p.relative_to(ROOT)):file_hash(p) for p in files},
        baseline_to_finish_seconds=time.time()-b['utc'],no_cleanup=True,no_layer_advance=True))


if __name__=='__main__':
    mode=sys.argv[1]
    if mode=='monitor':sys.exit(monitor())
    {'baseline':baseline,'worker':worker,'open':opened,'plan':planning,'finish':finish}[mode]()
