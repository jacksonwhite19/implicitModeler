"""Existing monitor preflight and read-only retained-L6 storage diagnosis.

Never launches a layer, transaction, recovery or retention executor. A failed
resource gate leaves only read-only diagnosis available. No cap redefinition.
"""
from contextlib import ExitStack
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
OUT=V4/'results/retained_l6_storage_plan'
from check_current_front_encoding import allocation
from path_preserving_deduplication import durable_json,file_hash,metadata,physical_key
from record_receipt_resume import disk,status
import immutable_state_access as access
from sdf_cfd_mesher_v4.common.authority import identity


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def compact(rows):return {k:v for k,v in allocation(rows).items() if k!='records'}
def forbidden_launch(*args,**kwargs):raise RuntimeError('StorageOnlyTaskCannotLaunchWorker')


def already_checked(checked,path,record):
    if path not in checked:return False
    if checked[path]!=record:raise ValueError('ConflictingRetainedReference',path)
    return True


def main():
    import psutil
    import finish_retained_l5 as monitored
    import single_quality_component as codec
    from recursive_replay_retention import authenticated_closure,verify_replay
    start=time.perf_counter();head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if head!='7dfcf814010e02a4377e7f4d1d9c6b24216e7e36':raise ValueError('UnexpectedStoragePlanHead')
    if OUT.exists():raise ValueError('PreservePriorStoragePlan')
    roots=[V4/'results'/n for n in ('wing_receipt_resume/A','current_front_transition_round_trip','wing_continuation_layers6to8')]
    prior=V4/'results/single_quality_storage'
    before=read(prior/'BASELINE.json');original=before['census']['records']
    fields=('device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams')
    for row in original:
        now=metadata(row['path'])
        if any(now[k]!=row[k] for k in fields):raise ValueError('ChangedRetainedStorageInput',row['path'])
    owned=['research/sdf_cfd_mesher_v4/'+p for p in ('CURRENT_TASK.md','README.md','RETAINED_L6_STORAGE_PLAN_REPORT.md',
        'tools/check_retained_l6_storage_plan.py','tests/test_retained_l6_storage_plan.py','results/retained_l6_storage_plan/')]
    def unrelated():return [r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in owned)]
    sources=[V4/'tools'/p for p in ('single_quality_component.py','critical_assembly.py','critical_prepare.py',
        'critical_transaction.py','composable_publication.py','composable_state.py','immutable_state_access.py',
        'incremental_pair_storage.py','recursive_production_publication.py','recursive_replay_retention.py','finish_retained_l5.py')]
    sources += [ROOT/'src/sdf/aerospace'/p for p in ('wing.rs','airfoil.rs','naca_spline.rs')]
    protected={str(p):file_hash(p) for p in sources}
    previous=read(prior/'EVIDENCE.json')
    for row in previous['dirty_kernel_sources']:
        if file_hash(ROOT/row['path'])!=row['sha256']:raise ValueError('DirtyKernelSnapshotChanged')
    initial=status();initial_unrelated=unrelated();initial_disk=disk()
    campaign=[p for root in roots for p in root.rglob('*') if p.is_file()]
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(head=head,branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        status=initial,unrelated=initial_unrelated,protected=protected,disk=initial_disk,
        previous_inventory_sha256=file_hash(prior/'BASELINE.json'),original_files=len(original),
        campaign=compact(campaign),scope='Previous bounded inventory reused; no new whole-repository inventory.'))
    # Run the REAL existing monitor's resource gate. Prevent all worker launch,
    # including an unexpected pass; a pass is not execution authorization here.
    monitored.OUT=OUT
    with patch.object(monitored.subprocess,'Popen',forbidden_launch):
        monitor_result=monitored.monitor(worker_command=['storage-only-no-worker'],extra_storage_roots=tuple(roots[1:]))
    if monitor_result!=1 or not (OUT/'RESOURCE_STOP.json').exists():
        raise ValueError('ExpectedResourceStopNotObserved')
    stop=read(OUT/'RESOURCE_STOP.json');limit=stop['resources']['output_limit_bytes']
    print('existing monitor stopped before worker launch',stop['measurement'],flush=True)
    # Only bounded read-only diagnosis follows the resource stop.
    l6=roots[2]/'A/L6';a=read(l6/'CANDIDATE_PLAN.json');b=read(l6/'replay/CANDIDATE_PLAN.json')
    acceptance=read(l6/'CANDIDATE_ACCEPTANCE.json');replay=read(l6/'REPLAY.json')
    parent=read(roots[1]/'A/L5/REOPEN.json')
    if a['parent']!=parent['reference'] or b['parent']!=a['parent']:raise ValueError('WrongRetainedL6Parent')
    if not acceptance['passed'] or acceptance['plan']!=identity(a):raise ValueError('WrongRetainedL6Acceptance')
    comparisons={k:b['components'].get(k)==v for k,v in a['components'].items()}
    if (not all(comparisons.values()) or comparisons!=replay['comparisons'] or not replay['passed']
        or set(a['components'])!=set(b['components']) or identity(a['content'])!=a['scientific']
        or identity(b['content'])!=b['scientific'] or a['scientific']!=b['scientific']
        or a['scientific']!=replay['content'] or a['scientific']!=replay['replay_content']):
        raise ValueError('RetainedL6ReplayMismatch')
    shared=read(prior/'retained_L6_rechecked/QUALITY_DELTA.json')
    payload=Path(shared['payload']['path'])
    if shared['encoder_sha256']!=file_hash(codec.__file__) or shared['logical']['sha256']!=a['components']['quality']['delta']:
        raise ValueError('QualitySourceOrLogicalMismatch')
    with authenticated_closure(parent['reference']) as (base,closure,nodes),ExitStack() as held:
        if base.layer!=5 or base.scientific!=parent['scientific']:raise ValueError('WrongGuardedL5')
        checked={};hashed_bytes=0;inherited_tokens=0
        for name,plan in (('primary',a),('replay',b)):
            for p,r in plan['references'].items():
                if already_checked(checked,p,r):continue
                held.enter_context(access.guard(p))
                if access.token(p)!=r['token']:raise ValueError('ChangedRetainedReference',p)
                # Reuse authenticated ancestry; independently hash new inputs.
                if p in closure and closure[p].get('sha256')==r['sha256']:inherited_tokens+=1
                else:
                    if file_hash(p)!=r['sha256']:raise ValueError('RetainedL6HashMismatch',p)
                    hashed_bytes+=Path(p).stat().st_size
                checked[p]=deepcopy(r)
            print(name,'retained references verified',len(checked),flush=True)
        for n,h in acceptance['evidence'].items():
            if file_hash(l6/n)!=h:raise ValueError('ChangedAcceptedL6Evidence',n)
        held.enter_context(access.guard(payload))
        if access.token(payload)!=shared['payload']['token'] or file_hash(payload)!=shared['payload']['sha256']:
            raise ValueError('ChangedSharedDiagnosticQuality')
        active_paths=[Path(p) for p in closure]
        active=compact(active_paths)
        in_campaign={str(p.resolve()) for p in campaign}
        active_here=compact([p for p in active_paths if str(p.resolve()) in in_campaign])
        active_elsewhere=compact([p for p in active_paths if str(p.resolve()) not in in_campaign])
        base.check();stats=dict(base.stats)
    retention_block=None
    try:verify_replay(l6)
    except FileNotFoundError:
        if (l6/'REOPEN.json').exists():raise
        retention_block='REOPEN.json absent; successful publication/reopen precondition is not satisfied.'
    if retention_block is None:raise ValueError('UnexpectedRetentionEligibility')
    current=compact(campaign);single=compact([payload,prior/'retained_L6_rechecked/QUALITY_DELTA.json'])
    cap_lower_bound=current['unique_allocation_bytes']+single['unique_allocation_bytes']
    plan=dict(passed=False,reason='Existing campaign exceeds unchanged cap before any worker.',
        output_limit_bytes=limit,campaign=current,
        campaign_logical_excess=max(0,current['logical_bytes']-limit),
        campaign_physical_excess=max(0,current['unique_allocation_bytes']-limit),
        proposed_shared_quality=single,
        one_reused_payload_lower_bound=cap_lower_bound,
        two_independent_payload_lower_bound=cap_lower_bound+single['unique_allocation_bytes'],
        two_payload_minimum_reduction=max(0,cap_lower_bound+single['unique_allocation_bytes']-limit),
        lower_bound_not_peak_upper_bound=True,
        omitted_additional_terms=['New transaction plans, acceptance, journals and recursive roots',
            'SQLite temporary/journal allocation and any existing index restart work'],
        no_original_failed_payload_replaced=True,no_successful_retention_credit=True,
        retention_block=retention_block,active_ancestry=active,
        active_ancestry_in_campaign=active_here,active_ancestry_outside_campaign=active_elsewhere,
        note='Outside active ancestry is NOT a deletion or redundancy classification. Shared input cannot escape planning accounting.')
    durable_json(OUT/'STORAGE_PLAN.json',plan)
    for p,h in protected.items():
        if file_hash(p)!=h:raise ValueError('ProtectedSourceChanged',p)
    for row in original:
        now=metadata(row['path'])
        if any(now[k]!=row[k] for k in fields):raise ValueError('OriginalCampaignChanged',row['path'])
    if unrelated()!=initial_unrelated:raise ValueError('ExactStatusChanged')
    result=dict(classification='V4RetainedL6RoundTripResourcePreflightBlocked',production_started=False,
        first_failure=dict(stage='before_worker_launch',code='RESOURCE_STOP',measurement=stop['measurement'],resources=stop['resources']),
        last_qualified_layer=5,last_qualified_scientific=base.scientific,last_qualified_root=parent['reference']['root'],
        retained_candidate=a['scientific'],retained_component_comparisons=comparisons,
        retained_reference_checks=len(checked),new_dependency_bytes_hashed=hashed_bytes,
        inherited_reference_tokens_reused=inherited_tokens,guarded_parent_stats=stats,
        ancestor_nodes=nodes,new_quality_payload_sha256=shared['payload']['sha256'],
        original_files_preserved=len(original),exact_status_preserved=True,baseline_status_count=len(initial_unrelated),
        final_disk=disk(),seconds=time.perf_counter()-start,peak_working_set=psutil.Process().memory_info().peak_wset,
        task_allocation=compact([p for p in OUT.rglob('*') if p.is_file()]),reclaimed_bytes=0,
        no_generation=True,no_proof_recomputation=True,no_publication=True,no_cleanup=True,
        hashes={p.name:file_hash(p) for p in (OUT/'BASELINE.json',OUT/'RESOURCE_STOP.json',OUT/'STORAGE_PLAN.json')})
    durable_json(OUT/'RESULT.json',result)
    print(json.dumps(dict(result=result,plan=plan)),flush=True)


def diagnose():
    """Metadata-only handoff after a failed preflight/open. Never retries either."""
    import psutil
    baseline=read(OUT/'BASELINE.json');stop=read(OUT/'RESOURCE_STOP.json')
    if (OUT/'DIAGNOSIS.json').exists():raise ValueError('PreserveExistingDiagnosis')
    if stop['stage']!='before_worker_launch':raise ValueError('UnsupportedDiagnosisBoundary')
    before=read(V4/'results/single_quality_storage/BASELINE.json');original=before['census']['records']
    fields=('device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams')
    for row in original:
        if any(metadata(row['path'])[k]!=row[k] for k in fields):raise ValueError('OriginalInputChanged')
    roots=[V4/'results'/n for n in ('wing_receipt_resume/A','current_front_transition_round_trip','wing_continuation_layers6to8')]
    files=[p for root in roots for p in root.rglob('*') if p.is_file()]
    actual=compact(files);l6=roots[2]/'A/L6'
    parent=read(roots[1]/'A/L5/REOPEN.json');reference=parent['reference'];directory=Path(reference['directory'])
    with access.guard(directory/'CURRENT.json'):
        pointer=read(directory/'CURRENT.json');root=directory/pointer['file']
        with access.guard(root):
            body=read(root)
            if identity(body)!=reference['root'] or file_hash(root)!=pointer['sha256']:
                raise ValueError('StoredL5HeaderIdentityMismatch')
            if body['scientific']!=parent['scientific']:raise ValueError('StoredL5ScientificMismatch')
            epochs=dict(stored=body['epoch'],current=access.epoch())
    a=read(l6/'CANDIDATE_PLAN.json');b=read(l6/'replay/CANDIDATE_PLAN.json');replay=read(l6/'REPLAY.json')
    comparisons={k:b['components'].get(k)==v for k,v in a['components'].items()}
    if (not all(comparisons.values()) or comparisons!=replay['comparisons'] or not replay['passed']
        or a['scientific']!=b['scientific'] or identity(a['content'])!=a['scientific']
        or identity(b['content'])!=b['scientific'] or a['parent']!=reference or b['parent']!=reference):
        raise ValueError('RetainedMetadataMismatch')
    shared_records={}
    for plan_record in (a,b):
        for path,binding in plan_record['references'].items():
            if not already_checked(shared_records,path,binding):shared_records[path]=deepcopy(binding)
    prior=V4/'results/single_quality_storage';envelope=prior/'retained_L6_rechecked/QUALITY_DELTA.json'
    record=read(envelope);payload=Path(record['payload']['path'])
    with access.guard(payload):
        if access.token(payload)!=record['payload']['token'] or file_hash(payload)!=record['payload']['sha256']:
            raise ValueError('DiagnosticQualityPayloadChanged')
    single=compact([payload,envelope]);cap=stop['resources']['output_limit_bytes']
    plan=dict(passed=False,campaign=actual,output_limit_bytes=cap,
        existing_logical_excess=actual['logical_bytes']-cap,existing_allocated_excess=actual['unique_allocation_bytes']-cap,
        reused_quality=single,
        one_payload_lower_bound=actual['unique_allocation_bytes']+single['unique_allocation_bytes'],
        two_payload_lower_bound=actual['unique_allocation_bytes']+2*single['unique_allocation_bytes'],
        two_payload_minimum_reduction=actual['unique_allocation_bytes']+2*single['unique_allocation_bytes']-cap,
        lower_bounds_not_peak_upper_bounds=True,
        omitted_additional_terms=['New output/control metadata','SQLite journals and temporary/index restart work'],
        retained_failed_inputs_not_subtracted=True,successful_retention_not_credited_before_reopen=True,
        external_quality_must_count_when_used=True,
        per_root=[dict(path=str(root),allocation=compact([p for p in root.rglob('*') if p.is_file()])) for root in roots],
        note='Per-root path totals are not independent capacities; shared allocation is separately measured. No active-ancestry classification because guarded access failed.')
    protected=baseline['protected']
    if any(file_hash(p)!=h for p,h in protected.items()):raise ValueError('ProtectedSourceChanged')
    owned=['research/sdf_cfd_mesher_v4/'+p for p in ('CURRENT_TASK.md','README.md','RETAINED_L6_STORAGE_PLAN_REPORT.md',
        'tools/check_retained_l6_storage_plan.py','tests/test_retained_l6_storage_plan.py','results/retained_l6_storage_plan/')]
    unrelated=[r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in owned)]
    if unrelated!=baseline['unrelated']:raise ValueError('BaselineStatusChanged')
    result=dict(classification='V4RetainedL6RoundTripResourceAndEpochBlocked',round_trip_completed=False,
        first_failure='RESOURCE_STOP before worker launch',diagnostic_failure='IncrementalEpochChanged',
        epoch_measurement=epochs,epoch_scope='Exact epoch values differ; real host reboot is not established.',
        last_qualified_layer=5,retained_l5_scientific=body['scientific'],retained_l5_root=reference['root'],
        static_header_hash_verified=True,fresh_guarded_open=False,
        retained_l6_scientific=a['scientific'],retained_metadata_comparisons=comparisons,
        shared_reference_records_consistent=True,unique_reference_records=len(shared_records),
        comparisons_scope='Existing retained component records, not a new transaction/replay qualification.',
        shared_payload_sha256=record['payload']['sha256'],plan=plan,
        original_files_metadata_preserved=len(original),status_entries_preserved=len(unrelated),
        source_hashes_preserved=len(protected),capsule_scope='Storage and dispatch implementations unchanged; prior retained-control evidence not rerun.',
        final_disk=disk(),diagnostic_peak_working_set=psutil.Process().memory_info().peak_wset,
        new_cells=0,new_proofs=0,production_worker_launches=0,published=False,reclaimed_bytes=0,
        hashes={p.name:file_hash(p) for p in (OUT/'BASELINE.json',OUT/'RESOURCE_STOP.json',root,envelope)},
        task_results=compact([p for p in OUT.rglob('*') if p.is_file()]))
    durable_json(OUT/'DIAGNOSIS.json',result);print(json.dumps(result),flush=True)


def run():
    try:main()
    except BaseException as error:
        # Unique failure record only; scientific failures are never retried.
        if OUT.exists() and not (OUT/'FAILED_ATTEMPT.json').exists():
            import traceback
            durable_json(OUT/'FAILED_ATTEMPT.json',dict(code=type(error).__name__,message=str(error),traceback=traceback.format_exc()))
        if (isinstance(error,ValueError) and str(error)=='IncrementalEpochChanged'
            and (OUT/'RESOURCE_STOP.json').exists() and not (OUT/'DIAGNOSIS.json').exists()):
            diagnose()  # Reads preserved headers only; never reopens or recovers.
        raise


def finish():
    """Read-only preservation and shared-record check; no guarded-open retry."""
    if (OUT/'HANDOFF.json').exists():raise ValueError('PreserveExistingHandoff')
    baseline=read(OUT/'BASELINE.json');diagnosis=read(OUT/'DIAGNOSIS.json')
    for path,sha in baseline['protected'].items():
        if file_hash(path)!=sha:raise ValueError('ProtectedSourceChanged',path)
    prior=read(V4/'results/single_quality_storage/BASELINE.json')
    fields=('device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams')
    for row in prior['census']['records']:
        current=metadata(row['path'])
        if any(current[k]!=row[k] for k in fields):raise ValueError('OriginalFileChanged',row['path'])
    owned=['research/sdf_cfd_mesher_v4/'+p for p in ('CURRENT_TASK.md','README.md','RETAINED_L6_STORAGE_PLAN_REPORT.md',
        'tools/check_retained_l6_storage_plan.py','tests/test_retained_l6_storage_plan.py','results/retained_l6_storage_plan/')]
    untouched=[r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in owned)]
    if untouched!=baseline['unrelated']:raise ValueError('StatusChanged')
    folder=V4/'results/wing_continuation_layers6to8/A/L6';shared={}
    for parent in (folder,folder/'replay'):
        for path,record in read(parent/'CANDIDATE_PLAN.json')['references'].items():
            if not already_checked(shared,path,record):shared[path]=deepcopy(record)
    records=dict(passed=True,scope='Metadata/source preservation only. Production remains resource-blocked; guarded parent access failed.',
        source_files_preserved=len(baseline['protected']),original_files_metadata_preserved=len(prior['census']['records']),
        exact_status_entries_preserved=len(untouched),shared_reference_records_agree=True,unique_reference_records=len(shared),
        seconds_since_baseline_file=time.time()-(OUT/'BASELINE.json').stat().st_mtime,
        final_disk=disk(),task_results=compact([p for p in OUT.rglob('*') if p.is_file()]),
        production_started=False,guarded_open_retried=False,reclaimed_bytes=0,
        hashes={str(p.relative_to(ROOT)):file_hash(p) for p in (
            OUT/'DIAGNOSIS.json',OUT/'RESOURCE_STOP.json',OUT/'FAILED_ATTEMPT.json',
            V4/'tools/check_retained_l6_storage_plan.py',V4/'RETAINED_L6_STORAGE_PLAN_REPORT.md')})
    durable_json(OUT/'HANDOFF.json',records);print(json.dumps(records),flush=True)


if __name__=='__main__':
    if sys.argv[1:]==['diagnose']:diagnose()
    elif sys.argv[1:]==['finish']:finish()
    elif not sys.argv[1:]:run()
    else:raise ValueError('UnsupportedDiagnosticArguments')
