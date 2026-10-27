"""Reduce the existing full-record control; preserve all old source locks.

Generated records are benchmark evidence only, never mesh acceptance records.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import time

import benchmark_compact_reuse as repeat
from check_current_front_encoding import allocation
import immutable_state_access as access

b=repeat.control
OWN=('CURRENT_TASK.md','README.md','COMPACT_REUSE_L6_REPORT.md',
     'tools/single_quality_component.py','tools/benchmark_retained_l6.py',
     'tools/benchmark_compact_reuse.py','tools/check_compact_reuse_result.py',
     'tests/test_compact_work_amplification.py','tests/test_compact_reuse.py',
     'tools/finish_retained_l5.py','tests/test_continuation_resource_monitor.py',
     'tests/test_l6_efficiency_benchmark.py','tests/test_compact_reuse_result.py','results/compact_reuse_l6/')


def sources():
    baseline=b.read(repeat.OUT/'BASELINE.json')
    return baseline,{p:b.file_hash(p) for p in sorted(b.runtime_paths(baseline))}


def audit_sources():
    baseline,current=sources()
    current[str(Path(__file__).resolve())]=b.file_hash(__file__)
    return baseline,current


def validate_audit_pin(record,current,original_review,original_result):
    """Post-run monitor fix has its own pin; never refresh the worker's pin."""
    if (record.get('passed') is not True or not record.get('reviewer') or record.get('runtime_sources')!=current or
        record.get('original_review_sha256')!=original_review or record.get('original_result_sha256')!=original_result):
        raise ValueError('UnreviewedRetainedAudit')


def validate_closeout(closeout,current,result,pins):
    if (closeout.get('passed') is not True or closeout.get('source_hashes')!=current or
        closeout.get('original_pins')!=pins or
        closeout.get('scope')!='Read-only retained-data closeout; not producer rerun, scientific publication or recovery.' or
        any(closeout.get(k) is not True for k in ('no_geometry_generated','no_proofs_recomputed','no_front_published')) or
        closeout['quality']['count']!=result['quality']['count'] or closeout['atlas']['count']!=result['atlas_rows'] or
        closeout['quality']['independent_readbacks']!=2 or closeout['quality']['sha256']!=result['transaction']['sha256'] or
        closeout['atlas']['semantic']!=result['atlas_semantic'] or
        closeout['original_monitoring_complete'] is not False or closeout['original_worker_exit_code'] is not None):
        raise ValueError('UnboundRetainedCloseout')


def validate_root_bindings(private_acceptance,retained_acceptance,bindings):
    from sdf_cfd_mesher_v4.common.authority import identity
    if (private_acceptance.get('passed') is not True or
        private_acceptance.get('binding_patch')!=retained_acceptance['binding_patch'] or
        identity(bindings)!=retained_acceptance['binding_patch']):
        raise ValueError('WrongRetainedBindingPatch')


def review():
    """Convert an independently supplied verdict into the existing source pin."""
    baseline,current=sources();verdict=b.read(repeat.OUT/'PRE_RUN_REVIEW.json')
    if verdict.get('passed') is not True or verdict.get('runtime_sources')!=current or not verdict.get('reviewer'):
        raise ValueError('MissingIndependentSourceReview')
    record=dict(passed=True,reviewer=verdict['reviewer'],
        baseline_sha256=b.file_hash(repeat.OUT/'BASELINE.json'),runtime_sources=current,
        intentional_changes={p:dict(before=h,after=current[p]) for p,h in baseline['protected'].items() if current[p]!=h},
        options=dict(codec_workers=4,codec_process=True,single_quality=True,indexed_atlas=True,verified_quality=True),
        atlas_comparison='existing_ordered_rows',review_evidence_sha256=b.file_hash(repeat.OUT/'PRE_RUN_REVIEW.json'))
    b.validate_review(baseline,record,current,record['baseline_sha256'])
    p=repeat.OUT/'REVIEW.json'
    if p.exists():raise ValueError('PreserveReviewRecord')
    b.durable_json(p,record);print(dict(runtime_files=len(current),review_sha256=b.file_hash(p)))


def validate_control(baseline,result,monitoring,preflight,stops=()):
    """Reject contradictory reduction inputs; this is not mesh qualification."""
    if (result.get('passed') is not True or type(monitoring.get('exit_code')) is not int or
        monitoring['exit_code']!=0 or stops):raise ValueError('FailedRetainedControl')
    expected=dict(single_quality=True,indexed_atlas=True,verified_quality=True,
        codec_workers=4,codec_process=True,atlas_comparison='existing_ordered_rows')
    from incremental_transaction import canonical
    if (canonical(result.get('options'))!=canonical(expected) or result.get('mode')!='optimized' or
        result.get('scope')!='Full retained L6 assembly-storage boundaries; not whole-transition timing or publication.' or
        any(result.get(k) is not True for k in ('no_geometry_generated','no_proofs_recomputed','no_front_published'))):
        raise ValueError('ContradictoryControlScope')
    resources=dict(baseline['resources'],inherited_output_limit_bytes=baseline['resources']['output_limit_bytes'],
        output_limit_bytes=None,total_storage_policy='monitor_without_output_cap')
    if canonical(preflight.get('resources'))!=canonical(resources):raise ValueError('ChangedControlResources')
    samples=monitoring.get('samples')
    if not isinstance(samples,list) or not samples:raise ValueError('MissingControlMonitoring')
    minimum=resources['minimum_free_bytes']
    for sample in [preflight['measurement'],*samples]:
        if type(sample.get('free')) is not int or sample['free']<minimum:raise ValueError('ControlResourceFailure')
    if monitoring['disk']['C:\\']['available']<minimum:raise ValueError('ControlResourceFailure')
    return resources


def audit():
    """New read-only closeout, not a replay of the completed producer."""
    import psutil
    from incremental_transaction import canonical
    from critical_transaction import Store
    import single_quality_component as quality
    from compact_front_atlas import snapshot as atlas_snapshot
    from check_single_quality_storage import RetainedRows
    from check_current_front_encoding import retained_rows
    run=repeat.OUT/'retained_audit';run.mkdir(exist_ok=False)
    started=time.perf_counter();phases={}
    def timed(name,call):
        tick=time.perf_counter();value=call();phases[name]=time.perf_counter()-tick
        b.durable_json(run/'PHASES.json',phases);return value
    try:
        with ExitStack() as held:
            baseline,current=audit_sources()
            for p,h in current.items():
                held.enter_context(access.guard(p))
                if b.file_hash(p)!=h:raise ValueError('ChangedAuditSource',p)
            pins={}
            for name in ('optimized/RESULT.json','REVIEW.json','MONITOR_EXIT_RACE.json','monitor/MONITOR.json',
                'monitor/RESOURCE_PREFLIGHT.json','monitor/WORKER.log','AUDIT_REVIEW.json'):
                p=repeat.OUT/name;held.enter_context(access.guard(p));pins[name]=b.file_hash(p)
            verdict=b.read(repeat.OUT/'AUDIT_REVIEW.json')
            validate_audit_pin(verdict,current,pins['REVIEW.json'],pins['optimized/RESULT.json'])
            original=b.read(repeat.OUT/'optimized/RESULT.json');original_review=b.read(repeat.OUT/'REVIEW.json')
            if original['source_hashes']!=original_review['runtime_sources']:raise ValueError('OriginalWorkerPinMismatch')
            for p,h in original['source_hashes'].items():
                if current[p]!=h and Path(p).name!='finish_retained_l5.py':raise ValueError('UnintendedPostRunSourceChange',p)
            plan,accepted,authentication=timed('authenticate',lambda:b.guarded_inputs(held))
            if authentication!=original['authentication'] or plan['scientific']!=original['scientific']:
                raise ValueError('WrongRetainedAuditParent')
            qpath=repeat.OUT/'optimized/QUALITY_DELTA.json'
            def verify_quality():
                with quality.snapshot(qpath,retain_verified=True) as snap:
                    if snap['rows'].body['logical']!=original['quality']:raise ValueError('AuditQualityMeaningMismatch')
                    count=0
                    for a,c in zip(RetainedRows(b.SOURCE/'QUALITY_DELTA.json'),snap['rows'],strict=True):
                        if canonical(a)!=canonical(c):raise ValueError('AuditQualityRowMismatch',count)
                        count+=1
                    snap['rows'].verify_readback();snap['rows'].verify_readback()
                    store=Store(repeat.OUT/'optimized/private_transaction')
                    root=store.current()['root']
                    with store.open(root) as body:
                        bindings=access.read(body['binding_patch'])
                        validate_root_bindings(body['acceptance'],accepted,bindings)
                        if (body['scientific']!=original['scientific'] or body['plan']['scope']!='Retained-L6-storage-control' or
                            body['plan']['references']!=snap['references'] or body['quality_delta']!=str(snap['rows'].path) or
                            body['acceptance']['quality_count']!=count or
                            body['acceptance']['quality_delta']!=quality.quality_identity(snap['rows']) or
                            body['plan']['retained_plan_sha256']!=baseline['inputs'][str(b.SOURCE/'CANDIDATE_PLAN.json')]):
                            raise ValueError('WrongPrivateCommittedAuditRoot')
                        if b.file_hash(snap['rows'].path)!=original['transaction']['sha256']:
                            raise ValueError('ChangedAuditQualityDatabase')
                        return dict(count=count,root=root,sha256=original['transaction']['sha256'],independent_readbacks=2,
                            retained_binding_patch=accepted['binding_patch'])
            quality_result=timed('all_quality_rows_and_readbacks',verify_quality)
            def verify_atlas():
                with atlas_snapshot(repeat.OUT/'optimized/NEW_ATLAS.json') as snap:
                    if snap['semantic']!=original['atlas_semantic']:raise ValueError('AuditAtlasMeaningMismatch')
                    count=0
                    for (key,value),row in zip(retained_rows(b.SOURCE/'NEW_ATLAS.json'),snap['view'].rows,strict=True):
                        if row['key']!=key or canonical(row['payload'])!=canonical(value):raise ValueError('AuditAtlasRowMismatch',key)
                        count+=1
                    if count!=original['atlas_rows']:raise ValueError('AuditAtlasCoverageMismatch')
                    return dict(count=count,semantic=snap['semantic'])
            atlas_result=timed('all_atlas_rows',verify_atlas)
            result=dict(passed=True,scope='Read-only retained-data closeout; not producer rerun, scientific publication or recovery.',
                original_pins=pins,source_hashes=current,quality=quality_result,atlas=atlas_result,
                original_monitoring_complete=False,original_worker_exit_code=None,
                no_geometry_generated=True,no_proofs_recomputed=True,no_front_published=True,
                phases=phases,seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset)
            b.durable_json(run/'RESULT.json',result);print(dict(passed=True,seconds=result['seconds'],quality=quality_result,atlas=atlas_result))
    except BaseException as e:
        import traceback
        b.durable_json(run/'FAILURE.json',dict(error=repr(e),traceback=traceback.format_exc(),phases=phases))
        raise


def finish():
    baseline,current=sources();result=b.read(repeat.OUT/'optimized/RESULT.json')
    # Preserve the original missing closeout. A separate audit cannot recover
    # its lost exit/sample history, but can independently verify retained data.
    monitoring=b.read(repeat.OUT/'audit_monitor/MONITOR_FINAL.json')
    preflight=b.read(repeat.OUT/'audit_monitor/RESOURCE_PREFLIGHT.json')
    closeout=b.read(repeat.OUT/'retained_audit/RESULT.json')
    verdict=b.read(repeat.OUT/'AUDIT_REVIEW.json')
    _,audit_current=audit_sources()
    validate_audit_pin(verdict,audit_current,b.file_hash(repeat.OUT/'REVIEW.json'),b.file_hash(repeat.OUT/'optimized/RESULT.json'))
    pins={n:b.file_hash(repeat.OUT/n) for n in ('optimized/RESULT.json','REVIEW.json','MONITOR_EXIT_RACE.json',
        'monitor/MONITOR.json','monitor/RESOURCE_PREFLIGHT.json','monitor/WORKER.log','AUDIT_REVIEW.json')}
    validate_closeout(closeout,audit_current,result,pins)
    stops=[str(p) for p in (repeat.OUT/'monitor/RESOURCE_STOP.json',repeat.OUT/'audit_monitor/RESOURCE_STOP.json',
        repeat.OUT/'retained_audit/FAILURE.json',repeat.OUT/'optimized/FAILURE.json',
        repeat.OUT/'optimized/STOP.json') if p.exists()]
    resources=validate_control(baseline,result,monitoring,preflight,stops)
    pinned=b.read(repeat.OUT/'REVIEW.json')
    b.validate_review(baseline,pinned,result['source_hashes'],b.file_hash(repeat.OUT/'BASELINE.json'))
    if pinned.get('atlas_comparison')!='existing_ordered_rows':raise ValueError('UnreviewedAtlasComparison')
    for p,h in result['source_hashes'].items():
        if current[p]!=h and Path(p).name!='finish_retained_l5.py':raise ValueError('ChangedRuntimeAfterControl',p)
    def unrelated(rows):
        return [r for r in rows if not any(r[3:]=='research/sdf_cfd_mesher_v4/'+p or
            p.endswith('/') and r[3:].startswith('research/sdf_cfd_mesher_v4/'+p) for p in OWN)]
    if unrelated(b.status())!=unrelated(baseline['unrelated']):raise ValueError('UnrelatedStatusChanged')
    for p,h in baseline['preservation'].items():
        if b.file_hash(p)!=h:raise ValueError('ProtectedContentChanged',p)
    if b.file_hash(repeat.OLD/'BASELINE.json')!=baseline['old_baseline_sha256']:raise ValueError('OldSourceLocksChanged')
    for p,h in {**baseline['inputs'],**baseline['original_control_results']}.items():
        if b.file_hash(p)!=h:raise ValueError('RetainedInputChanged',p)
    old={n:b.read(repeat.OLD/n/'RESULT.json') for n in ('baseline','optimized')}
    comparisons={}
    for n,r in old.items():
        comparisons[n]={k:result[k]==r[k] for k in ('scientific','quality','atlas_rows','atlas_semantic','authentication')}
    comparisons['quality_database_bytes']=result['transaction']==old['optimized']['transaction']
    # Independently hash the two actual atlas payloads. No cold-source-lock
    # refresh of the old envelope is needed or allowed for this byte comparison.
    with ExitStack() as held:
        atlas_payloads={}
        for label,root in (('old',repeat.OLD/'optimized'),('new',repeat.OUT/'optimized')):
            p=root/'atlas_rows.json';held.enter_context(access.guard(p));body=b.read(p)
            source=Path(body['payload']['path']);held.enter_context(access.guard(source))
            if access.token(source)!=body['payload']['token'] or b.file_hash(source)!=body['payload']['sha256']:
                raise ValueError('ChangedAtlasComparisonPayload',label)
            atlas_payloads[label]=dict(sha256=body['payload']['sha256'],bytes=source.stat().st_size)
    comparisons['atlas_database_bytes']=atlas_payloads['old']==atlas_payloads['new']
    if not all(all(v.values()) if isinstance(v,dict) else v for v in comparisons.values()):raise ValueError('RetainedMeaningMismatch')
    stats=result['transaction_stats']
    # Store uses Counter: an absent zero counter means zero, not a missing row.
    if (stats.get('quality_rows_written',0)!=0 or stats['quality_rows_verified']!=690963 or
        stats['quality_rows_reverified']!=690963 or stats['proof_payloads_written']!=0 or
        stats['historical_quality_rows_visited']!=0):raise ValueError('ChangedTransactionReuse')
    measured=allocation([p for p in repeat.OUT.rglob('*') if p.is_file()])
    evidence=dict(passed=True,classification='V4RetainedL6CompactVerificationWorkReduced',scope=result['scope'],
        comparisons=comparisons,atlas_payloads=atlas_payloads,result=result,
        old_controls={n:{k:r[k] for k in ('seconds','phases','storage','peak_working_set')} for n,r in old.items()},
        worker_speed_change_vs_previous_compact=1-result['seconds']/old['optimized']['seconds'],
        monitoring=dict(resources=resources,**{k:monitoring[k] for k in ('seconds','exit_code','peaks','disk')}),
        original_monitoring=dict(complete=False,worker_exit_code=None,monitor_exit_code=1,
            observed=b.read(repeat.OUT/'monitor/MONITOR.json'),failure=b.read(repeat.OUT/'MONITOR_EXIT_RACE.json'),
            limitation='No recorded breach; original historical minimum/exit/full sample list unestablished.'),
        retained_closeout=closeout,
        call_probe=b.read(repeat.OUT/'call_probe/RESULT.json'),
        preservation=dict(unrelated_entries=len(unrelated(baseline['unrelated'])),protected_hashes=baseline['preservation'],
            retained_input_hashes=baseline['inputs'],old_control_results=baseline['original_control_results'],
            source_lock_changes='Explicit new review pin. Original baseline, both result manifests and retained plan/acceptance hashes checked unchanged; no old output path written.',
            last_qualified='905511e4d6a2edcf0ca00dadd1f4ee8522bdb39634edfc1e83928eddf7074019'),
        retained_allocation={k:v for k,v in measured.items() if k!='records'},
        artifact_hashes={str(p):b.file_hash(p) for p in (repeat.OUT/'BASELINE.json',repeat.OUT/'REVIEW.json',
            repeat.OUT/'optimized/RESULT.json',repeat.OUT/'call_probe/RESULT.json')},
        elapsed_since_baseline=time.time()-baseline['start'],disk=b.disk(),cleanup='None; no hardlinks or scientific deletion.',
        no_layer_advanced=True,no_new_scientific_acceptance=True)
    p=repeat.OUT/'EVIDENCE.json'
    if p.exists():raise ValueError('PreserveRepeatResult')
    b.durable_json(p,evidence)
    print(dict(passed=True,seconds=result['seconds'],phases=result['phases'],comparisons=comparisons,
        retained_allocation=evidence['retained_allocation'],disk=evidence['disk'],evidence_sha256=b.file_hash(p)))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('command',choices=['sources','audit-sources','review','audit','audit-monitor','finish']);a=p.parse_args()
    if a.command=='sources':print(__import__('json').dumps(sources()[1],indent=2))
    elif a.command=='audit-sources':print(__import__('json').dumps(audit_sources()[1],indent=2))
    elif a.command=='review':review()
    elif a.command=='audit':audit()
    elif a.command=='audit-monitor':
        import finish_retained_l5 as monitor
        import sys
        monitor.OUT=repeat.OUT/'audit_monitor';monitor.OUT.mkdir(exist_ok=False)
        sys.exit(monitor.monitor(worker_command=[sys.executable,'-B',__file__,'audit'],
            extra_storage_roots=[b.V4/'results/current_front_transition_round_trip',
                b.V4/'results/wing_continuation_layers6to8',repeat.OUT/'optimized',repeat.OUT/'retained_audit'],
            enforce_output_cap=False))
    else:finish()
