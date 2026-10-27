"""Reduce the reviewed private control; never produce mesh acceptance evidence."""
from pathlib import Path
import time

import benchmark_publication_scope as b
from record_receipt_resume import status,disk
from sdf_cfd_mesher_v4.common.authority import identity

OWN=(*b.OWN,'tools/check_publication_scope_result.py')

def unrelated(rows):
    return [r for r in rows if not any(r[3:]=='research/sdf_cfd_mesher_v4/'+p or
        p.endswith('/') and r[3:].startswith('research/sdf_cfd_mesher_v4/'+p) for p in OWN)]


def bind_components(result,retained,accepted,previous):
    """Report only actual retained component meaning, never summary agreement."""
    expected=dict(quality=retained['components']['quality']['delta'],
        atlas=retained['components']['atlas']['replace'],forecast=retained['components']['forecast']['replace'])
    count=accepted['quality_count'];acount=previous['atlas_rows']
    if (accepted.get('passed') is not True or accepted['plan']!=identity(retained) or
        result['scientific_input']!=retained['scientific'] or previous['scientific']!=retained['scientific'] or
        previous['quality']['count']!=count or previous['quality']['identity']!=accepted['quality_delta'] or
        previous['quality']['sha256']!=expected['quality'] or previous['atlas_semantic']!=expected['atlas'] or
        result['expected_atlas']!=dict(count=acount,sha256=expected['atlas'])):
        raise ValueError('UnboundRetainedComponentSummary')
    for mode in ('cold','shared'):
        row=result['results'][mode]
        if (row['components']!=expected or row['quality_count']!=count or
            row['quality_delta']!=accepted['quality_delta'] or row['binding_patch']!=accepted['binding_patch']):
            raise ValueError('UnboundPrivateComponentSummary')
    return expected,count,acount


def verify(result,monitor,request,review,baseline):
    if (result.get('passed') is not True or type(monitor.get('exit_code')) is not int or
        type(monitor.get('worker_exit_code')) is not int or monitor['exit_code']!=0 or monitor['worker_exit_code']!=0 or
        monitor.get('resource_stopped') is not False or
        any(result.get(k) is not True for k in ('no_geometry_generated','no_proofs_recomputed','no_front_published'))):
        raise ValueError('FailedScopeControl')
    if (result['source_hashes']!=b.runtime() or request['runtime_sources']!=result['source_hashes'] or
        review.get('passed') is not True or review.get('request_sha256')!=b.file_hash(b.OUT/'REVIEW_REQUEST.json') or
        review['runtime_sources']!=result['source_hashes'] or request['baseline_sha256']!=b.file_hash(b.OUT/'BASELINE.json')):
        raise ValueError('StaleScopeReview')
    expected=dict(baseline['resources'])
    for k,v in expected.items():
        if monitor['resources'][k]!=v:raise ValueError('ChangedScopeResourcePolicy',k)
    if monitor['resources']['total_storage_policy']!='monitor_without_output_cap':raise ValueError('ChangedStorageMode')
    if not monitor['samples'] or any(s['free']<expected['minimum_free_bytes'] for s in monitor['samples']):
        raise ValueError('ScopeResourceFailure')
    previous=b.control.read(b.SOURCE/'RESULT.json');cold=result['results']['cold'];warm=result['results']['shared']
    retained=b.control.read(b.control.SOURCE/'CANDIDATE_PLAN.json')
    accepted=b.control.read(b.control.SOURCE/'CANDIDATE_ACCEPTANCE.json')
    expected,count,acount=bind_components(result,retained,accepted,previous)
    for key in ('components','quality_delta','quality_count','binding_patch'):
        if cold[key]!=warm[key]:raise ValueError('ScopeLogicalMismatch',key)
    if (cold['quality_count']!=count or cold['full_cold_checks']!=[count,acount,count,count] or
        warm['full_cold_checks']!=[count,acount]):raise ValueError('ScopeFullValidationCoverage')
    for mode in ('cold','shared'):
        row=result['results'][mode]
        if row['independent_readbacks']!=[count,count]:raise ValueError('MissingScopeReadback')
        if (row['stats']['quality_rows_verified']!=count or row['stats']['quality_rows_reverified']!=count or
            row['stats']['proof_payloads_written']!=0 or row['stats']['prepared_quality_payloads_reused']!=1):
            raise ValueError('ChangedScopeTransaction')
        private=b.OUT/mode/'candidate';body=b.control.read(private/('ROOT_'+row['private_root']+'.json'))
        recorded_plan=b.control.read(b.OUT/mode/'CANDIDATE_PLAN.json')
        recorded_acceptance=b.control.read(b.OUT/mode/'CANDIDATE_ACCEPTANCE.json')
        if (identity(body)!=row['private_root'] or body['scientific']!=result['scientific_input'] or
            body['plan']['scope']!='Private retained-L6 preparation/publication input control' or
            body['plan']!=recorded_plan or body['acceptance']!=recorded_acceptance or body['plan']['components']!=expected or
            body['plan']['content']['parent_scientific']!=retained['content']['parent_scientific'] or
            body['acceptance']['quality_count']!=count or
            body['acceptance'].get('passed') is not True or identity(body['acceptance'])!=body['acceptance_identity'] or
            body['acceptance']['plan']!=identity(body['plan']) or body['acceptance']['base']!=body['parent'] or
            body['acceptance']['quality_delta']!=cold['quality_delta'] or
            body['acceptance']['binding_patch']!=cold['binding_patch']):raise ValueError('WrongScopePrivateRoot')
        qbody=b.control.read(b.OUT/mode/'QUALITY_DELTA.json')
        if (body['quality_delta']!=qbody['payload']['path'] or qbody['payload']['sha256']!=previous['transaction']['sha256'] or
            qbody['logical']['identity']!=row['quality_delta'] or qbody['logical']['sha256']!=row['components']['quality']):
            raise ValueError('ChangedScopePayloadBytes')
    if unrelated(status())!=unrelated(baseline['unrelated']):raise ValueError('UnrelatedScopeStatusChanged')
    for p,h in {**baseline['protected'],**baseline['inputs']}.items():
        if b.file_hash(p)!=h:raise ValueError('ProtectedScopeContentChanged',p)


def finish():
    out=b.OUT
    if (out/'EVIDENCE.json').exists():raise ValueError('PreserveScopeCloseout')
    result=b.control.read(out/'RESULT.json');monitor=b.control.read(out/'monitor/MONITOR_FINAL.json')
    # Existing final-monitor schema stores samples/exits, not policy. Policy is
    # retained separately in its actual last sample and resource preflight.
    policy=b.control.read(out/'monitor/MONITOR.json')['resources']
    if policy!=b.control.read(out/'monitor/RESOURCE_PREFLIGHT.json')['resources']:
        raise ValueError('ChangedScopeMonitorPolicy')
    monitor=dict(monitor,resources=policy)
    baseline=b.control.read(out/'BASELINE.json');request=b.control.read(out/'REVIEW_REQUEST.json');review=b.control.read(out/'REVIEW.json')
    verify(result,monitor,request,review,baseline)
    paths=[p for p in out.rglob('*') if p.is_file()]
    sources=[b.V4/p for p in OWN if not p.endswith('/')]
    paths+= [p for p in sources if p.is_file()]
    cold=result['results']['cold'];warm=result['results']['shared']
    artifacts={str(p):b.file_hash(p) for p in out.rglob('*') if p.is_file() and p.stat().st_size<1000000}
    evidence=dict(classification='V4GuardedPublicationInputReuseRetainedL6Verified',passed=True,
        scope='Private retained preparation input-opening and transaction only; not whole layer or production publication',
        quality_records=cold['quality_count'],atlas_records=result['expected_atlas']['count'],
        cold_seconds=cold['seconds'],shared_seconds=warm['seconds'],
        measured_time_reduction_percent=100*(1-warm['seconds']/cold['seconds']),
        full_quality_validation_passes=dict(before=3,after=1),independent_readbacks_per_mode=2,
        logical_component_and_acceptance_fields_equal=True,payload_bytes_equal=True,
        storage=dict(cold=cold['storage'],shared=warm['storage'],new_sqlite_payloads=0,
            forecast_copy_bytes=(out/'cold/NEW_FORECAST_DEPENDENCIES.json').stat().st_size,
            binding_copy_bytes=(out/'cold/BINDING_PATCH.json').stat().st_size,
            reclaimed_bytes=0,cleanup='None; no hardlinks, scientific deletion or sealed-state edits'),
        monitor=dict(seconds=monitor['seconds'],peaks=monitor['peaks'],worker_exit=monitor['worker_exit_code'],
            exit=monitor['exit_code'],minimum_free_observed=min(s['free'] for s in monitor['samples'])),
        worker=dict(seconds=result['seconds'],peak_working_set=result['peak_working_set']),
        assumptions='Cold first then shared; cache effects not isolated; main-process memory peak cumulative; no linear scaling proof',
        preservation=dict(unrelated_status_entries=len(unrelated(baseline['unrelated'])),protected_hashes=len(baseline['protected']),
            input_hashes=len(baseline['inputs']),source_pins=len(result['source_hashes'])),
        starting_head=baseline['head'],disk=disk(),retained_allocation=b.control.totals(paths),
        elapsed_seconds=time.time()-baseline['start'],artifacts=artifacts,
        analysis_source_sha256=b.file_hash(__file__),no_geometry_generated=True,no_proofs_recomputed=True,no_front_published=True)
    b.durable_json(out/'EVIDENCE.json',evidence)
    print(dict(classification=evidence['classification'],reduction=evidence['measured_time_reduction_percent'],
        allocation=evidence['retained_allocation'],disk=evidence['disk'],evidence_sha256=b.file_hash(out/'EVIDENCE.json')))

if __name__=='__main__':finish()
