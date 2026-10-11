"""Compact outcome/preservation record. Never qualify or modify a mesh."""
import sys
from pathlib import Path
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from benchmark_retained_l6 import read,totals
from path_preserving_deduplication import file_hash,durable_json
from record_receipt_resume import disk,status
import profile_preparation_snapshots as profile

OUT=V4/'results/retained_l6_workflow'


def collect():
    baseline=read(profile.OUT/'BASELINE.json')
    owned=profile.OWN+['research/sdf_cfd_mesher_v4/tools/collect_retained_workflow.py']
    unrelated=[r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in owned)]
    if unrelated!=baseline['unrelated']:raise ValueError('ChangedUnrelatedWorkflowStatus')
    for kind in ('protected','inputs'):
        for p,h in baseline[kind].items():
            if file_hash(p)!=h:raise ValueError('ChangedProtectedWorkflowInput',p)
    result=read(OUT/'actual_prepare/RESULT.json');monitor=read(OUT/'audit_monitor/MONITOR_FINAL.json')
    audit=read(OUT/'retained_audit/RESULT.json')
    if not result['passed'] or not audit['passed'] or monitor['exit_code']!=0 or monitor['resource_stopped']:
        raise ValueError('IncompleteRetainedWorkflow')
    if result['no_front_published'] or result['publication']['layer']!=6:
        raise ValueError('UnpublishedRetainedWorkflow')
    files={str(p.relative_to(OUT)).replace('\\','/'):dict(bytes=p.stat().st_size,sha256=file_hash(p))
           for p in OUT.rglob('*') if p.is_file() and p.name!='EVIDENCE.json'}
    original=read(V4/'results/wing_continuation_layers6to8/SUMMARY.json')
    summary=next(r for r in original['layers'] if r['layer']==6)
    evidence=dict(classification='V4RetainedLiveSF2L6OptimizedReplayPublicationAndReopenVerified',
        result=result,independent_cold_audit=audit,
        original_monitor=read(OUT/'ORIGINAL_MONITOR_FAILURE.json'),
        audit_monitor={k:monitor[k] for k in ('seconds','exit_code','resource_stopped','peaks','disk')},
        retained_qualified_gates=summary['gates'],continuous=summary['continuous'],degree=summary['volume_degree'],
        gate_source_sha256=file_hash(V4/'results/wing_continuation_layers6to8/SUMMARY.json'),
        originals_unchanged=True,preservation=dict(exact_status_entries=len(unrelated),
            protected_files=len(baseline['protected']),input_pins=len(baseline['inputs'])),
        artifacts=files,retained_allocation=totals([p for p in OUT.rglob('*') if p.is_file()]),
        final_disk=disk(),cleanup='None. Unique output, original failures and active ancestry retained.',
        scope='Retained L6 completion, not fresh generation performance or arbitrary geometry qualification.')
    if (OUT/'EVIDENCE.json').exists():raise ValueError('PreserveRetainedWorkflowEvidence')
    durable_json(OUT/'EVIDENCE.json',evidence)
    print(evidence['classification'],result['scientific'],evidence['retained_allocation'],flush=True)


if __name__=='__main__':collect()
