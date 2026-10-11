"""Collect compact evidence for the bounded existing FastLayer benchmark."""
import json
import subprocess
import time
from pathlib import Path
from check_live_fast_scope import OUT, ROOT, V4, read
from check_current_front_encoding import allocation
from path_preserving_deduplication import durable_json, file_hash
from record_receipt_resume import disk, status


def require_review_binding(out, request):
    request_hash = file_hash(out/'REVIEW_REQUEST.json')
    for name in ('REVIEW.json', 'OUTCOME_REVIEW.json'):
        review = read(out/name)
        if review.get('passed') is not True or review.get('request_sha256') != request_hash:
            raise ValueError('ChangedExecutedReviewBinding', name)
    if file_hash(out/'BASELINE.json') != request['baseline']:
        raise ValueError('ChangedReviewedBaseline')
    return request_hash


def collect():
    baseline = read(OUT/'BASELINE.json')
    request = read(OUT/'REVIEW_REQUEST.json')
    require_review_binding(OUT, request)
    result = read(OUT/'run/RESULT.json')
    qualification = read(OUT/'run/QUALIFICATION.json')
    production = read(OUT/'run/PRODUCTION.json')
    monitor = read(OUT/'monitor/MONITOR_FINAL.json')
    if not result['passed'] or not qualification['passed'] or monitor['exit_code'] != 0 or monitor['resource_stopped']:
        raise ValueError('IncompleteFastBenchmark')
    own = ['research/sdf_cfd_mesher_v4/'+p for p in (
        'CURRENT_TASK.md', 'README.md', 'LIVE_WING_PRODUCTION_FAST_REPORT.md',
        'tools/production_fast_layer.py', 'tools/accepted_fast_binding.py',
        'tools/check_live_fast_scope.py', 'tools/collect_live_fast_scope.py',
        'tests/test_accepted_fast_binding.py', 'tests/test_collect_live_fast_scope.py')]
    unrelated = [r for r in status() if r[3:] not in own and not r[3:].startswith('research/sdf_cfd_mesher_v4/results/live_wing_fast_scope/')]
    if unrelated != baseline['unrelated']:
        raise ValueError('ChangedUnrelatedStatus')
    checks = {}
    for label, pins in [('protected', baseline['protected']), ('baseline_inputs', baseline['inputs'])] + list(request.items()):
        if label not in ('protected', 'baseline_inputs', 'runtime', 'binary', 'inputs'):
            continue
        failures = [p for p, h in pins.items() if file_hash(p) != h]
        if failures:
            raise ValueError('ChangedPinnedFastDependency', label, failures)
        checks[label] = dict(files=len(pins), all_bytes_equal=True)
    summary = dict(classification='V4AcceptedFrontProductionFastSingleTransitionQualifiedEquivalent',
        result=result, bootstrap=read(OUT/'run/BOOTSTRAP.json'), binding_seconds=read(OUT/'run/BINDING.json')['seconds'],
        production={k:production[k] for k in ('seconds','timings','counts','cells','relations','new_relations','degree_passed','exact_volume_balance','profile')},
        degree_counts=production['degree_metrics']['counts'], qualification=qualification,
        monitor_exit=monitor['exit_code'], resource_stopped=monitor['resource_stopped'],
        limits=dict(determinant_min=.0011,weight_min=.055,skewness_max=3.63636,symmetric_volume_ratio_min=.01,aspect_max=1000),
        last_qualified_committed_layer=8, publication=False, warm_multiple_transitions_supported=False)
    durable_json(OUT/'SUMMARY.json', summary)
    paths = [p for p in OUT.rglob('*') if p.is_file() and p.name != 'EVIDENCE.json']
    storage = allocation(paths); storage.pop('records')
    artifacts = {str(p.relative_to(ROOT)):dict(sha256=file_hash(p),bytes=p.stat().st_size) for p in paths}
    sources = {str((V4/'tools'/n).relative_to(ROOT)):file_hash(V4/'tools'/n) for n in (
        'accepted_fast_binding.py','production_fast_layer.py','check_live_fast_scope.py','collect_live_fast_scope.py')}
    evidence = dict(starting_head=baseline['head'], head_at_collection=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        elapsed_seconds=time.time()-baseline['start'], checks=checks, unrelated_status_count=len(unrelated),
        status_exact=True, sources=sources, artifacts=artifacts, allocation_before_this_manifest=storage,
        disk_initial=baseline['disk'], disk_final=disk(),
        retained_full_records=['run/PRODUCTION.json','monitor/MONITOR_FINAL.json','run/GENERATED_NEXT.npz'],
        cleanup=False, scope='Task output only; no inventory or modification of scientific ancestry. No new published state.')
    durable_json(OUT/'EVIDENCE.json', evidence)
    print(json.dumps({k:evidence[k] for k in ('elapsed_seconds','checks','unrelated_status_count','status_exact','allocation_before_this_manifest','disk_final')}))


if __name__ == '__main__':
    from contextlib import ExitStack
    from immutable_state_access import guard
    with ExitStack() as held:
        for name in ('BASELINE.json','REVIEW_REQUEST.json','REVIEW.json','OUTCOME_REVIEW.json',
                     'run/RESULT.json','run/QUALIFICATION.json','run/PRODUCTION.json',
                     'monitor/MONITOR_FINAL.json'):
            held.enter_context(guard(OUT/name))
        collect()
