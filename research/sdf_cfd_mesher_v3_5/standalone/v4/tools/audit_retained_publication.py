"""Independent cold readback of completed L6 after telemetry interruption.

No lost monitor history is reconstructed. No generation or publication retry.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import sys
import time
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from benchmark_retained_l6 import read
from path_preserving_deduplication import file_hash,durable_json
OUT=V4/'results/retained_l6_workflow'
RUN=OUT/'actual_prepare'
AUDIT=OUT/'retained_audit'


def request():
    original=read(OUT/'ACTUAL_REVIEW_REQUEST.json');current={p:file_hash(p) for p in original['runtime']}
    changes={p:dict(before=h,after=current[p]) for p,h in original['runtime'].items() if h!=current[p]}
    if set(Path(p).name for p in changes)!={'finish_retained_l5.py'}:raise ValueError('UnexpectedPostRunSourceChange',changes)
    current[str(Path(__file__).resolve())]=file_hash(__file__)
    names=['RESULT.json','PUBLICATION.json','REOPEN.json','REFERENCE.json','CANDIDATE_PLAN.json','CANDIDATE_ACCEPTANCE.json']
    inputs={str(RUN/n):file_hash(RUN/n) for n in names}
    inputs.update({str(OUT/n):file_hash(OUT/n) for n in ('ACTUAL_REVIEW_REQUEST.json','ACTUAL_REVIEW.json',
        'actual_monitor/MONITOR.json','actual_monitor/MONITOR.json.pending')})
    path=OUT/'AUDIT_REQUEST.json'
    if path.exists():raise ValueError('PreserveColdAuditRequest')
    durable_json(path,dict(runtime=current,inputs=inputs,intentional_monitor_change=changes,
        scope='Cold completed-L6 readback with repaired monitor. No original worker-exit/resource history recovery, science retry or L7.',
        original_monitor_complete=False,original_worker_exit_code=None,original_monitor_exit_code=1))


def worker():
    import psutil
    import immutable_state_access as access
    import single_quality_component as quality
    from composable_state import CommittedState
    from incremental_pair_storage import PairIncidence
    from sealed_front_connection import qualified_dispatch
    started=time.perf_counter()
    with ExitStack() as held:
        for p in (OUT/'AUDIT_REQUEST.json',OUT/'AUDIT_REVIEW.json'):held.enter_context(access.guard(p))
        q=read(OUT/'AUDIT_REQUEST.json');review=read(OUT/'AUDIT_REVIEW.json')
        if (review.get('passed') is not True or not review.get('reviewer')
            or review['request_sha256']!=file_hash(OUT/'AUDIT_REQUEST.json') or review['runtime']!=q['runtime']):
            raise ValueError('UnreviewedRetainedColdAudit')
        for p,h in {**q['runtime'],**q['inputs']}.items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedRetainedAuditInput',p)
        result=read(RUN/'RESULT.json');opened=read(RUN/'REOPEN.json');acceptance=read(RUN/'CANDIDATE_ACCEPTANCE.json')
        if not result['passed'] or not opened['passed']:raise ValueError('IncompleteOriginalScientificResult')
        with qualified_dispatch(),CommittedState(opened['reference']) as child:
            child.check()
            if child.layer!=6 or child.scientific!=result['scientific'] or child.scientific!=opened['scientific']:
                raise ValueError('WrongAuditedChild')
            with quality.snapshot(RUN/'QUALITY_DELTA.json',retain_verified=True) as snap:
                if quality.quality_identity(snap['rows'])!=acceptance['quality_delta'] or len(snap['rows'])!=acceptance['quality_count']:
                    raise ValueError('ChangedAuditedQualityMeaning')
                snap['rows'].verify_readback();snap['rows'].verify_readback()
            with PairIncidence(child,child.plan['pair_incidence_manifest'],child.plan['pair_incidence_root']) as pairs:
                if pairs.body['parent_count']!=len(child['certificate']):raise ValueError('ChangedAuditedPairCoverage')
                count=pairs.body['parent_count']
            child.check()
            durable_json(AUDIT/'RESULT.json',dict(passed=True,scientific=child.scientific,layer=6,
                reference=opened['reference'],quality_rows=acceptance['quality_count'],independent_readbacks=2,
                proof_count=count,no_publication_retry=True,no_new_layer=True,
                original_monitor_complete=False,original_worker_exit_code=None,
                seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset,
                request_sha256=file_hash(OUT/'AUDIT_REQUEST.json')))


def monitor():
    import finish_retained_l5 as m
    AUDIT.mkdir(exist_ok=False);m.OUT=OUT/'audit_monitor';m.OUT.mkdir(exist_ok=False)
    return m.monitor(worker_command=[sys.executable,'-B',__file__,'worker'],
        extra_storage_roots=[RUN,V4/'results/wing_continuation_layers6to8',V4/'results/current_front_transition_round_trip',AUDIT],
        enforce_output_cap=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['request','worker','monitor']);a=p.parse_args()
    if a.mode=='monitor':sys.exit(monitor())
    else:globals()[a.mode]()
