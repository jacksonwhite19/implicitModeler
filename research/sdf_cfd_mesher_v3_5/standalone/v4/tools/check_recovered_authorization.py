"""Bounded access-proof checks. No new geometry, proofs or front publication."""
import argparse
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys
import time
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from benchmark_retained_l6 import read,totals,SOURCE
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk,status
from sdf_cfd_mesher_v4.common.authority import identity
OUT=V4/'results/recovered_authorization_prepare'
OWN=['research/sdf_cfd_mesher_v4/'+p for p in ('CURRENT_TASK.md','README.md',
    'RECOVERED_AUTHORIZATION_PREPARE_REPORT.md','tools/recovered_parent_authorization.py',
    'tools/check_recovered_authorization.py','tools/benchmark_actual_prepare.py',
    'tests/test_recovered_parent_authorization.py','results/recovered_authorization_prepare/')]


def unrelated():return [r for r in status() if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in OWN)]


def baseline():
    if OUT.exists():raise ValueError('PreserveRecoveryAuthorizationEvidence')
    old=read(V4/'results/prepare_reference_cost/BASELINE.json')
    protected=set(old['protected'])|{str(V4/'tools'/p) for p in ('accepted_production_contract.py',
        'recursive_epoch_recovery.py','critical_prepare.py','critical_transaction.py','prism_receipt_witness.py','critical_index_pipeline.py')}
    inputs=set(old['inputs'])|{str(V4/'results/wing_continuation_layers6to8/AUTHORIZATION.json'),
        str(V4/'results/component_epoch_recovery/access/CURRENT.json')}
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),unrelated=unrelated(),
        protected={p:file_hash(p) for p in sorted(protected)},inputs={p:file_hash(p) for p in sorted(inputs)},disk=disk(),
        authorization='User: Accept it as needed. It is ok for now. Scoped use of authenticated recovered access for original approved sealed L5; no digest-only equality or new layer.',
        scope='Existing strict access-recovery proof; actual retained preparation only after independent review. No generation/assembly/acceptance/front publication.'))


def access_check():
    import psutil
    import accepted_production_contract as production
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure
    from recovered_parent_authorization import recovered_authorization
    from benchmark_actual_prepare import runtime
    started=time.perf_counter();raw=production.authorization_ancestry
    import immutable_state_access as access
    with ExitStack() as held:
        for p in (OUT/'ACTUAL_REVIEW_REQUEST.json',OUT/'ACTUAL_REVIEW.json',OUT/'BASELINE.json'):
            held.enter_context(access.guard(p))
        request=read(OUT/'ACTUAL_REVIEW_REQUEST.json');review=read(OUT/'ACTUAL_REVIEW.json')
        if (review.get('passed') is not True or not review.get('reviewer')
            or review['request_sha256']!=file_hash(OUT/'ACTUAL_REVIEW_REQUEST.json')
            or review['runtime']!=request['runtime'] or request['runtime']!=runtime()
            or request['baseline']!=file_hash(OUT/'BASELINE.json')):raise ValueError('UnreviewedRecoveryAccessCheck')
        baseline=read(OUT/'BASELINE.json')
        for p,h in baseline['inputs'].items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedRecoveryAccessBaselineInput',p)
        for p,h in {**request['runtime'],**request['inputs'],**request['binary']}.items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedReviewedAccessInput',p)
        auth=read(V4/'results/wing_continuation_layers6to8/AUTHORIZATION.json');before=identity(auth)
        pin=read(V4/'results/component_epoch_recovery/RECOVERY.json')['record']
        if identity(pin)!=request['recovery_identity']:raise ValueError('WrongRetainedRecoveryEvidence')
        record=discover(V4/'results/component_epoch_recovery/access',request['recovery_identity'])
        base,_,nodes=held.enter_context(authenticated_closure(record['reference']))
        with recovered_authorization(base,V4/'results/component_epoch_recovery/access',auth,request['recovery_identity']) as proof:
            layers,anchors=production.authorization_ancestry(auth)
            if anchors[5]['reference']!=base.reference or identity(auth)!=before:raise ValueError('AccessResolutionMismatch')
            durable_json(OUT/'ACCESS_CHECK.json',dict(passed=True,proof=proof,authorization_unchanged=True,
                original_reference=auth['approved_parent']['reference'],resolved_reference=anchors[5]['reference'],nodes=nodes,
                seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset))
    if production.authorization_ancestry is not raw:raise ValueError('RecoveryInstallerNotRestored')


def finish():
    b=read(OUT/'BASELINE.json');changed={p:file_hash(p) for p,h in b['protected'].items() if file_hash(p)!=h}
    inputs={p:file_hash(p) for p,h in b['inputs'].items() if file_hash(p)!=h}
    if unrelated()!=b['unrelated'] or changed or inputs:raise ValueError('RecoveryPreparationPreservationFailure',changed,inputs)
    durable_json(OUT/'PRESERVATION.json',dict(passed=True,exact_status_entries=len(b['unrelated']),protected_files=len(b['protected']),
        inputs=len(b['inputs']),disk=disk(),allocation=totals([p for p in OUT.rglob('*') if p.is_file()]),seconds=time.time()-b['start'],
        no_cleanup=True,no_front_published=True,no_new_layer=True))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','access_check','finish']);a=p.parse_args();globals()[a.mode]()
