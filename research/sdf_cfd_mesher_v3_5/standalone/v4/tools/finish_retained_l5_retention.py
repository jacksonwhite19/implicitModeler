"""Recover private replay access after the documented exact-copy rollback.

No sealed state or scientific record changes. Existing guarded/hash-checked
access refresh is restricted to files named by the completed rollback.
"""
import copy
from contextlib import ExitStack
import time
from finish_retained_l5 import OUT,FOLDER,read,save,file_hash,metadata,durable_json,allocation,CAMPAIGN,disk
import immutable_state_access as access
from recursive_epoch_recovery import refresh_records
import recursive_replay_retention as retention


def refresh_private(folder,failed,destination):
    if destination.exists():raise ValueError('PreservePrivateRecoveryEvidence')
    compact=read(failed/'COMPACT.json');oldpath=folder/'replay/CANDIDATE_PLAN.json'
    failed_plan=read(failed/'PLAN.json')
    if file_hash(failed/'COMPACT.json')!=failed_plan['compact_sha256'] or file_hash(failed/'INVENTORY.json')!=failed_plan['inventory_sha256']:
        raise ValueError('ChangedFailedRetentionEvidence')
    if file_hash(oldpath)!=compact['private_plan']:raise ValueError('ChangedPrivateRecoveryPlan')
    a,b,reopened=retention.verify_replay(folder)
    restored=read(failed/'RECOVERY.json')['restored']
    if len(restored)!=len(set(restored)):raise ValueError('DuplicateRecoveryPath')
    actions={r['path']:r for r in failed_plan['actions']}
    if not set(restored)<=actions.keys():raise ValueError('UnplannedRestoredPayload')
    with retention.authenticated_closure(compact['reference']) as (state,closure,nodes),ExitStack() as held:
        if closure!=compact['closure'] or nodes!=compact['nodes']:raise ValueError('ChangedRecoveryAncestry')
        records=copy.deepcopy(b['references']);restored_records={}
        for path in restored:
            row=actions[path];canonical=row['removal_proof']['canonical']
            retention.check_duplicate(path,canonical,folder/'replay',set(closure),row['sha256'])
            # Existing attest checks full bytes under deny-write guards.
            held.enter_context(access.guard(path))
            if file_hash(path)!=row['sha256']:raise ValueError('ChangedRestoredPayload')
            if path in records:
                if records[path]['sha256']!=row['sha256']:raise ValueError('ConflictingRestoredIdentity')
                restored_records[path]=records[path]
        for path,record in records.items():
            if path not in restored_records:
                held.enter_context(access.guard(path))
                if access.token(path)!=record['token']:raise ValueError('UnexplainedPrivateMutation')
        records.update(refresh_records(restored_records,held,{}))
        new=copy.deepcopy(b);new['references']=records
        assert {k:v for k,v in new.items() if k!='references'}=={k:v for k,v in b.items() if k!='references'}
        destination.mkdir()
        durable_json(destination/'PRIOR_PRIVATE_PLAN.json',b)
        if file_hash(destination/'PRIOR_PRIVATE_PLAN.json')!=compact['private_plan']:
            raise ValueError('PrivatePlanArchiveBytesDiffer')
        if file_hash(oldpath)!=compact['private_plan']:raise ValueError('PrivatePlanChangedDuringRecovery')
        durable_json(destination/'NEW_PRIVATE_PLAN.json',new)
        durable_json(destination/'ACCESS_RECOVERY.json',dict(original=compact['private_plan'],
            replacement=file_hash(destination/'NEW_PRIVATE_PLAN.json'),restored=restored,
            refreshed_paths=sorted(restored_records),science=b['scientific'],
            implementation=file_hash(refresh_records.__code__.co_filename),
            scope='Existing refresh_records on exact rollback copies only; scientific fields, active ancestry and all other tokens unchanged'))
        durable_json(oldpath,new)
        if file_hash(oldpath)!=file_hash(destination/'NEW_PRIVATE_PLAN.json'):raise ValueError('PrivateRecoveryWriteMismatch')
    return reopened


def main():
    import psutil
    start=time.perf_counter();attempt=OUT/'retention_recovery'
    reopened=refresh_private(FOLDER,FOLDER/'replay_retention',attempt)
    evidence=attempt/'retention'
    retention.plan(FOLDER,evidence)
    result=retention.apply(FOLDER,evidence)
    removed=[]
    for root in (FOLDER,FOLDER/'replay'):
        record=read(root/'INDEX_RESTART.json');p=retention.Path(record['reconstruction'])
        canonical=FOLDER/'objects/new-proof-key-index.sqlite'
        if p!=root/'objects/restart-proof-key-index.sqlite' or file_hash(p)!=record['sha256'] or file_hash(canonical)!=record['sha256']:
            raise ValueError('ChangedRestartIndexBeforeRetention')
        removed.append(dict(**metadata(p),sha256=record['sha256'],recover_from=str(canonical)))
    save('RESTART_INDEX_RETENTION.json',dict(successful_reopen=reopened,files=removed))
    for row in removed:retention.Path(row['path']).unlink()
    phases=read(OUT/'PHASES.json');phases['retention_recovery']=time.perf_counter()-start
    save('RESULT.json',dict(passed=True,last_qualified_layer=5,reopen=reopened,
        comparisons=read(FOLDER/'REPLAY.json')['comparisons'],phases=phases,
        original_worker_seconds=read(OUT/'STOP.json')['seconds'],recovery_seconds=time.perf_counter()-start,
        peak_main_working_set=read(OUT/'STOP.json')['peak_main_working_set'],
        recovery_peak_working_set=psutil.Process().memory_info().peak_wset,
        retention=result,retention_evidence=str(evidence),
        restart_index_allocation_reclaimed=sum(r['cluster_bytes'] for r in removed),
        no_generation=True,no_proof_recomputation=True,no_layer6=True,
        preserved_routine_failure=file_hash(OUT/'STOP.json'),private_access_recovery=file_hash(attempt/'ACCESS_RECOVERY.json')))
    print('Retention completed',result,flush=True)

if __name__=='__main__':main()
