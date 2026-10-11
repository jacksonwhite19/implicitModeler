"""Connect successful-only retention to recursive, independently replayed states.

No live checkpoint reference is retired. Only exact private replay duplicates
are eligible, after publication/reopen. Failed and incomplete replays stay intact.
The old private CANDIDATE_PLAN is retained as historical evidence, not advertised
as a restart root after retirement. Reuse the qualified recursive duplicate
predicate and exact recovery, as in recursive_production_verification's retain.
"""
from contextlib import ExitStack, contextmanager
import json
from pathlib import Path
import time

import immutable_state_access as access
from composable_state import CommittedState
from path_preserving_deduplication import durable_json, file_hash, handle_hash, metadata, physical_key, probe_file_id
from execute_storage_cleanup import deletion_handle, delete_held
from sdf_cfd_mesher_v4.common.authority import identity
import successful_retention_cleanup as cleanup
from retain_recursive_layer6 import check_duplicate, recover

SCHEMA = 'V4RecursiveSuccessfulReplayRetention/1'
# These are independently written scientific exports or arrays. All controls,
# plans, stage outcomes, failure records, and nonidentical payloads stay on disk.
EXPORTS = frozenset(('GEOMETRY_DELTA.json','QUALITY_DELTA.json','BINDING_PATCH.json',
    'NEW_RECEIPT_INPUTS.json','NEW_ATLAS.json','NEW_FORECAST_DEPENDENCIES.json',
    'CANDIDATE_HOT.json','NEW_QUALITY.json','AFFECTED_INCIDENCE.json',
    'CHANGED_INPUTS.json','VOLUME_DELTA.json','GENERATED_NEXT.npz','SCREEN_ARRAYS.npz'))


def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))


def eligible_payload(relative, descriptor, quality_files=frozenset()):
    parts = relative.parts
    return (len(parts)==1 and relative.name in EXPORTS or
        len(parts)==2 and parts[0]=='objects' and relative.name in
        {*(descriptor[k]['file'] for k in ('faces','pairs','codes','payloads')), 'new-proof-key-index.sqlite'} or
        len(parts)==2 and parts[0]=='quality_objects' and relative.as_posix() in quality_files)


def quality_payloads(folder,primary_plan,replay_plan):
    """Only the exact payload authenticated by both prepared quality records."""
    if not (folder/'quality_objects').exists() and not (folder/'replay/quality_objects').exists():
        return set()  # Unchanged legacy retention never expands quality JSON.
    from single_quality_component import snapshot
    result=set()
    with snapshot(folder/'QUALITY_DELTA.json') as a, snapshot(folder/'replay/QUALITY_DELTA.json') as b:
        if a['encoded']!=b['encoded']:raise ValueError('MixedQualityReplayEncoding')
        if not a['encoded']:return result
        if a['semantic']!=b['semantic']:raise ValueError('QualityReplayLogicalMismatch')
        for snap,plan in ((a,primary_plan),(b,replay_plan)):
            for path,record in snap['references'].items():
                if plan['references'].get(path)!=record:raise ValueError('UnboundQualityReplayPayload')
        canonical=a['rows'].path;private=b['rows'].path
        relative=private.relative_to(folder/'replay')
        if (relative.parts[0]!='quality_objects' or len(relative.parts)!=2
            or canonical!=folder/relative):raise ValueError('UnsupportedQualityReplayPath')
        result.add(relative.as_posix())
    return result


@contextmanager
def authenticated_closure(reference):
    """Use the existing guarded reader; never reconstruct full proof populations."""
    with CommittedState(reference) as state:
        state.check(); node=state; paths={}; nodes=[]
        while True:
            directory=Path(node.reference['directory'])
            pointer=directory/'CURRENT.json'; root=directory/read(pointer)['file']
            paths[str(pointer.resolve())]=dict(sha256=file_hash(pointer))
            paths[str(root.resolve())]=dict(sha256=file_hash(root))
            if node.kind=='recursive':
                paths.update(node.plan['references']);paths.update(node.body['produced'])
                nodes.append(dict(layer=node.layer,scientific=node.scientific,root=node.root))
                node=node.parent
            elif node.kind=='native':
                paths.update(node.wrapped.body['files'])
                nodes.append(dict(layer=node.layer,scientific=node.scientific,root=node.root))
                break
            else:raise ValueError('UnsupportedRetentionAncestry')
        yield state,paths,nodes
        state.check()


def verify_replay(folder):
    folder=Path(folder); replay=folder/'replay'
    result=read(folder/'REPLAY.json'); reopened=read(folder/'REOPEN.json')
    if result.get('passed') is not True or reopened.get('passed') is not True:
        raise ValueError('SuccessfulPublishedReplayRequired')
    if (replay/'candidate/CURRENT.json').exists():
        raise ValueError('ReplayIsACommittedAncestry')
    a=read(folder/'CANDIDATE_PLAN.json');b=read(replay/'CANDIDATE_PLAN.json')
    comparisons={k:b['components'].get(k)==v for k,v in a['components'].items()}
    if comparisons!=result['comparisons'] or not all(comparisons.values()) or set(a['components'])!=set(b['components']):
        raise ValueError('ReplayComponentMismatch')
    if not (a['scientific']==b['scientific']==result['content']==result['replay_content']==reopened['scientific']):
        raise ValueError('ReplayScientificMismatch')
    if any(identity(p['content'])!=p['scientific'] for p in (a,b)):
        raise ValueError('ChangedReplayContent')
    return a,b,reopened


def restored_references(folder, recovery):
    """Authenticate only exact private copies restored by existing rollback.

    Original plans/tokens stay historical and unchanged. The new retention
    evidence owns new tokens. Every unrelated stale reference still fails.
    This is not checkpoint recovery, replay acceptance, or a restart authority.
    """
    if recovery is None:return {}
    folder=Path(folder).resolve();previous=Path(recovery['directory']).resolve()
    names={'PLAN.json','COMPACT.json','INVENTORY.json','RECOVERY.json'}
    if set(recovery['sha256'])!=names:raise ValueError('IncompleteRetentionRecoveryBinding')
    if (previous/'RESULT.json').exists():raise ValueError('SuccessfulRetentionAlreadyCompleted')
    with ExitStack() as held:
        for name in sorted(names):
            path=previous/name;held.enter_context(access.guard(path))
            if file_hash(path)!=recovery['sha256'][name]:raise ValueError('ChangedRetentionRecoveryEvidence')
        old=read(previous/'PLAN.json');compact=read(previous/'COMPACT.json')
        if (old.get('successful') is not True or compact['schema']!=SCHEMA
            or old['compact_sha256']!=recovery['sha256']['COMPACT.json']
            or old['inventory_sha256']!=recovery['sha256']['INVENTORY.json']):
            raise ValueError('ChangedRetentionRecoveryPlan')
        cleanup.validate_plan(old,folder)
        a,b,reopened=verify_replay(folder)
        for path,expected in ((folder/'REPLAY.json',compact['replay']),
            (folder/'REOPEN.json',compact['reopen']),
            (folder/'CANDIDATE_PLAN.json',compact['primary_plan']),
            (folder/'replay/CANDIDATE_PLAN.json',compact['private_plan'])):
            held.enter_context(access.guard(path))
            if file_hash(path)!=expected:raise ValueError('ChangedRecoveryReplayEvidence')
        if reopened['reference']!=compact['reference'] or a['scientific']!=compact['layer_scientific']:
            raise ValueError('WrongRetentionRecoveryState')
        actions={r['path']:r for r in old['actions']}
        restored=read(previous/'RECOVERY.json')['restored']
        if len(set(restored))!=len(restored) or not set(restored)<=actions.keys():
            raise ValueError('UnboundRestoredReplayPath')
        renewed={}
        for path in restored:
            row=actions[path];canonical=row['removal_proof']['canonical']
            for p in (path,canonical):held.enter_context(access.guard(p))
            check_duplicate(path,canonical,folder/'replay',set(compact['closure']),row['sha256'])
            if path in b['references']:
                if b['references'][path]['sha256']!=row['sha256']:
                    raise ValueError('RestoredReplayBoundHashMismatch')
                renewed[path]=access.attest(path,row['sha256'])
        return renewed


def verify_private_references(folder, plan, recovery=None, expected_renewed=None):
    renewed=restored_references(folder,recovery)
    if expected_renewed is not None and renewed!=expected_renewed:
        raise ValueError('ChangedRestoredReplayReference')
    for path,record in plan['references'].items():
        current=renewed.get(path,record)
        if access.token(path)!=current['token']:raise ValueError('StaleReplayReference:'+path)
    return renewed


def plan(folder, evidence, protected_reference=None, *, recovery=None):
    """Build a byte-exact discard record; do not touch any scientific payload."""
    folder=Path(folder).resolve();evidence=Path(evidence).resolve()
    if evidence.exists():raise ValueError('PreserveRetentionEvidence')
    a,b,reopened=verify_replay(folder); replay=folder/'replay'
    descriptor=read(replay/'FRESH_CERTIFICATE.json')['segment']
    quality_files=quality_payloads(folder,a,b)
    actions=[];kept=[]
    protected_reference=protected_reference or reopened['reference']
    with authenticated_closure(protected_reference) as (state,closure,nodes):
        if a['scientific'] not in {n['scientific'] for n in nodes}:raise ValueError('PublishedScientificMismatch')
        # Before retirement, every private prepared reference must still carry
        # its original token. A stale private plan is not proof of a replay.
        renewed=verify_private_references(folder,b,recovery)
        if recovery is not None:
            old=read(Path(recovery['directory'])/'COMPACT.json')
            if closure!=old['closure'] or nodes!=old['nodes']:
                raise ValueError('ChangedRetentionRecoveryClosure')
        for p in sorted(replay.rglob('*')):
            if not p.is_file():continue
            relative=p.relative_to(replay);canonical=folder/relative
            if not eligible_payload(relative,descriptor,quality_files):continue
            if str(p.resolve()) in closure:raise ValueError('ActiveReplayDependency')
            row=metadata(p);row['sha256']=file_hash(p)
            if not canonical.is_file() or canonical.stat().st_size!=row['bytes'] or file_hash(canonical)!=row['sha256']:
                kept.append(dict(path=str(p),reason='NoByteIdenticalCanonical',sha256=row['sha256']));continue
            check_duplicate(p,canonical,replay,set(closure),row['sha256'])
            # Verify any hashed reference, not only equality between two files.
            for path,table in ((p,b['references']),(canonical,a['references'])):
                bound=table.get(str(path.resolve()))
                if bound is not None and bound['sha256']!=row['sha256']:
                    raise ValueError('ReplayBoundHashMismatch')
            if row['links']!=1:
                kept.append(dict(path=str(p),reason='SharedAllocationPreserved',sha256=row['sha256']));continue
            row.update(classification='E',restore_dependency=False,next_layer_dependency=False,failure_evidence=False,
                removal_proof=dict(kind='exact_retained_duplicate',sha256=row['sha256'],canonical=str(canonical)),
                immutable_scope='Completed independent replay; never a committed root')
            actions.append(row)
        compact=dict(schema=SCHEMA,scientific=state.scientific,layer_scientific=a['scientific'],reference=protected_reference,nodes=nodes,
            replay=file_hash(folder/'REPLAY.json'),reopen=file_hash(folder/'REOPEN.json'),
            primary_plan=file_hash(folder/'CANDIDATE_PLAN.json'),private_plan=file_hash(replay/'CANDIDATE_PLAN.json'),
            comparisons=a['components'],closure=closure,kept=kept,
            discarded=[dict(path=r['path'],bytes=r['bytes'],sha256=r['sha256'],proof=r['removal_proof']) for r in actions],
            scope='Only byte-identical successful private replay exports. Old private plan tokens become historical, not restart authority. Active roots and their tokens stay unchanged.',
            reproduction='Copy each named canonical to its recorded retired path to recover exact bytes; do not claim old host-local private tokens remain executable.')
        if recovery is not None:
            compact.update(recovery=recovery,renewed_private_references=renewed)
        evidence.mkdir(parents=True)
        durable_json(evidence/'COMPACT.json',compact)
        durable_json(evidence/'INVENTORY.json',dict(actions=actions,kept=kept))
        result=dict(successful=True,compact_sha256=file_hash(evidence/'COMPACT.json'),
            inventory_sha256=file_hash(evidence/'INVENTORY.json'),actions=actions)
        cleanup.validate_plan(result,folder)
        durable_json(evidence/'PLAN.json',result)
    return result


def apply(folder,evidence):
    """Qualified duplicate retirement and exact recovery; no checkpoint writes."""
    folder=Path(folder).resolve();evidence=Path(evidence).resolve();started=time.perf_counter()
    if (evidence/'RESULT.json').exists():raise ValueError('PreserveRetentionResult')
    plan_record=read(evidence/'PLAN.json');compact=read(evidence/'COMPACT.json')
    if compact['schema']!=SCHEMA or file_hash(evidence/'COMPACT.json')!=plan_record['compact_sha256'] or file_hash(evidence/'INVENTORY.json')!=plan_record['inventory_sha256']:
        raise ValueError('ChangedRetentionPlan')
    a,b,reopened=verify_replay(folder)
    for path,expected in ((folder/'REPLAY.json',compact['replay']),(folder/'REOPEN.json',compact['reopen']),
                          (folder/'CANDIDATE_PLAN.json',compact['primary_plan']),(folder/'replay/CANDIDATE_PLAN.json',compact['private_plan'])):
        if file_hash(path)!=expected:raise ValueError('ChangedReplayEvidence')
    verify_private_references(folder,b,compact.get('recovery'),compact.get('renewed_private_references'))
    with authenticated_closure(compact['reference']) as (state,closure,nodes):
        if closure!=compact['closure'] or nodes!=compact['nodes']:raise ValueError('ChangedRetentionClosure')
        if any(r['path'] in closure for r in plan_record['actions']):raise ValueError('ActiveRetentionObject')
        cleanup.validate_plan(plan_record,folder)
        rows=[dict(path=r['path'],canonical=r['removal_proof']['canonical'],sha256=r['sha256']) for r in plan_record['actions']]
        # As in qualified recursive retention, a preserved manifest plus exact
        # canonical copies supports recovery. Never link an active output.
        for r in rows:check_duplicate(r['path'],r['canonical'],folder/'replay',set(closure),r['sha256'])
    removed=[]
    # Some canonical exports are not checkpoint dependencies. Protect ALL
    # retained counterparts through deletion, independent reopen and recovery.
    with ExitStack() as retained:
        for path in sorted({r['canonical'] for r in rows}):
            retained.enter_context(access.guard(path))
        try:
            with authenticated_closure(compact['reference']) as (state,closure,nodes):
                if closure!=compact['closure'] or nodes!=compact['nodes']:
                    raise ValueError('ChangedRetentionClosure')
                for r,record in zip(rows,plan_record['actions']):
                    # Qualified cleanup primitive: deny writes/replacement and
                    # retire this authenticated object, never a path occupant.
                    with deletion_handle(r['path']) as handle:
                        check_duplicate(r['path'],r['canonical'],folder/'replay',set(closure),r['sha256'])
                        now=metadata(r['path'])
                        if physical_key(now)!=physical_key(record) or now['links']!=1:
                            raise ValueError('ChangedReplayAllocation')
                        if handle_hash(handle)!=r['sha256']:
                            raise ValueError('ChangedReplayContentBeforeRetirement')
                        delete_held(handle);removed.append(r)
            with authenticated_closure(compact['reference']) as (state,closure,nodes):
                if state.scientific!=compact['scientific'] or closure!=compact['closure'] or nodes!=compact['nodes']:
                    raise ValueError('PostRetentionReopenMismatch')
                for r in rows:
                    if file_hash(r['canonical'])!=r['sha256']:raise ValueError('RetainedCanonicalChanged')
                verification=dict(schema=SCHEMA,guarded_ancestry_reopen=True,unchanged_scientific=True,
                    active_closure_preserved=True,exact_retained_duplicates=True,complete_replay=True)
                durable_json(evidence/'POST_VERIFY.json',dict(verification=verification,nodes=nodes,reference=compact['reference']))
        except BaseException:
            restored=recover(removed,folder/'replay',set(compact['closure']))
            durable_json(evidence/'RECOVERY.json',dict(restored=restored))
            raise
    retired=[]
    for r in plan_record['actions']:
        if probe_file_id(r['file_id'])==0:raise ValueError('RetiredAllocationStillOpen')
        retired.append(r)
    import psutil
    result=dict(passed=True,files=len(removed),logical_removed=sum(r['bytes'] for r in retired),
        allocation_reclaimed=sum(r['cluster_bytes'] for r in {physical_key(r):r for r in retired}.values()),
        scientific=compact['scientific'],seconds=time.perf_counter()-started,
        peak_process_working_set=psutil.Process().memory_info().peak_wset,
        reproduction='Exact bytes remain at canonical paths in COMPACT.json; no unique evidence removed.')
    durable_json(evidence/'RESULT.json',result)
    return result


def retain(folder):
    evidence=Path(folder)/'replay_retention'
    plan(folder,evidence)
    return apply(folder,evidence)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['plan','apply']);p.add_argument('folder',type=Path);p.add_argument('evidence',type=Path)
    p.add_argument('--protect-reference',type=Path,help='Latest committed REOPEN.json; protects every active descendant too.')
    args=p.parse_args()
    if args.mode=='plan':
        result=plan(args.folder,args.evidence,read(args.protect_reference)['reference'] if args.protect_reference else None)
        print(json.dumps(dict(actions=len(result['actions']),bytes=sum(r['bytes'] for r in result['actions']))))
    else:print(json.dumps(apply(args.folder,args.evidence)))
