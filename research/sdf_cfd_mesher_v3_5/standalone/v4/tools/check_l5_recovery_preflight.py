"""Bounded sealed-root recovery preflight. No production or cleanup executor.

Root/hash evidence is not a fresh scientific-state open. The existing recovery
contract requires a pinned whole-state legacy digest as well as the component
root. Never substitute the forecast-only digest or pass None to obtain a seal.
"""
from contextlib import ExitStack
from pathlib import Path
import json
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
OUT = V4/'results/l5_recovery_preflight'
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from check_current_front_encoding import allocation
from record_receipt_resume import disk, status
import immutable_state_access as access
import recursive_epoch_recovery as recovery
from sdf_cfd_mesher_v4.common.authority import identity

EXPECTED = '905511e4d6a2edcf0ca00dadd1f4ee8522bdb39634edfc1e83928eddf7074019'
CAP = 45120503439  # Existing retained monitor record is verified below.


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def add_reference(table, path, record):
    if path in table and table[path] != record:
        raise ValueError('ConflictingSealedReference', path)
    table[path] = record


def sealed_inventory(reference):
    """Authenticate root bytes only; do NOT claim payload or same-epoch access."""
    nodes, records, roots, seen = [], {}, [], set()
    with ExitStack() as held:
        while True:
            if reference['root'] in seen:
                raise ValueError('AncestorCycle')
            seen.add(reference['root'])
            body = recovery.sealed_body(reference, held)
            folder = Path(reference['directory'])
            roots.extend([folder/'CURRENT.json', folder/('ROOT_'+reference['root']+'.json')])
            kind = reference['kind']
            if kind == 'recursive':
                p = body['plan']
                if identity(p['content']) != p['scientific'] or body['scientific'] != p['scientific']:
                    raise ValueError('WrongRecursiveContent')
                a = body['acceptance']
                if (identity(a) != body['acceptance_identity'] or a['plan'] != identity(p)
                        or a['base'] != body['parent'] or a.get('passed') is not True):
                    raise ValueError('WrongSealedAcceptance')
                refs = {**p['references'], **body['produced']}
                # A duplicate between those tables must agree, not merely overwrite.
                for path in p['references'].keys() & body['produced'].keys():
                    if p['references'][path] != body['produced'][path]:
                        raise ValueError('ConflictingSealedReference', path)
                content, layer = p['scientific'], p['expected']['layer']
                implementation = {str(V4/'tools/composable_state.py'): p['implementation']}
            elif kind == 'native':
                if body['schema'] != access.SCHEMA or identity(body['content']) != body['content_root']:
                    raise ValueError('WrongNativeContent')
                refs, content = body['files'], body['content_root']
                layer = body['expected']['layer']
                implementation = body['content']['implementation']
            else:
                raise ValueError('UnsupportedRecoveryKind')
            checks = {path: file_hash(path) == h for path, h in implementation.items()}
            for path, record in refs.items():
                add_reference(records, path, record)
            nodes.append(dict(reference=reference, layer=layer, content=content,
                              epoch=body['epoch'], implementation_checks=checks,
                              root_sha256=file_hash(roots[-1])))
            if kind == 'native':
                break
            reference = p['parent']
    return nodes, records, roots


def totals(paths):
    return {k: v for k, v in allocation(paths).items() if k != 'records'}


def main():
    import psutil
    start = time.perf_counter()
    if OUT.exists():
        raise ValueError('PreserveRecoveryPreflight')
    baseline = read(V4/'results/retained_l6_storage_plan/BASELINE.json')
    roots = [V4/'results'/p for p in ('wing_receipt_resume/A',
              'current_front_transition_round_trip', 'wing_continuation_layers6to8')]
    files = [p for root in roots for p in root.rglob('*') if p.is_file()]
    before = {str(p): metadata(p) for p in files}
    protected = dict(baseline['protected'])
    for name in ('recursive_epoch_recovery.py', 'recursive_replay_retention.py', 'incremental_transaction.py'):
        p = V4/'tools'/name
        protected[str(p)] = file_hash(p)
    for p, h in protected.items():
        if file_hash(p) != h:
            raise ValueError('ProtectedSourceChanged', p)
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json', dict(head=subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(), status=status(),
        disk=disk(), protected=protected, campaign_files=len(files),
        scope='Three existing campaign roots only; root and selected-source hashes, payload metadata.'))
    reopened = read(roots[1]/'A/L5/REOPEN.json')
    if reopened['scientific'] != EXPECTED or reopened.get('passed') is not True:
        raise ValueError('WrongQualifiedL5Record')
    nodes, refs, root_paths = sealed_inventory(reopened['reference'])
    if nodes[0]['layer'] != 5 or nodes[0]['content'] != EXPECTED:
        raise ValueError('WrongL5Source')
    declared = {str(Path(p).resolve()) for p in refs} | {str(p.resolve()) for p in root_paths}
    in_campaign = [p for p in files if str(p.resolve()) in declared]
    outside = [p for p in files if str(p.resolve()) not in declared]
    unresolved, missing = [], []
    for path, r in refs.items():
        p = Path(path)
        if not p.is_file():
            missing.append(path)
        elif access.token(p) != r['token']:
            unresolved.append(path)
    stop = read(V4/'results/retained_l6_storage_plan/RESOURCE_STOP.json')
    if stop['resources']['output_limit_bytes'] != CAP:
        raise ValueError('ResourcePolicyChanged')
    quality_folder = V4/'results/single_quality_storage'
    quality_envelope = quality_folder/'retained_L6_rechecked/QUALITY_DELTA.json'
    quality_payload = Path(read(quality_envelope)['payload']['path'])
    quality_allocation = totals([quality_envelope, quality_payload])['unique_allocation_bytes']
    # Search only bounded retained evidence, never decode scientific payloads.
    evidence, digest_candidates = [], []
    for root in (roots[0], roots[1], V4/'results/retained_l5_finish'):
        for p in sorted(root.rglob('*.json')):
            if p.stat().st_size > 2*1024*1024:
                continue
            value = read(p)
            if not isinstance(value, dict):
                continue
            for key in ('legacy_scientific', 'native_digest', 'native_scientific_digest', 'deep_verified'):
                if key in value:
                    evidence.append(dict(path=str(p), key=key, value=value[key], sha256=file_hash(p)))
            if p.name == 'DEEP.json' and value.get('passed') is True and value.get('content') == EXPECTED:
                digest_candidates.append(dict(path=str(p), scientific=value.get('scientific')))
    blocks = []
    if not digest_candidates:
        blocks.append('PinnedWholeStateLegacyDigestUnavailable')
    if missing:
        blocks.append('MissingDeclaredPayload')
    if any(not all(n['implementation_checks'].values()) for n in nodes):
        blocks.append('RecoveryImplementationMismatch')
    # No automatic recovery from an unpinned digest, no speculative cleanup.
    result = dict(classification='V4L5RecoveryPreflightBlocked' if blocks else 'ReviewRequired',
        recovery_started=False, fresh_committed_open=False, no_layer_advance=True,
        last_qualified_layer=5, content=EXPECTED, source_reference=reopened['reference'],
        current_epoch=access.epoch(), sealed_nodes=nodes, declared_files=len(refs),
        missing=missing, changed_tokens=unresolved, blockers=blocks,
        expected_legacy_digest_candidates=digest_candidates, bounded_digest_search=evidence,
        scope='Sealed root bytes authenticated; payloads NOT deeply reauthenticated. No access promotion.',
        campaign=totals(files), declared_in_campaign=totals(in_campaign),
        not_declared_by_L5=totals(outside),
        largest_not_declared=[dict(path=str(p), bytes=p.stat().st_size)
                              for p in sorted(outside, key=lambda p:p.stat().st_size, reverse=True)[:15]],
        deletion_eligible_bytes=0, reclaimed_bytes=0,
        retention='Outside L5 closure is not redundancy. Existing L6 replay/publication/reopen is incomplete; no early credit.',
        resources=dict(output_limit_bytes=CAP, single_quality_allocated_bytes=quality_allocation,
                       two_independent_payload_lower_bound=totals(files)['unique_allocation_bytes']+2*quality_allocation,
                       lower_bound_not_peak=True),
        seconds=time.perf_counter()-start, peak_working_set=psutil.Process().memory_info().peak_wset,
        disk=disk())
    for p, r in before.items():
        if metadata(p) != r:
            raise ValueError('OriginalCampaignChanged', p)
    for p, h in protected.items():
        if file_hash(p) != h:
            raise ValueError('ProtectedSourceChanged', p)
    result['protected_hashes_preserved'] = len(protected)
    result['original_campaign_metadata_preserved'] = len(before)
    durable_json(OUT/'PREFLIGHT.json', result)
    print(json.dumps(result, indent=2), flush=True)


def preserved_status(base_rows, prior_unrelated, current_rows, owned):
    def strip(rows):
        return [r for r in rows if not any(r[3:]==p or p.endswith('/') and r[3:].startswith(p) for p in owned)]
    expected = strip(base_rows)
    if expected != prior_unrelated or strip(current_rows) != expected:
        raise ValueError('ExactUnrelatedStatusChanged')
    return expected


def finish():
    """Verify final preservation; never mutate previous evidence or state."""
    if (OUT/'FINAL.json').exists():
        raise ValueError('PreserveFinalEvidence')
    base = read(OUT/'BASELINE.json')
    old = read(V4/'results/retained_l6_storage_plan/BASELINE.json')
    own = ['research/sdf_cfd_mesher_v4/'+p for p in (
        'CURRENT_TASK.md', 'README.md', 'L5_RECOVERY_PREFLIGHT_REPORT.md',
        'tools/check_l5_recovery_preflight.py', 'tests/test_l5_recovery_preflight.py',
        'results/l5_recovery_preflight/')]
    expected = preserved_status(base['status'], old['unrelated'], status(), own)
    for p, h in base['protected'].items():
        if file_hash(p) != h:
            raise ValueError('ProtectedSourceChanged', p)
    retained = read(V4/'results/single_quality_storage/BASELINE.json')['census']['records']
    fields = ('device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams')
    for r in retained:
        now = metadata(r['path'])
        if any(now[k] != r[k] for k in fields):
            raise ValueError('CampaignMetadataChanged', r['path'])
    files = [p for p in OUT.rglob('*') if p.is_file()]
    files += [ROOT/p for p in own[:-1]]
    durable_json(OUT/'FINAL.json', dict(
        classification=read(OUT/'PREFLIGHT.json')['classification'],
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        unrelated_status_exact=True, unrelated_entries=len(expected),
        protected_hashes_preserved=len(base['protected']), retained_files_preserved=len(retained),
        disk=disk(), task_allocation_before_final_record=totals(files),
        files={str(p.relative_to(ROOT)):file_hash(p) for p in files},
        baseline_to_finish_seconds=time.time()-(OUT/'BASELINE.json').stat().st_mtime,
        scope='Baseline-to-finish is not whole-turn elapsed time; checker peak only is measured.',
        no_recovery=True, no_cleanup=True, no_layer_advance=True))


if __name__ == '__main__':
    if sys.argv[1:] == ['finish']:
        finish()
    elif not sys.argv[1:]:
        main()
    else:
        raise ValueError('UnsupportedPreflightMode')
