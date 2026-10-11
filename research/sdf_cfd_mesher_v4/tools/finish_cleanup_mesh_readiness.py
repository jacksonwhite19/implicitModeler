"""Configure existing exact-duplicate retirement and authenticated access renewal.

No mesh executor, scientific predicate, recovery algorithm or index writer is
implemented here. Original scientific roots and their dependency bytes stay put.
"""
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
import json
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key, handle_hash
from record_receipt_resume import disk, status
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
import immutable_state_access as access
import execute_storage_cleanup as cleanup

OUT = V4/'results/storage_cleanup_mesh_readiness'
POINTER = V4/'results/fresh_successor_benchmark/current_parent_run/L8/REOPEN.json'
SCIENCE = 'e64b53451463bfe764fd2fb5726b5779658419def6eaf37bb2c63de4605be599'
OWN = ['research/sdf_cfd_mesher_v4/'+p for p in (
    'CURRENT_TASK.md', 'README.md', 'tools/finish_cleanup_mesh_readiness.py',
    'tools/native_reader_source_access.py', 'tests/test_native_reader_source_access.py',
    'STORAGE_CLEANUP_MESH_READINESS_REPORT.md', 'results/storage_cleanup_mesh_readiness/')]


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8'))


def check_baseline():
    b = read(OUT/'BASELINE.json')
    retired = set()
    if (OUT/'RESULT.json').exists():
        with ExitStack() as held:
            for name in ('PLAN.json', 'RESULT.json', 'CLEANUP_REVIEW.json'):
                held.enter_context(access.guard(OUT/name))
            result = read(OUT/'RESULT.json'); approved = read(OUT/'CLEANUP_REVIEW.json')
            if (result.get('passed') is not True or approved.get('passed') is not True
                or result['plan_sha256'] != approved['plan_sha256']
                or result['plan_sha256'] != file_hash(OUT/'PLAN.json')):
                raise ValueError('UnverifiedTaskRetirementStatus')
            # Only exact new deletion entries from the independently reviewed plan.
            # Modifications and every changed original entry still fail below.
            retired = {' D '+Path(r['target']).relative_to(ROOT).as_posix() for r in read(OUT/'PLAN.json')['rows']}
    rows = [r for r in status() if r not in retired and not any(r[3:] == p or p.endswith('/') and r[3:].startswith(p) for p in OWN)]
    if rows != b['status']:
        raise ValueError('ExactUnrelatedStatusChanged')
    if any(file_hash(p) != h for p, h in b['protected'].items()):
        raise ValueError('ProtectedSourceChanged')
    if file_hash(POINTER) != b['pointer_sha256'] or read(POINTER) != b['source']:
        raise ValueError('ChangedQualifiedL8Source')
    return b


def reviewed(held, phase):
    held.enter_context(access.guard(OUT/'BASELINE.json'))
    path = OUT/(phase+'_REVIEW.json')
    held.enter_context(access.guard(path))
    r = read(path)
    if r.get('passed') is not True or not r.get('reviewer') or r['baseline_sha256'] != file_hash(OUT/'BASELINE.json'):
        raise ValueError('IndependentReviewRequired')
    b = check_baseline()
    for p, h in r['sources'].items():
        held.enter_context(access.guard(p))
        if file_hash(p) != h:
            raise ValueError('ReviewedImplementationChanged', p)
    if r['sources'].get(str(Path(__file__).resolve())) != file_hash(__file__):
        raise ValueError('ReviewedHarnessRequired')
    return b, r


def source_access(held, approved):
    from native_reader_source_access import qualified_reader_access
    p = OUT/'SOURCE_CONNECTION.json'; review = OUT/'SOURCE_REVIEW.json'
    held.enter_context(access.guard(p)); held.enter_context(access.guard(review))
    if file_hash(p) != approved['source_connection_sha256']:
        raise ValueError('ChangedApprovedSourceConnection')
    return held.enter_context(qualified_reader_access(p, approved['source_connection_identity'],
        review, approved['source_review_sha256']))


def derive_source():
    from native_reader_source_access import derive
    if (OUT/'SOURCE_CONNECTION.json').exists():
        raise ValueError('PreserveSourceConnection')
    b = check_baseline()
    import recursive_epoch_recovery as recovery
    with ExitStack() as held:
        ref = b['source']['reference']
        while ref['kind'] != 'native':
            ref = recovery.sealed_body(ref, held)['plan']['parent']
        historical = V4/'results/capsule_source_compatibility/source/immutable_state_access.py'
        record = derive(ref, historical)
        durable_json(OUT/'SOURCE_CONNECTION.json', record)
    print('Reader source connection', identity(record), flush=True)


def baseline():
    if OUT.exists():
        raise ValueError('PreserveReadinessEvidence')
    old = read(V4/'results/storage_cleanup_execution/INITIAL.json')
    current = [r for r in status() if not any(r[3:] == p or p.endswith('/') and r[3:].startswith(p) for p in OWN)]
    if current != old['status']:
        raise ValueError('BaselineStatusChanged')
    if any(file_hash(p) != h for p, h in old['protected'].items()):
        raise ValueError('BaselineProtectedSourceChanged')
    source = read(POINTER)
    if source.get('passed') is not True or source['scientific'] != SCIENCE:
        raise ValueError('QualifiedL8Required')
    OUT.mkdir()
    from run_successor_benchmark import execution_binaries
    binaries = execution_binaries()
    controls = read(V4/'results/live_sf2_first_layer/PREFLIGHT_A.json')['resources']
    durable_json(OUT/'BASELINE.json', dict(start=time.time(),
        head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        status=current, protected=old['protected'], disk=disk(), source=source,
        pointer_sha256=file_hash(POINTER), binaries=binaries,
        minimum_free_bytes=controls['minimum_free_bytes'],
        scope='Access renewal and safe duplicate retirement only; no layer authorized or generated.'))
    print('Baseline', len(current), len(old['protected']), disk(), flush=True)


def recover():
    import recursive_epoch_recovery as recovery
    import psutil
    start = time.perf_counter()
    with ExitStack() as held:
        b, approved = reviewed(held, 'RECOVERY')
        source_access(held, approved)
        if disk()['C:\\']['available'] < b['minimum_free_bytes']:
            raise ValueError('MinimumFreeSpaceRequired')
        try:
            record = recovery.recover_components(b['source']['reference'], SCIENCE, OUT/'access',
                progress=lambda s: print(s, disk()['C:\\']['available'], flush=True))
            durable_json(OUT/'RECOVERY.json', dict(passed=True, record=record,
                seconds=time.perf_counter()-start, peak_working_set=psutil.Process().memory_info().peak_wset))
        except BaseException as exc:
            import traceback
            durable_json(OUT/'RECOVERY_FAILURE.json', dict(error=repr(exc), traceback=traceback.format_exc()))
            raise


def opened(renew=False):
    import numpy as np
    import psutil
    from capsule_reuse_dispatch import qualified_dispatch
    from composable_state import CommittedState
    from current_front_dependencies import install_reader
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure
    from persistent_index_renewal import IndexRenewal
    from persistent_numeric_incidence import open_geometry
    from incremental_pair_storage import PairIncidence
    from prism_persistent_hotpath import geometry_session
    from unittest.mock import patch
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    start = time.perf_counter()
    def denied(*args, **kwargs):
        raise AssertionError('NativeRestartNotNeeded')
    with ExitStack() as held:
        b, approved = reviewed(held, 'RECOVERY')
        source_access(held, approved)
        install_reader()
        rec = discover(OUT/'access')
        with qualified_dispatch(), patch.object(NativeCheckpointStore, 'restart', denied), authenticated_closure(rec['reference']) as (base, closure, nodes):
            if base.layer != 8 or base.scientific != SCIENCE:
                raise ValueError('WrongRecoveredMeshState')
            if renew:
                manifest_path = Path(approved['index_evidence']['path'])
                held.enter_context(access.guard(manifest_path))
                if file_hash(manifest_path) != approved['index_evidence']['sha256']:
                    raise ValueError('ChangedQualifiedIndexEvidence')
                manifest = read(manifest_path)
                # Existing evidence binds the original version-closure bytes.
                # It has no mutable payload and is not a missing cell/proof.
                records = dict(rec['files'])
                for relative in approved['index_evidence']['additional']:
                    p = str((manifest_path.parent/relative).resolve())
                    if p in records:
                        raise ValueError('UnexpectedSupplementalIndexDependency')
                    row = manifest['artifacts'][relative]
                    held.enter_context(access.guard(p))
                    records[p] = access.attest(p, row['sha256'])
                renewal = held.enter_context(IndexRenewal(OUT/'indices_verified', records, identity(records)))
                geometry = renewal.geometry(base.plan['persistent_geometry'])
                pairs = renewal.pairs(dict(path=base.plan['pair_incidence_manifest'], root=base.plan['pair_incidence_root']), base.content_root)
                durable_json(OUT/'INDEX_ACCESS.json', dict(geometry=geometry, pairs=pairs, evidence=renewal.evidence()))
            else:
                held.enter_context(access.guard(OUT/'INDEX_ACCESS.json'))
                held.enter_context(access.guard(OUT/'OPEN.json'))
                if file_hash(OUT/'INDEX_ACCESS.json') != read(OUT/'OPEN.json')['index_access_sha256']:
                    raise ValueError('ChangedRenewedIndexBinding')
                idx = read(OUT/'INDEX_ACCESS.json'); geometry, pairs = idx['geometry'], idx['pairs']
            # Inspect actual stored coordinates. No planner, forecast or predictor.
            front = np.asarray([base.hot['vertices'][f'layer:8:source:{i}'] for i in range(53153)], dtype='<f8')
            if array_hash(front) != base.expected['front_coordinates']:
                raise ValueError('StoredFrontCoordinateMismatch')
            base.numeric_incidence = geometry
            try:
                with open_geometry(geometry, base.content_root) as numeric, PairIncidence(base, pairs['path'], pairs['root']) as pair_view:
                    first_key = next(iter(numeric.delta))
                    face = numeric[first_key]
                    cells = sum(len(a) for a in numeric.cell_segments)
                    if cells != 850416:
                        raise ValueError('WrongCurrentCellCount')
                    pair_probe = pair_view.affected([])
                    view = base['forecast']; first = next(iter(view)); sample = view[first]
                    base.check()
                    result = dict(passed=True, layer=base.layer, scientific=base.scientific,
                        reference=base.reference, nodes=nodes, closure_files=len(closure),
                        vertices=9*len(front), cells=cells, faces=len(numeric),
                        incidence_sample_key=list(first_key), incidence_sample=face,
                        front_coordinates=array_hash(front), proofs=len(base['certificate']), quality=len(base['quality']),
                        pair_probe=pair_probe, forecast_rows=len(view), forecast_sample_key=first,
                        forecast_sample_identity=identity(sample), binaries_verified=len(b['binaries']),
                        index_access_sha256=file_hash(OUT/'INDEX_ACCESS.json'),
                        no_restart=True, no_forecast_or_generation=True,
                        next_layer_authorized=False, pending_scope='Existing authorization ended at L8.',
                        seconds=time.perf_counter()-start, peak_working_set=psutil.Process().memory_info().peak_wset)
                for p, h in b['binaries'].items():
                    if file_hash(p) != h:
                        raise ValueError('ChangedExecutionBinary', p)
            finally:
                del base.numeric_incidence
        path = OUT/('OPEN.json' if renew else 'REOPEN.json')
        if path.exists():
            raise ValueError('PreserveReadinessOpenEvidence')
        durable_json(path, result)
    print(json.dumps(result), flush=True)


def pairs():
    result = []
    roots = [
        ('fresh_successor_benchmark/current_parent_run/L7', 'ab40fe604eda815c8f25c877316de01de31e48356bcc754b9fa54986c4210af5'),
        ('fresh_successor_benchmark/current_parent_run/L8', '253639a54a717cb6e910901e45a51a6e6b7ab44f10e77978c2235da961800692'),
        ('wing_receipt_resume/A/L5', None), ('current_front_transition_round_trip/A/L5', None)]
    from recursive_replay_retention import verify_replay
    for root, sql in roots:
        folder = V4/'results'/root
        a, b, _ = verify_replay(folder)
        names = ['numeric_delta/winding.npy', 'numeric_lookup/hashes.npy']
        if sql:
            names.append('quality_objects/'+sql+'.sqlite')
        for name in names:
            target, canonical = folder/'replay'/name, folder/name
            left, right = a['references'].get(str(canonical.resolve())), b['references'].get(str(target.resolve()))
            if left is None or right is None or left['sha256'] != right['sha256']:
                raise ValueError('UnboundSuccessfulReplayDuplicate', name)
            if access.token(target) != right['token'] or access.token(canonical) != left['token']:
                raise ValueError('ChangedSelectedReplayReference', name)
            result.append(dict(target=str(target.resolve()), canonical=str(canonical.resolve()),
                sha256=left['sha256'], scope='successful private replay', folder=str(folder),
                evidence={str(folder/p):file_hash(folder/p) for p in ('REPLAY.json','REOPEN.json','CANDIDATE_PLAN.json','replay/CANDIDATE_PLAN.json')}))
    for n in range(1,21):
        target = V4/'results/task4i/fresh'/f'LAYER{n}_PLANNER.npz'
        canonical = V4/'results/task4l/fresh'/target.name
        # Both original authorities declare these exact file identities.
        evidence = {str(V4/'results'/x/'AUTHORITY.json'):file_hash(V4/'results'/x/'AUTHORITY.json') for x in ('task4i','task4l')}
        hashes = [read(V4/'results'/x/'AUTHORITY.json')['evidence_sha256'][(p.relative_to(V4)).as_posix()]
                  for x, p in (('task4i', target), ('task4l', canonical))]
        if hashes[0] != hashes[1]:
            raise ValueError('HistoricalPlannerAuthorityMismatch')
        result.append(dict(target=str(target.resolve()), canonical=str(canonical.resolve()),
            sha256=hashes[0], scope='ordinary historical planner copy', evidence=evidence))
    return result


def plan():
    check_baseline()
    if (OUT/'PLAN.json').exists():
        raise ValueError('PreserveRetirementPlan')
    result = []
    with ExitStack() as held:
        protected, roots, keys = cleanup.protected_namespace(held)
        for row in pairs():
            target = cleanup.checked_paths(row['target'], V4/'results')
            canonical = cleanup.checked_paths(row['canonical'], V4/'results')
            if str(target) in protected or target == canonical:
                raise ValueError('ProtectedCleanupTarget')
            held.enter_context(access.guard(target)); held.enter_context(access.guard(canonical))
            a, b = metadata(target), metadata(canonical)
            if a['links'] != 1 or a['streams'] != ['::$DATA'] or a['delete_pending']:
                raise ValueError('UnsupportedCleanupTarget')
            if physical_key(a) in keys or physical_key(a) == physical_key(b):
                raise ValueError('ProtectedOrSharedPhysicalTarget')
            if file_hash(target) != row['sha256'] or file_hash(canonical) != row['sha256']:
                raise ValueError('NotExactDuplicate')
            result.append(dict(row, target_metadata=a, canonical_metadata=b))
        durable_json(OUT/'PLAN.json', dict(rows=result, protected_files=len(protected), protected_roots=len(roots),
            logical_bytes=sum(r['target_metadata']['bytes'] for r in result),
            allocated_bytes=sum(r['target_metadata']['cluster_bytes'] for r in result),
            retained='Original authorities, proofs, failure records, all current ancestry and canonical copies.',
            restoration='Copy canonical bytes to original path before historical audits; file IDs and execution provenance are not restored.'))
    print('Plan', len(result), sum(r['target_metadata']['cluster_bytes'] for r in result), flush=True)


def apply():
    start = time.perf_counter(); removed = []
    if (OUT/'RESULT.json').exists():
        raise ValueError('PreserveRetirementResult')
    with ExitStack() as held:
        _, review = reviewed(held, 'CLEANUP')
        for name, sha in cleanup.SOURCES.items():
            path = V4/'tools'/name
            held.enter_context(access.guard(path))
            if file_hash(path) != sha:
                raise ValueError('ChangedQualifiedCleanupHelper', name)
        held.enter_context(access.guard(OUT/'PLAN.json'))
        if review['plan_sha256'] != file_hash(OUT/'PLAN.json'):
            raise ValueError('ChangedReviewedPlan')
        record = read(OUT/'PLAN.json')
        current = pairs()
        if current != [{k:v for k,v in r.items() if k not in ('target_metadata','canonical_metadata')} for r in record['rows']]:
            raise ValueError('ChangedRetirementSelection')
        protected, roots, keys = cleanup.protected_namespace(held)
        # Hold canonical payloads, original evidence and all path ancestors.
        directories = set()
        for row in record['rows']:
            for p, h in row['evidence'].items():
                held.enter_context(access.guard(p))
                if file_hash(p) != h:
                    raise ValueError('ChangedOriginalEvidence')
            c = cleanup.checked_paths(row['canonical'], V4/'results')
            held.enter_context(access.guard(c))
            if metadata(c) != row['canonical_metadata'] or file_hash(c) != row['sha256']:
                raise ValueError('ChangedCanonicalCopy')
            p = cleanup.checked_paths(row['target'], V4/'results')
            for path in (p, c):
                for d in path.parents:
                    if d == V4/'results': break
                    directories.add(d)
        for d in directories:
            held.enter_context(cleanup.directory_guard(d))
        initial_disk = disk()
        try:
            for row in record['rows']:
                p = cleanup.checked_paths(row['target'], V4/'results')
                if str(p) in protected:
                    raise ValueError('ProtectedTarget')
                with cleanup.deletion_handle(p) as h:
                    actual = metadata(p)
                    cleanup.same_metadata(actual, row['target_metadata'])
                    if physical_key(actual) in keys or handle_hash(h) != row['sha256']:
                        raise ValueError('ChangedTargetOrProtectedPhysicalFile')
                    durable_json(OUT/'INTENT.json', dict(pending=row,
                        completed=[r['target'] for r in removed], plan_sha256=file_hash(OUT/'PLAN.json')))
                    cleanup.delete_held(h)
                removed.append(row)
                durable_json(OUT/'PROGRESS.json', dict(removed=[r['target'] for r in removed], plan_sha256=file_hash(OUT/'PLAN.json')))
            if any(Path(r['target']).exists() or file_hash(r['canonical']) != r['sha256'] for r in removed):
                raise ValueError('CleanupReadbackFailure')
            for p, r in protected.items():
                if access.token(p) != r['token']:
                    raise ValueError('ProtectedNamespaceChanged')
        except BaseException as exc:
            durable_json(OUT/'FAILURE.json', dict(error=repr(exc), removed=[r['target'] for r in removed],
                recovery='PLAN.json retains exact canonical paths and SHA256; do not rerun apply blindly.'))
            raise
        durable_json(OUT/'RESULT.json', dict(passed=True, removed=len(removed),
            logical_bytes=sum(r['target_metadata']['bytes'] for r in removed),
            allocated_bytes=sum(r['target_metadata']['cluster_bytes'] for r in removed),
            initial_disk=initial_disk, final_disk=disk(), seconds=time.perf_counter()-start,
            protected_files=len(protected), protected_roots=len(roots), plan_sha256=file_hash(OUT/'PLAN.json')))


def finish():
    import psutil
    b = check_baseline()
    for name in ('OPEN.json', 'REOPEN.json', 'RESULT.json'):
        if read(OUT/name).get('passed') is not True:
            raise ValueError('RequiredReadinessPhaseMissing', name)
    a, z = read(OUT/'OPEN.json'), read(OUT/'REOPEN.json')
    for key in ('layer','scientific','cells','vertices','faces','proofs','quality','front_coordinates','forecast_sample_identity','index_access_sha256'):
        if a[key] != z[key]:
            raise ValueError('ReadinessReopenDifference', key)
    if a['layer'] != 8 or a['scientific'] != SCIENCE or read(OUT/'RESULT.json')['removed'] != 30:
        raise ValueError('IncompleteReadinessScope')
    if not read(OUT/'RECOVERY.json')['record'].get('components_verified'):
        raise ValueError('VerifiedRecoveryRequired')
    if read(OUT/'OUTCOME_REVIEW.json').get('passed') is not True:
        raise ValueError('IndependentOutcomeReviewRequired')
    for row in read(OUT/'PLAN.json')['rows']:
        if Path(row['target']).exists() or file_hash(row['canonical']) != row['sha256']:
            raise ValueError('RetirementFinalReadbackFailure')
    files = [p for p in OUT.rglob('*') if p.is_file()]
    rows = [metadata(p) for p in files]
    durable_json(OUT/'FINAL.json', dict(passed=True, disk=disk(), seconds=time.time()-b['start'],
        unrelated_entries=len(b['status']), protected_files=len(b['protected']),
        logical_bytes=sum(r['bytes'] for r in rows),
        allocated_bytes=sum(r['cluster_bytes'] for r in {physical_key(r):r for r in rows}.values()),
        hashes={str(p.relative_to(ROOT)):file_hash(p) for p in files},
        no_layer_advanced=True, next_layer_authorized=False,
        peak_working_set=psutil.Process().memory_info().peak_wset))


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'open': opened(True)
    elif mode == 'reopen': opened(False)
    else: {'baseline':baseline, 'derive-source':derive_source, 'recover':recover, 'plan':plan, 'apply':apply, 'finish':finish}[mode]()
