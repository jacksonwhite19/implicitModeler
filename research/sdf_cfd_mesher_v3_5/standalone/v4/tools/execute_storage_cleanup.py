"""Retire only the reviewed ordinary duplicate paths. Never retires states.

This is an exact-file cleanup, not checkpoint recovery or scientific reopen.
The compressed recovery manifest and every retained counterpart stay intact.
"""
import argparse
from contextlib import ExitStack, contextmanager
import ctypes
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path.insert(0, str(V4 / 'src'))
import immutable_state_access as access
from path_preserving_deduplication import (api, W, durable_json, file_hash,
                                         handle_hash, metadata, physical_key)
from recursive_epoch_recovery import sealed_body
from record_receipt_resume import disk

OUT = V4 / 'results/storage_cleanup_execution'
HANDOFF = V4 / 'results/storage_cleanup_handoff'
MANIFEST_SHA = '2a4c8ca53281b72e4a547b4abb8a42e5de88dd01dbc7739a4642e6c7a2a13e23'
RAW_SHA = '083c2037b52990efe42ab1d59dfb63058839c5d355d7fd23bf76a72758f155c0'
SOURCES = {
    'path_preserving_deduplication.py': 'a27bcece72535dd4ca9eaaf6b19439bb6a00ba051f5eba3d1498579d9296e5c9',
    'immutable_state_access.py': 'a5ac977ee17c109a0b5d2aa52b68b9a59a35c659b914496fa5bfb449ed374ec5',
    'recursive_epoch_recovery.py': '7d31d23a93a2030776176b2c44304057366843c855ae0061b5d0b08ff3d8066e',
    'rolling_layer_retention.py': '6e1d80c2d8be7f226322a5cb0b7abd936c6a16127a971ddcbe41279645b93051',
}
STARTS = (
    ('fresh_successor_benchmark/current_parent_run/L8/REOPEN.json', '33d63744abbe66fb58beb30f4c3e138bec29b6b065c4738916c9d6b7bf07822b'),
    ('capsule_source_compatibility/control/run/REOPEN.json', 'f716d73cf1453065dc0c35d4e1e00ada6c7c9939ece38cd9677843c52f1bb566'),
    ('assembly_binding_production/run/REOPEN.json', '22219bdc2db8e7596f082de074f17b40020652b251cd257f42c49ad20db9f0ec'),
)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def checked_paths(path, base):
    """Validate lexical ancestry before resolving; never accepts a reparse hop."""
    path, base = Path(path).absolute(), Path(base).resolve()
    if not path.is_relative_to(base) or path == base:
        raise ValueError('CleanupPathOutsideScope', str(path))
    for node in (path, *path.parents):
        if node.lstat().st_file_attributes & 0x400:
            raise ValueError('CleanupReparseAncestor', str(node))
    if path.resolve() != path:
        raise ValueError('CleanupPathAlias', str(path))
    return path


@contextmanager
def directory_guard(path):
    k = api()
    h = k.CreateFileW(str(path), 0x80000000, 1, None, 3, 0x02000000, None)
    if h == W.HANDLE(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        yield h
    finally:
        k.CloseHandle(h)


@contextmanager
def deletion_handle(path):
    """Hold this exact file against writers/replacement; delete by this handle."""
    k = api()
    h = k.CreateFileW(str(path), 0x80010000, 1, None, 3, 0, None)
    if h == W.HANDLE(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        yield h
    finally:
        k.CloseHandle(h)


def delete_held(handle):
    class Disposition(ctypes.Structure):
        _fields_ = [('delete', ctypes.c_ubyte)]
    k = api()
    k.SetFileInformationByHandle.argtypes = [W.HANDLE, ctypes.c_int, ctypes.c_void_p, W.DWORD]
    d = Disposition(1)
    if not k.SetFileInformationByHandle(handle, 4, ctypes.byref(d), ctypes.sizeof(d)):
        raise ctypes.WinError(ctypes.get_last_error())


def same_metadata(actual, expected):
    if any(actual.get(k) != expected[k] for k in expected if k != 'sha256'):
        raise ValueError('ChangedCleanupFileMetadata', expected['path'])
    if actual['links'] != 1 or actual['delete_pending'] or actual['streams'] != ['::$DATA']:
        raise ValueError('UnsupportedCleanupFile', actual['path'])


def manifest(held):
    p = HANDOFF / 'DUPLICATE_CANDIDATES.json.gz'
    held.enter_context(access.guard(p))
    if file_hash(p) != MANIFEST_SHA:
        raise ValueError('ChangedReviewedCleanupManifest')
    raw = gzip.decompress(p.read_bytes())
    if hashlib.sha256(raw).hexdigest() != RAW_SHA:
        raise ValueError('ChangedReviewedCleanupContents')
    r = json.loads(raw)
    if r['failures'] or len(r['candidates']) != 13306:
        raise ValueError('IncompleteReviewedCleanupPlan')
    for path, sha in r['manifest_hashes'].items():
        held.enter_context(access.guard(path))
        if file_hash(path) != sha:
            raise ValueError('ChangedOriginalStudyManifest', path)
    held.enter_context(access.guard(HANDOFF/'HANDOFF.json'))
    h = read(HANDOFF / 'HANDOFF.json')
    if (HANDOFF/'HANDOFF.json').read_bytes() != subprocess.check_output(
            ['git', 'show', 'HEAD:'+(HANDOFF/'HANDOFF.json').relative_to(ROOT).as_posix()], cwd=ROOT):
        raise ValueError('ChangedCommittedCleanupHandoff')
    for pin in h['origin_pins'].values():
        p = Path(pin['authority_path'])
        held.enter_context(access.guard(p))
        if file_hash(p) != pin['sha256']:
            raise ValueError('ChangedOriginalStudyAuthority')
        committed = subprocess.check_output(['git', 'show', 'HEAD:' + p.relative_to(ROOT).as_posix()], cwd=ROOT)
        if p.read_bytes() != committed:
            raise ValueError('ChangedCommittedStudyAuthority')
    return r


def protected_namespace(held, extra_starts=()):
    """Sealed dependency names only. Does not accept/renew host-epoch trust."""
    records, roots, seen, visiting = {}, [], set(), set()
    def add(rows):
        for p, r in rows.items():
            p = str(Path(p).resolve())
            if p in records and records[p] != r:
                raise ValueError('ConflictingSealedDependency', p)
            records[p] = r
    def walk(ref):
        key = (str(Path(ref['directory']).resolve()), ref['root'])
        if key in visiting:
            raise ValueError('CyclicProtectedAncestry')
        if key in seen:
            return
        seen.add(key)
        visiting.add(key)
        b = sealed_body(ref, held)
        directory = Path(key[0])
        roots.append(dict(reference=ref, schema=b['schema']))
        for p in (directory/'CURRENT.json', directory/('ROOT_'+ref['root']+'.json')):
            add({str(p.resolve()): dict(sha256=file_hash(p), token=access.token(p))})
        if b['schema'] == access.SCHEMA:
            if not b.get('complete'):
                raise ValueError('IncompleteNativeNamespace')
            add(b['files'])
            for row in b['hot'].values():
                p = str(Path(row['path']).resolve())
                if p not in b['files'] or b['files'][p]['sha256'] != row['sha256']:
                    raise ValueError('UndeclaredNativeDependency', p)
            for p in (Path(b['store'])/'CURRENT.json', Path(b['store'])/b['legacy_pointer']['checkpoint']):
                if str(p.resolve()) not in b['files']:
                    raise ValueError('UndeclaredNativeCheckpoint', str(p))
            # Reuse the native descriptor walker, including prism backing JSON.
            from rolling_layer_retention import checkpoint_closure
            for path, row in b['files'].items():
                held.enter_context(access.guard(path))
                if access.token(path) != row['token']:
                    raise ValueError('ChangedNativeDependencyToken', path)
            closure, envelope = checkpoint_closure(Path(b['store']))
            if envelope['components'] != b['components']:
                raise ValueError('ChangedNativeComponentMapping')
            for p, sha in closure.items():
                if p not in b['files'] or b['files'][p]['sha256'] != sha:
                    raise ValueError('IncompleteNativeDescriptorNamespace', p)
            visiting.remove(key)
            return
        if b['schema'] != 'V4IncrementalComponentTransaction/1':
            raise ValueError('UnsupportedCleanupProtectedSchema', b['schema'])
        p = b['plan']
        if p.get('schema') not in ('V4RecursiveCommittedState/1', 'V4CapsuleCompleteCorePlan/1'):
            raise ValueError('UnsupportedCleanupProtectedPlan', p.get('schema'))
        if not p.get('parent'):
            raise ValueError('IncompleteSealedProductionNamespace')
        add(p['references']); add(b['produced'])
        for name in ('quality_delta', 'binding_patch'):
            if str(Path(b[name]).resolve()) not in records:
                raise ValueError('UndeclaredProductionDependency', name)
        walk(p['parent'])
        visiting.remove(key)
    for name, expected in STARTS:
        p = V4 / 'results' / name
        held.enter_context(access.guard(p)); add({str(p.resolve()): dict(sha256=file_hash(p), token=access.token(p))})
        ref = read(p)['reference']
        if ref['root'] != expected:
            raise ValueError('ChangedProtectedStartingRoot', name)
        walk(ref)
    for ref in extra_starts:
        walk(ref)
    # Existing complete-core transaction uses its own plan, not a front plan.
    p = V4/'results/capsule_c1_core_qualification/COMPLETE_AUTHORITY.json'
    held.enter_context(access.guard(p))
    add({str(p.resolve()):dict(sha256=file_hash(p), token=access.token(p))})
    r = read(p)
    walk(dict(directory=str(p.parent/'complete_mesh'), root=r['access']))
    walk(dict(directory=str(V4/'results/capsule_complete_mesh_authority/access'),
              root='363fd43712f25bea209d81afef14a87f5da7af7ac4144ea64b76c0821d28e553'))
    # Protect entire small sealed surface folder, including its interface records.
    folder = V4/'results/live_sf2_surface/replay_20260929/authority'
    for p in sorted(folder.iterdir()):
        if not p.is_file():
            raise ValueError('UnsupportedSealedSurfaceEntry', str(p))
        held.enter_context(access.guard(p))
        add({str(p.resolve()):dict(sha256=file_hash(p), token=access.token(p))})
    folder = V4/'results/production_adaptive_capsule_surface_foundation/fresh'
    p = folder/'SURFACE_AUTHORITY.json'
    held.enter_context(access.guard(p))
    add({str(p.resolve()):dict(sha256=file_hash(p), token=access.token(p))})
    for name, sha in read(p)['files'].items():
        target = checked_paths(folder/name, folder)
        held.enter_context(access.guard(target))
        if file_hash(target) != sha:
            raise ValueError('ChangedCapsuleSurfaceDependency', name)
        add({str(target.resolve()):dict(sha256=sha, token=access.token(target))})
    # Every declared reader file remains held; tokens prove exclusion only.
    physical = set()
    for p, r in records.items():
        held.enter_context(access.guard(p))
        if access.token(p) != r['token']:
            raise ValueError('ChangedSealedDependencyToken', p)
        st = Path(p).stat(); physical.add((st.st_dev, str(st.st_ino)))
    return records, roots, physical


def execute(apply=False, extra_starts=(), preflight_record='PREFLIGHT_V2.json'):
    started = time.perf_counter()
    if Path(preflight_record).name != preflight_record or not preflight_record.startswith('PREFLIGHT') or not preflight_record.endswith('.json'):
        raise ValueError('InvalidCleanupPreflightPath')
    name = 'RESULT.json' if apply else preflight_record
    if (OUT/name).exists():
        raise ValueError('PreserveCleanupExecutionEvidence', name)
    result = dict(deletion_performed=False, removed=0, logical_bytes=0,
                  retired_allocation=0, initial_disk=disk(), manifest_sha256=MANIFEST_SHA,
                  scientific_reopen=False, namespace_scope='Sealed names/tokens for retirement exclusion only; host epoch not renewed.')
    OUT.mkdir(exist_ok=True)
    with ExitStack() as held:
        for source, sha in SOURCES.items():
            p = V4/'tools'/source
            held.enter_context(access.guard(p))
            if file_hash(p) != sha:
                raise ValueError('ChangedReviewedCleanupHelper', source)
        result['source_dependencies'] = SOURCES
        r = manifest(held)
        protected, roots, physical = protected_namespace(held, extra_starts)
        pairs = r['candidates']; archive = r['reconstructible_archive']
        targets = [row['target'] for row in pairs] + [archive['target']]
        canonical = {row['canonical']['path']: row['canonical'] for row in pairs}
        target_paths = {row['path'] for row in targets}
        target_keys = {physical_key(row) for row in targets}
        if len(target_paths) != len(targets) or len(target_keys) != len(targets):
            raise ValueError('RepeatedCleanupTarget')
        if target_paths & (set(protected) | set(canonical)) or target_keys & physical:
            raise ValueError('ProtectedCleanupTarget')
        directories = set()
        for path in (*target_paths, *canonical, *(p['path'] for p in archive['parts'])):
            p = checked_paths(path, V4/'results')
            directories.update(p.parents)
        for p in sorted(directories, key=lambda p:len(p.parts)):
            held.enter_context(directory_guard(p))
        for number, row in enumerate(targets):
            base = (V4/'results/capsule_sa_outer_map' if 'capsule_sa_outer_map' in Path(row['path']).parts
                    else V4/'results/complete_mesh_numerical_qualification')
            checked_paths(row['path'], base)
            same_metadata(metadata(row['path']), row)
            if file_hash(row['path']) != row['sha256']:
                raise ValueError('ChangedCleanupTargetHash', row['path'])
            if number % 1000 == 0:
                print('Checked targets', number, '/', len(targets), flush=True)
        for path, row in canonical.items():
            held.enter_context(access.guard(path))
            if metadata(path) != {k:v for k,v in row.items() if k != 'sha256'} or file_hash(path) != row['sha256']:
                raise ValueError('ChangedRetainedCounterpart', path)
        for row in pairs:
            if row['expected_sha256'] != row['target']['sha256'] or row['expected_sha256'] != row['canonical']['sha256']:
                raise ValueError('CleanupPairMismatch')
        joined = hashlib.sha256()
        for part in archive['parts']:
            p = Path(part['path']); held.enter_context(access.guard(p))
            if p.stat().st_size != part['bytes'] or file_hash(p) != part['sha256']:
                raise ValueError('ChangedArchivePart')
            with p.open('rb') as f:
                for block in iter(lambda:f.read(4*1024*1024), b''):
                    joined.update(block)
        if joined.hexdigest() != archive['expected_sha256']:
            raise ValueError('ArchiveJoinMismatch')
        result.update(protected_files=len(protected), protected_roots=roots, validated_targets=len(targets),
                      predicted_logical_bytes=sum(x['bytes'] for x in targets),
                      predicted_allocation=sum(x['cluster_bytes'] for x in targets))
        # The pinned compressed manifest is the complete durable recovery map.
        if apply:
            held.enter_context(access.guard(__file__))
            held.enter_context(access.guard(OUT/'REVIEW.json'))
            held.enter_context(access.guard(OUT/preflight_record))
            review = read(OUT/'REVIEW.json')
            if review.get('verdict') != 'PASS_FOR_EXECUTION' or review.get('source_sha256') != file_hash(__file__):
                raise ValueError('CleanupExecutionReviewRequired')
            if review.get('source_dependencies') != SOURCES:
                raise ValueError('UnreviewedCleanupHelpers')
            if review.get('preflight_sha256') != file_hash(OUT/preflight_record):
                raise ValueError('UnreviewedCleanupPreflight')
            if read(OUT/preflight_record)['protected_roots'] != roots:
                raise ValueError('ChangedProtectedRootGraph')
            durable_json(OUT/'RECOVERY.json', dict(manifest_sha256=MANIFEST_SHA, raw_sha256=RAW_SHA,
                compressed_manifest=str(HANDOFF/'DUPLICATE_CANDIDATES.json.gz'),
                authorization=read(OUT/'INITIAL.json')['authorization'],
                scope='Exact ordinary duplicate files only. Restore retired paths before historical audits AND analysis regeneration. Byte restoration does not restore old file IDs/tokens or execution provenance.',
                predicted_allocation=result['predicted_allocation']))
            try:
                for number, row in enumerate(targets):
                    result['current_index'] = number
                    result['current_target'] = row['path']
                    with deletion_handle(row['path']) as handle:
                        same_metadata(metadata(row['path']), row)
                        if handle_hash(handle) != row['sha256']:
                            raise ValueError('MutationBeforeCleanup', row['path'])
                        result['deletion_started'] = True
                        durable_json(OUT/'PENDING.json', dict(index=number, target=row['path'], sha256=row['sha256']))
                        delete_held(handle)
                        result['deletion_performed'] = True
                    if Path(row['path']).exists():
                        raise ValueError('CleanupDeleteIncomplete', row['path'])
                    result['removed'] += 1; result['logical_bytes'] += row['bytes']
                    result['retired_allocation'] += row['cluster_bytes']
                    if number % 500 == 0:
                        durable_json(OUT/'PROGRESS.json', result)
                        print('Removed', result['removed'], '/', len(targets), flush=True)
                result['deletion_performed'] = True
                if any(access.token(p) != row['token'] for p, row in protected.items()):
                    raise ValueError('ChangedProtectedDependencyAfterCleanup')
                result['survivors_verified_under_held_guards'] = True
            except Exception as error:
                result['failure'] = repr(error)
                durable_json(OUT/'STOP.json', result)
                raise
    import psutil
    result.update(elapsed_seconds=time.perf_counter()-started,
                  peak_process_working_set=psutil.Process().memory_info().peak_wset,
                  final_disk=disk())
    durable_json(OUT/name, result)
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('preflight', 'apply'))
    parser.add_argument('--extra-reference', action='append', default=[], help='JSON file with an explicit sealed directory/root reference')
    parser.add_argument('--preflight-record', default='PREFLIGHT_V2.json')
    args = parser.parse_args()
    execute(args.action == 'apply', [read(p) for p in args.extra_reference], args.preflight_record)
