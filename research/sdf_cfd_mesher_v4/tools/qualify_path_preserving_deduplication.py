"""Audit, bounded hardlink replacement, and fresh-process restore evidence."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
import psutil
from path_preserving_deduplication import (metadata, physical_key, file_hash,
    durable_json, eligible, replace_duplicate)

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
OUT = V4 / 'results/path_preserving_deduplication'
TREES = [V4 / 'results' / s for s in (
    'production_adaptive_capsule_first_layer', 'production_adaptive_capsule_layer2',
    'immutable_receipt_overlay_delta_publication')]
HEAD = '229a64fa112e055607382055f63d46c1d8ba566f'


def save(name, data):
    OUT.mkdir(exist_ok=True)
    durable_json(OUT / name, data)


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def resource():
    m = psutil.Process().memory_info()
    return dict(peak_rss=m.peak_wset, retained_rss=m.rss)


def reference_index():
    refs = defaultdict(list)
    for tree in TREES:
        manifest = tree / 'BULK_MANIFEST.json'
        value = json.loads(manifest.read_text())
        for row in value if isinstance(value, list) else value['files']:
            base = ROOT if row['path'].startswith('research/') else tree
            refs[str((base / row['path']).resolve())].append(
                dict(kind='sealed_bulk_manifest', source=str(manifest), sha256=row['sha256']))
        authority = tree / 'AUTHORITY.json'
        for name, digest in json.loads(authority.read_text()).get('evidence', {}).items():
            refs[str((tree / name).resolve())].append(
                dict(kind='authority', source=str(authority), sha256=digest))
        for checkpoint in tree.rglob('NATIVE_*.json'):
            def walk(value):
                if isinstance(value, dict):
                    if 'file' in value and 'sha256' in value:
                        p = (checkpoint.parent / 'objects' / value['file']).resolve()
                        refs[str(p)].append(dict(kind='checkpoint', source=str(checkpoint), sha256=value['sha256']))
                    for child in value.values():
                        walk(child)
                elif isinstance(value, list):
                    for child in value:
                        walk(child)
            walk(json.loads(checkpoint.read_text()))
    return refs


def inventory():
    refs = reference_index()
    rows, hashed = [], {}
    for tree in TREES:
        for path in sorted(tree.rglob('*')):
            if not path.is_file():
                continue
            row = metadata(path)
            key = physical_key(row)
            if key not in hashed:
                hashed[key] = file_hash(path)
            row['sha256'] = hashed[key]
            row['references'] = refs.get(str(path.resolve()), [])
            for ref in row['references']:
                if row['sha256'] != ref['sha256']:
                    raise ValueError('ReferenceHashMismatch: ' + str(path))
            row['immutable'] = (path.parent.name == 'objects' and path.parent.parent.name == 'checkpoints'
                                and bool(re.fullmatch(r'[a-f0-9]{64}\.(json|sqlite|npy|npz)', path.name))
                                and bool(row['references']))
            row['retention'] = ('A/D: immutable authority/restore object; preserve every path'
                                if row['immutable'] else 'A/B/F: retained evidence; mutability not qualified for sharing')
            rows.append(row)
    paths = {r['path'] for r in rows}
    for path in refs:
        if path not in paths:
            raise ValueError('MissingRequiredReference: ' + path)
    return rows


def totals(rows):
    physical = {}
    content = defaultdict(dict)
    for r in rows:
        physical[physical_key(r)] = r
        content[(r['sha256'], r['bytes'])][physical_key(r)] = r
    duplicate = sum(sum(x['cluster_bytes'] for x in copies.values()) - max(x['cluster_bytes'] for x in copies.values())
                    for copies in content.values())
    return dict(paths=len(rows), physical_objects=len(physical), content_groups=len(content),
        logical_bytes=sum(r['bytes'] for r in rows),
        allocated_path_sum=sum(r['allocated'] for r in rows),
        unique_physical_allocation=sum(r['cluster_bytes'] for r in physical.values()),
        resident_allocation_not_claimed=sum(r['allocated'] for r in physical.values() if not r['cluster_bytes']),
        duplicate_physical_allocation=duplicate,
        hardlinked_logical_bytes=sum(r['bytes'] for r in rows)-sum(r['bytes'] for r in physical.values()))


def logical_scope():
    total = v4 = 0
    errors, excluded = [], []
    def error(exc): errors.append(str(exc))
    for base, dirs, names in os.walk(ROOT, followlinks=False, onerror=error):
        for name in list(dirs):
            path = Path(base) / name
            rel = path.relative_to(ROOT).as_posix()
            if rel in ('.claude/worktrees', 'dual_contouring') or path.is_symlink() or path.is_junction():
                excluded.append(rel)
                dirs.remove(name)
        for name in names:
            p = Path(base) / name
            try:
                size = p.stat().st_size
                total += size
                if p.is_relative_to(V4): v4 += size
            except OSError as exc:
                errors.append(str(exc))
    return dict(repository_accessible_logical=total, v4_accessible_logical=v4,
                inaccessible=errors, excluded_trees=excluded)


def seals():
    import qualify_immutable_overlay_publication as prior
    expected = dict(prior.SEALS)
    expected.update({TREES[2]/'AUTHORITY.json':'8655759d628fbdb71698d65b47dc2cfe7d57e6f47abf101f17d89a2298911e7e',
        TREES[2]/'report.md':'b111ea1eafa65b6fc062d0b3d52d30373d374a400a4f7d5a8f9373806862d25c'})
    for tree in TREES:
        auth = json.loads((tree/'AUTHORITY.json').read_text())
        for name, digest in auth.get('evidence', {}).items(): expected[tree/name] = digest
        if 'manifest_sha256' in auth: expected[tree/'BULK_MANIFEST.json'] = auth['manifest_sha256']
        if 'report_sha256' in auth: expected[tree/'report.md'] = auth['report_sha256']
    result = []
    for path, digest in expected.items():
        actual = file_hash(path)
        assert actual == digest, str(path)
        result.append(dict(path=str(path), sha256=actual))
    return result


def audit():
    assert git('rev-parse', 'HEAD') == HEAD
    assert not (OUT/'BEFORE.json').exists(), 'Do not overwrite initial inventory'
    start = time.perf_counter()
    status = git('status', '--porcelain=v1').splitlines()
    unrelated = [r for r in status if 'research/sdf_cfd_mesher_v4/' not in r]
    assert len(unrelated) == 155
    save('GIT_BEFORE.json', dict(head=HEAD, branch=git('branch','--show-current'), unrelated=unrelated))
    free = shutil.disk_usage(ROOT).free
    protected = seals()
    rows = inventory()
    groups = defaultdict(list)
    for row in rows: groups[(row['sha256'],row['bytes'])].append(row)
    plan, unresolved, group_records = [], [], []
    for (digest,size), group in sorted(groups.items()):
        copies = defaultdict(list)
        for row in group: copies[physical_key(row)].append(row)
        if len(group) > 1:
            group_records.append(dict(sha256=digest, bytes=size, physical_copies=len(copies),
                paths=len(group), existing_shared_aliases=len(group)-len(copies),
                files=[dict(path=r['path'],file_id=r['file_id'],links=r['links'],allocated=r['allocated']) for r in group]))
        if len(copies) < 2: continue
        candidates = [r for r in group if eligible(r)]
        if not candidates:
            unresolved.append(dict(sha256=digest,reason='No qualified immutable canonical',paths=[r['path'] for r in group]))
            continue
        canonical = sorted(candidates, key=lambda r: (TREES.index(next(t for t in TREES if Path(r['path']).is_relative_to(t))),r['path']))[0]
        for key, aliases in copies.items():
            if key == physical_key(canonical): continue
            if len(aliases) != aliases[0]['links'] or not all(eligible(r) for r in aliases):
                unresolved.append(dict(sha256=digest,reason='Unknown external aliases or unqualified mutable role',paths=[r['path'] for r in aliases]))
                continue
            if any((r['security'],r['attributes'],r['streams'],r['device']) !=
                   (canonical['security'],canonical['attributes'],canonical['streams'],canonical['device']) for r in aliases):
                unresolved.append(dict(sha256=digest,reason='Metadata or volume mismatch',paths=[r['path'] for r in aliases]))
                continue
            if not aliases[0]['cluster_bytes']: continue
            plan.append(dict(canonical=canonical,aliases=aliases,reclaimable_allocation=aliases[0]['cluster_bytes']))
    save('BEFORE.json',dict(rows=rows,totals=totals(rows),free_bytes=free,scope=logical_scope(),
        protected=protected,inventory_hash_seconds=time.perf_counter()-start,**resource()))
    save('DUPLICATES.json',group_records)
    save('PLAN.json',dict(groups=plan,unresolved=unresolved,
        replacement_paths=sum(len(g['aliases']) for g in plan),
        physical_reclaimable=sum(g['reclaimable_allocation'] for g in plan)))
    print(json.dumps(dict(totals=totals(rows),replacement_paths=sum(len(g['aliases']) for g in plan),
        physical_reclaimable=sum(g['reclaimable_allocation'] for g in plan),unresolved_groups=len(unresolved)),indent=2),flush=True)


def apply():
    assert not (OUT/'ACTIONS.json').exists(), 'Use journal recovery instead of overwriting completed run'
    before = json.loads((OUT/'BEFORE.json').read_text())
    assert seals() == before['protected']
    plan = json.loads((OUT/'PLAN.json').read_text())
    (OUT/'journals').mkdir(exist_ok=True)
    start=time.perf_counter(); records=[]
    for group in plan['groups']:
        for row in group['aliases']:
            current=metadata(row['path'])
            assert physical_key(current)==physical_key(row), 'StaleGroup'
            assert current['links']==len(group['aliases']), 'UnknownExternalAlias'
        for row in group['aliases']:
            index=len(records)
            result=replace_duplicate(row,group['canonical'],OUT/'journals'/f'{index:04d}.json')
            records.append(result)
            print(index,'verified',row['path'],flush=True)
        assert records[-1]['old_handle_after_replace']['links']==0, 'OldAllocationStillNamed'
    save('ACTIONS.json',dict(records=records,seconds=time.perf_counter()-start,
        standalone_deleted_paths=[],logical_removed=0,logical_shared=sum(r['original']['bytes'] for r in records),**resource()))


def verify():
    start=time.perf_counter()
    before=json.loads((OUT/'BEFORE.json').read_text())
    rows=inventory()
    assert {(r['path'],r['bytes'],r['sha256']) for r in rows} == {(r['path'],r['bytes'],r['sha256']) for r in before['rows']}
    assert seals()==before['protected']
    after=totals(rows)
    plan=json.loads((OUT/'PLAN.json').read_text())
    assert before['totals']['unique_physical_allocation']-after['unique_physical_allocation']==plan['physical_reclaimable']
    for group in plan['groups']:
        canonical=metadata(group['canonical']['path'])
        for row in group['aliases']: assert physical_key(metadata(row['path']))==physical_key(canonical)
    save('AFTER.json',dict(rows=rows,totals=after,free_bytes=shutil.disk_usage(ROOT).free,
        scope=logical_scope(),protected=seals(),all_paths_bytes_unchanged=True,
        verification_seconds=time.perf_counter()-start,**resource()))
    print(json.dumps(after,indent=2),flush=True)


def restore(label):
    import numpy as np
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    from sdf_cfd_mesher_v4.common.authority import array_hash
    paths={f'L{k}_{run}':TREES[k-1]/run/'checkpoints' for k in (1,2) for run in ('A','B')}
    paths.update({name:TREES[2]/name/'checkpoints' for name in ('baseline','candidate_A','candidate_B','candidate_C')})
    path=paths[label];start=time.perf_counter()
    state=NativeCheckpointStore(path).restart()
    expected=('5b036bc4cd5180b9e4a7e3e6aafe57fe21c7b2ae1cae82cf3c42e446b1805fd0' if label.startswith('L1')
              else '707d9726da9f6a6f7dedc688f4437777aeb2e8fa5cf0958ca6aed83f8f7bfa44')
    assert scientific_hash(state)==expected
    layers=2 if label.startswith('L1') else 3
    coords=[array_hash(np.array([state['vertices'][f'layer:{k}:source:{i}'] for i in range(16895)])) for k in range(layers)]
    assert coords==['2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c',
        '34ca5df3c122a7436caf8a475c1c8cf0fbe629a6afb26fe1387862ef2f884cf7',
        'f57dc72c0ee93cb95529539f5986a206ee315ba59ca886eb284408deb86896cd'][:layers]
    pointer=json.loads((path/'CURRENT.json').read_text())
    assert file_hash(path/pointer['checkpoint'])==pointer['sha256']
    save(label+'_RESTORE.json',dict(path=str(path),scientific=expected,coordinates=coords,
        checkpoint=pointer,seconds=time.perf_counter()-start,cells=len(state['cells']),
        vertices=len(state['vertices']),receipts=len(state['certificate']),passed=True,**resource()))
    print(label,expected,'restored',time.perf_counter()-start,flush=True)


def retired_ids():
    from path_preserving_deduplication import probe_file_id
    import tempfile
    # A disposable test file supplies the host's actual retired-ID error code.
    with tempfile.TemporaryDirectory(prefix='v4-retired-id-control-') as directory:
        control=Path(directory)/'control';control.write_bytes(b'file ID control')
        old=metadata(control)['file_id'];assert probe_file_id(old)==0
        control.unlink();retired_error=probe_file_id(old);assert retired_error!=0
    rows=[]
    for r in json.loads((OUT/'ACTIONS.json').read_text())['records']:
        positive=probe_file_id(r['canonical']['file_id']);negative=probe_file_id(r['original']['file_id'])
        assert positive==0 and negative==retired_error
        rows.append(dict(path=r['original']['path'],old_file_id=r['original']['file_id'],
            canonical_file_id=r['canonical']['file_id'],canonical_error=positive,retired_error=negative))
    save('RETIRED_ID_CHECK.json',dict(rows=rows,deleted_control_error=retired_error,
        all_canonicals_open=True,all_retired_ids_match_deleted_control=True))
    print('All 93 retained canonical IDs open; all 93 retired IDs reject with measured deleted-control error',retired_error)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['audit','apply','verify','restore','retired_ids'])
    parser.add_argument('--label');args=parser.parse_args()
    if args.mode=='restore':restore(args.label)
    else:globals()[args.mode]()
