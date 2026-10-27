"""Read-only, scoped storage census. This tool never deletes or coalesces files."""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import os
import json
from pathlib import Path
import subprocess
import sys
import time

from path_preserving_deduplication import durable_json, metadata, physical_key, file_hash
from record_receipt_resume import ROOT, V4, disk, sha, status

OUT = V4 / 'results/storage_cleanup_handoff'


def baseline():
    previous = __import__('json').loads(
        (V4 / 'results/private_key_comparison/INITIAL.json').read_text(encoding='utf-8'))
    paths = sorted(previous['protected'])
    record = dict(utc=datetime.now(timezone.utc).isoformat(),
                  head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  branch=subprocess.check_output(['git', 'branch', '--show-current'], cwd=ROOT, text=True).strip(),
                  status=status(), protected={p: sha(ROOT / p) for p in paths}, disk=disk())
    if (OUT / 'INITIAL.json').exists():
        raise ValueError('PreserveInitialCleanupRecord')
    OUT.mkdir(parents=True, exist_ok=True)
    durable_json(OUT / 'INITIAL.json', record)
    print(record['head'], len(record['status']), record['disk'], flush=True)


def census():
    started = time.perf_counter()
    root = V4 / 'results'
    groups = defaultdict(lambda: dict(files=0, logical_bytes=0, measured_files=0,
                                     measured_logical_bytes=0, measured_path_allocation=0))
    large, skipped, failures = [], [], []
    # This is a reporting threshold, not a deletion or scientific threshold.
    threshold = 16 * 1024 * 1024
    stack = [root]
    while stack:
        directory = stack.pop()
        with os.scandir(directory) as entries:
            for entry in entries:
                st = entry.stat(follow_symlinks=False)
                if getattr(st, 'st_file_attributes', 0) & 0x400 or entry.is_symlink():
                    skipped.append(str(Path(entry.path).relative_to(V4)))
                    continue
                if entry.is_dir(follow_symlinks=False):
                    stack.append(Path(entry.path))
                elif entry.is_file(follow_symlinks=False):
                    path = Path(entry.path)
                    relative = path.relative_to(root)
                    group = relative.parts[0]
                    g = groups[group]
                    g['files'] += 1
                    g['logical_bytes'] += st.st_size
                    if st.st_size >= threshold:
                        try:
                            row = metadata(path)
                        except OSError as error:
                            failures.append(dict(path=str(path), error=str(error)))
                            continue
                        row.update(relative=relative.as_posix(), group=group)
                        large.append(row)
                        g['measured_files'] += 1
                        g['measured_logical_bytes'] += row['bytes']
                        g['measured_path_allocation'] += row['cluster_bytes']
    unique = {physical_key(r): r for r in large}
    aliases = defaultdict(list)
    for row in large:
        aliases[physical_key(row)].append(row)
    sharing = []
    for key, rows in aliases.items():
        if len(rows) > 1 or rows[0]['links'] > 1:
            sharing.append(dict(paths=[r['relative'] for r in rows],
                                links=rows[0]['links'], measured_aliases=len(rows),
                                logical_bytes_each=rows[0]['bytes'],
                                unique_allocation=rows[0]['cluster_bytes']))
    for name, g in groups.items():
        group_unique = {physical_key(r): r for r in large if r['group'] == name}
        g['measured_unique_allocation'] = sum(r['cluster_bytes'] for r in group_unique.values())
        g['unmeasured_small_logical_bytes'] = g['logical_bytes'] - g['measured_logical_bytes']
    result = dict(scope=str(root), no_deletions=True,
                  allocation_scope='Files at least 16 MiB only; small files logical-only. Cross-group shared allocations must not be summed.',
                  measurement_threshold_bytes=threshold, groups=dict(sorted(groups.items(),
                      key=lambda item: item[1]['logical_bytes'], reverse=True)),
                  logical_bytes=sum(g['logical_bytes'] for g in groups.values()),
                  files=sum(g['files'] for g in groups.values()),
                  measured_unique_allocation=sum(r['cluster_bytes'] for r in unique.values()),
                  measured_path_allocation=sum(r['cluster_bytes'] for r in large),
                  large_files=sorted(large, key=lambda r: r['bytes'], reverse=True),
                  shared_files=sharing, skipped_reparse_points=skipped, failures=failures,
                  elapsed_seconds=time.perf_counter() - started, disk=disk())
    if (OUT / 'CENSUS.json').exists():
        raise ValueError('PreserveCleanupCensus')
    durable_json(OUT / 'CENSUS.json', result)
    print('Total logical:', result['logical_bytes'], 'large-file physical:', result['measured_unique_allocation'])
    for name, g in list(result['groups'].items())[:25]:
        print(name, g)


def duplicate_candidates():
    """Verify named repeat/readback roles; never infer retirement from age."""
    started = time.perf_counter()
    checks, candidates, failures = {}, [], []
    index_root = V4 / 'results/complete_mesh_numerical_qualification'
    index_path = index_root / 'ARTIFACT_INDEX.json'
    index = json.loads(index_path.read_text(encoding='utf-8'))
    outer = V4 / 'results/capsule_sa_outer_map'
    manifest_path = outer / 'ARRAY_MANIFEST.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest_files = {Path(k).as_posix(): v for k, v in manifest['files'].items()}
    if len(manifest_files) != len(manifest['files']):
        raise ValueError('AmbiguousManifestPathAliases')
    pairs = []
    for relative, record in sorted(index.items()):
        if relative.startswith(('runs/A/handoff_readback/', 'runs/A2/handoff_readback/')):
            counterpart = relative.replace('/handoff_readback/', '/handoff/')
            if counterpart in index and record == index[counterpart]:
                pairs.append((index_root / relative, index_root / counterpart,
                              record['sha256'], 'Repeated independent numerical handoff readback', index_path))
    for relative, expected in sorted(manifest_files.items()):
        path = Path(relative)
        if len(path.parts) != 5 or path.parts[0] != 'outputs':
            continue
        mode, name = path.parts[2], path.name
        if any(s in mode for s in ('rejected', 'initial')) or mode == 'reference':
            continue
        if name.startswith('base1_'):
            counterpart = path.with_name('base0_' + name[len('base1_'):]).as_posix()
        elif mode not in ('base', 'zero') and name.startswith('d3_' if mode.endswith('_fine') else 'd4_'):
            prefix = 'd2_' if mode.endswith('_fine') else 'd3_'
            counterpart = path.with_name(prefix + name[3:]).as_posix()
        else:
            continue
        if manifest_files.get(counterpart) == expected:
            pairs.append((outer / relative, outer / counterpart, expected,
                          'Completed map replay output; primary arrays stay', manifest_path))
    selected_paths = {str(p.resolve()) for p, *_ in pairs}
    for number, (target, canonical, expected, reason, origin) in enumerate(pairs):
        if str(canonical.resolve()) in selected_paths:
            raise ValueError('CanonicalAlsoSelected')
        if not target.exists() or not canonical.exists():
            failures.append(dict(target=str(target), canonical=str(canonical), reason='MissingPair'))
            continue
        for p in (target, canonical):
            key = str(p.resolve())
            if key not in checks:
                checks[key] = dict(metadata(p), sha256=file_hash(p))
        a, b = checks[str(target.resolve())], checks[str(canonical.resolve())]
        if a['sha256'] != expected or b['sha256'] != expected or a['bytes'] != b['bytes']:
            failures.append(dict(target=str(target), canonical=str(canonical), reason='LiveHashMismatch'))
            continue
        candidates.append(dict(target=a, canonical=b, expected_sha256=expected,
                               reason=reason, evidence_manifest=str(origin),
                               status='REVIEW_REQUIRED',
                               restore='Copy canonical to original target; original historical index remains unchanged.',
                               historical_reader_effect='Restore target before rerunning any full historical file-path audit.',
                               predicted_reclaim_bytes=a['cluster_bytes'] if a['links'] == 1 else 0))
        if number % 500 == 0:
            print('Verified duplicate pairs:', number, '/', len(pairs), flush=True)
    # The published restart stream is the ordered concatenation of retained parts.
    archive_record = json.loads((index_root / 'RESTART_ARCHIVE.json').read_text(encoding='utf-8'))
    archive = index_root / 'NUMERICAL_RESTART.tar.zst'
    import hashlib
    joined = hashlib.sha256()
    parts = []
    for part in archive_record['parts']:
        p = index_root / part['path']
        if p.stat().st_size != part['bytes'] or file_hash(p) != part['sha256']:
            raise ValueError('ChangedArchivePart')
        with p.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                joined.update(block)
        parts.append(dict(part, path=str(p), relative=part['path']))
    archive_meta = dict(metadata(archive), sha256=file_hash(archive))
    if joined.hexdigest() != archive_record['sha256'] or archive_meta['sha256'] != joined.hexdigest():
        raise ValueError('ArchivePartJoinMismatch')
    result = dict(no_deletions=True, candidates=candidates, failures=failures,
                  manifest_hashes={str(p): file_hash(p) for p in (index_path, manifest_path, index_root / 'RESTART_ARCHIVE.json')},
                  reconstructible_archive=dict(target=archive_meta, parts=parts,
                      status='REVIEW_REQUIRED', expected_sha256=joined.hexdigest(),
                      recipe='Stream parts in exact recorded order into a new file; verify complete hash before use.'),
                  files=len(candidates), logical_bytes=sum(r['target']['bytes'] for r in candidates),
                  predicted_reclaim_bytes=sum(r['predicted_reclaim_bytes'] for r in candidates),
                  elapsed_seconds=time.perf_counter() - started, disk=disk())
    if (OUT / 'DUPLICATE_CANDIDATES.json').exists():
        raise ValueError('PreserveCandidateEvidence')
    durable_json(OUT / 'DUPLICATE_CANDIDATES.json', result)
    print('Pairs', len(candidates), 'logical', result['logical_bytes'],
          'potential physical', result['predicted_reclaim_bytes'], 'failures', len(failures))


def package():
    """Compact handoff copies, with byte-exact decompression checks. No removal."""
    import gzip
    from collections import Counter
    sys.path.insert(0, str(V4 / 'src'))
    from sdf_cfd_mesher_v4.common.authority import identity
    raw_path = OUT / 'DUPLICATE_CANDIDATES.json'
    record = json.loads(raw_path.read_text(encoding='utf-8'))
    counts, logical, allocated = Counter(), Counter(), Counter()
    for row in record['candidates']:
        key = Path(row['target']['path']).relative_to(V4 / 'results').parts[0]
        counts[key] += 1
        logical[key] += row['target']['bytes']
        allocated[key] += row['predicted_reclaim_bytes']
    origin_pins = {}
    for folder, authority_name, manifest_name in (
            ('capsule_sa_outer_map', 'RESULT.json', 'ARRAY_MANIFEST.json'),
            ('complete_mesh_numerical_qualification', 'AUTHORITY.json', 'ARTIFACT_INDEX.json')):
        root = V4 / 'results' / folder
        authority_path = root / authority_name
        authority = json.loads(authority_path.read_text(encoding='utf-8'))
        content = {k: v for k, v in authority.items() if k != 'identity'}
        if 'identity' in authority and identity(content) != authority['identity']:
            raise ValueError('ChangedStudyAuthority:' + folder)
        # Some older authorities are Git-pinned records, not self-sealed schemas.
        relative = authority_path.relative_to(ROOT).as_posix()
        committed = subprocess.check_output(['git', 'show', 'HEAD:' + relative], cwd=ROOT)
        if committed != authority_path.read_bytes():
            raise ValueError('ChangedCommittedStudyAuthority:' + folder)
        if file_hash(root / manifest_name) != authority['evidence'][manifest_name]:
            raise ValueError('ChangedStudyManifest:' + folder)
        origin_pins[folder] = dict(authority_path=str(authority_path),
                                  sha256=file_hash(authority_path), identity=authority.get('identity'),
                                  committed_bytes_verified=True,
                                  classification=authority['classification'])
    compressed = {}
    for name in ('CENSUS.json', 'DUPLICATE_CANDIDATES.json'):
        p = OUT / name
        raw = p.read_bytes()
        encoded = gzip.compress(raw, compresslevel=6, mtime=0)
        target = p.with_suffix('.json.gz')
        with target.open('xb') as stream:
            stream.write(encoded)
        if gzip.decompress(target.read_bytes()) != raw:
            raise ValueError('HandoffCompressionMismatch')
        compressed[name] = dict(path=str(target), bytes=len(encoded),
                                sha256=file_hash(target), uncompressed_sha256=file_hash(p))
    archive = record['reconstructible_archive']['target']
    result = dict(classification='V4PhysicalCleanupHandoffVerifiedDuplicateCandidates',
                  deletion_performed=False, cleanup_authorized_here=False,
                  execution_status='Await independent review and live dependency/identity checks; not an unconditional deletion list.',
                  by_group={k: dict(files=counts[k], logical_bytes=logical[k],
                                    potential_reclaim_bytes=allocated[k]) for k in counts},
                  archive=dict(path=archive['path'], logical_bytes=archive['bytes'],
                               potential_reclaim_bytes=archive['cluster_bytes'] if archive['links'] == 1 else 0,
                               sha256=archive['sha256'], retained_parts=6),
                  total_candidate_files=record['files'] + 1,
                  total_logical_bytes=record['logical_bytes'] + archive['bytes'],
                  total_potential_reclaim_bytes=record['predicted_reclaim_bytes'] + archive['cluster_bytes'],
                  failures=record['failures'], origin_pins=origin_pins, compressed=compressed,
                  read_effect='Retired duplicate paths are no longer directly readable; restore exact bytes from named canonicals before full historical audits.',
                  candidate_scan_seconds=record['elapsed_seconds'],
                  scientific_scope='No new qualification. Existing results and identities stay unchanged.', disk=disk())
    if (OUT / 'HANDOFF.json').exists():
        raise ValueError('PreserveHandoffSummary')
    durable_json(OUT / 'HANDOFF.json', result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('baseline', 'census', 'duplicates', 'package'))
    action = parser.parse_args().action
    {'baseline': baseline, 'census': census, 'duplicates': duplicate_candidates, 'package': package}[action]()
