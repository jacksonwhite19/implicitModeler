"""Bounded call-count diagnostics of existing codecs, not a layer benchmark.

Read retained comparison fixtures. Write only new private diagnostic components.
No scientific acceptance, checkpoint, trajectory or proof is produced here.
"""
import argparse
from collections import Counter
from contextlib import ExitStack, closing, contextmanager
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import time
from unittest.mock import patch
import zlib

V4 = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
import immutable_state_access as access
import single_quality_component as quality
import compact_front_atlas as atlas
from path_preserving_deduplication import durable_json, file_hash


@contextmanager
def count_work():
    """Count calls without changing their arguments, return values or checks.

    Single-process diagnostic only. No production installer is patched.
    File-hash byte counts are requested full-file hash bytes, not disk telemetry.
    """
    counts = Counter()
    def wrapper(name, operation):
        def call(*args, **kwargs):
            counts[name] += 1
            if name.endswith('.file_hash'):
                counts['requested_file_hash_bytes'] += Path(args[0]).stat().st_size
            return operation(*args, **kwargs)
        return call
    with ExitStack() as held:
        for module, names, prefix in (
            (quality, ('hashes', '_verify', '_verify_encoded', 'file_hash'), 'quality'),
            (atlas, ('digest', 'file_hash'), 'atlas'),
            (access, ('token', 'file_hash'), 'access'),
            (zlib, ('compress', 'decompress'), 'zlib'),
        ):
            for name in names:
                held.enter_context(patch.object(module, name, wrapper(prefix+'.'+name, getattr(module, name))))
        yield counts


def map_prefix(path, count=16, byte_limit=262144):
    """Take bounded original map entries; never read or rewrite the whole map."""
    with Path(path).open('rb') as stream:
        text = stream.read(byte_limit).decode('utf-8')
    decoder = json.JSONDecoder(); offset = 0; values = {}
    def skip():
        nonlocal offset
        while offset < len(text) and text[offset].isspace(): offset += 1
    skip()
    if text[offset:offset+1] != '{': raise ValueError('ExpectedRetainedAtlasMap')
    offset += 1
    for _ in range(count):
        skip()
        key, offset = decoder.raw_decode(text, offset)
        skip()
        if text[offset:offset+1] != ':': raise ValueError('InvalidRetainedAtlasFraming')
        offset += 1; skip()
        value, offset = decoder.raw_decode(text, offset)
        if not isinstance(key, str) or key in values: raise ValueError('DuplicateRetainedAtlasKey')
        values[key] = value
        skip()
        if text[offset:offset+1] == '}': break
        if text[offset:offset+1] != ',': raise ValueError('InvalidRetainedAtlasFraming')
        offset += 1
    # Reuse the actual supported-row validator and logical digest.
    atlas.digest(atlas.AtlasRows(values))
    return values


def measure(operation):
    started = time.perf_counter()
    with count_work() as counts: value = operation()
    return value, dict(seconds=time.perf_counter()-started, calls=dict(counts))


def probe(directory, rows, values):
    """Real codec operations on bounded retained records, with exact comparisons."""
    directory = Path(directory).resolve()
    directory.mkdir(exist_ok=False)
    phases = {}; path = directory/'QUALITY_DELTA.json'
    _, phases['quality_write'] = measure(lambda: quality.write(path, rows))
    for retained in (False, True):
        label = 'witness' if retained else 'cold'
        started = time.perf_counter()
        with count_work() as counts, quality.snapshot(path, retain_verified=retained) as snapshot:
            phases[label+'_snapshot'] = dict(seconds=time.perf_counter()-started, calls=dict(counts))
            if list(snapshot['rows']) != rows: raise ValueError('QualityDiagnosticMismatch')
            _, phases[label+'_two_readbacks'] = measure(lambda: (snapshot['rows'].verify_readback(), snapshot['rows'].verify_readback()))
    path = directory/'NEW_ATLAS.json'
    _, phases['atlas_write'] = measure(lambda: atlas.write(path, values))
    started = time.perf_counter()
    with count_work() as counts, atlas.snapshot(path) as snapshot:
        phases['atlas_snapshot'] = dict(seconds=time.perf_counter()-started, calls=dict(counts))
        queries = Counter()
        snapshot['view'].rows._db.set_trace_callback(lambda sql: queries.update(['select']) if sql.lstrip().upper().startswith('SELECT') else None)
        point, phases['atlas_point_traversal'] = measure(lambda: dict(snapshot['view']))
        phases['atlas_point_traversal']['select_statements'] = queries['select']
        queries.clear()
        bulk, phases['atlas_existing_ordered_traversal'] = measure(lambda: {row['key']:row['payload'] for row in snapshot['view'].rows})
        phases['atlas_existing_ordered_traversal']['select_statements'] = queries['select']
        snapshot['view'].rows._db.set_trace_callback(None)
        if point != values or bulk != values: raise ValueError('AtlasDiagnosticMismatch')
    return dict(passed=True, quality_rows=len(rows), quality_payloads=sum(r['payload'] is not None for r in rows),
                atlas_rows=len(values), phases=phases, exact_record_comparisons=True,
                scope='Bounded actual codec call counts. Not full L6 runtime, production authentication, scientific replay or qualification.')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); started = time.perf_counter()
    if args.output.exists(): raise ValueError('PreserveExistingDiagnosticOutput')
    resource = json.loads((V4/'results/l6_efficiency_benchmark/BASELINE.json').read_text(encoding='utf-8'))['resources']
    if shutil.disk_usage(V4).free < resource['minimum_free_bytes']: raise ValueError('InsufficientDiagnosticSpace')
    capsule = V4/'results/l6_efficiency_benchmark/capsule_control/capsule_control_verified/current_legacy'
    capsule_evidence = json.loads((V4/'results/l6_efficiency_benchmark/CAPSULE_VERIFIED.json').read_text(encoding='utf-8'))
    source = capsule/'objects'/(capsule_evidence['database_sha256']+'.sqlite')
    l6 = V4/'results/wing_continuation_layers6to8/A/L6'
    with ExitStack() as held:
        for p in (source, l6/'CANDIDATE_PLAN.json', l6/'NEW_ATLAS.json'): held.enter_context(access.guard(p))
        if file_hash(source) != capsule_evidence['database_sha256']: raise ValueError('ChangedCapsuleFixture')
        plan = json.loads((l6/'CANDIDATE_PLAN.json').read_text(encoding='utf-8'))
        ref = plan['references'][str((l6/'NEW_ATLAS.json').resolve())]
        if access.token(l6/'NEW_ATLAS.json') != ref['token'] or file_hash(l6/'NEW_ATLAS.json') != ref['sha256']:
            raise ValueError('ChangedRetainedAtlas')
        with closing(sqlite3.connect(source.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
            rows = [dict(key=k, inputs=json.loads(zlib.decompress(i)), payload=None if p is None else json.loads(zlib.decompress(p)))
                    for k,i,p in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key')]
        values = map_prefix(l6/'NEW_ATLAS.json')
        result = probe(args.output, rows, values)
        result.update(inputs={str(source):capsule_evidence['database_sha256'], str(l6/'NEW_ATLAS.json'):ref['sha256']},
                      workers=1, scope_note='One worker to count codec calls in this process. Both full L6 benchmark controls used four workers.',
                      implementation={str(Path(m.__file__).resolve()):file_hash(m.__file__) for m in (quality, atlas, access)},
                      seconds=time.perf_counter()-started, final_C=shutil.disk_usage(V4).free,
                      final_Z=shutil.disk_usage('Z:/').free)
    import psutil
    result['peak_main_working_set'] = psutil.Process().memory_info().peak_wset
    durable_json(args.output/'RESULT.json', result)
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__': main()
