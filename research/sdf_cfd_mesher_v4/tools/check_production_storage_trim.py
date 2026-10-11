"""Bounded retained-storage accounting. Never generates or alters a mesh."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
OUT = V4 / 'results/production_storage_trim'
sys.path[:0] = [str(V4 / 'tools'), str(V4 / 'src')]
OWNED = {'research/sdf_cfd_mesher_v4/CURRENT_TASK.md',
         'research/sdf_cfd_mesher_v4/README.md',
         'research/sdf_cfd_mesher_v4/PRODUCTION_STORAGE_TRIM_REPORT.md',
         'research/sdf_cfd_mesher_v4/tools/check_production_storage_trim.py',
         'research/sdf_cfd_mesher_v4/tools/quality_storage_layout.py',
         'research/sdf_cfd_mesher_v4/tools/assembly_quality_handoff.py',
         'research/sdf_cfd_mesher_v4/tools/prism_sql_profile.py',
         'research/sdf_cfd_mesher_v4/tests/test_generic_production_alignment.py',
         'research/sdf_cfd_mesher_v4/tests/test_quality_storage_layout.py',
         'research/sdf_cfd_mesher_v4/tools/production_prism_stack.py'}


def status():
    return subprocess.check_output(['git', '-c', 'core.quotepath=false', 'status',
        '--porcelain=v1', '--untracked-files=normal'], cwd=ROOT).decode('utf-8').splitlines()


def inventory():
    from collect_live_sf2_first_layer import allocation
    seen = set(); records = []; categories = defaultdict(lambda: dict(paths=0,logical=0,unique_logical=0,allocated=0))
    for run in ('fresh_cli', 'fresh_cli_L10_resume35'):
        folder = V4 / 'results/integrated_cli_headroom' / run
        for p in sorted(folder.rglob('*')):
            if not p.is_file(): continue
            stat = p.stat(); key = (stat.st_dev,stat.st_ino); relative = p.relative_to(folder)
            if 'quality_objects' in relative.parts: category = 'quality_and_atlas_sql'
            elif p.name.endswith('proof-key-index.sqlite'): category = 'proof_key_indices'
            elif p.suffix == '.jsonl': category = 'certificate_and_exact_logs'
            elif p.suffix in ('.npy','.npz'): category = 'numeric_arrays'
            elif p.suffix == '.sqlite': category = 'other_sql'
            else: category = 'json_and_other'
            layer = relative.parts[0] if len(relative.parts)>1 else 'root'
            group = categories[run+'/'+layer+'/'+category]
            group['paths'] += 1; group['logical'] += stat.st_size
            unique = key not in seen; allocated = allocation(p) if unique else 0
            if unique:
                seen.add(key); group['allocated'] += allocated; group['unique_logical'] += stat.st_size
            records.append(dict(path=p.relative_to(ROOT).as_posix(),bytes=stat.st_size,
                unique_allocation=allocated,physical_key=list(key),category=category))
    return dict(categories=dict(categories),records=records,
        logical=sum(r['bytes'] for r in records),unique_allocation=sum(r['unique_allocation'] for r in records),
        scope='Two fresh standalone run directories only. NTFS file identity deduplicated. No directory/MFT/Git bytes.')


def baseline():
    from path_preserving_deduplication import durable_json,file_hash
    if OUT.exists(): raise ValueError('PreserveExistingStorageTrimEvidence')
    files = [V4/'tools'/n for n in ('single_quality_component.py','compact_front_atlas.py',
        'critical_quality_lookup.py','critical_transaction.py','critical_assembly.py','incremental_pair_storage.py',
        'composable_state.py','standalone_tool.py')]
    files += [ROOT/'src/sdf/aerospace'/n for n in ('wing.rs','airfoil.rs','naca_spline.rs')]
    files += [V4/'runtime/RELEASE.json',V4/'results/integrated_cli_headroom/RESERVE35_RESULT.json',
        V4/'results/integrated_cli_headroom/fresh_cli_L10_resume35/RESTART.json']
    record = dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        status=[r for r in status() if r[3:] not in OWNED and not r[3:].startswith('research/sdf_cfd_mesher_v4/results/production_storage_trim/')],
        files=[dict(path=p.relative_to(ROOT).as_posix(),sha256=file_hash(p),bytes=p.stat().st_size) for p in files],
        initial_C=shutil.disk_usage(ROOT).free,initial_Z=shutil.disk_usage('Z:/').free,inventory=inventory())
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',record)
    print(json.dumps(dict(head=record['head'],status_rows=len(record['status']),initial_C=record['initial_C'],
        initial_Z=record['initial_Z'],unique_allocation=record['inventory']['unique_allocation']),indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline','layout','atlas','full','full_atlas'])
    parser.add_argument('--output-root',type=Path,default=OUT)
    args=parser.parse_args();OUT=args.output_root.resolve()
    if OUT==V4/'results' or not OUT.is_relative_to((V4/'results').resolve()):
        raise ValueError('StorageControlOutputOutsideResults')
    if args.mode=='baseline':baseline()
    elif args.mode in ('full','full_atlas'):
        from contextlib import closing
        import sqlite3
        import psutil
        import zlib
        import single_quality_component as q
        from quality_storage_layout import installed
        from path_preserving_deduplication import file_hash,durable_json
        from collect_live_sf2_first_layer import allocation
        is_atlas=args.mode=='full_atlas'
        source=V4/'results/integrated_cli_headroom/fresh_cli/L3'/('atlas_rows.json' if is_atlas else 'QUALITY_DELTA.json')
        body=json.loads(source.read_text(encoding='utf-8'));payload=Path(body['payload']['path'])
        folder=OUT/('full_atlas' if is_atlas else 'full_quality');folder.mkdir(exist_ok=False)
        started=time.perf_counter();times={}
        def resources():
            if shutil.disk_usage(ROOT).free<35_000_000_000:raise ValueError('ExistingOperationalReserveBreach')
        class Rows:
            def __iter__(self):
                with closing(sqlite3.connect(payload.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
                    for k,i,p in db.execute('SELECT key,inputs,payload FROM ordering JOIN updates USING(key) ORDER BY ordinal'):
                        yield dict(key=k,inputs=json.loads(zlib.decompress(i)),payload=None if p is None else json.loads(zlib.decompress(p)))
        with q.access.guard(source),q.access.guard(payload):
            source_hash=file_hash(source)
            if file_hash(payload)!=body['payload']['sha256'] or q.access.token(payload)!=body['payload']['token']:
                raise ValueError('ChangedRetainedQualitySource')
            tick=time.perf_counter()
            with installed(16384 if is_atlas else 8192) as layout:q.write(folder/'QUALITY_DELTA.json',Rows(),progress=resources)
            times['existing_writer']=time.perf_counter()-tick
            generated=json.loads((folder/'QUALITY_DELTA.json').read_text(encoding='utf-8'))
            if generated['logical']!=body['logical']:raise ValueError('FullQualityLogicalIdentityMismatch')
            tick=time.perf_counter()
            with q.snapshot(folder/'QUALITY_DELTA.json',retain_verified=True) as snap:
                if q.quality_identity(snap['rows'])!=body['logical']['identity']:raise ValueError('FullQualityIdentityMismatch')
                snap['rows'].verify_readback()
                if is_atlas:
                    import compact_front_atlas as atlas
                    actual=atlas.digest(snap['rows'])
                    retained=json.loads((source.parent/'NEW_ATLAS.json').read_text(encoding='utf-8'))['logical']
                    if actual!=retained:raise ValueError('FullAtlasMapIdentityMismatch')
                    view=atlas.AtlasView(snap['rows'])
                    with closing(sqlite3.connect(payload.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
                        for key,value in db.execute('SELECT key,payload FROM updates ORDER BY key LIMIT 32'):
                            if view[key]!=json.loads(zlib.decompress(value)):raise ValueError('FullAtlasIndexedReaderMismatch')
                with closing(sqlite3.connect(payload.as_uri()+'?mode=ro&immutable=1',uri=True)) as a,closing(sqlite3.connect(snap['rows'].path.as_uri()+'?mode=ro&immutable=1',uri=True)) as b:
                    count=0
                    for old,new in zip(a.execute('SELECT key,inputs,payload FROM updates ORDER BY key'),b.execute('SELECT key,inputs,payload FROM updates ORDER BY key'),strict=True):
                        if old!=new:raise ValueError('FullQualityBlobMismatch')
                        count+=1
                    if count!=body['logical']['count']:raise ValueError('FullQualityCoverageMismatch')
                    ordering_equal=all(old==new for old,new in zip(a.execute('SELECT ordinal,key FROM ordering ORDER BY ordinal'),b.execute('SELECT ordinal,key FROM ordering ORDER BY ordinal'),strict=True))
                    if not ordering_equal:raise ValueError('FullQualityOrderingMismatch')
                target=snap['rows'].path
            times['independent_reader_and_exact_comparison']=time.perf_counter()-tick
            if file_hash(source)!=source_hash or q.access.token(payload)!=body['payload']['token']:
                raise ValueError('OriginalFullQualityChanged')
        report=dict(passed=True,source=source.relative_to(ROOT).as_posix(),source_sha256=source_hash,
            original_payload_sha256=body['payload']['sha256'],output_payload_sha256=file_hash(target),
            original_logical_bytes=payload.stat().st_size,output_logical_bytes=target.stat().st_size,
            original_allocation=allocation(payload),output_allocation=allocation(target),rows=count,logical=generated['logical'],
            all_encoded_blobs_equal=True,all_ordering_equal=ordering_equal,original_unchanged=True,
            layout=dict(layout),timings=times,seconds=time.perf_counter()-started,
            peak_process_rss=psutil.Process().memory_info().peak_wset,
            scope='Full retained L3 quality storage; existing writer, guarded cold reader and independent encoded readback. No generated geometry/proofs or committed child.')
        report['scope']=('Full retained L3 atlas storage; existing codec, map digest and indexed AtlasView. No committed child.' if is_atlas else report['scope'])
        durable_json(OUT/('FULL_ATLAS.json' if is_atlas else 'FULL_QUALITY.json'),report);print(json.dumps(report,indent=2))
    else:
        from contextlib import closing
        import sqlite3
        import psutil
        import single_quality_component as q
        from quality_storage_layout import installed
        from path_preserving_deduplication import file_hash,durable_json
        from collect_live_sf2_first_layer import allocation
        is_atlas=args.mode=='atlas'
        source=V4/'results/integrated_cli_headroom/fresh_cli/L3'/('atlas_rows.json' if is_atlas else 'QUALITY_DELTA.json')
        body=json.loads(source.read_text(encoding='utf-8'));payload=Path(body['payload']['path'])
        start=time.perf_counter();records=[];selected=[];populations={}
        with q.access.guard(source),q.access.guard(payload):
            pin=file_hash(source);digest=file_hash(payload)
            if digest!=body['payload']['sha256'] or q.access.token(payload)!=body['payload']['token']:
                raise ValueError('ChangedRetainedQualitySource')
            with closing(sqlite3.connect(payload.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
                if is_atlas:
                    selected=list(db.execute('SELECT key,inputs,payload FROM updates ORDER BY key LIMIT 8192'))
                    populations['atlas']=len(selected)
                for prefix in (() if is_atlas else ('cell:','face:','final_incidence:')):
                    for null in (True,False):
                        query='SELECT key,inputs,payload FROM updates WHERE key>=? AND key<? AND payload IS '+('NULL' if null else 'NOT NULL')+' ORDER BY key LIMIT 4096'
                        values=list(db.execute(query,(prefix,prefix+'\uffff')))
                        populations[prefix+('input_only' if null else 'payload')]=len(values)
                        selected.extend(values)
            selected.sort(key=lambda row:row[0])
            if len({r[0] for r in selected})!=len(selected):raise ValueError('DuplicateDiagnosticSelection')
            import zlib
            rows=[dict(key=k,inputs=json.loads(zlib.decompress(i)),payload=None if p is None else json.loads(zlib.decompress(p))) for k,i,p in selected]
            expected=q.hashes(iter(rows))
            for size in (4096,8192,16384,32768,65536):
                folder=OUT/(('atlas_page_' if is_atlas else 'page_')+str(size));folder.mkdir(exist_ok=False)
                if shutil.disk_usage(ROOT).free<35_000_000_000:raise ValueError('ExistingOperationalReserveBreach')
                tick=time.perf_counter()
                with installed(size) as stats:q.write(folder/'QUALITY_DELTA.json',rows)
                elapsed=time.perf_counter()-tick
                with q.snapshot(folder/'QUALITY_DELTA.json') as snap:
                    if list(snap['rows'])!=rows or snap['rows'].body['logical']!=expected:
                        raise ValueError('LayoutLogicalMismatch')
                    with closing(sqlite3.connect(snap['rows'].path.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
                        encoded=list(db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'))
                        if encoded!=selected:raise ValueError('LayoutEncodedBlobMismatch')
                        actual=db.execute('PRAGMA page_size').fetchone()[0]
                    p=snap['rows'].path
                    records.append(dict(page_size=actual,logical_bytes=p.stat().st_size,allocation=allocation(p),
                        seconds=elapsed,payload_sha256=file_hash(p),component_sha256=file_hash(folder/'QUALITY_DELTA.json'),
                        all_rows_equal=True,all_encoded_blobs_equal=True,writer=dict(stats)))
            if file_hash(source)!=pin or q.access.token(payload)!=body['payload']['token']:
                raise ValueError('RetainedSourceChangedDuringLayoutControl')
        evidence=dict(source=str(source.relative_to(ROOT)),source_sha256=pin,payload_sha256=digest,
            source_rows=body['logical']['count'],selected_rows=len(rows),populations=populations,logical=expected,
            layouts=records,seconds=time.perf_counter()-start,peak_process_rss=psutil.Process().memory_info().peak_wset,
            scope='Bounded retained row populations. No mesh, proof, scientific gate or sealed storage rewritten.',
            original_unchanged=True)
        durable_json(OUT/('ATLAS_LAYOUT.json' if is_atlas else 'LAYOUT.json'),evidence)
        print(json.dumps(evidence,indent=2))
