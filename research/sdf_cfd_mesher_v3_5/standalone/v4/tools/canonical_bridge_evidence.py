"""Capture source bytes and unrelated-work identities for this bounded task."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile
import concurrent.futures
import gzip

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'research/sdf_cfd_mesher_v4/results/generic_canonical_bridge'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 << 20), b''):
            h.update(block)
    return h.hexdigest()


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


def source_bundle():
    OUT.mkdir(parents=True, exist_ok=True)
    paths = sorted([*ROOT.joinpath('src').rglob('*.rs'),
                    *ROOT.joinpath('shaders').glob('*.wgsl'),
                    ROOT / 'Cargo.toml', ROOT / 'Cargo.lock'])
    rows = [{'path': p.relative_to(ROOT).as_posix(), 'bytes': p.stat().st_size,
             'sha256': digest(p)} for p in paths]
    identity = hashlib.sha256(json.dumps(rows, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    with zipfile.ZipFile(OUT / 'KERNEL_SOURCE_SNAPSHOT.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for p in paths:
            z.writestr(zipfile.ZipInfo(p.relative_to(ROOT).as_posix(), (2026, 9, 27, 0, 0, 0)),
                       p.read_bytes(), compress_type=zipfile.ZIP_DEFLATED)
    record = dict(source_identity=identity, base_head=git('rev-parse', 'HEAD').decode().strip(),
                  rustc=subprocess.check_output(['rustc', '-Vv']).decode(), files=rows,
                  snapshot_sha256=digest(OUT / 'KERNEL_SOURCE_SNAPSHOT.zip'),
                  scope='Complete root Rust source, embedded shaders, Cargo manifest and lock; no scientific arrays.')
    (OUT / 'KERNEL_SOURCE_MANIFEST.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(dict(source_identity=identity, files=len(rows), bytes=sum(r['bytes'] for r in rows),
                          compressed_bytes=(OUT / 'KERNEL_SOURCE_SNAPSHOT.zip').stat().st_size)))


def preservation(mode):
    path = OUT / 'UNRELATED_BEFORE.json'
    if mode == 'inventory':
        before=json.loads(path.read_text() if path.exists() else gzip.decompress(path.with_suffix('.json.gz').read_bytes()))
        entries=[s for s in git('status','--porcelain=v1','-z').decode().split('\0') if s and not s[3:].startswith('research/sdf_cfd_mesher_v4/')]
        names=set()
        for s in entries:
            p=ROOT/s[3:]
            try:
                is_file,is_dir=p.is_file(),p.is_dir()
            except OSError:
                names.add(s[3:]);continue
            if is_file:names.add(s[3:])
            elif is_dir:
                nested=(p/'.git').exists()
                result=subprocess.run(['git','-C',str(p if nested else ROOT),'ls-files','--cached','--others','--exclude-standard','-z',*([] if nested else ['--',s[3:]])],capture_output=True,check=True)
                names.update(((p if nested else ROOT)/n).relative_to(ROOT).as_posix() for n in result.stdout.decode().split('\0') if n)
        previous={r['path'] for r in before['files']}
        result=dict(status_equal=entries==before['status'],added=sorted(names-previous),removed=sorted(previous-names),paths=len(names),scope=before['scope'])
        (OUT/'UNRELATED_INVENTORY_AFTER.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result));assert result['status_equal'] and not result['added'] and not result['removed']
        return
    if mode == 'before':
        status = git('status', '--porcelain=v1', '-z').decode().split('\0')
        entries = [s for s in status if s and not s[3:].startswith('research/sdf_cfd_mesher_v4/')]
        files = []
        initial_errors = []
        (OUT / 'UNRELATED_STATUS_BEFORE.json').write_text(json.dumps(entries, indent=2)+'\n')
        for s in entries:
            p = ROOT / s[3:]
            try:
                is_file, is_dir = p.is_file(), p.is_dir()
            except OSError as e:
                initial_errors.append(dict(path=s[3:], error=str(e)))
                continue
            if is_file:
                files.append(p)
            elif is_dir:
                # Preserve Git-visible content. Ignored build caches are outside dirty-state identity.
                nested = (p/'.git').exists()
                paths = subprocess.check_output(['git','-C',str(p if nested else ROOT),
                    'ls-files','--cached','--others','--exclude-standard','-z',
                    *([] if nested else ['--',s[3:]])]).decode().split('\0')
                files.extend((p if nested else ROOT)/n for n in paths if n)
        rows = initial_errors
        start = time.perf_counter()
        def measure(p):
            try:
                st = p.stat()
                if not p.is_file():
                    return dict(path=p.relative_to(ROOT).as_posix(),non_file=True)
                return dict(path=p.relative_to(ROOT).as_posix(), bytes=st.st_size, sha256=digest(p))
            except OSError as e:
                return dict(path=p.relative_to(ROOT).as_posix(), error=str(e))
        # WindowsPath equality folds case, but some WSL-managed directories do not.
        # Preserve distinct phi/Phi paths by deduplicating strings, not Path objects.
        file_names=sorted(set(str(p) for p in files))
        print(json.dumps(dict(planned_files=len(file_names))),flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            for i,row in enumerate(pool.map(measure,map(Path,file_names))):
                rows.append(row)
                if i % 10000 == 0:
                    print(json.dumps(dict(files=i, seconds=time.perf_counter()-start)), flush=True)
        path.write_text(json.dumps(dict(status=entries, files=rows,
            scope='Git-visible content, including nested checkout tracked and untracked files; ignored build outputs excluded. Earlier exhaustive ignored-output scan stopped read-only.'), separators=(',', ':')))
        print(json.dumps(dict(entries=len(entries), files=len(rows), seconds=time.perf_counter()-start)))
    else:
        before = json.loads(path.read_text() if path.exists() else gzip.decompress(path.with_suffix('.json.gz').read_bytes()))
        current = git('status', '--porcelain=v1', '-z').decode().split('\0')
        differences = [dict(status=s) for s in before['status'] if s not in current]
        def compare(r):
            if 'error' in r or 'non_file' in r:
                return None
            p = ROOT / r['path']
            if not p.is_file() or p.stat().st_size != r['bytes'] or digest(p) != r['sha256']:
                return dict(path=r['path'])
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            differences.extend(r for r in pool.map(compare,before['files']) if r is not None)
        result = dict(entries=len(before['status']), files=len(before['files']), differences=differences,
                      baseline_errors=[r for r in before['files'] if 'error' in r],
                      before_sha256=digest(path) if path.exists() else hashlib.sha256(gzip.decompress(path.with_suffix('.json.gz').read_bytes())).hexdigest())
        (OUT / 'UNRELATED_AFTER.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='baseline_errors'} | {'baseline_error_count':len(result['baseline_errors'])}))


if __name__ == '__main__':
    if sys.argv[1] == 'sources':
        source_bundle()
    else:
        preservation(sys.argv[1])
