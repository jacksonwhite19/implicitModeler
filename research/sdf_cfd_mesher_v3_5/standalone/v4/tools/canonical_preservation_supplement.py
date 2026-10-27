"""Read-only WSL identity of symlinks, missing entries, and nested repositories."""
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import gzip

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'research/sdf_cfd_mesher_v4/results/generic_canonical_bridge'


def sha(data):return hashlib.sha256(data).hexdigest()


def one(path):
    try:
        s=path.lstat()
    except FileNotFoundError:
        return dict(kind='missing')
    if stat.S_ISLNK(s.st_mode):
        target=os.readlink(path)
        return dict(kind='symlink',target=target,target_sha256=sha(target.encode()),mode=s.st_mode)
    if path.is_file():
        h=hashlib.sha256()
        with path.open('rb') as f:
            for b in iter(lambda:f.read(1<<20),b''):h.update(b)
        return dict(kind='file',bytes=s.st_size,sha256=h.hexdigest())
    if path.is_dir() and (path/'.git').exists():
        windows_git=os.environ.get('V4_NATIVE_GIT')=='1' or ((path/'.git').is_file() and 'C:/' in (path/'.git').read_text())
        executable='/mnt/c/Program Files/Git/cmd/git.exe' if windows_git else 'git'
        directory='C:'+str(path)[6:] if windows_git else str(path)
        def git(*args):return subprocess.check_output([executable,'-c','safe.directory=*','-c','core.longpaths=true','-C',directory,*args],stderr=subprocess.DEVNULL)
        # Git hashes tracked bytes; diff binds dirty tracked bytes. Untracked bytes are hashed explicitly.
        head=git('rev-parse','HEAD').decode().strip()
        status=git('status','--porcelain=v1','-z')
        diff=git('diff','--binary','HEAD','--')
        untracked=[]
        for name in git('ls-files','--others','--exclude-standard','-z').decode().split('\0'):
            if name:untracked.append(dict(path=name,identity=one(path/name)))
        return dict(kind='nested_repository',head=head,status_sha256=sha(status),diff_sha256=sha(diff),
                    untracked=untracked)
    return dict(kind='directory' if path.is_dir() else 'special',mode=s.st_mode)


label=sys.argv[1]
baseline=OUT/'UNRELATED_BEFORE.json'
before=json.loads(baseline.read_text() if baseline.exists() else gzip.decompress(baseline.with_suffix('.json.gz').read_bytes()))
rows=[]
for row in before['files']:
    if 'error' in row or 'non_file' in row:
        name=row['path']
        # Windows maps the Linux backslash symlink name into the private-use range.
        path=ROOT/('\\' if name=='\uf05c' else name)
        rows.append(dict(path=name,identity=one(path)))
record=dict(entries=rows,scope='Supplement to Windows file hashes; ignored files excluded')
(OUT/('UNRELATED_SUPPLEMENT_'+label+'.json')).write_text(json.dumps(record,indent=2)+'\n')
if label=='AFTER':assert record==json.loads((OUT/'UNRELATED_SUPPLEMENT_BEFORE.json').read_text())
print(json.dumps(dict(entries=len(rows),kinds={kind:sum(r['identity']['kind']==kind for r in rows)
    for kind in sorted(set(r['identity']['kind'] for r in rows))},matched=label=='AFTER')))
