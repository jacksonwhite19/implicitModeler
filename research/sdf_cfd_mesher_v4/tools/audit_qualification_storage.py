"""Read-only storage inventory. Never imports a geometry generator or deletes data."""
import collections
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
OUT = V4 / 'results/production_qualification_storage_audit'
L2 = V4 / 'results/production_adaptive_capsule_layer2'
L1 = V4 / 'results/production_adaptive_capsule_first_layer'

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def save(name, value):
    (OUT/name).write_text(json.dumps(value, indent=2, sort_keys=True)+'\n')

def main():
    start = time.perf_counter()
    OUT.mkdir(parents=True, exist_ok=True)
    status = subprocess.check_output(['git','status','--porcelain','-z'], cwd=ROOT)
    save('BASELINE.json', dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
         status_hex=status.hex(), free_bytes=shutil.disk_usage(ROOT).free))
    totals=collections.Counter(); rows=[]; errors=[]; by_store=collections.Counter()
    for base, dirs, files in os.walk(ROOT, onerror=lambda e: errors.append(str(e)), followlinks=False):
        dirs[:]=[d for d in dirs if not Path(base,d).is_symlink() and Path(base,d) not in (ROOT/'.claude/worktrees',ROOT/'dual_contouring')]
        for name in files:
            p=Path(base,name)
            try:s=p.stat()
            except OSError as e:errors.append(str(e));continue
            totals['repository_logical_bytes']+=s.st_size
            if p.is_relative_to(V4):
                totals['v4_logical_bytes']+=s.st_size
                parts=p.relative_to(V4).parts
                by_store['/'.join(parts[:2])]+=s.st_size
            if p.is_relative_to(L1) or p.is_relative_to(L2):
                rows.append(dict(path=p.relative_to(ROOT).as_posix(),bytes=s.st_size,device=s.st_dev,inode=s.st_ino,links=s.st_nlink,mtime_ns=s.st_mtime_ns))
    save('DISK.json',dict(totals=totals,stores=by_store,errors=errors,scan_seconds=time.perf_counter()-start,
         exclusions=['.claude/worktrees','dual_contouring','directory symlinks'],free_bytes=shutil.disk_usage(ROOT).free))
    print('inventory complete',len(rows),dict(totals),flush=True)
    hashes={}; hashed_bytes=0
    for i,r in enumerate(rows):
        key=(r['device'],r['inode']);p=ROOT/r['path']
        if key not in hashes:
            hashes[key]=digest(p);hashed_bytes+=r['bytes']
        s=p.stat()
        if (s.st_size,s.st_mtime_ns,s.st_ino)!=(r['bytes'],r['mtime_ns'],r['inode']):raise ValueError('Input changed during inventory')
        r['sha256']=hashes[key]
        if i%50==0:print('hashed',i,'/',len(rows),flush=True)
    save('FILES.json',rows)
    groups=collections.defaultdict(list)
    for r in rows:groups[(r['bytes'],r['sha256'])].append(r)
    duplicates=[]
    for (size,h),items in groups.items():
        if len(items)>1:
            inodes={(r['device'],r['inode']) for r in items}
            duplicates.append(dict(bytes=size,sha256=h,paths=[r['path'] for r in items],distinct_file_ids=len(inodes),
                duplicate_logical_bytes=size*(len(items)-1),distinct_duplicate_payload_bytes=size*(len(inodes)-1)))
    save('DUPLICATES.json',sorted(duplicates,key=lambda x:x['distinct_duplicate_payload_bytes'],reverse=True))
    import psutil
    m=psutil.Process().memory_info()
    save('RESOURCES.json',dict(seconds=time.perf_counter()-start,hashed_unique_file_bytes=hashed_bytes,
         peak_rss=getattr(m,'peak_wset',None),retained_rss=m.rss,free_bytes=shutil.disk_usage(ROOT).free,bytes_deleted=0))
    print('audit inventory done',flush=True)

if __name__=='__main__':main()
