"""Package compact retained evidence and measure task storage without cleanup."""
import ctypes
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[3]
V4=ROOT/'research/sdf_cfd_mesher_v4'
OUT=V4/'results/generic_canonical_bridge'
sys.path.insert(0,str(V4/'tools'))
from canonical_bridge_evidence import digest


def allocation(p):
    high=ctypes.c_ulong(0)
    fn=ctypes.WinDLL('kernel32',use_last_error=True).GetCompressedFileSizeW
    fn.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_ulong)]
    fn.restype=ctypes.c_ulong
    ctypes.set_last_error(0)
    low=fn(str(p),ctypes.byref(high))
    if low==0xffffffff and ctypes.get_last_error():return None
    return (high.value<<32)|low


def space(root):
    available,total,free=ctypes.c_ulonglong(),ctypes.c_ulonglong(),ctypes.c_ulonglong()
    fn=ctypes.WinDLL('kernel32',use_last_error=True).GetDiskFreeSpaceExW
    if not fn(ctypes.c_wchar_p(root),ctypes.byref(available),ctypes.byref(total),ctypes.byref(free)):
        raise ctypes.WinError(ctypes.get_last_error())
    return dict(available_to_caller_bytes=available.value,total_unused_bytes=free.value,total_bytes=total.value)


def main():
    before=OUT/'UNRELATED_BEFORE.json'
    compressed=OUT/'UNRELATED_BEFORE.json.gz'
    if before.exists():
        with before.open('rb') as source, compressed.open('wb') as raw, gzip.GzipFile(filename='',mode='wb',fileobj=raw,mtime=0) as target:
            shutil.copyfileobj(source,target)
        with gzip.open(compressed,'rb') as source:
            h=hashlib.sha256()
            for b in iter(lambda:source.read(1<<20),b''):h.update(b)
        assert h.hexdigest()==digest(before)
    base='fcca9782f734f24bc8f01b9f3dc5d623dfa6e708'
    original_paths=set(subprocess.check_output(['git','ls-tree','-r','--name-only',base,'--','research/sdf_cfd_mesher_v4'],cwd=ROOT).decode().splitlines())
    changed=subprocess.check_output(['git','diff','--name-only',base,'--','research/sdf_cfd_mesher_v4'],cwd=ROOT).decode().splitlines()
    untracked=subprocess.check_output(['git','-c','core.longpaths=true','ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4'],cwd=ROOT).decode().splitlines()
    paths=sorted(set(changed+untracked))
    rows=[]
    for name in paths:
        p=ROOT/name
        if p.is_file() and name not in ['research/sdf_cfd_mesher_v4/results/generic_canonical_bridge/ARTIFACT_MANIFEST.json',
                                        'research/sdf_cfd_mesher_v4/results/generic_canonical_bridge/STORAGE.json']:
            rows.append(dict(path=name,bytes=p.stat().st_size,compressed_storage_api_bytes=allocation(p),sha256=digest(p),new_file=name not in original_paths))
    (OUT/'ARTIFACT_MANIFEST.json').write_text(json.dumps(dict(files=rows,scope='Pre-commit task files; manifest and storage record excluded to avoid self-reference'),indent=2)+'\n')
    cache=subprocess.check_output(['wsl','-e','du','-s','-B1','/mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/target']).decode().strip()
    volumes=subprocess.check_output(['wsl','-e','df','-B1','/','/mnt/c']).decode()
    now=datetime.now(timezone.utc)
    new_paths=[r['path'] for r in rows if r['new_file']]
    du=subprocess.check_output(['wsl','-e','du','-B1','-c','--',*['/mnt/c/'+str(ROOT)[3:].replace('\\','/')+'/'+p for p in new_paths]]).decode()
    storage=dict(measured_at_utc=now.isoformat(),initial_c_free_bytes=180922318848,
        initial_z_free_bytes=847658938368,final_precommit_c_free_bytes=shutil.disk_usage(ROOT).free,
        final_precommit_z_free_bytes=space('Z:/')['available_to_caller_bytes'],
        windows_volume_details={drive:space(drive+':/') for drive in ['C','Z']},
        initial_target_allocated_bytes=10132897792,final_target_du=cache,
        task_new_logical_bytes=sum(r['bytes'] for r in rows if r['new_file']),
        task_new_allocated_bytes=int(du.splitlines()[-1].split()[0]),
        task_new_GetCompressedFileSizeW_bytes=sum(r['compressed_storage_api_bytes'] or 0 for r in rows if r['new_file']),
        task_all_listed_logical_bytes=sum(r['bytes'] for r in rows),
        allocation_measure='WSL du -B1 for task allocation and shared target cache. GetCompressedFileSizeW is separately reported and excludes allocation slack on ordinary files.',
        runtime_binary_bytes=(OUT/'canonical_cgal_seed').stat().st_size,
        runtime_binary_allocated_bytes=int(subprocess.check_output(['wsl','-e','du','-B1','/mnt/c/'+str(ROOT)[3:].replace('\\','/')+'/research/sdf_cfd_mesher_v4/results/generic_canonical_bridge/canonical_cgal_seed']).split()[0]),
        redundant_raw_manifest_bytes=before.stat().st_size if before.exists() else 0,
        compressed_manifest_bytes=compressed.stat().st_size,volumes=volumes,
        elapsed_since_first_task_artifact_seconds=now.timestamp()-(V4/'tools/canonical_bridge_evidence.py').stat().st_ctime,
        duration_limit='Artifact creation excludes earlier inspection; exact full-turn elapsed and compiler peak unavailable',
        backing='WSL root shares Z backing; /mnt/c shares C. Do not sum aliases.',
        cleanup='Removed only the 3144-byte task-owned intermediate admission backup and the redundant 87096526-byte raw baseline after verifying lossless gzip retention. No scientific mesh or capsule authority deleted.',
        projected_future_storage=None)
    (OUT/'STORAGE.json').write_text(json.dumps(storage,indent=2)+'\n')
    print(json.dumps(storage))


if __name__=='__main__':main()
