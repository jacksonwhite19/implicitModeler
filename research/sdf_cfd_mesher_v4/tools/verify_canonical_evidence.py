"""Compact source, historical-authority and storage checks; no mesh generation."""
import ctypes
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile
import numpy as np

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[3]
V4=ROOT/'research/sdf_cfd_mesher_v4'
OUT=V4/'results/generic_canonical_bridge'
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from canonical_bridge_evidence import digest
from geometry_family_authority import primitive


def source_checks():
    manifest=json.loads((OUT/'KERNEL_SOURCE_MANIFEST.json').read_text())
    hashed=hashlib.sha256()
    with zipfile.ZipFile(OUT/'KERNEL_SOURCE_SNAPSHOT.zip') as z:
        for row in manifest['files']:
            p=ROOT/row['path']; raw=p.read_bytes()
            assert hashlib.sha256(raw).hexdigest()==row['sha256']
            assert z.read(row['path'])==raw
            hashed.update(row['path'].encode());hashed.update(b'\0')
            hashed.update(len(raw).to_bytes(8,'little'));hashed.update(raw)
    direct=json.loads((OUT/'DIRECT.json').read_text())
    assert hashed.hexdigest()==direct['kernel_source_sha256']
    capsule=next(c for c in direct['cases'] if c['name']=='capsule')
    historical=primitive(np.array(capsule['points_m']),dict(primitive='capsule',radius=.04,half_length=.04))[0]
    errors=np.abs(historical-np.array(capsule['direct_m']))
    index=int(errors.argmax())
    old=subprocess.check_output(['git','show','fcca9782f734f24bc8f01b9f3dc5d623dfa6e708:research/sdf_cfd_mesher_v4/tools/production_adaptive_surface.py'],cwd=ROOT)
    moved=(V4/'tools/historical_capsule_surface.py').read_bytes()
    assert old.replace(b'\r\n',b'\n').rstrip()==moved.replace(b'\r\n',b'\n').rstrip()
    # All sealed capsule files tracked in the initial commit still equal that commit.
    paths=subprocess.check_output(['git','ls-files','-z','--','research/sdf_cfd_mesher_v4/results/production_adaptive_capsule_surface_foundation',
        'research/sdf_cfd_mesher_v4/tools/production_capsule_cgal_seed.cpp',
        'research/sdf_cfd_mesher_v4/native','research/sdf_cfd_mesher_v4/src/sdf_cfd_mesher_v4/sdf/native.py'],cwd=ROOT).decode().split('\0')
    preserved=[]
    for path in filter(None,paths):
        raw=(ROOT/path).read_bytes()
        oldblob=subprocess.check_output(['git','show','fcca9782f734f24bc8f01b9f3dc5d623dfa6e708:'+path],cwd=ROOT)
        # Git text normalization applies only to known text suffixes.
        normalized=raw.replace(b'\r\n',b'\n') if Path(path).suffix in ('.py','.rs','.json','.toml','.cpp','.h','.md','.txt','.csv','.off','.mesh','.lock') else raw
        assert normalized==oldblob or raw==oldblob,path
        preserved.append(dict(path=path,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
    result=dict(kernel_source_sha256=hashed.hexdigest(),snapshot_files=len(manifest['files']),
        snapshot_bytes=sum(r['bytes'] for r in manifest['files']),snapshot_matches_current_source=True,
        historical_code_equal_except_line_endings_and_final_blank_line=True,
        historical_code_sha256=digest(V4/'tools/historical_capsule_surface.py'),preserved_files=preserved,
        kernel_capsule_radius_f32=float(np.float32(.04)),historical_capsule_radius_f64=.04,
        capsule_historical_comparison=dict(points=len(errors),max_abs_difference_m=float(errors[index]),
            at_m=capsule['points_m'][index],historical_phi_m=float(historical[index]),kernel_phi_m=capsule['direct_m'][index],
            conclusion='different precision authorities; no exact substitution claimed'))
    (OUT/'SOURCE_AND_HISTORY_CHECK.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='preserved_files'}))


def link_record(label):
    code="import os,json,hashlib;p='/mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/'+chr(92);s=os.lstat(p);t=os.readlink(p);print(json.dumps(dict(path='U+F05C Windows name maps to backslash in WSL',mode=s.st_mode,bytes=s.st_size,target=t,target_sha256=hashlib.sha256(t.encode()).hexdigest())))"
    row=json.loads(subprocess.check_output(['wsl','-e','python3','-c',code]))
    (OUT/('SYMLINK_'+label+'.json')).write_text(json.dumps(row,indent=2)+'\n')
    if label=='AFTER': assert row==json.loads((OUT/'SYMLINK_BEFORE.json').read_text())
    print(json.dumps(row))


if __name__=='__main__':
    if sys.argv[1]=='sources': source_checks()
    else: link_record(sys.argv[1])
