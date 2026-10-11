"""Task-4C authority boundaries. Preserved geometry is read-only input."""
from pathlib import Path
import os,sys,json,hashlib,time,datetime,subprocess
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1];OUT=ROOT/'results/task4c';BASE=ROOT/'results/task4';B=ROOT/'results/task4b'
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
import numpy as np
from task3c_io import read,write
from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash,identity
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain
def save(name,value):
    OUT.mkdir(exist_ok=True);(OUT/name).write_text(json.dumps(plain(value),sort_keys=True,indent=2)+'\n')
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()
def geometry():
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    f=load_sf2_wall();t=build_topology(f.triangles,len(f.coordinates))
    return f,t,np.stack([f.coordinates]+[np.load(BASE/f'build_A/FRONT_{k:02}.npy') for k in range(1,21)])
def preflight():
    assert git('rev-parse','HEAD')=='1ce04e5304fc7d9838fe978673d7b53c6b8e5c2c'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    started=datetime.datetime.now(datetime.timezone.utc).isoformat();from task4_preflight import audit
    prior=audit();counts={}
    expected=[(BASE,'report.md','28fcb9be7fac30eec2dd84a3d292fbfbdead3daef3a4ed851a4df7744a213190'),(BASE,'AUTHORITY.json','be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9'),
      (B,'report.md','ec5686586f9eb5e2e65f55e72338dae3276b82e8f9d70a4e70efc5f610c7bba2'),(B,'AUTHORITY.json','8fe4af86b569705ac41399dcdbd0a02ddb7ff41b62a75fc254cd7998bd754aac')]
    for base,name,h in expected:assert file_hash(base/name)==h,(base,name)
    for base in (BASE,B):
        a=read(base/'AUTHORITY.json');count=0
        for group in ('source_sha256','evidence_sha256'):
            for p,h in a[group].items():assert file_hash(ROOT/p)==h,p;count+=1
        counts[base.name]=count
    final=read(BASE/'FINAL_VALIDATION.json');assert final['shell_scientific_hash']=='54a69dae10d792aeee02d945fcd095f053990a10db695e97626ab540ae24e51c'
    OUT.mkdir(exist_ok=True);save('PREFLIGHT.json',dict(started_utc=started,head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),status=git('status','--porcelain'),
        verified_counts=counts,prior=prior,shell_hash=final['shell_scientific_hash'],shell_prompt_note='Prompt has an extra final 0; frozen 64-character shell identity verified.',
        Build_A_checkpoint=file_hash(BASE/'build_A/CURRENT.json'),A2_checkpoint=file_hash(BASE/'replay_A2/build_A/CURRENT.json')))
    print('AUTHORITIES VERIFIED',counts,flush=True)
if __name__=='__main__':preflight()
