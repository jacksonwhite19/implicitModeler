"""Read preserved Build A; Task-4B outputs never enter committed geometry."""
from pathlib import Path
import sys,os,json,time,hashlib,datetime,subprocess
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
sys.dont_write_bytecode=True
import numpy as np
from task3c_io import read,write
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash
BASE=ROOT/'results/task4';OUT=ROOT/'results/task4b';OUT.mkdir(exist_ok=True)
def save(name,data):
    (OUT/name).write_text(json.dumps(data,sort_keys=True,indent=2)+'\n')
def geometry():
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    f=load_sf2_wall();t=build_topology(f.triangles,len(f.coordinates))
    x=np.stack([f.coordinates]+[np.load(BASE/f'build_A/FRONT_{k:02}.npy') for k in range(1,21)])
    return f,t,x
def preflight():
    from task4_preflight import audit,git
    assert git('rev-parse','HEAD')=='f76b30cda7a0cc5fcb2b9b768c480d91131a8d71'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    start=datetime.datetime.now(datetime.timezone.utc).isoformat();prior=audit()
    for name,h in [('report.md','28fcb9be7fac30eec2dd84a3d292fbfbdead3daef3a4ed851a4df7744a213190'),('AUTHORITY.json','be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9')]:assert file_hash(BASE/name)==h
    auth=json.loads((BASE/'AUTHORITY.json').read_text());count=0
    for group in ('source_sha256','evidence_sha256'):
        for p,h in auth[group].items():assert file_hash(ROOT/p)==h,p;count+=1
    final=read(BASE/'FINAL_VALIDATION.json');assert final['shell_scientific_hash']=='54a69dae10d792aeee02d945fcd095f053990a10db695e97626ab540ae24e51c'
    result=dict(started_utc=start,head=git('rev-parse','HEAD'),status=git('status','--porcelain'),prior=prior,verified_task4_files=count,
        shell_hash=final['shell_scientific_hash'],prompt_hash_note='Supplied shell hash has 65 characters; extra final 0. The sealed report and authority bind the 64-character value recorded here.',
        A_checkpoint=file_hash(BASE/'build_A/CURRENT.json'),A2_checkpoint=file_hash(BASE/'replay_A2/build_A/CURRENT.json'))
    save('PREFLIGHT.json',result);print('PREFLIGHT PASSED',count,flush=True)
if __name__=='__main__':preflight()
