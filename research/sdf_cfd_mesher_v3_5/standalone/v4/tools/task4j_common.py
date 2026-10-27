"""Certificate-only preserved geometry and bounded performance evidence."""
from pathlib import Path
import os,sys,json,time,datetime,subprocess,collections,hashlib
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
from task3c_io import read
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain,canonical
OUT=ROOT/'results/task4j';BASE=ROOT/'results/task4i/fresh'
def now():return datetime.datetime.now(datetime.timezone.utc)
def elapsed():return (now()-datetime.datetime.fromisoformat(read(OUT/'PREFLIGHT.json')['task_start'])).total_seconds()
def require_time():
    if elapsed()>=3300:raise RuntimeError('Task4JScientificCutoff')
def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.pending')
    tmp.write_bytes((json.dumps(plain(data),sort_keys=True,indent=2)+'\n').encode());os.replace(tmp,p)
def geometry():
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    f=load_sf2_wall();t=build_topology(f.triangles,len(f.coordinates))
    with np.load(BASE/'FINAL_WALL_PRISM_SHELL.npz') as z:x=z['coordinates']
    assert array_hash(x[0])==f.coordinate_hash
    return f,t,x
