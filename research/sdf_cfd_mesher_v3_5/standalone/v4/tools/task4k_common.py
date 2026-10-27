from pathlib import Path
import os,sys,json,time,datetime,collections,hashlib
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain,canonical
from task3c_io import read
OUT=ROOT/'results/task4k';BASE=ROOT/'results/task4i/fresh'
def now():return datetime.datetime.now(datetime.timezone.utc)
def elapsed():return (now()-datetime.datetime.fromisoformat(read(OUT/'PREFLIGHT.json')['task_start'])).total_seconds()
def guard():
    if elapsed()>=3300:raise RuntimeError('Task4KScientificCutoff')
def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.pending');data=plain(data)
    content=('[\n'+',\n'.join(json.dumps(row,sort_keys=True,separators=(',',':')) for row in data)+'\n]\n') if isinstance(data,list) and len(data)>100 else json.dumps(data,sort_keys=True,indent=2)+'\n'
    tmp.write_bytes(content.encode());os.replace(tmp,p)
