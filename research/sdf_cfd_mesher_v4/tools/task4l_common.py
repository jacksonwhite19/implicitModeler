"""Fresh integrated qualification. Reference data are comparison-only."""
from pathlib import Path
import os,sys,json,time,datetime,hashlib,subprocess
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
from task3c_io import read
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain
OUT=ROOT/'results/task4l';RUN=OUT/'fresh'
def now():return datetime.datetime.now(datetime.timezone.utc)
def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True);q=p.with_suffix(p.suffix+'.pending')
    q.write_bytes((json.dumps(plain(data),indent=2,sort_keys=True)+'\n').encode());os.replace(q,p)
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()
def optional(p,default=None):return read(p) if p.exists() else default
def scientific_seconds():return (now()-datetime.datetime.fromisoformat(read(OUT/'CLOCK.json')['scientific_start'])).total_seconds()
