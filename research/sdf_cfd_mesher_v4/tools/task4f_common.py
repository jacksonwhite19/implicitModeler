"""Read-only preserved-run inputs and deadline for the bounded runtime audit."""
from pathlib import Path
import os,sys,json,time,datetime,subprocess,hashlib
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
from task3c_io import read
from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
OUT=ROOT/'results/task4f';BASE=ROOT/'results/task4e/a2';E=ROOT/'results/task4e'
def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes((json.dumps(plain(data),sort_keys=True,indent=2)+'\n').encode())
def elapsed():return (datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(read(OUT/'CLOCK.json')['task_start'])).total_seconds()
def require_time():
    if elapsed()>=3300:raise RuntimeError('Task4FScientificCutoff')
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()
