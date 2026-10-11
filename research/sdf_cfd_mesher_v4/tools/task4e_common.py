"""Independent timed Task-4E run and read-only regression authorities."""
from pathlib import Path
import os,sys,json,time,datetime,hashlib,subprocess
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
OUT=ROOT/'results/task4e';RUN=OUT/'a2';BASE=ROOT/'results/task4'
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
from task3c_io import read
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain

def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True);temp=p.with_suffix(p.suffix+'.pending')
    temp.write_bytes((json.dumps(plain(data),sort_keys=True,indent=2)+'\n').encode());os.replace(temp,p)

def clock():return json.loads((OUT/'CLOCK.json').read_text())
def now():return datetime.datetime.now(datetime.timezone.utc)
def elapsed():return (now()-datetime.datetime.fromisoformat(clock()['task_start'])).total_seconds()
def git(*args):return subprocess.check_output(['git',*args],cwd=REPO,text=True).strip()

def compare_layer(layer):
    # Only this post-commit comparator reads Build-A geometry. None is passed
    # into the generating subprocess or its planning caches.
    from sdf_cfd_mesher_v4.certificate.replay_equivalence import compare_checkpoint
    a=read(BASE/f'build_A/LAYER_{layer:02}.json');b=read(RUN/f'build_A/LAYER_{layer:02}.json')
    x=np.load(BASE/f'build_A/FRONT_{layer:02}.npy');y=np.load(RUN/f'build_A/FRONT_{layer:02}.npy')
    assert np.array_equal(x.view(np.uint64),y.view(np.uint64)),'CoordinateMismatch'
    ref=read(ROOT/f'results/task4b/optimized_2/LAYER_{layer:02}.json')
    record=compare_checkpoint(a,b,ref)
    history='CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'
    assert scientific_hash(read(BASE/history))==scientific_hash(read(RUN/history)),'CorrectionHistoryMismatch'
    with np.load(BASE/f'LAYER_{layer:02}_CONDITIONING.npz') as u,np.load(RUN/f'LAYER_{layer:02}_CONDITIONING.npz') as v:
        assert u.files==v.files
        for key in u.files:assert np.array_equal(u[key].view(np.uint8),v[key].view(np.uint8)),('ConditioningMismatch',key)
    return dict(**record,coordinates_equal=True,conditioning_equal=True,corrections_equal=True,checkpoint_hash=file_hash(RUN/f'build_A/LAYER_{layer:02}.json'))
