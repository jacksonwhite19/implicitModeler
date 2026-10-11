"""Independent interpreter/hash-order qualification of exact proof bytes."""
from task3_root_regression import *
import subprocess,os

if __name__=='__main__':
    t=time.perf_counter();rows=[]
    command="import sys;sys.dont_write_bytecode=True;sys.path.insert(0,'research/sdf_cfd_mesher_v4/tools');from task3_root_regression import qualify,scientific_hash;print(scientific_hash(qualify()))"
    expected=scientific_hash(json.loads((OUT/'ROOT_REGRESSION.json').read_text()))
    for seed in ['1','17','123']:
        env=dict(os.environ,PYTHONHASHSEED=seed)
        got=subprocess.check_output([sys.executable,'-c',command],cwd=ROOT.parents[1],env=env,text=True).strip()
        assert got==expected,(seed,got,expected)
        rows.append(dict(python_hash_seed=seed,scientific_hash=got,passed=True))
    write('HASH_ORDER_REPLAY.json',dict(passed=True,rows=rows,scope='Interpreter hash ordering is varied only as a determinism test; no stochastic scientific search exists'))
    write('HASH_ORDER_TIMING.json',dict(seconds=time.perf_counter()-t))
    print(json.dumps(dict(passed=True,processes=len(rows),hash=expected)))
