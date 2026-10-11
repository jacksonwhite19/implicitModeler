"""Progress, disk guard and hard process bound for compact cavity workers."""
import sys,time,subprocess,shutil,json
from pathlib import Path
from run_fixed_core_trial import wsl,ROOT

def run(input_file,out,method,points,limit=120):
    free=shutil.disk_usage(out.parent).free
    # Measured previous output is the scratch reservation, in addition to input.
    previous=max([sum(p.stat().st_size for p in d.rglob('*') if p.is_file()) for d in out.parent.iterdir() if d.is_dir()]+[0])
    need=input_file.stat().st_size+previous
    if free<need:raise ValueError('InsufficientMeasuredCavitySpace')
    cmd=['wsl','--','env','OPENBLAS_NUM_THREADS=1','OMP_NUM_THREADS=1','PYTHONPATH='+wsl(ROOT/'src'),'timeout',str(limit)+'s','python3','-B',wsl(ROOT/'tools/constrained_cavity_gmsh.py'),wsl(input_file),wsl(out),method,points]
    tick=time.perf_counter()
    with out.with_suffix('.log').open('w') as f:
        p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
        while p.poll() is None:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:print(json.dumps(dict(stage=out.name,elapsed=time.perf_counter()-tick,free=shutil.disk_usage(out.parent).free)),flush=True)
    r=dict(seconds=time.perf_counter()-tick,exit_code=p.returncode,limit_seconds=limit,free_before=free,measured_scratch_reservation=need,free_after=shutil.disk_usage(out.parent).free)
    out.with_suffix('.process.json').write_text(json.dumps(r,indent=2));print(json.dumps(dict(trial=out.name,**r)),flush=True)
    return r

if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]),sys.argv[3],sys.argv[4])
