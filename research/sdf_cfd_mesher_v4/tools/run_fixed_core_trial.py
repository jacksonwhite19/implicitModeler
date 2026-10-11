"""Observable process bound and measured disk guard for a single core trial."""
import sys,time,json,subprocess,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def wsl(p):p=Path(p).resolve();return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')
def run(src,interface,dest,method,limit=120):
    required=sum(p.stat().st_size for p in Path(src).glob('*') if p.is_file())+sum(p.stat().st_size for p in Path(interface).glob('*.npy'))
    free=shutil.disk_usage(dest.parent).free
    if free<required:raise ValueError('InsufficientMeasuredTrialSpace')
    start=time.perf_counter();cmd=['wsl','--','env','PYTHONPATH='+wsl(ROOT/'src'),'timeout',str(limit)+'s','python3','-B',wsl(ROOT/'tools/fixed_core_gmsh_trial.py'),wsl(src),wsl(interface),wsl(dest),method]
    with (dest.parent/(dest.name+'.log')).open('w') as log:
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
        while p.poll() is None:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:print(json.dumps(dict(stage=method,elapsed=time.perf_counter()-start,free_bytes=shutil.disk_usage(dest.parent).free)),flush=True)
    result=dict(method=method,elapsed=time.perf_counter()-start,exit_code=p.returncode,limit_seconds=limit,free_before=free,measured_input_bytes=required,free_after=shutil.disk_usage(dest.parent).free)
    (dest.parent/(dest.name+'_PROCESS.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':run(*map(Path,sys.argv[1:4]),sys.argv[4])
