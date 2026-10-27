"""One scientific worker at a time; preserve logs and stop on execution errors."""
from pathlib import Path
import sys,subprocess,psutil,time,json
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/task3e'

if __name__=='__main__':
    waiting=[]
    for p in psutil.process_iter(['pid','name','cmdline']):
        args=p.info['cmdline'] or []
        if p.info['pid']!=psutil.Process().pid and p.info['name'].lower().startswith('python') and any(a.replace('\\','/').endswith('/task3e_study.py') for a in args) and '--branch' in args and args[args.index('--branch')+1]=='0':waiting.append(p)
    for p in waiting:
        while p.is_running():
            try:p.wait(timeout=30)
            except psutil.TimeoutExpired:continue
    assert (OUT/'BRANCH_0_HASH.json').exists(),'InitialBranchDidNotFinish'
    commands=[('branch1','task3e_study.py',['--branch','1','--depth','4']),
              ('branch2','task3e_study.py',['--branch','2','--depth','4']),
              ('replay0','task3e_study.py',['--branch','0','--depth','4','--replay']),
              ('replay1','task3e_study.py',['--branch','1','--depth','4','--replay']),
              ('replay2','task3e_study.py',['--branch','2','--depth','4','--replay']),
              ('diagnostics','task3e_diagnostics.py',[]),('diagnostics_replay','task3e_diagnostics.py',['--replay'])]
    timing=[]
    for name,script,args in commands:
        start=time.perf_counter();print('START',name,flush=True)
        with (OUT/(name+'.log')).open('w') as log:
            r=subprocess.run([sys.executable,'-B',str(ROOT/'tools'/script),*args],stdout=log,stderr=subprocess.STDOUT)
        timing.append(dict(stage=name,exit_code=r.returncode,seconds=time.perf_counter()-start))
        (OUT/'COORDINATOR_TIMINGS.json').write_text(json.dumps(timing,indent=2)+'\n')
        print('END',name,r.returncode,flush=True)
        if r.returncode:sys.exit(r.returncode)
