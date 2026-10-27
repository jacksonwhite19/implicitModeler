"""Finish only the new smaller-radius continuation after the existing worker."""
from pathlib import Path
import sys,subprocess,psutil,time,json
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/task3e'

if __name__=='__main__':
    waiting=[]
    for p in psutil.process_iter(['pid','name','cmdline']):
        args=p.info['cmdline'] or []
        if p.info['pid']!=psutil.Process().pid and p.info['name'].lower().startswith('python') and any(a.replace('\\','/').endswith('/task3e_continue.py') for a in args):waiting.append(p)
    for p in waiting:
        while p.is_running():
            try:p.wait(timeout=30)
            except psutil.TimeoutExpired:continue
    prior=json.loads((OUT/'COORDINATOR_TIMINGS.json').read_text());assert len(prior)==7 and all(r['exit_code']==0 for r in prior)
    commands=[('branch3','task3e_study.py',['--branch','3','--depth','4']),
              ('replay3','task3e_study.py',['--branch','3','--depth','4','--replay']),
              ('diagnostics_refined','task3e_diagnostics.py',['--refined']),
              ('diagnostics_refined_replay','task3e_diagnostics.py',['--refined','--replay']),
              ('tests','task3e_tests.py',[])]
    timing=[]
    for name,script,args in commands:
        start=time.perf_counter();print('START',name,flush=True)
        with (OUT/(name+'.log')).open('w') as log:
            r=subprocess.run([sys.executable,'-B',str(ROOT/'tools'/script),*args],stdout=log,stderr=subprocess.STDOUT)
        timing.append(dict(stage=name,exit_code=r.returncode,seconds=time.perf_counter()-start))
        (OUT/'REFINED_COORDINATOR_TIMINGS.json').write_text(json.dumps(timing,indent=2)+'\n')
        print('END',name,r.returncode,flush=True)
        if r.returncode:sys.exit(r.returncode)
