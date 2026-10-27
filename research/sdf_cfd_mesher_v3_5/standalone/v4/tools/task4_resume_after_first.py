"""Wait for the single initial certificate worker, then resume certified state."""
from pathlib import Path
import sys,subprocess,json,psutil
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/task4'

if __name__=='__main__':
    workers=[]
    for p in psutil.process_iter(['pid','name','cmdline']):
        args=p.info['cmdline'] or []
        if p.info['pid']!=psutil.Process().pid and p.info['name'].lower().startswith('python') and any(a.replace('\\','/').endswith('/task4_certify_candidate.py') for a in args):workers.append(p)
    for p in workers:
        while p.is_running():
            try:p.wait(timeout=30)
            except psutil.TimeoutExpired:continue
    pointer=json.loads((OUT/'build_A/CURRENT.json').read_text())
    if pointer['layer']!=1:raise RuntimeError('InitialWorkerDidNotCommitLayerOne: inspect preserved evidence')
    with (OUT/'rolling_build.log').open('w') as log:
        result=subprocess.run([sys.executable,'-B',str(ROOT/'tools/task4_build.py')],stdout=log,stderr=subprocess.STDOUT)
    print('ROLLING_WORKER_EXIT',result.returncode,flush=True)
    sys.exit(result.returncode)
