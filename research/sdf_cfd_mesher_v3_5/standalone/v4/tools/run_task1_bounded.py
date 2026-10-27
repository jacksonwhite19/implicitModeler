"""One child at a time; measured process-tree RSS and wall-clock caps."""
from pathlib import Path
import sys,subprocess,time,json,datetime,os
import psutil
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/task1';OUT.mkdir(parents=True,exist_ok=True)
start=json.loads((ROOT/'authorities/TASK1_START.json').read_text());deadline=datetime.datetime.fromisoformat(start['started_at'].replace('Z','+00:00')).timestamp()+start['execution_cap_seconds']
script=sys.argv[1];stamp=time.perf_counter();peak=0;reason=None
with (OUT/(Path(script).stem+'.log')).open('w') as stream:
    child=subprocess.Popen([sys.executable,'-B',str(ROOT/'tools'/script)],stdout=stream,stderr=subprocess.STDOUT,cwd=ROOT.parents[1],env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1'))
    proc=psutil.Process(child.pid)
    while child.poll() is None:
        try:rss=sum(p.memory_info().rss for p in [proc]+proc.children(recursive=True) if p.is_running());peak=max(peak,rss)
        except psutil.Error:pass
        if peak>start['max_rss_bytes'] or time.time()>deadline:
            reason='RSS limit' if peak>start['max_rss_bytes'] else 'execution deadline'
            for p in proc.children(recursive=True):p.terminate()
            proc.terminate();child.wait();break
        time.sleep(.1)
record=dict(script=script,exit_code=child.returncode,seconds=time.perf_counter()-stamp,peak_sampled_process_tree_rss_bytes=peak,sampling_period_seconds=.1,stopped_reason=reason)
(OUT/(Path(script).stem+'_RESOURCE.json')).write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record))
raise SystemExit(child.returncode or (1 if reason else 0))
