"""Deadline-supervised independent A2. No preserved candidates enter the runner."""
from task4e_common import *
import psutil,traceback

def can_start(elapsed_seconds,layer,final=False):
    if elapsed_seconds>=3300:return False
    # At the 50-minute review, only mandatory final qualification may start.
    # Remaining multi-stage layer work is not assumed to fit the reserve.
    return elapsed_seconds<3000 or final

def kill_tree(pid):
    try:root=psutil.Process(pid);procs=root.children(recursive=True)+[root]
    except psutil.NoSuchProcess:return []
    identities=[dict(pid=p.pid,created=p.create_time()) for p in procs]
    # Windows process handles retained by psutil guard PID reuse. Kill the
    # generating parent first, then every captured descendant; wait for exit.
    for p in reversed(procs):
        try:p.kill()
        except psutil.NoSuchProcess:pass
    _,alive=psutil.wait_procs(procs,timeout=3)
    for p in alive:
        try:p.kill()
        except psutil.NoSuchProcess:pass
    return identities

def run():
    assert read(OUT/'SMOKE_TESTS.json')['passed']
    env=dict(os.environ,TASK4_OUTPUT_DIRECTORY=str(RUN.resolve()),V4_CERTIFICATE_WORKERS='8')
    operations=[];comparisons=[];peak=0;process=None;status='Running';stage=None;layer=0
    tasks=[(0,'initial_forecast_scan','task4_initial_scan.py',[]),(1,'initial_correction','task4_correct_first.py',[]),(1,'initial_warning_atlas','task4_warning_cache.py',[])]
    for k in range(1,21):
        if k>1:tasks.append((k,'prepare','task4_prepare_next.py',['--layer',str(k)]))
        tasks.append((k,'certify_commit','task4_certify_candidate.py',['--layer',str(k)]))
    tasks.append((20,'final_validation','task4_final_validation.py',[]))
    tasks.append((20,'final_comparison','task4e_compare.py',[]))
    scientific_start=now().isoformat();save('EXECUTION.json',dict(status=status,scientific_start=scientific_start,elapsed=elapsed()))
    try:
        for layer,stage,tool,args in tasks:
            if not can_start(elapsed(),layer,stage.startswith('final_')):
                status='TimeBudgetStopBeforeStage';break
            started=now().isoformat();tick=time.perf_counter();log=RUN/f'{layer:02}_{stage}.log'
            print('START',layer,stage,elapsed(),flush=True)
            with log.open('w',encoding='utf-8') as stream:
                process=subprocess.Popen([sys.executable,'-B',str(ROOT/'tools'/tool),*args],cwd=REPO,env=env,stdout=stream,stderr=subprocess.STDOUT)
                save('LIVE_PROCESS.json',dict(pid=process.pid,layer=layer,stage=stage,started=started,cutoff=clock()['scientific_cutoff']))
                last_save=0
                while process.poll() is None:
                    e=elapsed()
                    try:
                        p=psutil.Process(process.pid);rss=0
                        for child in [p]+p.children(recursive=True):
                            try:rss+=child.memory_info().rss
                            except psutil.NoSuchProcess:pass
                        peak=max(peak,rss)
                    except psutil.NoSuchProcess:pass
                    # Start forced shutdown just before the absolute cutoff.
                    if e>=3297:
                        killed=kill_tree(process.pid);process.wait(timeout=3);status='ScientificCutoff'
                        save('TIMEOUT.json',dict(elapsed=e,stage=stage,layer=layer,killed=killed,cutoff=clock()['scientific_cutoff']));break
                    if e-last_save>=5:
                        save('LIVE_PROGRESS.json',dict(layer=layer,stage=stage,elapsed=e,peak_tree_rss=peak,committed=read(RUN/'build_A/CURRENT.json') if (RUN/'build_A/CURRENT.json').exists() else None));last_save=e
                    time.sleep(.2)
            operations.append(dict(layer=layer,stage=stage,started=started,finished=now().isoformat(),seconds=time.perf_counter()-tick,elapsed_task=elapsed(),exit_code=process.returncode,peak_tree_rss=peak))
            save('OPERATIONS.json',operations);process=None
            print('END',layer,stage,operations[-1]['seconds'],operations[-1]['exit_code'],elapsed(),flush=True)
            if status=='ScientificCutoff':break
            if operations[-1]['exit_code']:
                status='StageFailure';break
            if stage=='certify_commit':
                compare_started=time.perf_counter();row=compare_layer(layer);comparisons.append(row)
                save('PREFIX_COMPARISON.json',dict(layers=comparisons,all_equal=True));save('COMPARISON_TIMINGS.json',dict(last_layer=layer,last_seconds=time.perf_counter()-compare_started))
        else:status='ScientificStagesComplete'
    except BaseException as error:
        status='ScientificMismatch' if isinstance(error,(AssertionError,ValueError)) else 'CoordinatorFailure'
        save('FAILURE.json',dict(type=type(error).__name__,message=str(error),traceback=traceback.format_exc(),layer=layer,stage=stage,elapsed=elapsed()))
    finally:
        if process is not None and process.poll() is None:kill_tree(process.pid)
        save('EXECUTION.json',dict(status=status,scientific_start=scientific_start,scientific_stop=now().isoformat(),elapsed_task=elapsed(),
            seconds=(now()-datetime.datetime.fromisoformat(scientific_start)).total_seconds(),stage=stage,attempted_layer=layer,peak_tree_rss=peak,operations=len(operations),compared_layers=len(comparisons)))
    print('STOP',status,elapsed(),flush=True)

if __name__=='__main__':run()
