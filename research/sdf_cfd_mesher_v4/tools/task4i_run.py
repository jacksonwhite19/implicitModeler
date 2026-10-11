"""Independent production run with a process-tree scientific deadline."""
from task4i_common import *
from task4e_run import kill_tree
import psutil,traceback

def may_start(seconds):return seconds<1797
def tasks():
    result=[(0,'initial_forecast_scan','task4_initial_scan.py',[]),(1,'initial_correction','task4_correct_first.py',[]),(1,'initial_warning_atlas','task4_warning_cache.py',[])]
    for k in range(1,21):
        if k>1:result.append((k,'prepare','task4_prepare_next.py',['--layer',str(k)]))
        result.append((k,'certify_commit','task4_certify_candidate.py',['--layer',str(k)]))
    return result+[(20,'final_validation','task4_final_validation.py',[]),(20,'final_comparison','task4i_compare.py',[])]

def run():
    assert read(OUT/'SMOKE_TESTS.json')['passed']
    config=read(OUT/'RUNNER_CONFIGURATION.json')
    for p,h in config['source_hashes'].items():assert file_hash(ROOT/p)==h,p
    assert not (RUN/'INITIAL_FORECAST.npz').exists() and not (RUN/'build_A/CURRENT.json').exists()
    begin=now();save('CLOCK.json',dict(task_start=read(OUT/'PREFLIGHT_INITIAL.json')['task_start'],scientific_start=begin.isoformat(),
        scientific_cutoff=(begin+datetime.timedelta(seconds=1800)).isoformat(),sealing_deadline=(begin+datetime.timedelta(seconds=2100)).isoformat()))
    env=dict(os.environ,TASK4_OUTPUT_DIRECTORY=str(RUN.resolve()),V4_CERTIFICATE_WORKERS='8')
    operations=[];peak=0;process=None;status='Running';stage=None;layer=0;reviewed=False
    try:
        for layer,stage,tool,args in tasks():
            if not may_start(scientific_seconds()):status='ScientificCutoff';break
            started=now().isoformat();tick=time.perf_counter();name=f'{layer:02}_{stage}'
            print('START',name,scientific_seconds(),flush=True)
            with (RUN/(name+'.log')).open('w',encoding='utf-8') as stream:
                env['TASK4I_STAGE_NAME']=name
                process=subprocess.Popen([sys.executable,'-B',str(ROOT/'tools/task4i_stage.py'),tool,*args],cwd=REPO,env=env,stdout=stream,stderr=subprocess.STDOUT)
                save('LIVE_PROCESS.json',dict(pid=process.pid,stage=stage,layer=layer,started=started))
                last=0
                while process.poll() is None:
                    e=scientific_seconds()
                    try:
                        root=psutil.Process(process.pid);rss=0
                        for p in [root]+root.children(recursive=True):
                            try:rss+=p.memory_info().rss
                            except psutil.NoSuchProcess:pass
                        peak=max(peak,rss)
                    except psutil.NoSuchProcess:pass
                    if e>=1620 and not reviewed:
                        reviewed=True;save('MINUTE27_REVIEW.json',dict(seconds=e,stage=stage,layer=layer,policy='Only mandatory build/qualification work remains; no optional work. Hard stop unchanged.'))
                    if e>=1797:
                        killed=kill_tree(process.pid);process.wait(timeout=3);status='ScientificCutoff'
                        save('TIMEOUT.json',dict(seconds=e,stage=stage,layer=layer,killed=killed,committed=optional(RUN/'build_A/CURRENT.json')));break
                    if e-last>=5:
                        save('LIVE_PROGRESS.json',dict(seconds=e,stage=stage,layer=layer,peak_tree_rss=peak,committed=optional(RUN/'build_A/CURRENT.json')));last=e
                    time.sleep(.2)
            operations.append(dict(layer=layer,stage=stage,started=started,finished=now().isoformat(),seconds=time.perf_counter()-tick,
                elapsed_task=scientific_seconds(),exit_code=process.returncode,peak_tree_rss=peak))
            save('OPERATIONS.json',operations);process=None
            print('END',name,operations[-1]['seconds'],operations[-1]['exit_code'],scientific_seconds(),flush=True)
            if status=='ScientificCutoff':break
            if operations[-1]['exit_code']:status='ScientificMismatch' if stage=='final_comparison' else 'StageFailure';break
        else:status='ScientificStagesComplete'
    except BaseException as error:
        status='CoordinatorFailure';save('FAILURE.json',dict(message=str(error),traceback=traceback.format_exc(),stage=stage,layer=layer))
    finally:
        if process is not None and process.poll() is None:kill_tree(process.pid)
        stop=now();save('EXECUTION.json',dict(status=status,scientific_start=begin.isoformat(),scientific_stop=stop.isoformat(),
            seconds=(stop-begin).total_seconds(),stage=stage,attempted_layer=layer,peak_tree_rss=peak,operations=len(operations)))
    print('STOP',status,scientific_seconds(),flush=True)
if __name__=='__main__':run()
