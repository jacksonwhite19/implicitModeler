"""Operational supervision of one explicitly selected realization; no retry."""
import argparse,json,subprocess,sys,time,shutil
import psutil
from run_live_sf2_first_layer import OUT,ROOT,preflight,save,size

def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('label',choices=['A','B'])
    parser.add_argument('--profile',choices=['production','qualification'],default='qualification')
    parser.add_argument('--resume-initial',type=__import__('pathlib').Path)
    parser.add_argument('--output',type=__import__('pathlib').Path);parser.add_argument('--surface',type=__import__('pathlib').Path);parser.add_argument('--request',type=__import__('pathlib').Path);parser.add_argument('--surface-identity')
    args=parser.parse_args();extra=[]
    if args.profile=='production' and args.label!='A':raise ValueError('ProductionInitialSingleRealization')
    if args.resume_initial is not None and args.profile!='production':raise ValueError('ProductionInitialResumeRequired')
    if any(v is not None for v in (args.output,args.surface,args.request,args.surface_identity)):
        if any(v is None for v in (args.output,args.surface,args.request,args.surface_identity)):parser.error('Complete first-layer case inputs required')
        import run_live_sf2_first_layer as runner
        runner.configure_inputs(args.output,args.surface,args.request,args.surface_identity);OUT=runner.OUT
        extra=['--output',str(OUT),'--surface',str(runner.SURFACE),'--request',str(runner.REQUEST),'--surface-identity',runner.SURFACE_ID]
    if (OUT/args.label).exists():raise ValueError('RefuseExistingRealization')
    if args.label=='B':
        a=json.loads((OUT/'A/RESULT.json').read_text(encoding='utf-8'))
        if not a['passed'] or not a['committed']:raise ValueError('ReplayRequiresPassedCandidate')
    pre=preflight();save(OUT/('PREFLIGHT_'+args.label+'.json'),pre)
    budget=pre['resources']['output_limit_bytes'];floor=pre['resources']['minimum_free_bytes']
    if shutil.disk_usage(ROOT).free<floor:raise ValueError('OperationalFreeReserve')
    start=time.time();peak=0;last=0;maximum_output=size(OUT);breach=None
    with (OUT/(args.label+'.stdout.log')).open('w',encoding='utf-8') as stdout,(OUT/(args.label+'.stderr.log')).open('w',encoding='utf-8') as stderr:
        if args.profile=='production':
            command=[sys.executable,'-B',str(__import__('pathlib').Path(__file__).with_name('production_initial_front.py')),*extra]
            if args.resume_initial is not None:command += ['--resume-initial',str(args.resume_initial.resolve())]
        else:
            command=[sys.executable,'-B',str(__import__('pathlib').Path(__file__).with_name('run_live_sf2_first_layer.py')),'--worker',args.label,'--commit',*extra]
        p=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=stderr)
        process=psutil.Process(p.pid)
        while p.poll() is None:
            try:peak=max(peak,process.memory_info().peak_wset)
            except psutil.Error:pass
            now=time.time()
            if now-last>=10:
                total=size(OUT);free=shutil.disk_usage(ROOT).free;maximum_output=max(maximum_output,total)
                save(OUT/(args.label+'.PROGRESS.json'),dict(elapsed_seconds=now-start,peak_worker_wset=peak,output_bytes=total,free_bytes=free,pid=p.pid))
                if total>budget or free<floor:
                    breach=dict(code='OperationalResourceLimit',output_bytes=total,free_bytes=free,limit_bytes=budget,reserve_bytes=floor)
                    for child in process.children(recursive=True):child.terminate()
                    p.terminate();break
                last=now
            time.sleep(1)
        rc=p.wait()
    record=dict(returncode=rc,seconds=time.time()-start,peak_worker_working_set_sampled=peak,maximum_observed_output_bytes=maximum_output,final_free_bytes=shutil.disk_usage(ROOT).free,resource_failure=breach,attempts=1)
    save(OUT/(args.label+'.PROCESS.json'),record);print(json.dumps(record,indent=2))
    return 1 if breach else rc

if __name__=='__main__':raise SystemExit(main())
