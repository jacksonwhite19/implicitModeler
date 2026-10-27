"""Private routing-only copy; never import as a separate command owner."""
def _supervise_guarded(args):
    """One command/one worker, monitored throughout; no scientific retries."""
    import subprocess
    import psutil
    case,_,_=load(args.case);output=args.output.resolve()
    if output.exists():raise ValueError('OutputMustBeNew')
    output.parent.mkdir(parents=True,exist_ok=True)
    log=output.parent/(output.name+'.execution.log')
    if log.exists():raise ValueError('PreserveExecutionLog')
    from finite_research.worker_arguments import build
    command=build(args,_finite_collar_worker_prefix,retained_arguments)
    resources=effective_resources(case['resources'])
    start=time.perf_counter();peak=0;child_peak=0;breach=None
    with log.open('w',encoding='utf-8',newline='\n') as stream:
        import os
        env=dict(os.environ,V4_STANDALONE_SUPERVISOR=str(os.getpid()),
                 V4_STANDALONE_CASE_SHA256=file_hash(args.case),
                 V4_STANDALONE_IMPORTED_SOURCE_SHA256=held_import_sha256(args.case) or '')
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,env=env)
        observed=psutil.Process(process.pid)
        try:
          while process.poll() is None:
            try:
                peak=max(peak,observed.memory_info().rss)
                child_peak=max(child_peak,max((p.memory_info().rss for p in observed.children(recursive=True)),default=0))
                location=output if output.exists() else output.parent
                data=check_resources(location,case['resources']) if output.exists() else dict(
                    logical_bytes=0,free_bytes=shutil.disk_usage(location).free)
                if data['free_bytes']<resources['minimum_free_bytes']:raise ValueError('OperationalResourceLimit')
                if output.exists():write_monitor(output/'MONITOR.json',dict(data,elapsed_s=time.perf_counter()-start,
                    sampled_worker_rss_peak=peak,sampled_child_rss_peak=child_peak))
            except psutil.Error:pass
            except ValueError as error:
                breach=str(error)
                try:terminate_worker(output,case,observed,process)
                except BaseException as cancellation:breach+='; core cancellation: '+str(cancellation)
                break
            time.sleep(1)
          code=process.wait()
        finally:
          if process.poll() is None:
            terminate_worker(output,case,observed,process)
    if output.exists():
        record=dict(returncode=code,seconds=time.perf_counter()-start,
            sampled_worker_rss_peak=peak,sampled_child_rss_peak=child_peak,resource_failure=breach)
        durable_json(output/'PROCESS.json',record)
        try:check_resources(output,case['resources'])
        except ValueError as error:
            breach=str(error);record['resource_failure']=breach
            durable_json(output/'PROCESS.json',record)
    if code or breach:raise ValueError('MeshWorkerStopped',code,breach,str(log))
