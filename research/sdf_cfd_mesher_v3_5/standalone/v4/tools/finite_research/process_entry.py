"""Normal CLI process status and optional operational evidence. Never grants execution."""
from pathlib import Path
import hashlib,json,math,os,sys,time,traceback
import stage_observations as obs
from contextvars import ContextVar

_invocation=ContextVar('direct_cli_invocation',default=None)


def current_invocation():
    return _invocation.get()

def creation_filetime(handle=None):
    import ctypes
    from ctypes import wintypes
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetCurrentProcess.restype=wintypes.HANDLE
    kernel.GetProcessTimes.argtypes=[wintypes.HANDLE,*([ctypes.POINTER(wintypes.FILETIME)]*4)]
    kernel.GetProcessTimes.restype=wintypes.BOOL
    values=[wintypes.FILETIME() for unused in range(4)]
    if not kernel.GetProcessTimes(kernel.GetCurrentProcess() if handle is None else handle,*[ctypes.byref(value) for value in values]):
        raise ctypes.WinError(ctypes.get_last_error())
    return (values[0].dwHighDateTime<<32)|values[0].dwLowDateTime

def note_error(primary,note):
    """Keep the first exception on the owned Python 3.10 and Windows runtimes."""
    method=getattr(primary,'add_note',None)
    if callable(method):method(note)
    else:primary.__dict__.setdefault('secondary_notes',[]).append(note)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):h.update(obs.byte_value(block))
    return h.hexdigest()

def status(result):
    if type(result) is int:
        if not 0<=result<=255:raise ValueError('InvalidCLIProcessStatus')
        return result
    if type(result) is dict:return 1 if result.get('passed') is False else 0
    raise ValueError('UnsupportedCLIResultType')

def write(path,value):
    with Path(path).open('x',encoding='utf8',newline='\n') as stream:
        json.dump(value,stream,sort_keys=True,indent=2,allow_nan=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())

def _execute(call,context_path=None,context_sha256=None,evidence_dir=None):
    if context_path is None:
        if context_sha256 is not None or evidence_dir is not None:raise ValueError('CompleteProcessEvidenceContextRequired')
        value=call();print(json.dumps(value,sort_keys=True,allow_nan=False),flush=True)
        code=status(value)
        return code
    if context_sha256 is None or evidence_dir is None:raise ValueError('CompleteProcessEvidenceContextRequired')
    context_path=Path(context_path).resolve();out=Path(evidence_dir).resolve()
    if sha(context_path)!=context_sha256:raise ValueError('ChangedCLIProcessContext')
    ctx=json.loads(context_path.read_text(encoding='utf8'))
    if ctx.get('schema')!='OwnedCLIProcessContext/1' or str(out)!=ctx['evidence_dir'] or out.exists():raise ValueError('InvalidCLIProcessContext')
    expected_clock='monotonic' if os.name=='nt' else 'unix_epoch'
    if ctx.get('deadline_clock')!=expected_clock:raise ValueError('WrongCLIProcessClock')
    for key in ('deadline','parent_deadline_monotonic'):
        if type(ctx.get(key)) not in (int,float) or not math.isfinite(ctx[key]) or ctx[key]<=0:raise ValueError('InvalidCLIProcessDeadline',key)
    if ctx['deadline_clock']=='monotonic' and ctx['deadline']>ctx['parent_deadline_monotonic']:
        raise ValueError('CLIProcessDeadlineExceedsParent')
    out.mkdir();events=(out/'EVENTS.jsonl').open('x',encoding='utf8');primary=None;secondary=[];started=time.perf_counter();code=1;value=None
    def event(kind,**data):events.write(json.dumps(dict(kind=kind,**data))+'\n');events.flush();os.fsync(events.fileno())
    def tick():
        clock=time.monotonic if ctx['deadline_clock']=='monotonic' else time.time
        if ctx['deadline_clock'] not in ('monotonic','unix_epoch') or clock()>=ctx['deadline']:raise TimeoutError('InheritedCLIProcessDeadline')
    declared={};files={};modules={};record={'passed':False,'operation_result':None};phase='startup'
    try:
        event('begin',pid=os.getpid(),parent_pid=os.getppid(),argv=sys.argv,cwd=str(Path.cwd()))
        if sha(ctx['manifest'])!=ctx['manifest_sha256']:raise ValueError('ChangedCLIProcessManifest')
        declared=json.loads(Path(ctx['manifest']).read_text(encoding='utf8'))
        if os.getppid()!=ctx['parent_pid']:raise ValueError('WrongCLIProcessParent')
        if os.name=='nt':
            import psutil
            image=Path(psutil.Process().exe()).resolve()
        else:image=Path('/proc/self/exe').resolve()
        if str(image)!=ctx['python_executable'] or sha(image)!=ctx['python_sha256']:raise ValueError('WrongCLIProcessImage')
        normalized=list(sys.argv)
        for flag,expected in [('--process-context',str(context_path)),('--process-context-sha256',context_sha256),('--process-evidence-dir',str(out))]:
            if normalized.count(flag)!=1:raise ValueError('AmbiguousCLIProcessEvidenceArgument')
            i=normalized.index(flag)
            if i+1>=len(normalized) or normalized[i+1]!=expected:raise ValueError('ChangedCLIProcessEvidenceArgument')
            del normalized[i:i+2]
        if normalized!=ctx['argv']:raise ValueError('WrongCLIProcessArguments')
        startup=dict(schema='OwnedCLIProcessStartup/1',pid=os.getpid(),parent_pid=os.getppid(),argv=sys.argv,cwd=str(Path.cwd()),
            executable=str(image),executable_sha256=sha(image),file_identity=[image.stat().st_dev,image.stat().st_ino],
            deadline=ctx['deadline'],deadline_clock=ctx['deadline_clock'],parent_deadline_monotonic=ctx['parent_deadline_monotonic'],context_sha256=context_sha256,manifest_sha256=ctx['manifest_sha256'],pythonpath=os.environ.get('PYTHONPATH'))
        if os.name=='nt':
            import ctypes,psutil,native_scope
            image_path=Path(psutil.Process().exe()).resolve();parent=Path(psutil.Process(os.getppid()).exe()).resolve()
            stat=image_path.stat();parent_stat=parent.stat()
            if (not os.path.samefile(image_path,ctx['python_executable'])
                    or dict(device=stat.st_dev,file_id=stat.st_ino,size=stat.st_size)!=ctx['python_file_identity']
                    or not os.path.samefile(parent,ctx['parent_python_executable'])
                    or sha(parent)!=ctx['parent_python_sha256']
                    or dict(device=parent_stat.st_dev,file_id=parent_stat.st_ino,size=parent_stat.st_size)!=ctx['parent_python_file_identity']
                    or not os.path.samefile(psutil.Process().cmdline()[0],ctx['python_executable'])):raise ValueError('WrongObservedCLIImage')
            kernel=ctypes.WinDLL('kernel32',use_last_error=True);kernel.GetCurrentProcess.restype=ctypes.c_void_p
            kernel.IsProcessInJob.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.POINTER(ctypes.c_int)];inside=ctypes.c_int()
            if not kernel.IsProcessInJob(kernel.GetCurrentProcess(),None,ctypes.byref(inside)) or not inside.value:raise ValueError('CLIProcessOutsideJob')
            startup.update(os_image=str(image_path),parent_image=str(parent),parent_image_sha256=sha(parent),windows_in_job=True,transport=native_scope.require_transport())
            startup['creation_filetime']=creation_filetime()
        else:
            os_image=Path('/proc/self/exe').resolve();parent=Path('/proc/'+str(os.getppid())+'/exe').resolve()
            cgroup=Path('/proc/self/cgroup').read_text()
            paths=[line.split(':',2)[2] for line in cgroup.splitlines() if line.startswith('0::')]
            if paths!=[ctx['cgroup']] or str(os_image)!=str(image) or str(parent)!=ctx['parent_python_executable'] or sha(parent)!=ctx['parent_python_sha256']:raise ValueError('CLIProcessOutsideNativeScope')
            startup.update(os_image=str(os_image),parent_image=str(parent),parent_image_sha256=sha(parent),cgroup=cgroup)
        write(out/'STARTUP.json',startup);tick();event('operation_begin');phase='operation'
        if ctx.get('stage_observations') is not None:
            obs.activate_child(ctx['stage_observations'],out/'STAGE_TIMING.json','outer_native',context_sha256)
        with obs.span('native_operation','operation'):
            value=call()
        tick();code=status(value);record.update(passed=code==0,operation_result=value);event('operation_end',returncode=code)
        phase='stdout_publication';print(json.dumps(value,sort_keys=True,allow_nan=False),flush=True)
    except BaseException as error:
        primary=error;record.update(passed=False,primary_error=dict(stage=phase,type=type(error).__name__,summary=str(error)),error=repr(error),error_stage=phase,traceback=traceback.format_exc());code=1
        try:traceback.print_exc()
        except BaseException as e:secondary.append(dict(stage='stderr_publication',error=repr(e)))
        try:event('failure',error_type=type(error).__name__)
        except BaseException as e:secondary.append(dict(stage='failure_event',error=repr(e)))
    finally:
        try:
            modules={name:str(Path(m.__file__).resolve()) for name,m in list(sys.modules.items()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
            files={path:sha(path) for path in set(modules.values())};missing=sorted(set(files)-set(declared));changed=[n for n,h in files.items() if declared.get(n)!=h]
            write(out/'SOURCES.json',dict(modules=modules,files=files,unpinned=missing,changed=changed))
            if missing or changed:raise ValueError('UnheldCLIProcessSource',missing,changed)
        except BaseException as error:
            secondary.append(dict(stage='source_closure',error=repr(error)));code=1;record['passed']=False
        record.update(returncode=code,seconds=time.perf_counter()-started,secondary_errors=secondary,context_sha256=context_sha256)
        try:tick()
        except BaseException as e:secondary.append(dict(stage='final_deadline',error=repr(e)));code=1;record.update(passed=False,returncode=code)
        try:event('completion',returncode=code)
        except BaseException as e:secondary.append(dict(stage='completion_event',error=repr(e)));code=1;record.update(passed=False,returncode=code)
        try:events.close()
        except BaseException as e:secondary.append(dict(stage='events_close',error=repr(e)));code=1;record.update(passed=False,returncode=code)
        try:
            write(out/'RESULT.json',record)
            links={n:sha(out/n) for n in ('STARTUP.json','SOURCES.json','RESULT.json') if (out/n).is_file()}
            write(out/'COMPLETION.json',dict(returncode=code,passed=code==0,pid=os.getpid(),parent_pid=os.getppid(),files=links,deadline=ctx['deadline'],deadline_clock=ctx['deadline_clock'],parent_deadline_monotonic=ctx['parent_deadline_monotonic']))
            observation_finished=obs.finish('returned:'+str(code))
            admission=sys.modules.get('finite_research.direct_admission')
            if admission is not None and admission.has_pending() and obs.enabled():
                if observation_finished is not True or obs._active.incomplete or not obs._active.finished:
                    raise ValueError('IncompleteDirectCLIObservationClosure')
            tick()  # Same final deadline check, after diagnostic I/O as well.
        except BaseException as publication_error:
            if primary is not None:
                note_error(primary,'Process evidence publication: '+repr(publication_error));raise primary
            raise
    return code


def failure_outcome(code,invocation,root,*,synthetic):
    from contextlib import ExitStack
    from immutable_state_access import guard
    from finite_research import direct_publication as publication
    if code in (None,0) or not invocation['completed'] or not invocation['files']:
        raise ValueError('CompletedFailedInvocationRequired')
    out=Path(invocation['evidence_dir']).resolve()
    with ExitStack() as held:
        for path,digest in invocation['files'].items():
            held.enter_context(guard(path))
            if sha(path)!=digest:
                raise ValueError('ChangedFailedInvocationClosure')
        operation=json.loads((out/'RESULT.json').read_bytes())
        completion=json.loads((out/'COMPLETION.json').read_bytes())
        startup=json.loads((out/'STARTUP.json').read_bytes())
        sources=json.loads((out/'SOURCES.json').read_bytes())
        if (operation['returncode']!=code or completion['returncode']!=code
            or operation['passed'] is not False or completion['passed'] is not False
            or completion['pid']!=os.getpid() or startup['pid']!=os.getpid()
            or startup['context_sha256']!=invocation['context_sha256']
            or operation['context_sha256']!=invocation['context_sha256']
            or sources['unpinned'] or sources['changed']
            or set(completion['files'])!={'STARTUP.json','SOURCES.json','RESULT.json'}
            or any(sha(out/name)!=digest for name,digest in completion['files'].items())):
            raise ValueError('UnauthenticatedFailedInvocation')
        with publication.writer(root):
            if any((root/name).exists() for name in ('DIRECT_PUBLICATION_PREPARED.json','DIRECT_PUBLICATION_COMMITTED.json')):
                raise ValueError('ContradictoryPrepublicationFailure')
            previous=publication.snapshot(root/'CURRENT.json')
        return dict(publication_state='NOT_COMMITTED',phase='BEFORE_PUBLICATION',root=str(root),
            publication_started=False,admission_issued=False,synthetic_workload=synthetic,
            controller_creation_filetime=startup['creation_filetime'],closure_files=dict(invocation['files']),
            previous_current_sha256=hashlib.sha256(previous).hexdigest() if previous is not None else None)


def write_failed_terminal(code,failed,context_path,context_sha256,evidence_dir):
    try:
        write(Path(evidence_dir)/'SUPERVISOR_FAILURE.json',dict(publication=failed,
            operation_returncode=code,context_sha256=context_sha256,pid=os.getpid(),
            status='FAILED',normal_run_qualified=False,continuation_invalidated=True))
        context=json.loads(Path(context_path).read_bytes())
        if time.monotonic()>=min(context['deadline'],context['parent_deadline_monotonic']):
            raise TimeoutError('SupervisorTerminalEvidenceDeadline')
    except BaseException as error:
        error.direct_publication_outcome=failed
        error.operation_returncode=code
        error.operation_result_sha256=failed['closure_files'][str(Path(evidence_dir).resolve()/'RESULT.json')]
        note_error(error,'Secondary failure while reporting original nonzero operation '+str(code)
            +'; original RESULT.json sha256='+error.operation_result_sha256)
        raise


def execute(call,context_path=None,context_sha256=None,evidence_dir=None):
    if _invocation.get() is not None:
        raise ValueError('NestedCLIProcessInvocation')
    invocation=dict(identity=object(),context_path=context_path,context_sha256=context_sha256,
        evidence_dir=evidence_dir,completed=False,files={})
    reset=_invocation.set(invocation)
    code=None
    failed=None
    try:
        code=_execute(call,context_path,context_sha256,evidence_dir)
        if context_path is not None:
            out=Path(evidence_dir).resolve()
            invocation['files']={str(path):sha(path) for path in
                (Path(context_path).resolve(),out/'STARTUP.json',out/'SOURCES.json',out/'RESULT.json',out/'COMPLETION.json')}
        invocation['completed']=True
        admission=sys.modules.get('finite_research.direct_admission')
        probe=sys.modules.get('finite_research.ownership_probe')
        if code!=0:
            failed=admission.failed_process(code,invocation) if admission is not None else None
            if probe is not None:
                synthetic=probe.failed_process(code,invocation)
                if synthetic is not None:
                    if failed is not None:
                        raise ValueError('ConflictingFailedContinuations')
                    failed=synthetic
    finally:
        _invocation.reset(reset)
        admission=sys.modules.get('finite_research.direct_admission')
        if admission is not None and (code!=0 or not invocation['completed']):
            admission.invalidate_process(invocation)
    admission=sys.modules.get('finite_research.direct_admission')
    probe=sys.modules.get('finite_research.ownership_probe')
    if failed is not None:
        write_failed_terminal(code,failed,context_path,context_sha256,evidence_dir)
        return code
    if admission is not None or probe is not None:
        publication=None
        try:
            publication=admission.complete_process(code,invocation) if admission is not None else None
            if probe is not None:
                synthetic=probe.complete_process(code,invocation)
                if synthetic is not None:
                    if publication is not None:
                        raise ValueError('ConflictingTerminalContinuations')
                    publication=synthetic
            if publication is not None:
                write(Path(evidence_dir)/'SUPERVISOR_RETURN.json',dict(publication=publication,
                    operation_returncode=code,context_sha256=context_sha256,pid=os.getpid(),
                    status='AWAITING_EXTERNAL_TERMINAL',normal_run_qualified=False))
                context=json.loads(Path(context_path).read_bytes())
                if time.monotonic()>=min(context['deadline'],context['parent_deadline_monotonic']):
                    raise TimeoutError('SupervisorTerminalEvidenceDeadline')
        except BaseException as error:
            publication=getattr(error,'direct_publication_outcome',publication)
            if evidence_dir is not None:
                try:
                    write(Path(evidence_dir)/'SUPERVISOR_FAILURE.json',dict(error=repr(error),
                        publication=publication,context_sha256=context_sha256,pid=os.getpid(),
                        status='FAILED',normal_run_qualified=False))
                except BaseException as secondary:
                    note_error(error,'Supervisor terminal evidence: '+repr(secondary))
            raise
    return code
