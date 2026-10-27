from pathlib import Path
import ctypes,json,os,subprocess,sys,time

def validate_failed_terminal(terminal,operation,completion,context_path,context_sha256,evidence,root,code,creation,synthetic,pid):
    import hashlib
    from contextlib import ExitStack
    from immutable_state_access import guard
    from finite_research import direct_publication as publication
    captured=terminal['publication']
    required={str(path.resolve()) for path in (context_path,evidence/'STARTUP.json',evidence/'SOURCES.json',
        evidence/'RESULT.json',evidence/'COMPLETION.json')}
    if (code==0 or terminal.get('operation_returncode')!=code or operation['returncode']!=code
        or completion['returncode']!=code or operation['passed'] is not False or completion['passed'] is not False
        or terminal.get('continuation_invalidated') is not True or captured.get('publication_started') is not False
        or captured.get('admission_issued') is not False or captured.get('publication_state')!='NOT_COMMITTED'
        or captured.get('root')!=str(root) or captured.get('synthetic_workload') is not synthetic
        or captured.get('controller_creation_filetime')!=creation or set(captured.get('closure_files',{}))!=required
        or captured['closure_files'].get(str(context_path))!=context_sha256
        or (evidence/'SUPERVISOR_RETURN.json').exists()):
        raise ValueError('InvalidBeforePublicationFailure')
    with ExitStack() as held:
        for path,digest in captured['closure_files'].items():
            held.enter_context(guard(path))
            if publication.file_sha(path)!=digest:
                raise ValueError('ChangedBeforePublicationClosure')
        held.enter_context(guard(evidence/'SUPERVISOR_FAILURE.json'))
        if json.loads((evidence/'SUPERVISOR_FAILURE.json').read_bytes())!=terminal:
            raise ValueError('ChangedBeforePublicationTerminal')
        context=json.loads(context_path.read_bytes())
        manifest_path=Path(context['manifest'])
        held.enter_context(guard(manifest_path))
        if publication.file_sha(manifest_path)!=context['manifest_sha256']:
            raise ValueError('ChangedControllerManifest')
        manifest=json.loads(manifest_path.read_bytes())
        startup=json.loads((evidence/'STARTUP.json').read_bytes())
        sources=json.loads((evidence/'SOURCES.json').read_bytes())
        if (json.loads((evidence/'RESULT.json').read_bytes())!=operation
            or json.loads((evidence/'COMPLETION.json').read_bytes())!=completion
            or startup['pid']!=pid or startup['parent_pid']!=os.getpid()
            or completion['pid']!=pid or completion['parent_pid']!=os.getpid()
            or context['parent_pid']!=os.getpid() or startup.get('creation_filetime')!=creation
            or startup['context_sha256']!=context_sha256 or startup['manifest_sha256']!=context['manifest_sha256']
            or operation['context_sha256']!=context_sha256 or sources['unpinned'] or sources['changed']
            or any(manifest.get(name)!=digest for name,digest in sources['files'].items())
            or set(completion['files'])!={'STARTUP.json','SOURCES.json','RESULT.json'}
            or any(publication.file_sha(evidence/name)!=digest for name,digest in completion['files'].items())):
            raise ValueError('ChangedGuardedFailureSemantics')
        with publication.writer(root):
            if any((root/name).exists() for name in ('DIRECT_PUBLICATION_PREPARED.json','DIRECT_PUBLICATION_COMMITTED.json')):
                raise ValueError('ContradictoryPrepublicationFailure')
            previous=publication.snapshot(root/'CURRENT.json')
            digest=hashlib.sha256(previous).hexdigest() if previous is not None else None
            if captured.get('previous_current_sha256')!=digest:
                raise ValueError('ChangedBeforePublicationCurrent')
        return dict(terminal_outcome_sha256=publication.file_sha(evidence/'SUPERVISOR_FAILURE.json'),
            operation_completion_sha256=captured['closure_files'][str(evidence/'COMPLETION.json')],
            operation_result_sha256=captured['closure_files'][str(evidence/'RESULT.json')])


def terminal_readback(process,context_path,context_sha256,root,check,*,synthetic=False):
    from finite_research import direct_publication as publication,process_entry
    from immutable_state_access import guard
    check()
    context_path=Path(context_path).resolve()
    root=Path(root).resolve()
    with guard(context_path):
        if publication.file_sha(context_path)!=context_sha256:
            raise ValueError('ChangedControllerLifecycleContext')
        context=json.loads(context_path.read_bytes())
    if os.name!='nt' or context['parent_pid']!=os.getpid():
        raise ValueError('ActualExternalWindowsControllerParentRequired')
    code=process.poll()
    if code is None:
        raise ValueError('ControllerNotTerminal')
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetProcessId.argtypes=[ctypes.c_void_p]
    kernel.GetProcessId.restype=ctypes.c_ulong
    kernel.GetExitCodeProcess.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_ulong)]
    kernel.GetExitCodeProcess.restype=ctypes.c_int
    observed=ctypes.c_ulong()
    if (kernel.GetProcessId(int(process._handle))!=process.pid
        or not kernel.GetExitCodeProcess(int(process._handle),ctypes.byref(observed))
        or observed.value!=code or code==259):
        raise ValueError('ControllerTerminalHandleMismatch')
    creation=process_entry.creation_filetime(int(process._handle))
    evidence=Path(context['evidence_dir'])
    result=dict(schema='DirectExternalLifecycle/1',passed=False,controller_pid=process.pid,
        controller_creation_filetime=creation,controller_returncode=code,
        controller_state='SUCCEEDED' if code==0 else 'FAILED',publication_state='UNRESOLVED',
        context_sha256=context_sha256,cold_readback_state='NOT_RUN',synthetic_workload=synthetic,
        normal_run_qualified=False,solver_ready=False,enclosing_owner_terminal_required=True)
    errors=[]
    primary=None
    try:
        manifest_path=Path(context['manifest'])
        if publication.file_sha(manifest_path)!=context['manifest_sha256']:
            raise ValueError('ChangedControllerManifest')
        manifest=json.loads(manifest_path.read_bytes())
        completion=json.loads((evidence/'COMPLETION.json').read_bytes())
        if (completion['pid']!=process.pid or completion['parent_pid']!=os.getpid()
            or set(completion.get('files',{}))!={'STARTUP.json','SOURCES.json','RESULT.json'}):
            raise ValueError('ChangedControllerCompletionIdentityOrFiles')
        for name,digest in completion['files'].items():
            if publication.file_sha(evidence/name)!=digest:
                raise ValueError('ChangedControllerOperationClosure')
        startup=json.loads((evidence/'STARTUP.json').read_bytes())
        operation=json.loads((evidence/'RESULT.json').read_bytes())
        sources=json.loads((evidence/'SOURCES.json').read_bytes())
        if (startup['pid']!=process.pid or startup['parent_pid']!=os.getpid()
            or startup.get('creation_filetime')!=creation or startup['context_sha256']!=context_sha256
            or startup['manifest_sha256']!=context['manifest_sha256']
            or operation['context_sha256']!=context_sha256 or sources['unpinned'] or sources['changed']
            or any(manifest.get(name)!=digest for name,digest in sources['files'].items())):
            raise ValueError('ChangedControllerContextOrSourceClosure')
        terminal_name='SUPERVISOR_RETURN.json' if code==0 else 'SUPERVISOR_FAILURE.json'
        terminal=json.loads((evidence/terminal_name).read_bytes())
        if terminal['context_sha256']!=context_sha256 or terminal['pid']!=process.pid or terminal['normal_run_qualified'] is not False:
            raise ValueError('ChangedControllerTerminalBinding')
        if code==0 and (terminal['status']!='AWAITING_EXTERNAL_TERMINAL' or terminal['operation_returncode']!=0
            or operation['returncode']!=0 or operation['passed'] is not True or completion['returncode']!=0
            or completion['passed'] is not True or (evidence/'SUPERVISOR_FAILURE.json').exists()):
            raise ValueError('FailedControllerCannotPassLifecycle')
        if code!=0 and terminal['status']!='FAILED':
            raise ValueError('ControllerFailureEvidenceRequired')
        result['terminal_outcome_sha256']=publication.file_sha(evidence/terminal_name)
        result['operation_completion_sha256']=publication.file_sha(evidence/'COMPLETION.json')
        if terminal.get('publication',{}).get('phase')=='PARTIAL_BY_REQUEST':
            from finite_research.partial_stages import external
            partial=external(terminal['publication'],context,root,manifest,code)
            result.update(passed=True,publication_state='NOT_RUN_BY_REQUEST',cold_readback_state='NOT_RUN_BY_REQUEST',
                requested_stop_after=partial['requested_stop_after'],completed_stage=partial['completed_stage'],full_mesh_complete=False)
        elif terminal.get('publication',{}).get('phase')=='BEFORE_PUBLICATION':
            verified=validate_failed_terminal(terminal,operation,completion,context_path,context_sha256,evidence,root,code,creation,synthetic,process.pid)
            result.update(publication_state='NOT_COMMITTED',cold_readback_state='NOT_RUN',
                failure_phase='BEFORE_PUBLICATION',**verified)
        else:
            prepared=root/'DIRECT_PUBLICATION_PREPARED.json'
            prepared_sha256=publication.file_sha(prepared)
            journal=json.loads(prepared.read_bytes())
            if journal['supervisor']['pid']!=process.pid or journal['supervisor']['creation_filetime']!=creation:
                raise ValueError('WrongPublicationSupervisor')
            captured=terminal.get('publication')
            if not isinstance(captured,dict):
                raise ValueError('MissingControllerPublicationOutcome')
            commit=publication.committed(root,journal)
            if commit:
                if captured.get('publication_state')!='COMMITTED' or captured.get('publication_id')!=commit['publication_id']:
                    raise ValueError('ControllerCommitIdentityMismatch')
                if captured.get('commit_sha256') is not None:
                    if captured['commit_sha256']!=publication.file_sha(root/'DIRECT_PUBLICATION_COMMITTED.json'):
                        raise ValueError('ChangedControllerCommitReceipt')
                elif captured.get('commit')!=commit:
                    raise ValueError('MissingCapturedControllerCommitReceipt')
                result.update(publication_state='COMMITTED',commit=commit,prepared_sha256=prepared_sha256)
            elif captured.get('publication_state')!='NOT_COMMITTED':
                raise ValueError('ControllerNoncommitIdentityMismatch')
            recovered=publication.recover(root,prepared_sha256,check)
            result.update(publication_state=recovered['publication_state'],prepared_sha256=prepared_sha256)
            reader=Path(__file__).with_name('direct_readback_entry.py')
            if manifest.get(str(reader))!=publication.file_sha(reader):
                raise ValueError('UnboundExternalReaderSource')
            output=evidence/'COLD_READBACK.json'
            argv=[sys.executable,'-B',str(reader),str(root),prepared_sha256,str(output),
                str(context_path),context_sha256,'synthetic' if synthetic else 'scientific']
            remaining=min(check.remaining_seconds(),context['parent_deadline_monotonic']-time.monotonic())
            if remaining<=1:
                raise TimeoutError('ExternalReaderOwnerDeadline')
            with (evidence/'COLD_READBACK.stdout').open('x') as stdout,(evidence/'COLD_READBACK.stderr').open('x') as stderr:
                reader_process=subprocess.run(argv,stdout=stdout,stderr=stderr,timeout=remaining)
            result['cold_reader_returncode']=reader_process.returncode
            cold=json.loads(output.read_bytes()) if output.is_file() else None
            result.update(cold_readback_state='PASSED' if reader_process.returncode==0 and cold and cold.get('passed') else 'FAILED',
                cold_readback_sha256=publication.file_sha(output) if output.exists() else None)
            check()
            result['passed']=(code==0 and result['publication_state']=='COMMITTED' and result['cold_readback_state']=='PASSED')
    except BaseException as error:
        recovered_outcome=getattr(error,'direct_publication_outcome',None)
        if recovered_outcome and recovered_outcome.get('publication_state')!='UNRESOLVED':
            result.update(publication_state=recovered_outcome['publication_state'],recovery_outcome=recovered_outcome)
        primary=error
        errors.append(repr(error))
        result.update(passed=False,error=repr(error),cold_readback_state='FAILED_OR_NOT_REACHED')
    result['errors']=errors
    try:
        process_entry.write(evidence/'EXTERNAL_LIFECYCLE.json',result)
        check()
    except BaseException as error:
        result.update(passed=False,terminal_error=repr(error))
        if primary is None:
            primary=error
        else:
            primary.add_note('External lifecycle evidence: '+repr(error))
        try:
            process_entry.write(evidence/'EXTERNAL_LIFECYCLE_FAILURE.json',result)
        except BaseException as secondary:
            primary.add_note('External lifecycle failure evidence: '+repr(secondary))
    if primary is not None:
        primary.direct_publication_outcome=result
        raise primary
    return result
