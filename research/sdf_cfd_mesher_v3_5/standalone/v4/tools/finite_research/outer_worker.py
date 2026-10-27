"""Existing outer worker under the current owned process/native envelope."""
import stage_observations as obs
from contextlib import ExitStack
from pathlib import Path
import json,subprocess,time,uuid
from finite_research.inputs import V4,read,sha,write,native_sources

def linux(path):
    value=str(path).replace('\\','/')
    return '/mnt/'+value[0].lower()+value[2:] if len(value)>2 and value[1]==':' else value

@obs.observed('outer.invoke','child_lifecycle')
def invoke(root,arguments,check):
    import native_scope
    from immutable_state_access import guard
    check();transport=native_scope.require_transport()
    root=Path(root)/('outer_native_'+uuid.uuid4().hex);root.mkdir()
    manifest=root/'DECLARED_SOURCES.json';context=root/'CONTEXT.json';evidence=root/'evidence'
    declarations=native_sources()
    declarations.update({linux(p):h for p,h in list(declarations.items())})
    write(manifest,declarations)
    image=read(V4/'runtime/FINITE_NATIVE_BINDING.json')['executable']
    argv=[linux(V4/'tools/finite_cli.py'),'outer-backend','--release-sha256',sha(V4/'runtime/RELEASE.json'),*arguments]
    ctx=dict(schema='OwnedCLIProcessContext/1',evidence_dir=linux(evidence),manifest=linux(manifest),manifest_sha256=sha(manifest),
        parent_pid=transport['broker']['pid'],argv=argv,deadline_clock='unix_epoch',
        deadline=time.time()+check.remaining_seconds(),parent_deadline_monotonic=check.deadline,
        python_executable=image,python_sha256=declarations[image],parent_python_executable=image,
        parent_python_sha256=declarations[image],cgroup=transport['cgroup'])
    diagnostic=obs.child_binding(str(root))
    if diagnostic is not None:ctx['stage_observations']=diagnostic
    write(context,ctx)
    argv+=['--process-context',linux(context),'--process-context-sha256',sha(context),'--process-evidence-dir',linux(evidence)]
    command=['wsl.exe','-e','env','PYTHONDONTWRITEBYTECODE=1',
        'PYTHONPATH='+':'.join(linux(V4/p) for p in ('tools','src','runtime/lib/finite_native_packages','runtime/lib/gmsh')),image,'-B',*argv]
    held=ExitStack();primary=None
    try:
        held.enter_context(guard(manifest));held.enter_context(guard(context))
        out=held.enter_context((root/'stdout.txt').open('x',encoding='utf8'))
        err=held.enter_context((root/'stderr.txt').open('x',encoding='utf8'))
        try:
            with obs.span('child_transport_start_wait','child_wait'):
                process=subprocess.run(command,stdout=out,stderr=err,timeout=max(.001,check.remaining_seconds()))
        except BaseException as error:
            try:write(root/'WAIT_ERROR.json',dict(error=repr(error),command=command,primary=True))
            except BaseException as secondary:error.add_note('Outer wait evidence: '+repr(secondary))
            raise
        available={};secondary=[]
        for name in ('STARTUP.json','SOURCES.json','RESULT.json','COMPLETION.json'):
            try:
                if (evidence/name).is_file():
                    held.enter_context(guard(evidence/name))
                    available[name]=dict(sha256=sha(evidence/name),value=read(evidence/name))
            except BaseException as error:secondary.append(dict(file=name,error=repr(error)))
        child_error=(ValueError('OuterBackendChildStopped',str(root),process.returncode,
            available.get('RESULT.json',{}).get('value',{}).get('primary_error')) if process.returncode else None)
        try:
            write(root/'CHILD_RETURN.json',dict(returncode=process.returncode,
                available_receipts={n:v['sha256'] for n,v in available.items()},
                primary_error=available.get('RESULT.json',{}).get('value',{}).get('primary_error'),secondary_errors=secondary))
        except BaseException as error:
            if child_error is None:raise
            child_error.add_note('Outer return evidence: '+repr(error))
        if child_error is not None:raise child_error
        if secondary or len(available)!=4:raise ValueError('OuterBackendChildEvidenceIncomplete',str(root),secondary)
        check()
        start=read(evidence/'STARTUP.json');sources=read(evidence/'SOURCES.json')
        result=read(evidence/'RESULT.json');end=read(evidence/'COMPLETION.json')
        if (process.returncode!=0 or result.get('passed') is not True or end.get('passed') is not True
                or result['returncode']!=0 or end['returncode']!=0 or result['secondary_errors']
                or set(end['files'])!={'STARTUP.json','SOURCES.json','RESULT.json'}
                or any(sha(evidence/n)!=h for n,h in end['files'].items())
                or start['pid']!=end['pid'] or start['parent_pid']!=ctx['parent_pid']
                or start['argv']!=argv or start['context_sha256']!=sha(context)
                or result['context_sha256']!=sha(context) or start['manifest_sha256']!=sha(manifest)
                or end['parent_deadline_monotonic']!=check.deadline or start['parent_deadline_monotonic']!=check.deadline
                or sources['unpinned'] or sources['changed']
                or any(declarations.get(p)!=h for p,h in sources['files'].items())):
            raise ValueError('OuterBackendProcessAdmissionFailure',str(root),process.returncode,result.get('primary_error'))
        value=result['operation_result'];check()
    except BaseException as error:
        primary=error
        raise
    finally:
        try:held.close()
        except BaseException as error:
            if primary is None:raise
            primary.add_note('Outer process custody close: '+repr(error))
    return value

def selected_probe(method):
    from finite_research.prism_connection import terminal_execution_context
    root,check=terminal_execution_context()
    arguments=['--probe']+([] if method is None else ['--method',method])
    return invoke(root,arguments,check)
