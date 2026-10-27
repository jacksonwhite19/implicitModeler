from pathlib import Path
from types import SimpleNamespace
import hashlib,hmac,json,os,secrets,time

def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()

def identity(pid):
    import psutil
    process=psutil.Process(pid)
    image=Path(process.exe()).resolve()
    return dict(pid=pid,created=process.create_time(),image=str(image),image_sha256=digest(image),argv=process.cmdline())

def read(path):
    return json.loads(Path(path).read_text(encoding='utf8'))

def require_cli(scope,delegate):
    import psutil
    from immutable_state_access import guard
    process=psutil.Process(delegate['pid'])
    argv=delegate['argv']
    entry=Path(__file__).resolve().parents[2]/'finite_cli.py'
    if len(argv)<4 or argv[1]!='-B' or Path(argv[2]).resolve()!=entry or argv[3] not in ('run','ownership-probe'):
        raise ValueError('DesignatedOrdinaryCLIDelegateRequired')
    if scope.delegation_pins.get(str(entry))!=digest(entry):
        raise ValueError('UnpinnedDesignatedCLI')
    def argument(flag):
        if argv.count(flag)!=1:
            raise ValueError('AmbiguousDelegateContextArgument')
        return argv[argv.index(flag)+1]
    path=Path(argument('--process-context')).resolve()
    with guard(path):
        content=path.read_bytes()
        if hashlib.sha256(content).hexdigest()!=argument('--process-context-sha256'):
            raise ValueError('ChangedDelegateContext')
        context=json.loads(content)
        manifest=Path(context['manifest']).resolve()
        if (context.get('schema')!='OwnedCLIProcessContext/1' or context['parent_pid']!=process.ppid()
            or scope.delegation_pins.get(str(manifest))!=context['manifest_sha256']
            or digest(manifest)!=context['manifest_sha256']
            or context['deadline_clock']!='monotonic'
            or context['deadline']>context['parent_deadline_monotonic']
            or context['python_sha256']!=delegate['image_sha256']):
            raise ValueError('UnboundDesignatedCLIContext')
        return dict(context_sha256=hashlib.sha256(content).hexdigest(),context=context)

def request(scope,action):
    import native_scope
    capability=native_scope.windows_admission()
    try:
        capability.check()
    finally:
        capability.close()
    if scope.server_api.live_identity(scope.server_handle,scope.parent_descriptor['windows_job']['owner']['pid'])!=scope.parent_descriptor['windows_job']['owner']:
        raise ValueError('ChangedDelegationServerIdentity')
    if scope.channel.peer_pid()!=scope.parent_descriptor['windows_job']['owner']['pid']:
        raise ValueError('WrongDelegationServerProcess')
    value=dict(schema='NativeScopeDelegation/2',action=action,nonce=scope.nonce,
        descriptor_sha256=scope.parent_digest,binding=scope.binding,
        delegate=scope.delegate,deadline=scope.windows_deadline)
    scope.channel.send(value)
    while time.perf_counter()<scope.windows_deadline:
        result=scope.channel.receive()
        if result is not None:
            if result.get('passed') is not True or result.get('request')!=value:
                raise ValueError('DelegatedScopeOwnerRefusal',result)
            if action=='BORROW':
                scope.nonce=result['nonce']
                scope.job_token=result['job_token']
            elif result.get('nonce')!=scope.nonce:
                raise ValueError('ChangedDelegatedRetirementCapability')
            return result
        time.sleep(.025)
    raise TimeoutError('DelegatedScopeOwnerDeadline')

def advance(scope,job,check_resources):
    import native_scope
    state=getattr(scope,'delegation',None)
    if state is not None and state.get('retired'):
        return
    value=scope.delegation_channel.receive()
    if value is None:
        return
    action='BORROW' if state is None else 'RETIRE'
    peer=scope.delegation_channel.peer_pid()
    if (value.get('schema')!='NativeScopeDelegation/2' or value.get('action')!=action
        or value.get('descriptor_sha256')!=digest(scope.descriptor) or value.get('binding')!=scope.binding
        or type(value.get('deadline')) not in (int,float)
        or not time.perf_counter()<value['deadline']<=scope.windows_deadline
        or value.get('delegate',{}).get('pid')!=peer):
        raise ValueError('UnauthenticatedScopeDelegation')
    actual=identity(peer)
    if actual!=value['delegate']:
        raise ValueError('ChangedScopeDelegateIdentity')
    import psutil
    if os.getpid() not in [parent.pid for parent in psutil.Process(peer).parents()]:
        raise ValueError('ScopeDelegateIsNotDescendant')
    if state is None:
        api=native_scope.windows_job_module().Api()
        handle=api.k.OpenProcess(0x1000|0x100000,False,peer)
        if not handle:
            raise ValueError('ScopeDelegateProcessHandleUnavailable')
        scope.delegation_handle=(api,handle)
        state=dict(request=value,delegate=actual,retired=False,nonce=secrets.token_hex(32),
            context=require_cli(scope,actual),job_token=secrets.token_hex(32),windows_identity=api.live_identity(handle,peer))
        scope.delegation=state
    api,handle=scope.delegation_handle
    if api.live_identity(handle,peer)!=state['windows_identity']:
        raise ValueError('ChangedLiveDelegateHandleIdentity')
    job.verify(SimpleNamespace(pid=peer,_handle=handle))
    if identity(peer)!=state['delegate']:
        raise ValueError('ChangedAdmittedScopeDelegate')
    check_resources()
    if action=='BORROW':
        if value.get('nonce') is not None:
            raise ValueError('OwnerIssuedDelegationRequired')
        scope.privileged_control('delegate',[state['job_token']])
        result=dict(passed=True,request=value,binding=scope.binding,nonce=state['nonce'],job_token=state['job_token'])
    else:
        original=state['request']
        if (value.get('nonce')!=state['nonce']
            or any(value[key]!=original[key] for key in ('delegate','deadline','descriptor_sha256','binding'))):
            raise ValueError('ChangedScopeRetirementDelegate')
        if value['deadline']-time.perf_counter()<30:
            raise TimeoutError('DelegatedRetirementReserveRequired')
        scope.privileged_control('close_admission',[])
        retirement=scope.close()
        if (retirement.get('passed') is not True or retirement['after']['live_members']
            or not retirement['after']['retired'] or retirement.get('memory_error')
            or retirement.get('memory_before_retirement') is None):
            raise ValueError('DelegatedRetirementNotIndependentlyConfirmed')
        state['retired']=True
        result=dict(passed=True,request=value,nonce=state['nonce'],binding=scope.binding,
            retirement_path=str(scope.folder/'RETIREMENT.json'),
            retirement_sha256=digest(scope.folder/'RETIREMENT.json'),retirement=retirement)
    check_resources()
    native_scope.write(scope.folder/('DELEGATION_'+action+'.json'),value)
    native_scope.write(scope.folder/('DELEGATION_'+action+'_ACK.json'),result)
    scope.delegation_channel.send(result)

class BorrowedScope:
    def __init__(self,folder,runtime_seconds,resources,windows_job,windows_deadline,parent_path):
        import native_scope
        self.folder=Path(folder).resolve()
        self.folder.mkdir()
        self.parent_path=Path(parent_path)
        self.parent_digest=digest(self.parent_path)
        self.parent_descriptor=read(self.parent_path)
        self.parent_root=Path(self.parent_descriptor['root'])
        self.binding=read(self.parent_root/'BINDING.json')
        self.config=read(self.parent_root/'CONFIG.json')
        self.resources=resources
        self.windows_job=windows_job
        self.windows_deadline=min(windows_deadline,self.parent_descriptor['windows_job']['deadline'])
        self.runtime_seconds=runtime_seconds
        self.retirement=None
        self.descriptor=self.folder/'DESCRIPTOR.json'
        self.nonce=None
        self.channel=None
        self.server_api=None
        self.server_handle=None
        self.delegate=identity(os.getpid())
        if (self.config.get('resources')!=resources or self.parent_descriptor.get('delegation')!='exclusive_whole_scope_retirement_v2'
            or native_scope.rpc(self.parent_descriptor,dict(op='ping'))['binding']!=self.binding):
            raise ValueError('BorrowedScopeIdentityOrLimitsMismatch')
        native_scope.write(self.folder/'CONFIG.json',self.config)
        native_scope.write(self.folder/'BINDING.json',self.binding)

    def start(self):
        import native_scope
        from delegation_pipe import Pipe
        self.server_api=native_scope.windows_job_module().Api()
        self.server_handle=self.server_api.k.OpenProcess(0x1000|0x100000,False,self.parent_descriptor['windows_job']['owner']['pid'])
        if not self.server_handle:
            raise ValueError('DelegationServerProcessUnavailable')
        self.channel=Pipe(self.parent_descriptor['delegation_pipe'])
        try:
            self.channel.connect(self.windows_deadline)
            acknowledgement=request(self,'BORROW')
            descriptor=dict(self.parent_descriptor)
            descriptor['windows_job']=native_scope.windows_job_module().make_grant(self.windows_job,self.windows_deadline)
            descriptor['job_token']=self.job_token
            descriptor.pop('delegation',None)
            descriptor.pop('delegation_pipe',None)
            native_scope.write(self.descriptor,descriptor)
            native_scope.write(self.folder/'READY.json',dict(binding=self.binding,borrowed=True,acknowledgement=acknowledgement))
            native_scope.write(self.folder/'START_VALIDATED.json',dict(binding=self.binding,borrowed=True,parent_descriptor_sha256=self.parent_digest))
            self.memory_start=self.memory()
            return self
        except BaseException:
            self.close_delegation()
            raise

    def memory(self):
        import native_scope
        result=native_scope.rpc(self.parent_descriptor,dict(op='control',action='memory',extra=[]),timeout=4)
        if (result['memory_max_bytes']!=self.resources['linux_memory_max_bytes']
            or result['swap_max_bytes']!=self.resources['linux_swap_max_bytes']):
            raise ValueError('BorrowedScopeMemoryMismatch')
        return result

    def close_delegation(self):
        primary=None
        try:
            if self.channel is not None:
                self.channel.close()
                self.channel=None
        except BaseException as error:
            primary=error
        try:
            if self.server_handle is not None:
                self.server_api.close(self.server_handle)
                self.server_handle=None
        except BaseException as error:
            if primary is None:
                primary=error
            else:
                primary.add_note('Delegation server handle close: '+repr(error))
        if primary is not None:
            raise primary

    def close(self):
        import native_scope
        from immutable_state_access import guard
        if self.retirement is not None:
            return self.retirement
        try:
            acknowledgement=request(self,'RETIRE')
            path=self.parent_root/'RETIREMENT.json'
            if acknowledgement['retirement_path']!=str(path):
                raise ValueError('ChangedCanonicalScopeRetirementLocation')
            with guard(path):
                content=path.read_bytes()
                if (acknowledgement['binding']!=self.binding
                    or hashlib.sha256(content).hexdigest()!=acknowledgement['retirement_sha256']
                    or json.loads(content)!=acknowledgement['retirement']):
                    raise ValueError('ChangedIndependentScopeRetirement')
                result=dict(acknowledgement['retirement'],delegated=True,owner_confirmation=acknowledgement)
                native_scope.write(self.folder/'RETIREMENT.json',result)
                self.retirement=result
                return result
        finally:
            self.close_delegation()

class NestedJob:
    def __init__(self,job):
        self.job=job

    def __getattr__(self,name):
        return getattr(self.job,name)

    def verify(self,process):
        import native_scope
        self.job.verify(process)
        capability=native_scope.windows_admission()
        try:
            capability.verify(process)
        finally:
            capability.close()

    def popen(self,original,args,**options):
        import native_scope
        module=native_scope.atomic_module()
        from resource_limits import executable_options
        if options.get('shell') or options.get('creationflags',0)&(0x01000000|4):
            raise ValueError('OwnedProcessFlagsRequired')
        job=self
        class NestedPopen(module.AtomicMixin,original):
            def __init__(self,args,**kwargs):
                self._atomic_job=job
                kwargs['creationflags']=kwargs.get('creationflags',0)|4
                executable_options(args,kwargs)
                super().__init__(args,**kwargs)
                try:
                    self.atomic_verify()
                    self.atomic_resume()
                except BaseException as error:
                    self.atomic_abort(error)
                    raise
        return NestedPopen(args,**options)
