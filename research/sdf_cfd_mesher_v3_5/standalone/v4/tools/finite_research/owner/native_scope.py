"""Windows side of isolated fixture native custody; reuse existing supervisor."""
from contextlib import ExitStack, contextmanager
import hashlib,hmac
import json
import math
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import time
import threading
import uuid
from wire import receive, send, MAX_CHUNK

HERE = Path(__file__).resolve().parent
ENV = 'V4_FIXTURE_NATIVE_SCOPE'
ORIGINAL_POPEN = subprocess.Popen
_windows_lock = threading.RLock()
_windows_observations = []
_windows_descriptor = None
_windows_acquiring = False
_windows_closed = False


def read(path): return json.loads(Path(path).read_text(encoding='utf-8'))


def windows_job_module():
    import windows_job_capability
    if Path(windows_job_capability.__file__).resolve() != HERE/'windows_job_capability.py':
        raise ValueError('WrongWindowsJobCapabilityOrigin')
    return windows_job_capability


def atomic_module():
    import windows_atomic_process
    if Path(windows_atomic_process.__file__).resolve() != HERE/'windows_atomic_process.py':
        raise ValueError('WrongAtomicProcessOrigin')
    return windows_atomic_process


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, sort_keys=True, allow_nan=False)
        stream.flush(); os.fsync(stream.fileno())


def linux_path(path):
    value = str(Path(path).resolve())
    if len(value) < 3 or value[1:3] != ':\\': raise ValueError('LocalDrivePathRequired')
    return '/mnt/'+value[0].lower()+value[2:].replace('\\', '/')


def rpc(descriptor, header, timeout=15):
    with socket.create_connection((descriptor['host'], descriptor['port']), timeout=timeout) as channel:
        channel.sendall((json.dumps(dict(header, token=descriptor['token']))+'\n').encode())
        with channel.makefile('rb') as stream:
            line = stream.readline(65537)
        if not line or len(line) > 65536: raise RuntimeError('NativeScopeReplyMissing')
        result = json.loads(line)
        if 'error' in result: raise RuntimeError(result['error'])
        return result


class Scope:
    def __init__(self, folder, runtime_seconds, resources=None, windows_job=None, windows_deadline=None):
        if not math.isfinite(runtime_seconds) or runtime_seconds <= 2: raise ValueError('ScopeBudgetRequired')
        self.folder = Path(folder).resolve(); self.folder.mkdir(); (self.folder/'jobs').mkdir()
        # Linux ephemeral ports can overlap Windows excluded ranges, where WSL
        # localhost forwarding cannot listen. Let Windows choose an admissible
        # port first; a later collision fails startup before any payload.
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
        self.config = dict(unit='v4-fixture-'+uuid.uuid4().hex+'.scope', token=secrets.token_hex(32), port=port)
        self.control_secret=secrets.token_hex(32)
        self.config['delegation_control_sha256']=hashlib.sha256(self.control_secret.encode()).hexdigest()
        self.retirement_lock=threading.RLock()
        self.delegation_channel=None
        self.delegation_handle=None
        self.resources = resources; self.windows_job = windows_job
        self.windows_deadline = windows_deadline
        if resources is not None:
            from resource_limits import validate
            self.resources = validate(resources); self.config['resources'] = self.resources
            if windows_job is None: raise ValueError('ResourceScopeRequiresWindowsJob')
        self.runtime_seconds = float(runtime_seconds)
        self.binding = None; self.process = None; self.streams = []; self.retirement = None
        write(self.folder/'CONFIG.json', self.config)

    def control(self, action, path=None, extra=()):
        if self.binding is None: raise ValueError('MissingScopeOwnership')
        if self.resources is not None and hasattr(self,'live_descriptor') and path is None and action in ('inspect','lifetime','memory'):
            result=rpc(self.live_descriptor,dict(op='control',action=action,extra=[] if action=='memory' else list(extra)),timeout=4)
            if action=='memory' and (result.get('memory_max_bytes')!=self.resources['linux_memory_max_bytes'] or
                                    result.get('swap_max_bytes')!=self.resources['linux_swap_max_bytes']):
                raise ValueError('LinuxMemoryLimitReadbackMismatch')
            return result
        # Bound the Linux control process itself; a Windows WSL wait is not a
        # native-process deadline. Exact scope retirement is still verified.
        native_timeout, host_timeout = (6,8) if action == 'memory' else (20,24)
        command = ['wsl.exe', '-d', self.binding['distribution'], '-e',
                   '/usr/bin/timeout', '--signal=TERM', '--kill-after=1s', str(native_timeout)+'s',
                   '/usr/bin/python3', linux_path(HERE/'linux_scope.py'),
                   action, linux_path(path or self.folder/'BINDING.json')]
        command.extend(map(str, extra))
        if self.windows_job is None:
            result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', timeout=host_timeout)
        else:
            with self.windows_job.popen(ORIGINAL_POPEN, command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True, encoding='utf-8') as child:
                try: stdout, stderr = child.communicate(timeout=host_timeout)
                except BaseException:
                    child.kill(); child.communicate(timeout=5); raise
                result = subprocess.CompletedProcess(command, child.returncode, stdout, stderr)
        if result.returncode: raise RuntimeError('LinuxScopeControlFailed', action, result.stdout, result.stderr)
        return json.loads(result.stdout)

    def start(self):
        command = ['wsl.exe', '-e', 'systemd-run', '--user', '--scope', '--quiet', '--unit='+self.config['unit'],
            '--property=RuntimeMaxSec='+str(self.runtime_seconds)+'s', '--property=TimeoutStopSec=2s',
            '--property=KillMode=control-group', '--property=SendSIGKILL=yes', '--property=Delegate=no',
            '/usr/bin/python3', linux_path(HERE/'linux_scope.py'), 'serve', linux_path(self.folder/'CONFIG.json')]
        if self.resources is not None:
            at = command.index('/usr/bin/python3')
            command[at:at] = ['--property=MemoryMax='+str(self.resources['linux_memory_max_bytes']),
                             '--property=MemorySwapMax='+str(self.resources['linux_swap_max_bytes'])]
        self.streams = [(self.folder/n).open('xb') for n in ('broker.stdout', 'broker.stderr')]
        launch = ORIGINAL_POPEN if self.windows_job is None else lambda args, **kw: self.windows_job.popen(ORIGINAL_POPEN,args,**kw)
        until = time.perf_counter()+10
        try:
            if self.resources is not None:
                from delegation_pipe import Pipe
                self.delegation_channel=Pipe()
            self.process = launch(command, stdin=subprocess.PIPE, stdout=self.streams[0], stderr=self.streams[1])
            self.process.stdin.write((self.control_secret+'\n').encode())
            self.process.stdin.close()
            while not (self.folder/'READY.json').is_file():
                if self.process.poll() is not None: raise RuntimeError('LinuxScopeStartupFailed')
                if time.perf_counter() >= until: raise TimeoutError('LinuxScopeReadinessDeadline')
                time.sleep(.025)
            # An exclusively created JSON may still be completing its flush.
            while True:
                try: ready = read(self.folder/'READY.json'); break
                except json.JSONDecodeError:
                    if time.perf_counter() >= until: raise
                    time.sleep(.01)
            self.accept_binding(ready['binding'])
            descriptor = dict(host=ready['host'], port=ready['port'], token=self.config['token'],
                root=str(self.folder), distribution=self.binding['distribution'], schema='V4FixtureNativeScope/1')
            if descriptor['host'] != '127.0.0.1' or descriptor['port'] != self.config['port']:
                raise ValueError('LoopbackScopeRequired')
            while True:
                try:
                    observed = rpc(descriptor, dict(op='ping'), timeout=max(.1, until-time.perf_counter()))
                    break
                except ConnectionRefusedError:
                    if time.perf_counter() >= until: raise
                    time.sleep(.025)
            if observed['binding'] != self.binding: raise ValueError('NativeScopeHandshakeChanged')
            self.live_descriptor = descriptor
            check = self.control('inspect')
            if check['retired'] or not check['original_broker_live']: raise ValueError('NativeScopeNotLive')
            extended = self.control('lifetime', extra=(self.runtime_seconds,))
            if not extended['passed'] or extended['verified_microseconds'] != round(self.runtime_seconds*1000000):
                raise ValueError('ScopeLifetimeUnverified')
            if self.resources is not None:
                self.memory_start = self.memory()
                if any(self.memory_start['events'].get(key,0) for key in ('max','oom','oom_kill','oom_group_kill')):
                    raise ValueError('ResourceFailureDuringStartup')
            if self.resources is not None:
                descriptor['windows_job'] = windows_job_module().make_grant(self.windows_job,self.windows_deadline)
                descriptor['delegation']='exclusive_whole_scope_retirement_v2'
                descriptor['delegation_pipe']=self.delegation_channel.address
            self.descriptor = self.folder/'DESCRIPTOR.json'; write(self.descriptor, descriptor)
            write(self.folder/'START_VALIDATED.json', dict(binding=self.binding, inspection=check, lifetime=extended))
            return self
        except BaseException as primary:
            # Startup has not dispatched a worker or any native payload. The
            # broker's 12 s startup watchdog bounds an incomplete handshake.
            cleanup_error = None
            try:
                if (self.folder/'BINDING.json').is_file():
                    if self.binding is None: self.accept_binding(read(self.folder/'BINDING.json'))
                    self.close()
                else:
                    # No payload was dispatched. A failed/unreachable Linux
                    # startup has no authenticated cleanup target: fail closed.
                    if self.process is not None: self.process.wait(timeout=16)
            except BaseException as error:
                cleanup_error = repr(error)
            finally:
                if self.process is not None and self.process.poll() is None:
                    import psutil
                    from standalone_tool import terminate_worker
                    terminate_worker(self.folder, {}, psutil.Process(self.process.pid), self.process)
                write(self.folder/'STARTUP_FAILURE.json', dict(error=repr(primary), cleanup_error=cleanup_error,
                    confirmed_linux_retirement=self.retirement is not None, payloads_dispatched=0,
                    unbound_startup_limitation='No claim of Linux retirement before authenticated scope binding.',
                    transport_returncode=None if self.process is None else self.process.poll()))
                for stream in self.streams: stream.close()
                self.close_delegation()
            raise

    def memory(self):
        if self.resources is None: raise ValueError('MemoryContractRequired')
        if self.retirement is not None:
            result=self.retirement.get('memory_before_retirement')
            if result is None or self.retirement.get('memory_error'):
                raise ValueError('MissingTerminalNativeMemoryEvidence')
            return dict(result,retired=True)
        return self.control('memory', extra=(self.resources['linux_memory_max_bytes'],self.resources['linux_swap_max_bytes']))

    def delegation_tick(self,job,check_resources):
        from delegated_scope import advance
        advance(self,job,check_resources)

    def privileged_control(self,action,extra):
        message=dict(action=action,extra=extra,unit=self.config['unit'])
        signature=hmac.new(self.control_secret.encode(),json.dumps(message,sort_keys=True).encode(),'sha256').hexdigest()
        return rpc(self.live_descriptor,dict(op='control',action=action,extra=extra,owner_signature=signature),timeout=4)

    def close_delegation(self):
        if self.delegation_channel is not None:
            self.delegation_channel.close()
            self.delegation_channel=None
        if self.delegation_handle is not None:
            api,handle=self.delegation_handle
            api.close(handle)
            self.delegation_handle=None

    def accept_binding(self, binding):
        if (binding['unit'] != self.config['unit'] or
                binding['token_sha256'] != hashlib.sha256(self.config['token'].encode()).hexdigest() or
                not isinstance(binding.get('distribution'), str) or not binding['distribution']):
            raise ValueError('NativeScopeBindingMismatch')
        if self.binding is not None and self.binding != binding: raise ValueError('ScopeBindingChanged')
        self.binding = binding

    def close(self):
        with self.retirement_lock:
            return self._close()

    def _close(self):
        if self.retirement is not None: return self.retirement
        if self.binding is None: raise ValueError('MissingScopeOwnership')
        # Never take a replacement cleanup target from mutable child output.
        if read(self.folder/'BINDING.json') != self.binding: raise ValueError('ScopeBindingFileChanged')
        extra=() if self.resources is None else (self.resources['linux_memory_max_bytes'],self.resources['linux_swap_max_bytes'])
        result = self.control('retire',extra=extra)
        if not result.get('passed') or not result['after']['retired']: raise RuntimeError('ScopeRetirementUnverified')
        self.process.wait(timeout=6)  # Natural transport exit after verified Linux retirement.
        result['transport_returncode'] = self.process.returncode
        write(self.folder/'RETIREMENT.json', result)
        self.retirement = result
        for stream in self.streams: stream.close()
        return result


def native_argv(argv, distribution):
    if not isinstance(argv, (list, tuple)) or not argv: raise ValueError('ExplicitProcessArgvRequired')
    rest = list(map(str, argv[1:])); selected = None
    if rest[:1] in (['-d'], ['--distribution']):
        if len(rest) < 2: raise ValueError('MissingWslDistribution')
        selected = rest[1]; rest = rest[2:]
    if selected is not None and selected != distribution: raise ValueError('DifferentWslDistribution')
    if not rest or rest[0] not in ('-e', '--exec', '--'): raise ValueError('ExplicitWslExecRequired')
    if len(rest) < 2: raise ValueError('MissingNativeCommand')
    return rest[1:]


def windows_admission():
    """Acquire a fresh restricted capability for one child creation only."""
    global _windows_acquiring
    with _windows_lock:
        if _windows_closed: raise ValueError('ClosedWindowsChildAdmission')
        if _windows_descriptor is None: raise ValueError('WindowsChildAdmissionNotInstalled')
        if _windows_acquiring: raise ValueError('RecursiveWindowsChildAdmission')
        _windows_acquiring = True
        try:
            return windows_job_module().Capability(*_windows_descriptor)
        finally:
            _windows_acquiring = False


def close_windows_admission():
    """Close admission and report data; never acquire or retain a job handle."""
    global _windows_closed
    with _windows_lock:
        _windows_closed = True
        return dict(closed=True, persistent_job_handles=0,
                    launches=list(_windows_observations))


def install():
    """Process-local hook; descendants receive it via the isolated PYTHONPATH."""
    global _windows_descriptor
    if os.name != 'nt' or ENV not in os.environ: return
    descriptor_path = Path(os.environ[ENV]).resolve(); descriptor = read(descriptor_path)
    if descriptor.get('schema') != 'V4FixtureNativeScope/1': raise ValueError('NativeScopeDescriptorRequired')
    if getattr(subprocess.Popen, '_fixture_scope_hook', False): return
    if 'windows_job' in descriptor:
        _windows_descriptor = (descriptor_path,descriptor)
    original = subprocess.Popen
    class ScopedPopen(atomic_module().AtomicMixin, original):
        # Keep Popen's type, inherited lifecycle and subclass dispatch. In
        # particular asyncio.windows_utils derives its own Popen from this.
        _fixture_scope_hook = True

        def __init__(self, args, *positional, **kwargs):
            if positional: raise ValueError('ScopedPopenKeywordOptionsRequired')
            if not isinstance(args, (list, tuple)) or kwargs.get('shell'):
                raise ValueError('ScopedExplicitArgvRequired')
            if 'windows_job' not in descriptor:
                raise ValueError('AtomicWindowsDescriptorRequired')
            if kwargs.get('creationflags',0) & (0x01000000 | 4):
                raise ValueError('OwnedProcessFlagsRequired')
            kwargs['creationflags'] = kwargs.get('creationflags',0) | 4
            if args and Path(str(args[0])).name.lower() in ('wsl', 'wsl.exe'):
                if kwargs.get('executable') is not None: raise ValueError('ScopedNativeExecutableOverrideRejected')
                if kwargs.get('env') is not None: raise ValueError('ScopedNativeEnvironmentOverrideRejected')
                argv = native_argv(args, descriptor['distribution'])
                job = uuid.uuid4().hex; folder = Path(descriptor['root'])/'jobs'/job; folder.mkdir()
                request = dict(argv=argv, cwd=linux_path(kwargs.get('cwd') or os.getcwd()))
                digest = hashlib.sha256(json.dumps(request, sort_keys=True, allow_nan=False).encode()).hexdigest()
                write(folder/'REQUEST.json', request)
                args = [sys.executable, '-B', str(HERE/'native_scope.py'), 'proxy', str(descriptor_path), job, digest]
                if kwargs.get('stdin') is None: kwargs['stdin'] = subprocess.DEVNULL
            elif kwargs.get('env') is not None:
                # Preserve the isolated hook for descendant Python helpers.
                env = dict(kwargs['env']); env[ENV] = str(descriptor_path)
                env['PYTHONPATH'] = str(HERE)+os.pathsep+env.get('PYTHONPATH', '')
                kwargs['env'] = env
            if args and os.path.normcase(str(args[0])) == os.path.normcase(sys.executable):
                # Same Store-Python executable selection as the resource owner.
                actual=Path(sys.base_prefix)/'python.exe'
                if not actual.is_file(): raise ValueError('OwnedPythonExecutableMissing')
                if kwargs.get('executable') is not None: raise ValueError('OwnedPythonExecutableOverrideRejected')
                kwargs['executable']=str(actual)
            capability = windows_admission()
            self._atomic_job = capability
            try:
                super().__init__(args, **kwargs)
                self.atomic_verify()
                closed = capability.close()
                self._atomic_receipt['capability_closed_before_resume'] = closed['closed']
                self.atomic_resume()
                _windows_observations.append(dict(capability=closed, creation=dict(self._atomic_receipt)))
            except BaseException as primary:
                # Retire the suspended child before releasing a last duplicate.
                self.atomic_abort(primary)
                try: capability.close()
                except BaseException as cleanup:
                    primary.add_note('Temporary capability close: '+repr(cleanup))
                raise
            finally:
                self._atomic_job = None
    subprocess.Popen = ScopedPopen


def require_transport():
    """Admission check before the selected fixture generator is entered."""
    if os.name != 'nt' or ENV not in os.environ or not getattr(subprocess.Popen, '_fixture_scope_hook', False):
        raise ValueError('ScopedNativeTransportRequired')
    path = Path(os.environ[ENV]); descriptor = read(path)
    if descriptor.get('schema') != 'V4FixtureNativeScope/1' or descriptor['host'] != '127.0.0.1':
        raise ValueError('NativeScopeDescriptorRequired')
    binding = read(Path(descriptor['root'])/'BINDING.json')
    observed = rpc(descriptor, dict(op='ping'))
    if observed.get('binding') != binding or descriptor['distribution'] != binding['distribution']:
        raise ValueError('NativeScopeHandshakeChanged')
    return binding


def downstream_deadline(host_deadline):
    if not math.isfinite(host_deadline):
        raise ValueError('FiniteHostTicketDeadlineRequired')
    require_transport()
    descriptor = read(Path(os.environ[ENV]))
    binding = read(Path(descriptor['root'])/'BINDING.json')
    remaining = host_deadline-time.perf_counter()
    if remaining <= 1:
        raise TimeoutError('NativeTicketExpiredBeforeClockSample')
    observed = rpc(descriptor, dict(op='ping'), timeout=min(15., remaining))
    if observed.get('binding') != binding:
        raise ValueError('NativeClockScopeChanged')
    sample = observed.get('monotonic_seconds')
    if type(sample) not in (float, int) or not math.isfinite(sample) or sample < 0:
        raise ValueError('NativeMonotonicSampleRequired')
    remaining = host_deadline-time.perf_counter()
    if remaining <= 1:
        raise TimeoutError('NativeTicketExpiredDuringClockSample')
    return sample+remaining


def proxy(descriptor_path, job, request_sha256):
    descriptor = read(descriptor_path)
    with socket.create_connection((descriptor['host'], descriptor['port']), timeout=15) as channel:
        channel.sendall((json.dumps(dict(job=job, token=descriptor.get('job_token',descriptor['token']), request_sha256=request_sha256))+'\n').encode())
        channel.settimeout(None)
        def input_stream():
            try:
                while data := os.read(sys.stdin.fileno(), MAX_CHUNK): send(channel, b'I', data)
                send(channel, b'F')
            except (OSError, EOFError): pass  # Parent cancellation is owned by the fixture supervisor.
        threading.Thread(target=input_stream, daemon=True).start()
        with channel.makefile('rb') as stream:
            while True:
                kind, data = receive(stream)
                if kind == b'X': result = json.loads(data); break
                target = {b'O': sys.stdout.buffer, b'E': sys.stderr.buffer}.get(kind)
                if target is None: raise ValueError('NativeOutputFrameRequired')
                target.write(data); target.flush()
    return status_code(result)


def status_code(result):
    if result.get('transport_errors'): raise RuntimeError('NativeTransportFailed', result['transport_errors'])
    code = result['returncode']
    return code if code >= 0 else 128-code


def bounded(label, command, seconds, pins, supervisor, resources=None, windows_observer=None):
    """Use existing guards/Windows tree deadline, then verify Linux retirement.

    Reserve 60 s inside the caller's total bound for Windows retirement, Linux
    stop/escalation and confirmation. No fixture command runs during startup.
    """
    if seconds <= 70: raise ValueError('WholeWorkerReserveRequired')
    from immutable_state_access import guard
    started = time.perf_counter(); scope = None; result = None; worker = []
    job = None; watch = None; windows_retirement = None; resource_evidence = None
    windows_payload_retirement = None; monitor_retired = True; observer_failure = None; delegation_failure=None
    if resources is not None:
        from resource_limits import validate, WindowsJob, ResourceWatch, executable_options
        resources = validate(resources)
    previous = {key: os.environ.get(key) for key in (ENV, 'PYTHONPATH')}
    nested=previous[ENV] is not None and getattr(subprocess.Popen,'_fixture_scope_hook',False)
    if nested:
        require_transport()
        inherited=read(previous[ENV])['windows_job']['deadline']
        seconds=min(seconds,inherited-started)
        if seconds<=100:
            raise ValueError('InheritedWholeWorkerReserveRequired')
    with ExitStack() as held:
        for path, expected in pins.items():
            held.enter_context(guard(path))
            if supervisor.sha(path) != expected: raise ValueError('ScopedSupervisorSourceChanged')
        try:
            if resources is None:
                scope = Scope(Path(supervisor.HERE)/(label+'_SCOPE'), seconds-30)
            else:
                job = WindowsJob(resources['windows_job_commit_bytes'])
                if nested:
                    from delegated_scope import BorrowedScope,NestedJob
                    held.enter_context(guard(previous[ENV]))
                    job=NestedJob(job)
                    scope=BorrowedScope(Path(supervisor.HERE)/(label+'_SCOPE'),seconds-30,resources,job,
                        started+seconds,previous[ENV])
                else:
                    scope = Scope(Path(supervisor.HERE)/(label+'_SCOPE'), seconds-30,resources,job,
                                  windows_deadline=started+seconds-60)
                scope.descriptor = scope.folder/'DESCRIPTOR.json'
                scope.delegation_pins={str(path):digest for path,digest in pins.items()}
            if resources is None:
                scope.start()
                for name in ('CONFIG.json','BINDING.json','DESCRIPTOR.json','READY.json'):
                    held.enter_context(guard(scope.folder/name))
            os.environ[ENV] = str(scope.descriptor)
            os.environ['PYTHONPATH'] = str(HERE)+os.pathsep+(previous['PYTHONPATH'] or '')
            cleanup_reserve=90 if nested else 60
            remaining = seconds-(time.perf_counter()-started)-cleanup_reserve
            if remaining <= 0: raise TimeoutError('NoRemainingWorkerBudget')
            original = subprocess.Popen
            if issubclass(original,atomic_module().AtomicMixin) and not nested:
                raise ValueError('AlreadyAtomicOriginalPopenRejected')
            class TrackedPopen(atomic_module().AtomicMixin, ORIGINAL_POPEN):
                _fixture_scope_hook=getattr(original,'_fixture_scope_hook',False)
                def __new__(cls,args,*args_tail,**options):
                    if list(args)!=list(command) or job is None:
                        return original(args,*args_tail,**options)
                    return super().__new__(cls)

                def __init__(self, args, *a, **k):
                    nonlocal watch
                    self._observe_wait = False
                    owned = list(args) == list(command) and job is not None
                    if owned:
                        if a or k.get('shell') or k.get('creationflags',0) & (0x01000000 | 4):
                            raise ValueError('OwnedWorkerFlagsRequired')
                        k['creationflags']=k.get('creationflags',0) | 4
                        executable_options(args,k)
                    self._atomic_job = job if owned else None
                    super().__init__(args, *a, **k)
                    if list(args) == list(command): worker.append(self)
                    if owned:
                        try:
                            self.atomic_verify()
                            # Establish the job hierarchy with the actual suspended
                            # Python worker before adding WSL's Windows transport.
                            # Store Python can carry an existing app job; assigning
                            # it to a previously populated unrelated job fails.
                            scope.start()
                            for name in ('CONFIG.json','BINDING.json','DESCRIPTOR.json','READY.json'):
                                held.enter_context(guard(scope.folder/name))
                            watch = ResourceWatch(job,scope,Path(supervisor.HERE),label,resources,worker)
                            watch.start()
                            if watch.errors: raise RuntimeError('ResourceFailureBeforeWorkerResume')
                            if windows_observer is not None:
                                windows_observer.start(job, self)
                            self.atomic_resume()
                            self._observe_wait = windows_observer is not None
                        except BaseException as startup_error:
                            self.atomic_abort(startup_error)
                            raise
                def wait(self, timeout=None):
                    if not self._observe_wait:
                        return super().wait(timeout=timeout)
                    # Observe from the existing supervisor's wait call. No
                    # background thread can outlive the retained job handle.
                    end = time.monotonic()+timeout if timeout is not None else windows_observer.deadline
                    while True:
                        if self.poll() is not None:return super().wait(timeout=0)
                        if end is not None and time.monotonic()>=end:
                            raise subprocess.TimeoutExpired(self.args,timeout)
                        windows_observer.poll()
                        remaining = None if end is None else end-time.monotonic()
                        if remaining is not None and remaining<=0:
                            raise subprocess.TimeoutExpired(self.args,timeout)
                        try:return super().wait(timeout=.05 if remaining is None else min(.05,remaining))
                        except subprocess.TimeoutExpired:pass
            subprocess.Popen = TrackedPopen
            try:
                result = supervisor.bounded(label, command, remaining, pins)
            except BaseException:
                # The existing timeout owner handles its own time limit. Its
                # interrupted wait has no cleanup branch, so use the selected
                # CLI's Windows-first retirement owner for external cancellation.
                import psutil
                from standalone_tool import terminate_worker
                for process in worker:
                    if process.poll() is None:
                        terminate_worker(scope.folder, {}, psutil.Process(process.pid), process)
                raise
            finally: subprocess.Popen = original
        finally:
            primary = sys.exc_info()[1]; cleanup = None; retirement = None
            if windows_observer is not None:
                try: windows_observer.close()
                except BaseException as error:
                    observer_failure = error
                    if primary is not None: primary.add_note('Windows owner observation cleanup: '+repr(error))
            for key, value in previous.items():
                if value is None: os.environ.pop(key, None)
                else: os.environ[key] = value
            try:
                if watch is not None:
                    try: watch.stop()
                    except BaseException as error:
                        watch.failure(error)
                        monitor_retired=watch.thread is None or not watch.thread.is_alive()
                        if not monitor_retired:
                            raise RuntimeError('ResourceMonitorStillOwnsJobAndScope') from error
                    resource_evidence = watch.evidence()
                    if observer_failure is not None:
                        resource_evidence['passed'] = False
                        resource_evidence['errors'].append('WindowsOwnerObservationFailed: '+repr(observer_failure))
                # Release saturated payload commitment before creating cleanup
                # transports. Recreate a cleanup-only Job with the SAME cap only
                # after verified zero active payload processes and handle close.
                # Reusing a completed Store-app job hierarchy can reject new WSL
                # processes. These jobs never provide concurrent allowances.
                if job is not None:
                    windows_payload_retirement = job.retire()
                    if resource_evidence is not None and any(m['message'] in (9,10) for m in windows_payload_retirement['messages']):
                        resource_evidence['passed']=False
                        resource_evidence['errors'].append('ObservedWindowsMemoryDenialAtRetirement')
                    job.close(); job=WindowsJob(resources['windows_job_commit_bytes'])
                    if nested:
                        job=NestedJob(job)
                    scope.windows_job=job
                if scope is not None:
                    if scope.binding is None: raise RuntimeError('UnboundStartupRetirementUnverified')
                    retirement = scope.close()
                    if resources is not None and resource_evidence is not None:
                        memory=retirement.get('memory_before_retirement')
                        resource_evidence['native_counter_at_retirement']=memory
                        resource_evidence['native_final_counter_observation']='scope already retired; last live sample retained' if memory is None else 'read before native stop'
                        if retirement.get('memory_error') or (memory is not None and any(memory['events'].get(key,0) for key in ('max','oom','oom_kill','oom_group_kill'))):
                            resource_evidence['passed']=False
                            resource_evidence['errors'].append('NativeMemoryRetirementEvidenceFailed')
            except BaseException as error:
                cleanup = repr(error); raise
            finally:
                if watch is not None and watch.thread is not None:
                    watch.stop_event.set(); watch.thread.join(0 if not monitor_retired else 10)
                    monitor_retired=not watch.thread.is_alive()
                    if not monitor_retired: cleanup='ResourceMonitorRetirementUnverified'
                if scope is not None and monitor_retired and hasattr(scope,'close_delegation'):
                    try:
                        scope.close_delegation()
                    except BaseException as error:
                        cleanup=repr(error)
                        delegation_failure=error
                if job is not None and monitor_retired:
                    try:
                        windows_retirement = job.retire()
                        if resource_evidence is not None and any(m['message'] in (9,10) for m in windows_retirement['messages']):
                            resource_evidence['passed']=False
                            resource_evidence['errors'].append('ObservedCleanupWindowsMemoryDenial')
                    except BaseException as error:
                        cleanup = repr(error)
                        raise
                    finally:
                        job.close()
                        supervisor.publish(Path(supervisor.HERE)/(label+'_RESOURCE_RESULT.json'),dict(
                            passed=primary is None and cleanup is None and resource_evidence is not None
                                and resource_evidence['passed'] and result is not None
                                and result['returncode']==0 and not result.get('killed_at_limit',False),
                            limits=resources, observation=resource_evidence, windows_retirement=windows_retirement,
                            windows_payload_retirement=windows_payload_retirement,
                            native_memory_scope='broker, payload descendants and live inspection children in the authenticated cgroup; external bounded retirement verifier is outside it',
                            host_scope='parent control process and OS services are outside payload memory accounting',
                            monitor_retired=monitor_retired,
                            linux_retired=retirement is not None and retirement.get('passed') is True,
                            primary_error=repr(primary) if primary is not None else None, cleanup_error=cleanup,
                            worker_returncode=None if result is None else result['returncode'],
                            scientific_acceptance=False))
                if primary is not None or cleanup is not None:
                    supervisor.publish(Path(supervisor.HERE)/(label+'_SCOPE_FAILURE.json'),dict(
                        error=repr(primary), cleanup_error=cleanup,
                        confirmed_linux_retirement=retirement is not None and retirement.get('passed') is True,
                        observed_windows_worker_pids=[p.pid for p in worker],
                        observed_windows_worker_returncodes=[p.poll() for p in worker],
                        seconds=time.perf_counter()-started))
                if not monitor_retired: raise RuntimeError('ResourceMonitorRetirementUnverified')
                if delegation_failure is not None:
                    if primary is None:
                        raise delegation_failure
                    primary.add_note('Delegation handle cleanup: '+repr(delegation_failure))
        elapsed = time.perf_counter()-started
        if elapsed > seconds: raise TimeoutError('WholeFixtureBoundExceededAfterCleanup', elapsed)
        if resource_evidence is not None and not resource_evidence['passed']:
            result=dict(result,original_returncode=result['returncode'],returncode=125,resource_failure=True)
        supervisor.publish(Path(supervisor.HERE)/(label+'_SCOPE_RESULT.json'), dict(
            passed=True if resources is None else result['returncode']==0 and not result.get('killed_at_limit',False),
            whole_seconds=elapsed, whole_limit_seconds=seconds, cleanup_reserve_seconds=cleanup_reserve,
            windows_receipt=label+'_EXECUTION.json', linux_retirement_sha256=supervisor.sha(scope.folder/'RETIREMENT.json'),
            worker_returncode=result['returncode'], all_linux_retired=True))
        receipt_sha256 = supervisor.sha(Path(supervisor.HERE)/(label+'_SCOPE_RESULT.json'))
    supervisor.publish(Path(supervisor.HERE)/(label+'_SCOPE_CLOSURE.json'), dict(
        receipt_sha256=receipt_sha256, all_guards_released=True))
    if observer_failure is not None: raise observer_failure
    return result


if __name__ == '__main__':
    if sys.argv[1] != 'proxy': raise ValueError('UnknownNativeScopeAction')
    raise SystemExit(proxy(sys.argv[2], sys.argv[3], sys.argv[4]))
