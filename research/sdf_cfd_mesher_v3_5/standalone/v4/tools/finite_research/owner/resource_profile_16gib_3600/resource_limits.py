"""OS-native limits for this private fixture owner; no scientific acceptance."""
import ctypes as C
from ctypes import wintypes as W
import os
from pathlib import Path
import subprocess
import sys
import threading
import time


APPROVED = dict(windows_job_commit_bytes=16*1024**3, linux_memory_max_bytes=2*1024**3,
                linux_swap_max_bytes=0, output_limit_bytes=4_000_000_000,
                minimum_free_bytes=1_000_000_000)


def validate(limits):
    value = dict(limits)
    if set(value) != set(APPROVED): raise ValueError('ExplicitResourceContractRequired')
    for key in value:
        if type(value[key]) is not int: raise ValueError('IntegerResourceLimitRequired', key)
        if key == 'linux_swap_max_bytes':
            if value[key] != 0: raise ValueError('SwapMustRemainDisabled')
        elif value[key] <= 0: raise ValueError('PositiveResourceLimitRequired', key)
        if key == 'minimum_free_bytes' and value[key] < APPROVED[key]:
            raise ValueError('FreeSpaceFloorCannotBeLowered')
        if key != 'minimum_free_bytes' and value[key] > APPROVED[key]:
            raise ValueError('ResourceLimitExceedsAuthorization', key)
    return value


def executable_options(args, options):
    """Avoid Windows Store app-alias activation escaping suspended ownership."""
    if args and os.path.normcase(str(args[0])) == os.path.normcase(sys.executable):
        actual=Path(sys.base_prefix)/'python.exe'
        if not actual.is_file(): raise ValueError('OwnedPythonExecutableMissing')
        if options.get('executable') is not None: raise ValueError('OwnedPythonExecutableOverrideRejected')
        options['executable']=str(actual)
    return options


class BasicLimits(C.Structure):
    _fields_ = [('process_time', C.c_longlong), ('job_time', C.c_longlong), ('flags', W.DWORD),
        ('minimum_working_set', C.c_size_t), ('maximum_working_set', C.c_size_t),
        ('active_process_limit', W.DWORD), ('affinity', C.c_size_t),
        ('priority', W.DWORD), ('scheduling', W.DWORD)]


class Io(C.Structure):
    _fields_ = [(name, C.c_ulonglong) for name in ('read_ops','write_ops','other_ops','read_bytes','write_bytes','other_bytes')]


class ExtendedLimits(C.Structure):
    _fields_ = [('basic', BasicLimits), ('io', Io), ('process_memory', C.c_size_t),
        ('job_memory', C.c_size_t), ('peak_process_memory', C.c_size_t), ('peak_job_memory', C.c_size_t)]


class Accounting(C.Structure):
    _fields_ = [(name, C.c_longlong) for name in ('user','kernel','period_user','period_kernel')]+[
        (name, W.DWORD) for name in ('faults','processes','active','terminated')]


class Completion(C.Structure):
    _fields_ = [('key', C.c_void_p), ('port', W.HANDLE)]


class WindowsJob:
    """Associate children atomically before their primary thread runs."""
    FLAGS = 0x200 | 0x2000  # JOB_MEMORY and KILL_ON_JOB_CLOSE; no breakaway.

    def __init__(self, limit):
        if os.name != 'nt': raise ValueError('WindowsJobRequired')
        if type(limit) is not int or not 0 < limit <= APPROVED['windows_job_commit_bytes']:
            raise ValueError('InvalidWindowsJobLimit')
        self.limit = limit; self.handle = None; self.port = None; self.messages = []
        self.k = k = C.WinDLL('kernel32', use_last_error=True)
        signatures = {
            'CreateJobObjectW': ([C.c_void_p, W.LPCWSTR], W.HANDLE),
            'SetInformationJobObject': ([W.HANDLE,C.c_int,C.c_void_p,W.DWORD], W.BOOL),
            'QueryInformationJobObject': ([W.HANDLE,C.c_int,C.c_void_p,W.DWORD,C.c_void_p], W.BOOL),
            'GetProcessId': ([W.HANDLE], W.DWORD),
            'IsProcessInJob': ([W.HANDLE,W.HANDLE,C.POINTER(W.BOOL)], W.BOOL),
            'TerminateJobObject': ([W.HANDLE,W.UINT], W.BOOL),
            'CloseHandle': ([W.HANDLE], W.BOOL),
            'CreateIoCompletionPort': ([W.HANDLE,W.HANDLE,C.c_size_t,W.DWORD], W.HANDLE),
            'GetQueuedCompletionStatus': ([W.HANDLE,C.POINTER(W.DWORD),C.POINTER(C.c_size_t),C.POINTER(C.c_void_p),W.DWORD],W.BOOL),
        }
        for name, (args, result) in signatures.items():
            getattr(k,name).argtypes=args; getattr(k,name).restype=result
        try:
            self.handle = k.CreateJobObjectW(None, None)
            if not self.handle: raise C.WinError(C.get_last_error())
            limits = ExtendedLimits(); limits.basic.flags=self.FLAGS; limits.job_memory=limit
            self._set(9, limits)
            self.port=k.CreateIoCompletionPort(W.HANDLE(-1),None,0,1)
            if not self.port: raise C.WinError(C.get_last_error())
            self._set(7, Completion(1,self.port))
            self.readback()
        except BaseException:
            self.close(); raise

    def _set(self, kind, value):
        if not self.k.SetInformationJobObject(self.handle,kind,C.byref(value),C.sizeof(value)):
            raise C.WinError(C.get_last_error())

    def _query(self, kind, cls):
        value=cls()
        if not self.k.QueryInformationJobObject(self.handle,kind,C.byref(value),C.sizeof(value),None):
            raise C.WinError(C.get_last_error())
        return value

    def readback(self):
        value=self._query(9,ExtendedLimits)
        if value.basic.flags != self.FLAGS or value.job_memory != self.limit:
            raise ValueError('WindowsJobLimitReadbackMismatch')
        basic=self._query(1,Accounting)
        return dict(limit_bytes=value.job_memory, flags=value.basic.flags,
                    peak_job_commit_bytes=value.peak_job_memory, active_processes=basic.active,
                    total_processes=basic.processes)

    def verify(self, process):
        if self.k.GetProcessId(process._handle)!=process.pid:
            raise ValueError('WindowsChildHandleIdentityMismatch')
        inside=W.BOOL(); C.set_last_error(0)
        if not self.k.IsProcessInJob(W.HANDLE(process._handle),self.handle,C.byref(inside)):
            raise C.WinError(C.get_last_error())
        if not inside.value: raise ValueError('WindowsJobMembershipUnverified')
        return self.readback()

    def popen(self, original, args, *a, **kwargs):
        if a or kwargs.get('shell'): raise ValueError('OwnedExplicitProcessRequired')
        import windows_atomic_process
        if Path(windows_atomic_process.__file__).resolve() != Path(__file__).resolve().parent.parent/'windows_atomic_process.py':
            raise ValueError('WrongAtomicProcessOrigin')
        if issubclass(original,windows_atomic_process.AtomicMixin):
            raise ValueError('AlreadyAtomicOriginalPopenRejected')
        flags=kwargs.get('creationflags',0)
        if flags & (0x01000000 | 4): raise ValueError('OwnedProcessFlagsRequired')
        kwargs['creationflags']=flags | 4
        executable_options(args,kwargs)
        job=self
        class JobPopen(windows_atomic_process.AtomicMixin,original):
            def __init__(self, args, **kw):
                self._atomic_job=job
                super().__init__(args,**kw)
        process=JobPopen(args,**kwargs)
        try:
            process.atomic_verify()
            process.atomic_resume()
            return process
        except BaseException as primary:
            process.atomic_abort(primary)
            raise

    def drain(self):
        while True:
            code=W.DWORD(); key=C.c_size_t(); pid=C.c_void_p()
            ok=self.k.GetQueuedCompletionStatus(self.port,C.byref(code),C.byref(key),C.byref(pid),0)
            if not ok:
                error=C.get_last_error()
                if error == 258: break
                raise C.WinError(error)
            self.messages.append(dict(message=code.value,pid=pid.value))
        return list(self.messages)

    def retire(self):
        before=self.readback()
        if not self.k.TerminateJobObject(self.handle,125): raise C.WinError(C.get_last_error())
        until=time.perf_counter()+5
        while True:
            after=self.readback()
            if after['active_processes']==0: break
            if time.perf_counter()>=until: raise RuntimeError('WindowsJobRetirementUnverified')
            time.sleep(.025)
        return dict(passed=True,before=before,after=after,messages=self.drain(),
                    denial_observation='ordinary job notifications are not guaranteed')

    def close(self):
        if self.handle: self.k.CloseHandle(self.handle); self.handle=None
        if self.port: self.k.CloseHandle(self.port); self.port=None


def disk_check(root, label, limits):
    """Reuse the CLI checker on this invocation's trees, then aggregate its logs."""
    from standalone_tool import check_resources
    root=Path(root); logical=0; samples=[]
    for name in (label, label+'_SCOPE'):
        path=root/name
        if path.exists():
            if path.is_symlink() or not path.is_dir(): raise ValueError('OwnedOutputDirectoryRequired')
            sample=check_resources(path,limits)
            if sample['free_bytes'] < limits['minimum_free_bytes']: raise ValueError('OperationalResourceLimit')
            logical+=sample['logical_bytes']; samples.append(sample)
    # Sibling logs and receipts are also owned output, but historical runs are not.
    for path in root.iterdir():
        if path.is_file() and (path.name in (label+'.stdout',label+'.stderr') or path.name.startswith(label+'_')):
            logical+=path.stat().st_size
    if logical>limits['output_limit_bytes']: raise ValueError('OperationalResourceLimit')
    if not samples: raise ValueError('OwnedOutputScopeRequired')
    return dict(logical_bytes=logical,free_bytes=min(s['free_bytes'] for s in samples),
                measurement='sampled logical output bytes; not an atomic disk quota')


class ResourceWatch:
    """Sample operational evidence while the existing supervisor owns its wait."""
    def __init__(self, job, scope, root, label, limits, workers):
        self.job=job; self.scope=scope; self.root=root; self.label=label; self.limits=limits
        self.workers=workers; self.errors=[]; self.samples=0; self.last={}
        self.stop_event=threading.Event(); self.thread=None

    def sample(self):
        self.check_resources()
        if hasattr(self.scope,'delegation_tick'):
            self.scope.delegation_tick(self.job,self.check_resources)

    def check_resources(self):
        if self.errors:
            raise RuntimeError('PriorResourceFailurePreventsDelegation',self.errors)
        windows=self.job.readback(); messages=self.job.drain()
        linux=self.scope.memory(); disk=disk_check(self.root,self.label,self.limits)
        self.samples+=1; self.last=dict(windows=windows,linux=linux,disk=disk)
        if any(item['message'] in (9,10) for item in messages):
            raise RuntimeError('ObservedWindowsMemoryDenial',messages)
        if any(linux['events'].get(key,0) for key in ('max','oom','oom_kill','oom_group_kill')):
            raise RuntimeError('LinuxMemoryLimitEvent',linux)

    def failure(self, error):
        self.errors.append(repr(error))
        # Release the existing supervisor's blocking wait. The owner then retires
        # the complete Linux scope and Windows job, including orphan descendants.
        for process in list(self.workers):
            if process.poll() is None:
                try: process.kill()
                except OSError as failure: self.errors.append(repr(failure))

    def start(self):
        self.sample()  # Readback and disk preflight before worker dispatch.
        def monitor():
            while not self.stop_event.wait(.5):
                try: self.sample()
                except BaseException as error:
                    self.failure(error); return
        self.thread=threading.Thread(target=monitor,daemon=True); self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(40)
            if self.thread.is_alive(): raise RuntimeError('ResourceMonitorRetirementUnverified')
        try: self.sample()
        except BaseException as error: self.failure(error)

    def evidence(self):
        return dict(passed=not self.errors,errors=list(self.errors),samples=self.samples,last=self.last,
            windows_denial_observation='observed' if any(m['message'] in (9,10) for m in self.job.messages) else 'none observed; absence is not proof',
            contract='separate Windows aggregate commit and Linux cgroup charged-memory caps; no shared host-wide budget',
            disk_scope='invocation output tree, native scope tree, sibling logs and receipts; sampled, possible write overshoot')
