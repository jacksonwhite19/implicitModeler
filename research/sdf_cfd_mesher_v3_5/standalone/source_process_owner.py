"""Windows local process custody. Children wait before opening any source host."""
import ctypes, os, time, uuid
from contextlib import contextmanager

class _Basic(ctypes.Structure):
    _fields_=[('process_time',ctypes.c_longlong),('job_time',ctypes.c_longlong),('flags',ctypes.c_uint32),('min_ws',ctypes.c_size_t),('max_ws',ctypes.c_size_t),('active_limit',ctypes.c_uint32),('affinity',ctypes.c_size_t),('priority',ctypes.c_uint32),('scheduling',ctypes.c_uint32)]
class _IO(ctypes.Structure):
    _fields_=[(x,ctypes.c_ulonglong) for x in ('read_ops','write_ops','other_ops','read_bytes','write_bytes','other_bytes')]
class _Extended(ctypes.Structure):
    _fields_=[('basic',_Basic),('io',_IO),('process_mem',ctypes.c_size_t),('job_mem',ctypes.c_size_t),('peak_process',ctypes.c_size_t),('peak_job',ctypes.c_size_t)]
class _Accounting(ctypes.Structure):
    _fields_=[('user',ctypes.c_longlong),('kernel',ctypes.c_longlong),('period_user',ctypes.c_longlong),('period_kernel',ctypes.c_longlong),('faults',ctypes.c_uint32),('total',ctypes.c_uint32),('active',ctypes.c_uint32),('terminated',ctypes.c_uint32)]

def _api():
    k=ctypes.WinDLL('kernel32',use_last_error=True)
    signatures={
        'CreateJobObjectW':([ctypes.c_void_p,ctypes.c_wchar_p],ctypes.c_void_p),
        'SetInformationJobObject':([ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_uint32],ctypes.c_int),
        'AssignProcessToJobObject':([ctypes.c_void_p,ctypes.c_void_p],ctypes.c_int),
        'TerminateJobObject':([ctypes.c_void_p,ctypes.c_uint32],ctypes.c_int),
        'QueryInformationJobObject':([ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_uint32,ctypes.c_void_p],ctypes.c_int),
        'CloseHandle':([ctypes.c_void_p],ctypes.c_int),
        'CreateEventW':([ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_wchar_p],ctypes.c_void_p),
        'OpenEventW':([ctypes.c_uint32,ctypes.c_int,ctypes.c_wchar_p],ctypes.c_void_p),
        'SetEvent':([ctypes.c_void_p],ctypes.c_int),
        'WaitForSingleObject':([ctypes.c_void_p,ctypes.c_uint32],ctypes.c_uint32),
        'OpenProcess':([ctypes.c_uint32,ctypes.c_int,ctypes.c_uint32],ctypes.c_void_p),
    }
    for name,(args,result) in signatures.items():
        f=getattr(k,name);f.argtypes=args;f.restype=result
    return k

class Job:
    def __init__(self):
        self.k=_api();self.handle=self.k.CreateJobObjectW(None,None)
        if not self.handle:raise ctypes.WinError(ctypes.get_last_error())
        limits=_Extended();limits.basic.flags=0x2000 # KILL_ON_JOB_CLOSE; handle not inherited.
        if not self.k.SetInformationJobObject(self.handle,9,ctypes.byref(limits),ctypes.sizeof(limits)):
            self.close();raise ctypes.WinError(ctypes.get_last_error())
    def assign(self,handle):
        if not self.k.AssignProcessToJobObject(self.handle,ctypes.c_void_p(int(handle))):raise ctypes.WinError(ctypes.get_last_error())
    def active(self):
        value=_Accounting()
        if not self.k.QueryInformationJobObject(self.handle,1,ctypes.byref(value),ctypes.sizeof(value),None):raise ctypes.WinError(ctypes.get_last_error())
        return value.active
    def retire(self):
        if self.active() and not self.k.TerminateJobObject(self.handle,125):raise ctypes.WinError(ctypes.get_last_error())
        deadline=time.monotonic()+5
        while self.active():
            if time.monotonic()>deadline:raise TimeoutError('SourceOwnedProcessesSurvived')
            time.sleep(.02)
    def close(self):
        if self.handle:self.k.CloseHandle(self.handle);self.handle=None

def worker_start_gate():
    """Called at mesh module entry, before importing any evaluator modules."""
    name=os.environ.pop('V35_SOURCE_START_EVENT',None)
    if name is None:return
    k=_api();event=k.OpenEventW(0x100000,False,name)
    if not event:raise RuntimeError('SourceSupervisorStartGateMissing')
    parent=k.OpenProcess(0x100000,False,int(os.environ.pop('V35_SOURCE_SUPERVISOR_PID')))
    try:
        if not parent:raise RuntimeError('SourceSupervisorExitedBeforeStart')
        deadline=time.monotonic()+30
        while k.WaitForSingleObject(event,50)!=0:
            if k.WaitForSingleObject(parent,0)==0:raise RuntimeError('SourceSupervisorExitedBeforeStart')
            if time.monotonic()>deadline:raise TimeoutError('SourceSupervisorStartGateDeadline')
    finally:
        k.CloseHandle(event)
        if parent:k.CloseHandle(parent)

@contextmanager
def launch(command,*,contained,popen=None,**kwargs):
    import subprocess
    factory=subprocess.Popen if popen is None else popen
    if not contained:
        process=factory(command,**kwargs)
        try:yield process
        finally:
            if process.poll() is None:process.kill();process.wait(timeout=5)
        return
    job=Job();event=None;process=None
    try:
        name='Local\\V35SourceStart_'+uuid.uuid4().hex
        event=job.k.CreateEventW(None,True,False,name)
        if not event:raise ctypes.WinError(ctypes.get_last_error())
        env=dict(os.environ,V35_SOURCE_START_EVENT=name,V35_SOURCE_SUPERVISOR_PID=str(os.getpid()))
        process=factory(command,env=env,**kwargs)
        process._v35_source_job=job
        job.assign(process._handle)
        if not job.k.SetEvent(event):raise ctypes.WinError(ctypes.get_last_error())
        yield process
        if job.active():
            # The root must have exited; remaining descendants are a failed closure.
            raise RuntimeError('SourceSupervisorContainedProcessesRemain')
    finally:
        try:job.retire()
        finally:
            job.close()
            if event:job.k.CloseHandle(event)
            if process is not None:
                if process.poll() is None:process.kill()
                process.wait(timeout=5)
