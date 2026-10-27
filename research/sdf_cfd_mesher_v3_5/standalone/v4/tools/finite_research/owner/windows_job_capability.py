"""Restricted OS duplicate of the current owner's actual payload job."""
from contextlib import ExitStack
from pathlib import Path
import ctypes as C
from ctypes import wintypes as W
import hashlib,json,math,os,time
import psutil

HERE=Path(__file__).resolve().parent
RIGHTS=0x1|0x4  # JOB_OBJECT_ASSIGN_PROCESS | JOB_OBJECT_QUERY

# Exact Win32 layout used by resource_limits; no resource policy is defined here.
class BasicLimits(C.Structure):
    _fields_=[('process_time',C.c_longlong),('job_time',C.c_longlong),('flags',W.DWORD),
        ('minimum_working_set',C.c_size_t),('maximum_working_set',C.c_size_t),
        ('active_process_limit',W.DWORD),('affinity',C.c_size_t),
        ('priority',W.DWORD),('scheduling',W.DWORD)]

class Io(C.Structure):
    _fields_=[(name,C.c_ulonglong) for name in ('read_ops','write_ops','other_ops','read_bytes','write_bytes','other_bytes')]

class ExtendedLimits(C.Structure):
    _fields_=[('basic',BasicLimits),('io',Io),('process_memory',C.c_size_t),
        ('job_memory',C.c_size_t),('peak_process_memory',C.c_size_t),('peak_job_memory',C.c_size_t)]

def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

class Api:
    def __init__(self):
        self.k=k=C.WinDLL('kernel32',use_last_error=True)
        signatures={
            'GetCurrentProcess':([],W.HANDLE),
            'OpenProcess':([W.DWORD,W.BOOL,W.DWORD],W.HANDLE),
            'GetProcessId':([W.HANDLE],W.DWORD),
            'GetProcessTimes':([W.HANDLE,C.POINTER(W.FILETIME),C.POINTER(W.FILETIME),C.POINTER(W.FILETIME),C.POINTER(W.FILETIME)],W.BOOL),
            'WaitForSingleObject':([W.HANDLE,W.DWORD],W.DWORD),
            'DuplicateHandle':([W.HANDLE,W.HANDLE,W.HANDLE,C.POINTER(W.HANDLE),W.DWORD,W.BOOL,W.DWORD],W.BOOL),
            'CloseHandle':([W.HANDLE],W.BOOL),
            'IsProcessInJob':([W.HANDLE,W.HANDLE,C.POINTER(W.BOOL)],W.BOOL),
            'QueryInformationJobObject':([W.HANDLE,C.c_int,C.c_void_p,W.DWORD,C.c_void_p],W.BOOL),
        }
        for name,(args,result) in signatures.items():
            function=getattr(k,name);function.argtypes=args;function.restype=result

    def close(self,handle):
        C.set_last_error(0)
        if not self.k.CloseHandle(handle):raise C.WinError(C.get_last_error())

    def live_identity(self,handle,pid):
        if self.k.GetProcessId(handle)!=pid:raise ValueError('WindowsCapabilityPIDMismatch')
        C.set_last_error(0);state=self.k.WaitForSingleObject(handle,0)
        if state!=0x102:raise ValueError('WindowsCapabilityOwnerNotLive',state,C.get_last_error() if state==0xffffffff else None)
        values=[W.FILETIME() for _ in range(4)]
        C.set_last_error(0)
        if not self.k.GetProcessTimes(handle,*[C.byref(v) for v in values]):raise C.WinError(C.get_last_error())
        process=psutil.Process(pid);image=Path(process.exe()).resolve();stat=image.stat()
        return dict(pid=pid,creation_filetime=(values[0].dwHighDateTime<<32)|values[0].dwLowDateTime,
            image=str(image),image_sha256=sha(image),argv=process.cmdline(),
            file_identity=dict(device=stat.st_dev,file_id=stat.st_ino,size=stat.st_size))

    def membership(self,process,job):
        result=W.BOOL();C.set_last_error(0)
        if not self.k.IsProcessInJob(process,job,C.byref(result)):raise C.WinError(C.get_last_error())
        return bool(result.value)

    def member(self,process,job):
        if not self.membership(process,job):raise ValueError('WindowsCapabilityCurrentJobRequired')

    def limits(self,handle):
        limits=ExtendedLimits();C.set_last_error(0)
        if not self.k.QueryInformationJobObject(handle,9,C.byref(limits),C.sizeof(limits),None):
            raise C.WinError(C.get_last_error())
        return dict(flags=limits.basic.flags,limit_bytes=limits.job_memory)

def make_grant(job,deadline):
    if not math.isfinite(deadline) or deadline<=time.perf_counter():
        raise ValueError('WindowsCapabilityInheritedDeadlineRequired')
    api=Api();current=api.k.GetCurrentProcess();limits=job.readback()
    return dict(schema='V4ActualWindowsPayloadJob/1',owner=api.live_identity(current,os.getpid()),
        owner_local_job_handle=int(job.handle),rights=RIGHTS,deadline=deadline,
        flags=limits['flags'],limit_bytes=limits['limit_bytes'],
        sources={str(HERE/'native_scope.py'):sha(HERE/'native_scope.py'),
                 str(Path(__file__).resolve()):sha(Path(__file__).resolve()),
                 str(HERE/'windows_atomic_process.py'):sha(HERE/'windows_atomic_process.py')})

def validate_grant(grant):
    if (grant.get('schema')!='V4ActualWindowsPayloadJob/1' or grant.get('rights')!=RIGHTS or
            type(grant.get('owner_local_job_handle')) is not int or grant['owner_local_job_handle']<=0 or
            type(grant.get('deadline')) not in (int,float) or not math.isfinite(grant['deadline']) or
            grant['deadline']<=time.perf_counter() or
            type(grant.get('flags')) is not int or grant['flags']!=(0x200|0x2000) or
            type(grant.get('limit_bytes')) is not int or grant['limit_bytes']<=0):
        raise ValueError('InvalidWindowsPayloadCapability')
    expected={str(HERE/'native_scope.py'),str(Path(__file__).resolve()),str(HERE/'windows_atomic_process.py')}
    if set(grant.get('sources',{}))!=expected:
        raise ValueError('WindowsCapabilitySourceSelectionMismatch')

class Capability:
    def __init__(self,descriptor_path,expected_descriptor):
        from immutable_state_access import guard
        self.stack=ExitStack();self.handle=None;self.closed=False
        self.api=Api();self.assignments=0;self.inherited=0;self.duplicates=0;self.descriptor_path=Path(descriptor_path)
        self.descriptor_sha256=None;root_handle=None;primary=None
        try:
            self.stack.enter_context(guard(self.descriptor_path))
            descriptor=json.loads(self.descriptor_path.read_text(encoding='utf8'))
            if descriptor!=expected_descriptor:raise ValueError('ChangedWindowsCapabilityDescriptor')
            self.descriptor_sha256=sha(self.descriptor_path);self.grant=descriptor['windows_job']
            validate_grant(self.grant)
            for name,digest in self.grant['sources'].items():
                self.stack.enter_context(guard(name))
                if sha(name)!=digest:raise ValueError('ChangedWindowsCapabilitySource')
            owner=self.grant['owner']
            self.stack.enter_context(guard(owner['image']))
            C.set_last_error(0)
            root_handle=self.api.k.OpenProcess(0x40|0x1000|0x100000,False,owner['pid'])
            if not root_handle:raise C.WinError(C.get_last_error())
            if self.api.live_identity(root_handle,owner['pid'])!=owner:
                raise ValueError('ChangedWindowsCapabilityOwner')
            if owner['pid'] not in {p.pid for p in psutil.Process().parents()}:
                raise ValueError('WindowsCapabilityOwnerNotAncestor')
            duplicate=W.HANDLE();C.set_last_error(0)
            if not self.api.k.DuplicateHandle(root_handle,self.grant['owner_local_job_handle'],
                    self.api.k.GetCurrentProcess(),C.byref(duplicate),RIGHTS,False,0):
                raise C.WinError(C.get_last_error())
            self.handle=duplicate.value;self.duplicates=1
            # The descriptor integer is never a local handle. Only the OS
            # duplicate is used, and only by an already-associated process.
            self.check()
        except BaseException as error:
            primary=error
            try:self.close()
            except BaseException as cleanup:primary.add_note('Capability acquisition cleanup: '+repr(cleanup))
            raise
        finally:
            if root_handle is not None:
                try:self.api.close(root_handle)
                except BaseException as error:
                    if primary is not None:primary.add_note('Owner process handle close: '+repr(error))
                    else:
                        try:self.close()
                        except BaseException as cleanup:error.add_note('Capability cleanup: '+repr(cleanup))
                        raise

    def check(self):
        if self.closed or self.handle is None:raise ValueError('ClosedWindowsJobCapability')
        validate_grant(self.grant)
        owner=self.grant['owner'];root=None;primary=None
        try:
            C.set_last_error(0);root=self.api.k.OpenProcess(0x1000|0x100000,False,owner['pid'])
            if not root:raise C.WinError(C.get_last_error())
            if self.api.live_identity(root,owner['pid'])!=owner:
                raise ValueError('ExpiredWindowsCapabilityOwner')
            self.api.member(self.api.k.GetCurrentProcess(),self.handle)
            if self.api.limits(self.handle)!={k:self.grant[k] for k in ('flags','limit_bytes')}:
                raise ValueError('WindowsCapabilityLimitsChanged')
        except BaseException as error:
            primary=error
            raise
        finally:
            if root is not None:
                try:self.api.close(root)
                except BaseException as error:
                    if primary is not None:primary.add_note('Owner liveness handle close: '+repr(error))
                    else:raise

    def verify(self,process):
        self.check()
        if self.api.k.GetProcessId(process._handle)!=process.pid:
            raise ValueError('WindowsChildHandleIdentityMismatch')
        self.api.member(process._handle,self.handle)
        self.inherited+=1
        self.check()

    def describe(self):
        return dict(schema='V4WindowsCapabilityObservation/1',owner_pid=self.grant['owner']['pid'],
            owner_creation_filetime=self.grant['owner']['creation_filetime'],
            descriptor_sha256=self.descriptor_sha256,rights=RIGHTS,noninheritable=True,
            os_duplicates=self.duplicates,children_assigned=self.assignments,
            children_atomically_associated=self.inherited,closed=self.closed,
            flags=self.grant['flags'],limit_bytes=self.grant['limit_bytes'],deadline=self.grant['deadline'])

    def close(self):
        if self.closed:return self.describe()
        primary=None
        try:
            if self.handle is not None:self.api.close(self.handle);self.handle=None
        except BaseException as error:primary=error
        try:self.stack.close()
        except BaseException as error:
            if primary is not None:primary.add_note('Capability guards: '+repr(error))
            else:primary=error
        if primary is not None:raise primary
        self.closed=True
        return self.describe() if hasattr(self,'grant') else dict(closed=True)
