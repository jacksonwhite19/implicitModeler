"""Private creation seam for the pinned CPython 3.12 Windows Popen.

Only process creation is replaced. Popen owns pipes, stream wrappers, waits and
process handles. Callers own the job, admission barrier and explicit resume.
"""
import ctypes as C
from ctypes import wintypes as W
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import _winapi

STDLIB_SHA256 = 'e9886605ccac88bcfe2330a1b4cd8f132cbaf97ebb6834b1dcb5a69b127b8d5b'
JOB_LIST = 0x0002000d
HANDLE_LIST = 0x00020002
SUSPENDED = 4
EXTENDED = 0x80000
UNICODE_ENVIRONMENT = 0x400


class StartupInfo(C.Structure):
    _fields_ = [('cb', W.DWORD), ('reserved', W.LPWSTR), ('desktop', W.LPWSTR),
                ('title', W.LPWSTR),
                *[(n, W.DWORD) for n in ('x','y','xsize','ysize','xchars','ychars','fill','flags')],
                ('show', W.WORD), ('reserved_bytes', W.WORD), ('reserved_pointer', C.c_void_p),
                ('stdin', W.HANDLE), ('stdout', W.HANDLE), ('stderr', W.HANDLE)]


class StartupInfoEx(C.Structure):
    _fields_ = [('startup', StartupInfo), ('attributes', C.c_void_p)]


class ProcessInformation(C.Structure):
    _fields_ = [('process', W.HANDLE), ('thread', W.HANDLE),
                ('pid', W.DWORD), ('tid', W.DWORD)]


class Api:
    def __init__(self):
        if (os.name != 'nt' or sys.version_info[:2] != (3,12)
                or C.sizeof(C.c_void_p) != 8 or C.sizeof(StartupInfo) != 104
                or C.sizeof(StartupInfoEx) != 112 or C.sizeof(ProcessInformation) != 24):
            raise ValueError('UnsupportedAtomicPopenABI')
        path = Path(sys.base_prefix)/'Lib/subprocess.py'
        if Path(subprocess.__file__).resolve() != path.resolve():
            raise ValueError('UnexpectedSubprocessOrigin')
        with path.open('rb') as f:
            if hashlib.file_digest(f,'sha256').hexdigest() != STDLIB_SHA256:
                raise ValueError('ChangedAtomicPopenABI')
        self.k = k = C.WinDLL('kernel32', use_last_error=True)
        signatures = {
            'InitializeProcThreadAttributeList': ([C.c_void_p,W.DWORD,W.DWORD,C.POINTER(C.c_size_t)], W.BOOL),
            'UpdateProcThreadAttribute': ([C.c_void_p,W.DWORD,C.c_size_t,C.c_void_p,C.c_size_t,C.c_void_p,C.c_void_p], W.BOOL),
            'DeleteProcThreadAttributeList': ([C.c_void_p], None),
            'CreateProcessW': ([W.LPCWSTR,W.LPWSTR,C.c_void_p,C.c_void_p,W.BOOL,W.DWORD,
                                C.c_void_p,W.LPCWSTR,C.POINTER(StartupInfoEx),C.POINTER(ProcessInformation)], W.BOOL),
            'GetHandleInformation': ([W.HANDLE,C.POINTER(W.DWORD)], W.BOOL),
            'CloseHandle': ([W.HANDLE], W.BOOL),
            'ResumeThread': ([W.HANDLE], W.DWORD),
            'CompareStringOrdinal': ([W.LPCWSTR,C.c_int,W.LPCWSTR,C.c_int,W.BOOL], C.c_int),
        }
        for name,(args,result) in signatures.items():
            getattr(k,name).argtypes=args
            getattr(k,name).restype=result

    def close(self, handle):
        C.set_last_error(0)
        if not self.k.CloseHandle(handle):
            raise C.WinError(C.get_last_error())

    def environment(self, env):
        if env is None:
            return None
        # These selected callers supply ordinary string dictionaries. Mirror
        # CPython's invariant uppercase ordering and last equal-key selection.
        if not isinstance(env, dict):
            raise TypeError('AtomicPopenStringEnvironmentDictionaryRequired')
        for key,value in env.items():
            if not isinstance(key,str) or not isinstance(value,str):
                raise TypeError('EnvironmentStringsRequired')
            if not key or '=' in key[1:] or '\0' in key or '\0' in value:
                raise ValueError('InvalidEnvironmentEntry')
        keys=sorted(env, key=lambda s:_winapi.LCMapStringEx('',0x200,s))
        selected=[]
        for i,key in enumerate(keys):
            if i+1<len(keys):
                C.set_last_error(0)
                comparison=self.k.CompareStringOrdinal(key,-1,keys[i+1],-1,True)
                if not comparison:
                    raise C.WinError(C.get_last_error())
                if comparison==2:
                    continue
            selected.append(key+'='+env[key])
        # Explicit size preserves the final double NUL, including empty maps.
        text='\0'.join(selected)+'\0\0'
        return C.create_unicode_buffer(text,len(text.encode('utf-16-le','surrogatepass'))//2)

    def create(self, executable, command, cwd, env, startup, flags, handles, job):
        bits=W.DWORD()
        C.set_last_error(0)
        if not self.k.GetHandleInformation(job,C.byref(bits)):
            raise C.WinError(C.get_last_error())
        if bits.value & 1 or int(job) in handles:
            raise ValueError('JobHandleMustNotBeInherited')
        size=C.c_size_t()
        count=1+bool(handles)
        C.set_last_error(0)
        initial=self.k.InitializeProcThreadAttributeList(None,count,0,C.byref(size))
        error=C.get_last_error()
        if initial or error!=122:
            raise OSError(error,'AttributeListSizingFailed')
        storage=C.create_string_buffer(size.value)
        initialized=False
        arrays=[(JOB_LIST,(W.HANDLE*1)(job))]
        if handles:
            arrays.append((HANDLE_LIST,(W.HANDLE*len(handles))(*handles)))
        environment=self.environment(env)
        command_buffer=C.create_unicode_buffer(command)
        info=ProcessInformation()
        try:
            C.set_last_error(0)
            if not self.k.InitializeProcThreadAttributeList(storage,count,0,C.byref(size)):
                raise C.WinError(C.get_last_error())
            initialized=True
            for kind,values in arrays:
                C.set_last_error(0)
                if not self.k.UpdateProcThreadAttribute(storage,0,kind,values,C.sizeof(values),None,None):
                    raise C.WinError(C.get_last_error())
            startup.attributes=C.cast(storage,C.c_void_p)
            sys.audit('_winapi.CreateProcess',executable,command,cwd)
            C.set_last_error(0)
            ok=self.k.CreateProcessW(executable,command_buffer,None,None,bool(handles),
                flags|EXTENDED|UNICODE_ENVIRONMENT,environment,cwd,C.byref(startup),C.byref(info))
            error=C.get_last_error() if not ok else None
            if not ok:
                failure=C.WinError(error)
                failure.atomic_creation=dict(api_return=False,error=error,child_created=False,
                                             job_list=True,handle_list_count=len(handles))
                raise failure
            return info,dict(api_return=True,error=None,job_list=True,
                             handle_list_count=len(handles),child_suspended=True)
        finally:
            if initialized:
                self.k.DeleteProcThreadAttributeList(storage)


class AtomicMixin:
    """All selected instances must set _atomic_job before Popen.__init__."""
    def _atomic_create(self, api, *args):
        return api.create(*args)

    def _execute_child(self, args, executable, preexec_fn, close_fds,
                       pass_fds, cwd, env, startupinfo, creationflags, shell,
                       p2cread, p2cwrite, c2pread, c2pwrite, errread, errwrite,
                       unused_restore_signals, unused_gid, unused_gids, unused_uid,
                       unused_umask, unused_start_new_session, unused_process_group):
        primary=None
        self._atomic_thread=None
        self._atomic_receipt=dict(created=False,resumed=False,thread_closed=False)
        try:
            if (not isinstance(args,(list,tuple)) or not args or shell or preexec_fn is not None
                    or pass_fds or not close_fds or creationflags != SUSPENDED):
                raise ValueError('UnsupportedAtomicPopenOptions')
            if (unused_gid is not None or unused_gids is not None or unused_uid is not None
                    or unused_umask != -1 or unused_start_new_session or unused_process_group != -1):
                raise ValueError('UnsupportedAtomicPopenPosixOptions')
            job=self._atomic_job
            if job is None or not job.handle:
                raise ValueError('AtomicJobRequired')
            api=self._atomic_api=Api()
            if hasattr(job,'check'):
                job.check()
            else:
                job.readback()
            startupinfo=subprocess.STARTUPINFO() if startupinfo is None else startupinfo.copy()
            attributes=startupinfo.lpAttributeList or {}
            if set(attributes)-{'handle_list'}:
                raise ValueError('UnsupportedAtomicStartupAttribute')
            if startupinfo.dwFlags & ~(subprocess.STARTF_USESTDHANDLES|subprocess.STARTF_USESHOWWINDOW):
                raise ValueError('UnsupportedAtomicStartupFlags')
            if -1 not in (p2cread,c2pwrite,errwrite):
                startupinfo.dwFlags |= subprocess.STARTF_USESTDHANDLES
                startupinfo.hStdInput=p2cread
                startupinfo.hStdOutput=c2pwrite
                startupinfo.hStdError=errwrite
            handles=list(attributes.get('handle_list') or [])
            if startupinfo.dwFlags & subprocess.STARTF_USESTDHANDLES:
                handles.extend([startupinfo.hStdInput,startupinfo.hStdOutput,startupinfo.hStdError])
            handles=self._filter_handle_list(handles)
            handles=[int(h) for h in handles]
            startup=StartupInfoEx()
            startup.startup.cb=C.sizeof(startup)
            startup.startup.flags=startupinfo.dwFlags
            startup.startup.show=startupinfo.wShowWindow
            startup.startup.stdin=startupinfo.hStdInput
            startup.startup.stdout=startupinfo.hStdOutput
            startup.startup.stderr=startupinfo.hStdError
            executable=None if executable is None else os.fsdecode(executable)
            cwd=None if cwd is None else os.fsdecode(cwd)
            command=subprocess.list2cmdline(args)
            if any('\0' in value for value in (command,executable or '',cwd or '')):
                raise ValueError('EmbeddedNulInProcessArguments')
            sys.audit('subprocess.Popen',executable,command,cwd,env)
            info,receipt=self._atomic_create(api,executable,command,cwd,env,startup,
                                              creationflags,handles,job.handle)
            self._child_created=True
            self._handle=subprocess.Handle(info.process)
            self.pid=info.pid
            self._atomic_thread=info.thread
            self._atomic_receipt.update(receipt,created=True,pid=info.pid,tid=info.tid)
        except BaseException as error:
            primary=error
            if getattr(self,'_child_created',False):
                self.atomic_abort(error)
            raise
        finally:
            try:
                self._close_pipe_fds(p2cread,p2cwrite,c2pread,c2pwrite,errread,errwrite)
            except BaseException as error:
                if primary is not None:
                    primary.add_note('Child pipe close: '+repr(error))
                else:
                    self.atomic_abort(error)
                    raise

    def atomic_verify(self):
        self._atomic_job.verify(self)
        self._atomic_receipt['membership_verified']=True

    def atomic_resume(self):
        if not self._atomic_receipt.get('membership_verified'):
            raise ValueError('AtomicAdmissionRequiredBeforeResume')
        C.set_last_error(0)
        previous=self._atomic_api.k.ResumeThread(self._atomic_thread)
        if previous==0xffffffff:
            raise C.WinError(C.get_last_error())
        if previous!=1:
            raise ValueError('UnexpectedPrimaryThreadSuspendCount',previous)
        self._atomic_receipt['resumed']=True
        self._atomic_api.close(self._atomic_thread)
        self._atomic_thread=None
        self._atomic_receipt['thread_closed']=True

    def atomic_abort(self, primary):
        if getattr(self,'_child_created',False):
            try:
                self.kill()
                self.wait(timeout=5)
                self._atomic_receipt['aborted_returncode']=self.returncode
            except BaseException as error:
                primary.add_note('Suspended child retirement: '+repr(error))
        if getattr(self,'_atomic_thread',None) is not None:
            try:
                self._atomic_api.close(self._atomic_thread)
                self._atomic_thread=None
                self._atomic_receipt['thread_closed']=True
            except BaseException as error:
                primary.add_note('Primary thread close: '+repr(error))
        if getattr(self,'_child_created',False):
            try:
                self._handle.Close()
            except BaseException as error:
                primary.add_note('Process handle close: '+repr(error))
