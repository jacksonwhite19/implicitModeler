"""Unrestricted Windows job accounting, including exited descendant workers.

No limits, priorities, affinity, or scientific controls are changed.
https://learn.microsoft.com/en-us/windows/win32/api/jobapi2/nf-jobapi2-queryinformationjobobject
"""
import ctypes as C
from ctypes import wintypes as W
from pathlib import Path
import subprocess
import sys
import time
import psutil
import capsule_user_domain_handoff as h


class Basic(C.Structure):
    _fields_=[(k,C.c_longlong) for k in ('user','kernel','period_user','period_kernel')]+[(k,W.DWORD) for k in ('faults','processes','active','terminated')]


class Io(C.Structure):
    _fields_=[(k,C.c_ulonglong) for k in ('read_ops','write_ops','other_ops','read_bytes','write_bytes','other_bytes')]


class Accounting(C.Structure):
    _fields_=[('basic',Basic),('io',Io)]


if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'results/remaining_prism_critical_path'
    mode=sys.argv[1]
    if not mode.replace('_','').isalnum():raise ValueError('InvalidControlName')
    k=C.WinDLL('kernel32',use_last_error=True)
    k.CreateJobObjectW.argtypes=[C.c_void_p,W.LPCWSTR];k.CreateJobObjectW.restype=W.HANDLE
    k.AssignProcessToJobObject.argtypes=[W.HANDLE,W.HANDLE];k.AssignProcessToJobObject.restype=W.BOOL
    k.QueryInformationJobObject.argtypes=[W.HANDLE,C.c_int,C.c_void_p,W.DWORD,C.c_void_p];k.QueryInformationJobObject.restype=W.BOOL
    k.CloseHandle.argtypes=[W.HANDLE]
    job=k.CreateJobObjectW(None,None)
    if not job:raise C.WinError(C.get_last_error())
    begin=time.perf_counter()
    try:
        with (out/(mode+'_PROCESS.log')).open('wb') as log:
            process=subprocess.Popen([sys.executable,str(Path(__file__).with_name('profile_remaining_prism.py')),'normal',mode],stdout=log,stderr=subprocess.STDOUT,creationflags=4)
            if not k.AssignProcessToJobObject(job,W.HANDLE(process._handle)):
                process.kill();raise C.WinError(C.get_last_error())
            psutil.Process(process.pid).resume()
            result=process.wait()
        value=Accounting()
        if not k.QueryInformationJobObject(job,8,C.byref(value),C.sizeof(value),None):raise C.WinError(C.get_last_error())
        h.durable_json(out/(mode+'_JOB.json'),dict(exit_code=result,launcher_wall=time.perf_counter()-begin,
            cpu=(value.basic.user+value.basic.kernel)/1e7,processes=value.basic.processes,active=value.basic.active,
            io={name:getattr(value.io,name) for name,_ in Io._fields_},scope='Whole process family including imports, worker startup, instrumentation and exit; clean production phases are in NORMAL.json'))
        raise SystemExit(result)
    finally:
        k.CloseHandle(job)
