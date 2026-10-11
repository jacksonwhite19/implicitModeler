"""Capacity management for newly created, non-shared study receipt artifacts.

This is not part of authenticated block storage or its performance claim.
Never compress a hard link shared with a preserved external authority.
Run between processes or after the producer closes an immutable component.
Never compress active SQLite scratch. Logical bytes and hashes stay exact.
"""
import ctypes,hashlib,json,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'results/canonical_block_authenticated_storage'

def allocated(path):
    high=ctypes.c_ulong()
    fn=ctypes.windll.kernel32.GetCompressedFileSizeW
    fn.argtypes=[ctypes.c_wchar_p,ctypes.POINTER(ctypes.c_ulong)]
    fn.restype=ctypes.c_ulong
    low=fn(str(path.resolve()),ctypes.byref(high))
    return (high.value<<32)|low

def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def compress(path):
    # Windows' native operation supports extended paths used by checkpoints.
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    create=kernel.CreateFileW
    create.argtypes=[ctypes.c_wchar_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_void_p]
    create.restype=ctypes.c_void_p
    flags=0x02000000 if path.is_dir() else 0
    handle=create('\\\\?\\'+str(path.resolve()),0xC0000000,7,None,3,flags,None)
    if handle==ctypes.c_void_p(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    control=kernel.DeviceIoControl
    control.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_void_p,ctypes.c_void_p]
    close=kernel.CloseHandle;close.argtypes=[ctypes.c_void_p]
    try:
        mode=ctypes.c_ushort(1);returned=ctypes.c_ulong()
        if not control(handle,0x9C040,ctypes.byref(mode),2,None,0,ctypes.byref(returned),None):raise ctypes.WinError(ctypes.get_last_error())
    finally:close(handle)

def run(directory=None):
    log=ROOT/'FILESYSTEM_CAPACITY.jsonl'
    directory=Path(directory or ROOT).resolve()
    assert directory.is_relative_to(ROOT.resolve())
    for path in sorted(directory.rglob('*')):
        if not path.is_file() or path.suffix not in ('.sqlite','.json'):continue
        stat=path.stat()
        if stat.st_nlink!=1 or stat.st_size<16*1024*1024:continue
        if allocated(path)<stat.st_size:continue
        assert path.resolve().is_relative_to(ROOT.resolve())
        before=digest(path);physical=allocated(path);tick=time.perf_counter()
        compress(path)
        assert path.stat().st_nlink==1 and path.stat().st_size==stat.st_size and digest(path)==before
        row=dict(path=path.relative_to(ROOT).as_posix(),sha256=before,logical_bytes=stat.st_size,
                 physical_before=physical,physical_after=allocated(path),seconds=time.perf_counter()-tick)
        with log.open('a') as out:out.write(json.dumps(row)+'\n')
        print('CAPACITY '+json.dumps(row),flush=True)

if __name__=='__main__':run()
