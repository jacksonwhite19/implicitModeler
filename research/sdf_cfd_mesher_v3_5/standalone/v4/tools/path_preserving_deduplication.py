"""Bounded NTFS hardlink coalescing for sealed, immutable native objects.

Never rewrites object contents or checkpoint descriptors. A durable journal and
same-directory atomic rename preserve the old or the identical new pathname.
This is storage tooling, not a meshing or checkpoint publication implementation.
"""
import ctypes
from ctypes import wintypes as W
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
from functools import lru_cache


class Standard(ctypes.Structure):
    _fields_ = [('allocation', ctypes.c_longlong), ('end', ctypes.c_longlong),
                ('links', W.DWORD), ('pending', ctypes.c_ubyte), ('directory', ctypes.c_ubyte)]


def api():
    k = ctypes.WinDLL('kernel32', use_last_error=True)
    k.CreateFileW.argtypes = [W.LPCWSTR, W.DWORD, W.DWORD, ctypes.c_void_p, W.DWORD, W.DWORD, W.HANDLE]
    k.CreateFileW.restype = W.HANDLE
    k.CloseHandle.argtypes = [W.HANDLE]
    k.GetFileInformationByHandleEx.argtypes = [W.HANDLE, ctypes.c_int, ctypes.c_void_p, W.DWORD]
    k.ReadFile.argtypes = [W.HANDLE, ctypes.c_void_p, W.DWORD, ctypes.POINTER(W.DWORD), ctypes.c_void_p]
    k.MoveFileExW.argtypes = [W.LPCWSTR, W.LPCWSTR, W.DWORD]
    return k


@contextmanager
def locked(path):
    """Deny content writers while permitting the required atomic rename."""
    k = api()
    h = k.CreateFileW(str(Path(path).resolve()), 0x80000000, 5, None, 3, 0, None)
    if h == W.HANDLE(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        yield h
    finally:
        k.CloseHandle(h)


def standard(handle):
    result = Standard()
    if not api().GetFileInformationByHandleEx(handle, 1, ctypes.byref(result), ctypes.sizeof(result)):
        raise ctypes.WinError(ctypes.get_last_error())
    return dict(allocated=result.allocation, bytes=result.end, links=result.links,
                delete_pending=bool(result.pending))


@lru_cache(maxsize=None)
def cluster_size(root):
    k=api()
    k.GetDiskFreeSpaceW.argtypes=[W.LPCWSTR,*([ctypes.POINTER(W.DWORD)]*4)]
    sectors,sector_bytes,free,total=(W.DWORD() for _ in range(4))
    if not k.GetDiskFreeSpaceW(root,ctypes.byref(sectors),ctypes.byref(sector_bytes),ctypes.byref(free),ctypes.byref(total)):
        raise ctypes.WinError(ctypes.get_last_error())
    return sectors.value*sector_bytes.value


def cluster_allocation(handle,cluster_bytes):
    """Count NTFS allocated clusters from retrieval pointers; resident data=0.

    MFT record reuse is deliberately not claimed as recovered disk allocation.
    Cluster size is measured from the file's volume, never inferred from size.
    """
    k=api()
    k.DeviceIoControl.argtypes=[W.HANDLE,W.DWORD,ctypes.c_void_p,W.DWORD,
                               ctypes.c_void_p,W.DWORD,ctypes.POINTER(W.DWORD),ctypes.c_void_p]
    position=ctypes.c_longlong(0); total=0; count=W.DWORD()
    buf=ctypes.create_string_buffer(65536)
    while True:
        ok=k.DeviceIoControl(handle,0x90073,ctypes.byref(position),8,buf,len(buf),ctypes.byref(count),None)
        error=ctypes.get_last_error()
        if not ok and error==38: return total*cluster_bytes
        if not ok and error!=234: raise ctypes.WinError(error)
        import struct
        extents=struct.unpack_from('<I',buf.raw,0)[0]
        current=struct.unpack_from('<q',buf.raw,8)[0]
        for i in range(extents):
            next_vcn,lcn=struct.unpack_from('<qq',buf.raw,16+16*i)
            if lcn>=0: total+=next_vcn-current
            current=next_vcn
        if ok:return total*cluster_bytes
        position.value=current


def handle_hash(handle):
    digest = hashlib.sha256()
    block = ctypes.create_string_buffer(4 * 1024 * 1024)
    count = W.DWORD()
    k = api()
    while True:
        if not k.ReadFile(handle, block, len(block), ctypes.byref(count), None):
            raise ctypes.WinError(ctypes.get_last_error())
        if not count.value:
            break
        digest.update(block.raw[:count.value])
    return digest.hexdigest()


def file_hash(path):
    with locked(path) as h:
        return handle_hash(h)


def security_hash(path):
    a = ctypes.WinDLL('advapi32', use_last_error=True)
    a.GetFileSecurityW.argtypes = [W.LPCWSTR, W.DWORD, ctypes.c_void_p, W.DWORD, ctypes.POINTER(W.DWORD)]
    size = W.DWORD()
    a.GetFileSecurityW(str(path), 7, None, 0, ctypes.byref(size))
    buf = ctypes.create_string_buffer(size.value)
    if not a.GetFileSecurityW(str(path), 7, buf, len(buf), ctypes.byref(size)):
        raise ctypes.WinError(ctypes.get_last_error())
    return hashlib.sha256(buf.raw[:size.value]).hexdigest()


def streams(path):
    class Data(ctypes.Structure):
        _fields_ = [('size', ctypes.c_longlong), ('name', W.WCHAR * 296)]
    k = api()
    k.FindFirstStreamW.argtypes = [W.LPCWSTR, ctypes.c_int, ctypes.c_void_p, W.DWORD]
    k.FindFirstStreamW.restype = W.HANDLE
    k.FindNextStreamW.argtypes = [W.HANDLE, ctypes.c_void_p]
    k.FindClose.argtypes = [W.HANDLE]
    d = Data()
    h = k.FindFirstStreamW(str(path), 0, ctypes.byref(d), 0)
    if h == W.HANDLE(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    result = [d.name]
    try:
        while k.FindNextStreamW(h, ctypes.byref(d)):
            result.append(d.name)
        if ctypes.get_last_error() != 38:
            raise ctypes.WinError(ctypes.get_last_error())
    finally:
        k.FindClose(h)
    return result


def metadata(path, content=False):
    if Path(path).lstat().st_file_attributes & 0x400:
        raise ValueError('ReparsePointNotSupported')
    path = Path(path).resolve()
    with locked(path) as h:
        st = path.stat()
        result = dict(path=str(path), device=st.st_dev, file_id=str(st.st_ino),
                      attributes=st.st_file_attributes, mtime_ns=st.st_mtime_ns,
                      security=security_hash(path), streams=streams(path), **standard(h))
        result['cluster_bytes']=cluster_allocation(h,cluster_size(path.anchor))
        if content:
            result['sha256'] = handle_hash(h)
        assert path.stat().st_ino == st.st_ino, 'ConcurrentPathReplacement'
        return result


def physical_key(row):
    return (row['device'], row['file_id'])


def probe_file_id(file_id,drive='C:'):
    """Return native error code, or zero if the physical file ID opens.

    A retired ID returns ERROR_INVALID_PARAMETER on the qualified NTFS host;
    callers must measure a deleted-file control, not assume FILE_NOT_FOUND.
    """
    class IdUnion(ctypes.Union):
        _fields_=[('file_id',ctypes.c_longlong),('extended',ctypes.c_ubyte*16)]
    class Descriptor(ctypes.Structure):
        _fields_=[('size',W.DWORD),('kind',ctypes.c_int),('identity',IdUnion)]
    k=api()
    k.OpenFileById.argtypes=[W.HANDLE,ctypes.POINTER(Descriptor),W.DWORD,W.DWORD,ctypes.c_void_p,W.DWORD]
    k.OpenFileById.restype=W.HANDLE
    volume=k.CreateFileW('\\\\.\\'+drive,0,7,None,3,0,None)
    if volume==W.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    try:
        d=Descriptor();d.size=ctypes.sizeof(d);d.kind=0;d.identity.file_id=int(file_id)
        h=k.OpenFileById(volume,ctypes.byref(d),0x80,7,None,0)
        if h==W.HANDLE(-1).value:return ctypes.get_last_error()
        k.CloseHandle(h)
        return 0
    finally:k.CloseHandle(volume)


def durable_json(path, value):
    path = Path(path)
    pending = path.with_name(path.name + '.pending')
    with pending.open('w', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    atomic_replace(pending, path)


def atomic_replace(source, target):
    # Same-volume rename; COPY_ALLOWED is deliberately absent.
    if not api().MoveFileExW(str(Path(source).resolve()), str(Path(target).resolve()), 1 | 8):
        raise ctypes.WinError(ctypes.get_last_error())


def replace_open_destination(source,target):
    """Atomic NTFS rename while content-write exclusion stays in force."""
    class Rename(ctypes.Structure):
        _fields_=[('flags',W.DWORD),('root',W.HANDLE),('length',W.DWORD),('name',W.WCHAR*1)]
    name=str(Path(target).resolve()).encode('utf-16-le')
    buf=ctypes.create_string_buffer(max(ctypes.sizeof(Rename),Rename.name.offset+len(name)+2))
    record=Rename.from_buffer(buf)
    record.flags=3  # REPLACE_IF_EXISTS | POSIX_SEMANTICS
    record.root=None;record.length=len(name)
    ctypes.memmove(ctypes.addressof(buf)+Rename.name.offset,name,len(name))
    k=api();k.SetFileInformationByHandle.argtypes=[W.HANDLE,ctypes.c_int,ctypes.c_void_p,W.DWORD]
    h=k.CreateFileW(str(Path(source).resolve()),0x10000,7,None,3,0,None)
    if h==W.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    try:
        if not k.SetFileInformationByHandle(h,22,buf,len(buf)):
            raise ctypes.WinError(ctypes.get_last_error())
    finally:k.CloseHandle(h)


def eligible(row):
    return (row.get('immutable') is True and bool(row.get('references'))
            and all(ref.get('sha256') == row.get('sha256') for ref in row['references'])
            and row['streams'] == ['::$DATA']
            and not row['attributes'] & (0x400 | 0x800 | 0x200 | 0x4000 | 1))


def replace_duplicate(original, canonical, journal, interrupt=lambda stage: None):
    """Only callers with a sealed immutable manifest may authorize a row.

    Re-runs accept a fully completed replacement. Interrupted stages are
    recoverable from the journal and exact canonical bytes, without deleting a
    scientific pathname or retaining a second full payload backup.
    """
    if not eligible(original) or not eligible(canonical):
        raise ValueError('MutableOrUnauthenticatedObject')
    if original['device'] != canonical['device']:
        raise ValueError('CrossVolume')
    if original['sha256'] != canonical['sha256'] or original['bytes'] != canonical['bytes']:
        raise ValueError('ContentMismatch')
    if (original['security'], original['attributes'], original['streams']) != (
            canonical['security'], canonical['attributes'], canonical['streams']):
        raise ValueError('MetadataMismatch')
    target, source = Path(original['path']), Path(canonical['path'])
    temporary = target.with_name(target.name + '.dedup-link')
    with locked(source) as sh, locked(target) as th:
        now_source, now_target = metadata(source), metadata(target)
        if physical_key(now_source) != physical_key(canonical):
            raise ValueError('StaleCanonical')
        for now, expected in ((now_source, canonical), (now_target, original)):
            for key in ('bytes', 'cluster_bytes', 'security', 'attributes', 'streams'):
                if now[key] != expected[key]:
                    raise ValueError('StaleMetadata')
        if handle_hash(sh) != canonical['sha256'] or handle_hash(th) != original['sha256']:
            raise ValueError('StaleContent')
        if physical_key(now_source) == physical_key(now_target):
            if Path(journal).exists():
                recovered=json.loads(Path(journal).read_text())
                if recovered.get('original',{}).get('path') != str(target):
                    raise ValueError('ForeignJournal')
                recovered.update(stage='verified',recovered=True,after=metadata(target,content=True))
                durable_json(journal,recovered)
            return dict(status='already_shared', path=str(target), canonical=str(source))
        if physical_key(now_target) != physical_key(original):
            raise ValueError('StaleVictim')
        record = dict(original=original, canonical=canonical, temporary=str(temporary), stage='prepared')
        durable_json(journal, record)
        interrupt('prepared')
        if temporary.exists():
            if physical_key(metadata(temporary)) != physical_key(now_source):
                raise ValueError('ForeignTemporary')
        else:
            os.link(source, temporary)
        record['stage'] = 'linked'
        durable_json(journal, record)
        interrupt('linked')
        replace_open_destination(temporary, target)
        record['stage'] = 'replaced'
        record['old_handle_after_replace'] = standard(th)
        durable_json(journal, record)
        interrupt('replaced')
    after = metadata(target, content=True)
    if after['sha256'] != original['sha256'] or physical_key(after) != physical_key(canonical):
        raise ValueError('ReplacementVerificationFailed')
    record.update(stage='verified', after=after)
    durable_json(journal, record)
    return record
