"""Windows no-reparse paths and handles for local compact publication only."""
from contextlib import contextmanager
from pathlib import Path, PurePosixPath
import ctypes
from ctypes import wintypes as W
import os, stat, json,uuid,hashlib


def checked(path, *, exists=True):
    raw=Path(path)
    if '..' in raw.parts:raise ValueError('CompactPathTraversal')
    path=Path(os.path.abspath(raw))
    for p in reversed((path,*path.parents)):
        try:s=p.lstat()
        except FileNotFoundError:
            if p==path and not exists:continue
            raise
        if stat.S_ISLNK(s.st_mode) or getattr(s,'st_file_attributes',0)&0x400:
            raise ValueError('CompactReparsePoint',str(p))
    resolved=path.resolve()
    if resolved!=path:
        try:
            state=path.lstat();metadata=dict(exists=True,mode=state.st_mode,attributes=getattr(state,'st_file_attributes',0),identity=[state.st_dev,state.st_ino])
        except FileNotFoundError:metadata=dict(exists=False)
        raise ValueError('CompactPathRedirect',dict(operation='resolve',requested=str(path),resolved=str(resolved),metadata_after_comparison=metadata))
    return path


def allowed_parent(parent):
    from source_custody import ROOT
    parent=checked(parent)
    if parent.is_relative_to(ROOT) and not parent.is_relative_to(ROOT/'results'):
        raise ValueError('CompactOutputInsideRuntime')
    return parent


def member(root,name):
    p=PurePosixPath(name)
    if not name or p.is_absolute() or p.as_posix()!=name or any(x in ('.','..') or ':' in x or '\\' in x or x.endswith((' ','.')) or x.split('.')[0].upper() in {'CON','PRN','AUX','NUL',*[f'COM{i}' for i in range(1,10)],*[f'LPT{i}' for i in range(1,10)]} for x in p.parts):
        raise ValueError('CompactMemberPath',name)
    value=checked(root/name)
    if not value.is_relative_to(root) or value==root:raise ValueError('CompactPathEscape')
    return value


def file_id(path):
    s=path.lstat();return (s.st_dev,s.st_ino)


class Handle:
    """No write/replacement sharing; DELETE capability is never serialized."""
    def __init__(self,path,*,delete=False,strict=True):
        if os.name!='nt':raise ValueError('CompactRequiresQualifiedWindowsFilesystem')
        self.path=checked(path);before=file_id(self.path)
        k=self.k=ctypes.WinDLL('kernel32',use_last_error=True)
        k.CreateFileW.argtypes=[W.LPCWSTR,W.DWORD,W.DWORD,ctypes.c_void_p,W.DWORD,W.DWORD,W.HANDLE]
        k.CreateFileW.restype=W.HANDLE
        k.CloseHandle.argtypes=[W.HANDLE];k.CloseHandle.restype=W.BOOL
        k.GetFileInformationByHandleEx.argtypes=[W.HANDLE,ctypes.c_int,ctypes.c_void_p,W.DWORD]
        k.GetFileInformationByHandleEx.restype=W.BOOL
        k.SetFileInformationByHandle.argtypes=[W.HANDLE,ctypes.c_int,ctypes.c_void_p,W.DWORD]
        k.SetFileInformationByHandle.restype=W.BOOL
        self.h=k.CreateFileW(str(self.path),0x80000000|(0x10000 if delete else 0),1 if strict else 3,None,3,0x02200000,None)
        if self.h==W.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
        self.delete_allowed=delete
        try:
            attrs=(W.DWORD*2)()
            if not k.GetFileInformationByHandleEx(self.h,9,ctypes.byref(attrs),ctypes.sizeof(attrs)):
                raise ctypes.WinError(ctypes.get_last_error())
            if attrs[0]&0x400 or file_id(self.path)!=before:raise ValueError('CompactHandleRedirect')
        except BaseException:self.close();raise
    def close(self):
        if self.h is not None:
            h,self.h=self.h,None
            if not self.k.CloseHandle(h):raise ctypes.WinError(ctypes.get_last_error())
    def dispose(self):
        if not self.delete_allowed:raise ValueError('NoOwnedDeleteCapability')
        flag=W.BOOL(True)
        if not self.k.SetFileInformationByHandle(self.h,4,ctypes.byref(flag),ctypes.sizeof(flag)):
            raise ctypes.WinError(ctypes.get_last_error())
    def rename_flushed(self,destination,payload):
        """Rename the same pinned file, refusing replacement; parent is pinned."""
        require=self.delete_allowed
        if not require:raise ValueError('RenameCapabilityMissing')
        self.k.ReadFile.argtypes=[W.HANDLE,ctypes.c_void_p,W.DWORD,ctypes.POINTER(W.DWORD),ctypes.c_void_p];self.k.ReadFile.restype=W.BOOL
        buffer=ctypes.create_string_buffer(len(payload)+1);count=W.DWORD()
        if not self.k.ReadFile(self.h,buffer,len(payload)+1,ctypes.byref(count),None):raise ctypes.WinError(ctypes.get_last_error())
        if buffer.raw[:count.value]!=payload:raise ValueError('ChangedPendingCompactCommit')
        encoded=str(destination).encode('utf-16-le')
        class Rename(ctypes.Structure):
            _fields_=[('replace',ctypes.c_ubyte),('root',W.HANDLE),('length',W.DWORD),('name',W.WCHAR*1)]
        # Exact flexible-array extent, with an explicit UTF-16 terminator.
        # Never expose structure tail padding as part of the target name.
        size=Rename.name.offset+len(encoded)+2
        data=ctypes.create_string_buffer(size)
        value=ctypes.cast(data,ctypes.POINTER(Rename)).contents
        value.replace=0;value.root=None;value.length=len(encoded)
        ctypes.memmove(ctypes.addressof(data)+Rename.name.offset,encoded,len(encoded))
        if not self.k.SetFileInformationByHandle(self.h,3,data,size):raise ctypes.WinError(ctypes.get_last_error())
    def __enter__(self):return self
    def __exit__(self,*args):self.close()


def hold_parents(path,scope):
    path=checked(path)
    for p in reversed((path,*path.parents)):
        # Ancestors stay nonreplaceable. Pinned child directories keep them
        # nonempty; Windows refuses setting reparse points on nonempty dirs.
        scope.enter_context(Handle(p,strict=False))
    return path


def inventory(root):
    root=checked(root);files=[]
    for directory,dirs,names in os.walk(root,followlinks=False):
        for name in dirs+names:checked(Path(directory)/name)
        files.extend(Path(directory)/n for n in names)
    return sorted(files)


def hold_tree(root,scope,*,strict_root=True):
    root=hold_parents(root,scope)
    if strict_root:scope.enter_context(Handle(root))
    for directory,dirs,names in os.walk(root,followlinks=False):
        if Path(directory)!=root:scope.enter_context(Handle(directory))
        for name in dirs+names:checked(Path(directory)/name)
    return root


def publish(folder,value):
    """Atomic no-replace file commit after a flushed private marker.

    The reserved directory remains provisional until COMPACT.json appears.
    Guarded payloads and the pinned nonempty root remain held across commit.
    """
    if os.name!='nt':raise ValueError('CompactAtomicNoReplaceRequiresWindows')
    folder=checked(folder);path=folder/'COMPACT.json';checked(path,exists=False)
    payload=(json.dumps(value,indent=2,allow_nan=False)+'\n').encode('utf-8')
    pending=folder/('.COMPACT-'+uuid.uuid4().hex+'.tmp')
    with pending.open('xb') as f:f.write(payload);f.flush();os.fsync(f.fileno())
    with Handle(pending,delete=True) as h:h.rename_flushed(path,payload)
    return hashlib.sha256(payload).hexdigest()
