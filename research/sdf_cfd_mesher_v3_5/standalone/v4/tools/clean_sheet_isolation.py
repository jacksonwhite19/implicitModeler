"""Allowlisted candidate I/O. Oracle paths never become candidate inputs.

Python open/sqlite/import events and explicit Win32 opens are recorded. Native
oracle handles deny read sharing as a second barrier. The hooks are process
lifetime; use a fresh worker for each phase. This is not a hostile-code sandbox.
"""
from collections import Counter
from pathlib import Path
import ctypes,json,os,sys
from contextlib import contextmanager

class OracleIsolationError(RuntimeError):pass

class ReadBoundary:
    def __init__(self,repo,files,output,runtime=()):
        self.repo=Path(repo).resolve();self.output=Path(output).resolve()
        self.files={os.path.normcase(str(Path(p).resolve())) for p in files}
        self.runtime=[Path(p).resolve() for p in runtime if p and not Path(p).resolve().is_relative_to(self.repo)]
        self.reads=Counter();self.writes=Counter();self.denials=[];self.installed=False
    def allowed(self,p,write=False):
        if p.is_relative_to(self.output):return True
        if write:return False
        if os.path.normcase(str(p)) in self.files:return True
        v4=self.repo/'research/sdf_cfd_mesher_v4'
        if any(p.is_relative_to(v4/k) for k in ('src','tools')) and p.suffix in ('.py','.pyc','.pyd','.dll'):return True
        # The qualified surface loader imports Authority/Seed record classes
        # from this existing V3 implementation. No data or writes are allowed.
        for folder in ('adaptive_surface_generic_qualification','prism_feasibility_contract'):
            source=self.repo/'research/sdf_cfd_mesher_v3'/folder
            if p.is_relative_to(source) and p.suffix in ('.py','.pyc'):return True
        return any(p.is_relative_to(r) for r in self.runtime)
    def check(self,path,write=False,event='open'):
        if isinstance(path,int):return
        p=Path(os.fsdecode(path)).resolve()
        if not self.allowed(p,write):
            self.denials.append(dict(path=str(p),event=event,write=write))
            raise OracleIsolationError('UnapprovedCandidateIO:'+str(p))
        (self.writes if write else self.reads)[(str(p),event)]+=1
    def hook(self,event,args):
        if event=='open':
            path,mode,flags=args;write=bool(flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
            self.check(path,write,event)
        elif event=='sqlite3.connect':
            path=args[0]
            if path==':memory:':return
            if str(path).startswith('file:'):
                from urllib.parse import urlparse,unquote
                path=unquote(urlparse(str(path)).path)
                if len(path)>2 and path[0]=='/' and path[2]==':':path=path[1:]
            self.check(path,False,event)
        elif event in ('subprocess.Popen','os.system','os.spawn','os.exec'):
            self.denials.append(dict(event=event));raise OracleIsolationError('UnqualifiedChildProcess')
    def install(self):
        sys.addaudithook(self.hook);self.installed=True
    def record(self):
        def rows(c):return [dict(path=p,event=e,count=n) for (p,e),n in sorted(c.items())]
        return dict(reads=rows(self.reads),writes=rows(self.writes),denials=list(self.denials),
            allowed_files=sorted(self.files),runtime=[str(p) for p in self.runtime],oracle_unlocked=False)

@contextmanager
def deny_native_reads(paths,allowed_ids):
    """Delete-access exclusive handles; no reads, deletion, or ACL edits.

An alias of a legitimate L4 file is exempt from the physical lock. The candidate
path allowlist still prohibits opening that file through an oracle pathname.
"""
    from path_preserving_deduplication import api,W
    k=api();handles=[];rows=[]
    try:
        for p in paths:
            p=Path(p);s=p.stat();key=(s.st_dev,s.st_ino)
            if key in allowed_ids:
                rows.append(dict(path=str(p),role='L4 immutable alias; path still denied'));continue
            h=k.CreateFileW(str(p.resolve()),0x00010000,0,None,3,0,None)
            if h==W.HANDLE(-1).value:raise OSError(ctypes.get_last_error(),'CannotExcludeOracleRead',str(p))
            handles.append(h);rows.append(dict(path=str(p),role='Read sharing denied; DELETE access only; no delete operation'))
        yield rows
    finally:
        for h in handles:k.CloseHandle(h)
