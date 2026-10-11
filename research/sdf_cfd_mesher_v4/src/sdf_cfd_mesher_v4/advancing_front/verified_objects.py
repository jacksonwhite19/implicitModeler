"""Process-local byte verification of immutable content-addressed objects."""
from collections import OrderedDict
from pathlib import Path
from ..common.authority import file_hash

VERIFIED=OrderedDict()
STATS=dict(bytes_verified=0,reused=0)


def verify(path,expected):
    from .native_checkpoint import file_change_time
    path=Path(path);s=path.stat()
    token=(str(path.resolve()),s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,file_change_time(path,s),expected)
    if token in VERIFIED:
        VERIFIED.move_to_end(token);STATS['reused']+=1
        return path
    if file_hash(path)!=expected:raise ValueError('AuthenticatedObjectCorrupt')
    after=path.stat()
    if (after.st_size,after.st_mtime_ns,file_change_time(path,after))!=(s.st_size,s.st_mtime_ns,token[5]):
        raise ValueError('AuthenticatedObjectChangedDuringVerification')
    VERIFIED[token]=True;STATS['bytes_verified']+=s.st_size
    if len(VERIFIED)>4096:VERIFIED.popitem(last=False)
    return path
