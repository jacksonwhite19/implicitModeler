"""Explicit current/transaction/replay reachability for a working block store.

Only local authenticated block/node objects are collected. Sealed external authorities are never
opened for writing. Old unpinned working checkpoint envelopes are provenance,
not a promise of indefinite restartability. Pin before replacing CURRENT.
"""
import contextlib,hashlib,json,os,threading
from pathlib import Path
from ..common.authority import identity

VERSION='V4BlockWorkingStore/1'
_HELD=threading.local()

def root_identity(row):
    return None if row is None else (row['authority'],row['schema'],row['root'],row['count'])

def write(path,data):
    from .native_checkpoint import atomic_json
    atomic_json(path,dict(payload=data,sha256=identity(data)))

def read(path,default=None):
    if not path.exists():return default
    row=json.loads(path.read_text())
    if identity(row['payload'])!=row['sha256']:raise ValueError('BlockRetentionManifestCorrupt')
    return row['payload']

@contextlib.contextmanager
def locked(path):
    path=Path(path);path.mkdir(parents=True,exist_ok=True)
    key=os.path.normcase(str(path.resolve()));held=getattr(_HELD,'paths',None)
    if held is None:held=_HELD.paths=set()
    if key in held:
        yield
        return
    with (path/'BLOCK_STORE.lock').open('a+b') as stream:
        if stream.tell()==0:stream.write(b'0');stream.flush()
        stream.seek(0)
        if os.name=='nt':
            import msvcrt
            msvcrt.locking(stream.fileno(),msvcrt.LK_NBLCK,1)
        else:
            import fcntl
            fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        held.add(key)
        try:yield
        finally:
            held.remove(key)
            stream.seek(0)
            if os.name=='nt':msvcrt.locking(stream.fileno(),msvcrt.LK_UNLCK,1)
            else:fcntl.flock(stream,fcntl.LOCK_UN)

def current(path):
    pointer=Path(path)/'CURRENT.json'
    if not pointer.exists():return None
    p=json.loads(pointer.read_text());name=p['checkpoint']
    if Path(name).name!=name:raise ValueError('BlockRetentionCheckpointPath')
    data=(Path(path)/name).read_bytes()
    if hashlib.sha256(data).hexdigest()!=p['sha256']:raise ValueError('BlockRetentionCheckpointCorrupt')
    envelope=json.loads(data)
    return envelope['components']['certificate'].get('authenticated')

def collect(path):
    """Caller holds store lock; current and explicit pins are the GC roots."""
    from .canonical_blocks import AUTHORITY,verify
    path=Path(path);objects=(path/'objects').resolve();roots=[]
    now=current(path)
    if now:roots.append(now)
    pins=read(path/'BLOCK_PINS.json',{})
    roots.extend(pins.values())
    transaction=read(path/'BLOCK_TRANSACTION.json',None)
    if transaction:roots.extend(x for x in (transaction.get('before'),transaction.get('candidate')) if x)
    for root in roots:
        if root['authority']==AUTHORITY:verify(root,objects)
        else:
            from .authenticated_receipts import AUTHORITY as LEGACY
            from .verified_objects import verify as verify_file
            if root['authority']!=LEGACY:raise ValueError('UnknownRetentionRootAuthority')
            for row in root['files']:
                if Path(row['file']).name!=row['file']:raise ValueError('RetentionObjectPath')
                verify_file(objects/row['file'],row['sha256'])
    live={x['file'] for root in roots for x in root['files']}
    removed=0;removed_bytes=0;retained=0;legacy_removed=0
    for item in list(objects.glob('*.cblock'))+list(objects.glob('*.merkle.sqlite')):
        if item.resolve().parent!=objects:raise ValueError('BlockCollectionOutsideStore')
        size=item.stat().st_size
        if item.name not in live:
            item.unlink();removed+=1;removed_bytes+=size
            if item.name.endswith('.merkle.sqlite'):legacy_removed+=size
        else:retained+=size
    result=dict(schema=VERSION,current_root=now['root'] if now else None,pinned_roots={k:v['root'] for k,v in pins.items()},removed_blocks=removed,removed_bytes=removed_bytes,legacy_node_bytes_unlinked=legacy_removed,retained_bytes=retained,retained_blocks=len(live))
    write(path/'BLOCK_STORAGE.json',result)
    return result

def pin(path,name,descriptor):
    from .canonical_blocks import verify
    path=Path(path)
    with locked(path):
        verify(descriptor,path/'objects');pins=read(path/'BLOCK_PINS.json',{})
        if name in pins and pins[name]!=descriptor:raise ValueError('ReplayPinAlreadyExists')
        pins[name]=descriptor;write(path/'BLOCK_PINS.json',pins)

def unpin(path,name):
    path=Path(path)
    with locked(path):
        pins=read(path/'BLOCK_PINS.json',{});del pins[name];write(path/'BLOCK_PINS.json',pins)
        return collect(path)

def recover(path):
    path=Path(path)
    with locked(path):
        transaction=read(path/'BLOCK_TRANSACTION.json',None)
        if transaction:
            now=current(path)
            if root_identity(now) not in (root_identity(transaction['before']),root_identity(transaction['candidate'])):raise ValueError('BlockTransactionCurrentConflict')
            (path/'BLOCK_TRANSACTION.json').unlink()
        return collect(path)

@contextlib.contextmanager
def publication(path,candidate):
    path=Path(path)
    with locked(path):
        if (path/'BLOCK_TRANSACTION.json').exists():raise ValueError('BlockTransactionRecoveryRequired')
        write(path/'BLOCK_TRANSACTION.json',dict(schema=VERSION,before=current(path),candidate=candidate))
        yield
        if root_identity(current(path))!=root_identity(candidate):raise ValueError('BlockPublicationRootMismatch')
        (path/'BLOCK_TRANSACTION.json').unlink()
        collect(path)
