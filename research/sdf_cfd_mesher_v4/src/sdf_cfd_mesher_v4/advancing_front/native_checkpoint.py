"""Native component checkpoints with immutable disk-backed evidence tables.

SQLite is storage, not a proof engine. Canonical scientific JSON defines table
identity independently of SQLite page order. CURRENT is the only publisher.
"""
from collections.abc import Mapping, MutableMapping
from collections import OrderedDict
from copy import deepcopy
import heapq
from contextlib import closing
from pathlib import Path
import hashlib,json,sqlite3,os
from ..common.authority import file_hash, identity
from .rolling_state import atomic_json

FORMAT='V4NativeMixedCheckpoint/1'
ENCODER=json.JSONEncoder(sort_keys=True,separators=(',',':'),allow_nan=False)
_VERIFIED_RECEIPTS=OrderedDict()
RECEIPT_AUDIT_STATS=dict(full=0,reused=0)


def file_change_time(path, stat):
    if os.name!='nt':return stat.st_ctime_ns
    # Windows st_ctime is creation time, not inode change time. Query the
    # actual change timestamp so copying/restoring LastWriteTime cannot hide
    # an altered immutable component from the in-process identity cache.
    import ctypes
    from ctypes import wintypes
    class BasicInfo(ctypes.Structure):
        _fields_=[(k,ctypes.c_longlong) for k in ('creation','access','write','change')]+[('attributes',wintypes.DWORD)]
    api=ctypes.WinDLL('kernel32',use_last_error=True)
    api.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE]
    api.CreateFileW.restype=wintypes.HANDLE
    api.GetFileInformationByHandleEx.argtypes=[wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD]
    api.GetFileInformationByHandleEx.restype=wintypes.BOOL
    api.CloseHandle.argtypes=[wintypes.HANDLE]
    handle=api.CreateFileW(str(path.resolve()),0x80,7,None,3,0,None)
    if handle==wintypes.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    try:
        result=BasicInfo()
        if not api.GetFileInformationByHandleEx(handle,0,ctypes.byref(result),ctypes.sizeof(result)):
            raise ctypes.WinError(ctypes.get_last_error())
        return result.change
    finally:api.CloseHandle(handle)


def receipt_storage_signature(table):
    from .native_receipt_delta import ReceiptDelta
    if isinstance(table,ReceiptDelta):
        return (receipt_storage_signature(table.base),receipt_storage_signature(table.updates),
                identity(sorted(table.removed)))
    stat=table.path.stat()
    return (str(table.path.resolve()),stat.st_dev,stat.st_ino,stat.st_size,stat.st_mtime_ns,file_change_time(table.path,stat))


def verified_receipt_token(table):
    while isinstance(table,ReceiptOverlay):
        if table.edits or table.removed:return None
        table=table.base
    if not isinstance(table,ReceiptTable):return None
    verified=getattr(table,'_verified_storage',None)
    if verified is None or verified[1]!=receipt_storage_signature(table):return None
    return verified[0]


def canonical_chunks(value):
    if hasattr(value,'native_scientific'):
        yield from canonical_chunks(value.native_scientific());return
    if hasattr(value,'native_canonical_chunks'):
        yield from value.native_canonical_chunks();return
    if isinstance(value,(ReceiptTable,ReceiptOverlay)):
        yield from value.canonical_chunks();return
    if isinstance(value,Mapping):
        yield b'{'
        for index,key in enumerate(sorted(value)):
            if not isinstance(key,str):raise ValueError('NativeScientificKeysMustBeStrings')
            if index:yield b','
            yield ENCODER.encode(key).encode();yield b':'
            item=value[key]
            # A single entity receipt is bounded; component maps are streamed.
            if isinstance(item,Mapping):yield from canonical_chunks(item)
            else:
                for chunk in ENCODER.iterencode(item):yield chunk.encode()
        yield b'}'
    else:
        for chunk in ENCODER.iterencode(value):yield chunk.encode()


def scientific_digest(value):
    h=hashlib.sha256()
    for block in canonical_chunks(value):h.update(block)
    return h.hexdigest()


class ReceiptTable(Mapping):
    """Immutable source-keyed evidence, with no process-wide row cache."""
    def __init__(self,path,count,scientific):
        self.path=Path(path);self.count=int(count);self.scientific=scientific
    def __deepcopy__(self,memo):return ReceiptOverlay(self)
    def __len__(self):return self.count
    def __eq__(self,other):
        if not isinstance(other,Mapping):return NotImplemented
        return len(self)==len(other) and scientific_digest(self)==scientific_digest(other)
    def _connect(self):return sqlite3.connect(self.path.resolve().as_uri()+'?mode=ro',uri=True)
    def __iter__(self):
        with closing(self._connect()) as db:
            for row in db.execute('SELECT key FROM receipts ORDER BY key'):yield row[0]
    def __getitem__(self,key):
        with closing(self._connect()) as db:row=db.execute('SELECT value FROM receipts WHERE key=?',(key,)).fetchone()
        if row is None:raise KeyError(key)
        return json.loads(row[0])
    def items(self):
        with closing(self._connect()) as db:
            for key,value in db.execute('SELECT key,value FROM receipts ORDER BY key'):yield key,json.loads(value)
    def values(self):
        for _,value in self.items():yield value
    def canonical_chunks(self):
        yield b'{'
        with closing(self._connect()) as db:
            for index,(key,value) in enumerate(db.execute('SELECT key,value FROM receipts ORDER BY key')):
                if index:yield b','
                yield ENCODER.encode(key).encode();yield b':';yield value.encode()
        yield b'}'


def receipt_table_from_rows(directory, rows):
    """Stream unique scientific receipts into the existing native table format.

    Input order is not scientific. The canonical digest is computed in key order
    after insertion. No second expanded receipt population is constructed.
    """
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    import tempfile
    fd,name=tempfile.mkstemp(prefix='binding-',suffix='.pending',dir=directory)
    os.close(fd);temporary=Path(name)
    try:
        with closing(sqlite3.connect(temporary)) as db:
            db.execute('CREATE TABLE receipts(key TEXT PRIMARY KEY,value TEXT NOT NULL) WITHOUT ROWID')
            count=0
            try:
                for key,row in rows:
                    db.execute('INSERT INTO receipts VALUES(?,?)',(key,ENCODER.encode(row)));count+=1
            except sqlite3.IntegrityError as exc:
                raise ValueError('DuplicateMixedProof') from exc
            db.commit()
        provisional=ReceiptTable(temporary,count,'')
        scientific=scientific_digest(provisional);target=directory/(scientific+'.sqlite')
        if target.exists():
            if scientific_digest(ReceiptTable(target,count,scientific))!=scientific:
                raise ValueError('ExistingNativeTableCorrupt')
            temporary.unlink()
        else:os.replace(temporary,target)
        return ReceiptTable(target,count,scientific)
    finally:
        if temporary.exists():temporary.unlink()


class ReceiptOverlay(MutableMapping):
    """Private transaction edits share the immutable base, not mutable rows."""
    def __init__(self,base):self.base=base;self.edits={};self.removed=set()
    def __deepcopy__(self,memo):
        result=ReceiptOverlay(self.base)
        result.edits=deepcopy(self.edits,memo);result.removed=set(self.removed)
        return result
    def __getitem__(self,key):
        if key in self.removed:raise KeyError(key)
        if key not in self.edits:self.edits[key]=self.base[key]
        return self.edits[key]
    def __setitem__(self,key,value):self.edits[key]=value;self.removed.discard(key)
    def __delitem__(self,key):
        self[key];self.edits.pop(key,None);self.removed.add(key)
    def __iter__(self):
        for key,_ in self.items():yield key
    def __len__(self):
        return len(self.base)+sum(k not in self.base for k in self.edits)-sum(k in self.base for k in self.removed)
    def __eq__(self,other):
        if not isinstance(other,Mapping):return NotImplemented
        return len(self)==len(other) and scientific_digest(self)==scientific_digest(other)
    def items(self):
        base=((k,v) for k,v in self.base.items() if k not in self.edits and k not in self.removed)
        yield from heapq.merge(base,sorted(self.edits.items()),key=lambda row:row[0])
    def canonical_chunks(self):
        yield b'{'
        for index,(key,value) in enumerate(self.items()):
            if index:yield b','
            yield ENCODER.encode(key).encode();yield b':'
            yield from canonical_chunks(value)
        yield b'}'


class NativeCheckpointStore:
    def __init__(self,path):
        self.path=Path(path);self.objects=self.path/'objects';self.objects.mkdir(parents=True,exist_ok=True)
    def _object(self,descriptor):
        name=descriptor['file']
        if Path(name).name!=name:raise ValueError('InvalidNativeObjectPath')
        p=self.objects/name
        if getattr(self,'authenticated_objects',False):
            from .verified_objects import verify
            return verify(p,descriptor['sha256'])
        if file_hash(p)!=descriptor['sha256']:raise ValueError('NativeObjectCorrupt')
        return p
    def _table(self,value,content_only=False):
        from .native_receipt_delta import ReceiptDelta
        # Only the journal's untouched, direct wrapper is equivalent. Reads
        # through __getitem__ populate edits and deliberately use the fallback.
        # Do not generalize this to subclasses, nested or edited overlays.
        if (type(value) is ReceiptOverlay and not value.edits and not value.removed
                and set(vars(value)) == {'base','edits','removed'}
                and type(value.base) is ReceiptDelta
                and not hasattr(value.base,'authenticated')):
            row=self._table(value.base,content_only=content_only)
            # Reject stale counts/content and missing/corrupt children before
            # CURRENT can switch, including objects already in this store.
            self._receipt(row)
            return row
        if content_only:return self._table_payload(value,content_only=True)
        if hasattr(value,'authenticated'):
            self.authenticated_objects=True
            from .native_receipt_delta import ReceiptDelta
            def parts(table):
                return parts(table.base)|parts(table.updates) if isinstance(table,ReceiptDelta) else {table.path.resolve()}
            with closing(sqlite3.connect(':memory:')) as db:limit=db.getlimit(sqlite3.SQLITE_LIMIT_ATTACHED)
            if len(parts(value))>limit:
                # Explicit backend-limit maintenance; content root unchanged.
                old=value;value=receipt_table_from_rows(self.objects,old.items())
                from .authenticated_receipts import storage_identity
                value.authenticated=dict(old.authenticated,storage=storage_identity(value))
                value.auth_objects=old.auth_objects
                for name in ('qualified_context','proof_segments','proof_objects'):
                    if hasattr(old,name):setattr(value,name,getattr(old,name))
        row=self._table_payload(value)
        if hasattr(value,'authenticated'):
            from .authenticated_receipts import verify
            verify(value,value.auth_objects)
            for descriptor in value.authenticated['files']:
                target=self.objects/descriptor['file'];source=value.auth_objects/descriptor['file']
                if not target.exists():
                    try:os.link(source,target)
                    except OSError:
                        import shutil
                        shutil.copyfile(source,target)
            row['authenticated']=value.authenticated
        context=getattr(value,'qualified_context',None)
        if context is not None:row['qualified_context']=context
        segments=getattr(value,'proof_segments',None)
        if segments is not None:
            source=getattr(value,'proof_objects',value.path.parent)
            for segment in segments:
                for descriptor in [segment[k] for k in ('faces','pairs','codes','payloads')]+list(segment.get('dependency',{}).values()):
                    if Path(descriptor['file']).name!=descriptor['file']:raise ValueError('NativeProofIndexPath')
                    target=self.objects/descriptor['file']
                    if not target.exists():
                        old=source/descriptor['file']
                        if file_hash(old)!=descriptor['sha256']:raise ValueError('NativeProofIndexImportCorrupt')
                        try:os.link(old,target)
                        except OSError:
                            import shutil
                            shutil.copyfile(old,target)
                    if getattr(self,'authenticated_objects',False):
                        from .verified_objects import verify as verify_file
                        verify_file(target,descriptor['sha256'])
                    elif file_hash(target)!=descriptor['sha256']:raise ValueError('NativeProofIndexImportCorrupt')
            row['proof_segments']=segments
        return row
    def _table_payload(self,value,content_only=False):
        from .native_receipt_delta import ReceiptDelta
        if isinstance(value,ReceiptDelta):
            def parts(table):
                return parts(table.base)|parts(table.updates) if isinstance(table,ReceiptDelta) else {table.path.resolve()}
            with closing(sqlite3.connect(':memory:')) as db:
                limit=db.getlimit(sqlite3.SQLITE_LIMIT_ATTACHED)
            # Compact only when the actual SQLite backend attachment limit
            # would prevent the existing validator's SQL lookup interface.
            if len(parts(value))>limit and not hasattr(value,'authenticated'):
                value=receipt_table_from_rows(self.objects,value.items())
            else:
                from .canonical_blocks import AUTHORITY
                children_content=content_only or getattr(value,'authenticated',{}).get('authority')==AUTHORITY
                return dict(encoding='receipt_delta',base=self._table(value.base,content_only=children_content),
                            updates=self._table(value.updates,content_only=children_content),removed=self._json(sorted(value.removed)),
                            count=len(value),scientific=value.scientific)
        return self._flat_table(value)
    def _flat_table(self,value):
        if isinstance(value,ReceiptTable) and value.path.parent.resolve()==self.objects.resolve():
            if getattr(self,'authenticated_objects',False):
                from .authenticated_receipts import storage_identity
                checksum=storage_identity(value)
            else:checksum=file_hash(value.path)
            return dict(encoding='receipt_table',file=value.path.name,sha256=checksum,count=len(value),scientific=value.scientific)
        if isinstance(value,ReceiptTable):
            if verified_receipt_token(value) is None and not hasattr(value,'authenticated'):
                with closing(value._connect()) as db:count=db.execute('SELECT COUNT(*) FROM receipts').fetchone()[0]
                if count!=len(value) or scientific_digest(value)!=value.scientific:raise ValueError('NativeReceiptImportIdentityMismatch')
            checksum=file_hash(value.path);target=self.objects/(checksum+'.sqlite')
            if not target.exists():
                try:os.link(value.path,target)
                except OSError:
                    import shutil
                    shutil.copyfile(value.path,target)
            if file_hash(target)!=checksum:raise ValueError('NativeReceiptImportCorrupt')
            return dict(encoding='receipt_table',file=target.name,sha256=checksum,count=len(value),scientific=value.scientific)
        temporary=self.objects/'receipts.pending'
        if temporary.exists():temporary.unlink()
        db=sqlite3.connect(temporary)
        try:
            db.execute('CREATE TABLE receipts(key TEXT PRIMARY KEY,value TEXT NOT NULL) WITHOUT ROWID')
            h=hashlib.sha256();h.update(b'{');count=0
            iterator=value.items() if isinstance(value,(ReceiptTable,ReceiptOverlay)) else ((k,value[k]) for k in sorted(value))
            for key,row in iterator:
                encoded=ENCODER.encode(row)
                db.execute('INSERT INTO receipts VALUES(?,?)',(key,encoded))
                if count:h.update(b',')
                h.update(ENCODER.encode(key).encode());h.update(b':');h.update(encoded.encode());count+=1
            h.update(b'}');db.commit()
        except BaseException:
            # Unpublished scratch has no checkpoint authority.
            db.close();temporary.unlink(missing_ok=True)
            Path(str(temporary)+'-journal').unlink(missing_ok=True)
            raise
        finally:db.close()
        scientific=h.hexdigest();target=self.objects/(scientific+'.sqlite')
        if target.exists():
            table=ReceiptTable(target,count,scientific)
            if scientific_digest(table)!=scientific:raise ValueError('ExistingNativeTableCorrupt')
            temporary.unlink()
        else:os.replace(temporary,target)
        return dict(encoding='receipt_table',file=target.name,sha256=file_hash(target),count=count,scientific=scientific)

    def _receipt(self,row,verify=True,canonical=True):
        from .native_receipt_delta import ReceiptDelta
        if row['encoding']=='receipt_delta':
            children_canonical=canonical and 'authenticated' not in row
            base=self._receipt(row['base'],verify,children_canonical)
            updates=self._receipt(row['updates'],verify,children_canonical)
            path=self._object(row['removed']) if verify else self.objects/row['removed']['file']
            table=ReceiptDelta(base,updates,json.loads(path.read_text()),count=row['count'],scientific=row['scientific'])
        elif row['encoding']=='receipt_table':
            path=self._object(row) if verify else self.objects/row['file']
            table=ReceiptTable(path,row['count'],row['scientific'])
            if verify:table._content_hash=(receipt_storage_signature(table),row['sha256'])
        else:raise ValueError('UnsupportedNativeReceiptEncoding')
        if 'authenticated' in row:
            from .authenticated_receipts import verify as verify_root
            table.authenticated=row['authenticated'];table.auth_objects=self.objects
            if verify:verify_root(table,self.objects)
        if 'proof_segments' in row:
            for segment in row['proof_segments']:
                body=dict(segment)
                if body.pop('identity')!=identity(body):raise ValueError('NativeProofSegmentIdentity')
                if verify:
                    for descriptor in [segment[k] for k in ('faces','pairs','codes','payloads')]+list(segment.get('dependency',{}).values()):self._object(descriptor)
            table.proof_segments=row['proof_segments']
            table.proof_objects=self.objects
        if verify:
            # Every referenced object's complete byte checksum was checked
            # above, including all base/update objects. Identical authenticated
            # bytes and metadata have the identical canonical row population.
            verification_key=identity(row)
            if not canonical and not hasattr(table,'authenticated'):
                # The enclosing authenticated root binds these verified bytes.
                # Do not render a historical delta merely to reconstruct its
                # redundant legacy digest. No legacy verification token is set.
                RECEIPT_AUDIT_STATS['authenticated_components']=RECEIPT_AUDIT_STATS.get('authenticated_components',0)+1
                if 'qualified_context' in row:table.qualified_context=row['qualified_context']
                return table
            if verification_key in _VERIFIED_RECEIPTS:
                RECEIPT_AUDIT_STATS['reused']+=1
                _VERIFIED_RECEIPTS.move_to_end(verification_key)
            else:
                RECEIPT_AUDIT_STATS['full']+=1
                if hasattr(table,'authenticated'):
                    count,digest=len(table),row['scientific']
                elif isinstance(table,ReceiptDelta):
                    count,digest=table.measure_identity()
                else:
                    with closing(table._connect()) as db:
                        count=db.execute('SELECT COUNT(*) FROM receipts').fetchone()[0]
                    digest=scientific_digest(table)
                if count!=len(table):raise ValueError('NativeReceiptCountMismatch')
                if digest!=row['scientific']:raise ValueError('NativeReceiptIdentityMismatch')
                _VERIFIED_RECEIPTS[verification_key]=True
                if len(_VERIFIED_RECEIPTS)>32:_VERIFIED_RECEIPTS.popitem(last=False)
            table._verified_storage=(verification_key,receipt_storage_signature(table))
        if 'qualified_context' in row:table.qualified_context=row['qualified_context']
        return table
    def _json(self,value):
        temporary=self.objects/'component.pending';h=hashlib.sha256()
        with temporary.open('wb') as f:
            for chunk in canonical_chunks(value):f.write(chunk);h.update(chunk)
            f.flush();os.fsync(f.fileno())
        scientific=h.hexdigest();target=self.objects/(scientific+'.json')
        if target.exists():
            if file_hash(target)!=scientific:raise ValueError('ExistingNativeComponentCorrupt')
            temporary.unlink()
        else:os.replace(temporary,target)
        return dict(encoding='json',file=target.name,scientific=scientific,sha256=scientific)
    def _component(self,value):
        # Existing table import/restore preserves a map's canonical identity.
        # Cold atlas binding uses this format; ordinary JSON remains unchanged.
        if isinstance(value,ReceiptTable):return self._table(value)
        if hasattr(value,'native_descriptor'):
            from .native_prism_entities import PrismEntities
            if not isinstance(value,PrismEntities):raise ValueError('UnsupportedNativeView')
            if value.backing.objects.resolve()!=self.objects.resolve():raise ValueError('NativeViewRequiresObjectImport')
            row=self._json(value.native_descriptor());row['encoding']='prism_entity_view'
            return row
        return self._json(value)
    def publish(self,state,*,validate=None):
        from .canonical_blocks import AUTHORITY
        root=getattr(state['certificate'],'authenticated',{})
        if root.get('authority')==AUTHORITY:
            from .block_retention import publication
            with publication(self.path,root):return self._publish(state,validate=validate)
        return self._publish(state,validate=validate)
    def _publish(self,state,*,validate=None):
        from .mixed_front import VERSION, scientific_hash
        if state['version']!=VERSION:raise ValueError('MixedFrontSchemaMismatch')
        components={}
        for key,value in state.items():components[key]=self._table(value) if key in ('quality','certificate') else self._component(value)
        # Hash from native evidence tables, never a second full JSON envelope.
        transient=dict(state)
        for key in ('quality','certificate'):
            transient[key]=self._receipt(components[key],verify=False)
        scientific=scientific_hash(transient)
        envelope=dict(format=FORMAT,scientific=scientific,components=components)
        name='NATIVE_'+scientific_digest(envelope)+'.json';atomic_json(self.path/name,envelope)
        pointer=dict(format=FORMAT,checkpoint=name,sha256=file_hash(self.path/name),scientific=scientific)
        if validate is not None:
            if validate(self._restore(pointer)) is not True:raise ValueError('NativePrepublicationValidationFailed')
        atomic_json(self.path/'CURRENT.json',pointer)
        return scientific
    def restart(self):
        if (self.path/'BLOCK_TRANSACTION.json').exists():
            from .block_retention import recover
            recover(self.path)
        pointer=json.loads((self.path/'CURRENT.json').read_text())
        return self._restore(pointer)
    def _restore(self,pointer):
        from .mixed_front import scientific_hash
        if pointer.get('format')!=FORMAT:raise ValueError('NativeCheckpointImportRequired')
        name=pointer['checkpoint']
        if Path(name).name!=name:raise ValueError('NativeCheckpointPath')
        p=self.path/name
        if file_hash(p)!=pointer['sha256']:raise ValueError('NativeCheckpointCorrupt')
        envelope=json.loads(p.read_text());state={};backings={}
        if envelope.get('format')!=FORMAT:raise ValueError('NativeCheckpointFormat')
        self.authenticated_objects=any('authenticated' in row for row in envelope['components'].values())
        for key,row in envelope['components'].items():
            if row['encoding'] in ('receipt_table','receipt_delta'):
                state[key]=self._receipt(row)
                continue
            p=self._object(row)
            if row['encoding']=='json':state[key]=json.loads(p.read_text())
            elif row['encoding']=='prism_entity_view':
                from .native_prism_entities import PrismBacking,PrismEntities,PrismEdits
                descriptor=json.loads(p.read_text());digest=scientific_digest(descriptor['backing'])
                if digest not in backings:
                    backings[digest]=PrismBacking(self.objects,descriptor['backing'])
                    for name in descriptor['backing']['arrays']:backings[digest].array(name)
                state[key]=(PrismEdits(backings[digest],descriptor['kind'],descriptor['edits'],descriptor['removed']) if 'edits' in descriptor else PrismEntities(backings[digest],descriptor['kind']))
            else:raise ValueError('UnsupportedNativeEncoding')
        if scientific_hash(state)!=envelope['scientific'] or envelope['scientific']!=pointer['scientific']:raise ValueError('NativeScientificIdentityMismatch')
        return state

    def import_mixed_checkpoint(self, source):
        """One-time verified import of the historical mixed JSON envelope.

        This does not infer missing prism-only state or manufacture receipts.
        The source remains read-only; the destination publishes native objects.
        """
        from .mixed_front import VERSION, scientific_hash
        source=Path(source)
        if (self.path/'CURRENT.json').exists():raise ValueError('NativeImportDestinationNotEmpty')
        pointer=json.loads((source/'CURRENT.json').read_text())
        name=pointer['checkpoint']
        if Path(name).name!=name or pointer.get('version')!=VERSION:raise ValueError('LegacyMixedPointerMismatch')
        path=source/name
        if file_hash(path)!=pointer['sha256']:raise ValueError('LegacyMixedCheckpointCorrupt')
        with path.open(encoding='utf-8') as stream:envelope=json.load(stream)
        state=envelope['state']
        expected=scientific_hash(state)
        if expected!=envelope['scientific'] or expected!=pointer['scientific']:raise ValueError('LegacyMixedScientificMismatch')
        self.publish(state)
        del state,envelope
        return self.restart()
