"""Pinned, host-local immutable access views. No mesh or receipt publisher.

Normal access trusts a previously deep-validated binding only within its host
boot epoch and while NTFS file identity/change tokens match. Held handles deny
writes and replacement. Requested content is SHA-256 checked before decoding.
Unrequested cold content is checked by explicit deep audit, not row traversal.
"""
from collections import Counter
from collections.abc import Mapping
from contextlib import ExitStack, contextmanager
from copy import deepcopy
import ctypes
import json
import os
from pathlib import Path
import socket
import sqlite3
import psutil
import numpy as np
from path_preserving_deduplication import api, W, file_hash, durable_json
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import file_change_time, NativeCheckpointStore, scientific_digest

SCHEMA='V4ImmutableLegacyAccess/1'

def read(path):return json.loads(Path(path).read_text())
def epoch():return dict(host=socket.gethostname(),boot=int(psutil.boot_time()))
def token(path):
    p=Path(path).resolve();s=p.stat()
    return dict(path=str(p),device=s.st_dev,file_id=s.st_ino,size=s.st_size,mtime=s.st_mtime_ns,change=file_change_time(p,s))

@contextmanager
def guard(path):
    if os.name!='nt':raise ValueError('UnqualifiedImmutableFilesystem')
    k=api();h=k.CreateFileW(str(Path(path).resolve()),0x80000000,1,None,3,0,None)
    if h==W.HANDLE(-1).value:raise OSError(ctypes.get_last_error(),'ImmutableObjectUnavailableOrWriterActive',str(path))
    try:yield h
    finally:k.CloseHandle(h)

def attest(path,sha256):
    with guard(path):
        before=token(path)
        if file_hash(path)!=sha256:raise ValueError('ComponentContentMismatch')
        if token(path)!=before:raise ValueError('MutationDuringAuthentication')
        return dict(sha256=sha256,token=before)

def publish_manifest(directory,body,expected_previous=None,interrupt=None):
    """Only access metadata is published. Legacy checkpoint is never written."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    k=api();lock=k.CreateFileW(str((directory/'ACCESS.lock').resolve()),0xC0000000,0,None,4,0,None)
    if lock==W.HANDLE(-1).value:raise ValueError('ConcurrentAccessMetadataWriter')
    try:
        current=directory/'CURRENT.json';old=read(current)['root'] if current.exists() else None
        if old!=expected_previous:raise ValueError('StaleAccessMetadataWriter')
        root=identity(body);path=directory/('ROOT_'+root+'.json')
        if interrupt:interrupt('before_root')
        durable_json(path,body)
        if interrupt:interrupt('after_root_before_pointer')
        durable_json(current,dict(root=root,file=path.name,sha256=file_hash(path)))
        return root
    finally:k.CloseHandle(lock)

class CopiedMap(Mapping):
    """Mutating a returned value cannot modify the authenticated view."""
    def __init__(self,data,owner):self._data=data;self._owner=owner
    def __len__(self):self._owner._live();return len(self._data)
    def __iter__(self):self._owner._live();return iter(self._data)
    def __getitem__(self,k):self._owner._live();return deepcopy(self._data[k])

class VertexMap(Mapping):
    def __init__(self,keys,values,owner=None):self._keys=tuple(keys);self._index={k:i for i,k in enumerate(keys)};self._values=values;self._owner=owner
    def _live(self):
        if self._owner is not None:self._owner._live()
    def __len__(self):self._live();return len(self._keys)
    def __iter__(self):self._live();return iter(self._keys)
    def __getitem__(self,k):self._live();return self._values[self._index[k]].tolist()

class FaceMap(Mapping):
    def __init__(self,owner,hot):self._owner=owner;self._hot=hot
    def __len__(self):self._owner._live();return self._owner.body['counts']['faces']
    def __iter__(self):self._owner._live();return iter(self._owner._component_json('faces'))
    def __getitem__(self,k):
        self._owner._live()
        if k in self._hot:return deepcopy(self._hot[k])
        return deepcopy(self._owner._component_json('faces')[k])

class ColdReceipts(Mapping):
    def __init__(self,owner,kind):self._owner=owner;self._kind=kind;self._row=owner.body['components'][kind]
    def __len__(self):self._owner._live();return self._row['count']
    def __iter__(self):raise ValueError('ExplicitDeepAuditRequiredForReceiptTraversal')
    def __getitem__(self,key):
        self._owner._live()
        def lookup(row):
            if row['encoding']=='receipt_delta':
                removed=self._owner._read_object_json(row['removed'])
                if key in removed:raise KeyError(key)
                try:return lookup(row['updates'])
                except KeyError:return lookup(row['base'])
            path=self._owner._object_path(row);self._owner._verify_read(path)
            db=sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)
            try:
                self._owner.stats['sqlite_point_queries']+=1
                found=db.execute('SELECT value FROM receipts WHERE key=?',(key,)).fetchone()
            finally:db.close()
            if found is None:raise KeyError(key)
            self._owner.stats['sqlite_rows_visited']+=1
            self._owner.stats['proof_receipts_deserialized' if self._kind=='certificate' else 'quality_rows_visited']+=1
            return json.loads(found[0])
        return lookup(self._row)

class ImmutableState(Mapping):
    """Complete legacy state exposed lazily through immutable read interfaces."""
    def __init__(self,directory,expected_root,overlay=None):
        self.directory=Path(directory);self._root=expected_root;self.stats=Counter();self._stack=ExitStack();self._cache={};self._verified=set();self.closed=False;self._implementation_access=[]
        try:
            self._stack.enter_context(guard(self.directory/'CURRENT.json'))
            pointer=read(self.directory/'CURRENT.json')
            if pointer['root']!=expected_root:raise ValueError('WrongPinnedAccessRoot')
            if pointer['file']!='ROOT_'+expected_root+'.json':raise ValueError('AccessRootPath')
            root_path=self.directory/pointer['file'];self._stack.enter_context(guard(root_path))
            if file_hash(root_path)!=pointer['sha256']:raise ValueError('AccessRootContainerMismatch')
            self.body=read(root_path)
            if identity(self.body)!=expected_root or self.body.get('schema')!=SCHEMA:raise ValueError('AccessRootIdentityMismatch')
            if self.body.get('epoch')!=epoch():raise ValueError('DeepReauthenticationRequiredForEpoch')
            if self.body.get('complete') is not True:raise ValueError('IncompleteAccessRoot')
            content=self.body['content']
            if identity(content)!=self.body['content_root']:raise ValueError('ContentRootMismatch')
            required={'version','wall','first_layer','vertices','cells','faces','columns','outer','prism_front','versions','authority','atlas','forecast','quality','certificate','events'}
            if set(self.body['components'])!=required or self.body['components']!=content['legacy_components']:raise ValueError('IncompleteLegacyMapping')
            self._files=self.body['files'];self.store=Path(self.body['store'])
            from rolling_layer_retention import descriptors
            needed={str((self.store/'objects'/r['file']).resolve()):r['sha256'] for r in descriptors(self.body['components'])}
            for role,d in self.body['hot'].items():needed[str(Path(d['path']).resolve())]=d['sha256']
            if set(self.body['hot'])!={'vertex_keys','vertex_values','front_faces'}:raise ValueError('IncompleteHotMapping')
            for p,h in needed.items():
                if p not in self._files or self._files[p]['sha256']!=h:raise ValueError('UnboundComponent')
            if {k:v['sha256'] for k,v in self.body['hot'].items()}!=content['hot']:raise ValueError('HotMappingMismatch')
            for p,h in content['implementation'].items():
                from historical_implementation_access import check_pinned
                proof=check_pinned(p,h,self.body['content_root'],'AccessImplementationChanged')
                if proof is not None and proof not in self._implementation_access:self._implementation_access.append(proof)
            for path,record in self._files.items():
                self._stack.enter_context(guard(path));self.stats['component_token_checks']+=1
                if token(path)!=record['token']:raise ValueError('ImmutableComponentMutation:'+path)
            self.stats['root_authentications']+=1
            legacy=read(self.store/'CURRENT.json')
            if legacy!=self.body['legacy_pointer'] or legacy['scientific']!=content['legacy_scientific']:raise ValueError('StaleLegacyBase')
            if str((self.store/'CURRENT.json').resolve()) not in self._files:raise ValueError('UnboundLegacyPointer')
            envelope=self.store/legacy['checkpoint']
            if str(envelope.resolve()) not in self._files or self._files[str(envelope.resolve())]['sha256']!=legacy['sha256']:raise ValueError('UnboundLegacyCheckpoint')
            if read(envelope)['components']!=self.body['components']:raise ValueError('LegacyMappingMismatch')
            if overlay is not None:
                if overlay.get('base')!=self.body['content_root']:raise ValueError('StaleOverlay')
                if overlay.get('edits') or overlay.get('removed') or set(overlay)-{'base','edits','removed'}:raise ValueError('EditedOverlayNotQualified')
        except BaseException:self.close();raise
    def close(self):
        if not self.closed:
            self._cache.clear();self._stack.close();self.closed=True
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def _live(self):
        if self.closed:raise ValueError('ClosedImmutableAccess')
        for proof in self._implementation_access:proof.require_open()
    def check(self):
        self._live()
        for proof in self._implementation_access:proof.check()
        if identity(self.body)!=self._root:raise ValueError('InMemoryRootMutation')
        for p,r in self._files.items():
            if token(p)!=r['token']:raise ValueError('ImmutableComponentMutation')
    def verified_legacy_identity(self):self.check();return self.body['content']['legacy_scientific']
    def _verify_read(self,path):
        self._live()
        key=str(Path(path).resolve())
        if key not in self._files:raise ValueError('UnboundRead')
        if key not in self._verified:
            if file_hash(key)!=self._files[key]['sha256']:raise ValueError('ReadContentCorrupt')
            self.stats['component_bytes_hashed_on_read']+=Path(key).stat().st_size;self.stats['component_content_checks']+=1;self._verified.add(key)
    def _object_path(self,row):return (self.store/'objects'/row['file']).resolve()
    def _read_object_json(self,row):
        p=self._object_path(row);self._verify_read(p);return read(p)
    def _component_json(self,key):
        cache='json:'+key
        if cache not in self._cache:self._cache[cache]=self._read_object_json(self.body['components'][key])
        return self._cache[cache]
    def __iter__(self):return iter(self.body['components'])
    def __len__(self):return len(self.body['components'])
    def __getitem__(self,key):
        self._live()
        if key not in self.body['components']:raise KeyError(key)
        cache='view:'+key
        if cache not in self._cache:
            if key=='vertices':
                paths=[Path(self.body['hot'][x]['path']) for x in ('vertex_keys','vertex_values')]
                for p in paths:self._verify_read(p)
                values=np.load(paths[1],mmap_mode='r',allow_pickle=False);self.stats['bytes_mapped']+=values.nbytes
                self._cache[cache]=VertexMap(read(paths[0]),values,self)
            elif key=='faces':
                p=Path(self.body['hot']['front_faces']['path']);self._verify_read(p);self._cache[cache]=FaceMap(self,read(p))
            elif key in ('quality','certificate'):self._cache[cache]=ColdReceipts(self,key)
            else:
                value=self._component_json(key);self._cache[cache]=CopiedMap(value,self) if isinstance(value,dict) else value
        value=self._cache[cache]
        return deepcopy(value) if isinstance(value,list) else value

    def deep_verify(self):
        self.check()
        for p,r in self._files.items():
            if file_hash(p)!=r['sha256']:raise ValueError('DeepComponentCorrupt:'+p)
        state=NativeCheckpointStore(self.store).restart()
        from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
        if scientific_hash(state)!=self.verified_legacy_identity():raise ValueError('DeepLegacyIdentityMismatch')
        if scientific_digest(self['vertices'])!=scientific_digest(state['vertices']):raise ValueError('PackedGeometryMismatch')
        hot=read(self.body['hot']['front_faces']['path'])
        if hot!={k:state['faces'][k] for k in state['prism_front'].values()}:raise ValueError('HotFaceMappingMismatch')
        return dict(legacy_scientific=scientific_hash(state),all_legacy_components_exact=True,
            proof_count=len(state['certificate']),quality_count=len(state['quality']),packed_geometry_exact=True,
            front_faces_exact=True,cold_components='Exact same authenticated immutable files and descriptors; no re-encoding',
            component_scientific={k:d.get('scientific') for k,d in self.body['components'].items()})
