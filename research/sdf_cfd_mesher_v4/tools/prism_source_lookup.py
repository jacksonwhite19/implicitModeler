"""Exact source-face point lookup derived from authenticated packed incidence."""
from collections.abc import Mapping
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch
import numpy as np
import immutable_state_access as access
from sdf_cfd_mesher_v4.advancing_front.mixed_front import cycle,face_key
from sdf_cfd_mesher_v4.common.authority import identity
from path_preserving_deduplication import durable_json,file_hash


def source_face(rows,n):
    converted={str(owner):list(cycle(tuple(f'layer:{v//n}:source:{v%n}' for v in face))) for owner,face in rows}
    first=next(iter(converted.values()))
    return dict(vertices=list(min(cycle(first),cycle(tuple(reversed(first))))),owners=converted)


def write_lookup(directory,rows,n,index_root,content):
    directory=Path(directory);directory.mkdir(exist_ok=False)
    items=[]
    for i,(_,values) in enumerate(rows):
        row=source_face(values,n);items.append((face_key(row['vertices']),i))
    items.sort()
    if len({k for k,_ in items})!=len(items):raise ValueError('DuplicateScientificFaceIdentity')
    files={}
    for name,array in (('hashes',np.array([k for k,_ in items],dtype='S64')),('rows',np.array([i for _,i in items],dtype='<i8'))):
        p=directory/(name+'.npy')
        with p.open('wb') as f:
            np.save(f,array,allow_pickle=False);f.flush()
            import os
            os.fsync(f.fileno())
        files[name]=dict(path=str(p.resolve()),**access.attest(p,file_hash(p)))
    body=dict(schema='V4SourceFaceLookup/1',index_root=index_root,content=content,vertex_stride=n,rows=len(items),files=files,epoch=access.epoch())
    durable_json(directory/'MANIFEST.json',body)
    return dict(directory=str(directory.resolve()),root=identity(body))


class SourceLookup:
    def __init__(self,index,reference,held):
        p=Path(reference['directory'])/'MANIFEST.json';held.enter_context(access.guard(p));body=access.read(p)
        if identity(body)!=reference['root'] or body['index_root']!=index.root or body['content']!=index.body['parent_content']:
            raise ValueError('WrongSourceLookupBinding')
        if body['epoch']!=access.epoch():raise ValueError('IncrementalEpochChanged')
        self.index=index;self.body=body;self.queries=0;self.hits=0
        for name,r in body['files'].items():
            held.enter_context(access.guard(r['path']))
            if access.token(r['path'])!=r['token']:raise ValueError('CorruptSourceLookup')
            setattr(self,name,np.load(r['path'],mmap_mode='r',allow_pickle=False))
        if len(self.hashes)!=len(index) or self.rows.shape!=(len(index),):raise ValueError('WrongSourceLookupCount')
    def row(self,key):
        self.index.live();self.queries+=1
        if not isinstance(key,str) or len(key)!=64:raise KeyError(key)
        try:needle=key.encode('ascii')
        except UnicodeEncodeError:raise KeyError(key)
        i=int(np.searchsorted(self.hashes,needle))
        if i==len(self.hashes) or self.hashes[i]!=needle:raise KeyError(key)
        self.hits+=1
        j=int(self.rows[i]);owners=self.index.owners[j];winding=self.index.winding[j]
        values=[(int(owner),tuple(int(v) for v in winding[k] if v>=0)) for k,owner in enumerate(owners) if owner>=0]
        return source_face(values,self.body['vertex_stride'])


@contextmanager
def point_views(base,index,adapter):
    from composable_state import CommittedState
    original=CommittedState.__getitem__
    source=original(base,'faces');vertices=original(base,'vertices')
    n=len(adapter.front.coordinates)
    class Faces(Mapping):
        def __getitem__(self,key):base.check_live();return index.scientific_face(key)
        def __contains__(self,key):
            try:self[key];return True
            except KeyError:return False
        def __iter__(self):return iter(source)
        def __len__(self):return len(index)
    class Vertices(Mapping):
        def __getitem__(self,key):
            base.check_live()
            try:
                parts=key.split(':');layer=int(parts[1]);i=int(parts[3])
                if key!=f'layer:{layer}:source:{i}' or not 0<=layer<len(adapter.committed_prefix) or not 0<=i<n:raise KeyError(key)
                return adapter.committed_prefix[layer][i].tolist()
            except (ValueError,IndexError,AttributeError):raise KeyError(key)
        def __contains__(self,key):
            try:self[key];return True
            except KeyError:return False
        def __iter__(self):return iter(vertices)
        def __len__(self):return len(vertices)
    faces=Faces();points=Vertices()
    def get(state,key):
        if state is base and key=='faces':return faces
        if state is base and key=='vertices':return points
        return original(state,key)
    if hasattr(base,'numeric_point_views'):raise ValueError('NestedNumericPointViews')
    base.numeric_point_views={'faces':faces,'vertices':points}
    try:
        with patch.object(CommittedState,'__getitem__',get):yield
    finally:del base.numeric_point_views
