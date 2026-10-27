"""Compact source-keyed entity views in the native mixed checkpoint schema.

Regular prism blocks are storage compression, not a second committed state.
Arrays are native immutable objects. Entity access returns ordinary mixed rows.
"""
from collections.abc import Mapping,MutableMapping
from pathlib import Path
from copy import deepcopy
import numpy as np
import os
from ..common.authority import identity,array_hash,file_hash


class PrismBacking:
    def __init__(self,objects,descriptor):
        self.objects=Path(objects);self.descriptor=descriptor;self.arrays={}
    def array(self,name):
        if name not in self.arrays:
            row=self.descriptor['arrays'][name];path=self.objects/row['file']
            if Path(row['file']).name!=row['file'] or file_hash(path)!=row['sha256']:raise ValueError('NativePrismArrayCorrupt')
            self.arrays[name]=np.load(path,mmap_mode='r',allow_pickle=False)
        return self.arrays[name]
    @property
    def layer(self):return self.descriptor['layer']
    @property
    def triangles(self):return self.array('triangles')
    def coordinates(self,k):return self.array(f'coordinates_{k}')


class PrismEntities(Mapping):
    def __init__(self,backing,kind):self.backing=backing;self.kind=kind
    def __deepcopy__(self,memo):return self # immutable rows; replacement uses new blocks
    def transaction_copy(self):return PrismEdits(self.backing,self.kind)
    def __eq__(self,other):
        if isinstance(other,PrismEntities):return self.native_scientific()==other.native_scientific()
        return NotImplemented
    def preserves(self,other):
        if not isinstance(other,PrismEntities) or self.kind!=other.kind:return False
        a=self.backing.descriptor;b=other.backing.descriptor
        return a['layer']>=b['layer'] and all(k in a['arrays'] and a['arrays'][k]['scientific']==v['scientific']
            and self.backing.array(k).shape==other.backing.array(k).shape
            and self.backing.array(k).dtype==other.backing.array(k).dtype for k,v in b['arrays'].items())
    def native_scientific(self):
        d=self.backing.descriptor
        return dict(version='V4PrismEntityBlocks/1',kind=self.kind,layer=d['layer'],
            arrays={k:v['scientific'] for k,v in d['arrays'].items()})
    def native_descriptor(self):return dict(kind=self.kind,backing=self.backing.descriptor)
    def __len__(self):
        n=len(self.backing.coordinates(0));c=len(self.backing.triangles);k=self.backing.layer
        return {'vertices':n*(k+1),'cells':c*k,'columns':c,'prism_front':c,'faces':c*(k+1)+len(self.backing.array('edges'))*k}[self.kind]
    def __iter__(self):
        b=self.backing;n=len(b.coordinates(0));c=len(b.triangles)
        if self.kind=='vertices':
            for k in range(b.layer+1):
                for i in range(n):yield f'layer:{k}:source:{i}'
        elif self.kind=='cells':
            for k in range(1,b.layer+1):
                for i in range(c):yield f'prism:{k}:{i}'
        elif self.kind=='faces':
            for k in range(b.layer+1):
                for i in range(c):yield f'cap:{k}:{i}'
            for k in range(1,b.layer+1):
                for i in range(len(b.array('edges'))):yield f'side:{k}:{i}'
        else:
            for i in range(c):yield str(i)
    def __getitem__(self,key):
        b=self.backing
        def v(k,i):return f'layer:{k}:source:{int(i)}'
        if self.kind=='faces':
            tag,k,i=key.split(':');k=int(k);i=int(i);owners={}
            if tag=='cap' and 0<=k<=b.layer and 0<=i<len(b.triangles):
                face=[v(k,j) for j in b.triangles[i]]
                if k:owners[f'prism:{k}:{i}']=face
                if k<b.layer:owners[f'prism:{k+1}:{i}']=list(reversed(face))
            elif tag=='side' and 1<=k<=b.layer and 0<=i<len(b.array('edges')):
                a,z=map(int,b.array('edges')[i]);face=[v(k-1,a),v(k-1,z),v(k,z),v(k,a)]
                for c in np.flatnonzero(np.any(b.triangles==a,axis=1)&np.any(b.triangles==z,axis=1)):
                    tri=list(map(int,b.triangles[c]));forward=tri[(tri.index(a)+1)%3]==z
                    owners[f'prism:{k}:{int(c)}']=face if forward else list(reversed(face))
            else:raise KeyError(key)
            return dict(vertices=face,owners=owners)
        if self.kind=='vertices':
            try:_,k,_,i=key.split(':');k=int(k);i=int(i)
            except (ValueError,AttributeError):raise KeyError(key)
            if k<0 or k>b.layer or i<0 or i>=len(b.coordinates(0)):raise KeyError(key)
            return b.coordinates(k)[i].tolist()
        if self.kind=='cells':
            try:tag,k,i=key.split(':');k=int(k);i=int(i)
            except (ValueError,AttributeError):raise KeyError(key)
            if tag!='prism' or not 1<=k<=b.layer or not 0<=i<len(b.triangles):raise KeyError(key)
            t=b.triangles[i];low=[v(k-1,j) for j in t];high=[v(k,j) for j in t]
            faces=[list(reversed(low)),high]+[[low[a],low[z],high[z],high[a]] for a,z in ((0,1),(1,2),(2,0))]
            return dict(kind='prism',source=str(i),layer=k,vertices=low+high,faces=faces)
        try:i=int(key)
        except ValueError:raise KeyError(key)
        if str(i)!=key or not 0<=i<len(b.triangles):raise KeyError(key)
        face=f'cap:{b.layer}:{i}'
        if self.kind=='prism_front':return face
        return dict(status='ActivePrismColumn',last_prism_layer=b.layer,
                    exposed_prism_face=face,outer_ownership=[face])


def write_array(objects,value):
    objects=Path(objects);objects.mkdir(parents=True,exist_ok=True)
    value=np.asarray(value);scientific=array_hash(value)
    descriptor=dict(scientific=scientific,dtype=value.dtype.str,shape=list(value.shape))
    path=objects/(identity(descriptor)+'.npy')
    if not path.exists():
        temporary=path.with_suffix('.pending')
        with temporary.open('wb') as stream:
            np.save(stream,value,allow_pickle=False);stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,path)
    else:
        existing=np.load(path,mmap_mode='r',allow_pickle=False)
        if existing.dtype!=value.dtype or existing.shape!=value.shape or array_hash(existing)!=scientific:raise ValueError('ExistingNativeArrayCorrupt')
    return dict(descriptor,file=path.name,sha256=file_hash(path))


class PrismEdits(PrismEntities,MutableMapping):
    """Bounded committed-cell rollback without expanding the regular blocks."""
    def __init__(self,backing,kind,edits=None,removed=()):
        super().__init__(backing,kind);self.edits={} if edits is None else edits;self.removed=set(removed)
    def __deepcopy__(self,memo):return PrismEdits(self.backing,self.kind,deepcopy(self.edits,memo),self.removed)
    def transaction_copy(self):return deepcopy(self)
    def __getitem__(self,key):
        if key in self.removed:raise KeyError(key)
        if key not in self.edits:self.edits[key]=super().__getitem__(key)
        return self.edits[key]
    def __setitem__(self,key,value):self.edits[key]=value;self.removed.discard(key)
    def __delitem__(self,key):self[key];self.edits.pop(key,None);self.removed.add(key)
    def __iter__(self):
        for key in super().__iter__():
            if key not in self.removed:yield key
        base=PrismEntities(self.backing,self.kind)
        for key in sorted(self.edits):
            if key not in base:yield key
    def __len__(self):
        base=PrismEntities(self.backing,self.kind)
        return len(base)-sum(k in base for k in self.removed)+sum(k not in base for k in self.edits)
    def native_descriptor(self):
        return dict(super().native_descriptor(),edits=self.edits,removed=sorted(self.removed))
    def native_scientific(self):
        base=PrismEntities(self.backing,self.kind)
        edits={k:v for k,v in self.edits.items() if k not in base or v!=base[k]}
        if not edits and not self.removed:return base.native_scientific()
        return dict(base=base.native_scientific(),edits=edits,removed=sorted(self.removed))


def prism_views(objects,coordinates,triangles,edges):
    arrays={'triangles':write_array(objects,triangles),'edges':write_array(objects,edges)}
    arrays.update({f'coordinates_{k}':write_array(objects,x) for k,x in enumerate(coordinates)})
    backing=PrismBacking(objects,dict(layer=len(coordinates)-1,arrays=arrays))
    return {kind:PrismEntities(backing,kind) for kind in ('vertices','cells','columns','prism_front','faces')}
