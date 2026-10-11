"""Unselected native whole-query leaf/traversal inside the existing Field.

No callback bridge, backend dispatch or sizing-policy change. Unsupported native
queries replay the whole original owner. Geometry capture has no mutable aliases.
"""
import ctypes
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import platform
import threading
from types import MappingProxyType
from typing import NamedTuple
import weakref

import numpy as np
from background_sizing import Field as OriginalField,arithmetic_runtime
from capsule_sizing_control import MeasuredCallback as OriginalMeasuredCallback

from owned_runtime import V4
HERE=V4/'runtime/source/background/direct'
DEFAULT_ERR=dict(divide='warn',over='warn',under='ignore',invalid='warn')
FLAGS=['-std=c++17','-shared','-fPIC','-O3','-fno-fast-math','-fno-associative-math','-ffp-contract=off']
_issued=weakref.WeakKeyDictionary()
_BOUND=('triangles','seeds','normals','normal2','edges','edge2','nodes','bounds','g','h_max','constraints','dll')


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def query_record():
    from background_query_build import record
    return record('query')


def frozen_array(value):
    return np.frombuffer(value.tobytes(order='C'),dtype=value.dtype).reshape(value.shape)


class FrozenNode(NamedTuple):
    lo:object
    hi:object
    seed:float
    left:int
    right:int
    index_bytes:bytes

    @property
    def ids(self):return np.frombuffer(self.index_bytes,dtype=np.int64)


@dataclass(frozen=True)
class QueryOwner:
    library:object
    pointer:object
    binding:tuple
    input_identity:str
    build:object
    lock:object
    counts:object
    finalizer:object
    array_metadata:tuple


def context_arrays(field):
    boxes=np.asarray([[*n.lo,*n.hi,n.seed] for n in field.nodes],dtype=np.float64)
    children=np.asarray([(n.left,n.right) for n in field.nodes],dtype=np.int64)
    offsets=[0];indices=[]
    for node in field.nodes:indices.extend(map(int,node.ids));offsets.append(len(indices))
    return tuple(frozen_array(a) for a in (boxes,children,np.asarray(offsets,dtype=np.uint64),np.asarray(indices,dtype=np.uint64)))


class NativeQueryField(OriginalField):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        try:self._capture()
        except BaseException:
            OriginalField.close(self)
            raise

    def _capture(self):
        self._closed=False
        self.constraints=tuple(self.constraints)
        for name in ('triangles','seeds','normals','normal2'):
            setattr(self,name,frozen_array(getattr(self,name)))
        self.edges=tuple(frozen_array(a) for a in self.edges)
        self.edge2=tuple(frozen_array(a) for a in self.edge2)
        self.nodes=tuple(FrozenNode(tuple(map(float,n.lo)),tuple(map(float,n.hi)),n.seed,n.left,n.right,np.asarray(n.ids,dtype=np.int64).tobytes()) for n in self.nodes)
        self.bounds=MappingProxyType({id(n):(tuple(map(float,n.lo)),tuple(map(float,n.hi)),max(1.,*map(abs,n.lo),*map(abs,n.hi))) for n in self.nodes})
        arrays=(self.triangles,self.normals,self.normal2,*self.edges,*self.edge2,self.seeds)
        tree=context_arrays(self)
        identity=hashlib.sha256()
        for array in (*arrays,*tree):
            identity.update(str((array.dtype.str,array.shape)).encode());identity.update(array.tobytes())
        identity.update(float(self.g).hex().encode());identity.update(float(self.h_max).hex().encode())
        library=pointer=build=None
        supported=(platform.system()=='Linux' and np.__version__=='1.26.4' and self.dll is not None
            and not self.constraints and np.geterr()==DEFAULT_ERR
            and all(type(a) is np.ndarray and a.flags.c_contiguous and a.flags.aligned for a in (*arrays,*tree))
            and all(len(n.ids)<=16 for n in self.nodes))
        if supported:
            build,path=query_record();library=ctypes.CDLL(str(path));p=ctypes.c_void_p;s=ctypes.c_size_t
            library.query_create.argtypes=[s,s,*([p]*14),ctypes.c_double,ctypes.c_double,p]
            library.query_create.restype=p
            library.query_value.argtypes=[p,p,p,p];library.query_value.restype=ctypes.c_uint8
            library.query_destroy.argtypes=[p];library.query_destroy.restype=None
            status=ctypes.c_int(1)
            pointer=library.query_create(len(self.seeds),len(self.nodes),
                *[a.ctypes.data for a in (*arrays,*tree)],self.g,self.h_max,ctypes.byref(status))
            if status.value==2:raise MemoryError('NativeQueryContextAllocation')
            if status.value!=0:pointer=None
            elif not pointer:raise ValueError('MissingNativeQueryContext')
        finalizer=None
        try:
            if pointer:finalizer=weakref.finalize(self,library.query_destroy,pointer)
            _issued[self]=QueryOwner(library,pointer,tuple(getattr(self,k) for k in _BOUND),
                identity.hexdigest(),build,threading.RLock(),dict(native_calls=0,fallback_calls=0),finalizer,
                tuple((a,a.dtype.str,a.shape,a.strides,a.ctypes.data) for a in arrays))
        except BaseException:
            if finalizer:finalizer()
            elif pointer:library.query_destroy(pointer)
            raise

    def _owner(self):
        owner=_issued.get(self)
        if owner is None:raise ValueError('ClosedNativeQueryField')
        if any(getattr(self,k) is not value for k,value in zip(_BOUND,owner.binding)):
            raise ValueError('ChangedNativeQueryBinding')
        if any(a.dtype.str!=dtype or a.shape!=shape or a.strides!=strides or a.ctypes.data!=pointer or a.flags.writeable
               for a,dtype,shape,strides,pointer in owner.array_metadata):
            raise ValueError('ChangedNativeQueryArrayMetadata')
        return owner

    def evaluate_with_work(self,position):
        owner=self._owner()
        with owner.lock:
            if _issued.get(self) is not owner:raise ValueError('ClosedNativeQueryField')
            self._owner()
            p=self.point(position)
            if owner.pointer and np.geterr()==DEFAULT_ERR:
                captured=frozen_array(np.ascontiguousarray(p));value=ctypes.c_double();work=(ctypes.c_uint64*2)()
                status=owner.library.query_value(owner.pointer,captured.ctypes.data,ctypes.byref(value),work)
                if status==0:
                    self._owner()
                    owner.counts['native_calls']+=1
                    return value.value,dict(nodes=int(work[0]),triangles=int(work[1]))
            owner.counts['fallback_calls']+=1
            result=super().evaluate_with_work(p)
            self._owner()
            return result

    def native_report(self):
        owner=self._owner()
        with owner.lock:
            self._owner()
            return dict(owner.counts,native_enabled=bool(owner.pointer),input_identity=owner.input_identity,
                build=owner.build,scope='Unselected whole-query traversal; work counts may differ. Existing Python callback bridge and source sizing policy.')

    def close(self):
        owner=_issued.get(self)
        if owner is not None:
            with owner.lock:
                if _issued.get(self) is owner:
                    _issued.pop(self)
                    if owner.finalizer:owner.finalizer()
        super().close()


class BoundMeasuredCallback(OriginalMeasuredCallback):
    """Use inside the existing LockedCallback's first-error handler.

    Bind cached hits as well as fresh native queries, including field expiry.
    Context close waits for this call. The existing callback retains its cache,
    counters, finite fallback and first-error rejection.
    """
    def __init__(self,field):
        super().__init__(field)
        self._captured_field=field
        self._captured_owner=field._owner()

    def __call__(self,*args):
        if self.field is not self._captured_field or self.field._owner() is not self._captured_owner:
            raise ValueError('ChangedNativeCallbackField')
        owner=self.field._owner()
        with owner.lock:
            self.field._owner()
            result=super().__call__(*args)
            self.field._owner()
            return result
