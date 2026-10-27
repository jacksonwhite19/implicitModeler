"""Unselected direct C callback manager with owned whole-query fallback.

Registration lifetime is separate from original-field lifetime. No mesh policy,
acceptance token, checkpoint or production dispatcher is created here.
"""
import ctypes
from functools import wraps
import hashlib
import json
import math
from pathlib import Path
import threading
from types import MappingProxyType

import numpy as np
import background_native_query as native
from optimized_core_sizing import ExactCachedSizingField
from generic_core_sizing import CoreSizingField
from background_query_build import DIRECT_FLAGS as FLAGS

from owned_runtime import V4
HERE=V4/'runtime/source/background/direct'
FALLBACK=ctypes.CFUNCTYPE(ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_double))
CALLBACK=ctypes.CFUNCTYPE(ctypes.c_double,ctypes.c_int,ctypes.c_int,*([ctypes.c_double]*4),ctypes.c_void_p)
QUARANTINE=[]
LIVE=[]  # Retain native state, library and fallback function until explicit close.

def lifecycle(method):
    @wraps(method)
    def locked(self,*args,**kwargs):
        with self.lock:return method(self,*args,**kwargs)
    return locked

def build_record():
    from background_query_build import record
    return record('direct')

class FrozenFallback:
    """Existing arithmetic methods over sealed bytes; no geometry constructor."""
    __slots__=('__state','__nodes','__bounds')
    point=staticmethod(CoreSizingField.point)
    values=ExactCachedSizingField.values
    lower_bound=ExactCachedSizingField.lower_bound
    evaluate_with_work=CoreSizingField.evaluate_with_work
    evaluate=CoreSizingField.evaluate
    brute_force=CoreSizingField.brute_force

    def __setattr__(self,name,value):raise AttributeError('SealedFallbackCapture')

    @staticmethod
    def view(spec):
        data,dtype,shape=spec
        # Each metadata owner is fresh. No private canonical ndarray is exposed.
        return np.frombuffer(data,dtype=np.dtype(dtype)).reshape(shape)

    def __init__(self,arrays,g,maximum):
        names=('triangles','normals','normal2','e0','e1','e2','s0','s1','s2','seeds','boxes','children','offsets','ids')
        specs={name:(a.tobytes(),a.dtype.str,a.shape) for name,a in zip(names,arrays)}
        specs.update(g=float(g),h_max=float(maximum),constraints=())
        object.__setattr__(self,'_FrozenFallback__state',MappingProxyType(specs))
        boxes,children,offsets,indices=arrays[10:]
        nodes=tuple(native.FrozenNode(tuple(map(float,b[:3])),tuple(map(float,b[3:6])),float(b[6]),
            int(c[0]),int(c[1]),np.asarray(indices[int(offsets[i]):int(offsets[i+1])],dtype=np.int64).tobytes())
            for i,(b,c) in enumerate(zip(boxes,children)))
        object.__setattr__(self,'_FrozenFallback__nodes',nodes)
        object.__setattr__(self,'_FrozenFallback__bounds',MappingProxyType({id(n):(n.lo,n.hi,max(1.,*map(abs,n.lo),*map(abs,n.hi))) for n in nodes}))

    def __getattr__(self,name):
        state=self.__state
        if name in ('edges','edge2'):
            return tuple(self.view(state[p+str(i)]) for i in range(3) for p in (('e' if name=='edges' else 's'),))
        if name in ('triangles','normals','normal2','seeds'):return self.view(state[name])
        if name in ('g','h_max','constraints'):return state[name]
        if name=='nodes':return self.__nodes
        if name=='bounds':return self.__bounds
        raise AttributeError(name)

    def identity(self):
        h=hashlib.sha256()
        for name in ('triangles','normals','normal2','e0','e1','e2','s0','s1','s2','seeds','boxes','children','offsets','ids'):
            data,dtype,shape=self.__state[name]
            h.update(str((dtype,shape)).encode());h.update(data)
        h.update(self.g.hex().encode());h.update(self.h_max.hex().encode())
        return h.hexdigest()

def declare(library):
    p=ctypes.c_void_p;s=ctypes.c_size_t;i=ctypes.c_int;u=ctypes.c_uint64;d=ctypes.c_double
    declarations={'direct_clone':([p,FALLBACK,ctypes.POINTER(i)],p),'direct_destroy':([p],None),
        'direct_expire':([p],None),'direct_info':([p,ctypes.POINTER(u),ctypes.POINTER(d)],i),
        'direct_export':([p,i,p,s],i),'direct_report':([p,ctypes.POINTER(u),ctypes.POINTER(i)],i),
        'direct_callback':([i,i,d,d,d,d,p],d),
        'direct_control_force':([p,i],None),
        'direct_control_threads':([p,p,s,p,ctypes.c_uint,i],i)}
    for name,(args,result) in declarations.items():
        function=getattr(library,name);function.argtypes=args;function.restype=result

class DirectCallback:
    def __init__(self,field):
        self.field=field;self.original_owner=field._owner();self.lock=threading.RLock()
        self.error_lock=threading.Lock()
        self.first_python_error=None;self.pointer=None;self.registered=False;self.quarantined=False
        self.registrations=0;self.removals=0;self.fallback_capture=None
        self.build,path=build_record();self.library=ctypes.CDLL(str(path));declare(self.library)
        capture=None
        def fallback(x,y,z,out):
            try:
                value=capture.evaluate((x,y,z))
                if not math.isfinite(value) or value<=0:raise ValueError('InvalidDirectFallback')
                out[0]=value;return 0
            except BaseException as error:
                with self.error_lock:
                    if self.first_python_error is None:self.first_python_error=error
                return 1
        self.fallback_function=FALLBACK(fallback)
        self.function=ctypes.cast(self.library.direct_callback,CALLBACK)
        try:
            with self.original_owner.lock:
                self.require_binding()
                status=ctypes.c_int(1)
                self.pointer=self.library.direct_clone(self.original_owner.pointer,self.fallback_function,ctypes.byref(status))
                if status.value or not self.pointer:raise ValueError('DirectCloneFailed',status.value)
                sizes=(ctypes.c_uint64*2)();policy=(ctypes.c_double*2)()
                if self.library.direct_info(self.pointer,sizes,policy):raise ValueError('DirectInfoFailed')
                n,nn=map(int,sizes)
                shapes=((n,3,3),(n,3),(n,),(n,3),(n,3),(n,3),(n,),(n,),(n,),(n,),(nn,7),(nn,2),(nn+1,),(n,))
                arrays=[]
                for index,shape in enumerate(shapes):
                    dtype=np.float64 if index<11 else (np.int64 if index==11 else np.uint64)
                    array=np.empty(shape,dtype=dtype)
                    if self.library.direct_export(self.pointer,index,array.ctypes.data,array.nbytes):raise ValueError('DirectExportFailed',index)
                    arrays.append(array)
                capture=FrozenFallback(arrays,*map(float,policy))
                self.fallback_capture=capture
                if capture.identity()!=self.original_owner.input_identity:raise ValueError('ChangedDirectCaptureIdentity')
                self.require_binding()
            LIVE.append(self)
        except BaseException:
            if self.pointer:self.library.direct_destroy(self.pointer);self.pointer=None
            raise

    def require_binding(self):
        if self.field._owner() is not self.original_owner:raise ValueError('ChangedDirectOriginalOwner')

    @lifecycle
    def require_passed(self):
        if self.quarantined:raise ValueError('QuarantinedDirectCallback')
        report=self.report()
        if self.first_python_error is not None:raise ValueError('DirectPythonFallbackFailure') from self.first_python_error
        if report['native_failure']:raise ValueError('DirectNativeCallbackFailure',report['native_failure'])

    @lifecycle
    def report(self):
        if not self.pointer:raise ValueError('ClosedDirectCallback')
        counts=(ctypes.c_uint64*5)();failure=ctypes.c_int()
        if self.library.direct_report(self.pointer,counts,ctypes.byref(failure)):raise ValueError('DirectReportFailed')
        return dict(zip(('evaluations','native_calls','fallback_calls','nodes_tested','triangles_tested'),map(int,counts)),
            native_failure=failure.value,python_failure=self.first_python_error is not None,
            registrations=self.registrations,removals=self.removals,quarantined=self.quarantined,
            input_identity=self.original_owner.input_identity,cache_capacity=0,exact_coordinate_cache_hits=0,
            scope='Direct C query; every nonzero query status replays fresh whole-query Python arithmetic. No cache. Work diagnostics differ; no callback timing claim.')

    @lifecycle
    def install(self,gmsh):
        self.require_binding();self.require_passed()
        if self.registered:return
        if not self.pointer or self.quarantined:raise ValueError('UnusableDirectCallback')
        setter=gmsh.lib.gmshModelMeshSetSizeCallback
        setter.argtypes=[CALLBACK,ctypes.c_void_p,ctypes.POINTER(ctypes.c_int)];setter.restype=None
        # Treat registration as live before the call: uncertain success quarantines.
        self.registered=True
        status=ctypes.c_int()
        try:
            setter(self.function,self.pointer,ctypes.byref(status))
            if status.value:raise ValueError('DirectCallbackInstallFailed',status.value)
        except BaseException:self.quarantine();raise
        self.registrations+=1

    @lifecycle
    def remove(self,gmsh):
        if not self.registered:return
        remover=gmsh.lib.gmshModelMeshRemoveSizeCallback
        remover.argtypes=[ctypes.POINTER(ctypes.c_int)];remover.restype=None
        status=ctypes.c_int()
        try:
            remover(ctypes.byref(status))
            if status.value:raise ValueError('DirectCallbackRemovalFailed',status.value)
        except BaseException:self.quarantine();raise
        self.registered=False;self.removals+=1

    @lifecycle
    def quarantine(self):
        if not self.quarantined:
            self.quarantined=True;QUARANTINE.append(self)
        if self.pointer:self.library.direct_expire(self.pointer)

    def close(self):
        with self.lock:
            if self.registered or self.quarantined:
                self.quarantine();return
            if self.pointer:
                self.library.direct_expire(self.pointer);self.library.direct_destroy(self.pointer);self.pointer=None
            if self in LIVE:LIVE.remove(self)

    def generate(self,operation,*args,**kwargs):
        # Synchronous consumer lease: close waits even when callbacks are in C.
        with self.lock:
            self.require_binding();self.require_passed()
            try:return operation(*args,**kwargs)
            finally:
                self.require_binding();self.require_passed()

class SessionBridge:
    """Two generation registrations; uncertain cleanup retains all userdata."""
    def __init__(self,gmsh,generate,finalize):
        self.gmsh=gmsh;self.original_generate=generate;self.original_finalize=finalize;self.owner=None

    def install(self,callback):
        if self.owner is None:raise ValueError('MissingDirectCallbackOwner')
        if callback is not self.owner and getattr(callback,'field',None) is not self.owner.field:
            raise ValueError('ChangedDirectCallbackInstaller')
        self.owner.install(self.gmsh)

    def generate(self,*args,**kwargs):
        owner=self.owner
        if owner is None:raise ValueError('UnregisteredDirectGenerate')
        with owner.lock:
            if self.owner is not owner or not owner.registered:raise ValueError('UnregisteredDirectGenerate')
            return owner.generate(self.original_generate,*args,**kwargs)

    def finalize(self,*args,**kwargs):
        owner=self.owner
        if owner is None:return self.original_finalize(*args,**kwargs)
        with owner.lock:
            failure=None;result=None
            try:owner.remove(self.gmsh)
            except BaseException as error:failure=error;owner.quarantine()
            try:result=self.original_finalize(*args,**kwargs)
            except BaseException as error:
                owner.quarantine()
                if failure is None:failure=error
                else:failure.add_note('DirectFinalizationFailure: '+repr(error))
            if failure is not None:raise failure
            return result
