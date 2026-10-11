"""Exact-safe interval proofs with frozen exact fallback. No mesh mutations."""
from pathlib import Path
import ctypes,subprocess,hashlib
import numpy as np
SOURCE=Path(__file__).with_name('efficient_native.rs')
ROOT=SOURCE.parents[3]
class Buffer(ctypes.Structure):
    _fields_=[('data',ctypes.POINTER(ctypes.c_uint64)),('length',ctypes.c_size_t),('capacity',ctypes.c_size_t)]
class Filter:
    def __init__(self):
        self.source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
        path=ROOT/'runtime/bin'/('proof_'+self.source_hash+'.dll')
        if not path.exists():raise RuntimeError('InstalledCertificateBinaryRequired')
        self.dll=ctypes.CDLL(str(path));p=ctypes.c_void_p;s=ctypes.c_size_t
        self.dll.screen.argtypes=[p,p,s,p,s,p,p];self.dll.screen.restype=None
        self.dll.validity.argtypes=[p,s,p];self.dll.validity.restype=None
        self.dll.forest_new.argtypes=[p,s,s];self.dll.forest_new.restype=p
        self.dll.forest_free.argtypes=[p];self.dll.forest_free.restype=None
        self.dll.forest_append.argtypes=[p,p];self.dll.forest_append.restype=None
        self.dll.ieee_environment.argtypes=[];self.dll.ieee_environment.restype=ctypes.c_uint8
        if not self.dll.ieee_environment():raise RuntimeError('GradualUnderflowRequired')
        self.dll.forest_pairs.argtypes=[p,s];self.dll.forest_pairs.restype=Buffer
        self.dll.buffer_free.argtypes=[Buffer];self.dll.buffer_free.restype=None
    def screen(self,q,labels,pairs):
        q=np.ascontiguousarray(q,dtype=np.float64);labels=np.ascontiguousarray(labels,dtype=np.uint64);pairs=np.ascontiguousarray(pairs,dtype=np.uint32).reshape(-1,2)
        if q.ndim!=3 or q.shape[1:]!=(4,3) or labels.shape!=(len(q),4) or not np.isfinite(q).all():raise ValueError('InvalidFilterGeometry')
        if len(pairs) and int(pairs.max())>=len(q):raise ValueError('InvalidFilterPair')
        out=np.empty(len(pairs),np.uint8);w=out.copy();self.dll.screen(q.ctypes.data,labels.ctypes.data,len(q),pairs.ctypes.data,len(pairs),out.ctypes.data,w.ctypes.data);return out,w
    def valid(self,q):
        q=np.ascontiguousarray(q,dtype=np.float64)
        if q.ndim!=3 or q.shape[1:]!=(4,3) or not np.isfinite(q).all():raise ValueError('InvalidFilterGeometry')
        out=np.empty(len(q),np.uint8);self.dll.validity(q.ctypes.data,len(q),out.ctypes.data);return out
    def forest(self,q,width):return Forest(self,q,width)
class Forest:
    def __init__(self,native,q,width):
        self.handle=None
        q=np.ascontiguousarray(q,dtype=np.float64)
        if q.ndim!=3 or q.shape[1:]!=(4,3) or not np.isfinite(q).all() or width<=0 or len(q)%width or len(q)>=2**32:raise ValueError('InvalidForestGeometry')
        self.native=native;self.layers=len(q)//width;self.width=width;self.handle=native.dll.forest_new(q.ctypes.data,len(q),width)
    def append(self,q):
        q=np.ascontiguousarray(q,dtype=np.float64)
        if q.shape!=(self.width,4,3) or not np.isfinite(q).all() or (self.layers+1)*self.width>=2**32:raise ValueError('InvalidForestAppend')
        self.native.dll.forest_append(self.handle,q.ctypes.data);self.layers+=1
    def pairs(self,layer):
        if not self.handle or not 1<=layer<=self.layers:raise ValueError('InvalidForestLayer')
        b=self.native.dll.forest_pairs(self.handle,layer)
        try:return np.ctypeslib.as_array(b.data,shape=(b.length,)).copy()
        finally:self.native.dll.buffer_free(b)
    def close(self):
        if self.handle:self.native.dll.forest_free(self.handle);self.handle=None
    def __del__(self):self.close()
