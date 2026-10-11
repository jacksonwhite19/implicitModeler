"""Exact inclusive point query over already outward-rounded projected bounds."""
import ctypes
import numpy as np
from optimized_mixed_certificate import Native, Buffer


class ProjectedBoundaryIndex:
    def __init__(self,lo,hi):
        self.handle=None
        self.lo=np.asarray(lo);self.hi=np.asarray(hi)
        self.native=Native()
        self.native.dll.containing_point.argtypes=[ctypes.c_void_p,ctypes.c_void_p]
        self.native.dll.containing_point.restype=Buffer
        if not np.isfinite(self.lo).all() or not np.isfinite(self.hi).all() or not len(lo):return
        if self.lo.shape!=self.hi.shape or self.lo.shape[1:]!=(2,) or np.any(self.lo>self.hi):raise ValueError('InvalidAABBBounds')
        q=np.zeros((len(lo),4,3),dtype=np.float64)
        q[:,:2,:2]=self.lo[:,None,:];q[:,2:,:2]=self.hi[:,None,:]
        self.handle=self.native.dll.forest_new(q.ctypes.data,len(q),len(q))

    def containing(self,point):
        if not self.handle:
            return np.flatnonzero(np.all(point>=self.lo,axis=1)&np.all(point<=self.hi,axis=1)).tolist()
        p=np.array([point[0],point[1],0.],dtype=np.float64)
        b=self.native.dll.containing_point(self.handle,p.ctypes.data)
        try:return np.ctypeslib.as_array(b.data,shape=(b.length,)).tolist()
        finally:self.native.dll.buffer_free(b)

    def __del__(self):
        if self.handle:self.native.dll.forest_free(self.handle);self.handle=None
