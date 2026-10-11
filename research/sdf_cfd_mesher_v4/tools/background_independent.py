"""Isolated compiled execution of the existing independent Python tree walk."""
from pathlib import Path
import ctypes, hashlib
import numpy as np
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex as ExistingIndex
from sdf_cfd_mesher_v4.certificate.efficient import Buffer

HERE=Path(__file__).resolve().parents[1]/'runtime/source/background'
class AABBIndex(ExistingIndex):
    def __init__(self,lo,hi,leaf_size=32,*,directory=None):
        super().__init__(lo,hi,leaf_size)
        self.directory=HERE if directory is None else Path(directory)

    def pair_blocks(self,active=None):
        # Native execution is only a binary64 fast path. Preserve the existing
        # integer/other dtype comparisons instead of rounding large integers.
        if self.lo.dtype!=np.dtype('float64') or self.hi.dtype!=np.dtype('float64'):
            yield from super().pair_blocks(active)
            return
        mask=np.ones(len(self.lo),np.uint8) if active is None else np.zeros(len(self.lo),np.uint8)
        if active is not None:
            ids=list(set(active))
            if any(not isinstance(i,(int,np.integer)) or isinstance(i,(bool,np.bool_)) or i<0 or i>=len(self.lo) for i in ids):
                raise ValueError('InvalidActiveAABB')
            if not ids:return
            mask[np.asarray(ids,dtype=np.intp)]=1
        if not len(self.lo):return
        source=self.directory/'independent_walk.rs';digest=hashlib.sha256(source.read_bytes()).hexdigest()
        from background_native_guard import held_library
        with held_library(self.directory/('independent_'+digest+'.dll'),{'independent_walk.rs':digest}) as dll:
            yield from self._native_blocks(dll,mask)

    def _native_blocks(self,dll,mask):
        p=ctypes.c_void_p;s=ctypes.c_size_t
        dll.walk_new.argtypes=[p,p,s,p,p,p,s,p,s,p];dll.walk_new.restype=p
        dll.walk_next.argtypes=[p,s];dll.walk_next.restype=Buffer
        dll.walk_failed.argtypes=[p];dll.walk_failed.restype=ctypes.c_uint8
        dll.walk_free.argtypes=[p];dll.walk_free.restype=None
        dll.walk_buffer_free.argtypes=[Buffer];dll.walk_buffer_free.restype=None
        lower=np.empty((len(self.nodes),3));upper=np.empty_like(lower);desc=np.zeros((len(self.nodes),4),np.int64)
        leaves=[]
        for i,(lo,hi,left,right) in enumerate(self.nodes):
            lower[i]=lo;upper[i]=hi
            if right is None:
                desc[i]=(-1,-1,len(leaves),len(left));leaves.extend(left)
            else:desc[i]=(left,right,0,0)
        leaves=np.asarray(leaves,np.uint64)
        lo=np.require(self.lo,dtype=np.float64,requirements=['C','A'])
        hi=np.require(self.hi,dtype=np.float64,requirements=['C','A'])
        handle=dll.walk_new(lo.ctypes.data,hi.ctypes.data,len(lo),lower.ctypes.data,upper.ctypes.data,
            desc.ctypes.data,len(desc),leaves.ctypes.data,len(leaves),mask.ctypes.data)
        if not handle:raise ValueError('InvalidIndependentTree')
        del lower,upper,desc,leaves
        try:
            while True:
                block=dll.walk_next(handle,16384)
                if dll.walk_failed(handle):raise ValueError('IndependentTraversalFailure')
                try:
                    if not block.length:break
                    rows=np.ctypeslib.as_array(block.data,shape=(block.length,)).copy().reshape(-1,2)
                finally:dll.walk_buffer_free(block)
                yield rows
        finally:dll.walk_free(handle)
