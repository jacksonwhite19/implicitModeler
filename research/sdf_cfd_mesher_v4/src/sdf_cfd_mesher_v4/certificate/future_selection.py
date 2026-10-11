"""Exact-predicate swept-box queries for diagnostic future pair selection.

Sorted endpoints only reduce the number of boxes tested. Every returned box
still satisfies the original six IEEE comparisons with the original padding.
The shortest endpoint prefix/suffix is a superset of that conjunction. There
is no geometric margin change and this index makes no certificate decision.
"""
import numpy as np

class SortedBounds:
    def __init__(self,lo,hi):
        self.lo=np.asarray(lo);self.hi=np.asarray(hi)
        if self.lo.shape!=self.hi.shape or self.lo.ndim!=2 or self.lo.shape[1]!=3 or not np.isfinite(self.lo).all() or not np.isfinite(self.hi).all():raise ValueError('InvalidSpatialBounds')
        self.orders=[];self.values=[]
        for bounds in (self.lo,self.hi):
            for axis in range(3):
                ids=np.argsort(bounds[:,axis],kind='stable');self.orders.append(ids);self.values.append(bounds[ids,axis])
    def query(self,low,high,padding):
        lower=low-padding;upper=high+padding;n=len(self.lo);options=[]
        for axis in range(3):
            end=int(np.searchsorted(self.values[axis],upper[axis],side='right'));options.append((end,axis,0,end))
            begin=int(np.searchsorted(self.values[axis+3],lower[axis],side='left'));options.append((n-begin,axis+3,begin,n))
        count,which,begin,end=min(options);ids=self.orders[which][begin:end]
        mask=np.all(self.hi[ids]>=lower,axis=1)&np.all(self.lo[ids]<=upper,axis=1)
        return np.sort(ids[mask]),count
