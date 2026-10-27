"""Bounded exact wedge receipts, keyed by all 18 binary64 coordinates."""
from collections import OrderedDict
from copy import deepcopy
import numpy as np
from ..exact_dyadic import exact_wedge

class ExactJacobianCache:
    def __init__(self,max_entries=32768):
        if max_entries<1:raise ValueError('PositiveExactCacheBoundRequired')
        self.entries=OrderedDict();self.max_entries=max_entries;self.hits=0;self.misses=0
    def evaluate(self,prism):
        p=np.ascontiguousarray(prism,dtype='<f8')
        if p.shape!=(6,3):raise ValueError('ExactPrismShape')
        key=p.tobytes()
        if key in self.entries:
            self.hits+=1;self.entries.move_to_end(key);return deepcopy(self.entries[key])
        self.misses+=1;r=exact_wedge(p);self.entries[key]=r
        while len(self.entries)>self.max_entries:self.entries.popitem(last=False)
        return deepcopy(r)

# Process-local only. No receipt survives a changed implementation executable.
LOCAL_EXACT=ExactJacobianCache()
