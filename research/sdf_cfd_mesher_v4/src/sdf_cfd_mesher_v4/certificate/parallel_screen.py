"""Disjoint deterministic slices of the unchanged qualified native kernel."""
from concurrent.futures import ThreadPoolExecutor
import numpy as np

def screen(native,q,labels,pairs,workers=4):
    if workers<1:raise ValueError('InvalidScreenWorkerCount')
    q=np.ascontiguousarray(q,dtype=np.float64);labels=np.ascontiguousarray(labels,dtype=np.uint64);pairs=np.ascontiguousarray(pairs,dtype=np.uint32).reshape(-1,2)
    if q.ndim!=3 or q.shape[1:]!=(4,3) or labels.shape!=(len(q),4) or not np.isfinite(q).all():raise ValueError('InvalidFilterGeometry')
    if len(pairs) and int(pairs.max())>=len(q):raise ValueError('InvalidFilterPair')
    out=np.empty(len(pairs),np.uint8);w=np.empty_like(out)
    def block(start,end):
        if not native.dll.ieee_environment():raise RuntimeError('GradualUnderflowRequired')
        native.dll.screen(q.ctypes.data,labels.ctypes.data,len(q),pairs[start:end].ctypes.data,end-start,out[start:end].ctypes.data,w[start:end].ctypes.data)
    bounds=np.linspace(0,len(pairs),min(workers,max(1,len(pairs)))+1,dtype=np.int64)
    with ThreadPoolExecutor(max_workers=workers) as pool:
        list(pool.map(lambda ab:block(*ab),zip(bounds[:-1],bounds[1:])))
    return out,w
