"""Measured reachable object storage; native disk rows remain on disk."""
import sys
from collections.abc import Mapping
import numpy as np
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ReceiptTable, ReceiptOverlay


def retained_storage(roots):
    seen=set();result={}
    def walk(value):
        key=id(value)
        if key in seen:return 0
        seen.add(key);size=sys.getsizeof(value)
        if isinstance(value,(ReceiptTable,ReceiptOverlay)):return size+walk(value.__dict__)
        if isinstance(value,np.ndarray):return size+(walk(value.base) if value.base is not None else 0)
        if isinstance(value,Mapping):return size+sum(walk(k)+walk(v) for k,v in value.items())
        if isinstance(value,(list,tuple,set,frozenset)):return size+sum(walk(v) for v in value)
        return size
    for name,value in roots.items():result[name]=walk(value)
    # The recursive closure can survive until cyclic GC. Its visited-ID set
    # must not retain a large profiling allocation across scientific phases.
    seen.clear()
    return dict(bytes_by_first_owner=result,total_unique_python_bytes=sum(result.values()),
        scope='Reachable buffers and Python objects; disk receipt rows excluded; shared objects counted once; allocator arenas and native library working sets excluded')
