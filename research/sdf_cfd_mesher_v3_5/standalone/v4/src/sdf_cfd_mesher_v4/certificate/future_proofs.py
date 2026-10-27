"""Bounded future pair proofs using the qualified Task-4B filter and kernel.

Keys contain every binary64 corner bit, ordered original labels, the frozen
kernel version and the qualified filter source hash. Neither source IDs alone
nor unchanged AABBs authorize reuse. Filter uncertainty always falls through.
This changes proof representation only; selection and decision rules stay fixed.
"""
from collections import OrderedDict,Counter
from copy import deepcopy
import time
import numpy as np
from .efficient import Filter
from .efficient_proof import PROOF_CLASSES
from .efficient_root_keys import memoized_root_keys
from .continuous_prism_certificate import repaired_kernel as kernel
from .continuous_prism_certificate.repaired_runtime import CertificateSession
from ..common.authority import identity,array_hash

VERSION='V4FutureExactProofs/1'

class FutureSpotProofs:
    def __init__(self,capacity=4096):
        if capacity<0:raise ValueError('NegativeCacheCapacity')
        self.capacity=capacity;self.native=Filter();self.cache=OrderedDict();self.patches=OrderedDict();self.stats=Counter();self.times=Counter();self.session=CertificateSession()
    def _store(self,cache,key,value):
        if self.capacity:
            cache[key]=value
            if len(cache)>self.capacity:cache.popitem(last=False)
    def snapshot(self):
        rows=[dict(coordinates=key[3].hex(),labels=key[4],result=deepcopy(value)) for key,value in self.cache.items()]
        data=dict(version=VERSION,kernel=kernel.VERSION,filter=self.native.source_hash,rows=rows)
        return dict(data,cache_hash=identity(data))
    def restore(self,snapshot):
        data=dict(snapshot);h=data.pop('cache_hash',None)
        if h!=identity(data) or data.get('version')!=VERSION or data.get('kernel')!=kernel.VERSION or data.get('filter')!=self.native.source_hash:raise ValueError('FutureProofCacheIdentityMismatch')
        restored=OrderedDict()
        for row in data['rows']:
            raw=bytes.fromhex(row['coordinates']);labels=tuple(tuple(tuple(map(int,v)) for v in s) for s in row['labels'])
            if len(raw)!=192 or len(labels)!=2 or any(len(s)!=4 for s in labels) or not np.isfinite(np.frombuffer(raw,dtype='<f8')).all():raise ValueError('InvalidFutureProofCacheInputs')
            key=(VERSION,kernel.VERSION,self.native.source_hash,raw,labels)
            if key in restored:raise ValueError('DuplicateFutureProofCacheKey')
            result=deepcopy(row['result'])
            if 'proof' in result:result['proof']['labels']=labels
            restored[key]=result
        if len(restored)>self.capacity:raise ValueError('FutureProofCacheCapacityExceeded')
        self.cache=restored
    def future_pair(self,quads,labels):
        q=np.ascontiguousarray(quads,dtype='<f8')
        labels=tuple(tuple(tuple(map(int,v)) for v in side) for side in labels)
        if q.shape!=(2,4,3) or len(labels)!=2 or any(len(x)!=4 for x in labels) or not np.isfinite(q).all():raise ValueError('InvalidFuturePairInputs')
        key=(VERSION,kernel.VERSION,self.native.source_hash,q.tobytes(),labels)
        self.stats['requests']+=1
        if key in self.cache:
            self.stats['hits']+=1;r=self.cache.pop(key);self.cache[key]=r;return deepcopy(r)
        self.stats['misses']+=1;t=time.perf_counter();patches=[]
        valid=self.native.valid(q)
        for i in range(2):
            if not valid[i]:
                pk=(q[i].tobytes(),labels[i]);p=self.patches.get(pk)
                if p is None:p=kernel.Patch.binary(q[i],labels[i]);self._store(self.patches,pk,p);self.stats['patches_constructed']+=1
                ok,why=p.valid();self.stats['validity_fallback']+=1
                if not ok:
                    r=dict(classification='InvalidSidePatch',reason=why);self._store(self.cache,key,deepcopy(r));return r
            else:p=None
            patches.append(p)
        self.times['validity']+=time.perf_counter()-t
        # Injective relabeling preserves every shared-vertex/shared-edge rule.
        ordered=sorted(set(labels[0]+labels[1]));ids={v:i for i,v in enumerate(ordered)}
        encoded=np.array([[ids[v] for v in side] for side in labels],dtype=np.uint64)
        t=time.perf_counter();codes,witness=self.native.screen(q,encoded,[[0,1]]);self.times['filter']+=time.perf_counter()-t
        code=int(codes[0]);self.stats['code:'+str(code)]+=1
        if code:
            r=dict(classification='CertifiedSeparated' if code<=2 else 'CertifiedAllowedIncidentContact',method=PROOF_CLASSES[code],
                   proof=dict(version=VERSION,kernel=kernel.VERSION,filter=self.native.source_hash,geometry=array_hash(q),labels=labels,code=code,witness=int(witness[0])))
        else:
            for i,p in enumerate(patches):
                if p is None:
                    pk=(q[i].tobytes(),labels[i]);p=self.patches.get(pk)
                    if p is None:p=kernel.Patch.binary(q[i],labels[i]);self._store(self.patches,pk,p);self.stats['patches_constructed']+=1
                    patches[i]=p
            t=time.perf_counter()
            with memoized_root_keys() as root_keys:r=self.session.pair(*patches)
            self.times['kernel']+=time.perf_counter()-t;self.stats['kernel_calls']+=1;self.stats['method:'+r.get('method','unspecified')]+=1
            self.stats['rank:'+str(tuple(sorted((p.rank for p in patches))))]+=1
            self.stats['root_key_hits']+=root_keys.cache_info().hits;self.stats['root_key_misses']+=root_keys.cache_info().misses
        self._store(self.cache,key,deepcopy(r));return deepcopy(r)

def verify_bound_record(quads,labels,record):
    """Recompute the exact-safe proof, including geometry/label identity."""
    p=record.get('proof',{});engine=FutureSpotProofs(capacity=0)
    if not p or p.get('code',0)==0:raise ValueError('ExpectedCertifiedBoundProof')
    rebuilt=engine.future_pair(quads,labels)
    if rebuilt!=record:raise ValueError('FutureBoundProofMismatch')
    return True
