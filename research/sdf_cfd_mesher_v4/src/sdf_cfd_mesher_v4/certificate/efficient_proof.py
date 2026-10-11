"""Batch proof hierarchy. A filter never accepts an uncertain mathematical sign."""
import time,collections
import numpy as np
from .incremental import kernel
from .efficient import Filter
from .efficient_ledger import digest
from .efficient_root_keys import memoized_root_keys
from .exact_preparation import memoized_exact_preparation
from .parallel_screen import screen
from .cad_algebra_cache import memoized_cad_algebra,cache_info as cad_cache_info
PROOF_CLASSES={1:'DisjointByCornerAABB',2:'DisjointByCertifiedSupportingPlane',3:'OwnedVertexByCertifiedSupportingPlane',4:'OwnedEdgeByCertifiedSupportingPlane',5:'OwnedVertexByAccumulatedCertifiedPlanes',6:'OwnedEdgeByAccumulatedCertifiedPlanes'}
def exact_batch(job):
    before=cad_cache_info()
    with memoized_cad_algebra(), memoized_root_keys() as keys, memoized_exact_preparation() as stats:
        rows,times=_exact_batch(job)
    times.update({"root_preparation:"+k:v for k,v in stats.items()})
    times.update({"root_keys:"+k:v for k,v in keys.cache_info()._asdict().items() if k!="maxsize"})
    for name,info in cad_cache_info().items():
        for key in ('hits','misses'):
            times['cad_algebra:'+name+':'+key]=info[key]-before[name][key]
    return rows,times

def _exact_batch(job):
    indices,quads,labels,pairs=job;patches={};rows=[];times=collections.Counter();kernel.clear_kernel_caches()
    def patch(i):
        if i not in patches:
            t=time.perf_counter();p=kernel.Patch.binary(quads[i],tuple(map(int,labels[i])));ok,why=p.valid();times['exact_patch_preparation']+=time.perf_counter()-t
            if not ok:raise ValueError(('InvalidSidePatch',int(i),why))
            patches[i]=p
        return patches[i]
    for index,(a,b) in zip(indices,pairs):
        pa,pb=patch(a),patch(b);t=time.perf_counter();r=kernel.certificate.certify(pa,pb);elapsed=time.perf_counter()-t
        times['kernel']+=elapsed;times['method:'+r['method']]+=elapsed;rows.append((int(index),r,tuple(sorted((pa.rank,pb.rank)))))
    return rows,dict(times)

@memoized_cad_algebra()
@memoized_root_keys()
def prove(q,labels,packed,native=None,executor=None,screen_workers=1):
    native=Filter() if native is None else native;started=time.perf_counter()
    pairs=np.column_stack((packed>>np.uint64(32),packed&np.uint64(0xffffffff))).astype(np.uint32)
    codes,witness=screen(native,q,labels,pairs,screen_workers);filter_seconds=time.perf_counter()-started
    outcomes=np.where(codes<=2,1,2).astype(np.uint8);fallback=[];patches={};methods=collections.Counter();ranks=collections.Counter();times=collections.Counter()
    if executor is not None:
        pending=[];unknown=np.flatnonzero(codes==0);started=time.perf_counter()
        for start in range(0,len(unknown),8):
            ix=unknown[start:start+8];vertices,local=np.unique(pairs[ix],return_inverse=True)
            pending.append(executor.submit(exact_batch,(ix,q[vertices],labels[vertices],local.reshape(-1,2))))
        dispatch=time.perf_counter()-started
        for future in pending:
            rows,work=future.result();times.update(work)
            for i,r,rank in rows:
                outcomes[i]=1 if r['classification']=='CertifiedSeparated' else 2 if r['classification']=='CertifiedAllowedIncidentContact' else 3
                fallback.append(dict(pair=list(map(int,pairs[i])),result=r));methods[r['method']]+=1;ranks[str(rank)]+=1
        return dict(codes=codes,witness=witness,outcomes=outcomes,fallback=fallback),dict(filter_seconds=filter_seconds,
            remaining_exact_seconds=time.perf_counter()-started,exact_calls=len(fallback),exact_methods=dict(methods),exact_ranks=dict(ranks),details=dict(times),dispatch_seconds=dispatch)
    def patch(i):
        if i not in patches:
            t=time.perf_counter();p=kernel.Patch.binary(q[i],tuple(map(int,labels[i])));ok,why=p.valid();times['exact_patch_preparation']+=time.perf_counter()-t
            if not ok:raise ValueError(('InvalidSidePatch',i,why))
            patches[i]=p
        return patches[i]
    started=time.perf_counter();kernel.clear_kernel_caches()
    for i in np.flatnonzero(codes==0):
        a,b=map(int,pairs[i]);pa,pb=patch(a),patch(b);t=time.perf_counter();r=kernel.certificate.certify(pa,pb);times['kernel']+=time.perf_counter()-t
        methods[r['method']]+=1;times['method:'+r['method']]+=time.perf_counter()-t;ranks[str(tuple(sorted((pa.rank,pb.rank))))]+=1
        outcomes[i]=1 if r['classification']=='CertifiedSeparated' else 2 if r['classification']=='CertifiedAllowedIncidentContact' else 3
        fallback.append(dict(pair=[a,b],result=r))
    return dict(codes=codes,witness=witness,outcomes=outcomes,fallback=fallback),dict(filter_seconds=filter_seconds,
        remaining_exact_seconds=time.perf_counter()-started,exact_calls=len(fallback),exact_methods=dict(methods),exact_ranks=dict(ranks),details=dict(times))
