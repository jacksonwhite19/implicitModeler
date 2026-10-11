"""Complete immediate proof as disjoint bounded source-edge ownership batches.

Every pair belongs to the smallest newly created source-edge ID in the pair.
No prior/prior proof is invalidated. The frozen exact pair kernel is unchanged.
"""
from dataclasses import dataclass,replace
from types import SimpleNamespace
from pathlib import Path
import gzip,hashlib,time,json
from concurrent.futures import ProcessPoolExecutor,wait,FIRST_COMPLETED
import numpy as np
from .incremental import SurfaceScope,build_scope,certify,ProofCache
from ..forecast.detailed.continuous import SideIndex
from ..advancing_front.private_rolling import restore_exact_proofs

@dataclass(frozen=True)
class OwnedScope(SurfaceScope):
    new_layer:int
    first_edge:int
    last_edge:int
    def pairs(self):
        pairs,performance=super().pairs()
        if not len(pairs):return pairs,performance
        owner=np.array([e if k==self.new_layer else np.iinfo(np.int64).max for k,e in self.sides],dtype=np.int64)
        responsible=np.minimum(owner[pairs[:,0]],owner[pairs[:,1]])
        return pairs[(responsible>=self.first_edge)&(responsible<self.last_edge)],performance
    def scope_identity(self):
        return hashlib.sha256((super().scope_identity()+f':owned:{self.new_layer}:{self.first_edge}:{self.last_edge}').encode()).hexdigest()

def owned_scope(index,layer,begin,end):
    base=build_scope(index,np.arange(begin,end),max_edges=len(index.topology.edges)-1)
    core=np.array([bool(flag) and k==layer for flag,(k,e) in zip(base.core,base.sides)])
    return OwnedScope(base.quads,base.labels,base.sides,core,base.topology,base.edges,layer,begin,end)

def prove_scope(scope,record_path,meta_path,cache=None):
    """One independent immutable exact scope, with the retained kernel."""
    import psutil
    started=time.perf_counter();cache=ProofCache() if cache is None else cache
    record_path=Path(record_path);meta_path=Path(meta_path)
    if meta_path.exists():
        previous=json.loads(meta_path.read_text())
        with gzip.open(record_path,'rb') as f:restore_exact_proofs(scope,previous,f.read(),cache)
    with record_path.with_suffix('.pending').open('wb') as file:
        with gzip.GzipFile(filename='',mode='wb',fileobj=file,mtime=0) as sink:
            proof,perf=certify(scope,cache=cache,seconds_budget=float('inf'),record_sink=sink.write)
    record_path.with_suffix('.pending').replace(record_path)
    meta_path.write_text(json.dumps(proof,sort_keys=True,indent=2)+'\n')
    perf.update(batch_seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset)
    return proof,perf

_worker_index=None
def initialize_worker(wall,edges,topology_hash,prefix):
    global _worker_index
    _worker_index=SideIndex(SimpleNamespace(coordinates=wall),SimpleNamespace(vertex={'coordinates':prefix}),SimpleNamespace(edges=edges,sha256=topology_hash))

def batch_worker(job):
    layer,begin,end,path=job;started=time.perf_counter();scope=owned_scope(_worker_index,layer,begin,end)
    path=Path(path);proof,perf=prove_scope(scope,path/f'PAIRS_{begin:06}_{end:06}.jsonl.gz',path/f'PROOF_{begin:06}_{end:06}.json')
    perf['scope_and_batch_seconds']=time.perf_counter()-started
    perf.update(scope_source_edges=len(scope.edges),scope_side_surfaces=len(scope.sides),new_core_surfaces=int(scope.core.sum()))
    (path/f'TIMING_{begin:06}_{end:06}.json').write_text(json.dumps(perf,sort_keys=True,indent=2)+'\n')
    return begin,end,proof,perf

def verify_batch_worker(job):
    """Read-only complete re-enumeration of an already committed proof scope."""
    layer,batch,path=job;begin=batch['first_edge'];end=batch['last_edge'];started=time.perf_counter()
    scope=owned_scope(_worker_index,layer,begin,end);path=Path(path)
    stored=json.loads((path/f'PROOF_{begin:06}_{end:06}.json').read_text())
    assert stored=={k:v for k,v in batch.items() if k not in ('first_edge','last_edge')},'FinalBatchAuthorityMismatch'
    assert stored['complete'] and not any(stored[k] for k in ('forbidden','unresolved','invalid_surfaces'))
    with gzip.open(path/f'PAIRS_{begin:06}_{end:06}.jsonl.gz','rb') as f:raw=f.read()
    count=restore_exact_proofs(scope,stored,raw,ProofCache(0))
    return dict(first_edge=begin,last_edge=end,pairs=count,records_sha256=stored['records_sha256']),dict(seconds=time.perf_counter()-started,scope_source_edges=len(scope.edges),scope_side_surfaces=len(scope.sides))

def verify_committed_layer(front,top,prefix,path,ledger,*,workers=1):
    """Finished geometry must reproduce every disjoint committed pair scope."""
    layer=len(prefix);assert ledger['layer']==layer and ledger['complete']
    jobs=[(layer,b,str(path)) for b in ledger['batches']];rows=[];performance=[]
    if workers==1:
        initialize_worker(front.coordinates,top.edges,top.sha256,prefix)
        results=map(verify_batch_worker,jobs)
        for row,perf in results:rows.append(row);performance.append(perf)
    else:
        with ProcessPoolExecutor(max_workers=workers,initializer=initialize_worker,initargs=(front.coordinates,top.edges,top.sha256,prefix)) as pool:
            for row,perf in ordered_bounded_results(jobs,lambda j:pool.submit(verify_batch_worker,j),workers):rows.append(row);performance.append(perf)
    cursor=0;count=0
    for row in rows:assert row['first_edge']==cursor;cursor=row['last_edge'];count+=row['pairs']
    assert cursor==len(top.edges) and count==ledger['completed_pairs']==ledger['candidate_pairs']
    return count,performance

def ordered_bounded_results(jobs,submit,workers):
    """Keep workers occupied but yield compact results in input order."""
    jobs=iter(enumerate(jobs));pending={};ready={};next_result=0
    def fill():
        while len(pending)<workers:
            item=next(jobs,None)
            if item is None:break
            i,job=item;pending[submit(job)]=i
    fill()
    try:
        while pending:
            completed,_=wait(pending,return_when=FIRST_COMPLETED)
            for future in completed:ready[pending.pop(future)]=future.result()
            fill()
            while next_result in ready:
                yield ready.pop(next_result);next_result+=1
        while next_result in ready:
            yield ready.pop(next_result);next_result+=1
    finally:
        for future in pending:future.cancel()

def certify_immediate(front,top,prefix,path,*,batch=512,cache=None,progress=None,workers=1):
    path=Path(path);path.mkdir(parents=True,exist_ok=True);cache=ProofCache() if cache is None else cache
    layer=len(prefix);index=SideIndex(front,SimpleNamespace(vertex={'coordinates':prefix}),top)
    proofs=[];timings=[]
    if workers<1:raise ValueError('InvalidCertificateWorkerCount')
    def accept(begin,end,proof,perf):
        proofs.append(dict(first_edge=begin,last_edge=end,**proof));timings.append(perf)
        if progress:progress(layer,begin,end,proof,perf)
        return proof['complete'] and not any(proof[k] for k in ('forbidden','unresolved','invalid_surfaces'))
    if workers==1:
        for begin in range(0,len(top.edges),batch):
            end=min(begin+batch,len(top.edges));scope=owned_scope(index,layer,begin,end)
            proof,perf=prove_scope(scope,path/f'PAIRS_{begin:06}_{end:06}.jsonl.gz',path/f'PROOF_{begin:06}_{end:06}.json',cache)
            perf.update(scope_source_edges=len(scope.edges),scope_side_surfaces=len(scope.sides),new_core_surfaces=int(scope.core.sum()))
            (path/f'TIMING_{begin:06}_{end:06}.json').write_text(json.dumps(perf,sort_keys=True,indent=2)+'\n')
            if not accept(begin,end,proof,perf):break
    else:
        # Only workers scopes may be in flight. Collection follows source-edge
        # order, independent of scheduling. No optimizer or global sensitivity
        # is involved, and every scope still runs the unchanged exact kernel.
        jobs=((layer,b,min(b+batch,len(top.edges)),str(path)) for b in range(0,len(top.edges),batch))
        with ProcessPoolExecutor(max_workers=workers,initializer=initialize_worker,initargs=(front.coordinates,top.edges,top.sha256,prefix)) as pool:
            results=ordered_bounded_results(jobs,lambda job:pool.submit(batch_worker,job),workers)
            try:
                for result in results:
                    if not accept(*result):break
            finally:results.close()
    result=dict(complete=bool(proofs) and proofs[-1]['last_edge']==len(top.edges) and all(p['complete'] for p in proofs),
        candidate_pairs=sum(p['candidate_pairs'] for p in proofs),completed_pairs=sum(p['completed_pairs'] for p in proofs),
        forbidden=[r for p in proofs for r in p['forbidden']],unresolved=[r for p in proofs for r in p['unresolved']],
        invalid_surfaces=[r for p in proofs for r in p['invalid_surfaces']],batches=proofs,layer=layer)
    return result,timings
