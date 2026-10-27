"""Cold worker pools separate preparation reuse from prior SymPy warming."""
from task4j_common import *
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.certificate.efficient_proof import exact_batch,_exact_batch
from sdf_cfd_mesher_v4.certificate.efficient_root_keys import memoized_root_keys
from concurrent.futures import ProcessPoolExecutor
import gzip
@memoized_root_keys()
def original_job(job):return _exact_batch(job)
def run():
    f,t,x=geometry();q,labels=surface_arrays(f.coordinates,t.edges,x[1:]);results=[]
    for layer in (2,12,20):
        path=BASE/f'certificate_layer_{layer:02d}'
        with np.load(path/'COMPACT.npz') as z:p=z['packed'];ix=np.flatnonzero(z['codes']==0)
        pairs=np.column_stack((p>>np.uint64(32),p&np.uint64(0xffffffff))).astype(np.uint32)
        with gzip.open(path/'EXACT.json.gz','rb') as z:expected=json.load(z)
        for mode,function in (('original',original_job),('preparation_cache',exact_batch)):
            require_time();s=time.perf_counter();proof=[];details=collections.Counter()
            with ProcessPoolExecutor(max_workers=8) as pool:
                jobs=[]
                for start in range(0,len(ix),8):
                    indices=ix[start:start+8];vertices,local=np.unique(pairs[indices],return_inverse=True)
                    jobs.append(pool.submit(function,(indices,q[vertices],labels[vertices],local.reshape(-1,2))))
                for job in jobs:
                    rows,stats=job.result();details.update(stats)
                    proof.extend(dict(pair=list(map(int,pairs[i])),result=r) for i,r,rank in rows)
            assert proof==expected
            results.append(dict(layer=layer,mode=mode,seconds=time.perf_counter()-s,details=dict(details),exact_calls=len(ix),scientific_hash=scientific_hash(proof)));save('CONTROLLED_ROOT_PREPARATION.json',results);print(layer,mode,results[-1]['seconds'],flush=True)
if __name__=='__main__':run()
