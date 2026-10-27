from task4j_common import *
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.certificate.efficient_proof import exact_batch
from concurrent.futures import ProcessPoolExecutor
import gzip
def run():
    f,t,x=geometry();q,labels=surface_arrays(f.coordinates,t.edges,x[1:12+1]);path=BASE/'certificate_layer_12'
    with np.load(path/'COMPACT.npz') as z:p=z['packed'];ix=np.flatnonzero(z['codes']==0)
    pairs=np.column_stack((p>>np.uint64(32),p&np.uint64(0xffffffff))).astype(np.uint32)
    with gzip.open(path/'EXACT.json.gz','rb') as z:expected=json.load(z)
    rows=[]
    for size in (8,32,64):
        require_time();s=time.perf_counter();jobs=[];proof=[];details=collections.Counter()
        with ProcessPoolExecutor(max_workers=8) as pool:
            ds=time.perf_counter()
            for start in range(0,len(ix),size):
                indices=ix[start:start+size];vertices,local=np.unique(pairs[indices],return_inverse=True)
                jobs.append(pool.submit(exact_batch,(indices,q[vertices],labels[vertices],local.reshape(-1,2))))
            dispatch=time.perf_counter()-ds
            for job in jobs:
                results,stats=job.result();details.update(stats)
                proof.extend(dict(pair=list(map(int,pairs[i])),result=r) for i,r,rank in results)
        assert proof==expected
        rows.append(dict(batch=size,seconds=time.perf_counter()-s,dispatch_seconds=dispatch,jobs=len(jobs),calls=len(ix),details=dict(details),proof_hash=scientific_hash(proof)));save('EXACT_BATCH_EXPERIMENT.json',rows);print(size,rows[-1]['seconds'],flush=True)
if __name__=='__main__':run()
