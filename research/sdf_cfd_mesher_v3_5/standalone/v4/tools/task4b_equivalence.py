from task4b_layer_probe import *
import gzip
from sdf_cfd_mesher_v4.certificate.incremental import kernel
def run():
    f,t,x=geometry();q,ids=arrays(x[:2],t.edges,1);E=len(t.edges)
    with np.load(OUT/'LAYER_01_PROBE.npz') as z:packed=z['packed'];codes=z['codes']
    start=time.perf_counter();oldpairs=[];oldclasses=[]
    for file in sorted((BASE/'certificate_layer_01').glob('PAIRS*.gz')):
        for line in gzip.open(file,'rb'):
            r=json.loads(line);a=(r['side_a'][0]-1)*E+r['side_a'][1];b=(r['side_b'][0]-1)*E+r['side_b'][1]
            oldpairs.append((min(a,b)<<32)|max(a,b));oldclasses.append(1 if r['classification']=='CertifiedSeparated' else 2 if r['classification']=='CertifiedAllowedIncidentContact' else 3)
    order=np.argsort(np.array(oldpairs,np.uint64));oldpairs=np.array(oldpairs,np.uint64)[order];oldclasses=np.array(oldclasses,np.uint8)[order]
    assert np.array_equal(oldpairs,packed) and len(np.unique(packed))==len(packed)
    baseline_read=time.perf_counter()-start;result=np.where(codes<=2,1,2).astype(np.uint8);exact=[];patches={};start=time.perf_counter();kernel.clear_kernel_caches()
    def patch(i):
        if i not in patches:patches[i]=kernel.Patch.binary(q[i],tuple(map(int,ids[i])))
        return patches[i]
    for i in np.flatnonzero(codes==0):
        a=int(packed[i])>>32;b=int(packed[i])&0xffffffff
        r=kernel.certificate.certify(patch(a),patch(b));result[i]=1 if r['classification']=='CertifiedSeparated' else 2 if r['classification']=='CertifiedAllowedIncidentContact' else 3
        exact.append(dict(pair=[a,b],result=r))
    assert np.array_equal(result,oldclasses)
    save('LAYER_01_EQUIVALENCE.json',dict(pair_count=len(packed),pair_population_identical=True,outcomes_identical=True,old_record_read_seconds=baseline_read,
        remaining_exact_seconds=time.perf_counter()-start,remaining_exact_calls=len(exact),pair_hash=array_hash(packed),outcome_hash=array_hash(result),exact=exact))
    print('LAYER1 EQUIVALENCE PASSED',len(packed),'fallback',len(exact),'seconds',time.perf_counter()-start,flush=True)
if __name__=='__main__':run()
