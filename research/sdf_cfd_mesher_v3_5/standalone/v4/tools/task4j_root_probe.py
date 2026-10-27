"""Same selected pairs, pure root preparation off/on, full proof equality."""
from task4j_common import *
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.certificate.incremental import kernel
from sdf_cfd_mesher_v4.certificate.efficient_root_keys import memoized_root_keys
from sdf_cfd_mesher_v4.certificate.exact_preparation import memoized_exact_preparation
import gzip,contextlib,psutil
def run():
    f,t,x=geometry();q,labels=surface_arrays(f.coordinates,t.edges,x[1:]);selection=read(OUT/'BOUNDED_KERNEL_PROFILE.json');results=[]
    for mode in ('old','memoized','memoized_replay'):
        records=[];total=0.;counts=collections.Counter()
        for batch_start in range(0,len(selection),8):
            require_time();kernel.clear_kernel_caches()
            with memoized_root_keys(),(memoized_exact_preparation() if mode!='old' else contextlib.nullcontext({})) as stats:
                for row in selection[batch_start:batch_start+8]:
                    a,b=(kernel.Patch.binary(q[i],tuple(map(int,labels[i]))) for i in row['pair'])
                    start=time.perf_counter();r=kernel.certificate.certify(a,b);dt=time.perf_counter()-start;total+=dt
                    assert scientific_hash(r)==row['result_hash'],('ProofChanged',mode,row['pair'])
                    records.append(dict(layer=row['layer'],pair=row['pair'],method=r['method'],result_hash=scientific_hash(r),seconds=dt))
            counts.update(stats)
        result=dict(mode=mode,seconds=total,counts=dict(counts),records=records,scientific_hash=scientific_hash([{k:v for k,v in r.items() if k!='seconds'} for r in records]),peak_rss=psutil.Process().memory_info().peak_wset)
        results.append(result);save('ROOT_PREPARATION_EXPERIMENT.json',results);print(mode,total,dict(counts),flush=True)
if __name__=='__main__':run()
