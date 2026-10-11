from task4b_layer_probe import *
from sdf_cfd_mesher_v4.certificate.incremental import kernel
from sdf_cfd_mesher_v4.certificate.efficient_root_keys import memoized_root_keys
if __name__=='__main__':
    f,t,x=geometry();q,labels=arrays(x[:3],t.edges,2);rows=[]
    for record in read(OUT/'RESIDUAL_LAYER2_EXACT.json'):
        if record['result']['method']!='NoncoincidentRank3ExactCAD':continue
        a,b=[kernel.Patch.binary(q[i],tuple(map(int,labels[i]))) for i in record['pair']]
        kernel.clear_kernel_caches();start=time.perf_counter();old=kernel.certificate.certify(a,b);oldtime=time.perf_counter()-start
        kernel.clear_kernel_caches();start=time.perf_counter()
        with memoized_root_keys() as keys:
            new=kernel.certificate.certify(a,b);info=keys.cache_info()
        newtime=time.perf_counter()-start
        assert old==new==record['result']
        row=dict(pair=record['pair'],old_seconds=oldtime,new_seconds=newtime,key_hits=info.hits,key_misses=info.misses,exact_records_identical=True)
        rows.append(row);print(row,flush=True)
    save('ROOT_KEY_PROBE.json',rows)
