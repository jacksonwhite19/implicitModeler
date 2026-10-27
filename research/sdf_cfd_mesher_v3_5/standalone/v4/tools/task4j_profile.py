"""Bounded exact calls and native proof classes, before any optimization."""
from task4j_common import *
import gzip,cProfile,pstats,psutil
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from sdf_cfd_mesher_v4.certificate.incremental import kernel
from sdf_cfd_mesher_v4.certificate.efficient_root_keys import memoized_root_keys

def run():
    require_time();front,top,x=geometry();q,labels=surface_arrays(front.coordinates,top.edges,x[1:]);rows=[];profile=cProfile.Profile();native=Filter();classes=[]
    for layer in (2,12,20):
        with gzip.open(BASE/f'certificate_layer_{layer:02}/EXACT.json.gz','rt') as f:exact=json.load(f)
        methods=collections.defaultdict(list)
        for r in exact:methods[r['result']['method']].append(r)
        selected=[]
        for method,items in sorted(methods.items()):
            positions=sorted(set([0,len(items)//2,len(items)-1]));selected.extend(items[i] for i in positions)
        with memoized_root_keys() as keys:
            for index,row in enumerate(selected):
                require_time()
                if index%8==0:kernel.clear_kernel_caches()
                s=time.perf_counter();a,b=(kernel.Patch.binary(q[i],tuple(map(int,labels[i]))) for i in row['pair']);a.valid();b.valid();prep=time.perf_counter()-s
                start=time.perf_counter();profile.enable();result=kernel.certificate.certify(a,b);profile.disable();seconds=time.perf_counter()-start
                assert scientific_hash(result)==scientific_hash(row['result']),('FrozenProfileMismatch',row['pair'])
                enc=time.perf_counter();payload=canonical(result);encoding=time.perf_counter()-enc
                rows.append(dict(layer=layer,pair=row['pair'],method=result['method'],seconds=seconds,patch_seconds=prep,canonical_bytes=len(payload),canonical_seconds=encoding,root_keys=keys.cache_info()._asdict(),result_hash=scientific_hash(result)))
                save('BOUNDED_KERNEL_PROFILE.json',rows);print('PROFILE',layer,result['method'],seconds,flush=True)
        with np.load(BASE/f'certificate_layer_{layer:02}/COMPACT.npz') as z:packed=z['packed'];codes=z['codes'];w=z['witness']
        pairs=np.column_stack((packed>>np.uint64(32),packed&np.uint64(0xffffffff))).astype(np.uint32)
        for c in np.unique(codes):
            ix=np.flatnonzero(codes==c);subset=pairs[ix];out=np.empty(len(ix),np.uint8);witness=out.copy()
            start=time.perf_counter();native.dll.screen(q.ctypes.data,labels.ctypes.data,len(q),subset.ctypes.data,len(subset),out.ctypes.data,witness.ctypes.data);dt=time.perf_counter()-start
            assert np.array_equal(out,codes[ix]) and np.array_equal(witness,w[ix])
            classes.append(dict(layer=layer,code=int(c),pairs=len(ix),native_seconds=dt,mean_seconds=dt/len(ix),native_calls=1))
    stats=pstats.Stats(profile);records=[]
    for (path,line,name),(primitive,calls,self_time,total,callers) in stats.stats.items():
        records.append(dict(file=Path(path).name,line=line,function=name,primitive_calls=primitive,calls=calls,self_seconds=self_time,total_seconds=total))
    save('CALL_TREE.json',sorted(records,key=lambda x:-x['total_seconds'])[:160]);save('NATIVE_CLASS_PROFILE.json',classes)
    save('PROFILE_RESOURCE.json',dict(peak_rss=psutil.Process().memory_info().peak_wset,elapsed_task=elapsed(),profiled_calls=len(rows)))
if __name__=='__main__':run()
