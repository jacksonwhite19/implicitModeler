from task4b_common import *
import gzip,collections,psutil
from types import SimpleNamespace
from sdf_cfd_mesher_v4.certificate.rolling import owned_scope
from sdf_cfd_mesher_v4.certificate.incremental import kernel
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from sdf_cfd_mesher_v4.forecast.detailed.continuous import SideIndex
def encoded_labels(scope,nv):return np.array([[k*nv+v for k,v in row] for row in scope.labels],np.uint64)
def run():
    f,t,x=geometry();native=Filter();results=[]
    for sample in json.loads((OUT/'BASELINE.json').read_text())['profiles']:
        k=sample['layer'];index=SideIndex(f,SimpleNamespace(vertex={'coordinates':x[1:k+1]}),t);scope=owned_scope(index,k,sample['begin'],sample['end']);pairs,_=scope.pairs()
        records=[json.loads(line) for line in gzip.open(OUT/f"profile_{sample['name']}.jsonl.gz",'rt')]
        start=time.perf_counter();valid=native.valid(scope.quads);codes,w=native.screen(scope.quads,encoded_labels(scope,len(f.coordinates)),pairs);seconds=time.perf_counter()-start
        for i in np.flatnonzero(valid==0):assert scope.patch(i).valid()[0]
        mismatch=[];fallback=[];kernel.clear_kernel_caches();ts=time.perf_counter()
        for i,((a,b),old) in enumerate(zip(pairs,records)):
            if codes[i]==0:r=kernel.certificate.certify(scope.patch(a),scope.patch(b));fallback.append(dict(pair=i,result=r));status=r['classification']
            else:status='CertifiedSeparated' if codes[i]<=2 else 'CertifiedAllowedIncidentContact'
            if status!=old['classification']:mismatch.append(dict(pair=i,old=old,status=status))
        row=dict(name=sample['name'],pairs=len(pairs),code_counts=dict(collections.Counter(map(int,codes))),mismatches=mismatch,
            validity_fallbacks=int((valid==0).sum()),filter_seconds=seconds,remaining_exact_seconds=time.perf_counter()-ts,
            codes_hash=array_hash(codes),witness_hash=array_hash(w),fallbacks=fallback,peak_rss=psutil.Process().memory_info().peak_wset)
        results.append(row);print(row['name'],row['code_counts'],'seconds',seconds,row['remaining_exact_seconds'],'mismatch',len(mismatch),flush=True)
        assert not mismatch
    save('LOCAL_EQUIVALENCE.json',dict(native_source=native.source_hash,scopes=results))
if __name__=='__main__':run()
