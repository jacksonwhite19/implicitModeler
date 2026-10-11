from task4b_common import *
import collections,psutil
from sdf_cfd_mesher_v4.certificate.efficient import Filter
def arrays(x,edges,layers):
    q=np.stack((x[:-1,edges[:,0]],x[:-1,edges[:,1]],x[1:,edges[:,0]],x[1:,edges[:,1]]),axis=2).reshape(-1,4,3)
    E=len(edges);nv=x.shape[1];labels=np.stack([np.stack((k*nv+edges[:,0],k*nv+edges[:,1],(k+1)*nv+edges[:,0],(k+1)*nv+edges[:,1]),axis=1) for k in range(layers)])
    return q,labels.reshape(-1,4).astype(np.uint64)
def run(layer=1):
    f,t,x=geometry();q,ids=arrays(x[:layer+1],t.edges,layer);native=Filter();start=time.perf_counter();forest=native.forest(q,len(t.edges));build=time.perf_counter()-start
    s=time.perf_counter();packed=forest.pairs(layer);enum=time.perf_counter()-s
    pairs=np.column_stack((packed>>np.uint64(32),packed&np.uint64(0xffffffff))).astype(np.uint32)
    s=time.perf_counter();codes,w=native.screen(q,ids,pairs);seconds=time.perf_counter()-s
    s=time.perf_counter();valid=native.valid(q[-len(t.edges):]);validsec=time.perf_counter()-s
    row=dict(layer=layer,pairs=len(pairs),build_seconds=build,enumeration_seconds=enum,filter_seconds=seconds,validity_seconds=validsec,validity_fallbacks=int((valid==0).sum()),codes=dict(collections.Counter(map(int,codes))),peak_rss=psutil.Process().memory_info().peak_wset)
    save(f'LAYER_{layer:02}_FILTER_PROBE.json',row)
    np.savez_compressed(OUT/f'LAYER_{layer:02}_PROBE.npz',packed=packed,codes=codes,witness=w)
    print(json.dumps(row),flush=True)
if __name__=='__main__':run(int(sys.argv[1]) if len(sys.argv)>1 else 1)
