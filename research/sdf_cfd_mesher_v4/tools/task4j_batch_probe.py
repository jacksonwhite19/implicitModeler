from task4j_common import *
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from sdf_cfd_mesher_v4.certificate.parallel_screen import screen
import psutil
def run():
    f,t,x=geometry();q,labels=surface_arrays(f.coordinates,t.edges,x[1:]);native=Filter();rows=[]
    for layer in (2,12,20):
        with np.load(BASE/f'certificate_layer_{layer:02d}/COMPACT.npz') as z:p=z['packed'];codes=z['codes'];witness=z['witness']
        pairs=np.column_stack((p>>np.uint64(32),p&np.uint64(0xffffffff))).astype(np.uint32)
        for workers in (1,2,4,8):
            require_time();s=time.perf_counter();a,b=screen(native,q,labels,pairs,workers);dt=time.perf_counter()-s
            assert np.array_equal(a,codes) and np.array_equal(b,witness)
            rows.append(dict(layer=layer,workers=workers,pairs=len(p),seconds=dt,code_hash=array_hash(a),witness_hash=array_hash(b),peak_rss=psutil.Process().memory_info().peak_wset));save('BATCH_EXPERIMENT.json',rows);print(rows[-1],flush=True)
if __name__=='__main__':run()
