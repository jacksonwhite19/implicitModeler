"""Matched fresh-process measurement of full certificate-array hashing."""
import sys,json,time,hashlib
from pathlib import Path
import numpy as np
from native_memory_observer import Observer
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.common.authority import array_hash
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native_runner_memory_profile'
def reference_surface_arrays(wall,edges,prefix):
    """Frozen pre-task allocation layout, only for matched memory reproduction."""
    x=np.concatenate((wall[None],np.asarray(prefix)))
    q=np.stack((x[:-1,edges[:,0]],x[:-1,edges[:,1]],x[1:,edges[:,0]],x[1:,edges[:,1]]),axis=2).reshape(-1,4,3)
    nv=len(wall)
    labels=np.stack([np.stack((k*nv+edges[:,0],k*nv+edges[:,1],(k+1)*nv+edges[:,0],(k+1)*nv+edges[:,1]),axis=1) for k in range(len(prefix))])
    return q,labels.reshape(-1,4).astype(np.uint64)

def run():
    mode=sys.argv[1];observer=Observer(OUT/('copy_'+mode))
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    prefix=[np.load(ROOT/f'results/task4l/fresh/build_A/FRONT_{k:02}.npy',mmap_mode='r') for k in range(1,21)]
    build=surface_arrays if mode=='bounded' else reference_surface_arrays
    q,labels=build(front.coordinates,top.edges,prefix);observer.mark('array_inputs',dict(q=q,labels=labels))
    observer.stack=['array_hash'];start=time.perf_counter()
    digest=(lambda a:hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()) if mode=='before' else array_hash
    hashes=[digest(q),digest(labels)];seconds=time.perf_counter()-start;observer.mark('hash_complete')
    peak=observer.finish();(OUT/f'COPY_{mode.upper()}.json').write_text(json.dumps(dict(hashes=hashes,seconds=seconds,peak=peak,array_bytes=q.nbytes+labels.nbytes),indent=2)+'\n')
if __name__=='__main__':run()
