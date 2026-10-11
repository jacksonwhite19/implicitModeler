"""Compare saved trajectories outside the exact union of local invalidations."""
from task4_initial_scan import *
from task3c_io import read
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.runtime.regions import dependencies

def run(layer):
    started=time.perf_counter();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    _,before,bn=load_candidate(OUT/f'LAYER{layer-1}_CANDIDATE.npz',OUT/'INITIAL_FORECAST.npz')
    candidate,after,an=load_candidate(OUT/f'LAYER{layer}_CANDIDATE.npz',OUT/'INITIAL_FORECAST.npz')
    outcomes=read(OUT/f'LAYER{layer}_CORRECTIONS.json');ids=np.array(sorted({int(v) for o in outcomes if o.get('accepted') for v in o['scope']['active']}),dtype=np.int64)
    unchanged=np.ones(len(front.coordinates),bool);unchanged[ids]=False
    exact=lambda a,b:np.array_equal(np.ascontiguousarray(a).view(np.uint64),np.ascontiguousarray(b).view(np.uint64))
    assert exact(before[:,unchanged],after[:,unchanged]);assert exact(bn[:,unchanged],an[:,unchanged])
    assert exact(before[:layer-1],after[:layer-1]);assert exact(candidate,after[layer-1])
    _,edges,triangles=dependencies(top,ids)
    actual=np.flatnonzero(np.any(before.view(np.uint64)!=after.view(np.uint64),axis=(0,2)))
    assert set(actual)<=set(ids)
    evidence=dict(layer=layer,scope='saved uncommitted candidate and diagnostics; publication is separate',
        active_union_vertices=len(ids),actual_changed_vertices=len(actual),dependent_edges=len(edges),dependent_triangles=len(triangles),
        exact_retained_vertex_trajectories=int(unchanged.sum()),retained_coordinates_exact=True,retained_normals_exact=True,
        prior_committed_layers_exact=True,active_ids_sha256=array_hash(ids),before_sha256=array_hash(before),after_sha256=array_hash(after))
    write(OUT/f'LAYER{layer}_CACHE_AUDIT.json',evidence)
    (OUT/f'LAYER{layer}_CACHE_AUDIT_TIMINGS.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print(json.dumps(evidence,sort_keys=True),flush=True)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--layer',required=True,type=int);run(p.parse_args().layer)
