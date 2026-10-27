"""Bounded read-only audit of the published prefix and frozen inputs."""
from task4_initial_scan import *
from task4_preflight import audit,git
if OUT.is_relative_to((ROOT/'results/task4i').resolve()):
    from task4i_authority import audit
if OUT.is_relative_to((ROOT/'results/task4l').resolve()):
    from task4l_authority import audit
from task3c_io import read
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore
from sdf_cfd_mesher_v4.forecast.detailed.rolling_spots import selected_pairs

def run():
    started=time.perf_counter();authority=audit();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule()
    state=CommitStore(OUT/'build_A',front,top,schedule).restart()
    counts=np.diff(top.edge_triangles.offsets);assert np.all(counts==2)
    directed=top.triangles[:,[[0,1],[1,2],[2,0]]].reshape(-1,2)
    signs=np.where(directed[:,0]<directed[:,1],1,-1)
    balance=np.bincount(top.triangle_edges.ravel(),weights=signs,minlength=len(top.edges));assert np.all(balance==0)
    assert len(np.unique(np.sort(top.triangles,axis=1),axis=0))==len(top.triangles)
    assert len(np.unique(top.triangles))==len(front.coordinates)
    assert len(set(front.vertex_ids))==len(front.coordinates) and all(front.source_ancestry)
    assert front.coordinate_hash=='3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94'
    assert top.sha256=='69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7'
    sdf=NativeSdf();q=sdf.evaluate(state.coordinates);target=0. if state.layer==0 else schedule.targets[state.layer-1]
    assert np.all(abs(q.phi-target)<=q.source_error_bound)
    pre=json.loads((OUT/'PREFLIGHT.json').read_text())
    def outside(status):return [line for line in status.splitlines() if 'research/sdf_cfd_mesher_v4/' not in line]
    now=git('status','--porcelain');old=outside(pre['starting_status']);new=outside(now)
    result=dict(authorities=authority,published_layer=state.layer,published_state_hash=state.state_hash,
        coordinate_hash=array_hash(state.coordinates),topology_hash=top.sha256,wall_vertices=len(front.coordinates),
        edges=len(top.edges),columns=len(top.triangles),two_owners_per_edge=True,opposite_edge_orientation=True,
        duplicate_triangles=0,unused_vertices=0,unique_source_ids=True,source_ancestry_present=True,
        schedule_hash=array_hash(schedule.targets),maximum_sdf_residual=float(abs(q.phi-target).max()),
        unrelated_status_unchanged=old==new,unrelated_status_added=sorted(set(new)-set(old)),unrelated_status_removed=sorted(set(old)-set(new)))
    write(OUT/f'VERIFICATION_LAYER_{state.layer:02}.json',result)
    (OUT/f'VERIFICATION_LAYER_{state.layer:02}_TIMINGS.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print('VERIFIED',state.layer,state.state_hash,'unrelated_unchanged',old==new,flush=True)
    return result

if __name__=='__main__':run()
