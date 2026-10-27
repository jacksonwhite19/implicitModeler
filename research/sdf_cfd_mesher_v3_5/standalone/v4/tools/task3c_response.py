"""Verify changed-base forecasts and warm locality, without advancing a layer."""
from task3_repair_study import *
from sdf_cfd_mesher_v4.forecast.engine import vertex_trajectories
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import direction_coverage
from task3c_io import read as read_evidence
OUT=ROOT/'results/task3c'

def run(replay=False):
    start=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,*_=ctx;rows=[];timings={}
    for name in ('component_00','component_01','component_02','component_04'):
        beg=time.perf_counter();r=read_evidence(OUT/f'{name}_closed_support.json');ev=next(e for e in r['history'] if e.get('potential'))
        ids=np.array(r['final']['patch']['active']);trial=np.array(ev['projection']['projected'])
        again=predict_next_front(np.array(ev['projection']['pre_projection']),schedule.targets[0],sdf)
        assert np.array_equal(again.coordinates,trial)
        fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);a=fc.forecast(ids,trial);cold=fc.last_stats.copy()
        assert fc.forecast(ids,trial) is a;warm=fc.last_stats.copy()
        changed=np.any(trial!=coords[0,ids],axis=1);b=fc.forecast(ids[changed],trial[changed])
        assert np.array_equal(a.vertex['coordinates'][:,ids],b.vertex['coordinates'][:,ids])
        origin_hashes={}
        # Read-only forecast associativity. These are not accepted fronts or
        # steering steps; coordinates come from the same provisional path.
        selected=np.arange(min(4,len(ids)))
        for layer in (1,2,5):
            base=a.local['coordinates'][layer-1,selected]
            origin=SimpleNamespace(committed_layer=layer,coordinates=base,target_distances=np.full(len(base),schedule.targets[layer-1]))
            f,_=vertex_trajectories(origin,schedule,sdf,np.arange(len(base)))
            assert np.array_equal(f['coordinates'],a.local['coordinates'][layer:,selected])
            origin_hashes[layer]=array_hash(f['coordinates'])
        q=sdf.evaluate(a.local['coordinates'].reshape(-1,3))
        assert np.array_equal(q.normal,q.gradient/q.gradient_norm[:,None])
        assert np.all(abs(q.phi-np.repeat(schedule.targets,len(ids)))<=q.source_error_bound)
        for stats in (cold,warm):
            for key in ('seconds','projection_seconds','descriptor_seconds'):stats.pop(key,None)
        rows.append(dict(component=name,projection_replay_exact=True,full_vs_changed_overlay_exact=True,
            immutable_front_hash=array_hash(front.coordinates),committed_layer=front.committed_layer,
            private_advancing_steps=0,read_only_origin_hashes=origin_hashes,forecast_key=a.key,
            forecast_scientific_hash=scientific_hash(a.local),cold=cold,warm=warm,normalized_gradient_exact=True,
            target_residual_max=float(abs(q.phi-np.repeat(schedule.targets,len(ids))).max())))
        timings[name]=dict(seconds=time.perf_counter()-beg,peak_rss=psutil.Process().memory_info().peak_wset)
    h=scientific_hash(rows)
    if replay:assert scientific_hash(json.loads((OUT/'FORECAST_RESPONSE.json').read_text()))==h
    else:(OUT/'FORECAST_RESPONSE.json').write_text(json.dumps(plain(rows),indent=2,sort_keys=True)+'\n')
    (OUT/('FORECAST_RESPONSE_REPLAY.json' if replay else 'FORECAST_RESPONSE_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('FORECAST_RESPONSE_REPLAY_TIMINGS.json' if replay else 'FORECAST_RESPONSE_TIMINGS.json')).write_text(json.dumps(timings|dict(total_seconds=time.perf_counter()-start),indent=2,sort_keys=True)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
