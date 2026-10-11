"""One direct risk scan of the restored initial cheap forecast; no SDF rebuild."""
from task4c_common import *
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer,discover_regions
import cProfile,pstats,psutil

def run():
    started=time.perf_counter();front,top,_=geometry()
    with np.load(BASE/'INITIAL_FORECAST.npz') as z:x=z['coordinates']
    frozen=read(BASE/'INITIAL_SCAN.json');previous=None;low=front.coordinates;events=[];rows=[];times=[]
    for layer,high in enumerate(x,1):
        profile=cProfile.Profile() if layer==12 else None
        if profile:profile.enable()
        r=evaluate_layer(top,low,high,layer,previous)
        if profile:
            profile.disable();stats=pstats.Stats(profile)
            functions=[dict(file=f[0].replace(str(REPO),'<repo>'),line=f[1],name=f[2],primitive=v[0],calls=v[1],self_seconds=v[2],cumulative_seconds=v[3]) for f,v in stats.stats.items()]
            save('IMMEDIATE_METRIC_PROFILE.json',sorted(functions,key=lambda r:r['cumulative_seconds'],reverse=True)[:70])
        rec=dict(layer=layer,gate=r['gate'],events=r['events'])
        assert scientific_hash(rec)==scientific_hash(frozen['records'][layer-1]),layer
        rows.append(dict(layer=layer,record_hash=scientific_hash(rec),event_count=len(r['events'])))
        times.append(dict(layer=layer,seconds=r['seconds'],profiled=bool(profile)))
        events+=r['events'];previous=r['cells'];low=high
        print('INITIAL SCAN MATCH',layer,flush=True)
    # This calls the unchanged generic connected-risk discovery over direct events.
    regions=discover_regions(top,events)
    save('INITIAL_SCAN_REPLAY.json',dict(rows=rows,regions=[dict(vertices=r.vertex_ids,columns=r.triangle_ids,oversized=r.oversized) for r in regions],
        forecast_hash=array_hash(x),scientific_hash=scientific_hash(rows),all_initial_events_exact=True))
    save('INITIAL_SCAN_TIMINGS.json',dict(layers=times,total_seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset))

def active_profile():
    from types import SimpleNamespace
    from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
    from sdf_cfd_mesher_v4.advancing_front.rolling_correction import bounded_patch
    from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
    f,t,_=geometry();_,x,_=load_candidate(BASE/'LAYER8_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    # This is profiling of a preserved generic support, not a selection rule.
    ids=np.array(read(BASE/'LAYER8_CORRECTIONS.json')[0]['scope']['active']);p=bounded_patch(t,ids)
    profile=cProfile.Profile();tick=time.perf_counter();profile.enable()
    r=measure(f,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':x}),t,p.detailed);profile.disable()
    functions=[dict(name=k[2],file=k[0].replace(str(REPO),'<repo>'),calls=v[1],self_seconds=v[2],cumulative_seconds=v[3]) for k,v in pstats.Stats(profile).stats.items()]
    save('ACTIVE_METRIC_PROFILE.json',dict(seconds=time.perf_counter()-tick,counts=r['counts'],functions=sorted(functions,key=lambda r:r['cumulative_seconds'],reverse=True)[:60]))

if __name__=='__main__':run();active_profile()
