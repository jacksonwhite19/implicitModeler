"""Real saved-state restart and one complete cold metric comparison."""
from task4d_common import *
from task4d_replay import layer_replay
from types import SimpleNamespace
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.forecast.detailed.patch import patch_key
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure

def run():
    started=time.perf_counter();f,t,x=geometry();_,coords,_=load_candidate(BASE/'LAYER9_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    planner=PersistentPlanner();planner.load(OUT/'sequence_2/LAYER9_PLANNER.npz',9,array_hash(coords));checks=[]
    forecast=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
    for patch in planner.supports.values():
        key=patch_key(f,forecast,t,patch.detailed)
        if key not in planner.detailed.entries:continue
        cold=measure(f,forecast,t,patch.detailed);a=scientific_hash(cold);b=scientific_hash(planner.detailed.entries[key]);assert a==b
        checks.append(dict(key=key,cold=a,cached=b,counts=cold['counts']))
    assert checks
    path=OUT/'restart_10';path.mkdir(exist_ok=True);row,timing=layer_replay(10,planner,path)
    original=next(r for r in read(OUT/'sequence_2_REPLAY.json')['rows'] if r['layer']==10)
    assert row==original
    save('RESTART.json',dict(metric_checks=checks,transaction=row,scientific_hash=scientific_hash(row),passed=True))
    save('RESTART_TIMINGS.json',dict(seconds=time.perf_counter()-started,prepare=timing,reuse=planner.summary()))
    print('RESTART AND COLD METRICS EXACT',len(checks),flush=True)

if __name__=='__main__':run()
