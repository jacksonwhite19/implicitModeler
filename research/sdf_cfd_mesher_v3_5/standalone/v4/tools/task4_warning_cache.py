"""Initialize changed-region direct warnings without another whole-shell scan."""
from task4_correct_first import *
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import replace_local_events
from sdf_cfd_mesher_v4.forecast.detailed.rolling_spots import compare_spots
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.advancing_front.risk_atlas import RiskAtlas
from sdf_cfd_mesher_v4.advancing_front.production_risk import select_supports

def initialize_warnings(context=None):
    OUT=context.output if context is not None else globals()['OUT']
    start=time.perf_counter()
    front=load_sf2_wall() if context is None else context.front
    top=build_topology(front.triangles,len(front.coordinates)) if context is None else context.top
    _,coords,normals=load_candidate(OUT/'LAYER1_CANDIDATE.npz',OUT/'INITIAL_FORECAST.npz')
    with np.load(OUT/'INITIAL_FORECAST.npz') as z:base=z['coordinates'];base_normals=z['normals']
    schedule=load_schedule() if context is None else context.schedule
    sdf=NativeSdf() if context is None else context.sdf
    atlas=RiskAtlas(front,top,schedule,sdf.source_identity,base,base_normals)
    planner=PersistentPlanner();planner.load(OUT/'LAYER1_PLANNER.npz',1,array_hash(coords))
    initial=read(OUT/'INITIAL_SCAN.json');all_events=[e for r in initial['records'] for e in r['events']]
    specs,_=select_supports(front.coordinates,top,base[0],initial['records'][0]['events'],all_events,1)
    atlas.activate(specs,all_events,1)
    baseline=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':base});spots=[]
    future=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
    history=read(OUT/'CORRECTION_HISTORY.json');events=[e for r in read(OUT/'INITIAL_SCAN.json')['records'] for e in r['events']]
    for outcome in history['outcomes']:
        if not outcome['corrected']:continue
        p=bounded_patch(top,np.array(outcome['patch']['active']));r=planner.detailed.get(front,future,top,p.detailed);events=replace_local_events(events,p.detailed,r)
        spot=compare_spots(front,top,baseline,future,p.detailed,1,planner=planner.spots);spots.append(spot);events+=spot['warnings']
    accepted=[np.asarray(r['patch']['active'],np.int64) for r in history['outcomes'] if r['corrected']]
    atlas.adopt_evaluated(1,coords,normals,[e for e in events if e['layer']>1],accepted)
    atlas.save(OUT/'LAYER1_ATLAS.npz')
    planner.save(OUT/'LAYER1_PLANNER.npz',array_hash(coords))
    write(OUT/'LAYER1_WARNINGS.json',[e for e in events if e['layer']>1])
    write(OUT/'LAYER1_FUTURE_SPOTS.json',spots)
    (OUT/'WARNING_INIT_TIMINGS.json').write_text(json.dumps(dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print('Warnings',len(events),min((e['layer'] for e in events),default=None))

if __name__=='__main__':initialize_warnings()
