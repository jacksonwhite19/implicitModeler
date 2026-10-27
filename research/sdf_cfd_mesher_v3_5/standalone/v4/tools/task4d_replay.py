"""Independent bounded transactions over frozen checkpoints, never a build.

Each transaction starts from Build A. Persistence is carried between calls;
it does not publish coordinates, advance a commit store, or touch A2.
"""
from task4d_common import *
from types import SimpleNamespace
import psutil
import task4_prepare_next as runner
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner
from sdf_cfd_mesher_v4.advancing_front.rolling_state import RollingState
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate,save_candidate

def first(planner,path):
    """Replay the existing first-layer two bounded searches, not initialization."""
    f=runner.load_sf2_wall();top=runner.build_topology(f.triangles,len(f.coordinates));sdf=runner.NativeSdf();schedule=runner.load_schedule()
    with np.load(BASE/'INITIAL_FORECAST.npz') as z:coords=z['coordinates'];normals=z['normals']
    events=read(BASE/'INITIAL_SCAN.json')['records'][0]['events'];candidate=coords[0].copy()
    regions,field,traces=runner.controlling_support(f,top,candidate,events)
    specs=[dict(vertices=r.vertex_ids,name='immediate') for r in regions];planner.begin(1,specs,top.sha256)
    rows=[];outcomes=[];times=[]
    for ri,region in enumerate(regions):
        pt=time.perf_counter();p=planner.patch(top,np.asarray(region.vertex_ids));support_seconds=time.perf_counter()-pt;ids=p.active;fc=runner.RollingForecastCache(f,top,schedule,sdf,coords,normals)
        bt=time.perf_counter()
        nominal=candidate[ids].copy();before=planner.detailed.get(f,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords}),top,p.detailed)
        baseline_seconds=time.perf_counter()-bt
        prior=runner.future_failures(before,1);oldfirst=min(prior.values(),default=21);accepted=False;seen=set()
        for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
            if accepted:break
            for name,mode in runner.correction_fields(f,top,p,field):
                tick=time.perf_counter();proposed,projection=runner.try_tangent_proposal(nominal,mode,np.full(len(ids),factor*schedule.increments[0]),sdf,schedule.targets[0])
                if proposed is None:rows.append(dict(region=ri,mode=name,factor=factor,reason='ProjectionFailure'));continue
                projection_seconds=time.perf_counter()-tick;h=array_hash(proposed)
                if h in seen:continue
                seen.add(h);overlay=runner.CoordinateOverlay(coords,ids,np.concatenate((proposed[None],coords[1:,ids])))
                qt=time.perf_counter();quick=runner.current_layer_metrics(f,top,p.detailed,overlay[0]);gate=runner.immediate_numeric(quick,1);quick_seconds=time.perf_counter()-qt
                forecast_seconds=detail_seconds=exact_seconds=0.
                event=dict(region=ri,mode=name,factor=factor,coordinates_hash=h,gate=gate,displacement=float(np.linalg.norm(proposed-nominal,axis=1).max()),accepted=False)
                if gate['passed']:
                    ft=time.perf_counter();future=fc.forecast(runner.PrivateState(ids),proposed);forecast_seconds=time.perf_counter()-ft
                    dt=time.perf_counter();r=planner.detailed.get(f,future,top,p.detailed);detail_seconds=time.perf_counter()-dt
                    et=time.perf_counter();ex=runner.exact_layer(f,top,future,p.detailed,1);exact_seconds=time.perf_counter()-et
                    failures=runner.future_failures(r,1)
                    event.update(exact=ex,first_future_before=oldfirst,first_future_after=min(failures.values(),default=21),new_entity_failures=runner.no_new_earlier_failure(prior,failures),forecast_hash=array_hash(future.local['coordinates']))
                    if ex['passed'] and runner.future_admission(prior,failures)['passed']:
                        accepted=True;event['accepted']=True;coords[:,ids]=future.local['coordinates'];normals[:,ids]=future.local['normal'];candidate[ids]=proposed
                        planner.accepted(top,p,'immediate',name,factor,nominal,proposed)
                rows.append(event);times.append(dict(region=ri,seconds=time.perf_counter()-tick,support_seconds=support_seconds,baseline_detailed_seconds=baseline_seconds,projection_seconds=projection_seconds,quick_metric_seconds=quick_seconds,forecast_seconds=forecast_seconds,detailed_seconds=detail_seconds,exact_sign_seconds=exact_seconds))
                if accepted:break
        outcomes.append(dict(region=ri,patch=p.record(),corrected=accepted,trial_count=len(seen)))
    history=dict(history=rows,outcomes=outcomes,production_committed_layer=0)
    assert scientific_hash(history)==scientific_hash(read(BASE/'CORRECTION_HISTORY.json'))
    final=runner.evaluate_layer(top,f.coordinates,candidate,1)
    gate=dict(gate=final['gate'],events=final['events'],coordinate_hash=array_hash(candidate))
    assert scientific_hash(gate)==scientific_hash(read(BASE/'LAYER1_GATE.json'))
    old,x,n=load_candidate(BASE/'LAYER1_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    assert np.array_equal(coords.view(np.uint64),x.view(np.uint64)) and np.array_equal(normals.view(np.uint64),n.view(np.uint64))
    write(path/'LAYER1_SEARCH.json',history);planner.save(path/'LAYER1_PLANNER.npz',array_hash(coords))
    return dict(layer=1,decision_hash=scientific_hash(history),candidate_hash=array_hash(candidate),forecast_hash=array_hash(coords),normal_hash=array_hash(normals),gate_hash=scientific_hash(gate),warning_hash=scientific_hash(read(BASE/'LAYER1_WARNINGS.json')),equivalent=True,planner_events=planner.events.copy()),dict(stages=times)

def layer_replay(layer,planner,path,cold=False):
    m=read(BASE/f'build_A/LAYER_{layer-1:02}.json')
    class ReadOnlyStore:
        def __init__(self,*_):self.path=BASE/'build_A'
        def restart(self):
            x=np.load(self.path/f'FRONT_{layer-1:02}.npy');assert array_hash(x)==m['coordinate_hash']
            return RollingState(layer-1,x,m['topology_hash'],m['state_hash'],m['prism_hash'],m['side_hash'],m['certificate_hash'],m['forecast_key'])
        def commit(self,*_):raise AssertionError('NoCommitInPerformanceReplay')
    runner.OUT=path;runner.CommitStore=ReadOnlyStore
    runner.load_candidate=lambda p,b:load_candidate(BASE/p.name,BASE/b.name)
    runner.save_candidate=lambda p,c,x,n,b:save_candidate(p,c,x,n,BASE/b.name)
    runner.read=lambda p:read(BASE/p.name)
    assert runner.prepare(layer,planner=None if cold else planner,use_persistent=not cold)
    c,x,n=load_candidate(path/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    oc,ox,on=load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    for a,b in ((c,oc),(x,ox),(n,on)):assert np.array_equal(a.view(np.uint64),b.view(np.uint64)),('Coordinates',layer)
    hashes={}
    for key,suffix in (('decision','SEARCH'),('gate','GATE'),('warning','WARNINGS')):
        new=read(path/f'LAYER{layer}_{suffix}.json');old=read(BASE/f'LAYER{layer}_{suffix}.json')
        assert scientific_hash(new)==scientific_hash(old),(key,layer)
        hashes[key+'_hash']=scientific_hash(new)
    result=dict(layer=layer,**hashes,candidate_hash=array_hash(c),forecast_hash=array_hash(x),normal_hash=array_hash(n),equivalent=True,
                planner_events=[] if cold else [r for r in planner.events if r['layer']==layer])
    return result,read(path/f'LAYER{layer}_PREPARE_TIMINGS.json')

def run(name,start=1,stop=20,cold=False):
    path=OUT/name;path.mkdir(parents=True,exist_ok=True);planner=PersistentPlanner();rows=[];times=[]
    if start>1 and (path/f'LAYER{start-1}_PLANNER.npz').exists():
        m=read(BASE/f'build_A/LAYER_{start-1:02}.json');planner.load(path/f'LAYER{start-1}_PLANNER.npz',start-1,m['forecast_key'])
    started=time.perf_counter()
    for layer in range(start,stop+1):
        t=time.perf_counter();before=planner.summary()
        row,timing=first(planner,path) if layer==1 else layer_replay(layer,planner,path,cold)
        rows.append(row);times.append(dict(layer=layer,seconds=time.perf_counter()-t,stage=timing,before=before,after=planner.summary(),peak_rss=psutil.Process().memory_info().peak_wset))
        save(name+'_REPLAY.json',dict(rows=rows,scientific_hash=scientific_hash(rows),geometry_rebuilt=False,A2_resumed=False,production_commit_called=False))
        save(name+'_TIMINGS.json',dict(rows=times,seconds=time.perf_counter()-started))
        print('EQUIVALENT',name,layer,times[-1]['seconds'],planner.summary()['selection'],flush=True)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--start',type=int,default=1);p.add_argument('--stop',type=int,default=20);p.add_argument('--cold',action='store_true');a=p.parse_args();run(a.name,a.start,a.stop,a.cold)
