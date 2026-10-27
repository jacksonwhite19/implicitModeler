"""Deterministic bounded layer-1 experiments, with no geometry commitment."""
from pathlib import Path
import os,sys,json,time,threading,collections
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
import numpy as np,psutil
from types import SimpleNamespace
from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
from sdf_cfd_mesher_v4.forecast.detailed.continuous import SideIndex
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import *
from sdf_cfd_mesher_v4.advancing_front.local_certificate import *
from forensics_intersection import reproduce,make_side
from task2_describe import read as oldread

OUT=ROOT/'results/task3'
CONFIG=dict(active_rings=[0,1],max_active_vertices=96,max_local_columns=256,radius_rounds=6,initial_radius=1.,max_accepted_steps=3,
            note='Computational development bounds, not scientific convergence or health thresholds. Radius uses preserved V3 geometric maximum; rejected polls halve it.')

def write(name,v):(OUT/name).write_text(json.dumps(plain(v),indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf-8')

def load():
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));sdf=NativeSdf();schedule=load_schedule();coords=[];normals=[]
    for first in range(1,21,5):
        with np.load(ROOT/f'results/task1/forecast_layers_{first:02}_{first+4:02}.npz') as z:coords.append(z['coordinates']);normals.append(z['normals'])
    coords=np.concatenate(coords);normals=np.concatenate(normals);coords.setflags(write=False);normals.setflags(write=False)
    baseline=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
    index=SideIndex(front,baseline,top);one=SideIndex(front,SimpleNamespace(vertex={'coordinates':coords[:1]}),top)
    x=np.concatenate((front.coordinates[None],coords));known=reproduce(front,top,x,sdf,schedule)
    assert known['classification']=='GenuineNominalForecastHazard'
    return front,top,sdf,schedule,coords,normals,index,one,known

def components():
    s=json.loads((ROOT/'results/nominal_forecast_forensics/FAILURE_COMPONENTS.json').read_text())
    metrics=['weight','jacobian','jacobian','determinant','jacobian']
    return [dict(name=r['component'],columns=r['columns'],metric=m,control=False,known_intersection=r['component']=='component_01') for r,m in zip(s,metrics)]+[
        dict(name='control_mid_chord',columns=[22890],metric='jacobian',control=True,known_intersection=False),
        dict(name='control_forward_surface',columns=[3497],metric='jacobian',control=True,known_intersection=False)]

def pair_evidence(front,top,future):
    patches=[]
    for edge,k in [(1,1),(729,10)]:
        a,b=top.edges[edge];lower=front.coordinates[[a,b]] if k==1 else future.vertex['coordinates'][k-2,[a,b]]
        upper=future.vertex['coordinates'][k-1,[a,b]];pts=np.concatenate((lower,upper));ids=((k-1,int(a)),(k-1,int(b)),(k,int(a)),(k,int(b)))
        patches.append(kernel.Patch.binary(pts,ids))
    return production_pair(CertificateSession(),*patches)

def scientific_history(history):
    for event in history:
        event.pop('seconds',None);event.pop('detailed_seconds',None)
        if 'cache' in event:
            for key in ['seconds','projection_seconds','descriptor_seconds']:event['cache'].pop(key,None)
    return history

def timed_work(history):
    return dict(forecast_seconds=sum(e.get('cache',{}).get('seconds',0.) for e in history),
                detailed_seconds=sum(e.get('detailed_seconds',0.) for e in history))

def minima(r):
    out=numeric_gates(r,candidate_only=True)
    return out

def run_component(spec,context,*,probe=False):
    front,top,sdf,schedule,coords,normals,fullindex,oneindex,_=context
    started=time.perf_counter();peak=psutil.Process().memory_info().rss
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);dc=DetailedCache();history=[];accepted=[];scopes=[];final=None
    original_cells={r['cell_id']:r for r in oldread('DIRECT_PRISM_ROWS.json')}
    # An unrelated control is a concrete detailed-cache hit, not an inferred count.
    unrelated_patch=build_patch(top,[22890 if not spec['control'] else 92793]).detailed
    unrelated=dc.get(front,fc.baseline,top,unrelated_patch)
    for rings in CONFIG['active_rings']:
        if spec['control'] and rings:break
        try:patch=build_patch(top,spec['columns'],active_rings=rings,max_vertices=CONFIG['max_active_vertices'],max_columns=CONFIG['max_local_columns'])
        except LocalityFailure as e:
            scopes.append(dict(active_rings=rings,reason=str(e)));break
        baseline=dc.get(front,fc.baseline,top,patch.detailed);baseg=minima(baseline);basefail=failure_times(baseline);basetarget=target_status(baseline,spec['columns'],spec['metric'])
        matches=0
        for k in range(20):
            for j,col in enumerate(patch.detailed.core_columns):
                cellid=k*len(top.triangles)+int(col)
                if cellid in original_cells:
                    for key,v in baseline['cells'].items():assert np.array_equal(v[k,j],original_cells[cellid][key],equal_nan=True),(cellid,key)
                    matches+=1
        scopes.append(dict(active_rings=rings,patch=patch.record(),baseline_candidate_gate=baseg,baseline_target=basetarget,baseline_first_future_invalid=min(basefail.values(),default=None),original_exact_cell_matches=matches))
        incumbent=coords[0,patch.active].copy();incfuture=fc.baseline;incresult=baseline;inctarget=basetarget
        final=dict(patch=patch.record(),active_rings=rings,baseline=baseline,baseline_tails=sorted_tails(baseline),candidate=baseline,
                   final_tails=sorted_tails(baseline),layer1_coordinates=incumbent,baseline_layer1_coordinates=incumbent.copy(),
                   classification='NoUsefulCorrectionFound',baseline_first_invalid=min(basefail.values(),default=None),final_first_invalid=min(basefail.values(),default=None),accepted=False)
        if spec['control']:
            assert not control_needs_move(baseg,basefail),'ControlHasForecastHardHazard'
            final.update(reason='NoTargetHazard; no movement requested',coordinates_identical=True)
            break
        radius=CONFIG['initial_radius'];scope_accepted=0
        for round_id in range(CONFIG['radius_rounds']):
            fields,radii=search_fields(front,top,patch,incfuture.vertex['coordinates'][0],schedule.targets[-1]);poll_accepted=False
            for field_name,field in fields:
                trial_start=time.perf_counter();event=dict(scope=rings,round=round_id,field=field_name,radius=radius,accepted=False)
                try:
                    trial,projection=project_tangent_move(incumbent,field,radius*radii,sdf,schedule.targets[0]);event['projection']=projection
                    if np.array_equal(trial,incumbent):event['reason']='ExactNoMove';history.append(event);continue
                    future=fc.forecast(patch.active,trial);cache_stats=fc.last_stats.copy();detail_start=time.perf_counter();r=dc.get(front,future,top,patch.detailed);detail_seconds=time.perf_counter()-detail_start
                    assert dc.get(front,future,top,unrelated_patch) is unrelated,'UnrelatedDetailedCacheChanged'
                    g=minima(r);target=target_status(r,spec['columns'],spec['metric']);times=failure_times(r);collateral=no_new_earlier_failure(basefail,times)
                    event.update(candidate_gate=g,target=target,new_earlier_failures=collateral,first_future_invalid=min(times.values(),default=None),
                                 coordinate_hash=array_hash(trial),candidate_displacement=float(np.linalg.norm(trial-coords[0,patch.active],axis=1).max()),
                                 cache=cache_stats,detailed_seconds=detail_seconds)
                    if spec['known_intersection']:event['known_pair']=pair_evidence(front,top,future)
                    if not g['passed']:event['reason']='CandidateNumericHardGate'
                    elif collateral:event['reason']='NewEarlierVerificationFailure'
                    elif not hazard_improves(inctarget,target) and not (not baseg['passed'] and target['removed']):event['reason']='NoTargetImprovement'
                    elif probe:event['reason']='ProbeFeasibleImprovement';event['potential']=True
                    else:
                        ex=exact_candidate_jacobians(front,top,future,patch.detailed);event['exact_jacobians']=ex
                        if not ex['passed']:event['reason']='CandidateExactSign'
                        else:
                            e=dependencies(top,patch.active)[1]
                            currentfuture=SimpleNamespace(vertex={'coordinates':future.vertex['coordinates'][:1]})
                            cs=certify_all(updated_index(oneindex,currentfuture,e),patch.detailed)
                            event['current_certificate']=cs
                            if not cs['complete'] or cs['unresolved']:raise RuntimeError('CurrentCertificateCannotClose')
                            if not certificate_passed(cs):event['reason']='CandidateContinuousHardGate'
                            else:
                                # Compare complete local future evidence only for a promising candidate.
                                bc=certify_all(fullindex,patch.detailed);cc=certify_all(updated_index(fullindex,future,e),patch.detailed)
                                event['baseline_future_certificate']=bc;event['future_certificate']=cc
                                if not bc['complete'] or not cc['complete'] or bc['unresolved'] or cc['unresolved']:raise RuntimeError('FutureCertificateCannotClose')
                                new=new_intersections(bc,cc);event['new_future_intersections']=new
                                if new:event['reason']='NewNearbyFutureIntersection'
                                elif cc['invalid_surfaces']:event['reason']='InvalidFutureSideImage'
                                elif spec['known_intersection'] and event['known_pair']['classification']=='CertifiedForbiddenIntersection':event['reason']='KnownIntersectionRemains'
                                else:
                                    event['accepted']=True;event['reason']='FeasibleLocalHazardImprovement';incumbent=choose_trial(incumbent,trial,True);incfuture=future;incresult=r;inctarget=target
                                    scope_accepted+=1;poll_accepted=True
                                    final.update(candidate=r,final_tails=sorted_tails(r),layer1_coordinates=trial.copy(),final_first_invalid=min(times.values(),default=None),accepted=True,
                                                 candidate_gate=g,current_certificate=cs,future_certificate=cc,exact_jacobians=ex,
                                                 projection=projection,future_changed_coordinates=future.local['coordinates'],baseline_future_certificate=bc,
                                                 classification='HazardResolved' if target['removed'] else 'HazardImproved')
                                    accepted.append(dict(history_index=len(history),coordinate_hash=array_hash(trial)))
                except (ValueError,RuntimeError,np.linalg.LinAlgError) as err:
                    event.update(reason=type(err).__name__+':'+str(err),projection_failed='Projection' in str(err))
                    if isinstance(err,CertificateStop):event['certificate_error']=err.record
                    if 'CertificateCannotClose' in str(err):
                        history.append(event);final.update(classification='LocalCorrectionBlocked',reason=str(err));timed=timed_work(history)
                        return dict(spec=spec,scopes=scopes,history=scientific_history(history),final=final,accepted_steps=accepted),dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().rss,forecast_hits=fc.hits,forecast_misses=fc.misses,detailed_hits=dc.hits,detailed_misses=dc.misses,**timed)
                event['seconds']=time.perf_counter()-trial_start;history.append(event);peak=max(peak,psutil.Process().memory_info().rss)
                if poll_accepted:break
            oldradius=radius;radius,action,why=adjust_trust(radius,poll_accepted,prediction_ratio=None)
            history.append(dict(trust_update=dict(before=oldradius,after=radius,action=action,reason=why),scope=rings,round=round_id))
            if poll_accepted and (inctarget['removed'] or scope_accepted>=CONFIG['max_accepted_steps']):break
        if final['accepted']:break
    assert np.array_equal(front.coordinates,load_sf2_wall().coordinates)
    performance=dict(seconds=time.perf_counter()-started,peak_rss=peak,forecast_hits=fc.hits,forecast_misses=fc.misses,detailed_hits=dc.hits,detailed_misses=dc.misses,**timed_work(history))
    # Operational fields do not enter scientific serialization.
    scientific_history(history)
    return dict(spec=spec,scopes=scopes,history=history,final=final,accepted_steps=accepted),performance


if __name__=='__main__':
    probe='--probe' in sys.argv;context=load();write('CONFIG.json',CONFIG);write('BASELINE_INTERSECTION.json',context[-1]);results=[];timings={}
    for spec in components():
        t=time.perf_counter();r,p=run_component(spec,context,probe=probe);results.append(r);timings[spec['name']]=p
        write(('PROBE_' if probe else '')+spec['name']+'.json',r);write('PROBE_TIMINGS.json' if probe else 'TIMINGS.json',timings)
        reasons=collections.Counter(e.get('reason') for e in r['history'] if 'reason' in e)
        print(spec['name'],r['final']['classification'],dict(reasons),p,flush=True)
        if r['final']['classification']=='LocalCorrectionBlocked':break
    write('PROBE_SUMMARY.json' if probe else 'SUMMARY.json',dict(results=[dict(name=r['spec']['name'],classification=r['final']['classification']) for r in results],scientific_hash=scientific_hash(results)))
