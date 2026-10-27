"""Fresh Task-4 layer-1 correction, selected only from complete current scan."""
from task4_initial_scan import *
from types import SimpleNamespace
from sdf_cfd_mesher_v4.advancing_front.rolling_correction import controlling_support,bounded_patch,correction_fields
from sdf_cfd_mesher_v4.advancing_front.private_rolling import PrivateState,RollingForecastCache,try_tangent_proposal,immediate_numeric,exact_layer,future_failures
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import current_layer_metrics
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import CoordinateOverlay,no_new_earlier_failure
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import save_candidate
from task3c_io import read
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import future_admission
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner
from sdf_cfd_mesher_v4.advancing_front.staged_evaluation import immediate_numeric_screen,exact_current,staged_detail

def correct_first(context=None):
    OUT=context.output if context is not None else globals()['OUT']
    start=time.perf_counter()
    if context is None:
        front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));sdf=NativeSdf();schedule=load_schedule()
        store=CommitStore(OUT/'build_A',front,top,schedule);state=store.initialize()
        with np.load(OUT/'INITIAL_FORECAST.npz') as z:coords=z['coordinates'];normals=z['normals']
    else:
        front,top,sdf,schedule,state=context.front,context.top,context.sdf,context.schedule,context.current
        coords=context.forecast_coordinates.copy();normals=context.forecast_normals.copy()
    scan=read(OUT/'INITIAL_SCAN.json');events=scan['records'][0]['events'];candidate=coords[0].copy()
    regions,field,traces=controlling_support(front,top,candidate,events)
    write(OUT/'DISCOVERY.json',dict(regions=[dict(vertices=r.vertex_ids,columns=r.triangle_ids,oversized=r.oversized) for r in regions],corner_traces=traces,field=field[np.flatnonzero(np.any(field,axis=1))],field_vertices=np.flatnonzero(np.any(field,axis=1))))
    print('SUPPORTS',[(len(r.vertex_ids),len(r.triangle_ids)) for r in regions],flush=True)
    planner=PersistentPlanner();planner.begin(1,[dict(vertices=r.vertex_ids,name='immediate') for r in regions],top.sha256)
    history=[];outcomes=[];timings=[];dc=planner.detailed
    for ri,region in enumerate(regions):
        if region.oversized:raise RuntimeError('BoundedCorrectionSupportExceeded')
        patch=planner.patch(top,np.array(region.vertex_ids));ids=patch.active;fc=RollingForecastCache(front,top,schedule,sdf,coords,normals)
        nominal=candidate[ids].copy();baseline=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
        before=dc.get(front,baseline,top,patch.detailed);before_events=future_failures(before,1);old_first=min(before_events.values(),default=21)
        state_local=PrivateState(ids);accepted=None;seen=set()
        # Computational amplitude poll; source increments set units, not health.
        for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
            if accepted is not None:break
            for name,mode in correction_fields(front,top,patch,field):
                tick=time.perf_counter();proposed,projection=try_tangent_proposal(nominal,mode,np.full(len(ids),factor*schedule.increments[0]),sdf,schedule.targets[0])
                if proposed is None:history.append(dict(region=ri,mode=name,factor=factor,reason='ProjectionFailure'));continue
                h=array_hash(proposed)
                if h in seen:continue
                seen.add(h);overlay=CoordinateOverlay(coords,ids,np.concatenate((proposed[None],coords[1:,ids])))
                screen=immediate_numeric_screen(front,top,coords,ids,proposed,patch.detailed,1);gate=screen['gate']
                event=dict(region=ri,mode=name,factor=factor,coordinates_hash=h,gate=gate,displacement=float(np.linalg.norm(proposed-nominal,axis=1).max()),accepted=False)
                if gate['passed']:
                    ex=exact_current(top,front.coordinates,candidate,ids,proposed,patch.detailed,1)
                    if not ex['passed']:
                        event.update(exact=ex,rejection_stage='ImmediateExactJacobian',future_evaluated=False)
                        history.append(event);timings.append(dict(region=ri,mode=name,factor=factor,seconds=time.perf_counter()-tick,peak_rss=psutil.Process().memory_info().peak_wset));continue
                    forecast=staged_detail(screen,lambda:fc.forecast(state_local,proposed));detail=dc.get(front,forecast,top,patch.detailed)
                    # Surviving records keep all original diagnostic extrema.
                    gate=immediate_numeric(detail,1);event['gate']=gate
                    failures=future_failures(detail,1);newfirst=min(failures.values(),default=21)
                    event.update(exact=ex,first_future_before=old_first,first_future_after=newfirst,new_entity_failures=no_new_earlier_failure(before_events,failures),forecast_hash=array_hash(forecast.local['coordinates']))
                    if ex['passed'] and future_admission(before_events,failures)['passed']:
                        # This changes only the uncommitted complete candidate.
                        # Whole-front validation and exact certification follow.
                        event['accepted']=True;accepted=proposed;coords[:,ids]=forecast.local['coordinates'];normals[:,ids]=forecast.local['normal'];candidate[ids]=proposed
                        planner.accepted(top,patch,'immediate',name,factor,nominal,proposed)
                        np.savez_compressed(OUT/f'REGION_{ri:02}_CANDIDATE.npz',vertices=ids,coordinates=proposed,forecast=forecast.local['coordinates'])
                history.append(event);timings.append(dict(region=ri,mode=name,factor=factor,seconds=time.perf_counter()-tick,peak_rss=psutil.Process().memory_info().peak_wset))
                print('TRIAL',ri,name,factor,gate['violations'],event['accepted'],flush=True)
                if accepted is not None:break
        outcomes.append(dict(region=ri,patch=patch.record(),corrected=accepted is not None,trial_count=len(seen)))
        write(OUT/'CORRECTION_HISTORY.json',dict(history=history,outcomes=outcomes,production_committed_layer=state.layer))
    save_candidate(OUT/'LAYER1_CANDIDATE.npz',candidate,coords,normals,OUT/'INITIAL_FORECAST.npz')
    final=evaluate_layer(top,front.coordinates,candidate,1)
    write(OUT/'LAYER1_GATE.json',dict(gate=final['gate'],events=final['events'],coordinate_hash=array_hash(candidate)))
    planner.save(OUT/'LAYER1_PLANNER.npz',array_hash(coords))
    (OUT/'CORRECTION_TIMINGS.json').write_text(json.dumps(dict(stages=timings,total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print('FINAL',final['gate'],flush=True)

if __name__=='__main__':correct_first()
