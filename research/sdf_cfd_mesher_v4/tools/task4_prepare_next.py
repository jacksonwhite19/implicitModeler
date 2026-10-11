"""Prepare one full next front from a certified checkpoint; no commit here."""
from task4_correct_first import *
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_cells,replace_local_events,first_events,detailed_events,mechanism_tail,mechanism_improves,future_admission
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate,save_candidate
from sdf_cfd_mesher_v4.runtime.regions import expand_vertices,dependencies
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import search_fields
from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
from sdf_cfd_mesher_v4.advancing_front.risk_atlas import RiskAtlas
from sdf_cfd_mesher_v4.advancing_front.production_risk import select_supports
from sdf_cfd_mesher_v4.forecast.detailed.rolling_spots import compare_spots,pair_family_improves
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.staged_evaluation import immediate_cell_screen,immediate_numeric_screen,exact_current,staged_detail

def prepare(layer,*,use_atlas=True,planner=None,use_persistent=True,context=None):
    OUT=context.output if context is not None else globals()['OUT']
    start=time.perf_counter()
    if context is None:
        front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));sdf=NativeSdf();schedule=load_schedule()
        store=CommitStore(OUT/'build_A',front,top,schedule);state=store.restart()
        _,coords,normals=load_candidate(OUT/f'LAYER{layer-1}_CANDIDATE.npz',OUT/'INITIAL_FORECAST.npz')
        prefix=tuple(np.load(store.path/f'FRONT_{k:02}.npy') for k in range(1,layer))
    else:
        front,top,sdf,schedule,state=context.front,context.top,context.sdf,context.schedule,context.current
        coords=context.forecast_coordinates.copy();normals=context.forecast_normals.copy();prefix=context.prefix
    assert state.layer==layer-1
    assert np.array_equal(coords[:layer-1],np.stack(prefix));candidate=coords[layer-1].copy()
    if use_persistent and planner is None:
        planner=PersistentPlanner();snapshot=OUT/f'LAYER{layer-1}_PLANNER.npz'
        if snapshot.exists():planner.load(snapshot,layer-1,state.forecast_key)
    atlas=(context.atlas if context is not None else RiskAtlas(front,top,schedule,sdf.source_identity,coords,normals)) if use_atlas else None
    if atlas is not None:
        atlas.require_committed_forecast(state.forecast_key)
        snapshot=OUT/f'LAYER{layer-1}_ATLAS.npz'
        if context is not None:pass # native checkpoint already restored the atlas
        elif snapshot.exists():atlas.load(snapshot)
        else:
            # Legacy checkpoint migration is authority-bound by the committed
            # forecast key, topology, frozen schedule, and exact prefix.
            atlas.layer=layer-1
        candidate=atlas.nominal(layer,state.coordinates);nominal_before=candidate.copy()
        fresh=SimpleNamespace(projection_seconds=0.)
    else:
        fresh=predict_next_front(state.coordinates,schedule.targets[layer-1],sdf)
        assert np.array_equal(fresh.coordinates,candidate),'NominalRebaseMismatch'
        nominal_before=candidate.copy()
    low=front.coordinates if layer==2 else prefix[-2]
    previous=evaluate_cells(top,low,state.coordinates,layer-1)
    nominal=evaluate_layer(top,state.coordinates,candidate,layer,previous)
    risk_started=time.perf_counter()
    events=read(OUT/f'LAYER{layer-1}_WARNINGS.json');history=[];outcomes=[];timings=[];dc=DetailedCache() if planner is None else planner.detailed;patches=[]
    current_front=SimpleNamespace(coordinates=state.coordinates)
    saved=read(OUT/'CORRECTION_HISTORY.json')['outcomes']
    retained=[np.asarray(r['patch']['active']) for r in saved if r['corrected']]
    for k in range(2,layer):
        for old in read(OUT/f'LAYER{k}_CORRECTIONS.json'):
            if old.get('accepted') and 'scope' in old:retained.append(np.asarray(old['scope']['active']))
    patches,traces=select_supports(state.coordinates,top,candidate,nominal['events'],events,layer,retained)
    if planner is not None:planner.begin(layer,patches,top.sha256)
    if atlas is not None:
        if not atlas.retained:atlas.retained=[x.copy() for x in retained]
        atlas_partition=atlas.activate(patches,events+nominal['events'],layer)
    risk_discovery_seconds=time.perf_counter()-risk_started
    search_started=time.perf_counter()
    region_profiles=[]
    for pi,spec in enumerate(patches):
        region_started=time.perf_counter();region_profile=dict(patch=pi,support_seconds=0.,baseline_detailed_seconds=0.,direction_seconds=0.,checkpoint_seconds=0.)
        accepted=False
        # Patch expansion only follows measured current failures. Optional
        # future guidance cannot force an arbitrary large coupled domain.
        for rings in (0,1,2) if spec['required'] else (0,):
            ids=expand_vertices(top,spec['vertices'],rings)
            support_started=time.perf_counter()
            try:p=bounded_patch(top,ids) if planner is None else planner.patch(top,ids)
            except RuntimeError as error:outcomes.append(dict(patch=pi,reason=str(error),rings=rings));break
            region_profile['support_seconds']+=time.perf_counter()-support_started
            fc=RollingForecastCache(front,top,schedule,sdf,coords,normals);private=PrivateState(p.active,tuple(x[p.active] for x in prefix));nominal_local=candidate[p.active].copy()
            baseline_started=time.perf_counter()
            baseline_hits=dc.hits
            before=dc.get(front,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords}),top,p.detailed)
            region_profile['baseline_detailed_seconds']+=time.perf_counter()-baseline_started
            region_profile['baseline_cache_hit']=dc.hits>baseline_hits;region_profile['baseline_counts']=before['counts']
            baseline_fail=future_failures(before,layer);first_before=min(baseline_fail.values(),default=21)
            mechanism=min(baseline_fail,key=lambda k:(baseline_fail[k],k)) if baseline_fail else None
            known=[e for e in events if e['kind']=='pair' and set(e['columns'])&set(map(int,p.detailed.core_columns))]
            target_pair=min((e for e in known if e['metric']=='CertifiedForbiddenIntersection'),key=lambda e:(e['layer'],str(e['sides'])),default=None)
            pair_priority=target_pair is not None and (mechanism is None or target_pair['layer']<first_before)
            baseline_tail=mechanism_tail(before,mechanism,layer) if mechanism else None
            direction_started=time.perf_counter();modes=correction_fields(current_front,top,p,spec['field'])
            extra,_=search_fields(current_front,top,p,candidate,schedule.targets[-1]);modes.extend(extra)
            if planner is not None:
                hazard=identity(dict(mechanism=mechanism,pair=target_pair))
                advice=planner.advisory(top,p,nominal_local,sdf,hazard=hazard)
                # Advisory transport is recorded, never allowed to skip earlier
                # canonical trials or replace the scientific acceptance rule.
                if advice is not None:planner.stats['canonical_order_preserved']+=1
            region_profile['direction_seconds']+=time.perf_counter()-direction_started
            seen=set();potential=[]
            for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
                if accepted:break
                for name,mode in modes:
                    tick=time.perf_counter();proposed,projection=try_tangent_proposal(nominal_local,mode,np.full(len(p.active),factor*schedule.increments[layer-1]),sdf,schedule.targets[layer-1]);projection_seconds=time.perf_counter()-tick
                    if proposed is None:history.append(dict(patch=pi,rings=rings,factor=factor,mode=name,reason='ProjectionFailure'));continue
                    h=array_hash(proposed)
                    if h in seen:continue
                    seen.add(h)
                    screening_started=time.perf_counter()
                    screen=immediate_cell_screen(top,state.coordinates,candidate,p.active,proposed,p.detailed,layer)
                    if not screen['rejected']:screen=immediate_numeric_screen(front,top,coords,p.active,proposed,p.detailed,layer)
                    pre_exact=None;pre_exact_seconds=0.
                    if not screen['rejected']:
                        et=time.perf_counter();pre_exact=exact_current(top,state.coordinates,candidate,p.active,proposed,p.detailed,layer);pre_exact_seconds=time.perf_counter()-et
                    screen_seconds=time.perf_counter()-screening_started
                    if screen['rejected'] or not pre_exact['passed']:
                        event=dict(patch=pi,rings=rings,factor=factor,mode=name,hash=h,gate=screen['gate'],accepted=False,
                            rejection_stage=screen['stage'] if screen['rejected'] else 'ImmediateExactJacobian',exact=pre_exact,
                            future_evaluated=False,conditioning_evaluated=False)
                        history.append(event)
                        timings.append(dict(patch=pi,rings=rings,mode=name,factor=factor,seconds=time.perf_counter()-tick,
                            proposal_projection_seconds=projection_seconds,screen_seconds=time.perf_counter()-screening_started,
                            forecast=dict(seconds=0.,projection_seconds=0.),detailed_seconds=0.,detailed_cache_hit=False,
                            detailed_counts=dict(prisms=0,faces=0,reconstruction_rows=0),exact_sign_seconds=pre_exact_seconds,spot_seconds=0.,
                            peak_rss=psutil.Process().memory_info().peak_wset))
                        print('EARLY_REJECT',layer,pi,name,event['rejection_stage'],flush=True)
                        continue
                    # Recompute only this bounded patch, never a cold shell.
                    forecast=staged_detail(screen,lambda:fc.forecast(private,proposed));ft=dict(fc.last_stats);dt=time.perf_counter();dh=dc.hits;r=dc.get(front,forecast,top,p.detailed);detailed_seconds=time.perf_counter()-dt;detailed_hit=dc.hits>dh;gate=immediate_numeric(r,layer)
                    event=dict(patch=pi,rings=rings,factor=factor,mode=name,hash=h,gate=gate,accepted=False,cache=fc.last_stats.copy())
                    event['cache'].pop('seconds',None);event['cache'].pop('projection_seconds',None)
                    exact_seconds=spot_seconds=0.
                    if gate['passed']:
                        et=time.perf_counter()
                        ex=pre_exact;after_fail=future_failures(r,layer);first_after=min(after_fail.values(),default=21)
                        exact_seconds=pre_exact_seconds+time.perf_counter()-et
                        admission=future_admission(baseline_fail,after_fail);earlier=admission['new_earlier']
                        # Keep verification-ring failures explicit; an earlier
                        # minimum alone must not hide damage outside support.
                        ring=set(map(int,p.verification_ring));ring_earlier=[]
                        for key,value in earlier.items():
                            kind,entity,metric=key.split(':');owners=set(map(int,top.edge_triangles.row(int(entity)))) if kind=='side' else {int(entity)}
                            if owners&ring:ring_earlier.append(key)
                        event.update(exact=ex,first_before=first_before,first_after=first_after,new_earlier=earlier,ring_earlier=ring_earlier)
                        candidate_tail=mechanism_tail(r,mechanism,layer) if mechanism else None
                        improves=mechanism_improves(baseline_tail,candidate_tail) if mechanism else False
                        event.update(target_mechanism=mechanism,baseline_mechanism=baseline_tail,candidate_mechanism=candidate_tail)
                        # Required current repair can maintain its diagnostic
                        # future prefix; optional motion needs a direct gain.
                        if ex['passed'] and admission['passed'] and (spec['required'] or improves or pair_priority):
                            st=time.perf_counter()
                            spot=compare_spots(front,top,SimpleNamespace(vertex={'coordinates':coords}),forecast,p.detailed,layer,known,planner=None if planner is None else planner.spots)
                            spot_seconds=time.perf_counter()-st
                            event['future_spots']=spot
                            target_improves=improves
                            if pair_priority:
                                comparison=pair_family_improves(spot['records'],[edge for k,edge in target_pair['sides']]);event['target_pair_comparison']=comparison
                                target_improves=comparison['improved']
                            if not spot['new_earlier_pair_families'] and (spec['required'] or target_improves):
                                accepted=True;event['accepted']=True;coords[:,p.active]=forecast.local['coordinates'];normals[:,p.active]=forecast.local['normal'];candidate[p.active]=proposed
                                events=replace_local_events(events,p.detailed,r)+spot['warnings']
                                event['displacement']=float(np.linalg.norm(proposed-nominal_local,axis=1).max())
                                if planner is not None:planner.accepted(top,p,spec['name'],name,factor,nominal_local,proposed,hazard=hazard)
                    history.append(event);timings.append(dict(patch=pi,rings=rings,mode=name,factor=factor,seconds=time.perf_counter()-tick,proposal_projection_seconds=projection_seconds,screen_seconds=screen_seconds,forecast=ft,detailed_seconds=detailed_seconds,detailed_cache_hit=detailed_hit,detailed_counts=r['counts'],dependency_metrics={k:v for k,v in dc.last_stats.items() if k!='changed_cell_ids'},exact_sign_seconds=exact_seconds,spot_seconds=spot_seconds,peak_rss=psutil.Process().memory_info().peak_wset))
                    print('POLL',layer,pi,rings,factor,name,gate['violations'],event['accepted'],flush=True)
                    if planner is None:
                        if len(dc.entries)>6:dc.entries.clear()
                    else:
                        while len(dc.entries)>6:dc.entries.pop(next(iter(dc.entries)))
                    if accepted:break
            outcomes.append(dict(patch=pi,rings=rings,kind=spec['name'],required=spec['required'],scope=p.record(),accepted=accepted,trials=len(seen)))
            checkpoint_started=time.perf_counter();write(OUT/f'LAYER{layer}_SEARCH.json',dict(history=history,outcomes=outcomes,corner_traces=traces))
            region_profile['checkpoint_seconds']+=time.perf_counter()-checkpoint_started
            if accepted:break
        region_profile['seconds']=time.perf_counter()-region_started;region_profiles.append(region_profile)
        if spec['required'] and not accepted:
            print('UNRESOLVED_IMMEDIATE',layer,pi,flush=True);break
    local_search_seconds=time.perf_counter()-search_started
    unchanged=np.array_equal(candidate.view(np.uint64),nominal_before.view(np.uint64))
    final=nominal if use_atlas and unchanged else evaluate_layer(top,state.coordinates,candidate,layer,previous)
    persistence_started=time.perf_counter()
    if atlas is not None:
        accepted_supports=[np.asarray(r['scope']['active'],np.int64) for r in outcomes if r.get('accepted')]
        invalidation=atlas.adopt_evaluated(layer,coords,normals,[e for e in events if e['layer']>layer],accepted_supports)
        atlas.save(OUT/f'LAYER{layer}_ATLAS.npz')
        write(OUT/f'LAYER{layer}_ATLAS.json',dict(partition=atlas_partition,invalidation=invalidation,scientific_hash=atlas.scientific_hash(),final_numeric_reused=unchanged))
    # A nominal-only transaction still has a deterministic empty decision log.
    write(OUT/f'LAYER{layer}_SEARCH.json',dict(history=history,outcomes=outcomes,corner_traces=traces))
    save_candidate(OUT/f'LAYER{layer}_CANDIDATE.npz',candidate,coords,normals,OUT/'INITIAL_FORECAST.npz')
    write(OUT/f'LAYER{layer}_WARNINGS.json',[e for e in events if e['layer']>layer])
    write(OUT/f'LAYER{layer}_CORRECTIONS.json',outcomes)
    write(OUT/f'LAYER{layer}_GATE.json',dict(gate=final['gate'],events=final['events'],coordinate_hash=array_hash(candidate),nominal_gate=nominal['gate']))
    if planner is not None:
        planner.save(OUT/f'LAYER{layer}_PLANNER.npz',array_hash(coords))
        write(OUT/f'LAYER{layer}_PLANNING.json',dict(events=planner.events,statistics=planner.summary()))
    persistence_seconds=time.perf_counter()-persistence_started
    (OUT/f'LAYER{layer}_PREPARE_TIMINGS.json').write_text(json.dumps(dict(stages=timings,region_profiles=region_profiles,total_seconds=time.perf_counter()-start,nominal_projection_seconds=fresh.projection_seconds,nominal_numeric_seconds=nominal['seconds'],final_numeric_seconds=0. if use_atlas and unchanged else final['seconds'],risk_discovery_seconds=risk_discovery_seconds,local_search_seconds=local_search_seconds,persistence_seconds=persistence_seconds,detailed_cache_hits=dc.hits,detailed_cache_misses=dc.misses,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print('PREPARED',layer,final['gate']['passed'],final['gate']['violations'],flush=True)
    return final['gate']['passed']

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--layer',type=int,required=True);a=p.parse_args()
    sys.exit(0 if prepare(a.layer) else 2)
