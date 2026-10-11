"""Preserved canonical proposal replay, not a full shell build or optimizer run.

Only missing local trial coordinates are reconstructed from preserved mode and
factor. Every proposal and complete forecast key is checked against Task-4F.
The old evaluator and two independent incremental caches see identical inputs.
"""
from task4g_common import *
from task4f_instrument import same_arrays
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.forecast.detailed.dependency_metrics import LocalMetricEngine
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
from sdf_cfd_mesher_v4.forecast.detailed.patch import patch_key
from sdf_cfd_mesher_v4.forecast.detailed.exact_cache import ExactJacobianCache
from sdf_cfd_mesher_v4.advancing_front.staged_evaluation import immediate_cell_screen
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import SpotPlanner
import psutil

def run(layers=range(1,12)):
    start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));sdf=prod.NativeSdf();schedule=prod.load_schedule()
    expected_keys={(r['layer'],r['trial']):r['key'] for r in read(ROOT/'results/task4f/CACHE_MISS_CLASSIFICATION.json')['events'] if r['trial'] is not None}
    engines=[LocalMetricEngine(),LocalMetricEngine()];excache=ExactJacobianCache();spots=SpotPlanner();rows=[];bases=[]
    for layer in layers:
        require_time()
        if layer==1:
            with np.load(BASE/'INITIAL_FORECAST.npz') as z:coords=z['coordinates'];normals=z['normals']
            prefix=();current=front;candidate=coords[0].copy();regions,field,_=prod.controlling_support(front,top,candidate,read(BASE/'INITIAL_SCAN.json')['records'][0]['events'])
            specs=[dict(vertices=np.asarray(r.vertex_ids),field=field,required=True,name='immediate') for r in regions];events=[]
        else:
            _,coords,normals=prod.load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
            prefix=tuple(np.load(BASE/'build_A'/f'FRONT_{k:02}.npy') for k in range(1,layer));candidate=coords[layer-1].copy();current=prod.SimpleNamespace(coordinates=prefix[-1])
            previous=prod.evaluate_cells(top,front.coordinates if layer==2 else prefix[-2],prefix[-1],layer-1)
            nominal=prod.evaluate_layer(top,prefix[-1],candidate,layer,previous);events=read(BASE/f'LAYER{layer-1}_WARNINGS.json')
            retained=[np.asarray(o['patch']['active']) for o in read(BASE/'CORRECTION_HISTORY.json')['outcomes'] if o['corrected']]
            for k in range(2,layer):retained += [np.asarray(o['scope']['active']) for o in read(BASE/f'LAYER{k}_CORRECTIONS.json') if o.get('accepted')]
            specs,_=prod.select_supports(prefix[-1],top,candidate,nominal['events'],events,layer,retained)
        hist=read(BASE/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'));last=None
        outcomes={(o.get('region',o.get('patch')),o.get('rings',0)):o.get('scope',o.get('patch')) for o in hist['outcomes'] if 'scope' in o or isinstance(o.get('patch'),dict)}
        for i,h in enumerate(hist['history']):
            require_time();group=(h.get('region',h.get('patch')),h.get('rings',0))
            if group!=last:
                spec=specs[group[0]];p=prod.bounded_patch(top,prod.expand_vertices(top,spec['vertices'],group[1]));assert p.record()==outcomes[group]
                fc=prod.RollingForecastCache(front,top,schedule,sdf,coords,normals);state=prod.PrivateState(p.active,tuple(x[p.active] for x in prefix));nominal_local=candidate[p.active].copy()
                modes=prod.correction_fields(current,top,p,spec['field'])
                if layer>1:modes.extend(prod.search_fields(current,top,p,candidate,schedule.targets[-1])[0])
                modes=dict(modes);baseline=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
                t=time.perf_counter();before=engines[0].measure(front,baseline,top,p.detailed);bt=time.perf_counter()-t
                before2=engines[1].measure(front,baseline,top,p.detailed);assert same_arrays(before,before2)
                bases.append(dict(layer=layer,group=group,seconds=bt,counts={k:v for k,v in engines[0].last_stats.items() if k!='changed_cell_ids'}))
                before_fail=prod.future_failures(before,layer);first=min(before_fail.values(),default=21)
                mechanism=min(before_fail,key=lambda k:(before_fail[k],k)) if before_fail else None
                tail=prod.mechanism_tail(before,mechanism,layer) if mechanism else None
                known=[e for e in events if e['kind']=='pair' and set(e['columns'])&set(map(int,p.detailed.core_columns))]
                target=min((e for e in known if e['metric']=='CertifiedForbiddenIntersection'),key=lambda e:(e['layer'],str(e['sides'])),default=None)
                pair_priority=target is not None and (mechanism is None or target['layer']<first);last=group
            t=time.perf_counter();proposed,_=prod.try_tangent_proposal(nominal_local,modes[h['mode']],np.full(len(p.active),h['factor']*schedule.increments[layer-1]),sdf,schedule.targets[layer-1]);projection_time=time.perf_counter()-t
            assert array_hash(proposed)==h.get('hash',h.get('coordinates_hash')),(layer,i,'ProposalChanged')
            t=time.perf_counter();screen=immediate_cell_screen(top,current.coordinates,candidate,p.active,proposed,p.detailed,layer);screen_time=time.perf_counter()-t
            assert all(v==h['gate']['violations'][k] for k,v in screen['gate']['violations'].items()),(layer,i,'ScreenGateMismatch')
            row=dict(layer=layer,trial=i,group=group,accepted=False,candidate_hash=array_hash(proposed),projection_seconds=projection_time,screen_seconds=screen_time,early_rejected=screen['rejected'],screen=screen)
            if screen['rejected']:
                assert not h['accepted'];row.update(old_decision=h['accepted'],decision_equal=True,detailed_seconds=0.,old_detailed_seconds=None,exact_calls=0,exact_hits=0,spot_seconds=0.,forecast_seconds=0.,counts={})
            else:
                t=time.perf_counter();forecast=fc.forecast(state,proposed);forecast_time=time.perf_counter()-t
                assert patch_key(front,forecast,top,p.detailed)==expected_keys[layer,i],(layer,i,'ForecastInputChanged')
                t=time.perf_counter();old=measure(front,forecast,top,p.detailed);oldtime=time.perf_counter()-t
                t=time.perf_counter();new=engines[0].measure(front,forecast,top,p.detailed);newtime=time.perf_counter()-t
                stats=engines[0].last_stats.copy();new2=engines[1].measure(front,forecast,top,p.detailed)
                assert same_arrays(old,new),(layer,i,'IncrementalMetricMismatch')
                assert same_arrays(new,new2) and stats==engines[1].last_stats,(layer,i,'ReplayMismatch')
                gate=prod.immediate_numeric(new,layer);assert gate==h['gate'],(layer,i,'HardGateMismatch')
                counts={k:v for k,v in stats.items() if k!='changed_cell_ids'}
                e0=(excache.hits,excache.misses);t=time.perf_counter();ex=prod.exact_layer(front,top,forecast,p.detailed,layer,cache=excache);extime=time.perf_counter()-t
                assert ex==h['exact'],(layer,i,'ExactReceiptChanged')
                fail=prod.future_failures(new,layer);admission=prod.future_admission(before_fail,fail)
                assert admission['new_earlier']==h.get('new_earlier',h.get('new_entity_failures')),(layer,i,'FutureChanged')
                aftertail=prod.mechanism_tail(new,mechanism,layer) if mechanism else None
                improves=prod.mechanism_improves(tail,aftertail) if mechanism else False
                if layer>1:
                    assert tail==h['baseline_mechanism'] and aftertail==h['candidate_mechanism'],(layer,i,'ObjectiveChanged')
                accept=ex['passed'] and admission['passed'];spot=None;spot_time=0.
                if layer>1:
                    accept &= spec['required'] or improves or pair_priority
                    if accept:
                        t=time.perf_counter();spot=prod.compare_spots(front,top,baseline,forecast,p.detailed,layer,known,planner=spots);spot_time=time.perf_counter()-t
                        assert plain(spot)==h['future_spots'],(layer,i,'ExactFutureSpotChanged')
                        target_improves=prod.pair_family_improves(spot['records'],[edge for k,edge in target['sides']])['improved'] if pair_priority else improves
                        accept=not spot['new_earlier_pair_families'] and (spec['required'] or target_improves)
                assert accept==h['accepted'],(layer,i,'DecisionChanged')
                row.update(accepted=bool(accept),old_decision=h['accepted'],decision_equal=True,detailed_seconds=newtime,old_detailed_seconds=oldtime,
                    metric_hash=metric_identity(new),replay_hash=metric_identity(new2),dependencies_hash=scientific_hash(stats),counts=counts,
                    exact_seconds=extime,exact_calls=excache.misses-e0[1],exact_hits=excache.hits-e0[0],exact_records=len(ex['records']),
                    forecast_seconds=forecast_time,spot_seconds=spot_time,gate=gate,future_hash=scientific_hash(fail),exact_hash=scientific_hash(ex),spot_hash=scientific_hash(spot))
                if accept:
                    coords[:,p.active]=forecast.local['coordinates'];normals[:,p.active]=forecast.local['normal'];candidate[p.active]=proposed
                    if layer>1:events=prod.replace_local_events(events,p.detailed,new)+spot['warnings']
            rows.append(row)
            print('REPLAY',layer,i,'early' if screen['rejected'] else 'accepted' if row['accepted'] else 'future reject',flush=True)
        assert array_hash(candidate)==read(BASE/f'LAYER{layer}_GATE.json')['coordinate_hash'],(layer,'FinalCandidateChanged')
        save('TRIAL_REPLAY.json',rows);save('BASELINE_REFRESHES.json',bases)
        save('REPLAY_PROGRESS.json',dict(completed_layer=layer,trials=len(rows),seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset,task_elapsed=elapsed()))
    print('REPLAY_COMPLETE',len(rows),time.perf_counter()-start,flush=True)
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--layers',default='1,2,3,4,5,6,7,8,9,10,11');a=p.parse_args();run(tuple(map(int,a.layers.split(','))))
