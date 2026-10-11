"""Replay immediate proposal identities and split the late rejection timer."""
from task4f_common import *
import task4_prepare_next as prod
from task4f_preserved import rejection
from task4f_instrument import same_arrays
import psutil

def run(cheap_only=False):
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));sdf=prod.NativeSdf();schedule=prod.load_schedule()
    rows=[];late=[];cheap=[]
    for layer in (3,6,11):
        _,coords,normals=prod.load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        _,final,finalnormals=prod.load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        prefix=tuple(np.load(BASE/'build_A'/f'FRONT_{k:02}.npy') for k in range(1,layer));candidate=coords[layer-1].copy();current=prod.SimpleNamespace(coordinates=prefix[-1])
        previous=prod.evaluate_cells(top,prefix[-2],prefix[-1],layer-1);nominal=prod.evaluate_layer(top,prefix[-1],candidate,layer,previous)
        events=read(BASE/f'LAYER{layer-1}_WARNINGS.json');retained=[np.array(o['patch']['active']) for o in read(BASE/'CORRECTION_HISTORY.json')['outcomes'] if o['corrected']]
        for k in range(2,layer):retained += [np.array(o['scope']['active']) for o in read(BASE/f'LAYER{k}_CORRECTIONS.json') if o.get('accepted')]
        specs,_=prod.select_supports(prefix[-1],top,candidate,nominal['events'],events,layer,retained)
        h=read(BASE/f'LAYER{layer}_SEARCH.json');last=None
        for i,r in enumerate(h['history']):
            group=(r['patch'],r['rings'])
            if group!=last:
                spec=specs[r['patch']];p=prod.bounded_patch(top,prod.expand_vertices(top,spec['vertices'],r['rings']));nom=candidate[p.active].copy()
                modes=prod.correction_fields(current,top,p,spec['field']);modes.extend(prod.search_fields(current,top,p,candidate,schedule.targets[-1])[0]);modes=dict(modes);last=group
            proposed,_=prod.try_tangent_proposal(nom,modes[r['mode']],np.full(len(p.active),r['factor']*schedule.increments[layer-1]),sdf,schedule.targets[layer-1]);assert array_hash(proposed)==r['hash']
            moved=p.active[np.any(nom.view(np.uint64)!=proposed.view(np.uint64),axis=1)]
            changed=np.isin(top.triangles[p.detailed.core_columns],moved).any(1)
            rows.append(dict(layer=layer,trial=i,candidate_hash=r['hash'],moved_vertices=len(moved),exact_current_calls=len(r.get('exact',{}).get('records',[])),exact_current_unchanged_input_calls=int((~changed).sum()) if 'exact' in r else 0))
            if cheap_only:
                tick=time.perf_counter();ids=top.triangles[p.detailed.core_columns];high=candidate[ids].copy();mask=np.isin(ids,p.active);high[mask]=proposed[np.searchsorted(p.active,ids[mask])]
                prisms=np.concatenate((prefix[-1][ids],high),axis=1);cq=prod.measure.__globals__['legacy_modules']()['component_quality']
                jac,volume=cq.PM._exact_wedge_jacobian_and_volume(prisms);det=cq.determinant(prisms,layer)
                count=dict(jacobian=int(np.count_nonzero((jac<=0)|~np.isfinite(jac))),signed_volume=int(np.count_nonzero((volume<=0)|~np.isfinite(volume))),determinant=int(np.count_nonzero((det<.0011)|~np.isfinite(det))))
                seconds=time.perf_counter()-tick
                assert all(v==r['gate']['violations'][k] for k,v in count.items())
                decisive=any(count.values())
                assert not decisive or not r['accepted']
                cheap.append(dict(layer=layer,trial=i,cells=len(ids),seconds=seconds,counts=count,decisive_rejection=decisive,original_rejection=rejection(r),candidate_hash=r['hash'],gate_counts_identical=True))
            if not cheap_only and (layer,i) in ((3,13),(6,1)):
                state=prod.PrivateState(p.active,tuple(a[p.active] for a in prefix));fc=prod.RollingForecastCache(front,top,schedule,sdf,coords,normals)
                forecast=fc.forecast(state,proposed);t=time.perf_counter();result=prod.measure(front,forecast,top,p.detailed);detail_seconds=time.perf_counter()-t
                sample=next(s for s in read(OUT/'BOUNDED_SAMPLES.json') if s['layer']==layer and s['trial']==i);assert scientific_hash(result)==sample['scientific_hash']
                t=time.perf_counter();exact=prod.exact_layer(front,top,forecast,p.detailed,layer);exact_seconds=time.perf_counter()-t;assert exact==r['exact']
                t=time.perf_counter();future=prod.future_failures(result,layer);future_seconds=time.perf_counter()-t
                assert min(future.values(),default=21)==r['first_after']
                assert rejection(r)=='FutureTrajectoryRegression' and r['new_earlier']
                late.append(dict(layer=layer,trial=i,detail_seconds=detail_seconds,exact_seconds=exact_seconds,future_failure_scan_seconds=future_seconds,
                    exact_records=len(exact['records']),exact_records_identical=True,metric_hash=scientific_hash(result),known_decisive_regression=r['new_earlier'],
                    measured_work_after_available_future_geometry_seconds=exact_seconds,
                    qualification='Future failure decision can be read from the completed metric result before exact current signs. Same preserved rejection; no accepted candidate skips certification.'))
            if r['accepted']:
                coords[:,p.active]=final[:,p.active];normals[:,p.active]=finalnormals[:,p.active];candidate[p.active]=proposed
    if cheap_only:save('CHEAP_CELL_REJECTION.json',dict(rows=cheap,decisive=sum(r['decisive_rejection'] for r in cheap),seconds=sum(r['seconds'] for r in cheap),no_production_change=True))
    else:save('CURRENT_INPUT_AUDIT.json',rows);save('LATE_REJECTION_BOUNDED.json',late)
    save('CHEAP_REJECTION_RUN.json' if cheap_only else 'REJECTION_RUN.json',dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset,rows=len(rows),task_elapsed=elapsed()))

if __name__=='__main__':run('--cheap-only' in sys.argv)
