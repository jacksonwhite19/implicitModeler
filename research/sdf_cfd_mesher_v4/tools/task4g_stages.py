"""All preserved immediate trial inputs, through the production staged checks."""
from task4g_common import *
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.advancing_front.staged_evaluation import immediate_cell_screen,immediate_numeric_screen,exact_current
from sdf_cfd_mesher_v4.forecast.detailed.exact_cache import ExactJacobianCache
import psutil

def run():
    require_time();t0=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));sdf=prod.NativeSdf();schedule=prod.load_schedule();cache=ExactJacobianCache();rows=[]
    for layer in range(1,12):
        if layer==1:
            with np.load(BASE/'INITIAL_FORECAST.npz') as z:coords=z['coordinates']
            current=front;candidate=coords[0].copy();regions,field,_=prod.controlling_support(front,top,candidate,read(BASE/'INITIAL_SCAN.json')['records'][0]['events'])
            specs=[dict(vertices=np.asarray(r.vertex_ids),field=field) for r in regions]
        else:
            _,coords,_=prod.load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
            current=prod.SimpleNamespace(coordinates=coords[layer-2]);candidate=coords[layer-1].copy()
            previous=prod.evaluate_cells(top,front.coordinates if layer==2 else coords[layer-3],current.coordinates,layer-1)
            nominal=prod.evaluate_layer(top,current.coordinates,candidate,layer,previous);events=read(BASE/f'LAYER{layer-1}_WARNINGS.json')
            retained=[np.asarray(o['patch']['active']) for o in read(BASE/'CORRECTION_HISTORY.json')['outcomes'] if o['corrected']]
            for k in range(2,layer):retained += [np.asarray(o['scope']['active']) for o in read(BASE/f'LAYER{k}_CORRECTIONS.json') if o.get('accepted')]
            specs,_=prod.select_supports(current.coordinates,top,candidate,nominal['events'],events,layer,retained)
        _,final,_=prod.load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        hist=read(BASE/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'));last=None
        for i,h in enumerate(hist['history']):
            require_time();group=(h.get('region',h.get('patch')),h.get('rings',0))
            if group!=last:
                spec=specs[group[0]];p=prod.bounded_patch(top,prod.expand_vertices(top,spec['vertices'],group[1]));nominal_local=candidate[p.active].copy()
                modes=prod.correction_fields(current,top,p,spec['field'])
                if layer>1:modes.extend(prod.search_fields(current,top,p,candidate,schedule.targets[-1])[0])
                modes=dict(modes);last=group
            proposed,_=prod.try_tangent_proposal(nominal_local,modes[h['mode']],np.full(len(p.active),h['factor']*schedule.increments[layer-1]),sdf,schedule.targets[layer-1]);assert array_hash(proposed)==h.get('hash',h.get('coordinates_hash'))
            t=time.perf_counter();screen=immediate_cell_screen(top,current.coordinates,candidate,p.active,proposed,p.detailed,layer);cheap_seconds=time.perf_counter()-t
            full_seconds=exact_seconds=0.;hits=misses=0
            if not screen['rejected']:
                t=time.perf_counter();screen=immediate_numeric_screen(front,top,coords,p.active,proposed,p.detailed,layer);full_seconds=time.perf_counter()-t
            for k,v in screen['gate']['violations'].items():assert v==h['gate']['violations'][k],(layer,i,k)
            for k,v in screen['gate']['extrema'].items():assert v==h['gate']['extrema'][k],(layer,i,k)
            assert screen['rejected']==(not h['gate']['passed'])
            if not screen['rejected']:
                b=(cache.hits,cache.misses);t=time.perf_counter();ex=exact_current(top,current.coordinates,candidate,p.active,proposed,p.detailed,layer,cache);exact_seconds=time.perf_counter()-t
                assert ex==h['exact'];hits=cache.hits-b[0];misses=cache.misses-b[1]
            rows.append(dict(layer=layer,trial=i,candidate_hash=array_hash(proposed),rejection_stage=screen['stage'] if screen['rejected'] else None,
                rejected_before_future=screen['rejected'],hard_values_equal=True,cheap_seconds=cheap_seconds,numeric_seconds=full_seconds,exact_seconds=exact_seconds,exact_hits=hits,exact_calls=misses,
                conditioning_before_decisive_rejection=0,future_calls_if_rejected=0))
            if h['accepted']:
                # Preserved accepted geometry is only the next input of this
                # stage-isolation test. It is never a production proposal.
                coords[:,p.active]=final[:,p.active];candidate[p.active]=proposed
        save('STAGED_REPLAY.json',rows);print('STAGED',layer,len(rows),flush=True)
    save('STAGED_RUNTIME.json',dict(seconds=time.perf_counter()-t0,peak_rss=psutil.Process().memory_info().peak_wset,trials=len(rows)))
if __name__=='__main__':run()
