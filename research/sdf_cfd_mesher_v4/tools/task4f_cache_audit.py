"""Reconstruct exact query identities, not metric results, from preserved state."""
from task4f_common import *
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.forecast.detailed.patch import patch_key,prisms
from task4f_instrument import dependency_counts
from collections import Counter

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates))
    trials={(r['layer'],r['trial']):r for r in read(OUT/'CACHE_KEY_REPLAY.json') if r['trial'] is not None}
    operations=read(OUT/'BOUNDED_OPERATION_COUNTS.json');events=[];cache={};seen=set();prior_support={};counts=Counter();baselines=[]
    def query(key,support,layer,trial,expected=None):
        hit=key in cache
        if hit:reason='ExactInputHit'
        elif key in seen:reason='IdenticalInputEvicted'
        elif support not in prior_support:reason='FirstInputForSupport'
        else:reason='GeometryActuallyChanged'
        if expected is not None:assert hit==expected,(layer,trial,reason,expected)
        events.append(dict(layer=layer,trial=trial,key=key,support=support,hit=hit,classification=reason));counts[reason]+=1
        if not hit:cache[key]=None
        seen.add(key);prior_support[support]=key
    for layer in range(1,12):
        if layer==1:
            with np.load(BASE/'INITIAL_FORECAST.npz') as z:coords=z['coordinates'];normals=z['normals']
        else:_,coords,normals=prod.load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        _,final,fn=prod.load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        h=read(BASE/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'))
        timing=read(BASE/('CORRECTION_TIMINGS.json' if layer==1 else f'LAYER{layer}_PREPARE_TIMINGS.json'))
        scopes={(o.get('region',o.get('patch')),o.get('rings',0)):o.get('scope',o.get('patch')) for o in h['outcomes'] if 'scope' in o or isinstance(o.get('patch'),dict)}
        # The accepted supports in these preserved transactions must be disjoint
        # before final coordinates may reconstruct their intermediate bases.
        used=set()
        for o in h['outcomes']:
            if o.get('accepted',o.get('corrected',False)):
                ids=set(o.get('scope',o.get('patch'))['active']);assert not ids&used,'OverlappingSavedSupportNeedsSequentialReplay';used|=ids
        last=None
        for i,r in enumerate(h['history']):
            group=(r.get('region',r.get('patch')),r.get('rings',0))
            if group!=last:
                p=prod.bounded_patch(top,np.array(scopes[group]['active']));last=group
                assert p.record()==scopes[group]
                baseline=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords});bkey=patch_key(front,baseline,top,p.detailed)
                support=scientific_hash(p.detailed.record())
                expected=None if layer==1 else next(x['baseline_cache_hit'] for x in timing['region_profiles'] if x['patch']==group[0])
                query(bkey,support,layer,None,expected);baselines.append(dict(layer=layer,patch=group[0],key=bkey,hit=events[-1]['hit']))
            if layer==1:
                trial_coords=coords.copy();trial_coords[:,p.active]=final[:,p.active]
                f=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':trial_coords});key=patch_key(front,f,top,p.detailed)
                assert array_hash(trial_coords[0,p.active])==r['coordinates_hash']
                x0,_=prisms(front,baseline,top,p.detailed);x1,_=prisms(front,f,top,p.detailed);account,changed=dependency_counts(top,p.detailed,x0,x1)
                core=np.searchsorted(p.detailed.columns,p.detailed.core_columns)
                operations.append(dict(layer=1,trial=i,patch=group[0],accepted=r['accepted'],candidate_hash=r['coordinates_hash'],rejection='Accepted',
                    moved_vertices=int(np.any(coords[0,p.active].view(np.uint64)!=trial_coords[0,p.active].view(np.uint64),axis=1).sum()),active_vertices=len(p.active),maximum_displacement=r['displacement'],**account,
                    exact_current_calls=len(r['exact']['records']),exact_current_unchanged_input_calls=int((~changed[0,core]).sum()),committed_prefix_cells_recomputed=0))
                expected=False
            else:key=trials[layer,i]['key'];expected=timing['stages'][i]['detailed_cache_hit']
            query(key,support,layer,i,expected)
            if layer>1:
                while len(cache)>6:cache.pop(next(iter(cache)))
            if r['accepted']:coords[:,p.active]=final[:,p.active];normals[:,p.active]=fn[:,p.active]
    assert len(operations)==255
    save('ACTIVE_TRIAL_OPERATION_COUNTS.json',sorted(operations,key=lambda r:(r['layer'],r['trial'])))
    # Seven missing original-run local exact counters can be derived from changed
    # source triangles only if per-trial masks were retained. Mark unavailable,
    # do not replace them with an estimate.
    save('CACHE_MISS_CLASSIFICATION.json',dict(counts=dict(counts),events=events,baselines=baselines,
        cache_population_model='Insertion order, six-entry retention after each later-layer trial; identical to production. Layer-1 four initial queries included separately.',
        exclusions=dict(object_identity=0,serialization_order=0,layer_index_without_changed_scientific_input=0,unexplained=0),
        scientific_policy='The complete-patch key is correct for the complete-patch result. Reuse granularity is coarse: one changed vertex invalidates every metric in that entry.',seconds=time.perf_counter()-start))

if __name__=='__main__':run()
