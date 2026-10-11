"""Selection uses current generic events; preserved corrections are replay inputs.

The sealed selective-exact warning cache is reused only after exact equality of
its entire forecast input. It is not a source of hard-coded production IDs.
"""
from task4c_common import *
from types import SimpleNamespace
import psutil
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.advancing_front.risk_atlas import RiskAtlas
from sdf_cfd_mesher_v4.advancing_front.production_risk import select_supports
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer,evaluate_cells,replace_local_events
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache

def digest_result(r):
    return scientific_hash({k:v for k,v in r.items() if k!='seconds'})

def run(name):
    started=time.perf_counter();front,top,frames=geometry();schedule=load_schedule();sdf=NativeSdf()
    t=time.perf_counter()
    with np.load(BASE/'INITIAL_FORECAST.npz') as z:coords=z['coordinates'];normals=z['normals']
    initial=read(BASE/'INITIAL_SCAN.json')
    assert array_hash(coords)==initial['forecast_coordinates']
    atlas=RiskAtlas(front,top,schedule,sdf.source_identity,coords,normals)
    startup=time.perf_counter()-t
    events=[e for r in initial['records'] for e in r['events']]
    rows=[];performance=[];previous=None;dc=DetailedCache();initial_hash=atlas.scientific_hash()
    for layer in range(1,21):
        tick=time.perf_counter();stages={};t=time.perf_counter()
        candidate=atlas.nominal(layer,frames[layer-1]);stages['passive_retrieval']=time.perf_counter()-t
        t=time.perf_counter();nominal=evaluate_layer(top,frames[layer-1],candidate,layer,previous);stages['immediate_nominal_validation']=time.perf_counter()-t
        if layer==1:
            assert scientific_hash(nominal['events'])==scientific_hash(initial['records'][0]['events'])
        t=time.perf_counter();specs,traces=select_supports(frames[layer-1],top,candidate,nominal['events'],events,layer,atlas.retained)
        partition=atlas.activate(specs,events+nominal['events'],layer);stages['risk_selection_and_atlas']=time.perf_counter()-t
        # Regression truth is read only AFTER generic selection.
        archived=read(BASE/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_CORRECTIONS.json'))
        archived=archived['outcomes'] if layer==1 else archived
        selected=[array_hash(p.active) for p in atlas.supports]
        required=[]
        for r in archived:
            if r.get('corrected',r.get('accepted',False)):
                ids=np.array((r['patch'] if layer==1 else r['scope'])['active'],np.int64)
                required.append(array_hash(ids))
        missing=[h for h in required if h not in selected]
        if missing:
            save(f'{name}_SUPPORT_MISMATCH_{layer}.json',dict(selected=[p.active for p in atlas.supports],retained=atlas.retained,archived=archived))
            raise AssertionError(('GenericSupportMismatch',layer,missing,selected))
        # No optimizer is rerun in this replay. Its preserved output must lie
        # inside current generic active support before it can replace geometry.
        t=time.perf_counter();changed,invalidated=atlas.apply_preserved_candidate(layer,frames[layer],sdf)
        stages['local_future_reforecast']=time.perf_counter()-t
        t=time.perf_counter();_,reference,reference_normals=load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        assert np.array_equal(atlas.coordinates.view(np.uint64),reference.view(np.uint64)),('ReforecastIdentity',layer)
        # Normals at accepted points may be derived by projection and refreshed
        # evaluation. Require exact stored normal identity as well.
        assert np.array_equal(atlas.normals.view(np.uint64),reference_normals.view(np.uint64)),('NormalIdentity',layer)
        stages['reference_input_verification']=time.perf_counter()-t
        t=time.perf_counter();refresh_events=events;detailed=[];beforehits=dc.hits;beforemisses=dc.misses
        for p in atlas.supports:
            r=dc.get(front,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':atlas.coordinates}),top,p.detailed)
            refresh_events=replace_local_events(refresh_events,p.detailed,r)
            detailed.append(dict(support=array_hash(p.active),result=digest_result(r),counts=r['counts']))
        stages['active_detailed_refresh']=time.perf_counter()-t
        warnings=read(BASE/f'LAYER{layer}_WARNINGS.json')
        # The sealed cache contains selective exact evidence not re-solved here.
        # This reuse is bound to identical source topology and complete coordinates.
        a=[e for e in refresh_events if e['kind']!='pair' and e['layer']>layer]
        b=[e for e in warnings if e['kind']!='pair']
        if sorted(map(scientific_hash,a))!=sorted(map(scientific_hash,b)):
            save(f'{name}_EVENT_MISMATCH_{layer}.json',dict(current=a,sealed=b))
            raise AssertionError(('DirectWarningMismatch',layer))
        atlas.refresh_complete(warnings);events=warnings;after=atlas.partition()
        t=time.perf_counter()
        final=nominal if len(changed)==0 else evaluate_layer(top,frames[layer-1],frames[layer],layer,previous)
        previous=final['cells'];assert final['gate']['passed']
        frozen=read(BASE/f'LAYER_{layer:02}_NUMERIC.json')
        assert scientific_hash(final['gate'])==scientific_hash(frozen['gate']),('FinalGateMismatch',layer)
        stages['final_immediate_validation']=time.perf_counter()-t
        snapshot=atlas.checkpoint();before=atlas.scientific_hash();atlas.restore(snapshot);assert atlas.scientific_hash()==before
        row=dict(layer=layer,partition=partition,after_refresh=after,selected_supports=selected,required_corrections=required,missing=missing,
            changed_vertices=changed,invalidation=invalidated,forecast_hash=array_hash(atlas.coordinates),normal_hash=array_hash(atlas.normals),
            passive_coordinates_exact=True,all_forecast_coordinates_exact=True,all_required_corrections_active=True,
            future_warning_hash=scientific_hash(warnings),selective_exact_cache_source=f'task4/LAYER{layer}_WARNINGS.json',
            immediate_gate=final['gate'],detailed=detailed,checkpoint_hash=atlas.scientific_hash(),
            detailed_hits=dc.hits-beforehits,detailed_misses=dc.misses-beforemisses,
            forecast_entries_reused=(20-layer)*len(front.coordinates)-len(changed)*(20-layer),
            forecast_entries_remaining=(20-layer)*len(front.coordinates))
        rows.append(row);performance.append(dict(layer=layer,stages=stages,total_seconds=time.perf_counter()-tick,peak_rss=psutil.Process().memory_info().peak_wset))
        write(OUT/f'{name}_ATLAS.json',dict(initial_hash=initial_hash,rows=rows,scientific_hash=scientific_hash(rows)))
        save(f'{name}_TIMINGS.json',dict(startup_seconds=startup,layers=performance,total_seconds=time.perf_counter()-started))
        print(name,layer,'active',partition['active_regions'],'columns',len(partition['active_columns']),'changed',len(changed),'seconds',performance[-1]['total_seconds'],flush=True)
    return rows

if __name__=='__main__':run(sys.argv[1] if len(sys.argv)>1 else 'atlas_1')
