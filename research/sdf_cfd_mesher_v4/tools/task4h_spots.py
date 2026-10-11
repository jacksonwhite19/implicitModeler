"""Replay preserved accepted local futures, never construct new shell geometry."""
from task4h_common import *
from collections import Counter
from types import SimpleNamespace
import psutil
import task4_prepare_next as prod
from task4h_legacy import SpotPlanner
from sdf_cfd_mesher_v4.forecast.detailed import rolling_spots as spots
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.repaired_runtime import CertificateSession

def cohort(front,top,layers=range(2,12)):
    for layer in layers:
        require_time();_,coordinates,_=prod.load_candidate(BASE/f'LAYER{layer-1}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        _,final,_=prod.load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
        history=read(BASE/f'LAYER{layer}_SEARCH.json');outcomes={(o['patch'],o.get('rings',0)):o['scope'] for o in history['outcomes']}
        events=[e for e in read(BASE/f'LAYER{layer-1}_WARNINGS.json') if e['kind']=='pair']
        for i,h in enumerate(history['history']):
            if not h['accepted']:continue
            scope=outcomes[h['patch'],h.get('rings',0)];patch=prod.bounded_patch(top,np.asarray(scope['active']))
            assert patch.record()==scope
            before=SimpleNamespace(vertex={'coordinates':coordinates.copy()})
            coordinates[:,patch.active]=final[:,patch.active]
            after=SimpleNamespace(vertex={'coordinates':coordinates})
            assert array_hash(coordinates[layer-1,patch.active])==h['hash']
            known=[e for e in events if set(e['columns'])&set(map(int,patch.detailed.core_columns))]
            yield layer,i,before,after,patch.detailed,known,h['future_spots']
            events=[e for e in events if not (set(e['columns'])&set(map(int,patch.detailed.core_columns)))]+h['future_spots']['warnings']
        assert array_hash(coordinates)==array_hash(final),(layer,'RestoredForecastMismatch')

def pair_inputs(front,top,future,pair):
    q=[];labels=[]
    for layer,edge in pair:
        ids=top.edges[edge];lo=front.coordinates[ids] if layer==1 else future.vertex['coordinates'][layer-2,ids]
        hi=future.vertex['coordinates'][layer-1,ids];q.append(np.concatenate((lo,hi)))
        labels.append(tuple((k,int(v)) for k in (layer-1,layer) for v in ids))
    return np.asarray(q),labels

def run(mode='old'):
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));planner=SpotPlanner()
    seen=set();pair_ids={};rows=[];times=Counter();counts=Counter();methods=Counter();ranks=Counter();session=planner.session
    original=session.pair
    def instrument(a,b):
        hit=session.hits;t=time.perf_counter();r=original(a,b);dt=time.perf_counter()-t
        times['session_pair']+=dt;counts['session_pair']+=1;methods[r.get('method','unspecified')]+=1;ranks[str(tuple(sorted((a.rank,b.rank))))]+=1
        if session.hits==hit:times['method:'+r.get('method','unspecified')]+=dt
        return r
    session.pair=instrument
    for layer,trial,before,after,patch,known,expected in cohort(front,top):
        require_time();t=time.perf_counter();a,na=planner.select(front,top,before,patch,layer,32);b,nb=planner.select(front,top,after,patch,layer,32);selection=time.perf_counter()-t
        pairs=sorted(set(a+b)|{tuple(tuple(v) for v in e['sides']) for e in known});assert pairs==[tuple(tuple(v) for v in r['sides']) for r in expected['records']]
        assert (na,nb)==(expected['broad_before'],expected['broad_after']);results=[];t=time.perf_counter()
        for pair,authority in zip(pairs,expected['records']):
            row=dict(sides=pair)
            for name,future in [('before',before),('after',after)]:
                q,labels=pair_inputs(front,top,future,pair);key=scientific_hash(dict(q=array_hash(q),labels=labels))
                counts['requests']+=1;counts['repeated_identity' if key in seen else 'unique_identity']+=1
                counts['same_id_changed_geometry' if pair in pair_ids and key not in pair_ids[pair] else 'first_or_identical_id']+=1
                seen.add(key);pair_ids.setdefault(pair,set()).add(key)
                st=time.perf_counter();result=spots.exact_pair(front,top,future,pair,session);times['exact_pair_total']+=time.perf_counter()-st
                assert plain(result)==authority[name],(layer,trial,pair,name,'FrozenSpotMismatch');row[name]=result
            results.append(row)
        pair_seconds=time.perf_counter()-t
        rows.append(dict(layer=layer,trial=trial,core_columns=len(patch.core_columns),pairs=len(pairs),broad_before=na,broad_after=nb,selection_seconds=selection,pair_seconds=pair_seconds,scientific_hash=scientific_hash(results),equal=True))
        save('SPOT_BASELINE.json',dict(rows=rows,times=dict(times),counts=dict(counts),methods=dict(methods),ranks=dict(ranks),session_hits=session.hits,session_misses=session.misses,selection_stats=dict(planner.stats),total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset))
        print('SPOT_BASELINE',layer,trial,selection,pair_seconds,flush=True)
if __name__=='__main__':run()
