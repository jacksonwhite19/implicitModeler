from task4h_spots import *
from task4h_legacy import SpatialQueries
from sdf_cfd_mesher_v4.certificate.future_selection import SortedBounds
from sdf_cfd_mesher_v4.certificate.future_proofs import FutureSpotProofs,verify_bound_record
from sdf_cfd_mesher_v4.forecast.detailed.patch import LocalityFailure

class IndexedSpatialQueries(SpatialQueries):
    def __init__(self):super().__init__();self.bounds=None
    def update(self,index):
        changed=self.lo is None or not np.array_equal(self.lo.view(np.uint64),index.lo.view(np.uint64)) or not np.array_equal(self.hi.view(np.uint64),index.hi.view(np.uint64))
        super().update(index)
        if changed:self.bounds=SortedBounds(self.lo,self.hi);self.stats['index_builds']+=1
    def candidates(self,core_edges,max_edges):
        selected=[];pad=128*np.finfo(float).eps
        for e in core_edges:
            e=int(e)
            if e not in self.rows:
                self.rows[e],tested=self.bounds.query(self.lo[e],self.hi[e],pad);self.stats['query_misses']+=1;self.stats['indexed_target_tests']+=tested;self.stats['full_target_tests_avoided']+=len(self.lo)-tested
            else:self.stats['query_hits']+=1
            selected.append(self.rows[e])
        ids=np.unique(np.concatenate(selected)) if selected else np.empty(0,dtype=np.int64)
        if len(ids)>max_edges or len(ids)==len(self.lo):raise LocalityFailure('ContinuousCandidateScopeExceedsCallerBound')
        return ids

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));rows=[];hashes=[]
    for replay in range(2):
        planner=SpotPlanner();planner.spatial=IndexedSpatialQueries();proofs=FutureSpotProofs();history=[]
        for layer,trial,before,after,patch,known,expected in cohort(front,top):
            require_time();t=time.perf_counter();a,na=planner.select(front,top,before,patch,layer,32);b,nb=planner.select(front,top,after,patch,layer,32);selection=time.perf_counter()-t
            pairs=sorted(set(a+b)|{tuple(tuple(v) for v in e['sides']) for e in known});assert pairs==[tuple(tuple(v) for v in r['sides']) for r in expected['records']]
            assert (na,nb)==(expected['broad_before'],expected['broad_after']);results=[];t=time.perf_counter()
            for pair,authority in zip(pairs,expected['records']):
                row=dict(sides=pair)
                for name,future in [('before',before),('after',after)]:
                    q,labels=pair_inputs(front,top,future,pair);r=proofs.future_pair(q,labels)
                    assert r['classification']==authority[name]['classification'],(layer,trial,pair,name,'FutureOutcomeMismatch')
                    row[name]=r
                results.append(row)
            pair_seconds=time.perf_counter()-t
            # These are all downstream consumers of exact future probes.
            assert spots.earlier_pair_families(results)==expected['new_earlier_pair_families']
            scientific=dict(layer=layer,trial=trial,pairs=pairs,broad_before=na,broad_after=nb,results=results)
            history.append(scientific_hash(scientific))
            rows.append(dict(replay=replay+1,layer=layer,trial=trial,pairs=len(pairs),selection_seconds=selection,pair_seconds=pair_seconds,scientific_hash=history[-1],equivalent=True))
            print('SPOT_NEW',replay+1,layer,trial,selection,pair_seconds,flush=True)
        hashes.append(scientific_hash(history));save(f'SPOT_REPLAY_{replay+1}.json',dict(scientific_hash=hashes[-1],history=history,proof_stats=dict(proofs.stats),proof_times=dict(proofs.times),spatial=dict(planner.spatial.stats),selection=dict(planner.stats),proof_cache_entries=len(proofs.cache)))
        save('SPOT_EQUIVALENCE.json',dict(rows=rows,hashes=hashes,peak_rss=psutil.Process().memory_info().peak_wset,total_seconds=time.perf_counter()-start))
    assert hashes[0]==hashes[1]
    # A genuine forbidden nominal future pair must still fall through exactly.
    with np.load(BASE/'INITIAL_FORECAST.npz') as z:f=SimpleNamespace(vertex={'coordinates':z['coordinates']})
    pair=((1,1),(10,729));q,labels=pair_inputs(front,top,f,pair);old=spots.exact_pair(front,top,f,pair,CertificateSession());new=FutureSpotProofs().future_pair(q,labels)
    assert old==new and old['classification']=='CertifiedForbiddenIntersection'
    save('FORBIDDEN_PAIR_REGRESSION.json',dict(pair=pair,result=new,exact_record_identical=True))
if __name__=='__main__':run()
