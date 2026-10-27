"""Instrument synthetic unit-test controls; these are not production risk patches."""
import sys
import time
import psutil
from pathlib import Path
from capsule_layer2_preflight import V4,save
sys.path.insert(0,str(V4/'tests'))
from test_committed_detailed_domain import control
from sdf_cfd_mesher_v4.forecast.detailed.committed_domain import CommittedDetailedCache
from sdf_cfd_mesher_v4.forecast.detailed.patch import column_patch
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash


def run():
    start=time.perf_counter();f,q,t,d=control();cache=CommittedDetailedCache(d)
    a=column_patch(t,0);b=column_patch(t,8)
    ra=cache.get(f,q,t,a);cold_a=cache.last_stats.copy()
    rb=cache.get(f,q,t,b);cold_b=cache.last_stats.copy()
    cache.get(f,q,t,a);warm=cache.last_stats.copy()
    h1=scientific_hash(ra);h2=scientific_hash(rb)
    began=time.perf_counter();invalid=cache.invalidate([0]);cache.get(f,q,t,b);retained=cache.last_stats.copy()
    new=cache.get(f,q,t,a);recompute=cache.last_stats.copy();seconds=time.perf_counter()-began
    second=CommittedDetailedCache(d);replay=second.get(f,q,t,a)
    result=dict(scope='Synthetic two-component octahedral unit-test control; not a production support',
        layers=q.layers.tolist(),support_a=a.record(),support_b=b.record(),counts=ra['counts'],
        cold_a=cold_a,cold_b=cold_b,warm=warm,invalidation=invalid,retained=retained,recompute=recompute,
        invalidation_seconds=seconds,outside_region_exact=scientific_hash(rb)==h2,
        recomputed_exact=scientific_hash(new)==h1,replay_exact=scientific_hash(replay)==h1,
        metrics_a=h1,metrics_b=h2,cache_identity=cache.scientific_identity(),domain_identity=d.sha256,
        peak_rss=psutil.Process().memory_info().peak_wset,total_seconds=time.perf_counter()-start)
    assert result['outside_region_exact'] and result['recomputed_exact'] and result['replay_exact']
    save(V4/'results/production_detailed_lookahead_committed_front/CONTROL_CACHE.json',result)
    print(result)


if __name__=='__main__':run()
