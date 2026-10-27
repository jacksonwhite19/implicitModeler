"""Read-only physical accounting of prefix dependencies after L5."""
from production_bounded_layers import *

def physical(paths):
    values={}
    for path in paths:
        r=metadata(path,content=True)
        values[physical_key(r)]=r
    return values

def audit():
    start=time.perf_counter();folder=configure(5)
    assert read(folder/'QUALIFIED.json')['passed']
    assert read(folder/'A/POST_RETENTION.json')['readiness']['binding']['next_layer']==6
    roots={3:L3,4:OUT/'L4',5:folder}
    closures={k:checkpoint_closure(p/'A/checkpoints')[0] for k,p in roots.items()}
    native={k:physical(paths) for k,paths in closures.items()}
    # The qualified diagnostics path reads each actual prefix front plus the
    # immediately previous quality export. Native restore uses the full
    # transitive receipt and component closure.
    required=dict(native[5])
    cache_paths=[p/'A/CANDIDATE.npz' for p in roots.values()]+[folder/'A/QUALITY.json']
    cache_paths.extend(V4/'results'/name/'A/CANDIDATE.npz' for name in ('production_adaptive_capsule_first_layer','production_adaptive_capsule_layer2'))
    required.update(physical(cache_paths))
    result={}
    for k,p in roots.items():
        active=dict(native[k]);active.update(physical([p/'A/QUALITY.json',p/'A/CANDIDATE.npz']))
        still={i:r for i,r in active.items() if i in required}
        historical={i:r for i,r in active.items() if i not in required and i in native[k]}
        old_cache={i:r for i,r in active.items() if i not in required and i not in native[k]}
        result[f'L{k}']=dict(
            active_snapshot_physical=sum(r['cluster_bytes'] for r in active.values()),
            still_required_for_L6_physical=sum(r['cluster_bytes'] for r in still.values()),
            historical_restore_only_physical=sum(r['cluster_bytes'] for r in historical.values()),
            old_diagnostic_cache_physical=sum(r['cluster_bytes'] for r in old_cache.values()),
            qualified_checkpoint_retirement_physical=0,
            still_required=list(still.values()),historical_restore_only=list(historical.values()),old_diagnostic_cache=list(old_cache.values()))
    record=dict(passed=True,layers=result,required_union_physical=sum(r['cluster_bytes'] for r in required.values()),
        no_forecast=True,no_generation=True,no_retirement_performed=True,
        interpretation='Categories overlap across layers through shared immutable objects. Historical-only native objects remain protected; no checkpoint retirement is qualified. Old quality exports are no longer read by the next diagnostic path but remain retained. No deletion authority is inferred from absence in current closure.',
        seconds=time.perf_counter()-start,**cycle.resources())
    save(OUT/'ACTIVE_RETIREMENT_ANALYSIS.json',record)

if __name__=='__main__':audit()
