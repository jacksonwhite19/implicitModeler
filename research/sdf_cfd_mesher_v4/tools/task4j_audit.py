"""Frozen per-layer wall/worker decomposition; no repeated baseline build."""
from task4j_common import *
def run():
    rows=[];times=collections.Counter();methods=collections.Counter();class_counts=collections.Counter();ranks=collections.Counter()
    for layer in range(1,21):
        p=BASE/f'certificate_layer_{layer:02}';ledger=read(BASE/f'LAYER_{layer:02}_CERTIFICATE.json');perf=read(BASE/f'LAYER_{layer:02}_TIMINGS.json')
        detail=collections.Counter();work_methods=collections.Counter();work_ranks=collections.Counter()
        for s in perf['certificate_stages']:
            for k,v in s.items():
                if isinstance(v,(int,float)):detail[k]+=v
            detail.update(s.get('details',{}));work_methods.update(s.get('exact_methods',{}));work_ranks.update(s.get('exact_ranks',{}))
        times.update(detail);methods.update(work_methods);ranks.update(work_ranks);class_counts.update(ledger['class_counts'])
        E=159453;potential=(layer-1)*E*E+E*(E-1)//2
        rows.append(dict(layer=layer,required_pairs=ledger['candidate_pairs'],potential_new_pairs=potential,
            strict_AABB_complement=potential-ledger['candidate_pairs'],topology_only_exclusions=0,
            conservative_proofs=ledger['candidate_pairs']-ledger['exact_calls'],frozen_kernel_calls=ledger['exact_calls'],
            certificate_seconds=perf['certificate_wall_seconds'],timing=dict(detail),method_counts=dict(work_methods),rank_counts=dict(work_ranks),
            classes=ledger['class_counts'],compact_bytes=(p/'COMPACT.npz').stat().st_size,rich_bytes=(p/'EXACT.json.gz').stat().st_size,
            tree_peak_rss=perf['observed_tree_peak_rss'],proof_cache_hits=0,root_hits='Not instrumented in the frozen run'))
    result=dict(rows=rows,aggregate_timing=dict(times),method_counts=dict(methods),rank_counts=dict(ranks),classes=dict(class_counts),
        certificate_seconds=sum(r['certificate_seconds'] for r in rows),slowest=max(rows,key=lambda x:x['certificate_seconds']),
        semantics='Per-layer elapsed times are wall time. details/kernel and method times sum worker CPU wall intervals and overlap across eight processes. Topological incidence alone never removes a required pair.',
        current_batch=8,current_native_granularity='One full required population per native screen call',
        prior_proofs='No committed-vs-committed proof payload is loaded by certify_immediate; complete prefix geometry and BVH are rebuilt per call.')
    save('BASELINE_PROFILE.json',result);print(json.dumps({k:v for k,v in result.items() if k not in ('rows','slowest')},indent=2));print('SLOWEST',result['slowest']['layer'],result['slowest']['certificate_seconds'])
if __name__=='__main__':run()
