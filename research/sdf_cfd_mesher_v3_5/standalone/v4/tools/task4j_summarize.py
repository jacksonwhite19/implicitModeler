"""Derive report tables from measured data, never inferred full-build times."""
from task4j_common import *

def aggregate(rows):
    values=collections.Counter();details=collections.Counter()
    for row in rows:
        p=row['performance'];values.update({k:v for k,v in p.items() if isinstance(v,(int,float))});details.update(p['details'])
    return dict(values),dict(details)

def run():
    baseline=read(OUT/'BASELINE_PROFILE.json');a=read(OUT/'REPLAY_A.json');b=read(OUT/'REPLAY_B.json');av,ad=aggregate(a);bv,bd=aggregate(b)
    audit=read(OUT/'FINAL_AUDIT.json');det=read(OUT/'DETERMINISM.json');native=read(OUT/'BATCH_EXPERIMENT.json');roots=read(OUT/'ROOT_PREPARATION_EXPERIMENT.json');storage=read(OUT/'STORAGE_EXPERIMENT.json');batch=read(OUT/'EXACT_BATCH_EXPERIMENT.json')
    profile=read(OUT/'BOUNDED_KERNEL_PROFILE.json');proof_cost=[]
    for method in sorted(set(r['method'] for r in profile)):
        rows=[r for r in profile if r['method']==method];seconds=sum(r['seconds'] for r in rows)
        proof_cost.append(dict(method=method,profiled_calls=len(rows),instrumented_seconds=seconds,mean_seconds=seconds/len(rows),maximum_seconds=max(r['seconds'] for r in rows),canonical_seconds=sum(r['canonical_seconds'] for r in rows),canonical_bytes=sum(r['canonical_bytes'] for r in rows)))
    save('PROOF_CLASS_COST.json',dict(bounded_instrumented_calls=proof_cost,native=read(OUT/'NATIVE_CLASS_PROFILE.json'),frozen_full_run_worker_times=baseline['aggregate_timing'],optimized_worker_times=ad,note='Worker times overlap; cProfile times include profiling overhead. No whole-run tail latency is inferred.'))
    table=[]
    for old,new in zip(baseline['rows'],a):
        p=new['performance'];path=OUT/'replay_A'/f"layer_{new['layer']:02d}"
        table.append(dict(layer=new['layer'],required_pairs=new['pairs'],old_seconds=old['certificate_seconds'],new_seconds=p['seconds'],filter_seconds=p['filter_seconds'],exact_phase_seconds=p['remaining_exact_seconds'],serialization_seconds=p['serialization_seconds'],new_frozen_calls=p['exact_calls'],method_counts=p['exact_methods'],root_key_hits=p['details'].get('root_keys:hits',0),root_preparation_hits=p['details'].get('root_preparation:hits',0),proof_cache_hits=0,topology_only_exclusions=0,proof_classes=old['classes'],old_proof_bytes=old['compact_bytes']+old['rich_bytes'],new_proof_bytes=sum(q.stat().st_size for q in path.iterdir()),cumulative_sampled_rss=new['peak_rss']))
    save('PER_LAYER_COMPARISON.json',table)
    save('RUNTIME_COMPARISON.json',dict(old_certificate_seconds=baseline['certificate_seconds'],old_adapter_seconds=baseline['aggregate_timing']['seconds'],new_A=av,new_B=bv,new_worker_details_A=ad,new_worker_details_B=bd,old_final_audit_seconds=29.01,new_final_audit_seconds=[r['seconds'] for r in audit],ratio=baseline['certificate_seconds']/av['seconds'],reduction_percent=100*(1-av['seconds']/baseline['certificate_seconds']),new_worst=max(table,key=lambda r:r['new_seconds']),old_worst=max(table,key=lambda r:r['old_seconds']),peak_sampled_rss=max(r['peak_rss'] for r in a+b)))
    save('DISPATCH_AUDIT.json',dict(old_dispatch_seconds=baseline['aggregate_timing']['dispatch_seconds'],new_dispatch_seconds=av['dispatch_seconds'],native_batches=native,exact_batch_sizes=batch,storage=storage,root_preparation=roots,findings=['Conservative proofs already used one native batch; no per-pair Python dispatch existed there.','Contiguous parallel native slices reduce elapsed screen time without changing individual operations.','Exact jobs remain eight pairs; larger jobs increased measured slow-layer wall time.','No rank preclassification or frozen mathematical dispatch was changed.','Existing compact transactions already avoid per-pair rich records for conservative proofs.','No prior proof payload is loaded or rebound on a new transaction.','Prefix geometry and BVH reconstruction remain measured implementation work.']))
    lines=['| Layer | Pairs | Old s | New s | Frozen calls | Root preparation hits |','|---:|---:|---:|---:|---:|---:|']
    lines += [f"| {r['layer']} | {r['required_pairs']:,} | {r['old_seconds']:.3f} | {r['new_seconds']:.3f} | {r['new_frozen_calls']:,} | {r['root_preparation_hits']:,} |" for r in table]
    (OUT/'LAYER_TABLE.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
if __name__=='__main__':run()
