"""Measured distributions; all times are explicitly worker sums or wall time."""
from task4k_common import *
import statistics

def aggregate(rows):
    functions={}
    for row in rows:
        for name,values in row['functions'].items():
            dst=functions.setdefault(name,collections.Counter());dst.update(values)
    caches={}
    for row in rows:
        for name,values in row['pure_cache'].items():caches.setdefault(name,collections.Counter()).update(values)
    adapters={}
    for row in rows:
        for name,values in row['adapter_cache'].items():adapters.setdefault(name,collections.Counter()).update(values)
    return dict(cases=len(rows),worker_seconds=sum(r['execution_seconds'] for r in rows),cpu_seconds=sum(r['cpu_seconds'] for r in rows),gc_seconds=sum(r['gc_seconds'] for r in rows),serialization_seconds=sum(r['serialization_seconds'] for r in rows),max_case_seconds=max(r['execution_seconds'] for r in rows),slowest_case=max(rows,key=lambda r:r['execution_seconds'])['index'],functions=functions,pure_cache=caches,adapter_cache=adapters,root_key_hits=sum(r['root_keys_hits'] for r in rows),root_key_misses=sum(r['root_keys_misses'] for r in rows))

def run():
    before=read(OUT/'BASELINE_CASES.json');after=read(OUT/'OPTIMIZED_CASES.json')
    # Older baseline rows predate the explicit pure-cache diagnostics field.
    for row in before:row.setdefault('pure_cache',{})
    assert len(before)==len(after)==1938
    for a,b in zip(before,after):
        for key in ('index','replay','case_identity','result_hash','comparison_trace','root_trace'):assert a[key]==b[key],('OptimizedCADMismatch',key,a['index'])
    replay={label:[aggregate([r for r in rows if r['replay']==i]) for i in range(3)] for label,rows in [('before',before),('after',after)]}
    case_rows=[]
    for i in range(646):
        item=dict(index=i)
        for label,rows in [('before',before),('after',after)]:
            values=[r['execution_seconds'] for r in rows if r['index']==i]
            item[label]=dict(seconds=values,minimum=min(values),maximum=max(values),median=statistics.median(values),spread=max(values)-min(values),ratio=max(values)/min(values))
        case_rows.append(item)
    concentration={}
    for label,rows in [('before',before),('after',after)]:
        phase=[r for r in rows if r['replay']==0];ordered=sorted(phase,key=lambda r:r['execution_seconds'],reverse=True);total=sum(r['execution_seconds'] for r in phase)
        concentration[label]=dict(top=[dict(index=r['index'],seconds=r['execution_seconds'],cpu_seconds=r['cpu_seconds'],worker=r['group']) for r in ordered[:20]],shares={str(n):sum(r['execution_seconds'] for r in ordered[:n])/total for n in (1,5,10,20,100)},layer_worker_seconds={str(layer):sum(r['execution_seconds'] for r in phase if read_cases[r['index']]['layer']==layer) for layer in range(1,21)})
    scientific=[{key:r[key] for key in ('index','case_identity','result_hash','comparison_trace','root_trace')} for r in after if r['replay']==0]
    save('CASE_VARIANCE.json',case_rows);save('CAD_SUMMARY.json',dict(replays=replay,concentration=concentration,baseline_wall=read(OUT/'BASELINE_SYSTEM.json')['seconds'],optimized_wall=read(OUT/'OPTIMIZED_SYSTEM.json')['seconds'],peak_rss=max(read(OUT/'BASELINE_SYSTEM.json')['peak_rss'],read(OUT/'OPTIMIZED_SYSTEM.json')['peak_rss']),phase_timing_note='Worker cohorts run sequential cold/warm/warm; different cohorts can overlap phases. Phase totals are summed worker execution, not elapsed wall time. Instrumentation adds overhead.'))
    save('CAD_EQUIVALENCE.json',dict(cases=646,executions_each=1938,proofs=True,ordered_comparisons=True,ordered_root_records=True,rank=True,witnesses=True,scientific_hash=scientific_hash(scientific),timing_excluded=True))

if __name__=='__main__':
    read_cases=read(OUT/'CAD_CASES.json');run()
