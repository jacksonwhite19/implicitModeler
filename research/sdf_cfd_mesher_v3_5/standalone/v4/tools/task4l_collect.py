"""Compact completed-stage accounting; no scientific kernels are executed."""
from task4l_common import *
from collections import Counter

def collect():
    import task4e_collect as old
    old.OUT=OUT;old.RUN=RUN;old.save=save
    result=old.collect();counts=Counter();kernels=Counter();observations=[]
    for path in sorted((OUT/'instrument').glob('*.json')):
        obj=read(path);layer=int(path.stem[:2]);detail=Counter();stage_counts=Counter()
        for r in obj['rows']:
            kernels[r['kernel']]+=r['seconds'];detail[r['kernel']]+=r['seconds']
            if r['kernel']=='detailed_future':
                c=r['cache'];counts['detailed_requests']+=1
                counts['detailed_exact_hits']+=int(c.get('exact_patch_hit',False));counts['detailed_refreshes']+=int(not c.get('exact_patch_hit',False))
                for key,value in c.items():
                    if isinstance(value,(int,float)) and not isinstance(value,bool):stage_counts[key]+=value;counts['metric:'+key]+=value
        for k,v in obj['exact_local'].items():counts['local_exact:'+k]+=v
        observations.append(dict(stage=path.stem,seconds=dict(detail),metric_counts=dict(stage_counts),exact_local=obj['exact_local']))
    for row in result['rows']:
        layer=row['layer'];instrument=[x for x in observations if int(x['stage'][:2])==layer]
        row['detailed_seconds']=sum(x['seconds'].get('detailed_future',0) for x in instrument)
        row['local_exact_seconds']=sum(x['seconds'].get('local_exact',0) for x in instrument)
        row['future_spot_seconds']=sum(x['seconds'].get('future_spots',0) for x in instrument)
        h=optional(RUN/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'),{})
        row['early_rejections']=sum(bool(x.get('rejection_stage')) for x in h.get('history',[]))
        # First-layer numeric failures already skipped future work before 4G.
        row['trials_without_future']=sum(not x.get('accepted') and ('first_after' not in x and 'first_future_after' not in x) for x in h.get('history',[]))
        counts['trials']+=row['trials'];counts['accepted']+=row['accepted'];counts['rejected']+=row['rejected'];counts['early_rejections']+=row['early_rejections']
        if row['committed']:counts['whole_exact_calls']+=106302;counts['whole_exact_materializations']+=3*106302
        counts['immediate_pairs']+=row.get('certificate_pairs') or 0;counts['immediate_frozen_kernel_calls']+=row.get('exact_calls') or 0
        times=optional(RUN/f'LAYER_{layer:02}_TIMINGS.json',{})
        for c in times.get('certificate_stages',[]):
            for name,value in c.get('exact_methods',{}).items():counts['immediate_method:'+name]+=value
        p=row.get('planner') or {}
        for family in ('spatial','selection','future_proofs'):
            for k,v in p.get(family,{}).items():
                if isinstance(v,(int,float)):counts['planner:'+family+':'+k]+=v
    result['stages']['active_detailed_seconds']=kernels['detailed_future']
    result['stages']['future_spot_checks_seconds']=kernels['future_spots']
    result['stages']['local_exact_seconds']=kernels['local_exact']
    result['stages']['screen_seconds']=sum(v for k,v in kernels.items() if k.startswith('screen:'))
    comp=optional(OUT/'SCIENTIFIC_COMPARISON.json',{})
    result['stages']['final_comparison_seconds']=comp.get('seconds')
    result['operation_counts']=dict(counts);result['instrumentation']=observations
    result['overlap_note']='Search includes detailed metrics, local exact signs, projections, screens and spots. Final validation includes the completeness audit. Wall stages in OPERATIONS are disjoint. Counter observations are process-local; retained proof cache rows survive, hit counters reset on process restart.'
    certificates=[]
    for row in result['rows']:
        layer=row['layer'];timing=optional(RUN/f'LAYER_{layer:02}_TIMINGS.json',{})
        for entry in timing.get('certificate_stages',[]):
            certificates.append(dict(layer=layer,**entry))
    save('IMMEDIATE_CERTIFICATION.json',certificates)
    save('ACTIVE_PASSIVE.json',[{k:r.get(k) for k in ('layer','active_vertices','active_fraction','dependency_columns','atlas_active_fraction','atlas_active_columns_fraction','atlas_invalidation')} for r in result['rows']])
    save('LAYER_PROGRESS.json',result);save('OPERATION_COUNTS.json',dict(counts));return result
if __name__=='__main__':print(collect()['stages'])
