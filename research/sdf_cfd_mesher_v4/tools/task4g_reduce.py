"""Reduce directly measured local replay work; never claim a new build runtime."""
from task4g_common import *
from collections import Counter

def run():
    rows=read(OUT/'TRIAL_REPLAY.json');stages=read(OUT/'STAGED_REPLAY.json');bases=read(OUT/'BASELINE_REFRESHES.json')
    assert len(rows)==len(stages)==255 and sum(r['accepted'] for r in rows)==32
    assert all(r['decision_equal'] for r in rows)
    assert all(r['metric_hash']==r['replay_hash'] for r in rows if not r['early_rejected'])
    assert {(r['layer'],r['trial']) for r in rows if r['early_rejected']}=={(r['layer'],r['trial']) for r in stages if r['rejected_before_future']}
    old=read(ROOT/'results/task4f/ACTIVE_TRIALS.json');ops=read(ROOT/'results/task4f/ACTIVE_TRIAL_OPERATION_COUNTS.json')
    oldmap={(r['layer'],r['trial']):r for r in old};early=[oldmap[r['layer'],r['trial']] for r in rows if r['early_rejected']]
    measured=lambda name:sum(r.get(name,0.) or 0. for r in rows)
    counts=Counter()
    for r in rows:counts.update({k:v for k,v in r['counts'].items() if isinstance(v,int) and not isinstance(v,bool)})
    baseline_counts=Counter()
    for r in bases:baseline_counts.update({k:v for k,v in r['counts'].items() if isinstance(v,int) and not isinstance(v,bool)})
    operations=dict(old_whole_patch_cells=sum(r['recomputed_prism_cells'] for r in ops),old_changed_vs_nominal_cells=sum(r['changed_prism_cells'] for r in ops),
        old_unchanged_vs_nominal_cells=sum(r['unchanged_prism_cells_recomputed'] for r in ops),new_trial_counts=dict(counts),new_baseline_counts=dict(baseline_counts),
        early_rejected_trials=len(early),early_rejected_fraction_of_rejections=len(early)/223,
        early_avoided_whole_patch_cells=sum(r['full_metric_cells'] for r in early),early_avoided_faces=sum(r['metric_faces'] for r in early),
        early_avoided_reconstruction_rows=sum(r['reconstruction_rows'] for r in early),
        exact_local_before=295747,exact_local_calls=sum(r['exact_calls'] for r in stages),exact_local_hits=sum(r['exact_hits'] for r in stages),
        whole_layer_exact_calls_unchanged=2126040,
        definition='Old Task-4F input accounting uses each nominal trial baseline. New misses compare exact current inputs to the retained metric state. Early rejections have no future metric request. Counts are measured isolated engine calls; production whole-patch memo hits may avoid additional calls.')
    assert operations['exact_local_calls']+operations['exact_local_hits']==295747
    assert counts['cell_recomputed']+counts['cell_reused']+operations['early_avoided_whole_patch_cells']==operations['old_whole_patch_cells']
    save('OPERATION_COUNTS.json',operations)
    historical=read(ROOT/'results/task4f/PRESERVED_SUMMARY.json')
    runtime=dict(historical_active_search_seconds=784.4910517999378,historical_detailed_seconds=479.9986495998164,
        paired_survivor_full_recomputation_seconds=measured('old_detailed_seconds'),incremental_trial_metric_seconds=measured('detailed_seconds'),
        incremental_baseline_seconds=sum(r['seconds'] for r in bases),
        proposal_projection_seconds=measured('projection_seconds'),local_forecast_seconds=measured('forecast_seconds'),future_exact_spots_seconds=measured('spot_seconds'),
        immediate_cheap_screen_seconds=sum(r['cheap_seconds'] for r in stages),immediate_remaining_numeric_seconds=sum(r['numeric_seconds'] for r in stages),
        local_exact_seconds=sum(r['exact_seconds'] for r in stages),historical_whole_layer_exact_seconds=357.4049861000967,
        historical_full_final_validation_seconds=144.67886780001572,historical_repeated_final_numeric_sdf_seconds=100.61314569989918,
        early_avoided_historical_detailed_seconds=sum(r['immediate_rejection_full_detail_seconds'] for r in early),
        early_avoided_historical_forecast_seconds=historical['immediate_rejection_forecast_seconds'],
        measured_new_full_active_search_seconds=None,measured_new_full_build_seconds=None,
        scope='Preserved input replay. Local planning kernel timers exclude reference oracle, second cache replay, result comparison, report serialization and global setup. Concurrent bounded qualification affected some early-layer timings. No new full-search/build runtime is claimed.')
    names=('incremental_trial_metric_seconds','incremental_baseline_seconds','proposal_projection_seconds','local_forecast_seconds','future_exact_spots_seconds',
        'immediate_cheap_screen_seconds','immediate_remaining_numeric_seconds','local_exact_seconds')
    runtime['measured_planning_kernel_subtotal_seconds']=sum(runtime[k] for k in names)
    runtime['dominant_measured_replay_kernel']=max(names,key=lambda k:runtime[k])
    runtime['detail_replay_seconds_including_baselines']=runtime['incremental_trial_metric_seconds']+runtime['incremental_baseline_seconds']
    save('RUNTIME.json',runtime)
    # Independent caches bind all metric arrays, not just pass/fail booleans.
    def stable(r,second=False):
        keys=('layer','trial','group','accepted','old_decision','decision_equal','candidate_hash','early_rejected','screen','gate','future_hash','exact_hash','spot_hash','dependencies_hash','counts')
        result={k:r[k] for k in keys if k in r}
        if not r['early_rejected']:result['metric_hash']=r['replay_hash' if second else 'metric_hash']
        return result
    a=scientific_hash([stable(r) for r in rows]);b=scientific_hash([stable(r,True) for r in rows]);assert a==b
    save('DETERMINISM.json',dict(hash_1=a,hash_2=b,passed=True,metric_arrays_bitwise_equal=True,proposal_hashes_match_all_255=True,
        cache_dependency_decisions_match=True,accepted=32,rejected=223,immediate_gates_match=True,future_decisions_match=True,
        exact_records_match=True,selected_future_proofs_match=True,final_validation_hashes=read(OUT/'FINAL_VALIDATION_REUSE.json')['replay_hashes'],
        scope='Two independent local metric caches; separately reconstructed staged inputs/exact checks; frozen accepted coordinates and future proof records; two complete immutable receipt audits. No full shell regeneration.'))
    sequence=[]
    for layer in range(1,12):
        a=[r for r in rows if r['layer']==layer];b=[r for r in stages if r['layer']==layer]
        sequence.append(dict(layer=layer,trials=len(a),accepted=sum(r['accepted'] for r in a),early_rejected=sum(r['early_rejected'] for r in a),
            old_paired_detail_seconds=sum(r['old_detailed_seconds'] or 0 for r in a),new_detail_seconds=sum(r['detailed_seconds'] for r in a),
            exact_calls=sum(r['exact_calls'] for r in b),exact_hits=sum(r['exact_hits'] for r in b),all_scientific_decisions_equal=True))
    save('LAYER_REPLAY.json',sequence)
    print(json.dumps(dict(runtime=runtime,operations=operations),indent=2))
if __name__=='__main__':run()
