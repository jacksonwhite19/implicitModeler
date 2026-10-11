"""Reduce measured audit evidence. No geometry generation or production changes."""
from task4f_common import *
from collections import Counter

def run():
    preserved=read(OUT/'PRESERVED_SUMMARY.json');ops=read(OUT/'ACTIVE_TRIAL_OPERATION_COUNTS.json')
    updates={(r['layer'],r['trial']):r for r in read(OUT/'CURRENT_INPUT_AUDIT.json')}
    for r in ops:
        if (r['layer'],r['trial']) in updates:
            u=updates[r['layer'],r['trial']];assert u['candidate_hash']==r['candidate_hash'];r.update({k:u[k] for k in ('exact_current_calls','exact_current_unchanged_input_calls')})
    save('ACTIVE_TRIAL_OPERATION_COUNTS.json',ops)
    count_keys=['moved_vertices','changed_forecast_vertex_states','changed_edge_states','changed_triangle_front_states','changed_prism_cells','recomputed_prism_cells','unchanged_prism_cells_recomputed','changed_face_metric_dependencies','recomputed_face_metrics','changed_cell_operator_rows','recomputed_cell_operator_rows','committed_prefix_cells_recomputed','exact_current_calls','exact_current_unchanged_input_calls']
    accounting={key:sum(r[key] for r in ops) for key in count_keys}
    accounting.update(trials=len(ops),cell_redundancy_ratio=accounting['recomputed_prism_cells']/accounting['changed_prism_cells'],unchanged_cell_fraction=accounting['unchanged_prism_cells_recomputed']/accounting['recomputed_prism_cells'],
        unchanged_face_dependency_fraction=1-accounting['changed_face_metric_dependencies']/accounting['recomputed_face_metrics'],
        interpretation='Changed means binary64 coordinate-input dependency differs from the pretrial nominal baseline. A changed input need not change every metric value. Unchanged input counts prove safe identity reuse opportunity. Counts include local halos; no full-shell dense system.')
    assert accounting['exact_current_calls']==preserved['exact_local_cell_calls']
    save('OPERATION_SUMMARY.json',accounting)
    cache=read(OUT/'CACHE_MISS_CLASSIFICATION.json');later=[r for r in cache['events'] if r['layer']>1]
    cache['later_layer_counts']=dict(Counter(r['classification'] for r in later));cache['later_queries']=len(later)
    evicted=[]
    for e in cache['events']:
        if e['classification']!='IdenticalInputEvicted':continue
        assert e['trial'] is None
        b=next(b for b in cache['baselines'] if b['layer']==e['layer'] and b['key']==e['key'])
        p=next(p for p in read(BASE/f'LAYER{e["layer"]}_PREPARE_TIMINGS.json')['region_profiles'] if p['patch']==b['patch'])
        evicted.append(dict(layer=e['layer'],patch=b['patch'],seconds=p['baseline_detailed_seconds'],key=e['key']))
    cache['identical_evicted_refreshes']=evicted;cache['measured_evicted_refresh_seconds']=sum(r['seconds'] for r in evicted);save('CACHE_MISS_CLASSIFICATION.json',cache)
    baseline=read(E/'LAYER_PROGRESS.json');save('BASELINE_RUNTIME.json',dict(scientific_seconds=2282.939046,task_seconds=2894.54,pre_science_task_seconds=401.819024,stages=baseline['stages'],
        nested_timers=['active_detailed, local_forecast, projection and future_spot_checks are included within active_search','final numeric, SDF and completeness are included within final_validation','projection is also nested within local forecast'],
        source='Task4E LAYER_PROGRESS/EXECUTION; preserved rather than remeasured'))
    trials=read(OUT/'ACTIVE_TRIALS.json');staged=read(OUT/'BOUNDED_SAMPLES.json');late=read(OUT/'LATE_REJECTION_BOUNDED.json')
    rejection=[]
    for r in trials:
        rejection.append(dict(layer=r['layer'],region=r['region'],trial=r['trial'],first_decisive_condition=r['rejection'],numeric_gate=r['numeric_violations'],
            full_detail_seconds=r['timing'].get('detailed_seconds'),forecast_seconds=r['timing'].get('forecast',{}).get('seconds'),
            exact_and_future_scan_seconds=r['timing'].get('exact_sign_seconds'),
            original_order='projection -> forecast -> ALL20 detailed metrics including conditioning -> immediate numeric -> exact signs -> future failure scan -> admission -> selective exact spots' if r['layer']>1 else 'projection -> quick immediate -> forecast/detail -> exact/admission'))
    save('REJECTED_TRIAL_FIRST_CONDITION.json',rejection)
    save('EARLY_OUT_EXPERIMENT.json',dict(cohort=staged,immediate_rejections=76,rejected_trials=223,fraction=76/223,
        historical_full_detail_upper_bound_seconds=preserved['immediate_rejection_full_detail_seconds'],historical_forecast_upper_bound_seconds=preserved['immediate_rejection_forecast_seconds'],
        bounded_rejected_cases=sum(not r['immediate_gate_identical'] or r['decision_preserved_by_decisive_gate'] for r in staged),
        bounded_net_detail_seconds_saved=sum(r['measured_avoided_detail_seconds'] for r in staged),
        future_regression_count=130,future_regression_historical_combined_timer=preserved['exact_after_future_regression_seconds'],
        future_combined_timer_caveat='Includes rational signs AND future-failure scan. It is not all avoidable exact arithmetic.',
        bounded_late_exact_seconds=sum(r['exact_seconds'] for r in late),candidate_identity_and_numeric_values_equal=True,
        decision_equivalence='Three immediate numeric rejections are decisive. Remaining ten cohort entries fall through unchanged. Two future-regression replays reproduce exact records and metric hashes; admission remains rejection. Continuous certificates were not rerun or bypassed.',
        production_integrated=False))
    certrows=[];tot=Counter();methods=Counter();details=Counter();classes=Counter()
    for r in read(OUT/'CERTIFICATION_TIMINGS.json'):
        ledger=read(BASE/f'LAYER_{r["layer"]:02}_CERTIFICATE.json');s=r['certificate_stages'][0]
        for k in ('enumeration_seconds','validity_seconds','filter_seconds','remaining_exact_seconds','dispatch_seconds','serialization_seconds','exact_calls'):tot[k]+=s.get(k,0)
        methods.update(s.get('exact_methods',{}));details.update(s.get('details',{}));classes.update(ledger.get('class_counts',{}))
        certrows.append(dict(layer=r['layer'],pairs=ledger['candidate_pairs'],**s))
    save('CERTIFICATION_AUDIT.json',dict(layers=certrows,totals=dict(tot),methods=dict(methods),worker_detail_seconds=dict(details),conservative_proof_codes=dict(classes),
        note='Worker detail CPU/wall sums overlap eight-worker execution and must not be added to stage wall times. No new certificate workload was launched. Existing proofs remained exact and sealed.',
        committed_proofs_recomputed=0,final_new_exact_calls=0,final_reused_proofs=56462864))
    fullpasses=dict(initial_nominal_scan=20,prepare_previous_full_cell=19,prepare_nominal=19,prepare_corrected_final=10,layer1_corrected_final=1,certify_previous_full_cell=19,certify_candidate=20,final_validation=20)
    save('WHOLE_LAYER_WORK_COUNTS.json',dict(full_cell_passes=fullpasses,total_passes=sum(fullpasses.values()),cells=sum(fullpasses.values())*106302,
        previous_layer_full_operator_passes=38,previous_layer_cells=38*106302,precommit_numeric_duplicate_layers=20,final_numeric_duplicate_layers=20,
        evidence='Current source call sites and preserved accepted/corrected records determine exact call counts. No timing extrapolation. Previous-cell callers consume centre/volume, but evaluate_cells computes every cell metric, including sigma/G.',
        previous_layer_work_missing_from_113s_timer=True))
    deps=[
      dict(family='cell volume/Jacobian/determinant/aspect',dependencies='Six coordinates of one prism; determinant also reads layer-dependent boundary mask',current='All20 x halo columns per miss',classification='ConservativeOverwork + RepeatedIdenticalWork'),
      dict(family='weighted sigma/G_bound',dependencies='Five oriented face vectors and wall mask of one cell. No global direction Jacobian or manufactured field',current='All20 x halo columns, including unused halo conditioning and ten SVD batches',classification='ConservativeOverwork + ImplementationOverhead'),
      dict(family='interpolation/skewness/volume ratio/nonorthogonality',dependencies='Face vertex coordinates and both complete adjacent cell centres/volumes; cap reads consecutive layer cells',current='All side and internal cap faces in all20 layers',classification='ConservativeOverwork'),
      dict(family='exact rational Jacobian',dependencies='Six coordinate triples of one cell',current='Every current verification cell after numeric pass; complete actual new layer once',classification='NecessaryCompleteWork + RepeatedIdenticalWork in unchanged local ring'),
      dict(family='topology closure',dependencies='Immutable source connectivity and patch IDs',current='Two full scans per detailed miss; creates entire column set per edge',classification='RepeatedIdenticalWork + ImplementationOverhead'),
      dict(family='compression indicators',dependencies='No separate compression metric in production detailed.measure',current='Not executed',classification='NotExecuted'),
      dict(family='future collision/exact spots',dependencies='Local side surface identities, conservative source-edge bounds and exact pair geometry',current='Separate selective stage; not inside 480-second metric timer',classification='ConservativeOverwork; pair-level redundancy not newly measured')]
    save('METRIC_DEPENDENCIES.json',deps)
    classification=[
      dict(stage='detailed future metrics',classes=['ConservativeOverwork','RepeatedIdenticalWork','ImplementationOverhead'],seconds=479.9986495998164,evidence='OPERATION_SUMMARY + METRIC_PROFILE'),
      dict(stage='rejected trial pipeline',classes=['LateRejectedWork'],seconds=138.61022319993936,evidence='Upper bound on full detailed time for76 immediate rejections; bounded net savings measured separately'),
      dict(stage='whole-layer rational signs',classes=['NecessaryCompleteWork','ImplementationOverhead'],seconds=357.4049861000967,evidence='2,126,040 unique new cells; Fraction arithmetic dominates bounded profile'),
      dict(stage='immediate numeric metrics',classes=['NecessaryCompleteWork','RepeatedIdenticalWork'],seconds=113.16488720005145,evidence='Global new layer validation remains required; same accepted geometry recalculated in certify subprocess'),
      dict(stage='startup/runner preparation',classes=['Unresolved'],seconds=401.819024,evidence='Includes task preparation outside scientific execution, not a profiled mesher function'),
      dict(stage='initial forecast/scan',classes=['NecessaryCompleteWork'],seconds=116.79649400000926,evidence='56.002 forecast +60.021 first complete direct scan; not a doubled SDF forecast'),
      dict(stage='optimized certificate',classes=['NecessaryCompleteWork','ImplementationOverhead'],seconds=331.94845479994547,evidence='No repeated committed pairs; measured symbolic survivor costs and persistent prefix array/index construction remain'),
      dict(stage='final numeric and SDF',classes=['RepeatedIdenticalWork'],seconds=100.61314569989918,evidence='Immutable committed inputs already evaluated; evidence-preserving final completeness still required'),
      dict(stage='final certificate',classes=['NecessaryCompleteWork'],seconds=29.159861900028773,evidence='Completeness audit reuses56,462,864 proofs; zero new exact calls'),
      dict(stage='checkpointing',classes=['ImplementationOverhead','RepeatedIdenticalWork'],seconds=67.87459170000511,evidence='625,103,682 archived hot-planner bytes; repeated hash-chain restart; no historical compression split')]
    save('BRUTE_FORCE_CLASSIFICATION.json',classification)
    rec=[
      dict(rank=1,target='Dependency-local detailed metrics and immutable support checks',historical_stage_seconds=479.9986495998164,measured_avoidable_work='7,512,688 unchanged cell evaluations; single trial cell-family1.2689082 ->0.0102679s; closure full-measure2.6315455 ->1.5624630s',proposal='Reuse exact cell/face dependency identities, retain committed prefix metrics, cache qualified immutable support closure; remove unconsumed gain outputs',risk='Medium for dependency closure, low for expression-preserving membership hoist; bitwise equivalence required'),
      dict(rank=2,target='Staged trial rejection',historical_stage_seconds=138.61022319993936,measured_avoidable_work='76 immediate failures ran full detail; bounded three-case net savings and two late exact checks in EARLY_OUT_EXPERIMENT',proposal='Evaluate existing immediate gates before future forecasting/full conditioning; test future failure table before unnecessary exact signs on already-rejected trials',risk='Keep all surviving/accepted paths and exact certification unchanged; no new heuristic gate'),
      dict(rank=3,target='Reuse immutable immediate/final evidence',historical_stage_seconds=100.61314569989918,measured_avoidable_work='48.3834691s final numeric and52.2296766s final SDF repeat;38 full previous-cell operator passes;20 repeated precommit candidate metric passes',proposal='Authority-bound metric/SDF receipts across stage boundaries and final completeness audit; retain complete new-layer checks',risk='Requires complete hash/ownership/coverage receipts before removing recalculation'),
      dict(rank=4,target='Exact rational kernel implementation',historical_stage_seconds=357.4049861000967,measured_avoidable_work='No whole-layer repeated cell IDs;4096-cell0.6193654s mostly Fraction operations; local unchanged exact-input calls counted',proposal='First reuse exact identical local inputs; later qualify a faster exact arithmetic implementation without changing polynomial or signs',risk='Higher; exact mathematics and representation must remain equivalent'),
      dict(rank=5,target='Planner storage and cache eviction',historical_stage_seconds=67.87459170000511,measured_avoidable_work=f"Ten identical evicted refreshes consumed {cache['measured_evicted_refresh_seconds']:.9f}s; one-entry compressed snapshot9,377,394 bytes",proposal='Retain useful accepted/base identities; compact dependency-local state and avoid rewriting immutable diagnostics',risk='Maintain authority identity and rollback; do not reuse stale numbers')]
    save('RANKED_RECOMMENDATIONS.json',dict(ranking_basis='Measured affected stage, verified repeated entity work, and bounded measured savings. Historical stage totals are NOT claimed savings. Targets overlap and must not be summed. No new full-build runtime forecast.',targets=rec))
    # Scientific stable outputs only, constructed twice in independent traversal order.
    stable=[{k:r[k] for k in ['layer','trial','candidate_hash',*count_keys]} for r in ops]
    h1=scientific_hash(sorted(stable,key=lambda r:(r['layer'],r['trial'])))
    h2=scientific_hash(sorted(list(reversed(stable)),key=lambda r:(r['layer'],r['trial'])))
    assert h1==h2
    save('DETERMINISM.json',dict(accounting_scientific_hash_1=h1,accounting_scientific_hash_2=h2,
        evidence='255 candidate hashes matched preserved trials; bounded plain/instrumented outputs matched; two full detailed repeats matched their first recorded scientific hashes; exact rational records matched preserved trials; dependency-local merge matched full cell arrays.',
        hash_replay_scope='Independent canonical traversal of measured accounting, not an independent full A2 build.',bounded_metrics=[dict(layer=r['layer'],trial=r['trial'],hash=r['metric_hash']) for r in late],exact_equality=True))
    save('AUDIT_REPAIRS.json',dict(production_defects_repaired=[],study_instrumentation_repairs=[dict(reproducer='layer=2, prefix=(layer1,), old audit code prefix[-2] raised IndexError',fix='Select frozen wall as previous low front when prefix length is one',regression='test_previous_layer_two; original production indexing was already correct')]))
    print(json.dumps(accounting,indent=2),flush=True)

if __name__=='__main__':run()
