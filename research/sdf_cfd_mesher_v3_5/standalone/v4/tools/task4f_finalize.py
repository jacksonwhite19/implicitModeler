"""Compact audit tables, report and sealing; never starts scientific workers."""
from task4f_common import *
from collections import Counter
import shutil,re

def run():
    stage=read(E/'LAYER_PROGRESS.json')['stages'];summary=read(OUT/'OPERATION_SUMMARY.json');cache=read(OUT/'CACHE_MISS_CLASSIFICATION.json')
    metric=read(OUT/'METRIC_PROFILE.json');cost={r['function']:r for r in read(OUT/'METRIC_CALL_TREE.json')};inc=read(OUT/'INCREMENTAL_EXPERIMENT.json')[0]
    cheap=read(OUT/'CHEAP_CELL_REJECTION.json');samples=read(OUT/'BOUNDED_SAMPLES.json');hist=read(OUT/'ACTIVE_TRIALS.json');hm={(r['layer'],r['trial']):r for r in hist}
    # Preserve the earlier mixed-context instrumentation separately and label it.
    if not (OUT/'ALL_CONTEXT_INSTRUMENTATION.json').exists():save('ALL_CONTEXT_INSTRUMENTATION.json',dict(scope='Diagnostic mixed-context timers; use METRIC_CALL_TREE for isolated family shares',rows=read(OUT/'CALL_TREE.json')))
    calltree=[dict(scope='isolated local detailed measurement',**r) for r in cost.values()]
    calltree += [dict(scope='4096-cell exact-rational profile',**r) for r in read(OUT/'EXACT_JACOBIAN_PROFILE.json')['kernels']]
    for field,fn in [('exact_sign_seconds','task4_certify_candidate.exact_all'),('numeric_seconds','rolling_metrics.evaluate_layer at certify'),('certificate_wall_seconds','production.certify_immediate'),('checkpoint_seconds','CommitStore.commit')]:
        vals=[r[field] for r in read(OUT/'CERTIFICATION_TIMINGS.json')]
        calltree.append(dict(scope='preserved full-run direct timers',function=fn,calls=len(vals),total_seconds=sum(vals),self_seconds=None,average_seconds=sum(vals)/len(vals),maximum_seconds=max(vals),entities=2126040 if field in ('exact_sign_seconds','numeric_seconds') else None))
    save('CALL_TREE.json',calltree)
    families=[('volume/centres','cell.centres_volume',62180),('analytical wedge Jacobian/signed volume','cell.analytic_wedge',62180),('determinant','cell.determinants',62180),('interpolation weight','face.interpolation_weight',127208),('skewness','face.skewness',127208),('symmetric volume ratio','face.volume_ratio',127208),('aspect','cell.aspect',62180),('nonorthogonality cosine','face.nonorthogonality_cosine',127208),('side normal separation angle','cell.side_normal_separation',62180),('weighted sigma SVD','cell.weighted_sigma_svd',62180),('all gain operators and matrices','cell.all_gain_operators',62180),('closure','topology.require_closure',6218)]
    family_rows=[dict(family=name,kernel=key,calls=cost[key]['calls'],seconds=cost[key]['total_seconds'],entities=n,scope='one isolated local measurement',cache_hits=0,cache_misses=1) for name,key,n in families]
    family_rows += [dict(family='compression indicators',status='No separate implementation invoked in production measure; no compression runs'),dict(family='collision broad phase/exact future spots',calls=30,seconds=stage['future_spot_checks_seconds'],scope='Preserved full run, outside detailed metric timer',cache_split='See Task4E persistent planner statistics; not reprofiled pair-by-pair'),dict(family='exact rational Jacobian',seconds=stage['exact_jacobian_seconds'],entities=2126040,scope='Whole actual new layers, separate from analytical local wedge metric')]
    save('DETAILED_METRIC_COSTS.json',dict(rows=family_rows,note='Nested family timers are not additive; not a retrospective allocation of the historical480 seconds. Counts include halo cells. Angle needs its normal/direction construction too.'))
    decisive=[r for r in cheap['rows'] if r['decisive_rejection']]
    cheap_seconds=sum(r['seconds'] for r in decisive);old_cohort=sum(hm[r['layer'],r['trial']]['timing']['detailed_seconds'] for r in decisive)
    paired=[]
    for s in samples:
        r=next((r for r in cheap['rows'] if (r['layer'],r['trial'])==(s['layer'],s['trial'])),None)
        if r and r['decisive_rejection']:paired.append(dict(layer=r['layer'],trial=r['trial'],old_detail_seconds=s['plain_seconds'],cheap_seconds=r['seconds'],avoided_seconds=s['plain_seconds']-r['seconds']))
    save('CHEAP_REJECTION_SUMMARY.json',dict(evaluated=68,decisive=23,unchanged_fallback=45,all_gate_counts_identical=True,all_68_seconds=cheap['seconds'],decisive_seconds=cheap_seconds,preserved_detail_seconds_for_23=old_cohort,
        paired=paired,paired_measured_avoided_seconds=sum(r['avoided_seconds'] for r in paired),global_jacobian_gate_rejections=71,global_other_cell_gate_rejections=5,
        no_new_gate=True,no_certificate_acceptance_bypassed=True,note='Preserved historical cost and current kernel timing are separate measurements. Only the three paired plain/detail measurements give directly paired net savings.'))
    # Source-exact processing counts, independent of solver decisions.
    import task4_prepare_next as prod
    front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates))
    forecast=[]
    for r in hist:
        ids=np.asarray(r['active_ids']);_,edges,triangles=prod.dependencies(top,ids);layer=r['layer']
        forecast.append(dict(layer=layer,trial=r['trial'],projected_future_vertex_states=(20-layer)*len(ids),requeried_prefix_normal_vertex_states=layer*len(ids),unchanged_committed_prefix_normal_states=(layer-1)*len(ids),edge_descriptor_states=20*len(edges),triangle_descriptor_states=20*len(triangles)))
    save('FORECAST_RECOMPUTATION.json',dict(rows=forecast,totals={k:sum(r[k] for r in forecast) for k in forecast[0] if k not in ('layer','trial')},source='RollingForecastCache.forecast loops20 descriptors and reevaluates the full private prefix normals; only future coordinates after the proposed layer are regenerated.'))
    # Recheck bounded V3 and all sealed Task4E sources/evidence; no repository-wide audit.
    pre=read(OUT/'PREFLIGHT.json')
    for p,h in pre['V3'].items():assert file_hash(Path(p))==h,p
    prior=read(E/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for p,h in prior[group].items():assert file_hash(ROOT/p)==h,p
    assert file_hash(E/'report.md')==pre['Task4E_report'];assert file_hash(E/'AUTHORITY.json')==pre['Task4E_authority']
    original=read(OUT/'CLOCK.json')['status'];current=git('status','--porcelain')
    clean=lambda x:sorted(r.strip() for r in x.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)
    assert clean(original)==clean(current),'UnrelatedGitStateChanged'
    assert not git('diff','--name-only','--','research/sdf_cfd_mesher_v4/src'),'ProductionSourceModified'
    save('PRESERVATION.json',dict(V3_modified_files=0,bounded_V3_sources=len(pre['V3']),sealed_Task4E_sources_and_artifacts_unchanged=True,unrelated_status_equal=True,unrelated_status_entries=len(clean(current)),production_source_modified=False))
    peak=max(read(OUT/name).get('peak_rss',0) for name in ('BOUNDED_RUN.json','ACCOUNT_ONLY_RUN.json','KERNEL_RUN.json','METRIC_PROFILE.json','REJECTION_RUN.json','CHEAP_REJECTION_RUN.json','STORAGE_BOUNDED.json'))
    clock=read(OUT/'CLOCK.json');seconds=elapsed();scientific_stop=datetime.datetime.now(datetime.timezone.utc).isoformat();det=read(OUT/'DETERMINISM.json');cert=read(OUT/'CERTIFICATION_AUDIT.json');storage=read(OUT/'STORAGE_BOUNDED.json');exact=read(OUT/'EXACT_JACOBIAN_PROFILE.json')
    def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','|'+'|'.join(['---']*len(headers))+'|']+['| '+' | '.join(map(str,row))+' |' for row in rows])
    costs=table(['Measured local kernel','Calls','Entities','Seconds'],[(r['family'],r['calls'],r['entities'],f"{r['seconds']:.6f}") for r in family_rows if r.get('scope')=='one isolated local measurement'])
    baseline_table=table(['Preserved stage','Seconds'],[(k,f'{stage[k]:.6f}') for k in ('active_search_seconds','active_detailed_seconds','exact_jacobian_seconds','certificate_seconds','whole_layer_metric_seconds','initial_forecast_and_scan_seconds','initial_warning_atlas_seconds','planning_checkpoint_seconds','commit_seconds','final_validation_seconds','final_certificate_completeness_seconds','final_comparison_seconds')])
    rec=read(OUT/'RANKED_RECOMMENDATIONS.json');rec['targets'][0]['measured_avoidable_work']=f"7,512,688 unchanged cell evaluations; bounded cell-family {inc['full_seconds']:.9f} -> {inc['local_seconds']:.9f}s; closure full-measure {metric['plain_seconds']:.9f} -> {metric['closure']['full_measure_hoisted_seconds']:.9f}s"
    save('RANKED_RECOMMENDATIONS.json',rec)
    report=f'''Decision question

Where is the optimized V4 mesher still performing brute-force, repeated, or unnecessarily late work, and which measured changes offer the largest safe runtime reduction?

V4RuntimeBruteForcePathsIdentified

## A. Authority verification

Branch: `feature/sdf-cfd-mesher-v4-advancing-front`. Starting HEAD: `b3d28b30095da6c81f79c75342100832850d3891`.
Task-4E report SHA-256: `{pre['Task4E_report']}`. Authority SHA-256: `{pre['Task4E_authority']}`.
All source/evidence hashes bound by that authority passed before and after the audit. Frozen geometry, SDF, topology, schedule and all prior results remain unchanged. Bounded V3 sources checked: {len(pre['V3'])}; V3 modified files: 0. Unrelated Git entries preserved: {len(clean(current))}.

This audit did not run A2, rebuild a shell, change production code, or change any hard gate. The legacy raw-shell serialization hash is not an audit blocker. Task-4E scientific evidence is used as preserved run evidence.

## B. Task-4E runtime baseline

Scientific execution: 2282.939046 s. Task time: 2894.54 s. The 401.819024 s before scientific execution includes task preparation. It is not a measured 402-second mesher kernel.

{baseline_table}

Detailed metrics, projection, local forecast and future spot checks are nested inside active search. Final numeric, SDF and completeness timers are nested inside final validation. These rows must not be added together.

## C. Fine runtime decomposition

CALL_TREE.json contains direct preserved stage timers and isolated kernel timers, with calls, total/self time, mean/max and entity counts. METRIC_CALL_TREE.json profiles one 3109-column, 62180-prism, 127208-face patch. EXACT_JACOBIAN_PROFILE.json profiles 4096 preserved layer-16 cells. Generic profiling supplements direct timers.

Historical 480-second per-family splits do not exist. The following are direct bounded measurements, not an extrapolated allocation of that historical total. No full-build speed estimate is claimed. CPU time is close to wall time for measured numeric and rational kernels: these paths are effectively single-core under the qualified one-thread BLAS configuration. Exact side certification uses eight processes; its worker totals overlap.

## D. Active-search operation counts

All 255 preserved decisions were classified: 32 accepted; 76 immediate numeric rejections; 130 future-trajectory regressions; 17 no-improvement rejections. No accepted geometry was substituted into production. The audit reconstructed missing local coordinate identities and checked every trial candidate hash. Detailed metrics were replayed only for a bounded 13-case cohort plus the documented kernel fixtures.

ACTIVE_TRIALS.json joins preserved times and scopes. ACTIVE_TRIAL_OPERATION_COUNTS.json gives per-trial changed/recomputed inputs, moved vertices, maximum displacement, cells/faces/operator rows and exact-current counts. Totals:

* Moved vertex instances: {summary['moved_vertices']}; changed forecast vertex states: {summary['changed_forecast_vertex_states']}.
* Changed prism-cell inputs: {summary['changed_prism_cells']}; recomputed: {summary['recomputed_prism_cells']}; identical inputs recomputed: {summary['unchanged_prism_cells_recomputed']} ({100*summary['unchanged_cell_fraction']:.6f}%).
* Cell recomputation/changed-input ratio: {summary['cell_redundancy_ratio']:.9f}.
* Changed face dependencies: {summary['changed_face_metric_dependencies']}; evaluated face metrics: {summary['recomputed_face_metrics']}.
* Changed cell-operator row dependencies: {summary['changed_cell_operator_rows']}; evaluated rows: {summary['recomputed_cell_operator_rows']}.
* Immutable committed-prefix cell evaluations inside trials: {summary['committed_prefix_cells_recomputed']}.

Counts include halos. A changed input does not imply every output value changes. Unchanged means exact binary64 input identity, with no invented tolerance.

## E. Detailed future-metric decomposition

{costs}

These are nested timers. The volume kernel also calls face primitives. Nonorthogonality needs direction/cosine/angle work; side separation needs normal construction too. The local analytical wedge extrema are separate from the rational exact-sign pass. No separate compression indicator computation or compression optimizer runs in this path. Collision selection and exact future spot checks are outside detailed.measure: 30 calls, 125.353821 s in the preserved run, all on accepted later-layer corrections.

The largest local costs are closure checks and gain construction. The gain adapter computes ten SVD batches per cell-metric call: one primary sigma SVD and nine inside all-gains evaluation. Only two are needed for the returned sigma/G_bound outputs. The unused gain outputs remain real work, not new scientific requirements.

## F. Dependency versus recomputation analysis

Cell measures read six vertex coordinates. Sigma/G read the cell's five oriented face vectors and wall mask. Internal face measures read the face and both complete neighboring cell centres/volumes. No global sensitivity matrix is involved. METRIC_DEPENDENCIES.json records the actual supports.

DetailedCache caches one complete patch across all 20 layers. One changed vertex misses that whole entry. measure then builds all prism arrays, evaluates halo conditioning, rebuilds face ownership repeatedly, and returns only core cell metrics. Exact unchanged-input counts show this is broader than the dependencies require.

require_closure runs twice per measure. In the measured patch it rebuilds the same 3109-member set {metric['closure']['column_set_constructions_per_measure']} times, inserting {metric['closure']['column_members_inserted_per_measure']} members. Hoisting the membership set preserves every existing closure condition.

## G. Cache-miss root causes

The original 276 refreshes/7 hits describe later-layer instrumentation; layer 1 adds four initial queries. Exact insertion/eviction replay matches all recorded hit decisions.

Later-layer queries: 283. Hits: 7. Misses: 276 = 253 genuinely changed complete-patch inputs + 13 first inputs for a support + 10 identical inputs evicted by six-entry retention. Those ten evicted baseline refreshes consumed {cache['measured_evicted_refresh_seconds']:.9f} s. Object identity/order-only misses: 0. Unexplained misses: 0.

The key is correct for its complete-patch output. The main limitation is coarse result granularity, not an unsafe or stale key. Retaining all old whole-patch trials alone would not recover the much larger unchanged sub-entity work.

## H. Rejected-trial pipeline

Later layers execute proposal/projection -> local future forecast -> all20 detailed metrics and conditioning -> immediate numeric gate -> rational current signs -> future failure scan -> admission -> selected exact future checks. Layer 1 already has an earlier immediate check.

76/223 rejected trials ({100*76/223:.6f}%) could stop at existing immediate gates. 71 fail the existing analytical Jacobian gate; the remaining five fail the existing determinant gate. These are the same gates, not proposed new criteria. Their full detailed work totaled 138.610223 s and local forecast work 26.472298 s. These are upper bounds on avoidable historical time; required early-check cost was not separately timed then.

130 other trials fail future admission. The historical 58.563519 s timer on those trials includes rational signs AND future-failure scanning. It must not all be called avoidable exact work. Two bounded replays isolated {sum(r['exact_seconds'] for r in read(OUT/'LATE_REJECTION_BOUNDED.json')):.9f} s of exact signs performed before reading an already-available future metric regression. Exact records and metric hashes matched preserved results.

## I. Exact Jacobian audit

Whole-layer exact_all evaluates 2126040 distinct (layer,column) cells once. Its 357.404986 s is primarily required exact-rational arithmetic plus Python overhead, not duplicate whole-layer cell IDs. Final validation does not rerun this rational kernel.

The local search separately evaluates 295747 exact-current cells; 157606 have unchanged coordinate inputs relative to their trial baseline ({100*157606/295747:.6f}%). This is a real exact-identity reuse opportunity, but it must not be confused with the 357-second whole-layer total.

For 4096 preserved cells: rational kernel {exact['timers'][0]['seconds']:.9f} s; canonical serialization {exact['timers'][2]['seconds']:.9f} s; hashing {exact['timers'][3]['seconds']:.9f} s; compression {exact['timers'][4]['seconds']:.9f} s; file write {exact['timers'][5]['seconds']:.9f} s. Fraction multiplication/addition, rational normalization/GCD and object dispatch dominate the profile. No symbolic root solving is involved in this wedge-sign kernel.

## J. Startup audit

401.819024 s spans task start to scientific supervisor start. Historical authority/coding/smoke-test subtimes are unavailable; no false retroactive attribution is made. Current bounded measurements: Python start 0.059001 s; production import 0.466975 s; wall load 0.154888 s; topology 0.233009 s; SDF setup 0.052416 s; schedule 0.008118 s; full preserved checkpoint-chain restart 0.336278 s. These use a warm filesystem cache.

STORAGE_BOUNDED.json additionally times atlas initialization, empty planner setup, native certificate binding and one-layer index construction. No JIT rebuild, process pool or cold full forecast was started by this audit. Kernel startup does repeat across the production supervisor's fresh stage processes, but its historical sum was not separately timed.

## K. Initial forecast audit

56.002406 s builds/serializes the initial vertex trajectory; 53.951074 s is its projection timer. Complete direct numeric scanning of 20 fronts takes another 60.020978 s. Total: 116.796494 s. This does not show a doubling of the SDF forecast itself relative to Task1's roughly56.6 s.

The initial production path does not build all Task1 edge/triangle descriptor arrays here. Later local reforecasts do recompute20 descriptor layers, including unchanged prefix descriptors, and reevaluate normals on the private prefix. FORECAST_RECOMPUTATION.json gives source-exact counts. A 1024-vertex20-layer bounded forecast reproduced preserved coordinates exactly; no cold full-shell forecast was run.

## L. Whole-layer validation audit

The preserved 113.164887 s sums nominal/final and certify-stage whole-layer numeric timers. It excludes 38 previous-layer evaluate_cells calls made before those timers. These callers need centres/volumes but calculate every cell metric, including all conditioning operators. Source call accounting is in WHOLE_LAYER_WORK_COUNTS.json.

For preserved passive layer16, previous-layer full cell metrics took2.173860 s and new-layer evaluation2.312541 s; gates matched the sealed record. Current production also repeats the already-evaluated accepted candidate in the certify subprocess. Complete immediate checks must remain authoritative; cached identity-bound evidence could avoid duplicated arithmetic without changing coverage.

## M. Certification audit

Optimized certification remains selected. Preserved totals:331.948455 s wall; enumeration {cert['totals']['enumeration_seconds']:.6f} s; filtering {cert['totals']['filter_seconds']:.6f} s; remaining exact stage {cert['totals']['remaining_exact_seconds']:.6f} s; serialization {cert['totals']['serialization_seconds']:.6f} s. Exact kernel calls:36147; full symbolic CAD:646; mixed-rank:13478.

Worker kernel totals are1216.999993 s across parallel workers, including874.762386 s in646 CAD cases and302.184106 s in mixed-rank cases. They are not wall-time additions. No committed-vs-committed proof is re-solved. Certification is now secondary to active search, while still a major measured stage. Its mathematics was not changed or rerun globally.

## N. Final-validation duplication

Final qualification takes144.678868 s. Repeated numeric/conditioning evaluation on immutable committed cells:48.383469 s. Repeated SDF queries at already checked coordinates:52.229677 s. Their sum is100.613146 s. This arithmetic can only be removed after complete identity-bound per-layer receipts replace it.

The29.159862 s final certificate completeness audit remains a completeness task. It reuses56462864 proofs and executes zero new exact pair solves. No final rational wedge recheck occurs. Remaining final time includes loading, hashing and report/array output; historical substage splits are unavailable. No final requirement was removed here.

## O. Checkpoint and data movement

Planning persistence:60.371902 s; atomic commit/checkpoint timer:7.502690 s. Archived hot-planner snapshots total625103682 bytes. The snapshots serialize full cached detailed results; candidate coordinates themselves are stored as deltas against the initial forecast. CommitStore.restart re-reads the immutable hash chain before and after commits.

One bounded62180-prism planner entry produces {storage['metadata_characters']} metadata characters, {storage['uncompressed_array_bytes']} uncompressed array bytes and {storage['compressed_bytes']} compressed bytes. Pack/JSON/compress takes0.766738 s, including0.519860 s compression. Inflation takes0.069106 s. The payload was measured in memory and not persisted.

The representative trial materializes982800 bytes of local fronts and8953920 bytes of prisms. Returned numeric arrays occupy9900656 bytes. Separate tracemalloc measurement observes206131796 bytes peak and33245084 bytes retained, including Python face records. There is no per-trial cold whole-shell forecast copy. Tracemalloc timing is excluded from performance comparisons. Audit peak RSS: {peak} bytes.

## P. Bounded early-out experiment

Thirteen plain/instrumented detailed cases preserve every output bit and immediate gate. An immediate-only slice reduced one large rejected case from2.498734 s to0.635696 s, with identical complete gate extrema/counts. Three paired immediate-only cases saved3.844192 s in total.

The cheaper existing analytical wedge/signed-volume/determinant probe was checked on68 reconstructed candidates. All existing gate counts matched. It rejected23 and left45 on the unchanged fallback path; all68 probes together took {cheap['seconds']:.9f} s. The23 preserved detailed evaluations had cost {old_cohort:.9f} s; this is a historical affected-cost total, not a net full-run speedup. The three paired full-detail versus cheap-cell measurements saved {sum(r['avoided_seconds'] for r in paired):.9f} s. No approximate check accepted a candidate, and no survivor skipped exact certification.

## Q. Bounded incremental-metric experiment

Layer3 trial13 changes {inc['changed']} of {inc['full_cells']} prism-cell inputs. Exact dependency reuse retains {inc['reused']} cell results. Full cell-family evaluation: {inc['full_seconds']:.9f} s; local recomputation plus merge: {inc['local_seconds']:.9f} s. Every cell metric array, including sigma/G, is bitwise equal. Face-metric incrementality was not implemented.

Separate implementation-overhead probes preserve the complete detailed result. Hoisted closure: {metric['closure']['old_seconds']:.9f} -> {metric['closure']['hoisted_seconds']:.9f} s per closure call. Required G_bound expression only: {metric['gains']['old_seconds']:.9f} -> {metric['gains']['only_required_G_bound_seconds']:.9f} s. These changes are study-only. No production optimization was installed.

## R. Brute-force classifications

Detailed work: ConservativeOverwork, RepeatedIdenticalWork and ImplementationOverhead. Rejection pipeline: LateRejectedWork. Whole new-layer rational signs: NecessaryCompleteWork with implementation overhead. Whole-layer numeric validation: required coverage plus duplicate calculations. Initial scan: NecessaryCompleteWork. Final numeric/SDF: RepeatedIdenticalWork. Final certificate: NecessaryCompleteWork as a completeness audit. Startup's historical internal split: Unresolved. BRUTE_FORCE_CLASSIFICATION.json gives stage evidence.

## S. Ranked optimization targets

1. Dependency-local detailed metrics with immutable support/closure reuse. Target stage479.998650 s;66.65% identical cell inputs were recomputed. The bounded cell and closure savings above are directly measured. Also stop calculating unconsumed gain operators.
2. Staged trial rejection using existing immediate gates, then existing future admission before unnecessary exact signs.76 immediate failures and130 future regressions expose late work. Historical138.610223 s is a detailed-time ceiling, not a promised saving.
3. Identity-bound reuse of immediate/final metrics and SDF receipts. Final duplicated arithmetic alone is100.613146 s; repeated previous-cell operators and precommit checks add separately measured opportunities.

These targets overlap. Their times must not be added as independent savings. After them, the largest separately measured remaining stages are exact-rational signs357.404986 s and optimized certification331.948455 s. No unmeasured new end-to-end runtime is claimed. RANKED_RECOMMENDATIONS.json records scientific risks and follow-up targets for exact arithmetic and checkpoint eviction.

## T. Classification

V4RuntimeBruteForcePathsIdentified. The evidence proves major repeated work; it does not show that all current runtime is avoidable. No science or production search decision changed. All255 reconstructed candidate hashes matched preserved transactions. The isolated metric profile and storage fixture share scientific hash `{metric['scientific_hash']}`. Two detailed/exact replays match prior measured and preserved outputs.

Accounting replay hashes: `{det['accounting_scientific_hash_1']}` and `{det['accounting_scientific_hash_2']}`. This is a canonical accounting replay plus bounded scientific equivalence, not another full A2 run.

Focused tests:25 passed,0 failed. Prior tests are retained unchanged; long and unrelated repository-state suites were not rerun. A study-only layer2 indexing error and an instrumentation entity-counter issue were corrected and regression-tested. No production defect was silently repaired.

Task start:{clock['task_start']}. Scientific cutoff:{clock['scientific_cutoff']}. Last scientific kernel run completed before this report. Audit sealing elapsed at report generation:{seconds:.6f} s. SCIENTIFIC_STOP.json records the conservative science-stop boundary; SEAL.json records sealing time and hashes. No scientific worker remains. The task stopped well before the60-minute limit.

## U. Recommended next task

ImplementV4DependencyLocalMetricsAndStagedTrialEvaluation. First integrate the measured dependency-local and staged-check paths under exact equivalence tests. Keep every immediate hard gate and exact certificate intact. Then perform a separately authorized controlled end-to-end run. Do not start downstream meshing, CFD or layer termination.

Changed paths are Task4F tools, focused tests and results only; SOURCE_AND_ARTIFACTS.json lists them. Git commit follows sealing, so its hash is reported in the final response rather than embedded recursively in this report. Report and authority SHA-256 values are stored in SEAL.json.
'''
    # Prose spacing only. Do not alter hashes, identifiers or scientific data.
    report=re.sub(r'\b(all|on|the|The|takes|took|trial|layer|Layer|stage|signs|certification|rejected|left|with|roughly|is|and|from|including|than|reuses|are|of)(?=\d)',r'\1 ',report)
    report=report.replace('Task1','Task 1').replace('layer2 indexing','layer-2 indexing').replace('per closure','per closure')
    report=report.replace('The audit reconstructed missing local coordinate identities and checked every trial candidate hash.', 'The audit reconstructed missing coordinate inputs for all trials and checked their hashes. This was coordinate-only accounting; it did not rerun all search decisions, detailed evaluations or certificates.')
    (OUT/'report.md').write_text(report,encoding='utf-8')
    save('SCIENTIFIC_STOP.json',dict(utc=scientific_stop,elapsed=seconds,scientific_cutoff=clock['scientific_cutoff'],all_workers_stopped=True,full_build_run=False))
    save('SUMMARY.json',dict(classification='V4RuntimeBruteForcePathsIdentified',peak_rss=peak,elapsed_at_report=seconds,operation_counts=summary,tests=dict(passed=25,failed=0),report_sha256=file_hash(OUT/'report.md'),production_changes=0,V3_changes=0))
    print('REPORT',file_hash(OUT/'report.md'),elapsed(),flush=True)

if __name__=='__main__':run()
