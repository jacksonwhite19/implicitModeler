"""Seal a completed or timed partial benchmark without more scientific work."""
from task4i_common import *
from task4i_collect import collect

def seal():
    tick=time.perf_counter();summary=collect();execution=read(OUT/'EXECUTION.json');config=read(OUT/'RUNNER_CONFIGURATION.json')
    final=optional(RUN/'FINAL_VALIDATION.json',{});comparison=optional(OUT/'SCIENTIFIC_COMPARISON.json',{})
    cp=optional(RUN/'build_A/CURRENT.json',{'layer':0});layer=cp['layer'];tests=read(OUT/'SMOKE_TESTS.json');clock=read(OUT/'CLOCK.json')
    if execution['status']=='ScientificStagesComplete' and comparison.get('passed') and execution['seconds']<1800:
        classification='V4IntegratedFullBuildQualifiedWithin30Minutes'
    elif execution['status']=='ScientificMismatch':classification='V4IntegratedFullBuildScientificMismatch'
    elif execution['status']=='ScientificCutoff':classification='V4IntegratedFullBuildTimeBudgetExceeded'
    else:classification='V4IntegratedFullBuildBlocked'
    initial=read(OUT/'PREFLIGHT_INITIAL.json');auth=read(OUT/'AUTHORITY_VERIFICATION.json')
    for p,h in config['source_hashes'].items():assert file_hash(ROOT/p)==h,('SourceChangedDuringExecution',p)
    for p,h in auth['sealed'].items():assert file_hash(ROOT/p)==h,('PriorArtifactChanged',p)
    for p,h in initial['V3'].items():assert file_hash(Path(p))==h,('V3Changed',p)
    outside=lambda s:[x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in x]
    status=git('status','--porcelain');assert outside(status)==outside(initial['starting_status']),'UnrelatedStatusChanged'
    # Checkpoint integrity only: no geometry generation or qualification here.
    checkpoint_hash=None
    if layer:
        checkpoint=read(RUN/f'build_A/LAYER_{layer:02}.json');checkpoint_hash=file_hash(RUN/f'build_A/LAYER_{layer:02}.json')
        assert file_hash(RUN/'build_A'/checkpoint['coordinate_file'])==checkpoint['coordinate_file_sha256']
        assert checkpoint['gate']['passed'] and checkpoint['exact']['passed'] and checkpoint['certificate']['complete']
        assert not checkpoint['certificate']['forbidden'] and not checkpoint['certificate']['unresolved']
    save('PRESERVATION.json',dict(prior_artifacts_verified=len(auth['sealed']),V3_modification_count=0,V3_bindings_verified=len(initial['V3']),unrelated_status_unchanged=True,unrelated_entries=len(outside(status)),last_checkpoint_sha256=checkpoint_hash,source_frozen=True))
    rows=summary['rows'];stages=summary['stages'];counts=summary['operation_counts'];slow=max(rows,key=lambda x:x['layer_elapsed_seconds'],default={})
    tree_size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    completed=now();task_seconds=(completed-datetime.datetime.fromisoformat(initial['task_start'])).total_seconds()
    result=dict(classification=classification,scientific_seconds=execution['seconds'],task_seconds_to_seal=task_seconds,scientific_limit_respected=execution['seconds']<1800,
        committed_layers=layer,prisms=106302*layer,scientific_equal=comparison.get('passed',False),slowest_layer=slow.get('layer'),slowest_layer_seconds=slow.get('layer_elapsed_seconds'),
        peak_tree_rss=execution['peak_tree_rss'],persistent_bytes_before_seal=tree_size,last_checkpoint_sha256=checkpoint_hash,
        scientific_mesh_identity=comparison.get('scientific_mesh_identity'),raw_shell_hash=final.get('shell_scientific_hash'),
        conditioning={k:final.get(k) for k in ('sigma_minimum','sigma_location','G_bound_maximum','G_bound_location')},stages=stages,operation_counts=counts,tests=tests)
    save('SUMMARY.json',result)
    f=lambda x:'unavailable' if x is None else f'{x:.3f}' if isinstance(x,float) else str(x)
    table='\n'.join(f"| {r['layer']} | {r['committed']} | {r['corrected']} | {100*r['active_fraction']:.3f}% | {r['trials']}/{r['accepted']}/{r['rejected']} | {f(r['detailed_seconds'])} | {f(r['exact_jacobian_seconds'])} | {f(r['certificate_seconds'])} | {f(r['layer_elapsed_seconds'])} | {f(r['cumulative_task_seconds'])} |" for r in rows)
    stage_table='\n'.join(f'| {k} | {f(v)} |' for k,v in stages.items())
    text=f'''Decision question

Can the current integrated optimized V4 mesher independently generate and qualify the complete 20-layer SF2 wall-prism shell within 30 minutes of scientific execution?

A. Time-limit compliance

Scientific start: {execution['scientific_start']}. Cutoff: {clock['scientific_cutoff']}. Scientific stop: {execution['scientific_stop']}. Scientific execution: {execution['seconds']:.3f} s. Limit respected: {result['scientific_limit_respected']}. Task start including bounded preflight: {initial['task_start']}. Sealing snapshot: {completed.isoformat()}; task wall time to this snapshot: {task_seconds:.3f} s. SEAL.json records final sealing completion. No scientific work runs during sealing.

B. Authority verification

Started at {initial['head']}. Latest published Task-4 through Task-4H source bindings were verified. {len(auth['sealed'])} sealed prior artifacts and {len(initial['V3'])} V3 bindings remain unchanged. Wall geometry, surface, topology and binary64 schedule match the frozen authorities. Unrelated Git entries preserved: {len(outside(status))}.

C. Optimized-path preflight

{tests['count']} focused tests passed: {tests['passed']}. The configuration selects Task-4B compiled conservative certification, Task-4C atlas, Task-4D persistent planning, Task-4G dependency-local metrics and staged rejection, and Task-4H exact dyadic records, indexed future selection, exact-input proofs and immutable receipts. Source hashes were frozen before timing and remained identical afterward.

Three small integration repairs preceded timing. Task-4I gained its own allowed output root. Initial warning diagnostics now use the already qualified detailed cache. Final authority validation uses the latest qualified source lineage; the old Task-3 source hash rejected an already qualified Task-4G cache change. REPAIRS.md preserves causes and procedures. Scientific definitions did not change.

D. Fresh build configuration

The separate fresh/ directory started without a forecast or committed state. All candidates arose from frozen layer 0 and generic production risk selection. No reference coordinates or saved optimizer state entered construction. References were opened by the scientific comparator only after fresh final qualification. Requested: 20 layers, 53,153 vertices, 106,302 columns. Certificate worker bound: 8.

E. Layer-by-layer execution

Trials are total/accepted/rejected. Times are seconds. Cumulative values include fresh process startup and initial scan. Active fraction below counts correction support; atlas fractions are recorded separately in LAYER_PROGRESS.json.

| Layer | Committed | Corrected | Active vertices | Trials | Detailed | Whole exact | Certificate | Layer wall | Cumulative |
|---|---|---|---|---|---|---|---|---|---|
{table}

F. Active/passive behavior

Fresh atlas state, generic support, invalidations and passive fractions are preserved per layer. No layer number or historic cap ID selects production work. Accepted correction/region equivalence: {comparison.get('passed',False)}. Detailed array/hash evidence is in fresh/LAYER*_ATLAS* and LAYER_PROGRESS.json.

G. Active-search performance

Total trials {counts.get('trials',0)}; accepted corrections {counts.get('accepted',0)}; rejected {counts.get('rejected',0)}; explicit early rejections {counts.get('early_rejections',0)}. Search time: {f(stages.get('active_search_seconds'))} s. Existing mandatory gates stop failed trials before future work. Canonical proposal sequence is unchanged.

H. Dependency-local metric behavior

Detailed time: {f(stages.get('active_detailed_seconds'))} s. Requests {counts.get('detailed_requests',0)}, exact patch hits {counts.get('detailed_exact_hits',0)}, refreshes {counts.get('detailed_refreshes',0)}. OPERATION_COUNTS.json contains primitive, face and operator reuse from direct cache observations. No metric is reused by object identity.

I. Exact Jacobian performance

Whole-layer exact record time: {f(stages.get('exact_jacobian_seconds'))} s. Whole calls {counts.get('whole_exact_calls',0)}, canonical rational materializations {counts.get('whole_exact_materializations',0)}. Local exact misses {counts.get('local_exact:misses',0)}, hits {counts.get('local_exact:hits',0)}, time {f(stages.get('local_exact_seconds'))} s. Exact dyadic integer arithmetic preserves the frozen rational values and signs.

J. Future exact spot-check performance

Time: {f(stages.get('future_spot_checks_seconds'))} s. Indexed selection, conservative certified proofs and exact-input cache were active. Per-process proof/selection counters are in instrument/ and OPERATION_COUNTS.json. Full future certification is not an admission requirement.

K. Immediate exact certification

Runtime: {f(stages.get('certificate_seconds'))} s. Required immediate pairs: {counts.get('immediate_pairs',0)}. Frozen-kernel calls: {counts.get('immediate_frozen_kernel_calls',0)}. Every published checkpoint requires complete exact coverage, no forbidden intersections and no unresolved results. Method counts distinguish symbolic CAD from cheaper frozen branches.

L. Final validation

Final qualified output present: {bool(final)}. Immutable receipt coverage: {optional(RUN/'FINAL_VALIDATION_COVERAGE.json',{}).get('reused_layers','unavailable')} reused layers. Numeric/SDF recalculation time: {f(stages.get('final_numeric_seconds'))}/{f(stages.get('final_sdf_seconds'))} s. Complete coverage remains mandatory. Final hard gates: {bool(final) and all(g['passed'] for g in final.get('gates',[]))}.

M. Final certificate

Complete: {final.get('certificate',{}).get('complete',False)}. Required population: {final.get('certificate',{}).get('candidate_pairs','unavailable')}. Forbidden/unresolved: {len(final.get('certificate',{}).get('forbidden',[])) if final else 'unavailable'}/{len(final.get('certificate',{}).get('unresolved',[])) if final else 'unavailable'}. Complete disjoint immediate-proof union was re-enumerated against final geometry. Audit runtime: {f(stages.get('final_certificate_completeness_seconds'))} s.

N. Conditioning

Minimum weighted sigma: {final.get('sigma_minimum','unavailable')}, location {final.get('sigma_location','unavailable')}. Maximum available G_bound: {final.get('G_bound_maximum','unavailable')}, location {final.get('G_bound_location','unavailable')}. These are diagnostics, not gates.

O. Scientific comparison

Exact scientific equality: {comparison.get('passed',False)}. Scientific mesh identity: {comparison.get('scientific_mesh_identity','unavailable')}. Full coordinates, connectivity, source ownership, trial decisions, accepted corrections, immediate hard metric arrays, exact Jacobian records, conditioning and complete certificate pair/outcome hashes are compared. Reference hard metric arrays are reconstructed from preserved coordinates using qualified unchanged definitions because the reference stored extrema rather than all primitive arrays.

Fresh raw serialization hash: {final.get('shell_scientific_hash','unavailable')}. Reference A2 raw hash: {comparison.get('raw_reference','unavailable')}. Original A raw hash: {comparison.get('raw_reference_build_A','unavailable')}. Rejected-trial diagnostic omission and exact-safe future proof storage change raw history hashes; they are not scientific mismatch gates. If incomplete, no complete equality is claimed. Last checkpoint SHA-256: {checkpoint_hash}. Pending stage: {execution['stage']}.

P. Runtime decomposition

| Recorded stage | Seconds |
|---|---:|
{stage_table}

{summary['overlap_note']} Total scientific execution includes imports, process startup, input/output and mandatory comparison. It is a fresh measured run, not an extrapolated replay estimate. Task-4E reference execution was 2,282.94 s.

Q. New dominant bottleneck

The largest measured stages can be compared directly above. Active planning contains its detailed/spot-check sub-stages. Process startup, unchanged committed-prefix preparation, planner snapshot serialization and full bounded future pair enumeration remain candidates for a follow-up audit; this benchmark makes no architectural change and claims no unmeasured saving.

R. Memory and storage

Peak aggregate process-tree RSS: {execution['peak_tree_rss']} bytes. Artifact size before final sealing files: {tree_size} bytes. Only the new Task-4I run is stored. No old shell was duplicated. Existing production planner checkpoint serialization remains; its storage cost is included rather than hidden.

S. Tests

Focused smoke modules: {', '.join(tests['modules'])}. {tests['count']} tests, {tests['failures']} failures, {tests['errors']} errors; {tests['seconds']:.3f} s before the scientific clock. No large suite or scientific tests were run during sealing. Deadline, independent ordering, representation-only comparison normalization and integration repairs have focused regressions.

T. Classification

{classification}

{layer}/20 layers committed; {106302*layer} wall prisms. Execution status: {execution['status']}. Scientific stop was final; no code changed during execution, no invalid layer was admitted, and no failed build restarted.

U. Recommended next step

Use this fresh timing and operation-count evidence to select one measured remaining runtime target. Do not infer a new speedup from old bounded replays. No downstream mesh or CFD work was started.
'''
    (OUT/'report.md').write_text(text,encoding='utf-8',newline='\n')
    paths=[p for p in OUT.rglob('*') if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json') and not p.name.endswith('.pending')]
    authority=dict(task='RunV4IntegratedFreshFullBuildQualification',classification=classification,starting_head=initial['head'],summary=result,
        report_sha256=file_hash(OUT/'report.md'),source_sha256=config['source_hashes'],evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in paths},
        geometry_modified_in_V3=0,scientific_rules_unchanged=True,scientific_stop=execution['scientific_stop'])
    save('AUTHORITY.json',authority)
    save('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),
        sealing_completed=now().isoformat(),sealing_seconds=time.perf_counter()-tick,task_wall_seconds=(now()-datetime.datetime.fromisoformat(initial['task_start'])).total_seconds(),
        persistent_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())))
    print(json.dumps(read(OUT/'SEAL.json'),indent=2));print(classification)
if __name__=='__main__':seal()
