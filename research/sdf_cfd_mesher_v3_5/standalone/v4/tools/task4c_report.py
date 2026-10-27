"""Seal the measured Task-4C study. Never run a mesh builder."""
from task4c_common import *
import shutil

def run():
    rows=read(OUT/'atlas_2_ATLAS.json')['rows'];times=read(OUT/'atlas_2_TIMINGS.json');cert=read(OUT/'CERTIFICATE_TIMINGS.json')
    tests=read(OUT/'TEST_SUMMARY.json');preserve=read(OUT/'PRESERVATION.json');det=read(OUT/'DETERMINISM.json')
    assert tests['all_scientific_tests_passed'] and det['passed'] and preserve['V3_modified_files']==0
    stage_names=sorted({k for r in times['layers'] for k in r['stages']})
    stages={k:sum(r['stages'].get(k,0.) for r in times['layers']) for k in stage_names}
    benchmarks={str(k):read(OUT/f'RUNNER_{k:02}_TIMINGS.json') for k in (2,8,12,20)}
    layers=[]
    for r in rows:
        p=r['partition'];inv=r['invalidation'];n=p['regions'];nc=106302
        layers.append(dict(layer=r['layer'],active_vertices=p['active_regions'],active_vertex_percent=100*p['active_regions']/n,
            passive_vertex_percent=100*p['passive_regions']/n,active_columns=len(p['active_columns']),active_column_percent=100*len(p['active_columns'])/nc,
            passive_column_percent=100*(nc-len(p['active_columns']))/nc,dependency_vertices=len(p['dependency_vertices']),
            selected_work_regions=p['work_regions'],corrected_work_regions=len(r['required_corrections']),
            active_work_without_correction=p['work_regions']-len(r['required_corrections']),
            changed_vertices=len(r['changed_vertices']),invalidated_regions=len(inv['regions']),invalidated_edges=len(inv['edges']),
            invalidated_columns=len(inv['columns']),exact_region_hits=inv['retained_regions'],detailed_hits=r['detailed_hits'],detailed_misses=r['detailed_misses'],
            future_entries_reused=r['forecast_entries_reused'],future_entries_remaining=r['forecast_entries_remaining'],
            detailed_prisms=sum(x['counts']['prisms'] for x in r['detailed']),
            immediate_gate_passed=r['immediate_gate']['passed']))
    reused=sum(r['future_entries_reused'] for r in layers);available=sum(r['future_entries_remaining'] for r in layers)
    cert_seconds=sum(r['certificate'][0]['seconds'] for r in cert['layers']);audit_seconds=sum(r['audit'][0]['seconds'] for r in cert['layers'])
    initial=read(OUT/'INITIAL_SCAN_TIMINGS.json');memory=read(OUT/'CERTIFICATE_MEMORY.json');metric=read(OUT/'ACTIVE_METRIC_PROFILE.json')
    preflight=read(OUT/'PREFLIGHT.json');elapsed=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(preflight['started_utc'])).total_seconds()
    files=[p for p in OUT.rglob('*') if p.is_file()];largest=max(files,key=lambda p:p.stat().st_size)
    classification='V4DynamicRiskAtlasPromising'
    old_prepare=sum(json.loads((BASE/f'LAYER{k}_PREPARE_TIMINGS.json').read_text())['total_seconds'] for k in range(2,21))
    summary=dict(classification=classification,base_head=preflight['head'],branch=preflight['branch'],layers=layers,
        all_32_correction_regions_activated_generically=True,passive_and_reforecast_coordinates_exact=True,
        future_entries_reused=reused,future_entries_available=available,forecast_reuse_percent=100*reused/available,
        selected_work_regions=sum(r['selected_work_regions'] for r in layers),detailed_cross_layer_hits=sum(r['detailed_hits'] for r in layers),
        detailed_refreshes=sum(r['detailed_misses'] for r in layers),planning_stage_seconds=stages,
        frozen_full_prepare_layers_2_20_seconds=old_prepare,atlas_replay_seconds=times['total_seconds'],atlas_replay_excludes_optimizer_execution=True,
        initial_direct_scan_seconds=initial['total_seconds'],certificate_seconds=cert_seconds,complete_certificate_audit_seconds=audit_seconds,
        required_pairs=56462864,exact_fallback_calls=sum(r['certificate'][0]['exact_calls'] for r in cert['layers']),forbidden=0,unresolved=0,
        runner_benchmarks={k:dict(old_seconds=v['frozen']['total_seconds'],new_seconds=v['current']['total_seconds'],
            old_search_seconds=v['frozen'].get('local_search_seconds'),new_search_seconds=v['current']['local_search_seconds']) for k,v in benchmarks.items()},
        scientific_replay_hash=det['atlas_run_1'],tests=tests,observed_tree_peak_rss=memory['peak_tree_rss'],
        study_elapsed_seconds=elapsed,V3_modified_files=0,Build_A_rebuilt=False,A2_resumed=False,
        A2_readiness='Technically ready for a separate controlled qualification with both integrated paths. A full production runtime improvement is not yet demonstrated.',
        study_bytes_before_seal=sum(p.stat().st_size for p in files),largest_new_file=largest.relative_to(ROOT).as_posix(),
        largest_new_file_bytes=largest.stat().st_size,free_disk_bytes=shutil.disk_usage(ROOT).free)
    save('SUMMARY.json',summary)
    table='\n'.join(f"| {r['layer']} | {r['active_vertices']} | {r['active_vertex_percent']:.6f}% | {r['active_columns']} | {r['active_column_percent']:.6f}% | {r['selected_work_regions']} | {r['changed_vertices']} | {r['invalidated_regions']} |" for r in layers)
    bt='\n'.join(f"| {k} | {v['frozen']['total_seconds']:.6f} | {v['current']['total_seconds']:.6f} | {v['current']['local_search_seconds']:.6f} | Exact |" for k,v in benchmarks.items())
    stage_table='\n'.join(f'| {k} | {v:.6f} |' for k,v in stages.items())
    report=f'''Decision question

Can V4 separate the shell into persistent passive nominal regions and bounded active risk regions so expensive future planning scales with the difficult fraction of the surface rather than the complete shell?

Classification: **{classification}**

The active/passive architecture reproduces the preserved science. It concentrates detailed work in small supports. It does **not** demonstrate a large new planning speedup: the old Task-4 runner already used local detailed searches and sparse future updates. Active search remains a meaningful bottleneck. Nominal layers avoid redundant projection and a duplicate whole-layer metric evaluation.

## A. Authority verification

- Branch: `{preflight['branch']}`. Starting HEAD: `{preflight['head']}`.
- Task-4 report: `28fcb9be7fac30eec2dd84a3d292fbfbdead3daef3a4ed851a4df7744a213190`.
- Task-4 authority: `be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9`.
- Task-4B report: `ec5686586f9eb5e2e65f55e72338dae3276b82e8f9d70a4e70efc5f610c7bba2`.
- Task-4B authority: `8fe4af86b569705ac41399dcdbd0a02ddb7ff41b62a75fc254cd7998bd754aac`.
- Frozen Build-A shell scientific hash: `54a69dae10d792aeee02d945fcd095f053990a10db695e97626ab540ae24e51c`. The prompt has an extra trailing zero; the frozen 64-character authority matches.
- {preserve['sealed_bindings_rechecked']:,} Task-4/4B bindings rechecked. Five Task-4 tool integrations are intentional source changes. Zero sealed result files changed. All Task-4B mathematical source remains unchanged.
- {preserve['V3_bounded_files']} bounded V3 source files rechecked; Task-4C V3 modifications: **0**. Build-A and A2 CURRENT identities are unchanged.
- The starting checkout was dirty. Unrelated changes were not staged. During this task {len(preserve['unrelated_added'])} additional unrelated untracked paths appeared; they were left untouched. See `PRESERVATION.json`.

## B. Task-4B certificate integration

`task4_certify_candidate.py` now imports `certificate.production.certify_immediate`. The final validator selects its matching verifier. This is the qualified native conservative filter plus the unchanged exact fallback kernel. It never accepts uncertain bounds. Old-format checkpoints retain their original verifier.

The actual production-selected function was called on all twenty preserved Build-A prefixes. Required pair IDs, proof codes, witnesses, outcomes, exact fallback records and validity hashes match the sealed Task-4B run for every layer. Required population: **56,462,864**. Frozen-kernel calls: **{summary['exact_fallback_calls']:,}**. Forbidden: **0**. Unresolved: **0**. All prior committed pair proofs remain unchanged. The complete coverage audit reused all 56,462,864 proofs, with no missing population.

The new adapter changes raw proof-storage and transaction-chain hashes. `replay_equivalence` compares all geometry, connectivity, SDF, gate, conditioning, decision and warning fields exactly, plus complete pair/outcome hashes through the sealed Task-4B one-for-one bridge. It reports unequal raw storage hashes explicitly. Tests reject coordinate, warning, pair-population, pair-outcome and conditioning changes. A2 was not executed.

## C. Existing Task-4 planning baseline

The old runner already had one initial global scan, local warning updates, bounded search, sparse future-coordinate deltas and no optimization in nominal-only layers. It still reprojected every vertex to assert nominal cache equality. It also repeated the same complete metric calculation when no correction changed the candidate.

The recorded preparation total for layers 2–20 is **{old_prepare:.6f} s**. These facts limit the speedup attributable to this task. Replacing the old local planner with a new global-versus-local narrative would misstate the baseline. No historical IDs, scientific gates, objective, trial modes, move factors or search ordering changed.

## D. Risk-atlas architecture

`RiskAtlas` stores `PassiveNominal`, `ActiveRisk`, `DependencyInvalidated` and `NeedsDetailedRefresh`. It binds frozen topology, geometry, surface, SDF source, binary64 schedule, committed prefix, per-vertex trajectory identities, current risk evidence and runtime-retained support. The common `production_risk.select_supports` is the factored existing Task-4 selector.

The cached nominal path is selected only after exact committed-coordinate and forecast-authority checks. A changed region cannot silently reuse unrelated backing. Atlas metadata cannot publish a prism layer. The normal complete immediate gates and atomic CommitStore remain authoritative.

## E. Region and dependency model

There are 53,153 stable vertex-owned regions because nominal SDF trajectories depend on one source vertex. Connected expensive work uses the existing dependency-complete patch machinery. This is not a spatial grid. Actual corner control, incident edges, incident triangles, face neighbors, reconstruction closure and verification rings determine support.

`REGION_DEPENDENCY_SPEC.md` documents ownership, state transitions, exact invalidation and rollback. Shared vertices have one authoritative coordinate row. No topology changes, arbitrary margin, extra risk threshold or global sensitivity matrix was introduced.

## F. Initial shell partition

Initial active vertices: **2,319 / 53,153 = {layers[0]['active_vertex_percent']:.9f}%**. Passive: **{layers[0]['passive_vertex_percent']:.9f}%**.

Initial active columns: **6,316 / 106,302 = {layers[0]['active_column_percent']:.9f}%**. Passive: **{layers[0]['passive_column_percent']:.9f}%**.

The initial immediate search selects two connected supports. Direct later warnings also remain active atlas evidence even when the current scheduling policy does not launch their optional search yet. Atomic active regions, selected search supports and dependency halos are therefore reported separately. Their counts are not interchangeable.

## G. Layer-by-layer atlas evolution

| Layer | Active vertices | Active vertex fraction | Active columns | Active column fraction | Search regions | Changed vertices | Invalidated regions |
|---:|---:|---:|---:|---:|---:|---:|---:|
{table}

The active set becomes empty at layer 12 through current evidence alone. No layer-12 switch exists. Layers 12â€“20 use the passive nominal path. Detailed work is zero there.

## H. Regression against actual Task-4 correction regions

All **32** preserved corrected supports are selected exactly by generic current evidence **before** the preserved optimizer output is read. No required correction remains passive. All 32 selected search regions had a preserved accepted correction; selected-search regions with no correction: **0**. This does not prove a population false-positive rate. Many active support vertices remain fixed or receive zero effective movement.

The replay substitutes preserved accepted coordinates for optimizer output. It then recomputes only changed future trajectories and verifies the entire forecast coordinate and normal arrays exactly. All initial direct events were independently recomputed once from the restored nominal forecast. Selective future exact warnings are reused only after complete forecast input equality is established.

## I. Passive nominal fast path

Passive coordinate retrieval is exact. The actual runner no longer invokes whole-front SDF projection merely to confirm unchanged cached nominal coordinates. If no correction changes the candidate bits, the complete nominal metric result also serves as the final preparation metric result. Commit-time full validation remains unchanged.

## J. Active detailed path

The qualified local search, SDF projection, future reforecast, direct hard metrics, sigma/G diagnostics, selective exact collision checks and verification-ring policy remain intact. The study refreshed **32** bounded detailed supports. Largest individual scope remains **3,138 columns / 62,760 provisional prisms**; no dense sensitivities are constructed.

The representative active metric profile measured {metric['counts']['prisms']:,} provisional prisms, {metric['counts']['measured_prisms']:,} core cells and {metric['counts']['faces']:,} faces in {metric['seconds']:.6f} s. See `ACTIVE_METRIC_PROFILE.json`. This is a twenty-layer local scope, not directly the same workload as one complete immediate layer.

## K. Invalidation and cache reuse

Across remaining-stack opportunities, **{reused:,} / {available:,} = {summary['forecast_reuse_percent']:.9f}%** vertex-layer coordinates were reused. Maximum changed vertices in one transaction: **779**. Maximum dependent atlas regions invalidated: **3,628 / 53,153**. Later cap corrections changed only 4, 3, 3 and 3 vertices at layers 8â€“11, with 21, 18, 18 and 18 dependent regions invalidated. All unrelated trajectories remained exact hits.

Cross-transaction detailed-result hits: **0**; selected detailed refreshes: **32**. Every selected detailed input changed. This is an important limit: the gain is passive forecast/diagnostic persistence, not reuse of unchanged active metric arrays. The old bounded trial cache still operates during search.

Two local implementation issues were fixed during development. The first replay driver retained stale initial warnings; the stored support-mismatch reproducer and replacement test cover it. The SF2 disk restart check then found Windows int32 support IDs hashed differently after int64 reload. Canonical little-endian int64 support hashing fixes this representation defect. Four original failing checkpoint hashes reproduce exactly; all four repaired SF2 restarts and rollbacks pass. Coordinates, source ID values, search decisions and metric results are unchanged. See `METADATA_INTEGER_REPAIR.json`.

## L. Immediate global validation

Every complete immediate layer in the twenty-state replay passed the frozen numeric gates. The four actual preparation calls reproduce all frozen gate extrema and violation counts exactly. No hard requirement was removed or localized. Exact Jacobian/volume sign generation, topology/ownership checks and SDF-distance checks remain selected by the commit runner; their preserved results bind to identical candidate geometry in this study.

Complete conditioning calculations remain in `evaluate_layer` and the commit path. Future conditioning runs only in active detailed work. `IMMEDIATE_METRIC_PROFILE.json` records the whole-layer cost and nested SVD/operator routines. Their cumulative timings overlap and must not be summed. No sigma or G_bound acceptance gate exists.

## M. Representative runner equivalence

The actual production preparation function ran independently from four read-only frozen checkpoint facades. It cannot call commit. Current and frozen complete candidate coordinates, all future coordinates, normals, complete accepted/rejected decision histories, warnings and gate values match exactly.

| Layer | Frozen preparation s | Current preparation s | Current local search s | Scientific comparison |
|---:|---:|---:|---:|---|
{bt}

These are direct observed timings, not a controlled same-time hardware experiment. Active cases did not demonstrate a speedup. Nominal cases avoided projection and duplicate validation. The initial run-12 and certificate benchmark overlapped part of the diagnostic replay; their wall times include that contention.

## N. Runtime decomposition

Restored initial forecast; no cold whole-shell SDF regeneration. One complete initial direct risk scan: **{initial['total_seconds']:.6f} s**, including one profiled layer. Independent atlas/control replay: **{times['total_seconds']:.6f} s**, with accepted geometry supplied from the frozen transactions.

| Measured replay stage | Seconds |
|---|---:|
{stage_table}

The replay also includes audit hashing, compact evidence I/O and checkpoint identity checks. Atomic mesh publication and optimization for the other sixteen layers were not rerun. Therefore **{times['total_seconds']:.6f} s is not a full build runtime**. No unmeasured solver work is extrapolated.

Production-selected optimized certification, all twenty preserved prefixes: **{cert_seconds:.6f} s**. Its final complete re-enumeration and bound-proof audit: **{audit_seconds:.6f} s**. Frozen Task-4B reference: 304.34 s for layers 2â€“20, 17.21 s final audit. This adapter benchmark includes process startup per layer and some concurrent diagnostic work. The old Task-4 certificate cost was 11,986.97 s for layers 2â€“20; no old slow certificate was rerun.

Observed peak process-tree RSS: **{memory['peak_tree_rss']:,} bytes**. Study elapsed time at sealing: **{elapsed:.6f} s**. This includes implementation, tests and verification, not only production work. Current artifact bytes before sealing: **{summary['study_bytes_before_seal']:,}**. Largest new file: `{summary['largest_new_file']}` ({summary['largest_new_file_bytes']:,} bytes). Free disk: **{summary['free_disk_bytes']:,} bytes**.

Fresh proof arrays were compared exactly, then replaced by explicit references to identical sealed Task-4B payloads. Manifests, native validity data, timings and binding hashes remain in Task-4C. Build-A geometry was not duplicated.

## O. New dominant bottleneck

Active local search dominates planning in the corrected representative cases. Complete conditioning/SVD work and selective future exact decisions remain in those searches. The whole-shell immediate checks and exact certification also remain measurable mandatory costs. The atlas does not remove these requirements. It does show that detailed future work is already concentrated in a small part of the surface.

The evidence supports explicit persistent scheduling and cheaper nominal advancement. It does not support claiming a large total production-path speedup from the atlas alone. No arbitrary runtime target was used.

## P. Determinism

Complete atlas run 1: `{det['atlas_run_1']}`.

Complete atlas run 2: `{det['atlas_run_2']}`.

Exact equality includes region identities, transitions, invalidations, reused forecasts, local detailed arrays and checkpoint identities. Both replay paths used canonical int64 active supports; the legacy JSON-width repair affects only the separate production metadata migration path. Four production runner decision/coordinate histories match frozen A. Every fresh optimized proof array and exact fallback hash matches the sealed independent Task-4B output. Timing/RSS are excluded; scientific fields are not.

## Q. Tests

`python -B research/sdf_cfd_mesher_v4/tools/task4c_tests.py`: **{tests['test_count']} tests; {tests['passed']} passed; {tests['failures']} inherited repository-state assertion failures; {tests['errors']} errors; {len(tests['unexpected_failures'])} unexpected scientific failures**. Existing tests and their contracts remain unchanged. Full details are in `TESTS.txt` and `TEST_SUMMARY.json`.

New checks cover atlas initialization, integer-width identity, passive persistence, state transitions, actual dependency closure, cached coordinates, shared-row ownership, restart, rollback, complete regression coverage, optimized adapter selection, immutable proof persistence, missing/tampered inputs, deterministic proof outcomes and certificate format equivalence.

## R. Classification

**{classification}**.

All scientific equivalence and locality requirements tested here pass. The active search bottleneck remains. Calling this a new large planning speedup would overstate the evidence. No new risk threshold, scientific gate change, V3 edit, shell rebuild, A2 advancement or downstream meshing occurred.

## S. A2 readiness

**A separate controlled A2 qualification is technically justified with both integrated paths. A2 was not resumed here.** The optimized certificate provider and persistent passive path are selected by the production tools. The qualified format bridge handles the change in proof storage while checking all mathematical outcomes and mesh science exactly. Existing old-format A2 prefix proofs remain valid and retain their original audit path.

A2 must still establish complete independent rolling reproduction and measure total production runtime. This study does not claim that runtime is solved. Preserve stage timing and stop for a scientific mismatch. Do not start support prisms, core meshing or CFD.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
    source_paths=[p for p in (ROOT/'tools').glob('task4c_*.py')]
    source_paths += [ROOT/'tools'/n for n in ('task4_prepare_next.py','task4_warning_cache.py','task4_certify_candidate.py','task4_final_validation.py','task4_compare_replay.py')]
    source_paths += [ROOT/'src/sdf_cfd_mesher_v4'/p for p in ('advancing_front/risk_atlas.py','advancing_front/production_risk.py','certificate/production.py','certificate/replay_equivalence.py')]
    source_paths += [ROOT/'tests'/p for p in ('test_dynamic_risk_atlas.py','test_production_certificate_adapter.py','test_certificate_replay_equivalence.py','test_atlas_study_regression.py')]
    excluded={'AUTHORITY.json','SEAL.json'}
    authority=dict(task='ImplementV4DynamicRiskAtlasAndFastRollingPlanner',classification=classification,base_head=preflight['head'],
        frozen_Task4_authority=file_hash(BASE/'AUTHORITY.json'),frozen_Task4B_authority=file_hash(B/'AUTHORITY.json'),
        scientific_hash=det['atlas_run_1'],report_sha256=file_hash(OUT/'report.md'),V3_modified_files=0,A2_resumed=False,Build_A_rebuilt=False,
        source_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(source_paths)},
        evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in excluded and p.suffix!='.log'})
    save('AUTHORITY.json',authority);save('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    print(classification);print(json.dumps(read(OUT/'SEAL.json'),indent=2))

if __name__=='__main__':run()
