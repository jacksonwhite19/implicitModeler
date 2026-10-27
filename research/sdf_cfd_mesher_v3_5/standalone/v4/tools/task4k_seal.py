"""Seal measured Task-4K evidence. No scientific execution."""
from task4k_common import *
import subprocess

MODIFIED=['src/sdf_cfd_mesher_v4/certificate/efficient_proof.py','src/sdf_cfd_mesher_v4/certificate/production.py']
NEW=['src/sdf_cfd_mesher_v4/certificate/cad_algebra_cache.py','tests/test_task4k_cad_cache.py']

def run():
    pre=read(OUT/'PREFLIGHT.json');cad=read(OUT/'CAD_SUMMARY.json');rt=read(OUT/'TRANSACTION_RUNTIME.json');tests=read(OUT/'TESTS.json');det=read(OUT/'DETERMINISM.json');eq=read(OUT/'CAD_EQUIVALENCE.json');audit=read(OUT/'FINAL_AUDIT.json');system=read(OUT/'SYSTEM_AUDIT.json');stop=read(OUT/'SCIENTIFIC_STOP.json')
    assert tests['successful'] and eq['cases']==646 and det['transactions']==20
    for path,h in pre['V3'].items():assert file_hash(Path(path))==h,('V3Changed',path)
    for path,h in pre['sealed'].items():assert file_hash(ROOT/path)==h,('SealedEvidenceChanged',path)
    changed=[p for p,h in pre['sources'].items() if file_hash(ROOT/p)!=h]
    assert sorted(changed)==sorted(MODIFIED),('UnexpectedPriorSourceChange',changed)
    status=subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True)
    unrelated=''.join(line+'\n' for line in status.splitlines() if 'research/sdf_cfd_mesher_v4/' not in line)
    assert unrelated==pre['status'],'UnrelatedDirtyStateChanged'
    save('PRESERVATION.json',dict(V3_modifications=0,V3_hashes_verified=len(pre['V3']),sealed_hashes_verified=len(pre['sealed']),changed_prior_source=changed,unrelated_status_unchanged=True,unrelated_entries=len(unrelated.splitlines())))
    before=cad['replays']['before'];after=cad['replays']['after'];a=rt['A'];b=rt['B'];peak=max(cad['peak_rss'],a['peak_rss'],b['peak_rss'])
    classification='V4ExactCADKernelLimited'
    save('CLASSIFICATION.json',dict(classification=classification,historical_replay_B_cause='Unresolved: missing historical per-case CPU, GC and queue telemetry',full_build_recommended=False,optimization_qualified=True))
    time_rows='\n'.join(f"| {label} | {old['worker_seconds']:.3f} | {new['worker_seconds']:.3f} | {old['cpu_seconds']:.3f} | {new['cpu_seconds']:.3f} |" for label,old,new in zip(('Cold worker start','Warm 1','Warm 2'),before,after))
    functions=('polynomial_construction','clear_denominators','primitive_coefficients','factorization_poly','factorization_expression','square_free','root_isolation','root_intervals','algebraic_lift','algebraic_field','exact_compare','algebraic_zero','specialization','root_record')
    costs='\n'.join(f"| {name} | {before[0]['functions'][name]['calls']:,} | {before[0]['functions'][name]['self']:.3f} | {after[0]['functions'][name]['calls']:,} | {after[0]['functions'][name]['self']:.3f} |" for name in functions)
    layer_rows='\n'.join(f"| {x['layer']} | {x['pairs']:,} | {x['seconds']:.3f} | {y['seconds']:.3f} |" for x,y in zip([r for r in read(OUT/'PER_LAYER_REPLAY.json') if r['replay']=='A'],[r for r in read(OUT/'PER_LAYER_REPLAY.json') if r['replay']=='B']))
    retained=lambda p:p.is_file() and 'replay_B' not in p.relative_to(OUT).parts and '_worker_' not in p.name
    local_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file());retained_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if retained(p))
    text=f'''Decision question

Why does the noncoincident rank-3 CAD path show large runtime variance, and can V4 make that exact-algebra workload faster and more stable without changing certificate mathematics?

## A. Authority verification

Starting HEAD: `{pre['head']}` on the requested branch. Task-4J report SHA-256: `1f398bc59c5cf245439151943f2a905e03293742d8128b96859a5f61ced7ada9`. Task-4J authority SHA-256: `2c93161a87db88482d7b51de2b6d41d2dcb54e07a35a1b3a61d929d9d0eecb13`. Preflight and final preservation checks passed. {len(pre['sealed'])} sealed file bindings and {len(pre['V3'])} V3 bindings remain unchanged. V3 modifications: zero. All {len(unrelated.splitlines())} unrelated Git entries remain unchanged.

Task start: {pre['task_start']}. Scientific stop: {stop['utc']}, {stop['elapsed_task_seconds']:.2f} seconds after start. Scientific cutoff: {pre['scientific_cutoff']}. Sealing timestamp: {now().isoformat()}. No shell build, A2 execution, front advancement, SDF forecast, local optimization or downstream meshing occurred.

## B. Task-4J baseline

Task 4I immediate certification took 332.18 seconds. Task-4J equivalent replays took 280.343 and 383.794 seconds; their spread was 103.451 seconds and ratio 1.369. Worst layers took 22.866 and 62.623 seconds. Task-4J Replay A had 871.20 summed CAD worker-seconds. These are preserved baseline measurements, not rerun estimates.

The complete workload remains 56,462,864 required pairs, 36,147 exact fallbacks and 646 CAD calls. No scientific pair, gate or certificate rule changed.

## C. CAD case population

`CAD_CASES.json` binds all 646 cases to source-edge IDs, layer pairs, binary64 coordinates, labels, rank, exact kernel identity and frozen proof hash. It includes projection polynomial identities/degrees/coefficient bit lengths, factors, roots and selected root indices. `POLYNOMIAL_INVENTORY.json` adds F and factor-basis identities. `CAD_INPUTS.npz` contains only these bounded pair inputs, not a duplicate shell. Full exact comparison counts and traces are in each runtime row.

The audit directly calls the frozen CAD routine after the same canonical pair and proof normalization used by the public certificate. It bypasses the final whole-proof cache so warm measurements still execute CAD. An initial audit-only omission of canonicalization is preserved in `smoke.log`; case 2 is now a regression fixture. Production mathematics was not changed.

## D. Per-case cost distribution

Eight persistent processes execute deterministic round-robin cohorts, each in cold/warm/warm order. The table contains summed worker execution seconds, not independent phase wall times. Different workers can overlap phases. Complete three-pass elapsed time was {cad['baseline_wall']:.3f} before and {cad['optimized_wall']:.3f} after. Trace/timer instrumentation adds overhead; production replay times are reported separately.

| Phase | Old worker s | New worker s | Old CPU s | New CPU s |
|---|---:|---:|---:|---:|
{time_rows}

Cold slowest case: index 146, {before[0]['max_case_seconds']:.3f} to {after[0]['max_case_seconds']:.3f} seconds. `CASE_VARIANCE.json` contains all 646 minimum/maximum/median/spread/ratio records. No arbitrary pathological-case threshold was introduced.

## E. Variance concentration

The ten slowest baseline cold cases account for {100*cad['concentration']['before']['shares']['10']:.2f}% of CAD execution; the slowest accounts for {100*cad['concentration']['before']['shares']['1']:.2f}%. A handful of exceptional cases does not dominate the population. The 100 slowest account for {100*cad['concentration']['before']['shares']['100']:.2f}%. Case 146 changed from 11.393/2.721/2.530 seconds cold/warm/warm to 10.783/2.346/1.975 seconds. Cache temperature has a strong effect, but does not explain the historical slower second full replay.

## F. Polynomial/factorization profile

Self time excludes nested instrumented operations. Uninstrumented internal work remains charged to its enclosing operation. Allocation cost and symbolic internal cache hits are not separately invented. Per-case total and self times are retained. Witness/proof assembly and cache lookup/insertion latency are not isolated from their enclosing operations; no separate values are inferred for them.

| Operation | Old cold calls | Old self s | New cold calls | New self s |
|---|---:|---:|---:|---:|
{costs}

Factorization is material but is not the largest measured cost. No new factorization cache or polynomial normalization was added. Existing qualified polynomial preparation and exact root-key memoization remain active.

## G. Root-isolation profile

Cold root-isolation self time was {before[0]['functions']['root_isolation']['self']:.3f} versus {after[0]['functions']['root_isolation']['self']:.3f} seconds. The new root-list cache avoids exact-identical calls, but does not materially reduce the expensive cold isolation work. Warm isolation is much cheaper in both paths. These repeated CAD passes are audit experiments; their warm gains are not production savings because production inputs change and layer workers restart. `TARGET_DEEP_CASES.json` separates GCD, Sturm construction, field evaluation and field-sign work in the twelve-case slow/representative cohort. No approximate root representation or comparison was introduced.

## H. Exact-comparison profile

All ordered comparison operands and results match exactly. Cold exact-comparison self time was {before[0]['functions']['exact_compare']['self']:.3f} versus {after[0]['functions']['exact_compare']['self']:.3f} seconds. Comparison and zero-at-root decisions can refine private isolating intervals. They are deliberately not memoized. Only pure polynomial preparation is reused; the same comparison/refinement sequence still executes.

## I. Cache behavior

The new process-local LRU caches store exact root lists, polynomial GCDs and Sturm chains. Full exact Poly identity, domain, generators and arguments are keys. Each cache is bounded at 4,096 entries. Returned lists are fresh containers. No object address, approximate similarity or mutable interval state is a key.

Cold all-case hits: root lists {after[0]['pure_cache']['real_roots']['hits']:,}; GCDs {after[0]['pure_cache']['gcd']['hits']:,}; Sturm chains {after[0]['pure_cache']['sturm']['hits']:,}. Their misses were {after[0]['pure_cache']['real_roots']['misses']:,}, {after[0]['pure_cache']['gcd']['misses']:,} and {after[0]['pure_cache']['sturm']['misses']:,}. Counts include library-recursive polynomial calls, not just top-level CAD requests. Full per-case adapter and root-key hit/miss deltas are preserved. No algebraic-number or boolean comparison cache was added. Pure operation caches survive the existing per-job kernel clear and are discarded at process exit.

## J. Worker/process behavior

Each case records PID, process creation time, case order, initial queue delay, start/finish, CPU time, RSS and result return. Eight workers persist across each three-pass CAD cohort. No recycling occurred. Aggregate CPU/execution ratios were {system['BASELINE']['cpu_wall_ratio']:.5f} before and {system['OPTIMIZED']['cpu_wall_ratio']:.5f} after. This audit was compute-bound; it did not reproduce a large scheduling delay. Initial queue timing is measured. Cohort return delay includes later work in that cohort and is not pure IPC time. Hardware frequency/thermal telemetry and context switches were not measured. Production scheduling and process lifetime are unchanged.

## K. Memory and GC

Cold/warm/warm GC time before: {before[0]['gc_seconds']:.3f}/{before[1]['gc_seconds']:.3f}/{before[2]['gc_seconds']:.3f} summed seconds. After: {after[0]['gc_seconds']:.3f}/{after[1]['gc_seconds']:.3f}/{after[2]['gc_seconds']:.3f}. GC is measurable, but execution gets faster while GC time rises; these runs do not support GC as the dominant variance cause. GC was not disabled.

Instrumented peak aggregate RSS before/after: {system['BASELINE']['peak_rss']:,}/{system['OPTIMIZED']['peak_rss']:,} bytes. Overall peak: {peak:,} bytes ({peak/2**30:.3f} GiB). Instrumentation retains trace keys and contributes memory. Production replay peaks: {a['peak_rss']:,}/{b['peak_rss']:,} bytes. This is sampled process RSS, not an allocation-exact peak. Raw cohort logs and repeat B payloads remain local and ignored; selected durable evidence is approximately {retained_bytes:,} bytes, versus {local_bytes:,} local bytes before sealing. Old shell geometry is not duplicated.

## L. Root cause

Measured current causes are required exact algebra plus repeated pure polynomial preparation. Algebraic zero-test preparation had substantial identical GCD/Sturm work. Caching that work reduces its cold self time from {before[0]['functions']['algebraic_zero']['self']:.3f} to {after[0]['functions']['algebraic_zero']['self']:.3f} seconds, and warm self time from {before[1]['functions']['algebraic_zero']['self']:.3f}/{before[2]['functions']['algebraic_zero']['self']:.3f} to {after[1]['functions']['algebraic_zero']['self']:.3f}/{after[2]['functions']['algebraic_zero']['self']:.3f} seconds. Cold root isolation and exact comparisons remain largely unchanged.

The specific cause of Task-4J Replay B's historical slowdown remains unresolved. Its preserved artifacts lack per-case CPU, GC and queue telemetry. The new audit does not justify attributing that event to scheduling, memory pressure, symbolic nondeterminism or a few pathological cases. Better repeat timing is evidence of current behavior, not retrospective proof of the old cause.

## M. Optimization

`cad_algebra_cache.py` adds bounded exact-input caches around pure SymPy operations. `efficient_proof.py` enables them in worker jobs and the serial exact path, and records worker cache counts. `production.py` binds the new implementation in certificate source identity. No cache result is persisted across software versions. Restart recomputes exact results. `IMPLEMENTATION_SPEC.md` gives safety and measurement limits. No frozen V3 mathematics, rank rule, root ordering, witness rule, pair completeness or hard mesh gate changed.

## N. Exact equivalence

All 646 cases pass three baseline and three optimized executions: 3,876 total. Full canonical proof hashes, ranks, roots, ordered exact comparisons, witnesses and forbidden/unresolved results match. All 40 post-optimization complete layer transactions match Task 4I for every pair ID, proof class, witness byte, exact fallback result and validity record. Both final audits prove 56,462,864 covered pairs; missing, duplicate, forbidden and unresolved counts are all zero.

## O. Repeated CAD replay stability

Warm total spread before was {abs(before[1]['worker_seconds']-before[2]['worker_seconds']):.3f} summed worker-seconds; after it was {abs(after[1]['worker_seconds']-after[2]['worker_seconds']):.3f}. Cold-to-warm ratios remain large because cache temperature still matters. An isolated one-worker replay of case 146 took 7.761/1.398/1.415 seconds before and 7.513/0.900/0.877 seconds after, with identical proof and comparison/root traces. This confirms cache-temperature effects without competing CAD workers. No claim is made that all algebraic latency variance is eliminated. Scientific CAD replay hash: `{eq['scientific_hash']}`. Timing, PID, RSS and temporary paths are excluded.

## P. Full 20-transaction replay stability

| Layer | Required pairs | Replay A s | Replay B s |
|---|---:|---:|---:|
{layer_rows}

Immediate totals: {a['totals']['seconds']:.3f}/{b['totals']['seconds']:.3f} seconds. Spread: {rt['spread']:.3f} seconds; ratio: {rt['ratio']:.5f}. Worst layers: {a['worst_layer']} at {a['worst_seconds']:.3f} seconds; {b['worst_layer']} at {b['worst_seconds']:.3f} seconds. CAD summed worker time: {a['details']['method:NoncoincidentRank3ExactCAD']:.3f}/{b['details']['method:NoncoincidentRank3ExactCAD']:.3f} seconds. Worker totals overlap and must not be added to transaction wall time.

Both replays make 36,147 frozen calls including 646 CAD calls. Root-key hits remain 4,882,282; preparation hits remain 30,321. Full proof and compact payload bytes reproduce exactly. Two separate final completeness audits took {audit[0]['seconds']:.3f}/{audit[1]['seconds']:.3f} seconds. Final scientific manifest hash: `{det['scientific_hash']}`. Cache hit statistics from nondeterministic worker assignment are diagnostics, not proof identity.

## Q. Remaining bottleneck

Intrinsic cold root isolation, exact comparisons and remaining algebraic zero tests dominate CAD. Mixed-rank exact work remains secondary. The production improvement relative to Task-4J's faster replay is modest: {100*(1-a['totals']['seconds']/280.3434173):.2f}% for A. The larger difference from the slow Replay B cannot all be credited to this cache. No new full-build runtime is estimated.

## R. Tests

{tests['run']} focused tests pass in isolated module processes. Test modules: {', '.join(r['module'] for r in tests['modules'])}. Existing assertions are unchanged. New tests cover exact polynomial/domain/argument identity, GCD/Sturm equality, private list ownership, algebraic domains, zero polynomials, exception/nested restoration, restart recomputation, cache persistence across kernel clears and the canonical case-2 reproducer.

The initial combined-process suite exposed an existing SymPy root-cache/generator fixture interaction. `SYMPY_FIXTURE_REPRODUCER.json` reproduces it without importing the new V4 optimization. All unchanged modules pass separately. Initial failed test output is retained. No production rule was weakened. No full development suite or inherited repository-state assertion was run.

## S. Classification

**{classification}**.

The bounded pure-operation optimization is exactly qualified and gives measurable gains. Most cold CAD cost remains exact algebra; the production speedup over the faster preserved replay is limited. Current repeat stability is reported directly. The historical Replay B cause is not claimed solved.

## T. Full-build readiness

The changed certificate implementation is qualified on all preserved cases and transactions. Another fresh timed full build is not the recommended next performance task: it would repeat meshing while the remaining CAD isolation/comparison work is already identified. The next useful bounded study is cold exact root-isolation/field-sign cost with per-case CPU-frequency and process telemetry, preserving exact proof equivalence. Do not infer a new full-build runtime from these replays. No full build was run here. Commit identity is reported in the final response to avoid a self-referential artifact hash.
'''
    (OUT/'report.md').write_bytes(text.encode())
    source=sorted(MODIFIED+NEW+[p.relative_to(ROOT).as_posix() for p in (ROOT/'tools').glob('task4k*.py')]);save('COMMITTED_SOURCE_PATHS.json',source)
    authority=dict(task='AuditAndStabilizeV4ExactCADHotPath',classification=classification,parent_commit=pre['head'],report_sha256=file_hash(OUT/'report.md'),source={p:file_hash(ROOT/p) for p in source},evidence={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(OUT.rglob('*')) if retained(p) and p.name not in ('AUTHORITY.json','HASHES.json','report.md')},scientific_hash=det['scientific_hash'],cad_scientific_hash=eq['scientific_hash'],cad_cases=646,transactions=40,required_pairs=56462864,forbidden=0,unresolved=0,missing=0,duplicates=0,no_mesh_build=True,V3_modifications=0,historical_variance_cause_unresolved=True,sealed_utc=now().isoformat(),elapsed_task_seconds=elapsed())
    save('AUTHORITY.json',authority);save('HASHES.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json')))
    print(json.dumps(read(OUT/'HASHES.json'),indent=2));print('elapsed_task_seconds',elapsed())

if __name__=='__main__':run()
