"""Seal the measured certificate study; no meshing or scientific replay."""
from task4j_common import *

MODIFIED=['src/sdf_cfd_mesher_v4/certificate/efficient_proof.py','src/sdf_cfd_mesher_v4/certificate/production.py']
NEW=['src/sdf_cfd_mesher_v4/certificate/'+n for n in ('exact_preparation.py','parallel_screen.py','compact_storage.py')]
def run():
    pre=read(OUT/'PREFLIGHT.json');runtime=read(OUT/'RUNTIME_COMPARISON.json');baseline=read(OUT/'BASELINE_PROFILE.json');tests=read(OUT/'TESTS.json');det=read(OUT/'DETERMINISM.json');audit=read(OUT/'FINAL_AUDIT.json');rows=read(OUT/'PER_LAYER_COMPARISON.json');control=read(OUT/'CONTROLLED_ROOT_PREPARATION.json')
    assert tests['successful'] and det['transactions']==20
    for path,expected in pre['V3'].items():assert file_hash(Path(path))==expected,('V3Changed',path)
    for path,expected in pre['sealed_evidence'].items():assert file_hash(ROOT/path)==expected,('SealedEvidenceChanged',path)
    changed=[p for p,h in pre['source_before'].items() if file_hash(ROOT/p)!=h]
    assert sorted(changed)==sorted(MODIFIED),('UnexpectedSourceChange',changed)
    status=subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True)
    unrelated=''.join(line+'\n' for line in status.splitlines() if 'research/sdf_cfd_mesher_v4/' not in line)
    assert unrelated==pre['status'],'UnrelatedGitStateChanged'
    save('PRESERVATION.json',dict(V3_modifications=0,V3_hashes_verified=len(pre['V3']),sealed_evidence_verified=len(pre['sealed_evidence']),changed_prior_source=changed,unrelated_status_unchanged=True,unrelated_entries=len(unrelated.splitlines())))
    a=runtime['new_A'];b=runtime['new_B'];details=runtime['new_worker_details_A'];old=baseline['aggregate_timing'];stop=read(OUT/'SCIENTIFIC_STOP.json');classification='V4ImmediateCertificationSubstantiallyImproved'
    cold='\n'.join(f"| {r['layer']} | {r['mode']} | {r['seconds']:.3f} | {r['exact_calls']:,} |" for r in control)
    storage_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    retained_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file() and not any(n in p.relative_to(OUT).parts for n in ('replay_B','replay_bounded')))
    text=f'''Decision question

Why does immediate exact certification still consume about 332 seconds, and can V4 reduce that runtime without changing any continuous-geometry guarantee?

## A. Authority verification

Starting HEAD: `{pre['head']}` on the requested branch. Task-4I report SHA-256:
`11dbda04f3f81ec5c7a7ea34c81677f94239ec2701e790d1a2928fd466898cb4`.
Task-4I authority SHA-256:
`c8e9eaf2f82431cae65875661064540dc570e5d8734cf0f4dc258b46d259f807`.
Preflight verified {len(pre['source_before'])} source bindings, {len(pre['sealed_evidence'])} sealed evidence bindings and {len(pre['V3'])} V3 bindings. Final preservation checks passed. Only the two listed existing V4 certificate files changed. New files are confined to Task 4J. V3 modifications: zero. All {len(unrelated.splitlines())} unrelated Git entries remain unchanged.

Task start: {pre['task_start']}. Scientific stop: {stop['utc']}, {stop['elapsed_task_seconds']:.2f} seconds after task start. Scientific cutoff: {pre['scientific_cutoff']}. Sealing timestamp: {now().isoformat()}. No fresh shell construction, A2 run, planner execution or downstream meshing occurred.

## B. Task-4I certificate baseline

The preserved qualified shell has 20 layers and 2,126,040 prisms. Its immediate certificate covers 56,462,864 required pairs. Frozen reported certificate time was 332.177986 seconds; the sum of inner adapter timers was 331.709526 seconds. The difference is outer instrumentation. Final audit baseline: 29.01 seconds. Task-4I full-build scientific runtime remains 1,312.36 seconds; this study does not replace it with a replay-derived estimate.

## C. Per-layer runtime profile

Both new replays use the same eight exact workers as Task 4I. Each transaction consumes preserved coordinates, reconstructs required certificate inputs, enumerates pairs, proves them, writes evidence and verifies coverage. Geometry is never generated or corrected.

{(OUT/'LAYER_TABLE.md').read_text()}

The primary replay's worst layer is {runtime['new_worst']['layer']} at {runtime['new_worst']['new_seconds']:.3f} seconds. The baseline worst layer is {runtime['old_worst']['layer']} at {runtime['old_worst']['old_seconds']:.3f} seconds. The independent repeat has substantial late timing variation; its worst layer and every measured time remain in `REPLAY_B.json`. Do not interpret the primary series as a uniform per-layer speedup.

## D. Proof-class cost profile

The complete population and all proof classes remain identical:

| Class | Count |
|---|---:|
| Certified supporting-plane separation | 6,699,515 |
| Certified owned-vertex contact | 30,566,340 |
| Certified owned-edge contact | 19,144,627 |
| Accumulated certified planes, owned vertex | 16,235 |
| Frozen exact fallback | 36,147 |

Fallback methods: 13,478 mixed rank-2/3; 16,830 fast owned supporting-plane; 5,189 rank-2 exact; 646 noncoincident rank-3 CAD; four fast supporting-plane calls. Rank-0, rank-1 and coincident rank-3 calls were not observed in these immediate populations. No timing is invented for unobserved branches. Topology alone excludes zero required pairs. Strict AABB complement counts are recorded per layer in `BASELINE_PROFILE.json`.

Frozen worker elapsed times were 876.16 seconds in noncoincident CAD and 299.57 seconds in mixed-rank work. These overlap across processes and are not wall-time totals. In primary replay A, these are {details['method:NoncoincidentRank3ExactCAD']:.2f} and {details['method:Rank2ExactMixedPlane']:.2f} seconds. Exact arithmetic remains dominant.

The 37-call bounded profile provides call counts, self/cumulative time, mean and sampled maximum latency. Nine CAD calls consumed 24.526 instrumented seconds; root preparation and comparisons dominate its call tree. These are cProfile-instrumented measurements, not uninstrumented whole-run latency estimates. `PROOF_CLASS_COST.json`, `CALL_TREE.json` and `NATIVE_CLASS_PROFILE.json` hold the detailed measurements.

## E. Dispatch and batching analysis

Baseline Python submission dispatch was {old['dispatch_seconds']:.3f} seconds; new A is {a['dispatch_seconds']:.3f} seconds. It was not the dominant cost. Conservative proofs already ran in one native batch, not one Python call per pair.

The new screen uses eight contiguous native slices, immutable shared inputs and disjoint output bytes. Native calls increase from 20 to 160 across the replay; each pair still receives the same native function. This is parallel scheduling, not fewer mathematical operations. Native screening decreased from {old['filter_seconds']:.3f} to {a['filter_seconds']:.3f} seconds in A. Every output code and witness is identical.

Layer-12 exact batches of 8, 32 and 64 pairs took 17.261, 24.241 and 31.093 seconds. Larger batches were rejected. Production retains eight-pair jobs and canonical submission/result order. No native rank preclassification or mathematical dispatch was introduced.

## F. Structural setup reuse

Pure exact polynomial preparation is cached within each worker job, with a 4,096-entry bound. Complete immutable polynomial/domain/generator identity controls reuse. It does not cache sign decisions, root comparisons or interval-refinement effects. Existing per-pair refinement reset remains. Primary A records {details['root_preparation:hits']:,} preparation hits and {details['root_preparation:misses']:,} misses across {details['root_preparation:requests']:,} requests, including constant/zero early returns. No stale or approximate algebraic reuse occurs.

Unchanged earlier proof payloads are not loaded or rebound by a later transaction. Prefix geometry preparation ({a['geometry_preparation_seconds']:.3f} seconds) and BVH enumeration ({a['enumeration_seconds']:.3f} seconds) remain. They were not the primary measured target.

## G. Root/polynomial reuse

The existing exact root-key memoizer records {details['root_keys:hits']:,} hits and {details['root_keys:misses']:,} misses in A. All cache counts reproduce in B.

The initial same-process 37-call experiment measured 9.378 seconds without the preparation cache and 5.574 seconds with it. That comparison can include SymPy warming. The cold-worker control below is the stronger attribution evidence:

| Layer | Mode | Seconds | Exact calls |
|---:|---|---:|---:|
{cold}

Full worker kernel time did not materially fall relative to the frozen baseline: {old['kernel']:.2f} versus {details['kernel']:.2f} overlapping worker seconds in A. Do not claim the small-cohort ratio as a complete-workload speedup. The main demonstrated end-to-end gains are screening and compression.

## H. Proof transaction architecture

The existing compact evidence model remains. Canonical packed pair IDs, proof codes, witness bytes, outcomes and validity arrays preserve every scientific value. Rich exact fallback JSON remains complete. NPZ compression changes from level six to level one, with fixed member dates and ordering. Rich gzip content is unchanged. No proof is dropped or deferred without evidence.

For layers 2/12/20, isolated compact writes fell from 1.002/1.058/1.057 seconds to 0.229/0.260/0.252 seconds, at a small measured storage increase. Complete serialization fell from {old['serialization_seconds']:.3f} to {a['serialization_seconds']:.3f} seconds in A. New components: compact writes {a['compact_serialization_seconds']:.3f}, rich writes {a['rich_serialization_seconds']:.3f}, manifest writes {a['manifest_serialization_seconds']:.3f} seconds. Hash/metadata binding costs {a['binding_seconds']:.3f} seconds. Pair ordering was already canonical; no sorting reduction is claimed.

`IMPLEMENTATION_SPEC.md` states safety and identity rules. The production digest includes all new implementation modules. Historical source bindings are not weakened or rewritten. New-source transactions are verified under the new binding; old/new proof fields are compared explicitly.

## I. Bounded equivalence

Early corrected layers 1/2, slow layer 12 and late nominal layer 20 passed. Native batch probes cover complete required populations on layers 2/12/20 at 1/2/4/8 threads. Exact batch probes compare full rich results at 8/32/64 pairs. Synthetic fixtures include forbidden contact, invalid inputs, exact algebraic domains, cache restoration, native proof witnesses and deterministic storage. No geometry or gate changes were required.

## J. Full 20-layer certificate replay

All 20 transactions in both A and B match Task 4I for required pair IDs, class codes, witnesses, outcomes, all rich exact fallback results, topology/geometry bindings and validity records. Each replay makes 36,147 frozen-kernel calls including all 646 CAD calls. Forbidden and unresolved sets are empty. Both replays independently calculate proofs; neither loads old results as acceptance inputs.

## K. Final audit replay

Separate final audits took {audit[0]['seconds']:.3f} and {audit[1]['seconds']:.3f} seconds, versus 29.01 seconds in Task 4I. They re-enumerate all transaction populations, verify source/geometry identity, strict uniqueness, immutable payload hashes and exact outcome coverage. Ownership by newest layer proves transactions do not duplicate each other's pairs.

Final coverage: 56,462,864; missing: zero; duplicate coverage: zero; forbidden: zero; unresolved: zero. All 56,462,864 proofs are reused, with zero new algebraic solves during the audit. The final manifest's scientific hash is `{det['scientific_hash']}`.

## L. Runtime comparison

| Stage | Task 4I s | Primary replay A s | Repeat B s |
|---|---:|---:|---:|
| Immediate certification | 332.178 | {a['seconds']:.3f} | {b['seconds']:.3f} |
| Native screening | {old['filter_seconds']:.3f} | {a['filter_seconds']:.3f} | {b['filter_seconds']:.3f} |
| Remaining exact phase | {old['remaining_exact_seconds']:.3f} | {a['remaining_exact_seconds']:.3f} | {b['remaining_exact_seconds']:.3f} |
| Serialization | {old['serialization_seconds']:.3f} | {a['serialization_seconds']:.3f} | {b['serialization_seconds']:.3f} |
| Enumeration | {old['enumeration_seconds']:.3f} | {a['enumeration_seconds']:.3f} | {b['enumeration_seconds']:.3f} |

A reduces certificate wall time by {runtime['reduction_percent']:.2f}% ({runtime['ratio']:.3f}x baseline/new). B has marked late timing drift despite identical operation and cache counts. One system-load sample is retained; it does not establish the cause. The evidence qualifies exact equivalence and isolated implementation gains, but does not establish a stable new full-build runtime or uniform speedup. No full-build time is extrapolated.

## M. Remaining bottleneck

The exact phase remains dominant. Noncoincident rank-3 CAD accounts for most overlapping worker time, followed by mixed-rank calculations. The next measured target is exact polynomial preparation, factorization and root isolation inside CAD. Larger job batches and further per-pair Python dispatch work are not supported as useful next changes by this study.

## N. Memory

Peak sampled aggregate parent/worker RSS: {runtime['peak_sampled_rss']:,} bytes ({runtime['peak_sampled_rss']/2**30:.3f} GiB), sampled every 0.1 seconds. This is not an allocation-exact peak. Per-replay resource records are retained. Persistent evidence before this report/seal is {storage_bytes:,} bytes locally; {retained_bytes:,} bytes are selected for durable evidence. One complete optimized proof payload set is committed. Automatic approval review rejected removal of duplicate B/bounded replay folders with the reason "blocked by policy". Those folders remain intact locally and are ignored for Git; their comparison hashes/tables are retained. No Build-A/Task-4I geometry is duplicated.

## O. Determinism

Both complete replays reproduce pair populations, classes, witnesses, exact outcomes, root-cache counts, compact NPZ bytes and rich gzip bytes exactly. Two final audits reproduce the manifest exactly. Scientific hashes exclude timing, RSS and temporary paths. Timing variance is not a scientific divergence.

## P. Tests

{tests['run']} focused tests passed; {tests['failures']} failures; {tests['errors']} errors; {tests['skipped']} skipped. Modules: {', '.join(tests['modules'])}. Runtime: {tests['seconds']:.3f} seconds. Existing relevant certificate, production adapter, format bridge and incremental tests were retained and run. No full development suite or inherited repository-state assertion was run. Initial new fixture setup errors used unsupported polynomial domains; the fixtures were corrected to the frozen kernel's supported exact domains. No certificate rule was changed to satisfy a fixture.

## Q. Classification

**{classification}**.

Complete exact equivalence is established. Measured native screening and storage gains reduce primary replay runtime. A major exact-arithmetic bottleneck and substantial repeat timing variation remain; this is not a claim that certificate runtime is fully resolved.

## R. Full-build readiness

The integrated certificate path is qualified for a controlled fresh build when requested. Another full build is not the most useful next performance experiment: first target the remaining measured CAD work and establish stable certificate timing. No fresh build was run here. All hard gates and certificate mathematics remain frozen. Git commit identity is reported after sealing in the final response to avoid a self-referential artifact hash.
'''
    (OUT/'report.md').write_bytes(text.encode())
    source=MODIFIED+NEW+[p.relative_to(ROOT).as_posix() for p in (ROOT/'tools').glob('task4j*.py')]+['tests/test_task4j_certification.py']
    source=sorted(set(source));save('COMMITTED_SOURCE_PATHS.json',source)
    authority=dict(task='OptimizeV4ImmediateCertificationKernelAndDispatch',classification=classification,parent_commit=pre['head'],report_sha256=file_hash(OUT/'report.md'),source={p:file_hash(ROOT/p) for p in source},evidence={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json','report.md') and not any(n in p.relative_to(OUT).parts for n in ('replay_B','replay_bounded'))},scientific_hash=det['scientific_hash'],required_pairs=56462864,forbidden=0,unresolved=0,missing=0,duplicates=0,no_mesh_build=True,V3_modifications=0,sealed_utc=now().isoformat(),elapsed_task_seconds=elapsed())
    save('AUTHORITY.json',authority);save('HASHES.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json')))
    print(json.dumps(read(OUT/'HASHES.json'),indent=2));print('elapsed_task_seconds',elapsed())
if __name__=='__main__':run()
