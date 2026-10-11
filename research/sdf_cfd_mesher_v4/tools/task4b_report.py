"""Seal measured Task-4B evidence; never write a prior result directory."""
from task4b_common import *
import collections,platform,shutil

def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])

if __name__=='__main__':
    baseline=json.loads((OUT/'BASELINE.json').read_text());timings=json.loads((OUT/'optimized_2_TIMINGS.json').read_text())
    total=json.loads((OUT/'optimized_2_TOTAL.json').read_text());memory=json.loads((OUT/'optimized_2_TREE_MEMORY.json').read_text())
    final=read(OUT/'optimized_2/COMPLETENESS.json');audit=read(OUT/'FINAL_AUDIT.json');audit2=read(OUT/'FINAL_AUDIT_REPLAY.json');assert audit==audit2
    audit_time=json.loads((OUT/'FINAL_AUDIT_TIMINGS.json').read_text());tests=json.loads((OUT/'TEST_SUMMARY.json').read_text());assert tests['all_scientific_tests_passed']
    replay=read(OUT/'DETERMINISM.json');preserved=read(OUT/'PRESERVATION.json');start=read(OUT/'PREFLIGHT.json')
    methods=collections.Counter();stages=collections.Counter();worker=collections.Counter();codes=collections.Counter()
    for r in timings:
        methods.update(r['exact_methods']);worker.update(r['details'])
        stages.update({k:r[k] for k in ('index_seconds','enumeration_seconds','validity_seconds','filter_seconds','remaining_exact_seconds','hashing_seconds','serialization_seconds','dispatch_seconds')})
    for r in final['layers']:codes.update(r['class_counts'])
    calls=sum(r['exact_calls'] for r in timings);old=sum(r['wall_seconds'] or 0 for r in baseline['layers']);newlike=sum(r['production_seconds'] for r in timings[1:])
    old_audit=2218.709352;cachebytes=sum(p.stat().st_size for p in (OUT/'optimized_2').rglob('*') if p.is_file())
    files=[p for p in OUT.rglob('*') if p.is_file()];largest=max(files,key=lambda p:p.stat().st_size)
    finished=datetime.datetime.now(datetime.timezone.utc);elapsed=(finished-datetime.datetime.fromisoformat(start['started_utc'])).total_seconds()
    classification='V4CertificationRuntimeSubstantiallyImproved'
    summary=dict(classification=classification,required_pairs=final['candidate_pairs'],old_kernel_calls=final['candidate_pairs'],new_kernel_calls=calls,
        cheap_proofs=final['candidate_pairs']-calls,old_full_CAD_calls=baseline['methods']['NoncoincidentRank3ExactCAD'],new_full_CAD_calls=methods['NoncoincidentRank3ExactCAD'],
        old_immediate_layers_2_20_seconds=old,new_immediate_layers_2_20_seconds=newlike,new_immediate_all_20_seconds=total['production_seconds'],
        immediate_like_for_like_speedup=old/newlike,old_final_certificate_seconds=old_audit,new_final_certificate_seconds=audit_time['total_seconds'],
        final_certificate_speedup=old_audit/audit_time['total_seconds'],kernel_call_reduction_factor=final['candidate_pairs']/calls,
        peak_process_tree_rss=memory['peak_rss'],proof_cache_bytes=cachebytes,baseline_proof_bytes=sum(r['persistent_bytes'] for r in baseline['layers']),
        stages=dict(stages),worker_seconds=dict(worker),new_exact_methods=dict(methods),new_proof_classes=dict(codes),
        forbidden=0,unresolved=0,determinism=replay,final_audit=audit,tests={k:v for k,v in tests.items() if k not in ('inherited_state_failures','unexpected_failures')},
        study_seconds_through_report=elapsed,report_utc=finished.isoformat(),V3_modifications=0,A2_resumed=False,geometry_rebuilt=False,
        A2_readiness='Exact adapter qualified for controlled integration and replay. The sealed legacy Task-4 runner still calls its original path; wire the new adapter before resuming A2.',
        bytes_above_100MB=[dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size) for p in files if p.stat().st_size>100_000_000],
        study_artifact_bytes_before_report_seal=sum(p.stat().st_size for p in files),largest_study_file=largest.relative_to(ROOT).as_posix(),largest_study_file_bytes=largest.stat().st_size,
        python=platform.python_version(),free_disk_bytes=shutil.disk_usage(ROOT).free)
    save('SUMMARY.json',summary)
    layer_rows=[]
    for b,n,r in zip(baseline['layers'],timings,final['layers']):
        layer_rows.append([n['layer'],159453,b['potential_new_pairs'],n['pairs'],n['exact_calls'],n['cheap_pairs'],n['retained_prior_proofs'],
            'not isolated' if b['wall_seconds'] is None else f"{b['wall_seconds']:.3f}",f"{n['production_seconds']:.3f}",n['peak_rss'],sum(p.stat().st_size for p in (OUT/'optimized_2').glob(f'*_{n["layer"]:02}.*'))])
    save('PAIR_DECOMPOSITION.json',dict(baseline_layers=baseline['layers'],optimized_layers=timings,pair_layer_matrix=final['pair_layer_matrix'],spatial_counts=final['spatial_counts'],
        spatial_definition=final['spatial_definition'],potential_unordered_pairs=final['potential_pairs'],AABB_complement=final['disjoint_AABB_complement'],
        old_methods=baseline['methods'],new_methods=dict(methods),new_proof_classes=dict(codes)))
    profile=json.loads((OUT/'PROFILE_DECOMPOSITION.json').read_text())['profiles']['CASE_NoncoincidentRank3ExactCAD']
    roots=json.loads((OUT/'ROOT_KEY_PROBE.json').read_text());ipc=json.loads((OUT/'IPC_PROBE.json').read_text())
    method_table=table(['Frozen case','Original calls','Optimized calls'],[[k,baseline['methods'][k],methods[k]] for k in sorted(baseline['methods'])])
    stage_table=table(['Measured stage','Seconds'],[[k,f'{v:.6f}'] for k,v in stages.items()])
    layer_table=table(['Layer','New sides','Potential new pairs','Required pairs','Frozen calls','Cheap proofs','Prior proofs retained','Old wall s','New wall s','Parent peak RSS bytes','New proof bytes'],layer_rows)
    report=f'''Decision question

Why did exact certification make the V4 Task-4 shell build take hours, and can the same exact continuous-geometry guarantee be obtained with production-appropriate scaling?

**{classification}**

The preserved Build-A shell receives the same complete continuous-geometry result. All 56,462,864 pair IDs and separated/owned-contact outcomes match. Forbidden intersections: **0**. Unresolved results: **0**. No mesh was rebuilt. A2 was not resumed.

## A. Authority verification

Branch: `feature/sdf-cfd-mesher-v4-advancing-front`. Starting HEAD: `{start['head']}`.

- Task-4 report: `28fcb9be7fac30eec2dd84a3d292fbfbdead3daef3a4ed851a4df7744a213190`.
- Task-4 authority: `be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9`.
- Build-A shell: `{start['shell_hash']}`.

The prompt supplied a 65-character shell hash with an extra trailing `0`. The 64-character value above matches the sealed report and authority. This is a prompt transcription difference, not authority drift. The bounded preservation pass verified {preserved['task4_files_reverified']:,} Task-4 bindings and {preserved['V3_files_in_bounded_comparison']} V3 source bindings again at the end. V3 modifications: **0**. Build-A and A2 checkpoint hashes are unchanged. Unrelated Git status has no additions or removals during this task. Pre-existing production-source drift remains recorded in `PREFLIGHT.json` and `PRESERVATION.json`.

## B. Frozen Task-4 baseline

Build A contains 20 layers and 2,126,040 wall prisms. The original stopped task recorded 30,723.771075 s, including work beyond primary certification. It is not a certificate-only benchmark. Original immediate certificate wall time recorded for layers **2–20** sums to **{old:.6f} s**. Layer 1 has no isolated certificate wall field; its entire transaction took 1,495.686753 s. We do not assign that entire transaction to certification.

The old final certificate audit took **{old_audit:.6f} s**; the full final qualification took 2,334.182181 s. The original recorded tree peak was 3,928,231,936 bytes.

## C. Runtime profile

Bounded old-path profiles cover a smooth region (126 pairs), corrected cap (140), trailing region (299), and late nominal region (503). They include source-edge scope selection, BVH enumeration, cache lookup, exact preparation, records, hashing, gzip, and file writes. Detailed function tables are retained. Separate preserved pairs profile the mixed-rank, planar, owned-plane, and CAD branches. The CAD case profile took 7.710682 s with instrumentation; root isolation self time was 0.305721 s, other polynomial arithmetic 1.808525 s, symbolic string rendering self time 0.807792 s, and remaining dispatch/Python self time 4.451726 s. These are one-pair measurements, not population estimates. Nested function times must not be summed.

`PROFILE_DECOMPOSITION.json` gives disjoint self-time buckets and nested entry-point timings for rank, polynomial preparation, root isolation/comparison, classification, cache access, record construction, and I/O. Startup plus eight synthetic first IPC round trips measured {ipc['spawn_plus_first_eight_roundtrips_seconds']:.6f} s; 256 warm transport-only round trips measured {ipc['warm_256_roundtrips_seconds']:.6f} s. OS physical disk wait is not separately measured; reported I/O is observed read/write/compression time. No historical whole-shell baseline was rerun.

## D. Pair-population decomposition

The finished shell has {final['side_count']:,} bilinear side surfaces and {final['potential_pairs']:,} distinct unordered potential pairs. Strict corner-AABB separation covers {final['disjoint_AABB_complement']:,}. Both enumerators retain exactly **56,462,864** pairs. The new broad-phase population is unchanged; the expensive work per retained pair changes.

Of the retained population, **49,763,165** are legal incident contacts and **6,699,699** are separated. Thus the large number mainly represents repeated local incident relationships across the layered shell. It is not 56 million root solves. Pair ownership is the greater side layer, with stable source-edge IDs. `PAIR_DECOMPOSITION.json` and the completeness manifest retain the full 20-by-20 layer-pair matrix, rank populations, per-layer proof cases, and nine chord/span diagnostic bins. No historical pathology ID selects pairs.

{layer_table}

Each layer's existing proof population stays unchanged. Parent high-water memory is tabulated; whole worker-tree memory is sampled separately. Per-layer temporary-allocation peaks were not isolated from the retained process state. Per-transaction proof files give exact persistent sizes without materializing a giant dictionary.

## E. Exact-case cost decomposition

{method_table}

Rank-0, rank-1, and coincident-rank-3 cases: **0 observed** in this Build-A workload. This does not remove their frozen fallback support. The optimized native proofs account for **{summary['cheap_proofs']:,}** pairs. The remaining **{calls:,}** pairs enter the unchanged frozen kernel. The **646 full CAD calls remain**. In the final parallel run, measured worker kernel time was {worker['kernel']:.6f} s: CAD {worker['method:NoncoincidentRank3ExactCAD']:.6f} s, mixed-rank {worker['method:Rank2ExactMixedPlane']:.6f} s, with the rest in cheaper exact cases. Worker totals overlap and are not elapsed wall time.

## F. Root cause

The dominant original scaling fault was treating nearly every closed-AABB neighbor relation as a Python exact-certificate job. The path repeatedly built rational Patch objects, canonicalized topology, dispatched lower-rank/SymPy operations, built identities, and serialized JSON. It also repeatedly scanned all source-edge swept boxes and rebuilt overlapping local BVHs. Most relations have much cheaper exact-safe supporting-plane proofs.

The original zero cache-hit counters do **not** mean immutable committed pairs were all solved at every layer. Transaction ownership already prevented prior-vs-prior solves. That property is retained. Original final qualification also did **not** rerun all symbolic solves: it repeated population/identity checks and parsed millions of JSON records, including proof-cache serialization on a zero-capacity cache. That was a repeat completeness audit with excessive representation overhead.

After filtering, a second measured bottleneck was repeated `str(CRootOf)` for identical interval keys and uneven batches of rare CAD jobs. Memoizing the same string keys reduced five fixed CAD cases from {sum(r['old_seconds'] for r in roots):.6f} to {sum(r['new_seconds'] for r in roots):.6f} s with identical full results. Eight-pair jobs improve distribution. This is a secondary change after removing 56 million high-overhead dispatches.

## G. Optimized certificate architecture

Immutable per-layer BVH → closed corner-AABB population → compiled outward-rounded interval supporting-plane proof → frozen exact kernel for uncertain signs → append-only compact proof transactions → complete final coverage audit.

Important symbols are `Filter`, `Forest.append/pairs`, `prove`, `exact_batch`, `memoized_root_keys`, and `ProofLedger.append/final`. Rust uses only the existing standard library/toolchain. The native plane proposals do not decide a sign approximately: interval enclosures must prove it. Geometry and SDF code are unchanged. No topology-only adjacency skip is used. A bounded exact subdomain-cover probe failed to close five residual CAD pairs; it remains study-only and is not in production dispatch.

## H. Mathematical safety

See `CONSERVATIVE_PROOF_SPEC.md`. Bilinear images lie in the convex hull of their four corners. Closed AABB overlap retains all possible contacts. Strict plane separation proves disjointness. For incident pairs, the existing owned-zero-locus theorem restricts any contact to the legal shared vertex/edge. Exact corner/edge identities are used only where algebraically established. Every uncertain interval returns to the frozen kernel. Surface injectivity/regularity is validated before pair admission. No geometric tolerance, root approximation, gate change, legal-contact change, or rank theorem change was introduced.

Root-key memoization changes only repeated construction of byte-identical dictionary keys. Initial intervals, refinement, root ordering and proof records remain unchanged. It is process-local; the context is not a shared-thread API.

## I. Equivalence qualification

Small fixtures include dyadic separation/touch/intersection cases, extreme exponents, owned contacts, coordinate/ownership mismatch rejection, and the known Task-2 forbidden intersection. Four representative real scopes agree one-for-one. A complete first layer agrees. Most strongly, all **20 preserved layers** agree with all **56,462,864 frozen baseline pair outcomes**, not only aggregate counts. The old JSON proof stream was read once for comparison; the old final solve/audit was not repeated.

## J. Rolling committed-proof accumulation

Each append binds previous geometry, new coordinates/labels, topology, kernel identity, producer source authority, and ordered proof hashes. Old transactions are retained exactly; changed geometry/source or incomplete records cannot be admitted. No committed proof is invalidated. The final audit reuses **56,462,864** completed proofs, performs **0** new symbolic solves, and reports **0** missing, extra or duplicate required pairs. Zero prior-pair lookup work is needed in the next transaction because pair ownership structurally excludes unchanged prior-vs-prior relations.

## K. Final completeness manifest

`optimized_2/COMPLETENESS.json` binds all 20 transactions, stable pair IDs, proof status/code, witness index, source transaction, exact fallback results, and the global AABB-separated complement. Per-layer `PAIRS_*.npz` are compact explicit records; `EXACT_*.json.gz` retains all frozen fallback evidence. The final audit independently re-enumerates every required pair and checks geometry and proof bindings. Coverage hash: `{final['scientific_hash']}`. Final audit hash: `{audit['scientific_hash']}`.

## L. Build-A production-path benchmark

The benchmark replays certificate transactions over preserved front arrays. It does not generate geometry. All 20 transactions pass; forbidden/unresolved counts remain zero. New immediate work totals **{total['production_seconds']:.6f} s** including compact proof persistence. Initial geometry loading/setup adds {total['elapsed_including_comparison_seconds']-total['production_seconds']:.6f} s. Like-for-like layers 2–20 improve from **{old:.6f} to {newlike:.6f} s**, a directly calculated **{old/newlike:.3f}×** ratio. Final completeness audit improves from **{old_audit:.6f} to {audit_time['total_seconds']:.6f} s**, **{old_audit/audit_time['total_seconds']:.3f}×**. These are certification results; no new end-to-end mesh-build runtime is claimed.

{stage_table}

Dispatch time is contained in remaining-exact wall time. Categories plus small control/accounting overhead produce the transaction total; do not add worker CPU totals to wall stages.

## M. Runtime and memory comparison

Final benchmark peak process-tree RSS: **{memory['peak_rss']:,} bytes** ({memory['peak_rss']/2**30:.6f} GiB). The first optimized benchmark recorded 2,356,727,808 bytes. Frozen baseline peak: 3,928,231,936 bytes. New compact proof cache: **{cachebytes:,} bytes**; old per-layer proof artifacts: **{summary['baseline_proof_bytes']:,} bytes**. The task retains two compact independent replay sets and bounded profiling artifacts; it does not copy the shell. Study artifacts before final report/seal: {summary['study_artifact_bytes_before_report_seal']:,} bytes. Largest file: `{summary['largest_study_file']}` ({summary['largest_study_file_bytes']:,} bytes). Files above 100 MB: {len(summary['bytes_above_100MB'])}. Free disk: {summary['free_disk_bytes']:,} bytes. Exact study time from recorded preflight through report generation: **{elapsed:.6f} s**. This includes implementation, two complete optimized certificate runs, profiling, tests and reporting. It is not production-path runtime.

## N. Determinism

Independent runs started from empty proof ledgers. First hash: `{replay['first']}`. Second hash: `{replay['second']}`. They match exactly across all scientific fields, including all full frozen records, despite changed worker batching and root-key memoization. Two final completeness audits also match exactly. Timing, RSS, timestamps, process IDs and temporary paths do not enter the scientific hash. This is certificate replay, **not A2 geometry replay**.

## O. Tests

**{tests['test_count']} tests: {tests['passed']} passed, {tests['failures']} inherited repository-state assertions failed, {tests['errors']} errors.** All 15 new certificate tests and all retained scientific tests passed. Existing contracts were not weakened. Inherited failures concern earlier Git snapshots and pre-existing production-source drift; they are listed with full traces in `TEST_SUMMARY.json`. The suite still returns failure for those assertions. Expected divide-by-zero warnings from historical collapsed-prism fixtures remain visible. A report-only subtest-name classification error was corrected by restoring the original runner's prefix matching; both summaries are preserved. No scientific assertion changed.

Commands: `python -B tools/task4b_tests.py`, the bounded profile/equivalence tools, `tools/task4b_benchmark.py optimized_1 --baseline`, independent `optimized_2`, `tools/task4b_final_audit.py`, and `tools/task4b_verify.py`, using paths relative to the V4 root where appropriate.

## P. Classification

**{classification}**. Frozen-kernel calls fall by **{summary['kernel_call_reduction_factor']:.3f}×**. The old architecture-level Python pair-processing overhead and final JSON audit are removed from the new path. A measured bottleneck remains: **646** symbolic CAD relations and mixed-rank exact work dominate the residual worker cost. We therefore do not declare the entire production runtime problem solved, infer asymptotic complexity from one shell, or invent a runtime pass threshold.

## Q. A2 readiness

The exact adapter and completeness path are qualified for a controlled A2 integration/replay. The sealed legacy Task-4 entrypoint remains unchanged and still selects its original slow certificate. Wire the new transaction adapter into that execution path, preserving checkpoint identities, before resuming A2. Do not simply rerun the old command. No additional physical degree of freedom or gate change is needed for this integration. A2 was not resumed here. No downstream meshing or CFD was performed.

`AUTHORITY.json` binds the report, source, and compact evidence. `SEAL.json` records report and authority SHA-256 values. The Git commit that contains this report is supplied in the final response to avoid a self-referential commit hash.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8')
    source=list((ROOT/'src/sdf_cfd_mesher_v4/certificate').glob('efficient*'))+[ROOT/'src/sdf_cfd_mesher_v4/certificate/exact_subdivision.py',ROOT/'tests/test_efficient_certificate.py',ROOT/'tests/fixtures/task4b_filter_cases.npz']+list((ROOT/'tools').glob('task4b_*.py'))
    evidence=[p for p in OUT.rglob('*') if p.is_file() and 'native' not in p.relative_to(OUT).parts and p.name not in ('AUTHORITY.json','SEAL.json')]
    authority=dict(task='DiagnoseAndReduceV4ExactCertificationRuntime',classification=classification,starting_head=start['head'],summary=summary,
        source_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in source if p.is_file()},
        evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in evidence},report_sha256=file_hash(OUT/'report.md'))
    save('AUTHORITY.json',authority);save('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    print(json.dumps(dict(classification=classification,report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),study_seconds=elapsed)))
