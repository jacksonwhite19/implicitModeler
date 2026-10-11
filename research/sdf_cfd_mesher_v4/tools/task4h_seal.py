"""Seal bounded exact-geometry evidence; never invoke a scientific runner."""
from task4h_common import *
from collections import Counter

def run():
    pre=read(OUT/'PREFLIGHT.json');outside=lambda s:[x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in x]
    assert outside(pre['status'])==outside(git('status','--porcelain'))
    for p,h in pre['V3'].items():assert file_hash(Path(p))==h,p
    for p,h in pre['sealed_evidence'].items():assert file_hash(ROOT/p)==h,p
    prior=read(ROOT/'results/task4e/AUTHORITY.json');verified=0
    for p,h in prior['evidence_sha256'].items():assert file_hash(ROOT/p)==h,p;verified+=1
    save('PRESERVATION.json',dict(V3_modified_files=0,V3_authority_files_verified=len(pre['V3']),prior_task4g_artifacts_verified=len(pre['sealed_evidence']),prior_task4e_artifacts_verified=verified,unrelated_entries=len(outside(pre['status'])),unrelated_status_identical=True))
    jac=read(OUT/'PRODUCTION_JACOBIANS.json');jbench=read(OUT/'JACOBIAN_EQUIVALENCE.json');old=read(OUT/'SPOT_BASELINE.json');new=read(OUT/'PRODUCTION_REPLAY.json');spots=read(OUT/'SPOT_REPLAY_1.json');second=read(OUT/'SPOT_REPLAY_2.json');final=read(OUT/'FINAL_REUSE.json');tests=read(OUT/'TEST_SUMMARY.json')
    assert len(jac['rows'])==20 and len(new['rows'])==30 and spots['scientific_hash']==second['scientific_hash']==new['scientific_hash']
    assert final['audits'][0]['scientific_hash']==final['audits'][1]['scientific_hash']
    counts=Counter()
    for r in new['rows']:counts.update(r['counts'])
    timing=dict(old_whole_jacobian_seconds=sum(r['old_seconds'] for r in jac['rows']),new_whole_jacobian_seconds=sum(r['seconds'] for r in jac['rows']),
        new_jacobian_kernel_seconds=sum(r['new_kernel_seconds'] for r in jbench['rows']),new_value_serialization_seconds=sum(r['serialization_compression_seconds'] for r in jbench['rows']),
        old_spot_selection_seconds=sum(r['selection_seconds'] for r in old['rows']),old_spot_pair_seconds=sum(r['pair_seconds'] for r in old['rows']),
        new_spot_seconds=sum(r['seconds'] for r in new['rows']),new_spot_selection_seconds=sum(r['selection_seconds'] for r in new['rows']),
        planner_roundtrip_seconds=sum(r['seconds'] for r in new['checkpoints']),final=final['audits'])
    timing['old_spot_seconds']=timing['old_spot_selection_seconds']+timing['old_spot_pair_seconds']
    g=read(ROOT/'results/task4g/FINAL_VALIDATION_REUSE.json');timing['old_receipt_establishment_numeric_sdf_seconds']=g['cold_numeric_sdf_seconds']
    save('RUNTIME_COMPARISON.json',timing)
    operation=dict(whole_cells=2126040,old_whole_exact_calls=2126040,new_whole_exact_calls=2126040,new_rational_materializations=3*2126040,
        spots=dict(counts),pair_comparisons=sum(r['pairs'] for r in old['rows']),old_spot_kernel_calls=old['session_misses'],spatial=spots['spatial'],
        final_numeric_calls=0,final_sdf_calls=0,final_new_exact_calls=0,final_receipt_layers_reused=20)
    save('OPERATION_COUNTS.json',operation)
    jd=scientific_hash([r['record_hash'] for r in jac['rows']]);jr=scientific_hash([r['replay_hash'] for r in jbench['rows']]);assert jd==jr
    det=dict(jacobian_hash=jd,jacobian_replay_hash=jr,spot_hash=spots['scientific_hash'],spot_replay_hash=second['scientific_hash'],production_restart_hash=new['scientific_hash'],final_hash=final['audits'][0]['scientific_hash'],final_replay_hash=final['audits'][1]['scientific_hash'],passed=True)
    save('DETERMINISM.json',det)
    peak=max(read(p).get('peak_rss',0) for p in OUT.glob('*.json') if isinstance(read(p),dict))
    save('PERFORMANCE.json',dict(maximum_recorded_process_peak_rss=peak,aggregate_concurrent_rss_not_sampled=True,scientific_stop='2026-09-17T20:47:24.662758+00:00',conservative_elapsed_at_scientific_stop_seconds=1644.662758,all_task4h_workers_stopped=True,full_build_performed=False,A2_run_performed=False))
    changed=git('diff','--name-only','--','research/sdf_cfd_mesher_v4').splitlines()
    files={str((REPO/p).relative_to(ROOT)).replace('\\','/') for p in changed if p}
    files.update(str(p.relative_to(ROOT)).replace('\\','/') for p in (ROOT/'tools').glob('task4h_*.py'))
    files.update(['src/sdf_cfd_mesher_v4/forecast/exact_dyadic.py','src/sdf_cfd_mesher_v4/certificate/future_proofs.py','src/sdf_cfd_mesher_v4/certificate/future_selection.py','src/sdf_cfd_mesher_v4/advancing_front/final_validation_reuse.py','tests/test_task4h_exact_geometry.py'])
    classification='V4ExactGeometryHotPathsQualified'
    oldprofile=read(OUT/'JACOBIAN_PROFILE.json');newprofile=read(OUT/'JACOBIAN_NEW_PROFILE.json')
    gcd=lambda p:sum(r['calls'] for r in p['call_tree'] if 'math.gcd' in r['name'])
    doc=f'''Decision question

Can V4 reduce its remaining exact-geometry cost without weakening exact Jacobian, continuous-intersection, or final-validation guarantees?

{classification}

## A. Authority verification

Started from `{pre['head']}` on `{pre['branch']}`. Task-4G report SHA-256: `2f011355e72f15e4cfd6a25357b9fc8793f197f8e21c690073ee40fb47dcc33f`. Task-4G authority SHA-256: `e2cb8b5d9fd9dfb6a1ad25c7d32359824cc13b890a218e96fa23064bca6e63ee`.

All referenced Task-4G source/evidence hashes passed at startup. The six qualified Task-4B certificate implementation hashes remain unchanged. See `CERTIFICATE_AUTHORITY_VERIFICATION.json`. End-of-task checks verified {verified} sealed Task-4E artifacts, {len(pre['sealed_evidence'])} Task-4G artifacts, and {len(pre['V3'])} V3 authority files. No unexplained drift was found.

## B. Current exact-geometry baseline

Task 4E remains the last measured full build: 20 layers, 2,126,040 prisms, 2,282.94 s scientific execution. This task did not rebuild geometry, rerun A2, or publish a prism layer. It read preserved fronts and qualified receipts in place.

The recorded whole-layer exact stage was {timing['old_whole_jacobian_seconds']:.6f} s. Task-4G future spot checks were 143.68 s; this task's same 30-comparison baseline measured {timing['old_spot_seconds']:.6f} s. The 105.60 s final-validation figure established receipts for old artifacts; it was not repeated work measured on new receipt-bearing production layers.

## C. Exact Jacobian profile

On 4,096 actual early-layer cells the frozen kernel took {oldprofile['unprofiled_seconds']:.6f} s without profiling. Its instrumented call tree contained {gcd(oldprofile):,} GCD calls, fraction arithmetic dispatch, rational object creation, and reduction. The new 4,096-cell profile contains {gcd(newprofile):,} GCD calls: three final rational materializations per cell. `JACOBIAN_PROFILE.json` contains self/cumulative time, calls and measured latency quantiles; `JACOBIAN_NEW_PROFILE.json` contains the new call tree. Profiling overhead is separate from benchmark time. Individual allocation time beyond these constructor calls was not isolated. There is no symbolic root solver in this kernel.

The 2,126,040 immediate cells are distinct required work. Final receipt reuse performs no new rational Jacobian calls. Local trial cache identity is unchanged and now calls the faster kernel on misses. This task did not rerun all local search trials.

## D. Exact Jacobian mathematical optimization

`forecast/exact_dyadic.py` eliminates common power-of-two denominators before polynomial arithmetic. Arbitrary-precision integer cross multiplication gives exact ordering and signs. Only minimum Jacobian, reference height and signed volume are reduced into canonical fractions. The frozen polynomial, stationary-point test, tie ordering, signed-volume integral and stored strings are unchanged. `SPECIFICATION.md` gives the derivation and consumer map.

The positivity-only function is qualified but is not used to omit values from consumers that store exact records. The production writer, local exact cache and controlling-support exact consumer use the full result. No new native compiler or library was needed: the measured waste was repeated Fraction construction/reduction, not lack of parallelism.

## E. Jacobian equivalence

All 2,126,040 preserved committed cells passed exact sign/value comparison through their canonical record hashes. All 20 compressed record files were reproduced byte for byte, including by the actual integrated production writer. Corrected layer 1 and nominal layer 16 also ran the frozen rational kernel one-for-one against the new kernel. Independent replay hashes match for all layers.

The 41 diagnosed nominal failures reproduce exactly: 26 zeros and 15 negative Jacobians. Healthy, subnormal-positive, exact-zero, negative, translated/scaled and deterministic deformed fixtures pass. The sign-only path matches the full rational oracle.

## F. Future spot-check decomposition

The preserved 30 accepted local comparisons request 1,074 unique pair IDs and 2,148 before/after pair evaluations. There are 1,491 unique complete geometric identities, 657 repeated identical inputs, and 417 new changed-geometry identities for already seen pair IDs.

Pair selection took {timing['old_spot_selection_seconds']:.6f} s; pair evaluation including preparation took {timing['old_spot_pair_seconds']:.6f} s. The old session had capacity zero, hence zero hits and 2,148 kernel calls despite repeated exact inputs. Exact session time was dominated by the preserved mixed rank-2 plane branch. There were no symbolic CAD calls in this production spot cohort. Root-key reuse was therefore zero here; no speedup is attributed to it.

The selection profile identifies repeated complete source-edge box scans as the largest cost. Four early comparisons spent 19.62 s in the source query method. Candidate selection, rather than exact algebra, explains most of the prior 143.68 s. All checks remain candidate-comparison evidence, plus preserved known warning pairs; no diagnostic checks were removed.

## G. Spot-check reuse and conservative filtering

The endpoint index preserves the old overlap predicate and source ordering. It avoided {spots['spatial']['full_target_tests_avoided']:,} source-box tests, with {spots['spatial']['indexed_target_tests']:,} remaining indexed target tests. The existing spatial update tested {spots['spatial']['incremental_target_tests']:,} changed-target relationships. The same 32,517,649 enumerated local pair entries and all 60 selection results remain covered. No pairing or probe-budget rule changed.

The exact-input proof cache supplied {counts['hits']} hits. Of {counts['misses']} misses, 116 passed corner-AABB proof, 1,373 passed certified supporting-plane proof, and 2 required the frozen mixed-rank kernel. Both sides retain validity checks. The known edge-1/layer-1 versus edge-729/layer-10 nominal forbidden intersection remains an identical exact forbidden result.

Bound proofs have explicit code, witness, source and geometry identities. Their storage representation differs from full kernel records; pair populations, classifications, warnings and decision evidence are identical. Old proof-storage hashes are not reused as if these representations were the same. Checkpoint/restart restores exact-input proof state. No ID-only or stale-geometry hit is allowed.

## H. Final-validation decomposition

The current Task-4G receipt-bearing production path already skips immutable numeric/SDF recalculation. Task-4H verified all 20 receipts in {final['audits'][0]['receipt_seconds']:.6f} s, including fresh hashes of actual input arrays. Loading all required preserved fronts/sidecars took {final['load_seconds']:.6f} s; the committed transaction-chain audit took {final['chain_seconds']:.6f} s. Final conditioning/SDF/hard-gate aggregation and comparison took {final['audits'][0]['global_aggregation_seconds']:.6f} s.

Required continuous completeness remains separate: full preserved pair-population audit took {final['audits'][0]['certificate_seconds']:.6f} s, with zero new pair solves. It verified 56,462,864 proofs, zero forbidden intersections and zero unresolved results. This task does not claim the complete final runner costs only the receipt-verification time; its separate prefix SDF check and final artifact serialization remain.

## I. Immutable final evidence reuse

The final runner now also requires `final_receipt_manifest`: all requested layers in order, all source entity ranges, schedule/target identity and adjacent front-input links. Individual geometry/algorithm/checkpoint/SDF/conditioning bindings remain enforced by the existing receipt implementation. The added final manifest is an audit of verified receipts, not a substitute for their verification or the final continuous proof audit.

All 2,126,040 cell receipts, 3,189,060 side ranges, 2,019,738 internal cap entities and 1,063,060 vertex-SDF entities are covered. Missing layer-16 evidence caused only that layer to be recalculated in {read(OUT/'FINAL_MISSING_RECEIPT.json')['seconds']:.6f} s; its result matched the sealed evidence exactly. No old artifacts were overwritten.

## J. Bounded scientific equivalence

The integrated production spot path reproduced all 30 preserved candidate comparisons, including early cap/trailing regions and layers 5–11. All pair sets, broad counts, statuses, warnings, first-forbidden layers and earlier-hazard decisions match. Per-layer planner save/load was exercised. Exact Jacobian production records match all 20 preserved layers. Final gates, SDF results, complete sigma/G arrays and qualification status match the completed A2 evidence. No candidate coordinates were generated or changed in this task.

## K. Runtime comparison

| Comparable operation | Old seconds | New seconds |
|---|---:|---:|
| Complete 20-layer exact-record writer on preserved fronts | {timing['old_whole_jacobian_seconds']:.6f} | {timing['new_whole_jacobian_seconds']:.6f} |
| All 30 future spot comparisons | {timing['old_spot_seconds']:.6f} | {timing['new_spot_seconds']:.6f} |
| Spot selection within those comparisons | {timing['old_spot_selection_seconds']:.6f} | {timing['new_spot_selection_seconds']:.6f} |
| One-time old numeric/SDF receipt establishment versus verified receipt reuse | {timing['old_receipt_establishment_numeric_sdf_seconds']:.6f} | {final['audits'][0]['receipt_seconds']:.6f} |

The writer includes per-cell assembly, exact arithmetic, canonical serialization, deterministic gzip and disk writes. It did not build geometry. Isolated new arithmetic was {timing['new_jacobian_kernel_seconds']:.6f} s. Separate materialization output serialization/compression was {timing['new_value_serialization_seconds']:.6f} s in the in-memory benchmark. Do not add nested measurements.

The integrated spot benchmark additionally measured {timing['planner_roundtrip_seconds']:.6f} s of planner checkpoint/restart work, reported outside the comparison stage. New selected-pair work was {timing['new_spot_seconds']-timing['new_spot_selection_seconds']:.6f} s including input/proof binding. Some bounded old/new profiles overlapped on separate processes; the production writer and spot loop times are direct observations, not a predicted full-build runtime. No new full-build time is claimed.

## L. Remaining dominant cost

Among these measured production-path replays, the exact-record writer ({timing['new_whole_jacobian_seconds']:.2f} s) and spot selection ({timing['new_spot_selection_seconds']:.2f} s) now dominate. Future selection still constructs complete local candidate populations to retain existing semantics. The last full-build immediate certificate cost remains 331.95 s; that subsystem was not rerun or redesigned here. Its relative position in a new complete build is unmeasured.

## M. Memory

Maximum recorded individual-process RSS was {peak:,} bytes ({peak/2**30:.3f} GiB). Concurrent aggregate RSS was not sampled. The bounded proof cache held 1,491 entries; capacity is 4,096. Endpoint indexes scale with source edges and do not create coupled optimization or sensitivities. Temporary exact writer output and the roundtrip planner snapshot were removed after hash/identity checks. Preserved Build A/A2 shells were not copied.

## N. Determinism

Jacobian scientific/replay hash: `{jd}`.

Future spot selection/outcome/proof hash, both independent replays and integrated checkpoint replay: `{spots['scientific_hash']}`.

Final receipt/qualification replay hash: `{final['audits'][0]['scientific_hash']}`.

Times, RSS and process information are excluded from scientific hashes. Candidate selection, exact records, cache reuse and final receipt decisions reproduce deterministically.

## O. Tests

The focused Task-4H suite passed 30/30. The single selected regression run passed {tests['tests']-len(tests['failures'])-len(tests['errors'])}/{tests['tests']}, with zero errors. Two inherited historical repository-state assertions failed: `test_current_task_preservation_and_sealed_task2` and `test_followup_preserves_unrelated_state`. Both compare this checkout to older unrelated Git-state snapshots. Their contracts were retained. Current-task unrelated state is unchanged. See `TESTS.txt`, `FOCUSED_TESTS.txt`, and `TEST_SUMMARY.json`.

## P. Classification

{classification}

Exact integer arithmetic and future proof/index improvements are integrated. Existing immutable receipt reuse is verified and its final completeness audit is strengthened. All required scientific equivalence checks passed. No gate, SDF authority, topology, schedule, risk criterion or certificate mathematics changed.

Scientific work stopped at 2026-09-17 20:47:24.662758 UTC, at most 1,644.663 s after the conservative 20:20:00 UTC task-start bound. The first retained clock sample was 20:22:04 UTC. This conservative bound is used for time-limit compliance rather than claiming an unavailable exact start timestamp. No scientific worker remained running. Sealing time is recorded separately in `SEAL.json`. The 55-minute scientific and 60-minute task limits were respected.

## Q. Full-build readiness

One new controlled, time-bounded full 20-layer qualification run is now justified. It must measure end-to-end performance and compare geometry, canonical decisions, gates, pair outcomes and conditioning. Proof-storage representation changes must remain separate from scientific equality. The last full-build runtime remains Task 4E's 2,282.94 s. Do not infer a new full-build runtime by summing these replays.

No full build, A2 rerun, downstream mesh, CFD or layer termination was started. V3 modifications from this task are zero. All {len(outside(pre['status']))} unrelated Git-state entries were preserved. The Task-4H commit contains only the source paths below and `results/task4h` evidence; its commit ID is reported after sealing.

Changed/added source paths:

'''+''.join(f'- `{p}`\n' for p in sorted(files))
    (OUT/'report.md').write_text(doc,encoding='utf-8',newline='\n')
    sources={p:file_hash(ROOT/p) for p in sorted(files)}
    evidence={str(p.relative_to(ROOT)).replace('\\','/'):file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')}
    authority=dict(task='OptimizeV4ExactGeometryHotPaths',classification=classification,starting_head=pre['head'],report_sha256=file_hash(OUT/'report.md'),source_sha256=sources,evidence_sha256=evidence,summary=dict(runtime=timing,operations=operation,determinism=det,tests=dict(total=tests['tests'],passed=tests['tests']-len(tests['failures']),inherited_state_failures=2,new_focused_passed=30),peak_process_rss=peak,V3_modifications=0,full_build_performed=False,A2_performed=False,full_build_readiness=True))
    save('AUTHORITY.json',authority)
    save('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),sealing_completed=datetime.datetime.now(datetime.timezone.utc).isoformat(),conservative_task_elapsed_seconds=elapsed(),persistent_result_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())))
    print(json.dumps(read(OUT/'SEAL.json'),indent=2))
if __name__=='__main__':run()
