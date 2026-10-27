"""Seal a bounded, preserved-input Task-4G result. No scientific execution."""
from task4g_common import *

def run():
    r=read(OUT/'RUNTIME.json');o=read(OUT/'OPERATION_COUNTS.json');d=read(OUT/'DETERMINISM.json');v=read(OUT/'FINAL_VALIDATION_REUSE.json');tests=read(OUT/'TEST_SUMMARY.json');pre=read(OUT/'PREFLIGHT.json')
    assert read(OUT/'REPLAY_PROGRESS.json')['trials']==255 and d['passed']
    assert len(tests['failures'])==3 and not tests['errors']
    v3={p:file_hash(Path(p)) for p in pre['V3']};assert v3==pre['V3']
    for p,h in pre['sealed_evidence'].items():assert file_hash(ROOT/p)==h,p
    for p,h in pre['authorities'].items():assert file_hash(ROOT/p)==h,p
    status=git('status','--porcelain')
    unrelated=lambda s:sorted(x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in x)
    assert unrelated(status)==unrelated(pre['status']),'UnrelatedGitStateChanged'
    save('PRESERVATION.json',dict(V3_files_verified=len(v3),V3_modifications=0,sealed_Task4F_unchanged=True,
        unrelated_git_entries=len(unrelated(status)),unrelated_state_equal=True,branch=git('branch','--show-current'),base_head=pre['head'],status=status))
    classification='V4DependencyLocalEvaluationSubstantiallyImproved'
    source_paths=sorted(set(git('diff','--name-only','--','research/sdf_cfd_mesher_v4').splitlines()+git('ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4').splitlines()))
    source_paths=[p for p in source_paths if '/results/' not in p and p.endswith(('.py','.md'))]
    evidence_size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    peak=max(read(OUT/'REPLAY_PROGRESS.json')['peak_rss'],v['peak_rss'],read(OUT/'STAGED_RUNTIME.json')['peak_rss'],read(OUT/'BOUNDED_EQUIVALENCE.json')['peak_rss'])
    summary=dict(classification=classification,scientific_equivalence=dict(trials=255,accepted=32,rejected=223,accepted_coordinates_equal=True,all_evaluated_metrics_bitwise_equal=True,exact_future_proofs_equal=True),
        runtime=r,operations=o,peak_process_rss_bytes=peak,evidence_bytes_before_report=evidence_size,
        new_tests=31,tests_run=tests['tests'],tests_passed=tests['tests']-len(tests['failures']),inherited_failures=3,
        deterministic_hash=d['hash_1'],validation_hash=v['replay_hashes'][0],V3_modifications=0,source_paths=source_paths,
        scope='No new shell build, A2 run, production geometry commit, or scientific-policy change.',
        remaining_work='Changed operators still rebuild their five raw rows. Local input gathering is patch-wide. Entity cache state is process-local; whole-layer exact checks and exact future probes are unchanged.',
        recommended_next='Profile and qualify a bounded faster exact-Jacobian arithmetic path against existing rational receipts; do not change exact signs or hard gates.')
    save('SUMMARY.json',summary)
    vr=read(OUT/'VALIDATION_REPLAY_1.json');vr2=read(OUT/'VALIDATION_REPLAY_2.json');b=read(OUT/'BOUNDED_EQUIVALENCE.json')['rows']
    layer_table='\n'.join(f"| {x['layer']} | {x['trials']} | {x['accepted']} | {x['early_rejected']} | {x['new_detail_seconds']:.3f} |" for x in read(OUT/'LAYER_REPLAY.json'))
    changed='\n'.join('- `'+p.removeprefix('research/sdf_cfd_mesher_v4/')+'`' for p in source_paths)
    text_report=f'''Decision question

Can V4 stop recomputing unchanged future geometry and stop evaluating expensive future metrics after a trial already fails an existing hard gate, while preserving exact scientific behavior?

Primary classification: **{classification}**.

## A. Authority verification

Starting HEAD: `{pre['head']}`. Branch: `{pre['branch']}`.
Task-4F report: `a83c57c584695acb5ad26ac88efd89bf4093e21c4360f5b3a4a7caf361b87dcf`.
Task-4F authority: `bdfdd32f5f2d8340c63b2426f241dbf2f066352874aa5ceb5da6bb91baf8760a`.
Both matched. All Task-4F sealed evidence matched. The Task-4E authority
`12d09114a1cbe7c4c06cadb7d42488683a7907f35f0c7e2546a0962fab4455ae`
and its 710 evidence files matched. All {len(v3)} bounded V3 source hashes stayed
unchanged. No repository-wide hash audit was used. The legacy raw-shell hash
representation difference is outside this task.

## B. Task-4F measured baseline

The preserved full run used 784.49 s for active search, including 480.00 s for
detailed future metrics. Whole-layer exact signs used 357.40 s. Final validation
used 144.68 s; 100.61 s repeated numeric/SDF work on immutable geometry.
The audited trial population requested 11,272,580 cell evaluations, with
3,759,892 changed inputs relative to each nominal trial baseline. The remaining
7,512,688 inputs were identical. The existing complete-patch cache was correct
but too coarse. No risk or quality threshold was introduced here.

## C. Dependency model

`geometry_dependencies` propagates exact changed vertex bits through actual
edge and triangle incidence to prism and continuous-side inputs. Cell outputs
depend on six ordered corners and global layer masks. Face outputs also depend
on both adjacent centres and volumes. The full specification is in
`DEPENDENCIES.md`. No arbitrary margin or new support ring was added.

## D. Cache identities and invalidation

`LocalMetricEngine` retains four bounded supports. Topology keys bind source IDs,
ownership, incidence, support and global topology identity. Numeric reuse uses
exact coordinate-input comparisons. Face reuse compares all face vertices and
adjacent centre/volume inputs. Signed-zero bits are retained. Returned outputs
cannot mutate later cache states. Persistent planner fingerprints include the
new cache implementation. Eviction changes runtime only.

## E. Topology-closure reuse

Validated closure, ownership maps, local index maps and stable face IDs persist
while their exact structure stays unchanged. The old closure checks remain;
their repeated column-set construction was hoisted. Missing neighbors, changed
topology and missing vertices still fail. No reconstruction stencil is truncated.

## F. Primitive metric reuse

Across all 255 preserved trials, {o['new_trial_counts']['cell_recomputed']:,} cell
components were recomputed and {o['new_trial_counts']['cell_reused']:,} reused.
Early rejection eliminated another {o['early_avoided_whole_patch_cells']:,}
whole-patch cell requests. Together these account for all 11,272,580 audited
trial requests. Baseline refreshes separately recomputed
{o['new_baseline_counts']['cell_recomputed']:,} cells and reused
{o['new_baseline_counts']['cell_reused']:,}.

The new dirty-input count is relative to the retained cache state, not to the
old nominal baseline. These two definitions must not be confused.
Face metrics recomputed {o['new_trial_counts']['face_recomputed']:,} inputs,
reused {o['new_trial_counts']['face_reused']:,}, and avoided
{o['early_avoided_faces']:,} requests through rejection.

## G. Reconstruction and conditioning reuse

The trial path reused {o['new_trial_counts']['reconstruction_rows_reused']:,}
operator rows and recomputed {o['new_trial_counts']['reconstruction_rows_recomputed']:,}.
Rows are counted as the five-row cell-operator dependency component. Normalized
weights and the production inverse couple those rows. The implementation still
rebuilds all raw face rows inside a changed cell; it does not claim minimum
possible raw-row work. Unchanged sigma and G_bound outputs are reused exactly.

The specialized gain path evaluates the unchanged expression
`svd(B @ Pb)[0]` and retains the original singular-cell fallback. It avoids
unconsumed gain SVDs. Sigma remains `sigma_min(sqrt(W) N)`. No conditioning gate
was added. Unavailable gain diagnostics serialize canonically; they cannot
create a new hard rejection.

## H. Exact Jacobian reuse

Local rational checks fell from 295,747 requests to {o['exact_local_calls']:,}
kernel calls, with {o['exact_local_hits']:,} exact-input hits. Every recomputed
and reused record matched the preserved exact result. Cache keys contain all
18 binary64 coordinates. No tolerance or floating-point acceptance replaced
the rational sign proof. The 2,126,040 distinct whole-layer exact checks remain
unchanged; the historical 357.40 s stage was not rerun or claimed eliminated.

## I. Staged rejection pipeline

Production runners now perform SDF projection, the existing analytical
wedge/determinant screen, all remaining immediate numeric gates without
conditioning, and exact rational signs before future metrics. Future comparison
and exact spot checks retain their original policy. The complete immediate
certificate still precedes any production commit.

All 255 staged inputs reproduced the preserved hard values. Exactly 76 of 223
rejections ({100*76/223:.2f}%) stopped before future forecasting and conditioning.
Those same trials historically consumed {r['early_avoided_historical_detailed_seconds']:.6f} s
of detailed work and {r['early_avoided_historical_forecast_seconds']:.6f} s of local
forecasting. These are measured eliminated workloads, not a predicted full-build
speedup. Rejected records omit unevaluated diagnostics and name the earlier
mandatory gate. The accepted/rejected set is unchanged.

## J. Immutable final-validation evidence

Production commits now write input-bound validation receipts. Deterministic ID
ranges cover every cell, side, internal cap and vertex. Receipts bind prior-layer
interface geometry, current coordinates, topology, SDF, schedule, implementation,
conditioning hashes, exact signs and the committed certificate.

On preserved completed A2, fresh numeric/SDF evaluation took
{v['cold_numeric_sdf_seconds']:.6f} s and matched all 20 stored layer records.
Two receipt audits reused all 20 layers in {vr['seconds']:.6f} s and
{vr2['seconds']:.6f} s. These audit timers include input hashing and receipt
verification; initial geometry/conditioning loading is outside those timers.
The real missing-receipt test recalculated only layer 16 and matched exactly.
Missing conditioning/SDF sidecars also use the production fallback.

This does not reduce the final proof to trusting old flags. The existing full
topology/transaction audit and exact pair-completeness re-enumeration remain in
`task4_final_validation.py`. Their cost was not removed from a claimed total.

## K. Bounded equivalence

The cohort includes accepted early corrections, immediate-gate rejections,
future-regression rejections, persistent layers 5–11, passive layer 16 and final
20-layer validation. A real 62,180-prism scope matched bitwise for cold input,
a changed vertex, rollback and identical input. The isolated changed-vertex
comparison took {b[1]['old_seconds']:.6f} s for full recomputation and
{b[1]['new_seconds']:.6f} s incrementally. This is supplementary evidence, not
the basis for declaring the whole task successful.

## L. Full preserved 255-trial replay

All 255 proposals matched their preserved coordinate hashes. All surviving
forecast inputs matched Task-4F keys. All evaluated metric arrays matched bit
for bit. All 32 accepted corrections and 223 rejections matched. All 11 corrected
front hashes matched. Selected exact future proof records matched as well.

Rejected coordinate arrays were not fully archived. Only the bounded saved
proposal/forecast inputs were reconstructed from recorded modes and factors,
then checked against preserved hashes. No new search choices, shell generation,
or A2 build occurred. Later checkpoints were regression inputs only.

| Layer | Trials | Accepted | Early reject | Incremental trial metrics (s) |
|---|---:|---:|---:|---:|
{layer_table}

## M. Operation-count reduction

`OPERATION_COUNTS.json` separates trial work, baseline work and requests removed
by staging. Primitive reuse: {o['new_trial_counts']['cell_reused']:,} cells.
Face reuse: {o['new_trial_counts']['face_reused']:,} faces. Conditioning reuse:
{o['new_trial_counts']['conditioning_reused']:,} operators. The exact rational
cache reused {o['exact_local_hits']:,} results. No new certificate rejection rule
or reduced certificate population was used. The production whole-patch memo can
avoid additional calls; the isolated engine accounting does not claim those hits.

## N. Runtime reduction

| Measurement | Previous/reference (s) | New measured work (s) |
|---|---:|---:|
| Full-run active search | 784.49 | Not rerun |
| Planning kernels, preserved replay | No equivalent aggregate baseline | {r['measured_planning_kernel_subtotal_seconds']:.3f} |
| Detailed metrics including baselines | 480.00 historical | {r['detail_replay_seconds_including_baselines']:.3f} |
| Paired surviving-trial full versus incremental metrics | {r['paired_survivor_full_recomputation_seconds']:.3f} | {r['incremental_trial_metric_seconds']:.3f} |
| Recomputed numeric/SDF final evidence | {v['cold_numeric_sdf_seconds']:.3f} | {vr['seconds']:.3f} receipt verification |

The {r['measured_planning_kernel_subtotal_seconds']:.3f} s subtotal combines directly
measured proposal, screen, forecast, local exact, metric and future-spot kernels.
It is not a replacement end-to-end active-search time. It excludes support setup,
some control/serialization work and qualification instrumentation. The full
qualification replay took {read(OUT/'REPLAY_PROGRESS.json')['seconds']:.3f} s,
including the old oracle and second independent cache. Some early-layer timings
overlapped other bounded qualification processes. No new full-build time is
claimed or extrapolated.

## O. Remaining dominant cost

Selected future exact spot checks dominated measured replay kernels at
{r['future_exact_spots_seconds']:.3f} s. Their scientific scope was unchanged.
Among untouched full-run stages, whole-layer exact rational signs (357.40 s)
and immediate certification (331.95 s) remain large. Changed operators still
rebuild five raw rows, input gathering still touches each local patch, and the
entity cache is process-local. These limits support the qualified classification
of substantial improvement, rather than a claim that repeated work is exhausted.

## P. Memory and storage

Maximum recorded process RSS: {peak:,} bytes ({peak/2**30:.3f} GiB).
This is a process maximum, not an invented aggregate peak across concurrent
qualification processes. Evidence size before report/seal: {evidence_size:,}
bytes. Final file sizes are recorded in the authority manifest. No Build-A/A2
geometry or large V3 artifact was duplicated. No planner snapshots were archived.

## Q. Determinism

Scientific replay hashes:

- `{d['hash_1']}`
- `{d['hash_2']}`

Both matched. Every scientific array bit is bound; timing and RSS are excluded.
Independent metric caches reproduced dependencies and outputs. The staged replay
independently checked proposal and exact-sign identities. Immutable coverage
hashes both equal `{v['replay_hashes'][0]}`.

## R. Tests

Focused regression run: {tests['tests']} tests; {tests['tests']-3} passed; three
inherited preservation assertions failed; zero errors. All 31 new tests passed.
Two old assertions compare the original fork manifest against the already
changed root `src/sdf/aerospace/naca_spline.rs`. One compares today's unrelated
Git state with an old Task-3 snapshot. Their contracts were not weakened.
The current bounded V3 preservation check passed with zero Task-4G modifications.

The tests cover dependency masks, face/primitive equivalence, closure invalidation,
cache rollback, exact input identity, staged rejection, absent evidence fallback,
complete manifest coverage, nonfinite gain serialization and deterministic replay.
The early implementation's receipt format could not encode unavailable gain
diagnostics; the minimal canonical-serialization fix and regression preserve the
absence of a hard gain gate. The complete development suite was not repeatedly run.

## S. Classification

**{classification}**.
Production dependency-local metrics, staged rejection and immutable-evidence
reuse are integrated. All preserved decisions match. Meaningful raw-row and
input-gathering work remains. No geometry was committed during this study.

Scientific work stopped at {read(OUT/'SCIENTIFIC_STOP.json')['utc']}.
Elapsed task time at scientific stop: {read(OUT/'SCIENTIFIC_STOP.json')['elapsed_seconds']:.3f} s.
Elapsed at this report seal: {elapsed():.3f} s. The 55-minute scientific cutoff
and 60-minute total cap were respected. This was not a timeout.

## T. Recommended next step

Qualify a bounded faster exact-Jacobian arithmetic implementation against the
existing rational receipts. Preserve exact signs and all gates. Also profile
the unchanged future spot-check stage before proposing further work. Do not
infer a full-build runtime from this replay and do not start another A2 here.

Changed source/test/tool paths:

{changed}

Only these V4 changes and `results/task4g` are eligible for the narrow commit.
The final commit hash is reported after sealing. All {len(unrelated(status))}
unrelated Git entries remain unchanged.
'''
    (OUT/'report.md').write_bytes(text_report.encode())
    sources={p.removeprefix('research/sdf_cfd_mesher_v4/'):file_hash(REPO/p) for p in source_paths}
    evidence={str(p.relative_to(ROOT)).replace('\\','/'):file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','report.md')}
    authority=dict(classification=classification,start_head=pre['head'],branch=pre['branch'],source_sha256=sources,evidence_sha256=evidence,
        report_sha256=file_hash(OUT/'report.md'),scientific_hash=d['hash_1'],validation_hash=v['replay_hashes'][0],
        V3_modifications=0,unrelated_git_entries=len(unrelated(status)),task_elapsed_at_seal=elapsed(),
        persistent_evidence_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),
        no_full_build=True,no_A2_run=True,no_scientific_policy_change=True)
    save('AUTHORITY.json',authority)
    print(json.dumps(dict(classification=classification,report_sha256=authority['report_sha256'],authority_sha256=file_hash(OUT/'AUTHORITY.json'),elapsed=elapsed()),indent=2))
if __name__=='__main__':run()
