# Thick-blend audit / production / offline qualification

PASS for one fixed thick blend on the configured host. Execution commit `3220a6071fd48fadbc79ec3b167a9f8fd97109db` contains unchanged runtime code from `714b83dd190b3c51dce2b24b837f2500817f368a`, identity `8e428c2a5e7bee83dc71c70660c40cbd1452f08ad99674ed5e4755456768c518` (1,873 protected files); intervening changes are documentation only. One attempt per stage; no runtime changes, scientific retries, tuning or broad test rerun. V3 complete-stack prisms feed the existing V4 direct transition/background and joined acceptance/export. There is no separate transition band. The frozen package is unchanged.

| Qualification | Acceptance | Producer / separate semantic checks | CLI wall seconds | Peak Windows tree MiB | Result logical MiB |
|---|---|---|---:|---:|---:|
| Fresh audit | audit_accepted | 12 / 12 | 2850.049 | 2211.97 | 875.44 |
| Fresh production | production_accepted; audit not_run | 12 / 0 | 2637.364 | 1968.93 | 797.92 |
| Separate offline audit | audit_passed; no fresh source authority | retained producer evidence / 12 | 229.903 | 872.19 | 78.20 |

Each dependent run started only after its predecessor closed successfully and passed its exact physical comparison. Normal runs passed twelve producer checks, actual native export correspondence and literal Mesh OK, source closure and resource supervision. Production keeps its distinct profile and audit not_run. The linked offline audit remains a separate result.

## Numerical identity and repairs

All five COMPLETE arrays match the frozen blend and each other in dtype, shape and bytes: points(38868,3), prisms(46570,6), tetrahedra(72871,4), wall(9314,3), farfield(3672,3). There are 119,441 cells and 38,868 points. All eight prism-plan arrays, the native SEALED_PLAN prefix, complete shape populations, native prism behavior, achieved schedules and neighbor QA match. All 56 saved surface query counters and saved seed/surface hashes match. These are retained-record comparisons, with no extra geometry queries.

Conditioning follows the same committed route: initial 72,849 core cells; existing-vertex 3-to-2 edit at history index 30 leaves 72,848; optimized depth 3 seed cavity native_07 removes 31/adds 54 and leaves 72,871. The existing-vertex stage's FiniteConnectivityFamilyRefusal after its one commit is the expected internal route into constrained conditioning, not a failed full run or retry. Exactly four points are inserted at IDs 38864..38867, with original coordinates unchanged. Full committed records, nine edited-state NPZ operands/candidates and canonical ordered cell/face metadata match frozen and paired audit. The retained trial history has 57 existing-vertex proposals, one existing-vertex commit, one cavity commit and eight constrained native calls. No apex vertex moves; one repair invocation. This does not claim separate examination of every uncommitted intermediate native snapshot.

Each normal run retains fifteen owned WSL operation scopes: two prism operations, two direct native workers, eight constrained cavity operations and three export operations. These are ownership scopes, not every in-process native predicate. Offline creates no native scopes and performs no remeshing or construction search. It replays committed edits, ownership and shape/coverage. Global collision evidence reuses retained classifications with fresh coverage checks. Each committed edit also replays local cavity admission and native changed-versus-all collision certification in process. No new native subprocess scope is created; zero owned scopes does not mean zero native predicate work. Native mesh conversion and checkMesh are not rerun.

Controls remain explicit adaptive, units 0.001, 20 micrometre first height, five rows, nominal growth 1.3, 200 micrometre nominal surface size and original direct policy/background settings. Normal bounds remain 43,200 s / 16 GiB / 16,000,000,000 bytes; offline deadline 10,800 s. There are 1,663 compressed vertices; total prism thickness 58.5731 to 180.862 micrometres, 2,429 scalar growth reversals and 2,559 physical-path reversals. These retained schedules are identical to the accepted baseline; nominal growth is not a claim of uniform achieved growth.

| Shape population | Count | Minimum | Mandatory floor | Distribution targets |
|---|---:|---:|---:|---|
| all_bulk_tetrahedra | 72871 | 0.2673284952388835 | 0.25 | TARGETS_MET |
| interface_tetrahedra | 9314 | 0.2969626531058274 | 0.25 | TARGET_NOT_MET |
| preserved_cap_triangles | 9314 | 0.3034890879831677 | 0.3 | TARGET_NOT_MET |

All mandatory minima pass. Cap/interface percentage targets are nonblocking misses and remain visible. Native checkMesh retains 41 faces above 70 degrees nonorthogonality, maximum 77.266 degrees. These are unchanged allowed diagnostics. The optional -meshQuality check remains not_run without an approved dictionary.

Offline used trusted production VERIFICATION SHA256 `c5d3a1ef0cc24afe0221bd27f0563e388f2a02512c6095bd9164ff4beb4a0632` and the exact supported verifier runtime. All 548 production files retained identical hashes and sizes, with no additions/removals. All twelve semantic replay receipts passed. No live lease or fresh source authority was created; production audit remains not_run.

## Measured costs

Root stage times do not overlap within each run. Direct workflow includes nested producer, export and replay work; do not add its nested timings to the root or CLI totals.

| Root stage seconds | Audit | Production |
|---|---:|---:|
| Surface | 710.305 | 716.753 |
| First front | 48.595 | 48.849 |
| V3 complete plan | 365.539 | 366.609 |
| Prospective shell | 121.948 | 121.791 |
| Native prism construction | 5.983 | 5.987 |
| Shell admission | 16.587 | 16.684 |
| Direct workflow including verification | 1571.902 | 1351.025 |

The audit's nested independent readback took 226.356 s. Offline joined replay took 174.574 s, with actual offline audit time 229.328 s. Complete finalizer cost is not separately instrumented: binding/serialization/sealing also occur inside joined proofs/export. The production finalization tail is not a measurement of the whole finalizer.

Production CLI wall time was 212.685 s (7.46%) lower than audit in this single sequential pair. Production plus offline totaled 2867.267 s versus fresh audit 2850.049 s. These are observations, not a repeatable or causally isolated speedup. Totals exclude comparison/reporting and idle gaps. Sampled RSS covers the launched Windows process tree, including normal supervisor; it excludes aggregate WSL Linux RSS. The 16 GiB Linux bound is per-process virtual address space. Output sizes are logical bytes inside result directories, excluding external logs; physical allocation/sharing is unmeasured.

## Scope and preservation

This closes the fixed thick-blend sequence at the stated settings, alongside the prior fixed sphere. Passage production/offline, arbitrary shapes/settings/carriers, clean-machine portability, continuous geometry and CFD/solver/y-plus accuracy remain unqualified. No passage/new-geometry run, frozen push/package replacement or automation was performed. No numerical threshold was changed.

Independent closed-result reviews examine actual manifests/receipts, physical and planning arrays, committed conditioning edits, inserted coordinates/ownership, offline witnesses and sequential gates. Preservation and retirement receipts cover frozen runtime files, unrelated original work, exact process identities and owned operation groups. Raw evidence is retained under standalone/results/blend_verify1_{audit,production,offline}_01. Exact commands, source/runtime closure, detailed stage/predicate/native costs, input inventory and review pins are in outputs/BLEND_VERIFY1_*.json.

Advisor dispatch3/5 is complete after callback delivery; callbacks do not increment the counter. Stop after this milestone.
