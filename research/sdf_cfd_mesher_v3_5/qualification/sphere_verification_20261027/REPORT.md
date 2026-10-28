# Sphere audit / production / offline qualification

PASS for one fixed sphere on the configured host, using runtime commit `714b83dd190b3c51dce2b24b837f2500817f368a` and identity `8e428c2a5e7bee83dc71c70660c40cbd1452f08ad99674ed5e4755456768c518` (1,873 protected files). One attempt per stage, no retries, implementation changes or broad test rerun. V3 complete prisms feed the existing V4 direct transition/background. The frozen package is unchanged.

| Qualification | Result | Producer / separate semantic checks | CLI wall seconds | Peak Windows tree MiB | Result logical MiB |
|---|---|---|---:|---:|---:|
| Fresh normal audit | audit_accepted | 12 / 12 | 524.486 | 985.56 | 299.87 |
| Fresh normal production | production_accepted; audit not_run | 12 / 0 | 436.574 | 811.02 | 274.44 |
| Separate post-production audit | audit_passed; no fresh source authority | retained producer evidence / 12 | 87.811 | 449.55 | 26.11 |

The dependent runs started only after their required predecessor acceptance and exact comparison passed. Source/resource closure passed in all stages. Normal audit and production required actual native conversion, exact correspondence and literal Mesh OK. Production emits its distinct composed profile/results/manifest without legacy passed/full_mesh_complete aliases. Its audit remains not_run after the separately linked offline audit; the source bundle was not rewritten.

## Numerical identity and behavior

All five COMPLETE arrays match frozen sphere and each other in dtype, shape and bytes: points(14798,3), prisms(12780,6), tetrahedra(40448,4), wall(2556,3), farfield(3672,3). Total53,228cells/14,798points. All eight saved plan arrays match exactly. Achieved/requested schedules and native prism geometry/ancestry, complete shape populations and neighbor QA match the frozen case. All56 recorded surface query counters, seed counts and saved surface hashes also match.

Settings remain explicit adaptive,20micrometre first height,five rows,growth1.3,200micrometre nominal surface size and original direct/background settings. Operational bounds remain43,200s/16GiB/16,000,000,000bytes for each normal run; offline deadline10,800s. Total prism thickness180.862micrometres, zero compression and zero scalar/path growth reversals. Mandatory shape minima are bulk0.4836897980570004,interface0.6060645893270594,cap0.7505408777371186; required floors0.25/0.25/0.30. All sphere population targets are met. CheckMesh warnings:none; -meshQuality remains not_run without an approved dictionary.

Each normal run records one background generation and one construction invocation, two direct native workers, seven owned WSL operation-scope receipts (two prism construction, two direct workers, three export operations), zero conditioning-native proposals/commits and zero changed apex vertices. These scope counts do not count every in-process/native predicate. Offline emitted no new owned native scopes and performed no remeshing/search; it replayed retained collision classifications with fresh population/coverage checks, rather than rerunning the native classifier or conversion/checkMesh.

The offline audit used trusted VERIFICATION SHA256 `dd9c6d85c60288d3ffbcf1b69843d9766255b42f9272f446878185a029298c02` and the exact supported verifier identity. All374 production files retained identical hashes/sizes, with no additions or removals. Twelve actual offline replay receipts passed. No live lease or fresh SDF authority was created.

## Measured costs

These root stage times do not overlap within a run. The direct workflow row includes its nested producer/export/replay costs; do not add nested timings to it or to the external CLI wall total.

| Root stage seconds | Audit | Production |
|---|---:|---:|
| Surface | 40.566 | 40.063 |
| First front | 11.112 | 11.316 |
| V3 complete plan | 8.013 | 8.022 |
| Prospective shell | 23.853 | 24.081 |
| Native prism construction | 3.714 | 3.667 |
| Shell admission | 4.191 | 4.203 |
| Direct workflow including verification | 428.745 | 340.438 |

Nested joined_proofs stages took92.553s audit/92.670s production, including binding work; the audit cold stage took87.198s. Production's finalization tail took0.231s, but the complete finalizer is not separately instrumented: binding/serialization/sealing costs are also inside joined_proofs/export. No whole-finalizer cost is inferred from that tail. The offline joined proof and overall audit costs are saved in SPHERE_VERIFY1_OFFLINE_COMPARISON.json.

Standalone production was 87.911s (16.76%) lower in observed CLI wall time. Production plus optional offline audit totaled 524.385s, essentially the same as fresh audit 524.486s. This single sequential pair does not establish statistically repeatable or causally isolated speedup. Totals exclude external comparison/reporting and idle gaps between runs. RSS samples cover the launched Windows process tree (normal runs include supervisor), not aggregate WSL Linux RSS; Linux16GiB is a per-process address-space bound. Result sizes are logical bytes inside each result directory, excluding external run logs; physical allocation/sharing is unmeasured.

## Scope and preservation

This closes the previously unqualified complete sphere audit/production/offline path at the stated settings. It does not qualify production for passage, blend, arbitrary shapes or other carriers/settings, CFD/solver/y-plus accuracy, continuous geometry, clean-machine portability or a statistical performance result. Numerical predicates/thresholds and construction settings are unchanged. No larger fixture was run; further expansion requires a new decision.

Independent closed-result reviews inspect actual arrays, manifests, all producer/replay receipts, native export, source closure, retained input inventory and sequential gates. Preservation confirms1,866 frozen runtime files in both original/published worktrees and78 unrelated hashes unchanged; frozen published worktree remains clean at eff955ed. Candidate runtime remains714b83dd/8e428c2a; only qualification/status documentation is updated after closure. No push or frozen-package replacement.

Raw runs are standalone/results/sphere_verify1_{audit,production,offline}_01. Exact commands, wall/RSS/size records, all comparisons, per-stage/predicate/native costs, source counters, complete offline input inventory, preservation and review pins are in outputs/SPHERE_VERIFY1_*.json. All raw evidence is retained. Report callback must be delivered before final response, then stop.
