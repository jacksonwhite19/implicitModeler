# Mesher V3.5 consolidation — Closed three-case consolidation

The accepted runtime is frozen. Passage and sphere are complete on exactly the same 1,866-file scientific/runtime build. Original thick-blend same-build verification is complete. No scientific/runtime source, algorithm, verification mode or acceptance gate changed. Git publication is separately recorded in the milestone delivery receipt; unrelated dirty work is outside the scoped freeze.

| Fixture | Explicit carrier mode | Current frozen-build status | Cells / points |
|---|---|---|---:|
| Passage | nodal | Complete engineering pass, NODAL5 | 546,464 / 207,705 |
| Sphere | adaptive | Complete pass; baseline-identical arrays | 53,228 / 14,798 |
| Original thick blend | adaptive | Complete pass; baseline-identical arrays | 119,441 / 38,868 |

Sphere's five final mesh-array entries and quality minima are byte-identical to its accepted baseline. Fresh worker time 495.110 s versus baseline 494.025 s; this is unchanged engineering output, not a controlled performance result. Sphere achieves all five nominal rows, total180.862 um, no compression or growth reversal; conditioning and apex movement are no-ops. checkMesh reports Mesh OK with no warnings, all twelve joined checks and separate semantic readback pass, and source custody closes cleanly.

Passage retains 298,660 prisms +247,804 tetrahedra. It achieves the full five-row schedule. Bulk conditioning is a no-op; the existing generic shared-apex repair moves two free outer vertices and clears the four reproduced historical low-weight interface faces. Current minimum weight0.05687894314413293>.055; mandatory bulk/interface/cap shape minima pass. Interface/cap percentage targets remain nonblocking misses. See NODAL5_REPORT.md for the full accepted result.

Blend closure details are recorded in the compact outcome, layer comparison and final reviews.

## Where the passage 115 minutes went

| Root stage | Seconds |
|---|---:|
| surface | 130.725 |
| first_front | 558.087 |
| v3_complete_plan | 1369.908 |
| v4_prospective_shell_checks | 1775.826 |
| v3_native_construction | 19.919 |
| v4_shell_admission | 107.256 |
| v4_direct_fill | 2902.121 |

Producer joined checks account for1111.873 s within direct fill. Separate full semantic replay accounts for1031.179 s. The existing FastProd surface path was already used (surface130.725 s); it does not disable required full-mesh validation.

The repeated work is intentional: planning source/collision checks validate a proposal; prospective shell checks independently validate the final shell before construction/admission; the later joined checks validate the actual prism/bulk assembly; separate post-lease replay checks the saved artifact through a different process. Some geometry predicates and array examinations are repeated across these boundaries, but these stages do not establish interchangeable claims. Relevant source: mesh.py execute; collision_policy.py construct/segment_guard; v4/tools/v35_shell.py prepare/enroll; direct_connection.py execute; direct_readback.py and its delegated semantic reader.

Omitting1031.179 s of replay would change the current acceptance contract. It is15.0% of observed worker time as an arithmetic component, not a measured achievable speedup; no alternative implementation or confidence policy was tested. Any future preview/production/audit-policy decision must state which independent checks are retained and which assurance claim changes. That policy and algorithm-efficiency work are deferred to a separate later task. This freeze adds no such controls and weakens no gate.

## Scope and limits

Surface size is a nominal target, not a hard maximum edge or final metric-ratio guarantee. Surface geometry is sampled numerical acceptance, with no whole-source completeness, continuous face accuracy, exact root/normal, native/real-target equivalence or determinism proof. Continuous SDF fluid clearance, CFD/solver/y-plus/flow accuracy and general-shape qualification are unclaimed. The parser supports settings beyond the tested rows, but those settings are not qualified by this matrix. Historical or closed FastProd authority is not reusable as fresh live authority.

Passage has all five nominal rows, total 180.862 um, no compression or growth reversals. Sphere likewise has no compression. The original thick-blend baseline uses local compression, with total thickness 58.5731–180.862 um and 2,429 scalar / 2,559 physical-path growth reversals; nominal growth 1.3 is not achieved everywhere. The current blend regression confirms those achieved values on the frozen build.

DirectFillEngineering/1 requires bulk/interface tetrahedron minima 0.25 and cap triangle minimum 0.30, plus actual-neighbour QA and all other current gates. Cap/interface population percentage targets can remain nonblocking TARGET_NOT_MET. A complete engineering pass is not a claim that every quality distribution target is met. checkMesh -meshQuality was not run because no applicable approved dictionary exists. Clean-machine portability is not tested.

The complete exact commands and configured-host installation instructions are in QUALIFICATION.md and INSTALL_FROZEN.md. The small next robustness proposal is preparatory only and will not run in this task. There is no new construction research branch, runtime/cell-count target or scheduled automation.

## Closed blend regression evidence

All five final mesh-array entries are byte-identical to the accepted original thick-blend baseline. Counts and all three quality minima are unchanged. Fresh worker 2771.970 s versus baseline 2871.605 s; these separate observations do not measure a speedup. Supervisor 2774.363 s, peak sampled Windows tree RSS 2,224,730,112 bytes, last sampled output 914,419,861 bytes. No resource stop, monitor error, source mutation or external retry occurred.

| Population | Minimum | Required minimum | Distribution target status |
|---|---:|---:|---|
| all_bulk_tetrahedra | 0.267328495238883 | 0.25 | TARGETS_MET |
| interface_tetrahedra | 0.296962653105827 | 0.25 | TARGET_NOT_MET |
| preserved_cap_triangles | 0.303489087983168 | 0.3 | TARGET_NOT_MET |

Five rows retain first-height request 20 um, nominal growth 1.3. Achieved scalar total thickness 58.573100–180.862000 um; scalar/path growth reversals 2429/2559. Compressed vertices: 1663; row schedule and first-normal evidence are recorded in CONSOLIDATE1_BLEND_LAYERS.json. This is a compressed stack, so the nominal growth schedule is not achieved everywhere.

Existing bulk conditioning outcome: AllBulkCellsAboveFloor; native calls 8; shared-apex changed vertices 0. No protected prism/boundary coordinates or connectivity are relaxed.

checkMesh: {"failed_lines": [], "literal_mesh_ok": true, "mesh_quality": "NOT_RUN_NO_APPLICABLE_APPROVED_DICTIONARY", "passed": true, "returncode": 0, "warnings": ["*Number of severely non-orthogonal (> 70 degrees) faces: 41."]}. All twelve joined producer checks and independent saved-artifact readback passed. Export and manifest pins match, source custody closes with the same frozen 1,866-file identity. No global exact-Q population or CFD qualification is inferred.

| Blend root stage | Seconds |
|---|---:|
| surface | 720.539 |
| first_front | 48.897 |
| v3_complete_plan | 366.373 |
| v4_prospective_shell_checks | 120.093 |
| v3_native_construction | 6.673 |
| v4_shell_admission | 17.169 |
| v4_direct_fill | 1486.459 |
