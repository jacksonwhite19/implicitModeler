# Sphere4 consolidation: unchanged LIVE3 implementation

The one fresh full normal-CLI sphere control passed source/surface/prism admission, genuine live V4 background handoff, all twelve joined checks, exact OpenFOAM conversion, literal Mesh OK, separate semantic readback and clean source closure. The current LIVE3 scientific/runtime files were unchanged from the accepted fresh blend build. No code change, numerical retry, tuning, broad test-suite rerun or new source/evidence archive was needed.

## Sphere regression control

The case bytes match the previously accepted sphere_direct_v4_02 exactly, SHA256 bfec3f4ff131f8783737b2fb0dbf5c91f2504493b7d7ecaab570c1703515cf8e. Geometry is sphere(1.0) at .001 metres/model-unit: radius1mm, diameter2mm. Surface size .2mm, true first height20um, five prism rows, nominal growth1.3, same V4 direct background and7200s whole-worker limit. Only the output directory changed.

Fresh counts are 12,780 prisms + 40,448 tetrahedra = 53,228 cells and 14,798 points. Old counts were12,780 +40,448 =53,228 cells and14,798 points. Fresh scalar prism total 180.862000–180.862000um; compressed vertices 0. Old scalar total180.862um, zero compressed vertices.

Actual conditioning outcome: AlreadyAboveFloor. No-search/no-native path observed: True. Existing-vertex repair invoked=False; constrained cavity invoked=False; local native calls=0; committed step count=0. The path was selected from actual fresh bulk labels, not forced. Original shared-apex repair remained separate and its receipt was replayed.

| Shape population | Old minimum | Fresh minimum | Mandatory floor |
|---|---:|---:|---:|
| All bulk tetrahedra | 0.483689798057 | 0.483689798057 | .25 |
| Interface tetrahedra | 0.606064589327 | 0.606064589327 | .25 |
| Preserved cap triangles | 0.750540877737 | 0.750540877737 | .30 |

Fresh percentage statuses: all_bulk_tetrahedra=TARGETS_MET, interface_tetrahedra=TARGETS_MET, preserved_cap_triangles=TARGETS_MET. Native warnings: []. Independent review verifies the numerical comparison, proof/manifest/artifact hashes, live parent chain, cold replay and source closure; see SPHERE4_INDEPENDENT_REVIEW.json for its exact scope.

Fresh worker 494.025s versus old 467.373s. Supervisor 495.812s, exit0, no monitor/resource stop. Peak sampled Windows tree RSS 957,759,488bytes; this is not aggregate Linux RSS. New source custody identity ed5336d7ac3d022ea6dec776acd1b6763f7bf244a6f5a3847cc621a38d346864, 1,858 files, closed clean. Background stage times and native budget are recorded in SPHERE4_METRICS.json. This control is not a performance benchmark or target.

## Consolidated demonstrated scope

The current generic V3-prism/V4-direct-outer path has now passed fresh complete normal-CLI sphere and original thick asymmetric-blend controls. There is no separate transition band. Blend LIVE3 produced46,570 prisms +72,871 tetrahedra =119,441 cells and38,868 points. It required one3-to-2 swap plus one31-to-54 constrained cavity replacement, four new interior sites, preserving all original coordinates/prisms/cap/farfield facets. Bulk minimum0.26732849523888347 passes the unchanged0.25 floor; all12joined proofs/export/MeshOK/coldreadback/sourceclosure passed.

Blend limitations remain unchanged:41faces above70degrees nonorthogonality, maximum77.26599753823804degrees; cap/interface percentage targets unmet under their existing nonblocking role; total prism thickness58.5731–180.862um and2,429scalar/2,559path growth reversals, despite nominal1.3. These were not tuned away. Neither result establishes CFD accuracy, yplus, arbitrary geometry, continuous SDF bulk-fluid proof or historical publication authority. A passing sphere regression plus one blend success is bounded integration evidence.

## Next challenge

Recommend ONE original curved passage in an ellipsoid for advisor dispatch5. Body semiaxes3.5/2.7/2.4mm; subtract an eight-sphere curved chain with radii1.00–1.35mm. Historical Task210 passed five prism layers but refused four of59,732 interface faces by interpolation weight alone: minimum0.04744306406398393 below mandatory0.055. Historical bulk/interface/cap shape minima already passed .25/.25/.30. Old percentage failures are nonblocking under the current engineering contract. This case tests inner-facing subtraction walls and actual prism/core coupling, distinct from the blend and sphere. Current V3.5 has not attempted it; static operator availability is not source admission or guaranteed success.

| Candidate | Retained result | Current status / decision |
|---|---|---|
| Curved passage, body envelope7×5.4×4.8mm | Mandatory interface-weight refusal | Next ONE proposed challenge; preserve200um surface/20umfirst/1.3/5rows and original numerical closure |
| Finite NACA0012 strip,1mchord/.5mspan | Historical second-row determinant refusal | Defer: polygon/spline geometry-authority caveat; no current V3.5 attempt |
| Concave five-sphere chain,radius.8mm | Historical complete V4 pass | New V3.5 coverage, not a prior failure; no current attempt |

SPHERE4_FIXTURE_INVENTORY.md/.json gives complete dimensions, source hashes and comparability. The missing original passage Rhai and historical public CASE were copied byte-identically into standalone fixtures only after sphere custody closed. Historical report/input/launch provenance was copied outside runtime; SPHERE4_PASSAGE_STAGING.json records every copy. No current derived passage CASE, geometry query or campaign was started. Existing generic operators need no fixture-specific adapter. Historical failures from older paths are not automatically current V3.5 failures.

## Exact recorded command

```powershell
python -B mesh.py --case fixtures/v4_case_template.json --geometry fixtures/sphere.rhai --metres-per-model-unit 0.001 --prism-first-height 0.00002 --prism-growth 1.3 --prism-layers 5 --surface-size 0.0002 --surface-policy size-led --background fixtures/background_direct_v4.json --output results/sphere_live4_01 --stop-after complete --timeout-seconds 7200
```

Run directory: C:\Users\Jackson\Desktop\02_Projects\09b_Implicit_CAD_claude\research\sdf_cfd_mesher_v3_5\standalone\results\sphere_live4_01. Previous sphere/blend outputs and archives remain preserved. Callback to advisor before final, then stop. No scheduled automation.
