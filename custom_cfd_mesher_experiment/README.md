# Custom CFD Mesher Experiment

Isolated working folder for building a CFD-oriented meshing path from the existing
SDF/STL exporter foundation.

The original target was a standalone mesher that writes CFD meshes directly,
without delegating volume generation to `snappyHexMesh`, Gmsh, or another
mesher. Current evidence says the direct custom path is useful for file-format
and near-wall experiments, but the optimizer's active OpenFOAM meshing path is
`snappyHexMesh`.

Current mesher policy:

- Use `snappyHexMesh` for optimizer candidate meshing, rough CFD scoring
  development, boundary-layer experiments, and current OpenFOAM runs.
- Treat Gmsh as historical plumbing/comparison evidence unless a task
  explicitly asks for a Gmsh comparison rerun.
- Do not switch optimizer runs back to Gmsh without a new evidence-backed
  decision.

Validated output targets:

- OpenFOAM `constant/polyMesh`
- Native SU2 ASCII `.su2`

## Current Recommended Pipeline

Use `scripts/run_gated_mesh_smoke.py` for one-command candidate smoke runs, or
`scripts/run_gmsh_cfd_pipeline.py` when you only need the mesh pipeline.

Optimizer-facing STL exports now default to an SDF-probed aircraft bbox through
`dual_contouring/direct_sparse_sdf_mc_experiment/scripts/optimizer_export_presets.py`.
Manual exporter bboxes are still allowed, but normal candidate exports should
not inherit the old reference-aircraft contour bbox.

The pipeline:

1. Loads an exporter-generated aircraft STL.
2. Optionally remeshes the high-resolution wall surface with PyMeshLab
   isotropic remeshing.
3. Scales from mm to m.
4. Imports the aircraft surface into Gmsh.
5. Uses `classifySurfaces` and `createTopology`.
6. Builds a padded farfield box.
7. Defines physical groups: `aircraft`, split farfield patches, and `fluid`.
8. Writes Gmsh MSH2 for OpenFOAM.
9. Writes native SU2.
10. Runs SU2 mesh validation.
11. Runs `gmshToFoam` and OpenFOAM `checkMesh`.
12. Runs sampled surface-fidelity audit against the pipeline input STL.
13. Renders an aircraft-only ISO screenshot.
14. Writes `pipeline_report.json`.

The full gated runner wraps the same path with source gates, remeshed-source
gates, mesh gates, OpenFOAM solver setup, a short `incompressibleFluid` smoke
run, and force-coefficient summary:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gated_mesh_smoke.py `
  --input-stl <candidate-aircraft.stl> `
  --run-dir <run-dir>
```

Optional source prep is available but disabled by default:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gated_mesh_smoke.py `
  --input-stl <candidate-aircraft.stl> `
  --run-dir <run-dir> `
  --prep-source-cap-loop-span-mm 2.0
```

Use this only when boundary localization shows tiny exporter/tile openings.
The cap report is preserved in `<run-dir>\source_prep\source_prep_report.json`.
Meaningful openings should fail the source gate instead of being repaired.

Current aerodynamic force convention:

- coordinate system: `X` streamwise/longitudinal, `Y` spanwise, `Z` vertical
- freestream default: `+X`
- drag direction: `+X`
- lift direction: `+Z`
- pitch axis: `+Y`
- `Cl` from older runs that used lift direction `+Y` is side-force-like and
  should not be compared directly to current vertical-lift `Cl`.

Current preferred clean-mesh recipe:

- PyMeshLab remesh target edge: `7.2 mm`
- PyMeshLab iterations: `5`
- PyMeshLab feature angle: `25 deg`
- PyMeshLab maximum surface distance: `0.25 mm`
- PyMeshLab smoothing: off
- `target_faces = 0` after remeshing; do not decimate the remeshed wall
- `surface_size = 0.0072 m`
- dynamic CFD farfield policy enabled by default:
  - upstream: `1.5 * x_extent`
  - downstream: `3.0 * x_extent`
  - side Y: `2.0 * y_extent`
  - side Z: `2.0 * z_extent`
  - minimum padding: `0.15 m`
- dynamic farfield mesh size enabled by default:
  - `max(surface_size, clamp(0.15 * max_aircraft_extent, 0.04 m, 0.35 m))`
- `geometry_mode = create-topology`
- `algorithm3d = 4`
- `optimize = default,Netgen,Relocate3D`
- `farfield_patches = split`

Experimental feature-curve sizing exists in `gmsh_external_flow_mesher.py`, but
is not part of the default pipeline. Both aggressive and lite curve-distance
background-field tests exceeded the 5 minute run window before producing a
usable mesh. Keep this opt-in until it is bounded and predictable.

Current best clean Gmsh run:
`runs/cleanmesh_surface_iso_7p2mm_nosmooth_gmsh_split_alg4`

- Aircraft input surface: 27,510 remeshed triangles.
- Farfield padding: `0.7,0.5,0.5` m.
- Surface size: `0.0072` m.
- Farfield size: `0.11` m.
- Gmsh 3D algorithm: `4` / Frontal.
- Gmsh optimizers: `default`, `Netgen`, `Relocate3D`.
- OpenFOAM conversion: `gmshToFoam` succeeded.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 379,086 tetrahedra.
- Points: 72,032.
- Aircraft patch: 27,510 faces.
- Split farfield patch total: 3,268 faces.
- Max aspect ratio: 106.12576.
- Max skewness: 3.8317653.
- Max non-orthogonality: 86.687852.
- Severe non-orthogonal faces: 69.
- SU2 validation: valid, 379,086 tetra elements, split farfield markers.
- Remeshed surface fidelity versus original exporter STL:
  - Bidirectional p95: 0.1365 mm.
  - Bidirectional p99: 0.1847 mm.
  - Bidirectional max: 0.3149 mm.
- Report:
  `runs/cleanmesh_surface_iso_7p2mm_nosmooth_gmsh_split_alg4/CLEAN_MESH_REPORT.md`.
- Solver rerun report:
  `runs/cleanmesh_surface_iso_7p2mm_nosmooth_gmsh_split_alg4/SOLVER_RERUN_REPORT.md`.
- Aircraft-only view set:
  `runs/cleanmesh_surface_iso_7p2mm_nosmooth_gmsh_split_alg4/views`.
- Default OpenFOAM `incompressibleFluid` behavior:
  - 30-step laminar run completed without floating point exception.
  - 60-step laminar run completed without floating point exception.
  - 60-step force-logging run completed without coefficient explosion.
  - Final 60-step force values: `Cd = 0.076177125`,
    `Cl = -0.0667940004`, `Cm = 0.00811865743`.
  - Automated force stability gate still fails on tail drift, so this remains
    smoke-test ready rather than scoring-ready.

Previous 10k decimated Gmsh run:
`runs/improve_gmsh_10k_hxt_netgen_far011`

- It remains useful as a lower-cell-count baseline.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 122,351 tetrahedra.
- Severe non-orthogonal faces: 1,485.
- Surface p95 deviation versus original exporter STL: 0.0977 mm.
- Solver smoke reports remain in that run folder.

Current split-farfield solver-ready follow-up:
`runs/improve_gmsh_10k_hxt_netgen_far011_split_farfield`

- Same 10k aircraft surface and same 122,351 tetrahedra.
- OpenFOAM boundary patches: `aircraft`, `inlet`, `outlet`, `side_ymin`,
  `side_ymax`, `side_zmin`, `side_zmax`.
- SU2 markers match the split patch names.
- Default 30-step `incompressibleFluid` still fails with a floating point
  exception around time step 25.
- Conservative controls complete 30-step and 60-step smoke runs.
- 60-step conservative result: completed, final p residual `5.1801076e-07`,
  final velocity residual `0.0011933942`.
- Report:
  `runs/improve_gmsh_10k_hxt_netgen_far011_split_farfield/SPLIT_FARFIELD_OPENFOAM_REPORT.md`.

Force-coefficient scoring readiness:

- Report:
  `runs/improve_gmsh_10k_hxt_netgen_far011_split_farfield/FORCE_COEFFS_SCORING_READINESS_REPORT.md`.
- `forceCoeffs` plumbing works and writes `forceCoeffs.dat`.
- `scripts/summarize_force_coeffs.py` automatically rejects unstable force
  histories.
- Potential-flow initialization improves stability versus uniform initialization
  but still fails scoring readiness.
- Current verdict: do not use OpenFOAM coefficients from this mesh for optimizer
  scoring.

Boundary-layer/prism status:

- Report: `GMSH_BOUNDARY_LAYER_FINDINGS.md`.
- Current verdict: direct Gmsh boundary layers are rejected for near-term
  scoring CFD, but the custom direct-prism-shell plus Gmsh outer-volume hybrid
  path is now viable for OpenFOAM plumbing.
- Best prism attempt: `runs/bl_gmsh_aircraft_5k_angle50_h0005_recombine`.
- SU2 validation passed for the mixed tet/prism file, but OpenFOAM strict
  checks still failed after `gmshToFoam -keepOrientation` with 3 negative-volume
  cells and 13 wrong-oriented faces.
- Local scalar-view layer suppression was rejected because it created invalid
  OpenFOAM topology.
- Keep the tetrahedral `improve_gmsh_10k_hxt_netgen_far011` run as the optimizer
  baseline until a boundary-layer strategy passes strict checks.

Hybrid direct-prism-shell plus Gmsh outer-volume status:

- Driver:
  `scripts/run_hybrid_shell_gmsh_exact_merge_smoke.py`.
- Interface merge utility:
  `scripts/merge_prism_shell_gmsh_outer.py`.
- Evidence root:
  `runs/hybrid_shell_gmsh_exact_merge_five_variant_netgen_v0_1_20260624`.
- Summary artifacts:
  `hybrid_summary.json`, `hybrid_summary.csv`, and
  `aircraft_iso_contact_sheet.png`.
- Pipeline:
  1. use a watertight conditioned aircraft surface,
  2. generate a three-layer direct prism shell at cumulative offsets
     `0.2/0.5/1.0 mm`,
  3. export `aircraft_wall.stl` and `outer_shell.stl`,
  4. build a Gmsh tetrahedral farfield volume from `outer_shell.stl`,
  5. run Gmsh optimizers `default,Netgen`,
  6. convert the Gmsh outer volume with `gmshToFoam`,
  7. exact-merge the shell and outer volume without OpenFOAM `stitchMesh`,
  8. run strict `checkMesh`,
  9. run `potentialFoam -writep` as plumbing smoke only.
- Five faired-cap variants passed strict `checkMesh` and completed
  `potentialFoam`.
- Cell range: `752,726` to `992,789`.
- Severe non-orthogonal face range: `178` to `470`.
- Runtime range: about `103` to `141` seconds per variant.
- Important lesson: OpenFOAM `mergeMeshes` plus `stitchMesh` did not reliably
  close the shell/Gmsh interface. Use the exact merge utility.
- Important lesson: a no-optimizer Gmsh outer-volume run can leave isolated
  high-aspect tets. Use `default,Netgen` for the current hybrid preset.
- Current limitation: this is still plumbing-ready, not scoring-ready. The
  aircraft wall/no-slip treatment, y+ policy, force coefficients, and validation
  against known aero expectations are still open.
- Latest no-slip steady probe:
  `runs/hybrid_shell_gmsh_simplefoam_forcecoeffs_probe_v0_1_20260624`.
  The hybrid fcv04 laminar no-slip steady runs failed by time step 2 with
  pressure-solver floating point exceptions. `forceCoeffs` wrote rows, but the
  coefficients were unstable. See `NO_SLIP_STEADY_PROBE_REPORT.md` in that run
  root. This confirms the hybrid mesh is plumbing-ready, while scoring CFD is
  still blocked by no-slip solver/wall-treatment stability.
- Latest split-farfield/transient diagnostic:
  `runs/hybrid_shell_gmsh_split_farfield_probe_v0_1_20260624`.
  Split farfield patch preservation now works through the exact merge and the
  split case passes strict `checkMesh` plus `potentialFoam -writep`. A matching
  split-farfield steady no-slip run still failed at time step 2 with a
  pressure-solver floating point exception, so single-versus-split farfield is
  not the primary blocker. A transient `icoFoam` run failed at `deltaT=1e-5 s`
  with max Courant number about `96.7`, but completed five tiny steps at
  `deltaT=1e-7 s`. A ten-step extension at the same timestep also completed,
  with final max Courant about `0.826` and final local continuity about
  `9.10e-15`, but took about `120 s` to advance only `1e-6 s` of physical
  time. Treat this as evidence that the mesh is not fundamentally unreadable or
  impossible, but the no-slip scoring path is currently limited by tiny
  near-wall cells, startup/time-scale control, pressure correction, and
  wall-treatment/y-plus policy.
- SU2 remains a plausible future solver backend, but it should not be treated
  as a cure for near-wall mesh/time-scale problems. It would need a native
  hybrid mesh export or robust converter plus the same marker, wall-treatment,
  reference-value, y-plus, and convergence gates.

Repeatability status:

- Report: `REPEATABILITY_VARIANT_REPORT.md`.
- Gated baseline report: `GATED_BASELINE_REPORT.md`.
- The 7.2 mm no-smooth PyMeshLab remesh plus Gmsh Frontal split-farfield path
  was rerun on additional aircraft variants.
- `direct_sdf_oml_spacing_1p0.stl` passed mesh generation, SU2 validation,
  OpenFOAM `checkMesh`, 60-step solver smoke, and the force stability gate.
- A gated 4% span-stretch stress variant passed source gates, mesh gates,
  OpenFOAM `checkMesh`, 60-step solver smoke, and the force stability gate.
- A full new exported aircraft variant with 35 degree wing sweep, 20% larger
  root chord, and 20% larger tail assembly passed the gated mesh path after
  using an explicit wider exporter bbox and capping one tiny four-edge source
  opening. The 60-step OpenFOAM smoke run completed without floating point
  exception, but the force stability summary rejected it for scoring due to
  lift/moment tail drift.
- `runs/sweep35_root120_tail120_runner_prep_skip_solver` verifies that the
  gated runner's optional source-prep flag can perform the tiny cap and reach
  mesh gates from the bboxwide STL in one command.
- `runs/sweep35_root120_tail120_autobbox_dynamic_farfield_skip_solver`
  verifies the automatic exporter bbox plus dynamic CFD farfield policy on the
  same swept/larger-tail variant:
  - auto-bbox export bounds: `[-64,-224,-288]` to `[736,112,544]` mm.
  - dynamic farfield padding: upstream `1.014 m`, downstream `2.029 m`,
    side Y `0.426 m`, side Z `1.432 m`.
  - dynamic farfield mesh size: `0.1074 m`.
  - mesh gate: pass, `407,749` cells, `79,224` points, `31,126` aircraft
    faces, zero `defaultFaces`.
  - 60-step OpenFOAM smoke from the dynamic-domain mesh completed without
    floating point exception, but force stability still failed for scoring.
- `runs/sweep35_root120_tail120_exporter_capfix_dynamic_forces_zlift`
  verifies the exporter-level tiny-loop cap, dynamic farfield policy, and
  corrected force convention with no mesher-side source prep:
  - source gate: pass from exporter STL, zero boundary edges.
  - mesh gate: pass, `404,723` cells, `78,688` points, `30,964` aircraft
    faces, zero `defaultFaces`.
  - force convention: lift `0,0,1`, drag `1,0,0`, pitch axis `0,1,0`.
  - 60-step OpenFOAM smoke completed without floating point exception.
  - force stability summary passed for this smoke window.
  - final smoke coefficients: `Cd=0.065971069`, `Cl=-0.113468059`,
    `Cm=0.0267205536`.
  - still not final scoring CFD because solver policy, reference values,
    angle-of-attack policy, and near-wall treatment remain provisional.
- Raw `basic_no_inlet_root180_tip70_spacing_1p0_spacing_1p0.stl` failed because
  the source STL was not watertight.
- A repaired root/tip case completed the solver and force gate, but required a
  10 mm cap, skipped Netgen after a Gmsh optimizer failure, and created 14
  OpenFOAM `defaultFaces`, so it is conditional rather than a clean pass.
- `scripts/cfd_mesh_gates.py` is now the baseline gate tool:
  - source gate: watertight, zero boundary edges, zero nonmanifold edges,
    one body.
  - mesh gate: valid SU2, OpenFOAM `Mesh OK`, expected split patches, zero
    `defaultFaces`, expected Gmsh optimizer provenance, bounded skew,
    non-orthogonality, severe non-orthogonal faces, and aspect ratio.

## Direct Mesher Status

Implemented direct mesh generators:

- `scripts/direct_cartesian_cfd_mesher.py`
  - Direct Cartesian external-flow cut-out.
  - Writes OpenFOAM and SU2 directly.
  - Validates OpenFOAM file writing, patch ordering, owner/neighbour ordering,
    and SU2 marker structure.
  - Limitation: aircraft wall is stair-stepped, so this is not the final
    high-fidelity aircraft boundary approach.

- `scripts/direct_prism_shell_mesher.py`
  - Direct body-fitted prism-shell mesh.
  - Uses aircraft surface triangles as the actual wall patch.
  - Extrudes an offset farfield shell.
  - Supports adaptive per-vertex offset caps from local triangle scale.
  - Writes OpenFOAM and SU2 directly.
  - Validated on a clean sphere and on a decimated aircraft shell.
  - Limitation: current farfield is an offset body shell, not a rectangular or
    tunnel-style farfield domain. It also needs robust multi-layer growth and
    high-fidelity surface cleanup before production CFD use.

## Current Verified Runs

### Direct Cartesian Writer Smoke Test

`runs/direct_cartesian_smoke_sorted`

- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 3,882 hexahedra.
- Boundary patches: `farfield`, `aircraft`.
- SU2 file emitted.
- Purpose: file-format and topology writer validation.

### Direct Body-Fitted Sphere Test

`runs/direct_prism_shell_sphere`

- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 5,120 prisms.
- Aircraft patch: 5,120 triangles.
- Purpose: validates body-fitted prism shell topology on clean geometry.

### Direct Body-Fitted Aircraft Test

`runs/direct_prism_shell_aircraft_1800_pass`

- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 1,800 prisms.
- Aircraft patch: 1,800 triangles.
- Farfield patch: 1,800 triangles.
- SU2 validation: valid structure, 1,800 prism elements, `aircraft` and
  `farfield` markers.
- Screenshot: `runs/direct_prism_shell_aircraft_1800_pass/aircraft_iso.png`.

This proves direct OpenFOAM/SU2 mesh generation on the aircraft. It is still too
coarse to be the final CFD surface quality.

`runs/direct_prism_shell_aircraft_4k_layers_adaptive`

- Current best direct custom aircraft mesh.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 12,000 prisms.
- Aircraft patch: 4,000 triangles.
- Farfield patch: 4,000 triangles.
- Layers: three cumulative adaptive layers requested at `0.001,0.002,0.005`.
- SU2 validation: valid structure, 12,000 prism elements, `aircraft` and
  `farfield` markers.
- Screenshot: `runs/direct_prism_shell_aircraft_4k_layers_adaptive/aircraft_iso.png`.

### Direct Multi-Layer Status

`runs/direct_prism_shell_sphere_layers`

- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 15,360 prisms.
- Offsets: `0.02,0.05,0.1`.
- Purpose: validates multi-layer prism extrusion on clean geometry.

`runs/direct_prism_shell_aircraft_1800_layers`

- SU2 structure validation: valid.
- OpenFOAM `checkMesh`: fails geometry checks once grown to 5 mm.
- Failure mode: local open/inverted cells around bad/high-curvature aircraft
  regions.
- Conclusion: aircraft layer growth needs local quality-controlled smoothing or
  rollback before this can become a full farfield mesh.

Adaptive caps resolve this failure for the same nominal offsets:

- `runs/direct_prism_shell_aircraft_1800_layers_adaptive`: passes OpenFOAM and
  SU2 validation.
- `runs/direct_prism_shell_aircraft_4k_layers_adaptive`: passes OpenFOAM and
  SU2 validation.
- `runs/direct_prism_shell_aircraft_5k_layers_adaptive`: still fails locally.

Observed current ceiling: about 4,000 aircraft wall triangles with the existing
exporter surface and this normal-growth algorithm.

### Direct Farfield Transition Status

Implemented experimental `radial-box` and `radial-sphere` outer modes in
`direct_prism_shell_mesher.py`.

Result:

- Near-wall body-fitted shell remains valid.
- Projecting the same surface topology to a large padded farfield creates
  inverted/open transition cells.
- `runs/direct_prism_shell_aircraft_1800_radial_box`: fails OpenFOAM geometry
  checks.
- `runs/direct_prism_shell_aircraft_1800_radial_sphere`: fails OpenFOAM geometry
  checks.

Conclusion: a production farfield volume needs either a real transition mesher
between the body-fitted shell and an outer Cartesian/tet domain, or an
off-the-shelf constrained volume mesher such as Gmsh.

### Gmsh External-Flow Status

Implemented `scripts/gmsh_external_flow_mesher.py`.

The working Gmsh path:

- Prepares/scales/optionally decimates the aircraft STL.
- Imports STL into Gmsh.
- Uses `classifySurfaces` and `createTopology`.
- Builds a padded farfield box.
- Defines physical groups: `aircraft`, `farfield`, `fluid`.
- Writes MSH2 for OpenFOAM `gmshToFoam`.
- Writes native SU2 and validates with `validate_su2_mesh.py`.

Working run: `runs/gmsh_aircraft_1800_topology_fine_opt`

- Aircraft input surface: 1,800 triangles.
- Farfield padding: `0.7,0.5,0.5` m.
- OpenFOAM conversion: `gmshToFoam` succeeded.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 19,434 tetrahedra.
- Max skewness: 2.9277767.
- SU2 validation: valid, 19,434 tetra elements, `aircraft` and `farfield`
  markers.
- Screenshot: `runs/gmsh_aircraft_1800_topology_fine_opt/aircraft_iso.png`.

Current recommended Gmsh baseline:
`runs/gmsh_aircraft_4k_topology_finer_opt`

- Aircraft input surface: 4,000 triangles.
- Farfield padding: `0.7,0.5,0.5` m.
- Surface size: `0.01` m.
- Farfield size: `0.12` m.
- Gmsh optimizers: `default`, `Netgen`, `Relocate3D`.
- OpenFOAM conversion: `gmshToFoam` succeeded.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 52,166 tetrahedra.
- Points: 10,553.
- Aircraft patch: 4,000 faces.
- Farfield patch: 2,916 faces.
- Max skewness: 3.2031642.
- Max non-orthogonality: 88.669617.
- SU2 validation: valid, 52,166 tetra elements, `aircraft` and `farfield`
  markers.
- Screenshot: `runs/gmsh_aircraft_4k_topology_finer_opt/aircraft_iso.png`.

Higher-fidelity Gmsh candidate:
`runs/gmsh_aircraft_8k_topology_finer_opt`

- Aircraft input surface: 8,000 triangles.
- Farfield padding: `0.7,0.5,0.5` m.
- Surface size: `0.008` m.
- Farfield size: `0.12` m.
- Gmsh optimizers: `default`, `Netgen`, `Relocate3D`.
- Gmsh warning: `2 ill-shaped tets are still in the mesh`.
- OpenFOAM conversion: `gmshToFoam` succeeded.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 91,586 tetrahedra.
- Points: 18,214.
- Aircraft patch: 8,000 faces.
- Farfield patch: 2,918 faces.
- Max skewness: 3.3085963.
- Max non-orthogonality: 88.702112.
- SU2 validation: valid, 91,586 tetra elements, `aircraft` and `farfield`
  markers.
- Screenshot: `runs/gmsh_aircraft_8k_topology_finer_opt/aircraft_iso.png`.

Repeatable pipeline proof:
`runs/pipeline_span_plus20_8k_repeatable`

- Input: `runs/wing_span_plus20_v3_transition/aircraft_surface.stl`.
- Aircraft input surface: 8,000 triangles.
- OpenFOAM conversion: `gmshToFoam` succeeded.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 91,586 tetrahedra.
- Points: 18,214.
- Max skewness: 3.3085963.
- Max non-orthogonality: 88.702112.
- SU2 validation: valid, 91,586 tetra elements, `aircraft` and `farfield`
  markers.
- Surface fidelity versus original exporter STL:
  - Bidirectional p95: 0.1026 mm.
  - Bidirectional p99: 0.1500 mm.
  - Bidirectional max: 0.3706 mm.
- Screenshot: `runs/pipeline_span_plus20_8k_repeatable/aircraft_iso.png`.
- Report: `runs/pipeline_span_plus20_8k_repeatable/pipeline_report.json`.

Changed-geometry pipeline proof:
`runs/pipeline_root180_tip70_8k_repeatable`

- Input:
  `../dual_contouring/direct_sparse_sdf_mc_experiment/stl/basic_no_inlet_root180_tip70_spacing_1p0_spacing_1p0.stl`.
- Aircraft input surface: 8,000 triangles.
- OpenFOAM conversion: `gmshToFoam` succeeded.
- OpenFOAM `checkMesh`: `Mesh OK.`
- Cells: 109,313 tetrahedra.
- Points: 19,461.
- Max skewness: 1.8948456.
- Max non-orthogonality: 88.641445.
- SU2 validation: valid, 109,313 tetra elements, `aircraft` and `farfield`
  markers.
- Surface fidelity versus original exporter STL:
  - Bidirectional p95: 0.1057 mm.
  - Bidirectional p99: 0.1560 mm.
  - Bidirectional max: 0.4102 mm.
- Screenshot: `runs/pipeline_root180_tip70_8k_repeatable/aircraft_iso.png`.
- Report: `runs/pipeline_root180_tip70_8k_repeatable/pipeline_report.json`.

Failed Gmsh path:

- `createGeometry` fails on this STL with
  `Wrong topology of boundary mesh for parametrization`.
- `createTopology` works and should be the default for STL-based Gmsh trials.

## Current Roadblock

The full exporter STL contains many tiny/sliver triangles. Adaptive caps can keep
the direct prism shell valid up to the current 4k wall-triangle case, but the caps
also limit actual farfield growth. A requested 100 mm shell on the 1.8k aircraft
case produced a valid mesh, but local caps limited the actual max offset to about
8.5 mm and the median offset to about 1.8 mm.

The next algorithmic work is:

1. Add aircraft-surface quality gates before extrusion.
2. Add local extrusion rollback or smoothing at sharp blends.
3. Improve the maximum safe layer-thickness estimator per vertex/face.
4. Keep the 8k Gmsh pipeline as the near-term CFD mesh baseline.
5. Bound feature-curve size fields so they improve LE/TE/blend capture without
   unbounded runtime.
6. Keep OpenFOAM/SU2 writers unchanged as output backends.

## Older OpenFOAM/Snappy Baseline

The earlier OpenFOAM snappy baseline remains useful only as a comparison point:

1. Take an aircraft surface STL from the exporter.
2. Preserve the aircraft surface geometry.
3. Repair only bounded surface defects that are clearly local holes.
4. Emit surface quality metrics.
5. Generate an OpenFOAM `snappyHexMesh` case.
6. Run OpenFOAM mesh checks and render an aircraft-only ISO screenshot.

## Current Known-Good Run

`runs/wing_span_plus20_v3_transition` is the first passing OpenFOAM mesh from the
basic no-inlet aircraft exporter STL.

Key result:

- Surface: watertight, no boundary edges, no non-manifold edges.
- `surfaceCheck`: closed surface, no illegal triangles, one normal-orientation zone.
- `snappyHexMesh`: finished without errors.
- `checkMesh`: `Mesh OK.`
- Cells: 235,936.
- Aircraft patch faces: 17,108.
- Max skewness: 3.24539.
- Max non-orthogonality: 69.999256.
- Screenshot: `runs/wing_span_plus20_v3_transition/aircraft_iso.png`.

The surface still contains many tiny low-quality triangles inherited from the
exporter STL. This is now explicitly reported instead of hidden.

## Layout

- `scripts/` - direct meshers, validators, OpenFOAM baseline generation, and
  screenshot utilities.
- `inputs/` - optional copied input surfaces or SDF definitions.
- `runs/` - generated run outputs.

## Direct Mesher Commands

Direct body-fitted aircraft shell:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\direct_prism_shell_mesher.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\direct_prism_shell_aircraft_1800_pass `
  --target-faces 1800 `
  --offset 0.001 `
  --write-openfoam `
  --write-su2
```

Direct multi-layer shell:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\direct_prism_shell_mesher.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\direct_prism_shell_aircraft_4k_layers_adaptive `
  --target-faces 4000 `
  --offsets 0.001,0.002,0.005 `
  --adaptive-offset-caps `
  --offset-cap-factor 0.5 `
  --offset-cap-floor 1e-5 `
  --write-openfoam `
  --write-su2
```

Validate OpenFOAM:

```powershell
wsl bash -lc 'source /opt/openfoam13/etc/bashrc && cd /mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/custom_cfd_mesher_experiment/runs/direct_prism_shell_aircraft_1800_pass/openfoam_case && checkMesh'
```

Validate SU2 structure:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\validate_su2_mesh.py `
  --mesh runs\direct_prism_shell_aircraft_1800_pass\mesh.su2
```

Direct Cartesian smoke mesh:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\direct_cartesian_cfd_mesher.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\direct_cartesian_smoke_sorted `
  --cells 18,12,18 `
  --write-openfoam `
  --write-su2
```

Gmsh external-flow mesh:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\gmsh_external_flow_mesher.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\gmsh_aircraft_4k_topology_finer_opt `
  --target-faces 4000 `
  --padding 0.7,0.5,0.5 `
  --surface-size 0.01 `
  --farfield-size 0.12 `
  --geometry-mode create-topology `
  --optimize default,Netgen,Relocate3D
```

Repeatable Gmsh CFD pipeline:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gmsh_cfd_pipeline.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\improve_gmsh_10k_hxt_netgen_far011 `
  --target-faces 10000 `
  --surface-size 0.0072 `
  --farfield-size 0.11 `
  --padding 0.7,0.5,0.5
```

Split-farfield Gmsh CFD pipeline:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gmsh_cfd_pipeline.py `
  --input-stl runs\wing_span_plus20_v3_transition\aircraft_surface.stl `
  --run-dir runs\improve_gmsh_10k_hxt_netgen_far011_split_farfield `
  --target-faces 10000 `
  --surface-size 0.0072 `
  --farfield-size 0.11 `
  --padding 0.7,0.5,0.5 `
  --algorithm3d 10 `
  --optimize default,Netgen,Relocate3D `
  --fidelity-samples 8000 `
  --farfield-patches split
```

OpenFOAM steady-solver smoke case:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\setup_incompressible_fluid_smoke.py `
  --source-case-dir runs\improve_gmsh_10k_hxt_netgen_far011\openfoam_case `
  --case-dir runs\improve_gmsh_10k_hxt_netgen_far011\openfoam_case_incompressible_smoke_15 `
  --velocity 22.352,0,0 `
  --nu 1.5e-5 `
  --end-time 15 `
  --write-interval 5 `
  --farfield-mode fixed
```

Run in WSL:

```powershell
wsl bash -lc "source /opt/openfoam13/etc/bashrc && cd /mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/custom_cfd_mesher_experiment/runs/improve_gmsh_10k_hxt_netgen_far011/openfoam_case_incompressible_smoke_15 && checkMesh > log.checkMesh 2>&1 && foamRun -solver incompressibleFluid > log.incompressibleFluid 2>&1"
```

Changed-geometry proof run:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gmsh_cfd_pipeline.py `
  --input-stl ..\dual_contouring\direct_sparse_sdf_mc_experiment\stl\basic_no_inlet_root180_tip70_spacing_1p0_spacing_1p0.stl `
  --run-dir runs\pipeline_root180_tip70_8k_repeatable `
  --target-faces 8000 `
  --surface-size 0.008 `
  --farfield-size 0.12 `
  --padding 0.7,0.5,0.5
```

Convert/check in OpenFOAM:

```powershell
wsl bash -lc 'source /opt/openfoam13/etc/bashrc && cd /mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/custom_cfd_mesher_experiment/runs/gmsh_aircraft_4k_topology_finer_opt/openfoam_case && gmshToFoam ../mesh.msh && checkMesh'
```

Render an aircraft-only ISO screenshot without ParaView:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\render_stl_iso_screenshot.py `
  --input-stl runs\gmsh_aircraft_4k_topology_finer_opt\aircraft_prepared_for_gmsh.stl `
  --output-png runs\gmsh_aircraft_4k_topology_finer_opt\aircraft_iso.png `
  --edges
```

## Snappy Baseline Commands

## Current Boundary-Layer Direction

The current Gmsh full external-volume backend remains the best default
OpenFOAM plumbing path. It is fast and has passed strict `checkMesh` plus
`potentialFoam` on the five faired-cap variants, but the one-layer Gmsh prism
setup is not scoring CFD.

Bounded fcv04 multi-layer Gmsh tests on 2026-06-24 failed strict `checkMesh`:

- `runs/gmsh_bl_multilayer_probe_v0_1_20260624/fcv04_layers2_noskip_h100_250_algo4`
- `runs/gmsh_bl_multilayer_probe_v0_1_20260624/fcv04_layers3_h50_120_250`
- `runs/gmsh_bl_multilayer_probe_v0_1_20260624/fcv04_layers2_noskip_h100_250_algo4_opt`
- `runs/gmsh_bl_multilayer_probe_v0_1_20260624/fcv04_layers3_norecombine_h50_120_250_algo4`

The useful new result is the high-fidelity direct prism shell:

`runs/direct_prism_shell_fcv04_86k_layers_small_cap025_20260624`

- Input: fcv04 selected 86,624-face remeshed aircraft surface.
- Layers: cumulative offsets `0.0002,0.0005,0.001` m with adaptive caps.
- OpenFOAM `checkMesh`: `Mesh OK`.
- Cells: 259,872 prisms.
- Max non-orthogonality: 81.637043.
- Severe non-orthogonal faces: 343.
- Max skewness: 3.5425877.
- Runtime: 23.7 s.
- Screenshot: `runs/direct_prism_shell_fcv04_86k_layers_small_cap025_20260624/aircraft_iso.png`.

Follow-up five-variant shell evidence:

`runs/direct_prism_shell_five_variant_v0_1_20260624`

- Four of five variants passed with the common selected Gmsh-BL remesh surface.
- `fcv01_long_glider` passed after remeshing with feature angle `45 deg`:
  `runs/direct_prism_shell_five_variant_v0_1_20260624_fcv01_remesh_tuning/fd45/direct_shell`.
- This means the direct shell needs a surface-remesh selector, but the prism
  layer settings themselves are repeatable.

This is not a complete external-flow CFD mesh because the outer boundary is an
offset aircraft shell, not a farfield box. The next high-value meshing path is a
hybrid: direct body-fitted prism shell near the aircraft plus a conformal
transition into the Gmsh/OpenFOAM farfield volume.

Hybrid transition probe:

`runs/hybrid_shell_gmsh_transition_probe_v0_1_20260624`

- Direct fcv04 shell passes strict `checkMesh`.
- Gmsh outer volume around the shell interface passes strict `checkMesh`.
- Simple all-prism radial transition fails.
- OpenFOAM `mergeMeshes` plus `stitchMesh` is not yet acceptable; it stitches
  about 99.98% of the interface and leaves enough unmatched/degenerate faces to
  fail strict `checkMesh`.
- Next step is an exact conformal merge utility that maps the Gmsh inner patch
  onto the direct shell outer patch by shared interface faces rather than using
  geometric patch intersection.

Prepare a CFD surface:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\cfd_surface_mesher.py `
  --input-stl ..\dual_contouring\direct_sparse_sdf_mc_experiment\stl\basic_no_inlet_wing_span_plus20_spacing_1p0.stl `
  --output-stl runs\wing_span_plus20_v1\aircraft_surface.stl `
  --report runs\wing_span_plus20_v1\surface_report.json
```

Generate an OpenFOAM case:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\make_openfoam_case.py `
  --input-stl runs\wing_span_plus20_v1\aircraft_surface.stl `
  --case-dir runs\wing_span_plus20_v1\openfoam_case `
  --base-cells 56,36,56 `
  --surface-min-level 2 `
  --surface-max-level 3 `
  --feature-level 2
```

Run the case in WSL:

```powershell
wsl bash -lc 'source /opt/openfoam13/etc/bashrc && cd /mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/custom_cfd_mesher_experiment/runs/wing_span_plus20_v1/openfoam_case && surfaceCheck constant/geometry/aircraft.stl && blockMesh && surfaceFeatures && snappyHexMesh -overwrite && checkMesh'
```

The first passing settings used:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\make_openfoam_case.py `
  --input-stl runs\wing_span_plus20_v2_normals\aircraft_surface.stl `
  --case-dir runs\wing_span_plus20_v3_transition\openfoam_case `
  --base-cells 56,36,56 `
  --surface-min-level 2 `
  --surface-max-level 3 `
  --feature-level 2 `
  --n-cells-between-levels 4 `
  --snap-tolerance 0.5 `
  --n-smooth-patch 10
```

Summarize a completed run:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\summarize_openfoam_run.py `
  --run-dir runs\wing_span_plus20_v3_transition
```

Or run the full OpenFOAM meshing pipeline from one command:

```powershell
.\scripts\run_openfoam_pipeline.ps1 `
  -InputStl ..\dual_contouring\direct_sparse_sdf_mc_experiment\stl\basic_no_inlet_wing_span_plus20_spacing_1p0.stl `
  -RunDir runs\wing_span_plus20_next
```
