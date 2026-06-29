# Gated Clean Mesh Baseline Report

Verdict: PASS for near-term optimizer mesh/solver baseline.

This report promotes the PyMeshLab + Gmsh Frontal split-farfield path to the
current baseline, with explicit gates that prevent known bad/conditional cases
from silently entering CFD scoring.

## Added Gate Tools

- `scripts/cfd_mesh_gates.py`
  - `source`: validates STL watertightness, boundary edges, nonmanifold edges,
    and body count before Gmsh.
  - `mesh`: validates SU2, OpenFOAM `checkMesh`, expected patch names, absence
    of `defaultFaces`, mesh-quality limits, and expected Gmsh optimizer
    provenance.
- `scripts/make_affine_stl_variant.py`
  - deterministic stress-variant generator used only for repeatability testing.
  - records scale/translation/provenance in JSON.
- `scripts/run_gated_mesh_smoke.py`
  - one-command candidate path: source gate, PyMeshLab remesh, remeshed-source
    gate, Gmsh volume mesh, mesh gate, OpenFOAM setup, 60-step solver smoke,
    and force-coefficient summary.
  - dynamic farfield domain sizing is enabled by default.

## Baseline Gates

Source-surface gate:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\cfd_mesh_gates.py source `
  --input-stl <candidate-aircraft.stl> `
  --report <run>\source_gate.json
```

Default pass requirements:

- `watertight == true`
- `boundary_edges == 0`
- `nonmanifold_edges == 0`
- `body_count <= 1`

Mesh-result gate:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\cfd_mesh_gates.py mesh `
  --pipeline-report <run>\pipeline_report.json `
  --case-dir <run>\openfoam_case `
  --report <run>\mesh_gate.json
```

Default pass requirements:

- SU2 validation valid
- OpenFOAM `checkMesh` reports `Mesh OK`
- expected OpenFOAM patches exactly:
  `aircraft`, `inlet`, `outlet`, `side_ymin`, `side_ymax`, `side_zmin`,
  `side_zmax`
- no unexpected patches
- `defaultFaces == 0`
- Gmsh optimizers exactly `default,Netgen,Relocate3D`
- `failed_checks == 0`
- `highly_skew_faces == 0`
- `severely_non_orthogonal_faces <= 250`
- `max_non_orthogonality <= 89 deg`
- `max_skewness <= 4.0`
- `max_aspect_ratio <= 250`

Farfield policy:

- Export bbox policy is separate from CFD farfield policy.
- Optimizer-facing STL exports should use SDF-probed auto-bbox unless explicit
  bounds are intentionally supplied.
- Gmsh/OpenFOAM cases now default to a measured-aircraft dynamic farfield:
  - upstream: `1.5 * x_extent`
  - downstream: `3.0 * x_extent`
  - side Y: `2.0 * y_extent`
  - side Z: `2.0 * z_extent`
  - minimum padding: `0.15 m`
  - farfield mesh size:
    `max(surface_size, clamp(0.15 * max_aircraft_extent, 0.04 m, 0.35 m))`

Aerodynamic force convention:

- Coordinate system: `X` streamwise/longitudinal, `Y` spanwise, `Z` vertical.
- Default freestream: `+X`.
- Drag direction: `+X`.
- Lift direction: `+Z`.
- Pitch axis: `+Y`.
- Older force runs used lift direction `+Y`; those `Cl` values are
  side-force-like and are not comparable to current vertical-lift `Cl`.

## New Gated Stress Variant

Variant:

- Source: `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_spacing_1p0.stl`
- Generated STL:
  `runs/gate_span_stretch_y104_source/aircraft_surface.stl`
- Change: scale `y` by `1.04` about the source bounds center.
- Label: `span_stretch_y104_from_direct_sdf_oml`

Variant creation:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\make_affine_stl_variant.py `
  --input-stl ..\dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_spacing_1p0.stl `
  --output-stl runs\gate_span_stretch_y104_source\aircraft_surface.stl `
  --report runs\gate_span_stretch_y104_source\variant_report.json `
  --scale 1.0,1.04,1.0 `
  --label span_stretch_y104_from_direct_sdf_oml
```

## Gated Run Result

Run folder:
`runs/gate_span_stretch_y104_iso7p2_nosmooth_gmsh_split_alg4`

Source gate:

- `runs/gate_span_stretch_y104_source/source_gate.json`: pass
- `runs/gate_span_stretch_y104_iso7p2_nosmooth/source_gate.json`: pass

Mesh gate:

- `runs/gate_span_stretch_y104_iso7p2_nosmooth_gmsh_split_alg4/mesh_gate.json`: pass

Mesh metrics:

| Metric | Value |
|---|---:|
| Cells | 381,769 |
| Points | 72,402 |
| Aircraft faces | 27,394 |
| Boundary patches | 7 |
| Unexpected patches | 0 |
| `defaultFaces` | 0 |
| Max aspect ratio | 154.66229 |
| Max skewness | 2.0536077 |
| Max non-orthogonality | 86.493216 |
| Severe non-orthogonal faces | 71 |
| SU2 validation | valid |
| OpenFOAM `checkMesh` | Mesh OK |

Solver smoke:

- Case:
  `runs/gate_span_stretch_y104_iso7p2_nosmooth_gmsh_split_alg4/openfoam_case_incompressible_default_60_forces`
- Completed: yes
- Floating point exception: no
- Last time: 60
- Final pressure residual: `4.7550541e-6`
- Final velocity residual: `1.8377939e-5`
- Final `Cd`: `0.0613020113`
- Final `Cl`: `-0.0486151424`
- Final `Cm`: `0.00704703821`
- Force stability gate: pass

## Exported Geometry Variant Result

Variant:

- Source Rhai:
  `dual_contouring/direct_sparse_sdf_mc_experiment/scratch/aircraft_oml_sweep35_root120_tail120_frame.rhai`
- Geometry change:
  - wing sweep: `35 deg`
  - wing root chord: `204 mm` (`+20%`)
  - horizontal tail dimensions: `+20%`
  - vertical tail dimensions: `+20%`
- Initial export:
  `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_sweep35_root120_tail120_spacing_1p0.stl`
  - failed strict exporter topology gate with `430` boundary edges.
  - boundary localization showed clipped loops on hard bbox planes.
- Wider-bbox export:
  `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_sweep35_root120_tail120_bboxwide_spacing_1p0.stl`
  - used `--bbox-min=-32,-224,-256 --bbox-max=736,112,528`
  - reduced boundary edges from `430` to `4`.
- Source prep:
  `runs/sweep35_root120_tail120_source_prep/aircraft_capped.stl`
  - capped one four-vertex loop.
  - cap span: `0.0809,0.0106,1.0099 mm`.
  - final source gate: pass, watertight, one body, zero boundary edges, zero
    nonmanifold edges.

Automatic bbox rerun:

- Command was run through
  `dual_contouring/direct_sparse_sdf_mc_experiment/scripts/optimizer_export_presets.py`
  with no manual bbox.
- Output STL:
  `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_sweep35_root120_tail120_autobbox_spacing_1p0.stl`
- Auto-bbox search:
  `[-128,-512,-384]` to `[1024,512,768]` mm.
- Auto-bbox active bounds before margin:
  `[-16,-176,-240]` to `[688,64,496]` mm.
- Final auto-bbox:
  `[-64,-224,-288]` to `[736,112,544]` mm.
- Result: no clipping recurrence. The export still had the known tiny
  four-edge tile opening, so strict topology gate failed until bounded source
  prep was enabled.

Run folder:
`runs/sweep35_root120_tail120_gated_smoke`

Command:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gated_mesh_smoke.py `
  --input-stl runs\sweep35_root120_tail120_source_prep\aircraft_capped.stl `
  --run-dir runs\sweep35_root120_tail120_gated_smoke
```

The same source-prep step is now available inside the runner as an explicit
opt-in:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gated_mesh_smoke.py `
  --input-stl ..\dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_sweep35_root120_tail120_bboxwide_spacing_1p0.stl `
  --run-dir runs\sweep35_root120_tail120_runner_prep_skip_solver `
  --prep-source-cap-loop-span-mm 2.0 `
  --skip-solver
```

Verification run:
`runs/sweep35_root120_tail120_runner_prep_skip_solver`

- Source prep: pass
- Source gate: pass
- Remeshed source gate: pass
- Gmsh pipeline: pass
- Mesh gate: pass
- Solver: skipped intentionally

Mesh gate:

- `runs/sweep35_root120_tail120_gated_smoke/mesh_gate.json`: pass

Mesh metrics:

| Metric | Value |
|---|---:|
| Cells | 357,474 |
| Points | 69,300 |
| Aircraft faces | 31,106 |
| Boundary patches | 7 |
| Unexpected patches | 0 |
| `defaultFaces` | 0 |
| Max aspect ratio | 160.1055 |
| Max skewness | 2.2681885 |
| Max non-orthogonality | 85.721881 |
| Severe non-orthogonal faces | 143 |
| SU2 validation | valid |
| OpenFOAM `checkMesh` | Mesh OK |

Surface fidelity versus remeshed source:

- Bidirectional p95: `0.0000053 mm`
- Bidirectional p99: `0.0000112 mm`
- Bidirectional max: `0.0744 mm`

Solver smoke:

- Case:
  `runs/sweep35_root120_tail120_gated_smoke/openfoam_case_incompressible_default_60_forces`
- Completed: yes
- Floating point exception: no
- Last time: 60
- Final pressure residual: `4.284199e-6`
- Final velocity residual: `1.3657828e-5`
- Final `Cd`: `0.0660081909`
- Final `Cl`: `0.0010694494`
- Final `Cm`: `-1.75269211e-05`
- Force stability gate: fail, because small lift and moment coefficients drift
  relative to the reference window.
- Scoring status: smoke-test evidence only, not scoring-ready.

Dynamic farfield verification:

- Run folder:
  `runs/sweep35_root120_tail120_autobbox_dynamic_farfield_skip_solver`
- Input STL:
  `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_sweep35_root120_tail120_autobbox_spacing_1p0.stl`
- Source prep: opt-in `--prep-source-cap-loop-span-mm 2.0`.
- Farfield policy:
  - aircraft extent: `0.6762,0.2130,0.7159 m`
  - upstream padding: `1.014 m`
  - downstream padding: `2.029 m`
  - side Y padding: `0.426 m`
  - side Z padding: `1.432 m`
  - farfield mesh size: `0.1074 m`
- Mesh gate: pass.

Dynamic farfield mesh metrics:

| Metric | Value |
|---|---:|
| Cells | 407,749 |
| Points | 79,224 |
| Aircraft faces | 31,126 |
| Boundary patches | 7 |
| Unexpected patches | 0 |
| `defaultFaces` | 0 |
| Max aspect ratio | 163.32878 |
| Max skewness | 1.8671639 |
| Max non-orthogonality | 85.703803 |
| Severe non-orthogonal faces | 116 |
| SU2 validation | valid |
| OpenFOAM `checkMesh` | Mesh OK |

Dynamic farfield solver smoke:

- Case:
  `runs/sweep35_root120_tail120_autobbox_dynamic_farfield_solver/openfoam_case_incompressible_default_60_forces`
- Completed: yes
- Floating point exception: no
- Last time: 60
- Final pressure residual: `3.2774257e-6`
- Final velocity residual: `7.835486e-6`
- Final `Cd`: `0.065960178`
- Final `Cl`: `0.000533424136`
- Final `Cm`: `-9.61068887e-05`
- Force stability gate: fail, because small lift and moment coefficients drift
  relative to the reference window.
- Scoring status: smoke-test evidence only, not scoring-ready.

Exporter-level cap fix and corrected force-axis rerun:

- Export result:
  `dual_contouring/direct_sparse_sdf_mc_experiment/logs/optimizer_export_result_sweep35_root120_tail120_autobbox_capfix.json`
- Exported STL:
  `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_sweep35_root120_tail120_autobbox_capfix_spacing_1p0.stl`
- Export status: pass.
- Export source gate metrics:
  - boundary edges: `0`
  - nonmanifold edges: `0`
  - connected components: `1`
  - duplicate triangles: `0`
  - long chord sections >= 75 mm: `0`
- Exporter cap provenance:
  - tiny boundary loop caps added: `1`
  - loop vertices: `4`
  - span: `0.0809,0.0106,1.0099 mm`
  - perimeter: `2.0290 mm`
  - new faces: `4`
- Gated run:
  `runs/sweep35_root120_tail120_exporter_capfix_dynamic_forces_zlift`
- Mesher-side source prep: not used.
- Force convention:
  - lift direction: `0,0,1`
  - drag direction: `1,0,0`
  - pitch axis: `0,1,0`

Corrected-axis mesh metrics:

| Metric | Value |
|---|---:|
| Cells | 404,723 |
| Points | 78,688 |
| Aircraft faces | 30,964 |
| Boundary patches | 7 |
| Unexpected patches | 0 |
| `defaultFaces` | 0 |
| Max aspect ratio | 161.57141 |
| Max skewness | 2.4848425 |
| Max non-orthogonality | 86.385123 |
| Severe non-orthogonal faces | 121 |
| SU2 validation | valid |
| OpenFOAM `checkMesh` | Mesh OK |

Corrected-axis solver smoke:

- Completed: yes
- Floating point exception: no
- Last time: 60
- Final pressure residual: `3.4496903e-6`
- Final velocity residual: `8.2616699e-6`
- Final `Cd`: `0.065971069`
- Final `Cl`: `-0.113468059`
- Final `Cm`: `0.0267205536`
- Force stability gate: pass for the 60-step smoke window.
- Scoring status: still smoke-test evidence only; final scoring needs an
  angle-of-attack policy, reference-area policy, solver convergence criteria,
  and near-wall treatment.

## Gate Regression Checks

The gates reject known conditional/failing cases:

- Raw root180/tip70 source:
  `runs/repeat_root180_tip70_raw_source_gate.json`
  - fails because surface is not watertight and has 26 boundary edges.
- Repaired root180/tip70 no-Netgen fallback:
  `runs/repeat_root180_tip70_cap10_iso7p2_nosmooth_gmsh_split_alg4_no_netgen/mesh_gate.json`
  - fails because Gmsh optimizers do not match the expected baseline list.
  - fails because OpenFOAM contains a 14-face `defaultFaces` patch.

## Baseline Decision

Promote this path as the current optimizer meshing baseline:

1. Source STL must pass `cfd_mesh_gates.py source`.
2. Remesh with PyMeshLab 7.2 mm no-smooth settings.
3. Run Gmsh Frontal split-farfield with `default,Netgen,Relocate3D`.
4. Mesh result must pass `cfd_mesh_gates.py mesh`.
5. Solver smoke can run after mesh gates pass.
6. Force coefficients are still smoke evidence, not final aerodynamic scoring,
   until longer runs and a real near-wall/boundary-layer strategy are validated.

Cases that require repair, fallback optimizer settings, or produce unexpected
patches are allowed as diagnostics only. They must not be silently promoted to
the clean optimizer baseline.
