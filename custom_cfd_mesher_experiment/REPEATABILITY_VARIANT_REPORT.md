# Clean Mesh Repeatability Variant Report

Verdict: CONDITIONAL PASS.

The clean mesh process is repeatable on watertight no-inlet aircraft variants
and produced stable 60-step OpenFOAM smoke results on two additional usable
cases. It is not fully robust to open source STLs or to every Gmsh optimizer
combination.

## Fixed Recipe Under Test

Surface remesh:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\remesh_aircraft_surface_pymeshlab.py `
  --target-edge-mm 7.2 `
  --iterations 5 `
  --feature-deg 25 `
  --max-surface-distance-mm 0.25 `
  --samples 12000 `
  --no-smooth
```

Gmsh/OpenFOAM mesh:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\run_gmsh_cfd_pipeline.py `
  --target-faces 0 `
  --surface-size 0.0072 `
  --farfield-size 0.11 `
  --padding 0.7,0.5,0.5 `
  --algorithm3d 4 `
  --optimize default,Netgen,Relocate3D `
  --fidelity-samples 8000 `
  --farfield-patches split
```

Solver smoke:

```powershell
..\su2_sandbox\.venv\Scripts\python scripts\setup_incompressible_fluid_smoke.py `
  --farfield-mode split `
  --end-time 60 `
  --write-interval 20 `
  --force-coeffs `
  --rho-inf 1.225 `
  --mag-u-inf 22.352 `
  --l-ref 0.7111267 `
  --a-ref 0.120 `
  --cofr 0.338,0,0 `
  --lift-dir 0,1,0 `
  --drag-dir 1,0,0 `
  --pitch-axis 0,0,1 `
  --force-write-interval 1
```

## Variant Results

| Variant | Source | Mesh Result | Solver Result | Force Gate | Notes |
|---|---|---|---|---|---|
| Span +20 clean case | `runs/wing_span_plus20_v3_transition/aircraft_surface.stl` | PASS | PASS | FAIL by small Cl drift | Original clean-mesh baseline. |
| Direct SDF OML baseline | `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_spacing_1p0.stl` | PASS | PASS | PASS | Best repeatability result. |
| Root 180 / Tip 70 raw | `dual_contouring/direct_sparse_sdf_mc_experiment/stl/basic_no_inlet_root180_tip70_spacing_1p0_spacing_1p0.stl` | FAIL | Not run | Not run | Raw source STL has open boundary edges; exact Gmsh run produced no volume elements. |
| Root 180 / Tip 70 repaired | `runs/repeat_root180_tip70_prepared_cap10/aircraft_surface.stl` | CONDITIONAL PASS | PASS | PASS | Required one 10 mm cap and skipping Netgen optimizer; OpenFOAM created 14 `defaultFaces`. |

## Metrics

| Metric | Span +20 clean baseline | Direct SDF OML baseline | Root 180 / Tip 70 repaired fallback |
|---|---:|---:|---:|
| Run folder | `runs/cleanmesh_surface_iso_7p2mm_nosmooth_gmsh_split_alg4` | `runs/repeat_direct_sdf_oml_iso7p2_nosmooth_gmsh_split_alg4` | `runs/repeat_root180_tip70_cap10_iso7p2_nosmooth_gmsh_split_alg4_no_netgen` |
| Source watertight after remesh | yes | yes | yes |
| Gmsh optimizers | default, Netgen, Relocate3D | default, Netgen, Relocate3D | default, Relocate3D |
| OpenFOAM `checkMesh` | OK | OK | OK |
| Cells | 379,086 | 377,033 | 506,281 |
| Points | 72,032 | 71,552 | 87,230 |
| Aircraft faces | 27,510 | 27,168 | 27,796 |
| Boundary patches | 7 | 7 | 8 |
| Unexpected `defaultFaces` | 0 | 0 | 14 |
| Max aspect ratio | 106.12576 | 150.00852 | 183.39312 |
| Max skewness | 3.8317653 | 2.4138457 | 2.4080312 |
| Max non-orthogonality | 86.687852 | 84.661825 | 88.252983 |
| Severe non-orthogonal faces | 69 | 72 | 158 |
| SU2 validation | valid | valid | valid |
| Solver completed | yes | yes | yes |
| Floating point exception | no | no | no |
| Final p residual | `1.7801072e-5` | `8.1336951e-6` | `7.4498467e-6` |
| Final U residual | `7.2476491e-5` | `1.8491511e-5` | `2.1747202e-5` |
| Final Cd | `0.076177125` | `0.0584683521` | `0.0625491094` |
| Final Cl | `-0.0667940004` | `-0.0459971506` | `-0.047301988` |
| Final Cm | `0.00811865743` | `0.00698267831` | `0.00589990799` |
| Force stability gate | fail | pass | pass |

## Failure Details

### Raw Root 180 / Tip 70

The raw root/tip source STL had 26 boundary edges. PyMeshLab remeshing reduced
that to 8 boundary edges but did not make the surface watertight. Gmsh produced
zero volume elements and the SU2 validation failed:

- `bad_node_references`: 31,064
- `points`: 0
- `volume_elements`: 0
- `valid`: false

This should be caught before meshing by a hard source-surface gate:

- `boundary_edges == 0`
- `watertight == true`
- `body_count == 1`

### Repaired Root 180 / Tip 70

The repair step capped one 26-vertex loop with span:

- X: 9.475891 mm
- Y: 3.285915 mm
- Z: 0.0 mm

After the 10 mm cap, the surface was watertight. The exact preferred Gmsh
optimizer list still failed during Netgen optimization:

```text
Warning : 6 ill-shaped tets are still in the mesh
Error   : Tetrahedron with unknown node - should never be here!
```

The fallback without Netgen succeeded and the solver was stable, but the mesh
created 14 OpenFOAM `defaultFaces`, so this is not a fully clean pass.

## Interpretation

The current path is no longer a one-off success. It works on at least two
watertight no-inlet aircraft surfaces with stable 60-step OpenFOAM behavior.

The root/tip experiment exposed two important acceptance gates:

1. Raw exporter STL closure matters. Open surfaces can fail before volume
   meshing or produce invalid SU2 files.
2. Netgen optimization is not universally robust. It improves some cases but
   can fail on a geometry that base Gmsh Frontal can otherwise mesh.

Near-term recommendation:

- Keep the PyMeshLab 7.2 mm no-smooth remesh plus Gmsh Frontal split-farfield
  path as the current baseline.
- Add a pre-Gmsh surface gate for watertightness, boundary edges, body count,
  and loop repair provenance.
- Add an automated fallback policy: if `default,Netgen,Relocate3D` fails, try
  `default,Relocate3D`, but mark the result `conditional` unless there are no
  unexpected OpenFOAM patches and strict quality still passes.
