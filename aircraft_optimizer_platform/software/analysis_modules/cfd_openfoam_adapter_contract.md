# OpenFOAM Adapter Contract

Status: v0.1 sandbox contract.

This contract defines the first optimizer-facing OpenFOAM path for repaired STL-based full-aircraft CFD smoke tests. It is not yet a production aerodynamic scoring contract.

## Scope

The v0.1 OpenFOAM path is:

```text
OML STL export
-> CFD surface quality gate
-> optional surface repair/remesh
-> snappyHexMesh case generation
-> checkMesh gate
-> potentialFoam smoke solve
-> OpenFOAM result validation
-> artifacts and metrics
```

The current solver is `potentialFoam`. It proves mesh and solver plumbing only. It does not provide viscous drag, validated lift/drag, stability derivatives, or force-based scoring.

The platform currently supports two OpenFOAM-facing operations:

- `check-openfoam-result`: validate an existing case/log set and print a JSON result.
- `persist-openfoam-smoke`: persist an existing case/log set into the optimizer database as `openfoam_smoke_validation`.

Neither operation launches OpenFOAM. Solver execution remains a sandbox/manual step until the execution adapter is reviewed.

## Required Boundary Semantics

Platform-facing boundary semantics remain stable:

- `aircraft`: aircraft outer mold line wall/slip boundary.
- `farfield`: external flow domain boundary.
- `fluid`: volume mesh region.

OpenFOAM case generation may store those semantics as patch names directly:

```text
aircraft
farfield
```

## Current Case Template

The sandbox template writes:

- `constant/geometry/aircraft.stl`
- `system/blockMeshDict`
- `system/surfaceFeaturesDict`
- `system/snappyHexMeshDict`
- `system/controlDict`
- `0/U`
- `0/p`

The current `potentialFoam` smoke boundary setup is:

- `farfield` velocity: fixed freestream.
- `farfield` pressure: fixed value `0`.
- `aircraft` velocity: `slip`.
- `aircraft` pressure: `zeroGradient`.

The current smoke velocity is `50 mph` / `22.352 m/s` in +X.

## Required Artifacts

A complete OpenFOAM smoke attempt should preserve:

- input STL.
- repaired/remeshed STL if repair was used.
- raw surface quality JSON.
- repaired surface quality JSON if repair was used.
- repair report.
- OpenFOAM case directory.
- `log.blockMesh`.
- `log.surfaceFeatures`.
- `log.snappyHexMesh`.
- `log.checkMesh`.
- `log.potentialFoam`.
- `log.foamToVTK_potential`.
- `potential_case_report.json`.
- `openfoam_result_validation.json`.
- VTK output directory.
- aircraft-only ISO screenshot, normally `aircraft_iso.png`, generated from the `aircraft` patch only.

## Acceptance Gates

Surface gate:

- `boundary_edges = 0`
- `nonmanifold_edges = 0`
- `duplicate_triangles = 0`
- `degenerate_triangles = 0`
- triangle and vertex counts under configured limits
- minimum triangle quality and edge length above configured limits

OpenFOAM mesh/solver gate:

- `checkMesh` reports `Mesh OK`.
- cells exceed the configured minimum.
- `aircraft` patch has positive face count.
- `farfield` patch has positive face count.
- max non-orthogonality is below policy limit.
- max skewness is below policy limit.
- `potentialFoam` reaches `End`.
- continuity error is below policy limit.
- interpolated velocity error is below policy limit.

Development relaxed mesh/solver gate:

- This mode is allowed only for smoke CFD, solver plumbing, and optimizer integration tests.
- Run `checkMesh -skewThreshold 6` and preserve the relaxed log separately, normally as `log.checkMesh_relaxed_skew6`.
- Preserve the strict default `log.checkMesh` result even when relaxed mode passes.
- Record acceptance mode as `relaxed_development`.
- Current relaxed skew limit is `max_skewness <= 6.0`.
- A relaxed-development pass must still satisfy non-orthogonality, cell-count, patch-count, solver-completion, continuity-error, and interpolated-velocity-error limits.
- Relaxed-development results are not scoring-quality CFD and must not be used as final aerodynamic truth.

## Optimizer Persistence

When a smoke case is recorded in the optimizer database, the module attempt should use:

- module name: `openfoam_smoke_validation`
- module kind: `analysis`
- acceptance mode: `strict` or `relaxed_development`
- scoring allowed: `false`
- analysis role: `solver_smoke_non_scoring`

The persisted metrics should include OpenFOAM readiness, mesh status, cell count, patch face counts, max non-orthogonality, max skewness, continuity error, and interpolated velocity error when available.

The persisted artifacts should include:

- `openfoam_smoke_result_json`
- `openfoam_case_dir`
- `openfoam_checkmesh_log`
- `openfoam_solver_log`
- `openfoam_strict_checkmesh_log` when available
- `openfoam_relaxed_checkmesh_log` when available
- `aircraft_iso_screenshot` when available

## Current Limitations

- `potentialFoam` is inviscid and not a production scoring solver.
- The current case has one combined `farfield` patch, not separate inlet/outlet/sides.
- There is no force coefficient reporting yet.
- Surface repair can make a candidate meshable while also changing the exact surface; the repair report must be preserved.
- Very small repaired edges may be tolerated by `snappyHexMesh`, but they should remain visible as warnings until the export surface improves.
- Every mesh run should produce an aircraft-only ISO screenshot for human review. The screenshot must not include the farfield box or full volume mesh.

## Next Contract Step

The next OpenFOAM contract should add a controlled incompressible external-flow solver case with force reporting, likely `simpleFoam` or an appropriate laminar/turbulence setup after user review.
