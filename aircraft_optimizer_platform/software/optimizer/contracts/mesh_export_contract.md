# CFD Mesh Export Contract

Date created: 2026-06-21

## Purpose

This contract defines the optimizer-facing handoff from geometry/export artifacts to CFD solver-ready volume meshes.

The platform should not treat an OML STL as equivalent to a CFD mesh. STL is a possible surface input to a meshing module. A solver-ready CFD mesh must contain a fluid volume, boundary faces, and boundary identities that the CFD solver can assign boundary conditions to.

## Scope

Initial scope:

- External-flow fixed-wing small UAV cases.
- Low-speed incompressible/inviscid SU2 smoke cases.
- Volume mesh generation for candidate screening and solver integration tests.

Still out of scope for scoring CFD:

- RANS wall-function/y-plus policy.
- Moving meshes.
- Overset meshes.
- Internal ducts/inlets.
- Multi-zone conjugate heat-transfer meshes.

## Accepted Mesh Formats

Initial accepted formats:

- Native SU2 mesh: `.su2`
- CGNS mesh: `.cgns`
- OpenFOAM case directory: `constant/polyMesh` plus required `0`, `constant`,
  and `system` dictionaries

Native SU2 is the preferred early integration format when marker names are preserved clearly.

CGNS is acceptable when the adapter records an explicit marker map from platform semantic names to solver-visible names.

OpenFOAM case directories are accepted for the Gmsh/OpenFOAM mesher-plumbing
path when the adapter records the original Gmsh mesh, conversion logs, boundary
patch map, strict `checkMesh` result, and solver-smoke result.

## Required Semantic Regions

Every external-flow CFD mesh must define these semantic regions:

```yaml
fluid:
  kind: volume
  required: true
aircraft:
  kind: surface
  required: true
  default_boundary_condition: euler_wall
farfield:
  kind: surface
  required: true
  default_boundary_condition: farfield
```

The semantic names above are the platform contract. Solver-visible names may differ if a mesh writer or format changes them.

## Marker Naming Policy

Do not expose arbitrary backend section names as the platform-level contract.

Stable platform names:

- `fluid`
- `aircraft`
- `farfield`

Backend names are adapter details.

Example native SU2 marker map:

```yaml
semantic_to_solver_markers:
  aircraft:
    - aircraft
  farfield:
    - farfield
  fluid:
    - fluid
```

Example CGNS marker map from the sandbox Gmsh export:

```yaml
semantic_to_solver_markers:
  aircraft:
    - 3_S_7
  farfield:
    - 3_S_8
    - 3_S_9
    - 3_S_10
    - 3_S_11
    - 3_S_12
    - 3_S_13
  fluid:
    - 5_V_1
```

The CFD case builder must generate solver configs from this mapping rather than assuming `aircraft` and `farfield` are always the solver-visible names.

## Required Mesh Result Record

```yaml
mesh_result_schema_version: 0.1.0
mesh_result_id: string
candidate_id: string
evaluation_id: string
source_geometry_artifact_id: string | null
source_surface_artifact_id: string | null
mesh_module_attempt_id: string
mesh_format: su2 | cgns | openfoam_case
mesh_artifact_id: string
mesh_path: string
case_units:
  length: m
  velocity: m/s
coordinate_frame:
  x: forward
  y: right
  z: up
semantic_to_solver_markers:
  fluid:
    - string
  aircraft:
    - string
  farfield:
    - string
cell_counts:
  nodes: integer
  volume_cells: integer
  surface_faces: integer
  tetrahedra: integer | null
  hexahedra: integer | null
  prisms: integer | null
  pyramids: integer | null
boundary_counts:
  aircraft_faces: integer
  farfield_faces: integer
quality:
  min_quality: number | null
  max_skewness: number | null
  max_non_orthogonality: number | null
  severe_non_orthogonal_faces: integer | null
  invalid_surface_elements: integer
  invalid_volume_elements: integer
status: success | failed
warnings:
  - string
failure: failure_record | null
```

## Minimum Acceptance Gates

The first solver-ready mesh gate must require:

- `mesh_format` is `su2`, `cgns`, or `openfoam_case`.
- `nodes > 0`.
- `volume_cells > 0`.
- `aircraft_faces > 0`.
- `farfield_faces > 0`.
- `semantic_to_solver_markers.aircraft` is not empty.
- `semantic_to_solver_markers.farfield` is not empty.
- Solver config can be rendered using the marker map.
- The target solver/toolchain can read the mesh far enough to report expected
  volume-cell count and marker/patch list.

For native SU2, the file should contain:

```text
NDIME= 3
NELEM= <positive integer>
NPOIN= <positive integer>
NMARK= <positive integer>
MARKER_TAG= <solver-visible aircraft marker>
MARKER_TAG= <solver-visible farfield marker>
```

For CGNS, the adapter must parse or capture the solver-visible section names that SU2 will use.

For OpenFOAM, the case should include:

```text
constant/polyMesh/boundary
log.gmshToFoam
log.checkMesh
```

and the adapter should preserve:

- OpenFOAM version.
- `gmshToFoam` command.
- `checkMesh` command and strict verdict.
- patch names and face counts.
- tetrahedra/prism/hex/pyramid counts when available.
- max skewness, max non-orthogonality, severe non-orthogonal face count.
- `potentialFoam -writep` smoke status for current plumbing runs.

## Boundary-Layer / Prism-Layer Extension

Boundary-layer meshes are now allowed as a non-scoring OpenFOAM plumbing
extension.

Current preferred OpenFOAM plumbing backend:

```text
software/optimizer/configs/hybrid_prism_gmsh_openfoam.v0_1.json
```

This preset uses a custom direct body-fitted prism shell near the aircraft,
generates a Gmsh tetrahedral external farfield volume from the shell outer
surface, converts the Gmsh volume with `gmshToFoam`, and then uses a custom
exact OpenFOAM polyMesh merge at the shared shell/outer-volume interface.

Evidence root:

```text
custom_cfd_mesher_experiment/runs/hybrid_shell_gmsh_exact_merge_five_variant_netgen_v0_1_20260624
```

The hybrid preset passed strict OpenFOAM `checkMesh` and completed
`potentialFoam -writep` on all five approved faired-cap variants. The mesh
contains three direct prism layers at cumulative offsets `0.2/0.5/1.0 mm` and a
Gmsh tetrahedral farfield volume. OpenFOAM `mergeMeshes` plus `stitchMesh` was
rejected for this path because it left small unmatched interface patches and
produced invalid merged meshes; the exact merge utility is required.

The latest bounded no-slip steady OpenFOAM probe on this hybrid mesh failed:
`foamRun -solver incompressibleFluid` diverged by time step 2 and hit a
pressure-solver floating point exception. This does not invalidate the mesher
as plumbing evidence, but it blocks scoring CFD.

Historical Gmsh BL development preset:

```text
software/optimizer/configs/gmsh_openfoam_external_aero_bl.v0_2.json
```

This preset uses Gmsh `create-topology` with one prism layer and has passed
strict OpenFOAM `checkMesh` plus `potentialFoam -writep` on the five approved
faired-cap variants. It is no longer the preferred OpenFOAM plumbing backend
now that the exact-merged hybrid shell path exists, but it remains useful
comparison evidence.

One prism layer is a non-scoring plumbing baseline, not the final wall model.
Naive Gmsh multi-layer extrusion is not yet acceptable: bounded fcv04 tests with
two and three Gmsh-extruded layers failed strict `checkMesh` with local
high-aspect cells and wrong-oriented faces. The current Gmsh backend should
remain the default full external-volume plumbing mesh, but not the trusted
boundary-layer solution.

The current leading boundary-layer research path is a custom direct prism shell:

```text
software/optimizer/configs/direct_prism_shell_boundary_layer.v0_1.json
```

The direct shell remains the reusable near-wall component inside the preferred
hybrid preset. Standalone shell cases are not complete CFD domains because the
outer patch is only an offset aircraft shell.

This does not make the mesh scoring-ready. A result must still remain
`scoring_allowed=false` until the platform has:

- stable no-slip OpenFOAM solver convergence.
- explicit wall-treatment and y-plus policy.
- force and moment coefficient setup.
- coefficient stability/convergence checks.
- evidence that the prism-layer strategy supports meaningful drag/lift.

## Surface Input Is Not Sufficient

Passing surface checks such as:

- `boundary_edges = 0`
- `nonmanifold_edges = 0`
- `connected_components = 1`
- `duplicate_triangles = 0`

is necessary but not sufficient.

A surface can pass these checks and still fail target volume meshing due to:

- invalid reparametrized patches
- overlapping or nearly self-intersecting facets
- poor local triangle quality
- ambiguous inner-boundary orientation
- failed boundary recovery
- target mesher limitations with imported STL

The mesh gate must validate the produced volume mesh, not just the input surface.

## Failure Categories

Recommended failure categories:

```yaml
meshing.surface_import_failed
meshing.surface_marker_classification_failed
meshing.volume_generation_failed
meshing.no_volume_cells
meshing.no_nodes
meshing.missing_aircraft_marker
meshing.missing_farfield_marker
meshing.invalid_surface_elements
meshing.invalid_volume_elements
meshing.backend_marker_mapping_failed
meshing.solver_preflight_failed
```

## Sandbox Evidence

The SU2 sandbox produced these observations:

- A clean 75 mm sphere STL could be converted through Gmsh to native `.su2`, and SU2 read the `aircraft` and `farfield` markers.
- Simple manifold wing STLs passed edge checks but Gmsh STL-import volume meshing produced `NELEM=0` / `NPOIN=0`.
- A native Gmsh/OCC lenticular wing produced a valid native `.su2` volume mesh with `aircraft` and `farfield` markers and converged in SU2.
- The same native Gmsh/OCC wing exported as CGNS could be read by SU2, but Gmsh exposed solver-visible section names such as `3_S_7` instead of friendly physical names.

Therefore, the platform should support both native SU2 and CGNS while treating marker mapping as explicit adapter metadata.

## Initial Integration Rule

Until a real mesh exporter exists, do not let a candidate advance to real CFD solely because an STL export passed topology gates.

The candidate must pass a volume-mesh preflight that produces a mesh result record conforming to this contract.
