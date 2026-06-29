# CFD Geometry Design Rules

Status: v0.1 development contract.

Purpose: define geometry and exported-surface rules that keep optimizer
candidates inside a CFD-compatible design space. These rules are separate from
mesher quality checks. The optimizer should reject or repair CFD-hostile
geometry before spending time on volume meshing or solver runs.

## Principle

A valid STL is not automatically a CFD-ready aircraft.

The platform should optimize inside a geometry envelope that is compatible with
automated meshing. Candidates that violate this envelope should be classified
as geometry or export failures, not CFD failures.

## Coordinate Convention

- `X`: fuselage length / freestream / drag axis.
- `Y`: span axis.
- `Z`: vertical / lift axis.
- Length units in geometry definitions and STL audits are millimeters unless
  explicitly converted.
- Solver case units are meters.

## Pre-Definition Rules

These rules should be checked from design variables before geometry generation.

| Rule | Initial Policy | Failure Category |
|---|---:|---|
| Minimum wing tip chord | campaign-defined, initially `>= 40 mm` | `geometry.tip_chord_too_small` |
| Minimum tail tip chord | campaign-defined, initially `>= 35 mm` | `geometry.tail_tip_chord_too_small` |
| Maximum wing sweep | campaign-defined, initially `<= 45 deg` | `geometry.sweep_out_of_bounds` |
| Maximum tail sweep | campaign-defined, initially `<= 55 deg` | `geometry.tail_sweep_out_of_bounds` |
| Minimum taper ratio | initially `>= 0.25` | `geometry.taper_too_aggressive` |
| Maximum aspect ratio for early CFD | initially `<= 12` | `geometry.aspect_ratio_too_high_for_cfd` |
| Minimum feature spacing | campaign-defined from mesher target size | `geometry.feature_spacing_too_small` |

These are initial development limits. They can be loosened after the mesher and
solver prove repeatable over a wider design space.

## Geometry-Definition Rules

These rules should be checked after the parameterized geometry definition is
assembled and before STL export.

### Trailing Edges

Trailing edges must not be mathematically razor sharp for automated CFD.

Initial policy:

- Minimum physical trailing-edge thickness: `>= 1.0 mm`.
- Preferred trailing-edge thickness for early automated CFD: `1.5-3.0 mm`.
- Minimum trailing-edge radius or bevel equivalent: `>= 0.5 mm`.
- Avoid long zero-thickness line intersections at wing or tail trailing edges.

Reason:

Razor trailing edges create sliver triangles and high non-orthogonality clusters
in the first volume cells. This is currently one of the dominant Gmsh BL risk
areas.

### Leading Edges

Leading edges must be smooth enough to support repeatable surface remeshing.

Initial policy:

- Minimum leading-edge radius: `>= 1.0 mm`.
- No local LE radius smaller than the planned first cell height by less than a
  factor of `20`.
- No kinked LE station transitions unless explicitly intended and locally
  refined.

### Wing/Fuselage and Tail/Fuselage Blends

Blends should be large enough for the target surface edge length.

Initial policy:

- Minimum blend radius: `>= 2.5 * target_surface_edge_length`.
- For the current 4 mm Gmsh BL preset, preferred blend radius is `>= 10 mm`.
- No narrow saddle or cusp features at the wing root, tail root, inlet cap, or
  vertical tail base.

### Inlets and Faired Caps

The faired-cap inlet body is allowed, but it must remain meshable.

Initial policy:

- Minimum inlet lip radius: `>= 1.0 mm`.
- Minimum cap fairing radius: `>= 2.0 mm`.
- Minimum inlet-to-wing or inlet-to-tail clearance: `>= 2 * target_surface_edge_length`.
- No hidden self-intersections or buried zero-thickness seams.

### Minimum Local Feature Size

Any surface feature smaller than the local surface mesh target should either be:

- removed from the CFD OML,
- smoothed into a larger blend,
- explicitly marked for local refinement,
- or treated as a geometry rejection.

Initial policy:

- Minimum CFD feature size: `>= 2 * target_surface_edge_length`.
- For current 4 mm Gmsh BL development meshes: `>= 8 mm`.

## Post-Export Surface Rules

These run on the exported STL or remeshed surface before volume meshing.

Hard fail:

- Boundary edges greater than `0`.
- Nonmanifold edges greater than `0`.
- Duplicate triangles greater than `0`.
- Connected components not matching campaign policy.
- Degenerate triangles greater than `0`.

Development fail or repair required:

- Triangle minimum angle below `2.5 deg`.
- Triangle quality below `0.05`.
- Triangle aspect ratio above `60`.
- Minimum triangle altitude below `0.02 mm`.
- Clusters of bad triangles at trailing edge, wing root, inlet cap, or tail
  root.

The current `bl_surface_gate.py` should remain diagnostic until the exporter
can consistently avoid these failures. Once exporter geometry rules are wired,
these surface conditions should become hard pre-mesh gates.

## Boundary-Layer Mesh Rules

The one-layer Gmsh BL mesh is acceptable for non-scoring plumbing, but not final
scoring CFD.

Near-term target for better viscous development meshes:

- At least `3` prism layers for laminar/no-slip development.
- At least `5-10` prism layers before RANS scoring attempts.
- Growth ratio target: `1.2-1.35`.
- First layer height derived from freestream, length scale, viscosity, and y+
  target.
- Total prism-layer thickness limited to avoid crossing narrow feature gaps.
- Fail if prism layers collapse, invert, or create worse strict `checkMesh`
  results.

Current evidence:

- Gmsh one-layer external meshes are the default full-domain plumbing path.
- Gmsh two/three-layer extrusion on fcv04 failed strict `checkMesh`; do not
  promote naive Gmsh multi-layer extrusion as the default.
- A high-fidelity custom direct-prism shell on fcv04 passed strict `checkMesh`
  with three layers, but it is only a near-wall shell. It needs a conformal
  transition to a real farfield domain before solver use.

## Solver-Readiness Rules

Before a no-slip solver run:

- Strict OpenFOAM `checkMesh` must pass.
- Aircraft/farfield patch names must be present.
- `potentialFoam -writep` must complete.
- Max skewness should remain below OpenFOAM strict limit.
- Max non-orthogonality above `85 deg` is allowed only as development evidence,
  not scoring CFD.
- Severe non-orthogonality clusters must be localized and classified.

Before scoring CFD:

- No-slip steady solver must complete.
- Force coefficients must pass a stability window.
- y+ must be reported.
- Wall treatment must be explicitly declared.
- Mesh must have an accepted prism-layer policy.
- Result remains `scoring_allowed=false` until all scoring gates pass.

## Optimizer Behavior

The optimizer should not treat all CFD failures equally.

Recommended failure classes:

- `geometry.cfd_design_rule_violation`
- `geometry.trailing_edge_too_sharp`
- `geometry.blend_radius_too_small`
- `geometry.local_feature_too_small`
- `export.surface_quality_failed`
- `meshing.boundary_layer_generation_failed`
- `meshing.strict_checkmesh_failed`
- `solver.potential_smoke_failed`
- `solver.no_slip_development_failed`
- `solver.coefficients_unstable`

The optimizer should learn from these categories. Repeated failures in a design
region should bias future proposals away from that region before meshing.
