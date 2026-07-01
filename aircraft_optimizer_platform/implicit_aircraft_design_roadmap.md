# Implicit Aircraft Design Roadmap

Date: 2026-06-30

## Position

This roadmap treats the implicit geometry kernel as the product. CFD,
optimization, AI, manufacturing, meshing, and visualization are clients of the
same parameter-driven geometry source of truth.

This document is an overlay on `roadmap.md`, not a replacement for the existing
optimizer-platform roadmap or the run evidence already recorded there. The
current optimizer and OpenFOAM/snappyHexMesh work remains valid production-path
evidence. This roadmap adds the longer-term kernel, SDF conditioning, geometry
query, CFD-data, and AI-surrogate sequence.

## Design Space

Initial target family:

- Fixed-wing UAVs.
- About 0.7 m to 2.0 m wingspan.
- Less than 150 mph, primarily less than 75 mph.
- Fully implicit, parameter-driven geometry.
- Automatic optimization.
- Future AI-assisted aerodynamic evaluation.

Explicit non-goal:

- Do not build a new CFD solver as the near-term production path.

Core rule:

- Parameters define aircraft.
- The implicit kernel is canonical.
- Meshes, STLs, OpenFOAM cases, dashboards, and AI datasets are disposable or
  derived artifacts.

## Long-Term Architecture

```text
Aircraft parameters
  -> Implicit geometry kernel
  -> Automatic local SDF conditioning
  -> Geometry query API
  -> Clients
       - STL export
       - OpenFOAM
       - Optimizer
       - PhysicsNeMo or other surrogate models
       - Manufacturing
       - Structural analysis
       - Visualization
       - Future SDF-native CFD research
```

## Phase 0: Research Gate

Goal:

- Complete a focused literature review before implementing conditioning or AI
  infrastructure.

Required research scope:

- SDF reinitialization, fast marching, fast sweeping, narrow bands, incremental
  updates, and dynamic SDF maintenance.
- Level-set local reinitialization, adaptive reconditioning, and gradient
  correction.
- Immersed-boundary, ghost-cell, cut-cell, and Cartesian-grid CFD.
- Existing software such as OpenVDB/NanoVDB, scikit-fmm, Voxblox/nvblox,
  Lethe, AMReX embedded boundaries, Basilisk embedded boundaries, and
  PhysicsNeMo.
- Adjacent fields including robotics, graphics, GPU ray tracing, collision
  detection, medical imaging, manufacturing, and voxel engines.

Current deliverable:

- `research/phase0_sdf_conditioning_literature_review.md`

Gate rule:

- Conditioning design may begin only after the review identifies which
  algorithms should be borrowed, integrated, prototyped, or avoided.
- AI-surrogate implementation remains blocked until optimizer automation and CFD
  data quality are trusted.

## Phase 1: Conditioned SDF Foundation

Goal:

- Upgrade the composed implicit field into a continuously reliable geometric
  representation.

Current problem:

- The composed SDF is useful for modeling, but offsets, smooth Booleans, shells,
  and blends can make it no longer behave as a true signed-distance field.

Required properties:

- Preserve watertightness.
- Preserve topology unless a topology-changing operation is explicit.
- Improve gradient quality.
- Minimize distance error.
- Preserve normals.
- Support asynchronous updates.
- Support local updates.
- Avoid full-aircraft recomputation whenever practical.

Recommended architecture:

- Each modeling operation emits a dirty region in native aircraft coordinates.
- Dirty regions expand by operation support radius, blend width, offset/shell
  width, grid stencil radius, and a merge halo.
- A background conditioning worker reinitializes only dirty blocks plus halo.
- Conditioned blocks carry version, source operation, grid spacing, residual
  metrics, and confidence metadata.
- Full-field reconditioning remains an explicit fallback, not the default edit
  path.

Permanent diagnostics:

- Gradient magnitude.
- Projection residual.
- Signed-distance error estimate.
- Conditioning confidence.
- Curvature.
- Local feature size.
- Dirty regions.
- Conditioning iteration count.
- Local topology summary.

Regression gates:

- Watertight export remains mandatory for CFD-ready artifacts.
- Shell integrity, blend integrity, offset correctness, and topology stability
  are regression tests.
- Small parameter changes should not introduce cracks or unexpected topology
  changes.

## Phase 2: Geometry Query API

Goal:

- Expose the kernel as a geometry service.

Initial queries:

- `distance()`
- `gradient()`
- `project_to_surface()`
- `surface_normal()`
- `closest_surface_point()`
- `ray_intersection()`
- `inside_outside()`
- `local_feature_size()`
- `wall_thickness()`
- `curvature()`
- `principal_curvature()`
- `cross_section()`
- `volume()`
- `wetted_area()`
- `frontal_area()`

Future metadata:

- Parameter ownership.
- Feature ownership.
- Component hierarchy.
- Manufacturing metadata.

Contract boundary:

- Optimizer, exporter, CFD, manufacturing, and visualization clients call the
  same query API rather than reconstructing aircraft geometry independently.

## Phase 3: Optimizer MVP

Goal:

- Continue the current optimizer path, using low-fidelity and rough-CFD signals
  to validate unattended optimization before building AI data infrastructure.

Primary flow:

```text
Parameters
  -> implicit geometry
  -> local SDF conditioning
  -> mesh/export artifact
  -> OpenFOAM or lower-fidelity analysis
  -> objective evaluation
  -> optimizer
  -> next candidate
```

Rules:

- Keep `direct_sparse_oml_fast` as the normal OML export path.
- Keep `direct_sparse_oml_high_fidelity` as the selected-design convergence or
  finalization rerun.
- Preserve geometry definition and STL/export validation as separate contracts.
- Run low-fidelity/rough-CFD first to validate optimizer automation, not to
  generate AI-quality training labels.

Success:

- Hundreds or thousands of candidate aircraft can run unattended with traceable
  failures, artifacts, lineage, and scores.

## Phase 4: High-Fidelity CFD Dataset

Goal:

- Once optimizer automation is stable, raise simulation fidelity and turn every
  qualified run into durable training data.

Standardize:

- Meshing.
- Solver settings.
- Turbulence model.
- Convergence criteria.
- Post-processing.
- Coordinate and force conventions.

Store for each sample:

- Geometry parameter vector.
- Geometry hash.
- Kernel version.
- Conditioning version.
- Export preset and validation metrics.
- Flight condition: angle of attack, velocity, Reynolds number.
- Outputs: CL, CD, CM, L/D, Cp, pressure distribution, residual history, and
  convergence metrics.

Rule:

- Failed and marginal runs are kept with labels. They are useful for gates,
  uncertainty, and failure prediction.

## Phase 5: AI Surrogate

Goal:

- Build a fast aerodynamic filter only after CFD quality and dataset schema are
  trusted.

Initial outputs:

- CL.
- CD.
- CM.
- L/D.
- Uncertainty estimate.

Later outputs:

- Pressure distribution.
- Separation likelihood.
- Optimization guidance.

Intended workflow:

```text
Generate many aircraft
  -> surrogate ranks/filter candidates
  -> top candidates proceed to CFD
  -> final few receive highest-fidelity validation
```

Rules:

- The surrogate is not the source of truth.
- The surrogate does not replace CFD.
- Uncertainty and applicability-domain metadata are mandatory.
- Direct SDF input is allowed as a research path, but only after the dataset
  baseline is stable.

## Phase 6: Advanced Geometry and CFD Research

Research topics:

- SDF-native CFD.
- Lethe and immersed-boundary methods.
- Cut-cell and Cartesian-grid CFD.
- Differentiable simulation.
- Differentiable geometry.
- Structural analysis.
- Manufacturing analysis.

Rule:

- None of these replace the production workflow until they demonstrate measured
  benefit in iteration time, robustness, or capability.

## Phase 7: Continuous Geometry Platform

Goal:

- Investigate replacing derived mesh workflows with direct conditioned geometry
  queries where practical.

Potential clients:

- PhysicsNeMo or other AI surrogate models.
- SDF-native CFD.
- Collision detection.
- Manufacturing analysis.
- Structural analysis.
- Ray tracing.
- Visualization.
- Geometry-aware optimizer gates.

Rule:

- The objective is not to eliminate meshes.
- The objective is to make meshes optional whenever downstream tools can consume
  the conditioned geometry directly.
- The canonical geometry graph remains the source of truth.
- The conditioned geometry cache remains derived, disposable, rebuildable, and
  versioned.
- Mesh-based production workflows remain available until direct-query workflows
  are validated against them.

## Immediate Next Actions

1. Treat `research/phase0_sdf_conditioning_literature_review.md` as the current
   Phase 0 recommendation.
2. Draft the dirty-region and conditioning contracts before changing runtime
   geometry behavior.
3. Prototype localized reconditioning behind diagnostics and regression gates.
4. Continue the existing optimizer/OpenFOAM path as the production iteration
   route.
5. Define the long-term CFD dataset schema before scaling high-fidelity runs.
6. Delay PhysicsNeMo or other surrogate work until optimizer automation,
   conditioning stability, and CFD data quality are established.
7. Treat the conditioned geometry cache as the formal derived representation
   between the canonical graph and downstream clients.

## Guiding Principles

1. The implicit kernel is the product.
2. Meshes are disposable artifacts.
3. Parameters define aircraft, not triangles.
4. Local updates should replace global recomputation whenever practical.
5. High-quality CFD data is more valuable than premature AI.
6. Finish optimizer automation before training a surrogate.
7. Every downstream tool should consume the same canonical geometry.
8. Borrow aggressively from existing research before inventing algorithms.
9. Complexity must reduce iteration time, increase robustness, or enable an
   otherwise impossible capability.
10. Optimize the engineering workflow, not only the mathematics.
