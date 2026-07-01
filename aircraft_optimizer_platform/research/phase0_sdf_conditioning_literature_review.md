# Phase 0 SDF Conditioning Literature Review

Date: 2026-06-30

## Purpose

This document answers the Phase 0 gate questions for local SDF conditioning,
geometry-query infrastructure, future SDF-native CFD, and AI surrogate planning.

The objective is not to build a new CFD solver. The objective is to make the
implicit aircraft kernel continuously reliable enough that meshing, OpenFOAM,
optimizer automation, manufacturing checks, visualization, and future AI clients
can all query the same canonical geometry.

## Executive Recommendation

Proceed with Phase 1 conditioning design, but keep the first implementation
narrow:

1. Implement dirty-region tracking around the existing analytic/composed SDF
   operations.
2. Prototype local redistancing on a narrow-band block cache using fast marching
   or fast sweeping, with PDE reinitialization reserved for corrective passes.
3. Build diagnostics and regression gates before changing optimizer-facing
   export behavior.
4. Evaluate OpenVDB/NanoVDB and scikit-fmm as references or prototype tools
   before writing a full sparse-volume stack.
5. Do not start SDF-native CFD, cut-cell solver work, or PhysicsNeMo training
   yet. Those remain downstream clients after optimizer automation, CFD data
   quality, and conditioning stability are stronger.

Gate verdict:

- Phase 1 conditioning architecture work is unblocked.
- Production AI-surrogate work remains blocked.
- Production SDF-native CFD remains blocked.
- Current OpenFOAM/snappyHexMesh optimizer path remains the production CFD path.

## Success Criteria

The conditioning system is considered successful if:

- At least 95% of normal parameter edits trigger only local conditioning.
- Conditioning runtime scales with modified volume rather than full aircraft
  size.
- Export quality is unchanged or improved relative to the current direct sparse
  pipeline.
- Projection residual remains below configurable tolerances inside the
  conditioning band.
- Gradient magnitude remains near unity inside the conditioning band.
- Local conditioning produces geometry equivalent to full conditioning within
  configured tolerance.
- All existing export, geometry, mesh-readiness, and optimizer regression suites
  continue to pass.
- The optimizer requires no awareness of whether geometry came from a freshly
  rebuilt cache, an incrementally updated cache, or a full-conditioning fallback.

## Explicit Non-Goals

This phase does not attempt to:

- Build a new CFD solver.
- Replace OpenFOAM.
- Train PhysicsNeMo or any other AI surrogate.
- Produce differentiable CFD.
- Implement GPU acceleration as a required capability.
- Replace the existing canonical geometry graph.
- Rewrite the direct sparse exporter.
- Build a sparse volume engine from scratch before evaluating OpenVDB/NanoVDB.
- Make meshes obsolete for the current production optimizer loop.

## Findings by Topic

### Signed-Distance Reinitialization

Existing algorithms already cover the core math.

- Interface-preserving PDE redistancing: Sussman, Smereka, Osher, and later
  constrained redistancing variants solve a reinitialization equation to restore
  signed-distance behavior while reducing interface drift.
- Fast Marching Method (FMM): Sethian's method solves Eikonal/Hamilton-Jacobi
  distance problems with an ordered upwind update and is a strong fit when a
  narrow-band front can be seeded from the zero level set.
- Fast Sweeping Method (FSM): Zhao's method uses directional Gauss-Seidel
  sweeps and is attractive on structured grids, especially when local
  characteristics are not too tangled.
- Narrow-band level sets: established level-set practice limits computation to
  an active band near the interface, exactly matching this project's need to
  avoid full-aircraft recomputation.

Recommendation:

- Implement or integrate a local Eikonal redistancing backend first.
- Use FMM where robustness around irregular dirty regions matters.
- Use FSM as a simpler structured-grid alternative for rectangular block
  updates.
- Keep PDE reinitialization as a corrective or quality-improvement pass where
  interface drift can be measured and bounded.

What not to reinvent:

- Do not invent a new distance-restoration equation.
- Do not hand-wave gradient quality. Measure `abs(|grad phi| - 1)`, projection
  residual, and interface displacement.

Key sources:

- Sussman et al. interface-preserving redistancing:
  https://epubs.siam.org/doi/10.1137/S1064827596298245
- Sethian fast marching methods:
  https://epubs.siam.org/doi/10.1137/S0036144598347059
- Zhao fast sweeping method:
  https://graphics.stanford.edu/courses/cs468-03-fall/Papers/zhao_fastsweep1.pdf
- Adalsteinsson and Sethian narrow-band level set method:
  https://www.semanticscholar.org/paper/A-Fast-Level-Set-Method-for-Propagating-Interfaces-Adalsteinsson-Sethian/08fa6b53aeb9490f0a4c86d6b8eba9ba89d540d8

### Sparse and Narrow-Band SDF Infrastructure

OpenVDB is the most relevant existing sparse-volume system. It provides sparse
3D grids, level-set/SDF tools, CSG-style volume operations, mesh conversion, and
efficient random access. NanoVDB adds GPU-friendly read/query support but has
static topology limitations, so it is better suited for query/export snapshots
than for the mutable authoring cache.

scikit-fmm is a useful Python reference for FMM behavior and quick experiments.
It is not a production geometry kernel by itself, but it can validate distance
updates against a known implementation.

Recommendation:

- Before building a large custom sparse field system, run a small OpenVDB
  evaluation around:
  - analytic SDF sampling into a sparse narrow band,
  - CSG/blend operation support,
  - redistancing/level-set filtering,
  - mesh/export round trip,
  - Rust/Python/C++ integration cost.
- Use scikit-fmm for small 2D/3D test fixtures and regression oracles.
- Treat NanoVDB as a future fast-query artifact format, not the first mutable
  conditioning representation.

What not to reinvent:

- Do not implement a custom sparse tree until OpenVDB/NanoVDB integration cost
  is measured.
- Do not assume dense grids will scale to whole-aircraft conditioning.

Key sources:

- OpenVDB overview:
  https://www.openvdb.org/about/
- OpenVDB GitHub/license:
  https://github.com/AcademySoftwareFoundation/openvdb
- OpenVDB mesh-to-SDF tool:
  https://www.openvdb.org/documentation/doxygen/MeshToVolume_8h.html
- OpenVDB topology-to-level-set tool:
  https://www.openvdb.org/documentation/doxygen/TopologyToLevelSet_8h.html
- NanoVDB FAQ:
  https://www.openvdb.org/documentation/doxygen/NanoVDB_FAQ.html
- scikit-fmm:
  https://github.com/scikit-fmm/scikit-fmm

### Incremental and Dynamic SDF Maintenance

Robotics has strong prior art for incremental distance fields. Voxblox builds
Euclidean signed distance fields incrementally from TSDFs for onboard MAV
planning. nvblox moves the same style of mapping to GPU-accelerated robotics
use cases.

These systems are not directly a CAD kernel. They are designed around sensor
fusion, occupancy/TSDF updates, and planning maps. The useful part for this
project is the block-structured incremental update pattern:

- divide space into blocks,
- track changed blocks,
- propagate distance changes locally,
- preserve queryable distance and gradient data for downstream planners.

Recommendation:

- Borrow the block cache, dirty queue, and incremental propagation concepts.
- Do not convert the canonical CAD field into a TSDF sensor-fusion map.
- For analytic CAD edits, dirty regions should come from operation support
  bounds, not from sensor observations.

What not to reinvent:

- Do not treat incremental updates as a novel project-specific idea.
- Do not adopt robotics TSDF assumptions without checking whether they preserve
  aircraft topology, shells, and thin features.

Key sources:

- Voxblox paper:
  https://arxiv.org/abs/1611.03631
- Voxblox GitHub:
  https://github.com/ethz-asl/voxblox
- nvblox documentation:
  https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html
- nvblox paper:
  https://arxiv.org/html/2311.00626v2

### Geometry Query API

Most required queries follow directly from a conditioned SDF:

- `distance(p)` is the field value.
- `gradient(p)` is finite difference, analytic derivative, or cached gradient.
- `surface_normal(p)` is normalized gradient.
- `project_to_surface(p)` is commonly `p - phi(p) * normalized_gradient(p)`
  when the field is well conditioned.
- `inside_outside(p)` is the sign of the field.
- `curvature` and `principal_curvature` come from gradient/Hessian estimates.
- `local_feature_size` and `wall_thickness` require more than the basic SDF:
  they need medial-axis-like estimates, opposing-surface searches, or ray/normal
  probes with ambiguity handling.

Recommendation:

- Define the API contract before implementing all query methods.
- Split queries into:
  - Tier 1: distance, sign, gradient, normal, projection.
  - Tier 2: ray intersection, closest point, cross-section, area/volume.
  - Tier 3: local feature size, wall thickness, curvature, principal curvature.
- Require every query result to report source, grid spacing, conditioning
  version, and confidence/validity metadata when applicable.

What not to reinvent:

- Do not duplicate query logic in exporter, optimizer, manufacturing, and CFD
  wrappers.

### CFD: Immersed Boundary, Ghost Cell, Cut Cell, Cartesian Grid

The CFD literature and software ecosystem already contains SDF/distance-field
consumers:

- Immersed-boundary methods use a fixed Cartesian/Eulerian grid with boundary
  forcing or reconstruction.
- Ghost-cell immersed-boundary methods impose boundary conditions by
  reconstructing values near immersed surfaces.
- Cut-cell/embedded-boundary methods intersect geometry with Cartesian cells and
  solve on irregular control volumes.
- AMReX embedded boundaries and Basilisk embedded boundaries are relevant
  open-source examples of Cartesian embedded-boundary infrastructure.
- Lethe is open-source CFD/DEM software with sharp-interface immersed-boundary
  support and SDF-based immersed-solid property models.

These are important research paths, but they should not replace the current
OpenFOAM/snappyHexMesh route yet. Viscous, low-speed aircraft CFD is sensitive
to boundary layers, wall treatment, convergence, and validation. Cartesian
cut-cell methods can be excellent for automation, but small cut cells and
viscous near-wall modeling are known practical challenges.

Recommendation:

- Keep OpenFOAM/snappyHexMesh as the production optimizer CFD path.
- Treat Lethe, AMReX EB, Basilisk EB, and custom immersed-boundary work as Phase
  6 research candidates.
- When conditioning is stable, add an SDF export/query adapter that could feed
  these tools without changing the production optimizer loop.

What not to reinvent:

- Do not start by writing a CFD solver.
- Do not assume SDF-native CFD is automatically better than a validated mesh
  workflow for this aircraft regime.

Key sources:

- Peskin immersed-boundary review:
  https://www.cambridge.org/core/journals/acta-numerica/article/immersed-boundary-method/95ECDAC5D1824285563270D6DD70DA9A
- Tseng and Ferziger ghost-cell immersed boundary:
  https://coda.oc.ntu.edu.tw/research/papers/2003_Tseng_IBM.pdf
- Lethe GitHub:
  https://github.com/chaos-polymtl/lethe
- Lethe SDF-based immersed-solid properties:
  https://chaos-polymtl.github.io/lethe/documentation/parameters/cfd/physical_properties.html
- AMReX embedded boundaries:
  https://amrex-codes.github.io/amrex/docs_html/EB_Chapter.html
- AMReX project/license:
  https://amrex-codes.github.io/
- Basilisk embedded-boundary notes:
  https://basilisk.fr/sandbox/ghigo/src/myembed.h
- Basilisk project:
  https://basilisk.fr/

### AI Surrogates and PhysicsNeMo

PhysicsNeMo is a plausible future framework because it supports neural
operators, GNNs, transformers, PINNs, and hybrid Physics-ML workflows. FNOs are
specifically relevant because neural operators learn mappings between function
spaces and have been demonstrated on PDE families including Navier-Stokes.

However, this project does not yet have enough trusted CFD data or conditioning
stability to train an authoritative surrogate. The correct order remains:

1. Stable optimizer.
2. Stable geometry conditioning and export validation.
3. Standardized CFD cases and postprocessing.
4. Durable dataset schema.
5. Surrogate model training and uncertainty evaluation.

Potential future inputs:

- Parameter vector.
- Distance-field samples.
- Gradient samples.
- Curvature/local feature channels.
- Component/feature IDs.
- Mesh-derived fields when using graph models.

Recommendation:

- Define the CFD dataset schema now.
- Delay model training.
- Start with scalar aerodynamic outputs and uncertainty, not pressure fields.
- Keep CFD as the source of truth and use the surrogate as a prefilter.

What not to reinvent:

- Do not build a custom ML framework.
- Do not train on rough-CFD data without fidelity labels and uncertainty.
- Do not let AI predictions overwrite CFD-backed records.

Key sources:

- PhysicsNeMo GitHub:
  https://github.com/NVIDIA/physicsnemo
- PhysicsNeMo FNO API:
  https://docs.nvidia.com/physicsnemo/25.11/physicsnemo/api/models/fnos.html
- Fourier Neural Operator paper:
  https://arxiv.org/abs/2010.08895

## Integration Matrix

| Candidate | Open source | Integrate now | Use as reference | Defer |
|---|---:|---:|---:|---:|
| Fast marching / Eikonal redistancing | Algorithm | Yes | Yes | No |
| Fast sweeping / structured redistancing | Algorithm | Yes | Yes | No |
| PDE reinitialization | Algorithm | Prototype | Yes | No |
| Narrow-band level sets | Algorithm | Yes | Yes | No |
| OpenVDB | Yes, Apache 2.0 | Evaluate | Yes | No |
| NanoVDB | Yes, OpenVDB project | Not mutable core | Yes | Query snapshots |
| scikit-fmm | Yes, BSD-style | Prototype only | Yes | Production core |
| Voxblox | Yes, BSD-3-Clause | No | Yes | Robotics-only pieces |
| nvblox | Yes | No | Yes | GPU mapping |
| Lethe | Yes | No | Yes | SDF-native CFD research |
| AMReX EB | Yes, BSD-3-Clause | No | Yes | SDF-native CFD research |
| Basilisk EB | Free software | No | Yes | SDF-native CFD research |
| PhysicsNeMo | Yes, Apache 2.0 | No | Yes | After CFD dataset |

## Proposed Phase 1 Architecture

### Canonical Graph, Conditioned Cache, Clients

The clean architecture split is:

```text
Canonical geometry graph
  -> Conditioned geometry cache
  -> Clients
       - STL export
       - OpenFOAM/snappyHexMesh
       - Manufacturing checks
       - Optimizer gates and metrics
       - Visualization
       - Future AI
       - Future SDF-native CFD
```

The canonical geometry graph remains the source of truth. It owns parameters,
procedural construction, feature definitions, and exact analytic/composed SDF
evaluation.

The conditioned geometry cache is a derived subsystem. It exists to make
downstream queries fast, stable, inspectable, and locally repairable. The cache
can be discarded and rebuilt without changing the aircraft definition.

Clients should not depend directly on procedural geometry internals unless they
are geometry-authoring tools. Exporters, optimizers, CFD wrappers,
manufacturing checks, AI dataset builders, and visualization tools should
consume either the conditioned cache, a typed query API backed by it, or derived
artifacts produced from it.

### Conditioned Geometry Cache

The conditioned geometry cache is not the source of truth.

It is a derived representation optimized for downstream clients.

Required properties:

- Versioned.
- Disposable.
- Rebuildable.
- Incrementally updated.
- Query optimized.
- Diagnostics backed.
- Confidence tagged.
- Safe to bypass through full conditioning when local updates fail.

Cache contents should include:

- Conditioned SDF samples in narrow-band blocks.
- Optional gradient samples or gradient reconstruction metadata.
- Dirty-region and halo metadata.
- Conditioning algorithm and parameter version.
- Source graph version/hash.
- Local residual and confidence metrics.
- Local topology and sign-change checks.
- Query validity bounds.

Clients:

- STL export.
- OpenFOAM/snappyHexMesh preparation.
- Manufacturing checks.
- Optimizer gates and metrics.
- Visualization.
- Future AI datasets and inference.
- Future SDF-native CFD research.
- Collision, ray tracing, and structural-analysis probes.

The canonical graph never changes because the cache changes. Only the derived
cache state changes.

### Field State

Maintain three related representations:

1. Analytic/composed SDF graph: canonical geometry definition.
2. Conditioned narrow-band cache: query/export acceleration and repaired local
   distance behavior.
3. Derived artifacts: STL, OpenFOAM case, screenshots, datasets, and meshes.

The analytic graph remains canonical. The conditioned cache is disposable and
versioned.

### Dirty Region Contract

Each modeling operation should report:

- Native-frame bounding box.
- Operation type.
- Feature/component ownership.
- Support radius.
- Blend or smoothing radius.
- Offset/shell thickness where applicable.
- Required grid spacing.
- Recommended conditioning halo.

The scheduler expands the dirty box by:

```text
conditioning_halo =
  max(operation_support_radius,
      blend_width,
      offset_or_shell_width,
      finite_difference_stencil_radius,
      local_feature_size_safety_margin)
```

### Local Reconditioning Flow

```text
operation changes parameter
  -> mark dirty region
  -> sample analytic SDF in dirty blocks plus halo
  -> seed zero-level interface
  -> redistance local narrow band
  -> compute diagnostics
  -> accept or reject cache update
  -> invalidate derived artifacts
```

### Acceptance Metrics

Required metrics:

- `gradient_abs_error_p50/p95/max`
- `projection_residual_p50/p95/max`
- `interface_displacement_estimate`
- `sign_flip_count_near_interface`
- `dirty_block_count`
- `halo_block_count`
- `iteration_count`
- `conditioned_sample_count`
- `local_topology_delta`
- `query_confidence`

Reject or quarantine local updates when:

- sign changes occur outside the dirty region plus halo,
- interface displacement exceeds tolerance,
- gradient error remains high near CFD/export surfaces,
- local topology changes are not tied to an explicit topology-changing
  operation,
- the updated cache creates export topology regression.

### Diagnostics First

Build diagnostics before optimizing performance:

- heatmap slices for `|grad phi|`,
- projection residual slices,
- dirty-region overlays,
- conditioning confidence maps,
- curvature maps,
- local feature-size maps,
- wall-thickness probes,
- before/after cross-sections.

## Technical Risks

### Highest Risk

- Local conditioning introduces interface drift.
- Thin shell regions become unstable.
- Dirty-region halos become much larger than expected.
- OpenVDB integration proves incompatible with the current architecture or
  Windows/Rust/Python workflow.
- Local updates fail to match globally conditioned fields within tolerance.
- Incremental updates accumulate gradient or projection error over many edits.

### Moderate Risk

- Memory usage grows unexpectedly.
- Diagnostics become expensive enough to erase local-update performance gains.
- Sparse block bookkeeping becomes more complex than full recomputation for
  common aircraft edits.
- Local feature-size and wall-thickness queries are unreliable near blends,
  shells, or opposing surfaces.
- Cache invalidation bugs create stale downstream artifacts.

### Mitigations

- Compare local conditioning against full conditioning on controlled fixtures.
- Require confidence metrics and residual checks for every accepted cache
  update.
- Preserve the ability to fall back to full conditioning per dirty region,
  per candidate, or per campaign.
- Treat topology or sign-change surprises as cache-update failures, not as
  accepted geometry.
- Keep the optimizer and exporter interfaces independent of the conditioning
  mechanism so fallback does not change campaign orchestration.
- Evaluate OpenVDB before committing to a custom sparse field engine.

## Fallback Architecture

If incremental conditioning fails to meet quality or performance targets, the
production workflow falls back to:

```text
Canonical geometry graph
  -> Full conditioning or direct analytic sampling
  -> Mesh/export artifact
  -> Optimizer and analysis clients
```

Fallback rules:

- Local conditioning remains experimental.
- The optimizer continues operating unchanged.
- The direct sparse exporter remains available.
- OpenFOAM/snappyHexMesh remains the production CFD path.
- Failed cache updates are recorded as conditioning failures, not geometry
  failures unless the canonical graph itself is invalid.
- Full conditioning may be slower, but it preserves the rest of the platform.

## Immediate Implementation Plan

1. Add a conditioning contract document under
   `software/sdf_generation_auto` or `software/optimizer/contracts`.
2. Add diagnostic metric names to the platform artifact/module vocabulary.
3. Build a small 2D or 3D fixture using scikit-fmm or a simple local FMM/FSM
   implementation to validate redistancing behavior.
4. Evaluate OpenVDB in a throwaway prototype before committing to a custom
   sparse block structure.
5. Wire dirty-region metadata into generated geometry/provider outputs without
   changing optimizer scoring.
6. Add regression tests that compare local conditioning against full
   reconditioning on small controlled shapes.

## Final Gate Answer

What existing algorithms solve the problem?

- Eikonal redistancing with fast marching or fast sweeping, PDE-based
  reinitialization, narrow-band level sets, sparse-volume caches, and
  block-based incremental ESDF update patterns.

Which are open source?

- OpenVDB/NanoVDB, scikit-fmm, Voxblox, nvblox, Lethe, AMReX, Basilisk, and
  PhysicsNeMo all provide useful open-source reference or integration surfaces.

Which can be integrated?

- scikit-fmm can be integrated for prototypes/tests.
- OpenVDB should be evaluated for sparse level-set storage and tools.
- NanoVDB can later serve fast query snapshots.
- PhysicsNeMo can later consume standardized CFD datasets.
- Lethe/AMReX/Basilisk should stay research integrations until conditioning and
  CFD validation mature.

Which should be implemented?

- Project-specific dirty-region tracking.
- A first-class conditioned geometry cache.
- A narrow-band conditioning scheduler.
- Local FMM/FSM redistancing if OpenVDB/scikit-fmm are not suitable for the core
  runtime.
- Diagnostics, confidence metrics, and regression gates.
- Geometry-query API contracts.

What is unnecessary to reinvent?

- The Eikonal/redistancing math.
- Sparse-volume data structures without first evaluating OpenVDB.
- Robotics-style incremental distance propagation concepts.
- CFD solvers.
- ML frameworks.
