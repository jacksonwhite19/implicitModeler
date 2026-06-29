# Mesh Comparison Acceptance Contract

Status: v0.1 comparison contract.

Purpose: define the common evidence package for comparing Gmsh and snappyHexMesh on full-aircraft small-UAV CFD meshing. This contract is for mesher selection and integration readiness. It is not a scoring-CFD acceptance standard yet.

## Scope

Each mesher must run the same aircraft variant set from:

```text
software/optimizer/configs/mesh_comparison_variants.v0_1.json
```

The comparison workflow is:

```text
aircraft variant
-> geometry definition
-> OML STL export
-> raw surface quality gate
-> mesher-specific surface preparation if needed
-> volume mesh generation
-> strict mesh quality check
-> solver smoke test
-> geometry-capture audit
-> artifact manifest and verdict
```

Do not use Poisson-smoothed fallback surfaces for the primary comparison. If a Poisson or global smoothing fallback is used, label it as a fallback and keep it out of the primary pass/fail table.

## Required Verdicts

Each variant receives one verdict per mesher:

- `pass_for_plumbing`: strict mesh checks pass, solver smoke completes, artifacts are complete, and geometry capture is acceptable for development.
- `conditional_for_plumbing`: the case is useful but has a bounded issue such as high runtime, localized quality warnings, or visible but non-catastrophic geometry degradation.
- `fail_for_plumbing`: the mesh cannot be generated, cannot pass strict checks, cannot run the smoke solver, or loses major aircraft geometry.
- `blocked`: the run could not be completed for environment/tooling reasons.

No current mesher result should be marked `scoring_ready` until boundary-layer strategy, force reporting, and solver validation are defined and passed.

## Required Per-Variant Artifacts

Each mesher must produce a run folder containing:

- raw exported STL or source surface reference.
- prepared/conditioned STL if any repair or conditioning was used.
- surface-preparation report, even if no preparation was needed.
- final volume mesh.
- solver case directory.
- all mesher logs.
- strict mesh-check log.
- relaxed mesh-check log only if used.
- failure-set geometry if strict checks fail, such as `skewFaces.vtk` or non-orthogonal face sets.
- solver smoke log.
- machine-readable mesh metrics JSON.
- machine-readable geometry-capture JSON.
- machine-readable run manifest JSON.
- machine-readable evidence bundle JSON.
- aircraft-only screenshot set.
- final human-readable evidence report.

The aircraft-only screenshot set must include:

- ISO.
- top.
- side.
- front.
- wing root/blend closeup.
- leading-edge closeup.
- trailing-edge closeup.
- wingtip closeup.
- nose closeup.
- tail closeup.

Screenshots must show only the aircraft patch/surface. Do not include the farfield box or the volume mesh unless a separate diagnostic view is explicitly labeled.

## Required Metrics

Each run manifest must record:

- mesher name and version.
- solver/tool versions.
- input variant id.
- input variable values.
- source STL path.
- source STL triangle count.
- source STL vertex count.
- raw surface topology: boundary edges, non-manifold edges, connected components, duplicate triangles.
- surface-preparation method.
- maximum surface-preparation displacement, if any.
- p95 surface-preparation displacement, if any.
- mesh runtime seconds.
- full pipeline runtime seconds.
- volume cell count.
- node/point count.
- aircraft boundary face count.
- farfield boundary face count.
- patch names.
- max skewness.
- max non-orthogonality when available.
- severe non-orthogonal face count when available.
- min cell volume when available.
- max cell volume when available.
- strict mesh-check pass/fail.
- relaxed mesh-check pass/fail if used.
- solver smoke started/completed.
- solver smoke residuals/errors.
- final verdict.
- failure taxonomy codes when any gate is failed, conditional, relaxed, or incomplete.

Geometry-capture metrics must include:

- source-to-mesh sampled distance p50, p95, p99, and max.
- mesh-to-source sampled distance p50, p95, p99, and max.
- aircraft patch face count as a percentage of source STL triangle count.
- feature-region proxy metrics for leading edge, trailing edge, wingtip, wing root/blend, fuselage nose, and fuselage tail.
- axis convention used by automated feature-region metrics.

Use point-to-triangle distance sampling when available. If only nearest-vertex distance is available, label it clearly and do not compare it as equivalent to point-to-triangle distance.
Automated feature-region buckets are screening metrics only; aircraft-only screenshots remain required for geometry-capture review.

## Comparison Thresholds

These thresholds are for comparison triage, not final scoring CFD:

| Gate | Target | Notes |
|---|---:|---|
| Strict mesh check | pass | Relaxed-only pass is not enough for primary comparison. |
| Solver smoke | complete | `potentialFoam` is acceptable as plumbing evidence. |
| Surface prep max displacement | <= 0.5 mm | Larger displacement requires explicit review. |
| Surface prep p95 displacement | <= 0.25 mm | Larger displacement suggests geometry distortion risk. |
| Source-to-mesh p95 distance | <= 1.5 mm preferred, <= 3.0 mm conditional | Exact threshold may tighten after feature metrics improve. |
| Mesh-to-source p95 distance | <= 1.0 mm preferred, <= 2.0 mm conditional | Exact threshold may tighten after feature metrics improve. |
| Geometry screenshots | all present | Missing screenshots make the case incomplete. |
| Major feature loss | none | Missing LE/TE/tip/blend details is a failure for scoring readiness. |
| Runtime | report only | Do not fail solely on runtime yet; use it for default-mesher selection. |

Current snappy comparison policy allows a conditional relaxed-quality result
when max skewness is below `8`, non-orthogonality remains within the configured
ceiling, and the relaxed result is explicitly labeled. Relaxed-quality results
are plumbing evidence, not scoring-CFD evidence.

## Mesher-Specific Notes

Gmsh:

- Preserve SU2 and OpenFOAM handoff artifacts when possible.
- Record whether tetrahedral mesh generation used a decimated/remeshed wall surface.
- Track severe non-orthogonality even if default OpenFOAM `checkMesh` reports `Mesh OK`.
- Feature refinement around LE/TE/tips/blends must be runtime-bounded before Gmsh can become the default scoring path.

snappyHexMesh:

- Preserve every repair iteration and failed strict-check case.
- Record local smoothing/capping displacement budgets.
- Preserve `checkMesh -writeSets -writeSurfaces` output whenever strict checks fail.
- Feature-aware LE/TE/tip/blend refinement is required before snappyHexMesh can become a scoring path.

## Cross-Mesher Decision Rule

After all five variants are run, compare:

- number of variants with `pass_for_plumbing`.
- number of variants with strict mesh-check pass.
- solver-smoke completion rate.
- geometry-capture p95/p99/max distances.
- feature-region visual quality.
- runtime spread and worst-case runtime.
- amount of surface repair or geometry displacement.
- failure diagnosability and artifact completeness.

The near-term default mesher should be the one with the best combination of repeatability, artifact completeness, geometry capture, and solver-readiness. If each mesher has different strengths, keep both and assign separate roles instead of forcing a single default.
