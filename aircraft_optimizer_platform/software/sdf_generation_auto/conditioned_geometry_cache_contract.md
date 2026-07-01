# Conditioned Geometry Cache Contract

Date created: 2026-06-30

## Purpose

This document defines the Phase 1 contract for a geometry-generator-owned
conditioned geometry cache between the canonical implicit geometry graph and
downstream clients.

The cache is a derived representation. It is not the aircraft source of truth.
It exists to provide stable, inspectable, query-optimized signed-distance data
for export, analysis, diagnostics, visualization, and future direct-geometry
clients.

## Architecture Boundary

```text
Canonical geometry graph
  -> conditioned geometry cache
  -> clients
       - STL export
       - OpenFOAM/snappyHexMesh preparation
       - manufacturing checks
       - geometry validation
       - visualization
       - future AI datasets/inference
       - future SDF-native CFD research
```

The canonical graph owns:

- aircraft parameters,
- procedural geometry construction,
- feature/component definitions,
- analytic or composed SDF evaluation,
- source hashes and generator versions.

The conditioned geometry cache is owned by the geometry generator layer and
owns:

- sampled and reconditioned SDF blocks,
- adaptive local overlay regions,
- sparse/dense narrow-band backing stores,
- dirty-region update records,
- conditioning diagnostics,
- publication confidence, quality confidence, and fallback metadata,
- query validity bounds,
- storage and memory accounting.

Downstream clients should not need to know whether a geometry query was served from
an incrementally updated cache, a rebuilt cache, or direct analytic fallback.

## Cache Properties

Required properties:

- Versioned.
- Disposable.
- Rebuildable.
- Incrementally updateable.
- Query optimized.
- Diagnostics backed.
- Confidence tagged.
- Safe to bypass through full conditioning or direct analytic sampling.

Required invariants:

- Cache updates must not mutate the canonical graph.
- Cache status must not redefine geometry identity.
- Cache failures are conditioning failures unless the canonical graph itself is
  invalid.
- Derived artifacts must record which graph/cache versions produced them.

## Cache Identity

```yaml
conditioned_cache_id: string
cache_contract_version: conditioned_geometry_cache.v1
geometry_revision_id: string
evaluation_id: string | null
geometry_provider_result_id: string
canonical_graph:
  provider_name: string
  provider_version: string
  provider_kind: string
  source_hash: string
  feature_name: string
  coordinate_frame: string
conditioning:
  conditioning_version: string
  conditioning_backend: fmm | fsm | pde_reinitialization | openvdb | fixture | none
  update_mode: local_incremental | full_rebuild | direct_analytic | unavailable
  grid_spacing_mm: number | null
  narrow_band_half_width_mm: number | null
  block_size_mm: number | null
status:
  cache_state: ready | partial | unavailable | rejected | experimental
  query_confidence: number | null
  fallback_available: boolean
  fallback_mode: full_conditioning | direct_analytic | exporter_direct_sampling | none
storage:
  block_sample_count: integer
  block_sample_bytes: integer
  adaptive_stored_sample_count: integer
  adaptive_stored_sample_bytes: integer
  adaptive_dense_region_count: integer
  adaptive_sparse_region_count: integer
  adaptive_dense_value_count: integer
  adaptive_sparse_value_count: integer
  total_stored_sample_count: integer
  total_stored_sample_bytes: integer
  estimated_metadata_bytes: integer
  estimated_total_bytes: integer
metadata:
  warnings:
    - string
  notes: string | null
```

## Dirty Region Contract

Every geometry operation or generated geometry revision should be able to report
a dirty-region summary. Precise metadata enables local conditioning; broad
full-geometry metadata remains a valid fallback when locality is not provable.

```yaml
dirty_region:
  dirty_region_id: string
  source: full_geometry | parameter_edit | feature_edit | unknown
  operation_type: geometry_generation | parameter_update | boolean | blend | shell | offset | transform | unknown
  feature_ids:
    - string
  component_ids:
    - string
  bbox_min_mm: [number, number, number]
  bbox_max_mm: [number, number, number]
  support_radius_mm: number | null
  blend_width_mm: number | null
  offset_or_shell_width_mm: number | null
  recommended_grid_spacing_mm: number | null
  recommended_halo_mm: number
  topology_change_expected: boolean
  notes: string | null
```

The initial generator path may conservatively mark the full geometry bbox as
dirty. That is valid metadata, but it does not prove local conditioning.

## Conditioning Request

```yaml
conditioning_request_id: string
geometry_revision_id: string
evaluation_id: string | null
geometry_provider_result_id: string
cache_contract_version: conditioned_geometry_cache.v1
requested_update_mode: local_incremental | full_rebuild | direct_analytic
requested_backend: fmm | fsm | pde_reinitialization | openvdb | fixture
dirty_regions:
  - dirty_region
quality_policy:
  max_projection_residual_p95_mm: number
  max_gradient_abs_error_p95: number
  max_interface_displacement_mm: number
  require_local_topology_unchanged: boolean
  require_export_quality_non_regression: boolean
resource_limits:
  timeout_seconds: number | null
  max_memory_mb: number | null
  max_blocks: number | null
metadata:
  requested_by: kernel | ui | script | test | background_worker
  reason: string | null
```

## Conditioning Result

```yaml
conditioning_result_id: string
conditioning_request_id: string
conditioned_cache_id: string
status: success | failed | skipped | fallback_used
accepted: boolean
update_mode_used: local_incremental | full_rebuild | direct_analytic | none
backend_used: fmm | fsm | pde_reinitialization | openvdb | fixture | none
cache_state: ready | partial | unavailable | rejected | experimental
dirty_block_count: integer
halo_block_count: integer
conditioned_sample_count: integer
iteration_count: integer | null
storage:
  block_sample_count: integer
  block_sample_bytes: integer
  adaptive_stored_sample_count: integer
  adaptive_stored_sample_bytes: integer
  adaptive_dense_region_count: integer
  adaptive_sparse_region_count: integer
  adaptive_dense_value_count: integer
  adaptive_sparse_value_count: integer
  total_stored_sample_count: integer
  total_stored_sample_bytes: integer
  estimated_metadata_bytes: integer
  estimated_total_bytes: integer
metrics:
  gradient_abs_error_p50:
    value: number
    unit: null
  gradient_abs_error_p95:
    value: number
    unit: null
  gradient_abs_error_max:
    value: number
    unit: null
  projection_residual_p50_mm:
    value: number
    unit: mm
  projection_residual_p95_mm:
    value: number
    unit: mm
  projection_residual_max_mm:
    value: number
    unit: mm
  interface_displacement_estimate_mm:
    value: number
    unit: mm
  sign_flip_count_near_interface:
    value: integer
    unit: count
  local_topology_delta:
    value: integer
    unit: count
  publication_confidence:
    value: number
    unit: null
  quality_confidence:
    value: number
    unit: null
artifacts:
  - artifact_id
fallback:
  fallback_available: boolean
  fallback_used: boolean
  fallback_mode: full_conditioning | direct_analytic | exporter_direct_sampling | none
warnings:
  - string
failure: failure_record | null
```

## Geometry-Generator Module Vocabulary

Conditioning modules are geometry-kernel modules. Exporters, CFD tools,
visualization, manufacturing checks, and future automation consume their
metadata and artifacts; they do not own cache construction.

Initial module name:

```yaml
module_name: conditioned_geometry_cache_fixture
module_kind: geometry
fidelity_level: geometry
expected_runtime_class: cheap
input_contract_version: conditioned_geometry_cache_request.v1
output_contract_version: conditioned_geometry_cache_result.v1
```

Future production module names may include:

- `conditioned_geometry_cache_fmm`
- `conditioned_geometry_cache_fsm`
- `conditioned_geometry_cache_openvdb`
- `conditioned_geometry_cache_full_rebuild`

## Artifact Vocabulary

Conditioning artifacts:

- `conditioning_request_json`
- `conditioning_result_json`
- `conditioned_cache_manifest_json`
- `conditioned_sdf_block_store`
- `conditioning_diagnostics_json`
- `conditioning_slice_plot`
- `conditioning_comparison_report`

These artifacts are optional until a real cache backend exists. Geometry
provider metadata may reference a planned or unavailable cache without emitting
these artifacts.

## Metric Vocabulary

Metric names should use the `conditioning.` prefix in module attempts:

- `conditioning.gradient_abs_error_p50`
- `conditioning.gradient_abs_error_p95`
- `conditioning.gradient_abs_error_max`
- `conditioning.projection_residual_p50_mm`
- `conditioning.projection_residual_p95_mm`
- `conditioning.projection_residual_max_mm`
- `conditioning.interface_displacement_estimate_mm`
- `conditioning.sign_flip_count_near_interface`
- `conditioning.local_topology_delta`
- `conditioning.dirty_region_count`
- `conditioning.dirty_block_count`
- `conditioning.halo_block_count`
- `conditioning.conditioned_sample_count`
- `conditioning.iteration_count`
- `conditioning.block_sample_count`
- `conditioning.block_sample_bytes`
- `conditioning.adaptive_stored_sample_count`
- `conditioning.adaptive_stored_sample_bytes`
- `conditioning.adaptive_dense_region_count`
- `conditioning.adaptive_sparse_region_count`
- `conditioning.adaptive_dense_value_count`
- `conditioning.adaptive_sparse_value_count`
- `conditioning.total_stored_sample_count`
- `conditioning.total_stored_sample_bytes`
- `conditioning.estimated_metadata_bytes`
- `conditioning.estimated_total_bytes`
- `conditioning.cache_ready`
- `conditioning.cache_partial`
- `conditioning.fallback_available`
- `conditioning.fallback_used`
- `conditioning.publication_confidence`
- `conditioning.quality_confidence`

## Geometry Provider Metadata

Geometry-provider results should include live cache metadata when a conditioned
kernel is available, and cache planning metadata when a legacy/direct path has
not yet produced a conditioned cache:

```yaml
conditioned_geometry_cache:
  cache_contract_version: conditioned_geometry_cache.v1
  canonical_graph_role: source_of_truth
  cache_role: derived_query_acceleration
  cache_state: unavailable | experimental | ready | partial
  update_mode: not_run | local_incremental | full_rebuild | direct_analytic
  fallback_available: true
  fallback_mode: exporter_direct_sampling | direct_analytic | full_conditioning
  client_awareness_required: false
dirty_regions:
  - dirty_region
```

For legacy generated geometry paths that have not entered the conditioned
backend, set:

```yaml
cache_state: unavailable
update_mode: not_run
fallback_mode: exporter_direct_sampling
client_awareness_required: false
```

This records readiness for future conditioning without changing legacy export
or analysis behavior. Backend-facing paths should prefer the live conditioned
kernel and publish `ready`, `partial`, or `rejected` state from the cache.

## Acceptance Criteria

A local conditioning update may be accepted only when:

- projection residual is within configured tolerance,
- gradient magnitude error is within configured tolerance,
- interface displacement is within configured tolerance,
- no unexpected sign flips occur outside the dirty region plus halo,
- local topology delta is zero unless a topology-changing operation is explicit,
- locality and memory gates stay within the active backend policy,
- local-vs-full equivalence remains within configured distance and gradient
  tolerances for regression/debug suites,
- export quality is unchanged or improved when an export comparison is part of
  the policy.

If any required check fails, the generator-side conditioning module must mark
the cache update as rejected or partial and expose fallback metadata.

## Fallback Semantics

Fallback is a first-class geometry-kernel result.

Valid fallback modes:

- `exporter_direct_sampling`: current direct sparse exporter samples the
  canonical SDF path without a conditioned cache.
- `direct_analytic`: query API calls the canonical graph directly.
- `full_conditioning`: rebuild a full cache instead of applying a local update.
- `none`: no fallback is available.

Downstream clients must continue to operate unchanged when fallback is used.

## Current Implementation Scope

The current backend implementation includes:

1. Contract and metadata wiring through `Sdf::metadata()`.
2. Kernel-owned `ConditionedGeometryKernel` and `ConditionedGeometryModel`.
3. Local incremental updates with full-rebuild fallback.
4. Fast-sweeping SDF reconditioning with interface anchors and sparse
   narrow-band fallback.
5. Adaptive local overlay regions.
6. Background scheduling with queued-edit coalescing.
7. Geometry-service queries over the conditioned field.
8. Cache storage and memory accounting in metadata and headless metrics.
9. Real-aircraft regression gates using the curated direct sparse OML aircraft
   script.

Do not adopt OpenVDB, NanoVDB, GPU acceleration, SDF-native CFD, or AI training
until a separate evidence-backed implementation decision is recorded.
