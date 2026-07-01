# Module Adapter Contract

Date created: 2026-06-20

## Purpose

A module adapter performs one bounded step in an evaluation pipeline.

Examples:

- Geometry validation.
- OML STL export.
- Screenshot generation.
- Geometry metrics.
- Mass/CG estimate.
- Low-fidelity aero.
- Medium-fidelity aero.
- CFD validation.
- Scoring.

## Module Specification

```yaml
module_name: string
module_version: string
module_kind: geometry | export | analysis | scoring | visualization | validation
input_contract_version: string
output_contract_version: string
fidelity_level: none | geometry | low | medium | high
expected_runtime_class: cheap | moderate | expensive
deterministic: boolean
cache_policy: none | input_hash | artifact_hash
```

## Module Request

```yaml
module_attempt_id: string
evaluation_id: string
candidate_id: string
module_name: string
module_version: string
inputs:
  candidate_ref: candidate_id
  geometry_provider_result_ref: string | null
  artifact_refs:
    - artifact_id
  parameters: map
resource_limits:
  timeout_seconds: number | null
  max_memory_mb: number | null
  max_workers: number | null
```

## Module Result

```yaml
module_attempt_id: string
evaluation_id: string
candidate_id: string
module_name: string
module_version: string
status: success | failed | skipped | cancelled
started_at: datetime
finished_at: datetime
runtime_seconds: number
metrics:
  metric_name:
    value: any
    unit: string | null
    confidence: number | null
    source: string
artifact_ids:
  - string
warnings:
  - string
failure: failure_record | null
metadata: map
```

## Failure Record

```yaml
failure_id: string
category: string
stage: string
severity: warning | recoverable | fatal
retryable: boolean
message: string
evidence_artifact_ids:
  - string
suggested_next_action: string | null
```

## Rules

- A module must not mutate the candidate.
- A module must register produced artifacts.
- A module must classify failures.
- A module should emit partial artifacts when useful.
- A module result must record version and runtime metadata.
- A module should be replaceable by another implementation with the same input/output contract.

## OML STL Exporter Fit

The current direct sparse OML STL exporter should be wrapped as:

```yaml
module_kind: export
fidelity_level: geometry
expected_runtime_class: moderate
```

It should consume a geometry provider result and produce:

- `oml_stl`
- `export_result_json`
- logs
- quality metrics
- failure classification if gates fail

## Conditioned Geometry Cache Fit

The conditioned geometry cache is owned by the geometry kernel. This section is
only a downstream recording contract for legacy evaluation evidence when a
conditioning attempt is present.

A generator-side conditioned geometry cache module should be recorded as:

```yaml
module_kind: geometry
fidelity_level: geometry
expected_runtime_class: cheap | moderate
input_contract_version: conditioned_geometry_cache_request.v1
output_contract_version: conditioned_geometry_cache_result.v1
```

Initial module name:

```text
conditioned_geometry_cache_fixture
```

Future module names may include:

- `conditioned_geometry_cache_fmm`
- `conditioned_geometry_cache_fsm`
- `conditioned_geometry_cache_openvdb`
- `conditioned_geometry_cache_full_rebuild`

It should consume a geometry provider result or generator cache request and
produce:

- `conditioning_request_json`
- `conditioning_result_json`
- optional `conditioned_cache_manifest_json`
- optional `conditioned_sdf_block_store`
- optional diagnostic plots/reports
- conditioning metrics
- confidence and fallback metadata

Metric names should use the `conditioning.` prefix:

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
- `conditioning.cache_ready`
- `conditioning.cache_partial`
- `conditioning.fallback_available`
- `conditioning.fallback_used`
- `conditioning.confidence`

The cache is derived, disposable, rebuildable, and versioned. A cache failure
should not fail downstream evaluation unless policy explicitly requires
conditioned geometry and no fallback is available.
