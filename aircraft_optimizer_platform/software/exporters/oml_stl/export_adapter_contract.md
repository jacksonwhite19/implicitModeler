# OML STL Export Adapter Contract

Date created: 2026-06-20

## Purpose

This document defines the optimizer-facing adapter contract for producing CFD-ready outer-mold-line STL artifacts from geometry-provider output.

This is not an exporter implementation. The current exporter remains external and referenced in place:

```text
dual_contouring\direct_sparse_sdf_mc_experiment
```

The adapter must conform to the platform module contract:

```text
software\optimizer\contracts\module_adapter_contract.md
```

## Boundary

The OML STL export adapter is a downstream module. It must not define candidate structure, geometry generation, scoring, optimization policy, or dashboard behavior.

It consumes:

- Candidate identity.
- Evaluation identity.
- Geometry-provider result identity.
- Geometry source metadata: script, feature, coordinate frame, bounds, and hash.
- Export preset and quality gates.

It produces:

- OML STL artifact.
- Machine-readable export result JSON.
- Logs.
- Mesh/export quality metrics.
- Failure records when gates fail.

## Module Specification

```yaml
module_name: oml_stl_export
module_kind: export
fidelity_level: geometry
expected_runtime_class: moderate
deterministic: true
cache_policy: input_hash
input_contract_version: oml_stl_export_request.v1
output_contract_version: oml_stl_export_result.v1
```

Runtime class may be recorded as `expensive` for selected high-fidelity reruns.

## Supported Presets

### Default Iteration Preset

```yaml
preset: direct_sparse_oml_fast
spacing_mm: 1.0
tile_size_mm: 16
coarse_spacing_mm: 4
narrow_band_voxels: 3
tile_dilation: 1
sdf_workers: 8
expected_runtime: about 4 minutes on the validated local setup
```

### Selected-Design Preset

```yaml
preset: direct_sparse_oml_high_fidelity
spacing_mm: 0.75
tile_size_mm: 24
sdf_workers: 8
expected_runtime: about 10 minutes on the validated local setup
```

The high-fidelity preset is for promising or final candidates. It is not the default campaign path.

## Request Schema

```yaml
module_attempt_id: string
evaluation_id: string
candidate_id: string
module_name: oml_stl_export
module_version: string
execution_mode: dry_run | execute
inputs:
  geometry_provider_result_id: string
  geometry_source:
    source_kind: rhai_script
    source_path: string
    source_hash: string
    feature_name: string
    coordinate_frame: native | platform | unknown
    bbox_min: [number, number, number]
    bbox_max: [number, number, number]
  preset:
    name: direct_sparse_oml_fast | direct_sparse_oml_high_fidelity
    settings:
      spacing_mm: number
      tile_size_mm: number
      sdf_workers: number
      coarse_spacing_mm: number | null
      narrow_band_voxels: number | null
      tile_dilation: number | null
  quality_limits:
    max_boundary_edges: 0
    max_nonmanifold_edges: 0
    max_connected_components: 1
    max_duplicate_triangles: 0
    max_long_chord_sections_ge_75mm: 0
    max_runtime_s: number | null
    max_high_aspect_count: number | null
    max_aspect_p99: number | null
    max_p99_edge_length: number | null
    max_edge_length: number | null
resource_limits:
  timeout_seconds: number | null
  max_memory_mb: number | null
  max_workers: number | null
metadata:
  requested_by: optimizer | human | test
  reason: string | null
```

`dry_run` may validate request completeness and dependency visibility without invoking the external exporter. Production execution must require explicit adapter enablement.

## Result Schema

```yaml
module_attempt_id: string
evaluation_id: string
candidate_id: string
module_name: oml_stl_export
module_version: string
status: success | failed | skipped | cancelled
started_at: datetime
finished_at: datetime
runtime_seconds: number
pass: boolean
artifact_ids:
  - string
metrics:
  export.boundary_edges:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.nonmanifold_edges:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.connected_components:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.duplicate_triangles:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.long_chord_sections_ge_75mm:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.vertices:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.triangles:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.runtime_seconds:
    value: number
    unit: s
    confidence: 1.0
    source: oml_stl_export
  export.sdf_sample_count:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
  export.active_tile_count:
    value: integer
    unit: count
    confidence: 1.0
    source: oml_stl_export
failure: failure_record | null
warnings:
  - string
metadata:
  preset_name: string
  preset_settings: map
  quality_limits: map
  gate_failures:
    - string
  command: string
  cwd: string
  exporter_source_path: string
  exporter_version: string | null
  exporter_hash: string | null
  sidecar_path: string | null
  sidecar_hash: string | null
  stdout_tail: string | null
  stderr_tail: string | null
  bbox_min: [number, number, number]
  bbox_max: [number, number, number]
  tile_spacing_compatible: boolean
```

## Artifact Types

The adapter should register the following artifact kinds when present:

| Artifact kind | Required | Purpose |
|---|---:|---|
| `oml_stl` | Yes on success | CFD-ready outer-mold-line mesh. |
| `export_result_json` | Yes | Machine-readable exporter result and gate evidence. |
| `stdout_log` | Yes when executed | External tool stdout. |
| `stderr_log` | Yes when executed | External tool stderr. |
| `mesh_quality_json` | Optional | Expanded mesh/topology metrics. |
| `section_audit_json` | Optional | Long-chord or section-level audit data. |
| `section_plot` | Optional | Human-readable section audit plot. |

Artifacts must include path, content hash, media type or semantic type, producing module attempt, and source metadata.

## Failure Mapping

| Failure code | Category | Stage | Retryable | Meaning |
|---|---|---|---:|---|
| `runtime.external_tool_missing` | runtime | export | No | Required script, Python environment, or binary is missing. |
| `runtime.environment_mismatch` | runtime | export | Maybe | Dependency version, executable hash, or working directory does not match the request. |
| `geometry.missing_feature` | geometry | export_input | No | Requested SDF feature is missing from the geometry source. |
| `geometry.invalid_bbox` | geometry | export_input | No | Bounds are absent, inverted, or outside allowed policy. |
| `meshing.boundary_edges_exceeded` | meshing | quality_gate | No | Boundary edge gate failed. |
| `meshing.nonmanifold_edges_exceeded` | meshing | quality_gate | No | Nonmanifold edge gate failed. |
| `meshing.connected_components_exceeded` | meshing | quality_gate | No | Connected-component gate failed. |
| `meshing.duplicate_triangles_exceeded` | meshing | quality_gate | No | Duplicate-triangle gate failed. |
| `meshing.long_chord_sections_exceeded` | meshing | quality_gate | No | Section/chord audit gate failed. |
| `meshing.timeout` | meshing | export | Yes | Export exceeded configured timeout. |
| `artifact.missing_expected_artifact` | artifact | registration | Maybe | Expected STL, result JSON, or log was not produced. |
| `artifact.hash_mismatch` | artifact | registration | No | Artifact hash does not match the recorded value. |

Gate failures are failed module attempts, not silent warnings. The failed candidate should remain queryable and useful for future failure prediction.

## Event Sequence

Expected event stream:

```text
module.started
artifact.created
module.completed
```

On failure:

```text
module.started
artifact.created
failure.recorded
module.failed
```

If useful partial artifacts exist after failure, they should still be registered before `module.failed`.

## Integration Rules

- Do not mutate candidate records.
- Do not infer or generate geometry inside the export adapter.
- Do not relax strict quality gates silently.
- Do not use shell/manufacturing export as the default optimizer path.
- Do not let dashboard state become the source of truth for export status.
- Do not call exporter scripts directly from optimizer orchestration code; use this adapter boundary.
- Preserve command, version, hash, working directory, runtime, limits, and artifact hashes for reproducibility.
- Preserve geometry source path and source hash so the STL can be traced back to the generated or manual definition.

## Implementation Phases

1. Contract and dry-run only.
2. Result JSON parser using a checked-in fixture result.
3. Real adapter execution behind an explicit local enable flag.
4. High-fidelity selected-design rerun path.
5. Batch execution and worker scheduling.

## Current Status

Status: contract defined, implementation not integrated.

The next implementation step is a parser/fixture test for exporter result JSON. Real exporter invocation should wait until the adapter can record module attempts, artifacts, metrics, failures, and events without special-case logic.
