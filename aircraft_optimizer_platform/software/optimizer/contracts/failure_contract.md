# Failure Contract

Date created: 2026-06-20

## Purpose

Failures are first-class records. A failed candidate, module, or evaluation is useful data for debugging, optimizer behavior, future ML failure prediction, and campaign triage.

## Failure Record

```yaml
failure_id: string
candidate_id: string | null
evaluation_id: string | null
module_attempt_id: string | null
category: string
stage: string
severity: warning | recoverable | fatal
retryable: boolean
message: string
evidence_artifact_ids:
  - string
suggested_next_action: string | null
created_at: datetime
metadata: map | null
```

## Category Prefixes

Use stable category prefixes:

- `input`
- `geometry`
- `metrics`
- `mass`
- `meshing`
- `aero_low`
- `aero_medium`
- `cfd`
- `scoring`
- `optimization`
- `artifact`
- `runtime`
- `dashboard`

## Initial Failure Categories

Input/config:

- `input.invalid_variable_schema`
- `input.missing_required_variable`
- `input.out_of_bounds`
- `input.unit_mismatch`
- `input.invalid_pipeline_config`

Geometry:

- `geometry.provider_error`
- `geometry.invalid_aircraft_definition`
- `geometry.missing_feature`
- `geometry.invalid_coordinate_frame`
- `geometry.invalid_bbox`
- `geometry.self_intersection`
- `geometry.non_manifold_surface`
- `geometry.open_surface`
- `geometry.export_source_missing`
- `geometry.timeout`

Meshing/export:

- `meshing.surface_mesh_failed`
- `meshing.bad_cell_quality`
- `meshing.boundary_edges_exceeded`
- `meshing.nonmanifold_edges_exceeded`
- `meshing.connected_components_exceeded`
- `meshing.duplicate_triangles_exceeded`
- `meshing.long_chord_sections_exceeded`
- `meshing.timeout`

Analysis:

- `metrics.failed`
- `mass.estimate_failed`
- `mass.cg_out_of_bounds`
- `aero_low.unsupported_geometry`
- `aero_low.invalid_coefficients`
- `aero_medium.solver_failed`
- `cfd.case_setup_failed`
- `cfd.divergence`
- `cfd.timeout`
- `cfd.invalid_force_coefficients`

Scoring/optimization:

- `scoring.missing_required_metric`
- `scoring.invalid_metric_value`
- `optimization.constraint_violation`
- `optimization.duplicate_candidate`
- `optimization.optimizer_state_error`

Artifact/runtime:

- `artifact.write_failed`
- `artifact.hash_mismatch`
- `artifact.missing_expected_artifact`
- `runtime.external_tool_missing`
- `runtime.environment_mismatch`
- `runtime.worker_crash`
- `runtime.cancelled_by_user`

## Severity Rules

- `warning`: issue recorded but pipeline can continue.
- `recoverable`: pipeline step failed but rerun or alternate module may work.
- `fatal`: evaluation cannot proceed.

## Retry Rule

Set `retryable = true` only when a rerun with changed resources/settings could plausibly succeed.

Examples:

- Timeout can be retryable.
- Missing external binary is not retryable until environment changes.
- Constraint violation is usually not retryable for the same candidate.

## Evidence Rule

Where possible, attach evidence artifacts:

- logs
- result JSON
- mesh quality report
- screenshot
- solver config
- stdout/stderr capture

Do not rely on a prose message alone if machine-readable evidence exists.
