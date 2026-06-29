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
