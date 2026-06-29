# Automatic Geometry Generator Contract

Date created: 2026-06-20

## Purpose

This document defines the future production contract for converting optimizer candidate variables into a geometry-provider output that downstream modules can consume.

This is not an implementation and does not adopt the existing `make_*.py` scripts as production code. Those scripts remain reference material for prior Rhai emission patterns.

## Boundary

The automatic geometry generator is responsible for:

- Reading a typed aircraft-family schema.
- Reading a candidate design vector.
- Producing a parameterized geometry definition artifact.
- Returning feature name, coordinate frame, bounding box, source hash, and parameter trace.

The generator is not responsible for:

- STL export.
- Mesh validation.
- CFD.
- Scoring.
- Optimizer ask/tell logic.
- Dashboard state.

## v1 Aircraft Family Scope

```yaml
aircraft_family: fixed_wing_uav_reference
topology_policy: fixed_topology
mission_focus: loiter_endurance
variables: continuous_only
```

Initial v1 must not add or remove wings, fins, nacelles, propulsion elements, or aircraft families. Future topology/configuration changes should be modeled as a higher-level family/configuration schema, not as hidden generator behavior.

## Generator Specification

```yaml
generator_name: fixed_wing_uav_rhai_generator
generator_version: string
generator_kind: automatic_sdf_definition
input_contract_version: geometry_generator_request.v1
output_contract_version: geometry_provider_result.v1
deterministic: true
cache_policy: input_hash
```

## Request Schema

```yaml
request_id: string
candidate_id: string
evaluation_id: string | null
aircraft_family: fixed_wing_uav_reference
family_schema_version: string
design_variables:
  wing.span_mm:
    value: number
    unit: mm
  wing.root_chord_mm:
    value: number
    unit: mm
  wing.tip_chord_mm:
    value: number
    unit: mm
  wing.sweep_deg:
    value: number
    unit: deg
normalized_design_vector:
  variable_name: number
constraints:
  variable_name:
    min: number | null
    max: number | null
coordinate_policy:
  requested_frame: native | platform
  units: mm
output_policy:
  representation: rhai_sdf
  output_directory: string
  include_debug_trace: boolean
metadata:
  requested_by: optimizer | human | test
  reason: string | null
```

The request should include every variable needed to reproduce the generated definition. Defaults must be captured explicitly in the emitted parameter trace.

## Result Schema

```yaml
geometry_provider_result_id: string
candidate_id: string
evaluation_id: string | null
provider_name: fixed_wing_uav_rhai_generator
provider_version: string
provider_kind: automatic_generator
status: success | failed | skipped
representation: rhai_sdf
script_path: string
feature_name: string
coordinate_frame: native | platform
bbox_min_mm: [number, number, number]
bbox_max_mm: [number, number, number]
source_hash: string
parameter_trace:
  aircraft_family: string
  family_schema_version: string
  generator_version: string
  design_variables: map
  normalized_design_vector: map
  derived_parameters: map
  defaults_applied: map
  constraints_checked: map
metadata:
  generator_source_hash: string
  generator_entrypoint: string
  source_template_hash: string | null
  warnings:
    - string
failure: failure_record | null
artifact_ids:
  - string
```

## Required Artifacts

| Artifact kind | Required | Purpose |
|---|---:|---|
| `geometry_source_rhai` | Yes | Generated geometry source consumed by exporter/validation modules. |
| `geometry_parameter_trace_json` | Yes | Complete variable/default/derived-parameter trace. |
| `geometry_generator_log` | Optional | Human-readable generator log. |
| `geometry_debug_snapshot` | Optional | Debug-only intermediate record. |

## Initial Variable Groups

### Wing

- `wing.span_mm`
- `wing.root_chord_mm`
- `wing.tip_chord_mm`
- `wing.sweep_deg`
- `wing.twist_root_deg`
- `wing.twist_tip_deg`
- `wing.thickness_root_ratio`
- `wing.thickness_tip_ratio`
- `wing.position_x_mm`
- `wing.position_z_mm`

### Fuselage

- `fuselage.length_mm`
- `fuselage.diameter_mm`
- `fuselage.scale_x`
- `fuselage.scale_y`
- `fuselage.scale_z`
- `fuselage.cross_section_blend`

### Inlets

- `inlet.size_scale`
- `inlet.position_x_mm`
- `inlet.position_y_mm`
- `inlet.position_z_mm`
- `inlet.lip_radius_mm`

This list is a contract seed, not a final validated aircraft model. Bounds and coupling rules must be defined before optimization uses these variables.

## Validation Responsibilities

The generator should perform cheap pre-export checks:

- Required variables present.
- Units match schema.
- Normalized values are in `[0, 1]` when present.
- Physical values are inside declared bounds.
- Fixed topology assumptions are respected.
- Required feature name is emitted.
- Bounding box is finite and correctly ordered.
- Generated source is non-empty and hashable.

It should not perform mesh topology validation. That belongs to export or geometry-validation modules.

## Failure Mapping

| Failure code | Category | Stage | Retryable | Meaning |
|---|---|---|---:|---|
| `geometry_schema.missing_variable` | geometry_schema | generation_input | No | Required variable is absent. |
| `geometry_schema.invalid_unit` | geometry_schema | generation_input | No | Variable unit does not match schema. |
| `geometry_schema.out_of_bounds` | geometry_schema | generation_input | No | Physical or normalized value violates bounds. |
| `geometry_schema.topology_policy_violation` | geometry_schema | generation_input | No | Request attempts a disallowed topology/configuration change. |
| `geometry_generation.template_error` | geometry_generation | generation | Maybe | Generator template could not render. |
| `geometry_generation.invalid_feature` | geometry_generation | generation | No | Required feature name was not produced. |
| `geometry_generation.invalid_bbox` | geometry_generation | generation | No | Bounding box is missing, invalid, or non-finite. |
| `artifact.missing_expected_artifact` | artifact | registration | Maybe | Generated source or trace artifact was not produced. |

## Current Reference Scripts

Reference-only scripts:

```text
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_rhai.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_wingtips_rhai.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_twin_side_inlet_aircraft_rhai.py
```

Use them to understand previous parameterization patterns only. Do not treat them as production generator code without review, tests, provenance cleanup, and contract conformance.

## Implementation Phases

1. Schema-only: define fixed-wing variable schema, bounds, and derived parameters.
2. Fixture generator: emit one deterministic checked-in Rhai source from a candidate record.
3. Validation-only adapter: persist generated source, trace, and metadata without calling exporter.
4. Export handoff: pass geometry-provider result to the OML STL adapter.
5. Family expansion: add VTOL/tailsitter schemas only after fixed-wing schema is stable.

## Current Status

Status: contract defined, implementation not started.

The next implementation step is a versioned fixed-wing variable schema and a no-execution fixture generator that produces a traceable geometry-provider result.
