# Geometry Provider Contract

Date created: 2026-06-20

## Purpose

The geometry provider converts a candidate definition into a geometry source the evaluation pipeline can consume.

The provider may return a manual Rhai reference, a generated Rhai script, or a future native geometry source. The optimizer core should only depend on this contract.

## Required Input

```yaml
candidate_id: string
campaign_id: string
aircraft_family: string
variable_schema_id: string
design_variables: map
geometry_request:
  requested_representation: rhai_sdf | native_sdf | mesh | other
  requested_role: outer_mold_line | shell | internal_structure | full_geometry
  fidelity_hint: reference | fast | high_fidelity
```

## Required Output

```yaml
geometry_provider_result_id: string
candidate_id: string
evaluation_id: string | null
provider_name: string
provider_version: string
provider_kind: manual_reference | generated_script | external_cad | mock
status: success | failed
geometry_source:
  representation: rhai_sdf
  script_path: string
  feature_name: string
  coordinate_frame: string
  bbox_min_mm: [number, number, number]
  bbox_max_mm: [number, number, number]
  source_hash: string
parameter_trace:
  aircraft_family: string
  variable_schema_id: string
  design_variables: map
  fixed_parameters: map
metadata:
  source_label: string
  source_original_path: string | null
  notes: string | null
warnings:
  - string
failure: failure_record | null
```

## Coordinate Frame

Every geometry provider result must declare a coordinate frame.

Known current frame examples:

- Native scratchpad frame: X longitudinal, Y spanwise, Z vertical.
- Oblique y-slice frame: used by current direct sparse OML exporter references.

Do not assume a frame from filename alone.

## Bbox Rules

- Bbox must be in millimeters.
- Bbox should enclose the returned feature.
- If bbox is inherited from embedded Rhai metadata, record that source.
- If bbox is estimated, record the estimation method.

## Provider Kinds

### manual_reference

Use for curated Rhai files copied into:

```text
software\sdf_generation_manual\curated_rhai
```

### generated_script

Use for future production automatic geometry generation.

Existing `make_*.py` scripts are reference/example material only. Do not treat them as production providers.

### external_cad

Use when calling the existing CAD/SDF system directly.

### mock

Use for v0.1 skeleton tests.

## Non-Goals

The geometry provider should not:

- Export STL.
- Run CFD.
- Score candidates.
- Decide optimizer ranking.
- Store final artifacts directly.

Those are downstream module and artifact registry responsibilities.
