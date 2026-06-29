# Candidate Contract

Date created: 2026-06-20

## Purpose

A candidate is a proposed aircraft design inside a campaign. It is defined by an aircraft family, a variable schema, and a design vector.

A candidate is not geometry. Geometry is produced later by a geometry provider.

## Required Fields

```yaml
candidate_id: string
campaign_id: string
aircraft_family: string
variable_schema_id: string
created_at: datetime
created_by: optimizer | user | import | seed | test
generation: integer
status: proposed | registered | evaluating | complete | failed | archived
design_variables:
  variable_name:
    value: number | string | boolean
    unit: string
normalized_design_vector:
  variable_name: number
constraints_declared:
  constraint_name:
    value: any
parent_candidate_ids:
  - string
lineage_event_id: string | null
notes: string | null
```

## Design Variable Rules

- v1 variables are continuous only.
- Units are required for dimensional variables.
- Variables must refer to a frozen variable schema.
- Topology changes are not allowed in v1.
- Aircraft family changes are not allowed inside one v1 campaign.

## Candidate Status

Recommended status values:

- `proposed`
- `registered`
- `queued`
- `evaluating`
- `complete`
- `failed`
- `rejected_by_bounds`
- `rejected_by_constraints`
- `archived`

## Lineage

Lineage is explicit. Do not infer parentage from generation number only.

Lineage event should capture:

- Parent candidate IDs.
- Operator.
- Operator version.
- Mutation/crossover details.
- Reason.
- Optimizer state reference.

Example:

```yaml
lineage_event:
  child_candidate_id: candidate_0108
  parent_candidate_ids:
    - candidate_0042
  operator: gaussian_mutation
  operator_version: 1.0.0
  reason: optimizer_exploration
  mutation_summary:
    - variable: wing.sweep_deg
      delta: 3.0
      unit: deg
    - variable: wing.taper_ratio
      delta: -0.05
      unit: dimensionless
```

## Non-Goals

Candidate records should not contain:

- STL paths.
- CFD solver settings.
- Exporter-specific tile settings.
- Dashboard-only notes as core fields.
- Analysis result values.

Those belong to geometry provider results, module results, artifacts, annotations, or scoring records.
