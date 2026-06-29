# Platform Interface Overview

Date created: 2026-06-20

## Position

The aircraft optimizer platform is centered on candidates, evaluations, modules, artifacts, and traceable records.

The OML STL exporter is a module inside the evaluation pipeline. It is not the platform root and should not define the global architecture.

Core flow:

```text
Campaign
  -> Candidate
  -> Geometry Provider
  -> Evaluation Pipeline
      -> Module: geometry validation
      -> Module: geometry metrics
      -> Module: OML STL export
      -> Module: mass/CG estimate
      -> Module: aero analysis
      -> Module: scoring
  -> Artifact Registry
  -> Run Database
  -> Dashboard / Reports
```

## Core Records

Minimum platform records:

- Candidate record.
- Geometry provider result.
- Evaluation record.
- Artifact record.
- Module result.
- Failure record.
- Lineage record.
- Event log record.

## Interface Boundaries

### Optimizer Core

Owns:

- Campaign configuration.
- Candidate creation.
- Variable schemas.
- Candidate lineage.
- Evaluation scheduling.
- Objective/constraint values.
- Optimizer state.

Does not own:

- CAD internals.
- Exporter internals.
- CFD internals.
- Dashboard-only state.

### Geometry Provider

Owns:

- Translating a candidate definition into a geometry source.
- Returning a Rhai path, feature name, coordinate frame, bbox, and parameter trace.

Does not own:

- STL export.
- CFD setup.
- Scoring.

### Module Adapters

Own:

- One bounded task.
- Typed inputs and outputs.
- Runtime metadata.
- Failure classification.
- Artifact production.

Examples:

- OML STL exporter.
- Geometry metrics.
- AVL adapter.
- SU2 adapter.
- Screenshot generator.
- Scoring module.

### Artifact Registry

Owns:

- Artifact identity.
- Paths.
- Hashes.
- Producer metadata.
- Candidate/evaluation links.

Does not own:

- Interpretation of analysis results.
- Optimizer state.

## Design Rules

- Every module consumes platform records and returns platform records.
- No module mutates candidate definitions.
- No module overwrites previous results.
- Reruns create new evaluation/module-result records.
- Failures are records, not missing data.
- Exporter-specific fields live in exporter module metadata, not in the candidate core.
- Dashboard actions write audited platform events.

## First Implementation Target

v0.1 should implement these records with mock modules before wiring real exporter or analysis tools.

Minimum v0.1 platform skeleton:

- Candidate record.
- Geometry provider result record.
- Evaluation record.
- Artifact record.
- Module result record.
- Event log record.
- Mock geometry provider.
- Mock module adapter.
