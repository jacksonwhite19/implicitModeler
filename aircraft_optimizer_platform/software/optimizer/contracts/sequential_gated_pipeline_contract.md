# Sequential Gated Pipeline Contract

This contract defines the primary optimizer execution model for the aircraft optimizer platform.

The default autonomous workflow is sequential:

```text
propose one design
cheap pre-export screening
geometry provider
policy-gated OML STL export
export quality gate
policy-gated analysis
record results
update optimizer state
propose next design
```

The platform should not batch-export every proposed candidate by default. Batch and DOE modes are useful later for parallel studies, surrogate dataset generation, stress tests, and dashboard demonstrations, but they are secondary workflows.

## Required Stage Order

1. `candidate_proposal`
   - Optimizer proposes one candidate from the current run state.
   - Candidate lineage records parent IDs, mutation deltas, and proposal rationale.

2. `variable_schema_validation`
   - Candidate variables are checked against the active aircraft-family schema.
   - v1 accepts continuous fixed-wing UAV variables only.

3. `pre_export_screening`
   - Cheap checks run before geometry export.
   - Initial checks should include estimated mass, estimated CG, span/packaging bounds, wing-loading proxy, static-margin proxy, variable sanity, and export-risk flags.
   - Candidates that fail here are preserved as failed/skipped evaluations.

4. `geometry_provider`
   - The existing implicit CAD/SDF system receives the accepted candidate definition through an adapter.
   - The optimizer does not own CAD internals.

5. `oml_stl_export`
   - Real export is policy-gated per candidate.
   - The normal iteration preset is `direct_sparse_oml_fast`.
   - Higher-fidelity export is an explicit rerun/finalization action for promising designs.

6. `export_quality_gate`
   - Export quality gates are recorded as structured metrics and failures.
   - Production CFD-ready defaults remain strict: zero boundary edges, zero nonmanifold edges, one connected component, zero duplicate triangles, and no known-section long chords.

7. `analysis`
   - Analysis is policy-gated and fidelity-aware.
   - CFD is not the default next action after every export. It runs only when the active policy requests it.

8. `results_recording`
   - Every module result, artifact, failure, runtime, and version reference is appended to the run database.

9. `optimizer_state_update`
   - The optimizer consumes the recorded result and proposes the next candidate.
   - Results from failed or skipped candidates remain useful training and decision data.

## Policy Rules

- `candidate_batch_size` is `1` for the primary workflow.
- Real export defaults to disabled until a candidate passes pre-export gates and the active policy enables export.
- Real CFD defaults to disabled.
- Failed candidates must be preserved.
- Reruns and rescoring create new records instead of overwriting old results.
- Batch sweeps such as `run-wing-options` are diagnostic studies, not the default autonomous optimizer loop.

