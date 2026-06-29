# Evaluation Contract

Date created: 2026-06-20

## Purpose

An evaluation is one execution attempt for a candidate through a specific pipeline version.

A candidate can have multiple evaluations. Reruns, higher-fidelity reruns, and rescoring attempts must create new records instead of overwriting old ones.

## Required Fields

```yaml
evaluation_id: string
candidate_id: string
campaign_id: string
pipeline_id: string
pipeline_version: string
status: queued | running | complete | failed | cancelled | partial
created_at: datetime
started_at: datetime | null
finished_at: datetime | null
requested_by: optimizer | user | system
reason: string
environment_fingerprint_id: string
module_attempts:
  - module_attempt_id: string
summary:
  objective_values: map | null
  constraint_values: map | null
  rank: number | null
  failure_id: string | null
```

## Pipeline Rules

- Evaluation pipeline ID and version are immutable for one evaluation.
- Module order and dependencies must be recorded.
- Partial results are valid and should be stored.
- A failed module can stop the pipeline or mark the evaluation partial depending on pipeline policy.

## Status Rules

- `queued`: evaluation exists but no module has started.
- `running`: one or more modules are active.
- `complete`: pipeline completed successfully.
- `failed`: pipeline stopped due a failure.
- `cancelled`: user or system cancelled the run.
- `partial`: some modules completed, but not enough to mark complete.

## Rerun Rules

Reruns must record:

- Original evaluation ID.
- Reason.
- Requested-by.
- Changed settings.
- New pipeline/config versions if applicable.

Do not mutate old evaluation status except to add audit events.
