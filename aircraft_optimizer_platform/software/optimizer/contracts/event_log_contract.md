# Event Log Contract

Date created: 2026-06-20

## Purpose

The event log records what happened in the platform. It is append-oriented and supports reproducibility, debugging, dashboard updates, and auditability.

The event log is not a replacement for structured records. It complements campaigns, candidates, evaluations, modules, artifacts, and failures.

## Event Record

```yaml
event_id: string
campaign_id: string | null
candidate_id: string | null
evaluation_id: string | null
module_attempt_id: string | null
event_type: string
event_time: datetime
actor: optimizer | user | system | module | dashboard
severity: debug | info | warning | error | critical
message: string
payload: map | null
```

## Required Event Types

Campaign:

- `campaign.created`
- `campaign.started`
- `campaign.paused`
- `campaign.resumed`
- `campaign.completed`
- `campaign.failed`
- `campaign.cancelled`

Candidate:

- `candidate.proposed`
- `candidate.registered`
- `candidate.queued`
- `candidate.rejected`
- `candidate.completed`
- `candidate.failed`
- `candidate.annotated`

Geometry provider:

- `geometry.requested`
- `geometry.completed`
- `geometry.failed`

Evaluation:

- `evaluation.created`
- `evaluation.started`
- `evaluation.completed`
- `evaluation.failed`
- `evaluation.partial`
- `evaluation.cancelled`

Module:

- `module.started`
- `module.completed`
- `module.failed`
- `module.skipped`

Artifact:

- `artifact.created`
- `artifact.missing`
- `artifact.archived`

Failure:

- `failure.recorded`

User/dashboard:

- `annotation.created`
- `annotation.resolved`
- `rerun.requested`
- `candidate.flagged`

## Event Payload Rules

Payload should be structured JSON and include only event-specific detail.

Good payload example:

```json
{
  "module_name": "oml_stl_export",
  "preset": "direct_sparse_oml_fast",
  "quality_gates": {
    "max_boundary_edges": 0,
    "max_nonmanifold_edges": 0
  }
}
```

Avoid dumping full logs into event payloads. Store logs as artifacts and link them.

## Append Rule

Events should be appended. Do not rewrite old events except through an explicit repair/migration operation that itself logs an event.

## Dashboard Rule

The dashboard should read events for live progress, but it should read structured tables for current state.
