# Database Schema Contract

Date created: 2026-06-20

## Purpose

This contract defines the minimum relational schema for the v0.1 platform skeleton.

The first implementation should use SQLite, but the schema should avoid SQLite-only assumptions where practical so it can migrate to PostgreSQL later.

## Database Position

The database is the local record of truth for:

- Campaigns.
- Candidates.
- Geometry provider results.
- Evaluations.
- Module attempts/results.
- Artifacts.
- Failures.
- Events.
- Lineage.
- Annotations.

The filesystem stores large artifacts. The database stores metadata, paths, hashes, and relationships.

## v0.1 Tables

### campaigns

```sql
campaign_id TEXT PRIMARY KEY,
name TEXT NOT NULL,
description TEXT,
aircraft_family TEXT NOT NULL,
variable_schema_id TEXT NOT NULL,
status TEXT NOT NULL,
created_at TEXT NOT NULL,
updated_at TEXT NOT NULL,
config_json TEXT NOT NULL
```

### variable_schemas

```sql
variable_schema_id TEXT PRIMARY KEY,
aircraft_family TEXT NOT NULL,
schema_version TEXT NOT NULL,
schema_json TEXT NOT NULL,
created_at TEXT NOT NULL
```

### candidates

```sql
candidate_id TEXT PRIMARY KEY,
campaign_id TEXT NOT NULL,
aircraft_family TEXT NOT NULL,
variable_schema_id TEXT NOT NULL,
generation INTEGER NOT NULL,
status TEXT NOT NULL,
created_by TEXT NOT NULL,
created_at TEXT NOT NULL,
design_variables_json TEXT NOT NULL,
normalized_design_vector_json TEXT,
constraints_declared_json TEXT,
lineage_event_id TEXT,
notes TEXT,
FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id),
FOREIGN KEY (variable_schema_id) REFERENCES variable_schemas(variable_schema_id)
```

### candidate_lineage

```sql
lineage_event_id TEXT PRIMARY KEY,
campaign_id TEXT NOT NULL,
child_candidate_id TEXT NOT NULL,
parent_candidate_ids_json TEXT NOT NULL,
operator TEXT NOT NULL,
operator_version TEXT,
reason TEXT NOT NULL,
mutation_summary_json TEXT,
optimizer_state_ref TEXT,
created_at TEXT NOT NULL,
FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id),
FOREIGN KEY (child_candidate_id) REFERENCES candidates(candidate_id)
```

### evaluations

```sql
evaluation_id TEXT PRIMARY KEY,
candidate_id TEXT NOT NULL,
campaign_id TEXT NOT NULL,
pipeline_id TEXT NOT NULL,
pipeline_version TEXT NOT NULL,
status TEXT NOT NULL,
requested_by TEXT NOT NULL,
reason TEXT,
created_at TEXT NOT NULL,
started_at TEXT,
finished_at TEXT,
environment_fingerprint_id TEXT,
summary_json TEXT,
failure_id TEXT,
FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id),
FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id)
```

### geometry_provider_results

```sql
geometry_provider_result_id TEXT PRIMARY KEY,
candidate_id TEXT NOT NULL,
evaluation_id TEXT,
provider_name TEXT NOT NULL,
provider_version TEXT NOT NULL,
provider_kind TEXT NOT NULL,
status TEXT NOT NULL,
representation TEXT NOT NULL,
script_path TEXT,
feature_name TEXT,
coordinate_frame TEXT,
bbox_min_mm_json TEXT,
bbox_max_mm_json TEXT,
source_hash TEXT,
parameter_trace_json TEXT NOT NULL,
metadata_json TEXT,
failure_id TEXT,
created_at TEXT NOT NULL,
FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id),
FOREIGN KEY (evaluation_id) REFERENCES evaluations(evaluation_id)
```

### module_attempts

```sql
module_attempt_id TEXT PRIMARY KEY,
evaluation_id TEXT NOT NULL,
candidate_id TEXT NOT NULL,
module_name TEXT NOT NULL,
module_version TEXT NOT NULL,
module_kind TEXT NOT NULL,
status TEXT NOT NULL,
started_at TEXT,
finished_at TEXT,
runtime_seconds REAL,
inputs_json TEXT NOT NULL,
metrics_json TEXT,
warnings_json TEXT,
metadata_json TEXT,
failure_id TEXT,
FOREIGN KEY (evaluation_id) REFERENCES evaluations(evaluation_id),
FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id)
```

### artifacts

```sql
artifact_id TEXT PRIMARY KEY,
candidate_id TEXT,
evaluation_id TEXT,
module_attempt_id TEXT,
artifact_type TEXT NOT NULL,
path TEXT NOT NULL,
content_hash TEXT,
hash_algorithm TEXT,
created_at TEXT NOT NULL,
producer_module TEXT,
producer_version TEXT,
mime_type TEXT,
size_bytes INTEGER,
metadata_json TEXT,
source_kind TEXT NOT NULL,
source_original_path TEXT,
status TEXT NOT NULL,
FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id),
FOREIGN KEY (evaluation_id) REFERENCES evaluations(evaluation_id),
FOREIGN KEY (module_attempt_id) REFERENCES module_attempts(module_attempt_id)
```

### failures

```sql
failure_id TEXT PRIMARY KEY,
candidate_id TEXT,
evaluation_id TEXT,
module_attempt_id TEXT,
category TEXT NOT NULL,
stage TEXT NOT NULL,
severity TEXT NOT NULL,
retryable INTEGER NOT NULL,
message TEXT NOT NULL,
evidence_artifact_ids_json TEXT,
suggested_next_action TEXT,
created_at TEXT NOT NULL,
metadata_json TEXT
```

### events

```sql
event_id TEXT PRIMARY KEY,
campaign_id TEXT,
candidate_id TEXT,
evaluation_id TEXT,
module_attempt_id TEXT,
event_type TEXT NOT NULL,
event_time TEXT NOT NULL,
actor TEXT NOT NULL,
severity TEXT NOT NULL,
message TEXT NOT NULL,
payload_json TEXT,
FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id),
FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id),
FOREIGN KEY (evaluation_id) REFERENCES evaluations(evaluation_id),
FOREIGN KEY (module_attempt_id) REFERENCES module_attempts(module_attempt_id)
```

### user_annotations

```sql
annotation_id TEXT PRIMARY KEY,
candidate_id TEXT NOT NULL,
evaluation_id TEXT,
user_label TEXT NOT NULL,
tag TEXT NOT NULL,
comment TEXT,
created_at TEXT NOT NULL,
resolved_at TEXT,
metadata_json TEXT,
FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id),
FOREIGN KEY (evaluation_id) REFERENCES evaluations(evaluation_id)
```

## v0.1 Indexes

Recommended indexes:

```sql
CREATE INDEX idx_candidates_campaign ON candidates(campaign_id);
CREATE INDEX idx_evaluations_candidate ON evaluations(candidate_id);
CREATE INDEX idx_evaluations_campaign ON evaluations(campaign_id);
CREATE INDEX idx_module_attempts_evaluation ON module_attempts(evaluation_id);
CREATE INDEX idx_artifacts_evaluation ON artifacts(evaluation_id);
CREATE INDEX idx_artifacts_candidate ON artifacts(candidate_id);
CREATE INDEX idx_events_campaign_time ON events(campaign_id, event_time);
CREATE INDEX idx_failures_category ON failures(category);
```

## JSON Field Rule

SQLite v0.1 may store structured fields as JSON text. These fields should map directly to future typed objects:

- `config_json`
- `schema_json`
- `design_variables_json`
- `parameter_trace_json`
- `metadata_json`
- `payload_json`
- `metrics_json`

Do not store only stringified human prose when a structured object is available.

## Migration Rule

Every schema change after v0.1 should have a migration file and a schema version update.
