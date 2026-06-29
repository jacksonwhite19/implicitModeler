PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS campaigns (
    campaign_id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    description TEXT,
    aircraft_family TEXT NOT NULL,
    variable_schema_id TEXT NOT NULL,
    status TEXT NOT NULL,
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL,
    config_json TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS variable_schemas (
    variable_schema_id TEXT PRIMARY KEY,
    aircraft_family TEXT NOT NULL,
    schema_version TEXT NOT NULL,
    schema_json TEXT NOT NULL,
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS candidates (
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
);

CREATE TABLE IF NOT EXISTS candidate_lineage (
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
);

CREATE TABLE IF NOT EXISTS optimizer_runs (
    optimizer_run_id TEXT PRIMARY KEY,
    campaign_id TEXT NOT NULL,
    optimizer_name TEXT NOT NULL,
    optimizer_version TEXT NOT NULL,
    status TEXT NOT NULL,
    objective_json TEXT NOT NULL,
    settings_json TEXT NOT NULL,
    created_at TEXT NOT NULL,
    started_at TEXT,
    finished_at TEXT,
    summary_json TEXT,
    FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id)
);

CREATE TABLE IF NOT EXISTS optimizer_iterations (
    optimizer_iteration_id TEXT PRIMARY KEY,
    optimizer_run_id TEXT NOT NULL,
    campaign_id TEXT NOT NULL,
    iteration_index INTEGER NOT NULL,
    status TEXT NOT NULL,
    proposed_candidate_ids_json TEXT NOT NULL,
    parent_candidate_ids_json TEXT NOT NULL,
    strategy TEXT NOT NULL,
    rationale TEXT NOT NULL,
    optimizer_state_json TEXT NOT NULL,
    created_at TEXT NOT NULL,
    completed_at TEXT,
    FOREIGN KEY (optimizer_run_id) REFERENCES optimizer_runs(optimizer_run_id),
    FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id)
);

CREATE TABLE IF NOT EXISTS candidate_runner_states (
    candidate_id TEXT PRIMARY KEY,
    campaign_id TEXT NOT NULL,
    optimizer_run_id TEXT,
    optimizer_iteration_id TEXT,
    evaluation_id TEXT,
    state TEXT NOT NULL,
    stage TEXT NOT NULL,
    active_module TEXT,
    progress REAL,
    priority INTEGER,
    queued_at TEXT,
    started_at TEXT,
    updated_at TEXT NOT NULL,
    finished_at TEXT,
    reason TEXT,
    message TEXT,
    metadata_json TEXT NOT NULL,
    FOREIGN KEY (candidate_id) REFERENCES candidates(candidate_id),
    FOREIGN KEY (campaign_id) REFERENCES campaigns(campaign_id),
    FOREIGN KEY (optimizer_run_id) REFERENCES optimizer_runs(optimizer_run_id),
    FOREIGN KEY (optimizer_iteration_id) REFERENCES optimizer_iterations(optimizer_iteration_id),
    FOREIGN KEY (evaluation_id) REFERENCES evaluations(evaluation_id)
);

CREATE TABLE IF NOT EXISTS evaluations (
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
);

CREATE TABLE IF NOT EXISTS environment_fingerprints (
    environment_fingerprint_id TEXT PRIMARY KEY,
    created_at TEXT NOT NULL,
    platform_json TEXT NOT NULL,
    python_json TEXT NOT NULL,
    tools_json TEXT NOT NULL,
    notes TEXT
);

CREATE TABLE IF NOT EXISTS geometry_provider_results (
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
);

CREATE TABLE IF NOT EXISTS module_attempts (
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
);

CREATE TABLE IF NOT EXISTS artifacts (
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
);

CREATE TABLE IF NOT EXISTS failures (
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
);

CREATE TABLE IF NOT EXISTS events (
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
);

CREATE TABLE IF NOT EXISTS user_annotations (
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
);

CREATE INDEX IF NOT EXISTS idx_candidates_campaign ON candidates(campaign_id);
CREATE INDEX IF NOT EXISTS idx_optimizer_runs_campaign ON optimizer_runs(campaign_id);
CREATE INDEX IF NOT EXISTS idx_optimizer_iterations_run ON optimizer_iterations(optimizer_run_id);
CREATE INDEX IF NOT EXISTS idx_candidate_runner_states_campaign ON candidate_runner_states(campaign_id);
CREATE INDEX IF NOT EXISTS idx_candidate_runner_states_state ON candidate_runner_states(state, stage);
CREATE INDEX IF NOT EXISTS idx_evaluations_candidate ON evaluations(candidate_id);
CREATE INDEX IF NOT EXISTS idx_evaluations_campaign ON evaluations(campaign_id);
CREATE INDEX IF NOT EXISTS idx_evaluations_environment ON evaluations(environment_fingerprint_id);
CREATE INDEX IF NOT EXISTS idx_module_attempts_evaluation ON module_attempts(evaluation_id);
CREATE INDEX IF NOT EXISTS idx_artifacts_evaluation ON artifacts(evaluation_id);
CREATE INDEX IF NOT EXISTS idx_artifacts_candidate ON artifacts(candidate_id);
CREATE INDEX IF NOT EXISTS idx_events_campaign_time ON events(campaign_id, event_time);
CREATE INDEX IF NOT EXISTS idx_failures_category ON failures(category);
