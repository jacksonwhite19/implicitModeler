from __future__ import annotations

import sqlite3
import sys
import platform
from typing import Any

from aircraft_optimizer.records import FailureDraft
from aircraft_optimizer.ids import new_id
from aircraft_optimizer.jsonutil import dumps
from aircraft_optimizer.time import utc_now_iso


def create_variable_schema(
    connection: sqlite3.Connection,
    *,
    aircraft_family: str,
    schema_version: str,
    schema: dict[str, Any],
) -> str:
    variable_schema_id = new_id("varschema")
    connection.execute(
        """
        INSERT INTO variable_schemas (
            variable_schema_id, aircraft_family, schema_version, schema_json, created_at
        )
        VALUES (?, ?, ?, ?, ?)
        """,
        (variable_schema_id, aircraft_family, schema_version, dumps(schema), utc_now_iso()),
    )
    return variable_schema_id


def create_campaign(
    connection: sqlite3.Connection,
    *,
    name: str,
    description: str,
    aircraft_family: str,
    variable_schema_id: str,
    config: dict[str, Any],
) -> str:
    now = utc_now_iso()
    campaign_id = new_id("campaign")
    connection.execute(
        """
        INSERT INTO campaigns (
            campaign_id, name, description, aircraft_family, variable_schema_id,
            status, created_at, updated_at, config_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            campaign_id,
            name,
            description,
            aircraft_family,
            variable_schema_id,
            "running",
            now,
            now,
            dumps(config),
        ),
    )
    return campaign_id


def complete_campaign(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    status: str = "complete",
) -> None:
    connection.execute(
        """
        UPDATE campaigns
        SET status = ?, updated_at = ?
        WHERE campaign_id = ?
        """,
        (status, utc_now_iso(), campaign_id),
    )


def create_optimizer_run(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    optimizer_name: str,
    optimizer_version: str,
    objective: dict[str, Any],
    settings: dict[str, Any],
) -> str:
    now = utc_now_iso()
    optimizer_run_id = new_id("optrun")
    connection.execute(
        """
        INSERT INTO optimizer_runs (
            optimizer_run_id, campaign_id, optimizer_name, optimizer_version,
            status, objective_json, settings_json, created_at, started_at
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            optimizer_run_id,
            campaign_id,
            optimizer_name,
            optimizer_version,
            "running",
            dumps(objective),
            dumps(settings),
            now,
            now,
        ),
    )
    return optimizer_run_id


def complete_optimizer_run(
    connection: sqlite3.Connection,
    *,
    optimizer_run_id: str,
    summary: dict[str, Any],
) -> None:
    connection.execute(
        """
        UPDATE optimizer_runs
        SET status = ?, finished_at = ?, summary_json = ?
        WHERE optimizer_run_id = ?
        """,
        ("complete", utc_now_iso(), dumps(summary), optimizer_run_id),
    )


def create_optimizer_iteration(
    connection: sqlite3.Connection,
    *,
    optimizer_run_id: str,
    campaign_id: str,
    iteration_index: int,
    parent_candidate_ids: list[str],
    proposed_candidate_ids: list[str],
    strategy: str,
    rationale: str,
    optimizer_state: dict[str, Any],
) -> str:
    optimizer_iteration_id = new_id("optiter")
    connection.execute(
        """
        INSERT INTO optimizer_iterations (
            optimizer_iteration_id, optimizer_run_id, campaign_id, iteration_index,
            status, proposed_candidate_ids_json, parent_candidate_ids_json,
            strategy, rationale, optimizer_state_json, created_at
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            optimizer_iteration_id,
            optimizer_run_id,
            campaign_id,
            iteration_index,
            "proposed",
            dumps(proposed_candidate_ids),
            dumps(parent_candidate_ids),
            strategy,
            rationale,
            dumps(optimizer_state),
            utc_now_iso(),
        ),
    )
    return optimizer_iteration_id


def complete_optimizer_iteration(
    connection: sqlite3.Connection,
    *,
    optimizer_iteration_id: str,
    proposed_candidate_ids: list[str],
    optimizer_state: dict[str, Any],
) -> None:
    connection.execute(
        """
        UPDATE optimizer_iterations
        SET status = ?, completed_at = ?, proposed_candidate_ids_json = ?,
            optimizer_state_json = ?
        WHERE optimizer_iteration_id = ?
        """,
        (
            "complete",
            utc_now_iso(),
            dumps(proposed_candidate_ids),
            dumps(optimizer_state),
            optimizer_iteration_id,
        ),
    )


def create_environment_fingerprint(
    connection: sqlite3.Connection,
    *,
    tools: dict[str, Any],
    notes: str | None = None,
) -> str:
    environment_fingerprint_id = new_id("env")
    connection.execute(
        """
        INSERT INTO environment_fingerprints (
            environment_fingerprint_id, created_at, platform_json,
            python_json, tools_json, notes
        )
        VALUES (?, ?, ?, ?, ?, ?)
        """,
        (
            environment_fingerprint_id,
            utc_now_iso(),
            dumps(
                {
                    "system": platform.system(),
                    "release": platform.release(),
                    "version": platform.version(),
                    "machine": platform.machine(),
                }
            ),
            dumps(
                {
                    "version": sys.version,
                    "executable": sys.executable,
                }
            ),
            dumps(tools),
            notes,
        ),
    )
    return environment_fingerprint_id


def create_candidate(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    aircraft_family: str,
    variable_schema_id: str,
    design_variables: dict[str, Any],
    normalized_design_vector: dict[str, Any],
    created_by: str = "seed",
    generation: int = 0,
    notes: str | None = None,
) -> str:
    candidate_id = new_id("candidate")
    connection.execute(
        """
        INSERT INTO candidates (
            candidate_id, campaign_id, aircraft_family, variable_schema_id,
            generation, status, created_by, created_at, design_variables_json,
            normalized_design_vector_json, constraints_declared_json,
            lineage_event_id, notes
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            candidate_id,
            campaign_id,
            aircraft_family,
            variable_schema_id,
            generation,
            "registered",
            created_by,
            utc_now_iso(),
            dumps(design_variables),
            dumps(normalized_design_vector),
            dumps({}),
            None,
            notes,
        ),
    )
    return candidate_id


def update_candidate_status(
    connection: sqlite3.Connection,
    *,
    candidate_id: str,
    status: str,
) -> None:
    connection.execute(
        """
        UPDATE candidates
        SET status = ?
        WHERE candidate_id = ?
        """,
        (status, candidate_id),
    )


def upsert_candidate_runner_state(
    connection: sqlite3.Connection,
    *,
    candidate_id: str,
    campaign_id: str,
    state: str,
    stage: str,
    optimizer_run_id: str | None = None,
    optimizer_iteration_id: str | None = None,
    evaluation_id: str | None = None,
    active_module: str | None = None,
    progress: float | None = None,
    priority: int | None = None,
    reason: str | None = None,
    message: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> None:
    now = utc_now_iso()
    existing = connection.execute(
        """
        SELECT queued_at, started_at
        FROM candidate_runner_states
        WHERE candidate_id = ?
        """,
        (candidate_id,),
    ).fetchone()
    queued_at = existing[0] if existing else None
    started_at = existing[1] if existing else None
    if queued_at is None and state == "queued":
        queued_at = now
    if started_at is None and state in {"running", "meshing", "cfd_running"}:
        started_at = now
    finished_at = now if state in {"scored", "promoted", "rejected", "failed"} else None
    connection.execute(
        """
        INSERT INTO candidate_runner_states (
            candidate_id, campaign_id, optimizer_run_id, optimizer_iteration_id,
            evaluation_id, state, stage, active_module, progress, priority,
            queued_at, started_at, updated_at, finished_at, reason, message,
            metadata_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(candidate_id) DO UPDATE SET
            campaign_id = excluded.campaign_id,
            optimizer_run_id = excluded.optimizer_run_id,
            optimizer_iteration_id = excluded.optimizer_iteration_id,
            evaluation_id = excluded.evaluation_id,
            state = excluded.state,
            stage = excluded.stage,
            active_module = excluded.active_module,
            progress = excluded.progress,
            priority = excluded.priority,
            queued_at = COALESCE(candidate_runner_states.queued_at, excluded.queued_at),
            started_at = COALESCE(candidate_runner_states.started_at, excluded.started_at),
            updated_at = excluded.updated_at,
            finished_at = excluded.finished_at,
            reason = excluded.reason,
            message = excluded.message,
            metadata_json = excluded.metadata_json
        """,
        (
            candidate_id,
            campaign_id,
            optimizer_run_id,
            optimizer_iteration_id,
            evaluation_id,
            state,
            stage,
            active_module,
            progress,
            priority,
            queued_at,
            started_at,
            now,
            finished_at,
            reason,
            message,
            dumps(metadata or {}),
        ),
    )


def create_candidate_lineage(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    child_candidate_id: str,
    parent_candidate_ids: list[str],
    operator: str,
    reason: str,
    operator_version: str | None = None,
    mutation_summary: dict[str, Any] | None = None,
    optimizer_state_ref: str | None = None,
) -> str:
    lineage_event_id = new_id("lineage")
    connection.execute(
        """
        INSERT INTO candidate_lineage (
            lineage_event_id, campaign_id, child_candidate_id,
            parent_candidate_ids_json, operator, operator_version, reason,
            mutation_summary_json, optimizer_state_ref, created_at
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            lineage_event_id,
            campaign_id,
            child_candidate_id,
            dumps(parent_candidate_ids),
            operator,
            operator_version,
            reason,
            dumps(mutation_summary or {}),
            optimizer_state_ref,
            utc_now_iso(),
        ),
    )
    connection.execute(
        """
        UPDATE candidates
        SET lineage_event_id = ?
        WHERE candidate_id = ?
        """,
        (lineage_event_id, child_candidate_id),
    )
    return lineage_event_id


def create_evaluation(
    connection: sqlite3.Connection,
    *,
    candidate_id: str,
    campaign_id: str,
    pipeline_id: str,
    pipeline_version: str,
    requested_by: str,
    reason: str,
    environment_fingerprint_id: str | None = None,
) -> str:
    now = utc_now_iso()
    evaluation_id = new_id("evaluation")
    connection.execute(
        """
        INSERT INTO evaluations (
            evaluation_id, candidate_id, campaign_id, pipeline_id, pipeline_version,
            status, requested_by, reason, created_at, started_at,
            environment_fingerprint_id
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            evaluation_id,
            candidate_id,
            campaign_id,
            pipeline_id,
            pipeline_version,
            "running",
            requested_by,
            reason,
            now,
            now,
            environment_fingerprint_id,
        ),
    )
    return evaluation_id


def complete_evaluation(
    connection: sqlite3.Connection,
    *,
    evaluation_id: str,
    summary: dict[str, Any],
) -> None:
    connection.execute(
        """
        UPDATE evaluations
        SET status = ?, finished_at = ?, summary_json = ?
        WHERE evaluation_id = ?
        """,
        ("complete", utc_now_iso(), dumps(summary), evaluation_id),
    )


def fail_evaluation(
    connection: sqlite3.Connection,
    *,
    evaluation_id: str,
    failure_id: str,
    summary: dict[str, Any],
) -> None:
    connection.execute(
        """
        UPDATE evaluations
        SET status = ?, finished_at = ?, summary_json = ?, failure_id = ?
        WHERE evaluation_id = ?
        """,
        ("failed", utc_now_iso(), dumps(summary), failure_id, evaluation_id),
    )


def create_geometry_provider_result(
    connection: sqlite3.Connection,
    *,
    candidate_id: str,
    evaluation_id: str,
    provider_name: str,
    provider_version: str,
    provider_kind: str,
    representation: str,
    script_path: str,
    feature_name: str,
    coordinate_frame: str,
    bbox_min_mm: list[float],
    bbox_max_mm: list[float],
    source_hash: str,
    parameter_trace: dict[str, Any],
    metadata: dict[str, Any],
) -> str:
    result_id = new_id("geometry")
    connection.execute(
        """
        INSERT INTO geometry_provider_results (
            geometry_provider_result_id, candidate_id, evaluation_id,
            provider_name, provider_version, provider_kind, status,
            representation, script_path, feature_name, coordinate_frame,
            bbox_min_mm_json, bbox_max_mm_json, source_hash,
            parameter_trace_json, metadata_json, created_at
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            result_id,
            candidate_id,
            evaluation_id,
            provider_name,
            provider_version,
            provider_kind,
            "success",
            representation,
            script_path,
            feature_name,
            coordinate_frame,
            dumps(bbox_min_mm),
            dumps(bbox_max_mm),
            source_hash,
            dumps(parameter_trace),
            dumps(metadata),
            utc_now_iso(),
        ),
    )
    return result_id


def create_module_attempt(
    connection: sqlite3.Connection,
    *,
    evaluation_id: str,
    candidate_id: str,
    module_name: str,
    module_version: str,
    module_kind: str,
    inputs: dict[str, Any],
) -> str:
    attempt_id = new_id("module")
    connection.execute(
        """
        INSERT INTO module_attempts (
            module_attempt_id, evaluation_id, candidate_id, module_name,
            module_version, module_kind, status, started_at, inputs_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            attempt_id,
            evaluation_id,
            candidate_id,
            module_name,
            module_version,
            module_kind,
            "running",
            utc_now_iso(),
            dumps(inputs),
        ),
    )
    return attempt_id


def complete_module_attempt(
    connection: sqlite3.Connection,
    *,
    module_attempt_id: str,
    metrics: dict[str, Any],
    warnings: list[str] | None = None,
    metadata: dict[str, Any] | None = None,
    runtime_seconds: float = 0.0,
) -> None:
    connection.execute(
        """
        UPDATE module_attempts
        SET status = ?, finished_at = ?, runtime_seconds = ?,
            metrics_json = ?, warnings_json = ?, metadata_json = ?
        WHERE module_attempt_id = ?
        """,
        (
            "success",
            utc_now_iso(),
            runtime_seconds,
            dumps(metrics),
            dumps(warnings or []),
            dumps(metadata or {}),
            module_attempt_id,
        ),
    )


def fail_module_attempt(
    connection: sqlite3.Connection,
    *,
    module_attempt_id: str,
    failure_id: str,
    metrics: dict[str, Any],
    warnings: list[str] | None = None,
    metadata: dict[str, Any] | None = None,
    runtime_seconds: float = 0.0,
) -> None:
    connection.execute(
        """
        UPDATE module_attempts
        SET status = ?, finished_at = ?, runtime_seconds = ?,
            metrics_json = ?, warnings_json = ?, metadata_json = ?, failure_id = ?
        WHERE module_attempt_id = ?
        """,
        (
            "failed",
            utc_now_iso(),
            runtime_seconds,
            dumps(metrics),
            dumps(warnings or []),
            dumps(metadata or {}),
            failure_id,
            module_attempt_id,
        ),
    )


def create_failure(
    connection: sqlite3.Connection,
    *,
    candidate_id: str | None,
    evaluation_id: str | None,
    module_attempt_id: str | None,
    failure: FailureDraft,
    evidence_artifact_ids: list[str] | None = None,
) -> str:
    failure.validate()
    failure_id = new_id("failure")
    evidence_ids = evidence_artifact_ids or failure.evidence_artifact_ids
    connection.execute(
        """
        INSERT INTO failures (
            failure_id, candidate_id, evaluation_id, module_attempt_id,
            category, stage, severity, retryable, message,
            evidence_artifact_ids_json, suggested_next_action, created_at,
            metadata_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            failure_id,
            candidate_id,
            evaluation_id,
            module_attempt_id,
            failure.category,
            failure.stage,
            failure.severity,
            1 if failure.retryable else 0,
            failure.message,
            dumps(evidence_ids),
            failure.suggested_next_action,
            utc_now_iso(),
            dumps(failure.metadata or {}),
        ),
    )
    return failure_id


def create_user_annotation(
    connection: sqlite3.Connection,
    *,
    candidate_id: str,
    user_label: str,
    tag: str,
    evaluation_id: str | None = None,
    comment: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> str:
    annotation_id = new_id("annotation")
    connection.execute(
        """
        INSERT INTO user_annotations (
            annotation_id, candidate_id, evaluation_id, user_label, tag,
            comment, created_at, resolved_at, metadata_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            annotation_id,
            candidate_id,
            evaluation_id,
            user_label,
            tag,
            comment,
            utc_now_iso(),
            None,
            dumps(metadata or {}),
        ),
    )
    return annotation_id
