from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

from aircraft_optimizer.artifacts.registry import (
    register_artifact,
    register_artifact_reference,
)
from aircraft_optimizer.db.repositories import (
    complete_module_attempt,
    create_failure,
    create_module_attempt,
    fail_module_attempt,
)
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.modules.mesh_result_validation import (
    MODULE_NAME,
    MODULE_VERSION,
    validate_mesh_result,
)
from aircraft_optimizer.records import FailureDraft


def persist_mesh_result_validation(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    mesh_path: Path,
    mesh_format: str,
    su2_log_path: Path | None = None,
    semantic_to_solver_markers: dict[str, list[str]] | None = None,
    upstream_module_attempt_id: str | None = None,
    requested_by: str = "system",
) -> str:
    inputs = {
        "mesh_path": str(mesh_path),
        "mesh_format": mesh_format,
        "su2_log_path": str(su2_log_path) if su2_log_path else None,
        "semantic_to_solver_markers": semantic_to_solver_markers,
        "upstream_module_attempt_id": upstream_module_attempt_id,
        "requested_by": requested_by,
    }
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind="mesh_validation",
        inputs=inputs,
    )
    log_event(
        connection,
        event_type="module.started",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message="Started CFD mesh-result validation.",
    )

    result = validate_mesh_result(
        mesh_path=mesh_path,
        mesh_format=mesh_format,
        su2_log_path=su2_log_path,
        semantic_to_solver_markers=semantic_to_solver_markers,
    )
    artifact_ids = _register_mesh_validation_artifacts(
        connection,
        artifact_root=artifact_root,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        mesh_path=mesh_path,
        su2_log_path=su2_log_path,
        result_payload={
            "passed": result.passed,
            "mesh_result": result.mesh_result,
            "metrics": result.metrics,
            "metadata": result.metadata,
            "warnings": result.warnings,
        },
    )
    for artifact_id in artifact_ids:
        log_event(
            connection,
            event_type="artifact.created",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            actor="module",
            message="Registered CFD mesh-result validation artifact.",
            payload={"artifact_id": artifact_id},
        )

    metadata = {
        **result.metadata,
        "artifact_ids": artifact_ids,
        "mesh_result": result.mesh_result,
    }
    if result.passed:
        complete_module_attempt(
            connection,
            module_attempt_id=module_attempt_id,
            metrics=result.metrics,
            warnings=result.warnings,
            metadata=metadata,
            runtime_seconds=0.0,
        )
        log_event(
            connection,
            event_type="module.completed",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            actor="module",
            message="Completed CFD mesh-result validation.",
        )
        return module_attempt_id

    failure = FailureDraft(
        category="meshing.volume_mesh_invalid",
        stage="pre_solver_mesh_validation",
        severity="recoverable",
        retryable=True,
        message="CFD mesh-result validation failed before solver execution.",
        suggested_next_action="Inspect failed mesh checks and regenerate the volume mesh.",
        metadata={
            "failed_checks": result.metadata["failed_checks"],
            "mesh_path": str(mesh_path),
            "mesh_format": mesh_format,
        },
    )
    failure_id = create_failure(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        failure=failure,
        evidence_artifact_ids=artifact_ids,
    )
    log_event(
        connection,
        event_type="failure.recorded",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        severity="error",
        message="Recorded CFD mesh-result validation failure.",
        payload={"failure_id": failure_id},
    )
    fail_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        failure_id=failure_id,
        metrics=result.metrics,
        warnings=result.warnings,
        metadata=metadata,
        runtime_seconds=0.0,
    )
    log_event(
        connection,
        event_type="module.failed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        severity="error",
        message="Failed CFD mesh-result validation.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id


def _register_mesh_validation_artifacts(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
    mesh_path: Path,
    su2_log_path: Path | None,
    result_payload: dict[str, Any],
) -> list[str]:
    result_dir = artifact_root / "_generated_mesh_results" / evaluation_id
    result_dir.mkdir(parents=True, exist_ok=True)
    result_path = result_dir / "cfd_mesh_result.json"
    result_path.write_text(
        json.dumps(result_payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifact_ids = [
        register_artifact(
            connection,
            artifact_root=artifact_root,
            source_path=result_path,
            artifact_type="cfd_mesh_result_json",
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            producer_module=MODULE_NAME,
            producer_version=MODULE_VERSION,
            source_kind="generated_by_adapter",
            source_original_path=str(result_path),
            metadata={"schema_version": "0.1.0"},
        ),
        register_artifact_reference(
            connection,
            artifact_type="cfd_volume_mesh",
            path=str(mesh_path),
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            producer_module=MODULE_NAME,
            producer_version=MODULE_VERSION,
            source_kind="mesh_validation_input",
            status="referenced_unverified",
            source_original_path=str(mesh_path),
            metadata={"validated_by": MODULE_NAME},
        ),
    ]
    if su2_log_path is not None:
        artifact_ids.append(
            register_artifact_reference(
                connection,
                artifact_type="su2_preprocessing_log",
                path=str(su2_log_path),
                candidate_id=candidate_id,
                evaluation_id=evaluation_id,
                module_attempt_id=module_attempt_id,
                producer_module=MODULE_NAME,
                producer_version=MODULE_VERSION,
                source_kind="mesh_validation_input",
                status="referenced_unverified",
                source_original_path=str(su2_log_path),
                metadata={"used_for_marker_map": True},
            )
        )
    return artifact_ids
