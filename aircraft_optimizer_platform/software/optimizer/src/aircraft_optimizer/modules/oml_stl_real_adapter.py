from __future__ import annotations

import sqlite3
import subprocess
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
from aircraft_optimizer.exporters import parse_oml_stl_export_result
from aircraft_optimizer.external.real_adapter_boundary import RealNoInletExportRequest
from aircraft_optimizer.records import FailureDraft

MODULE_NAME = "oml_stl_export"
MODULE_VERSION = "0.1.0"


def execute_real_oml_stl_export(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    export_request: RealNoInletExportRequest,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    geometry_provider_result_id: str,
    timeout_seconds: int = 1200,
) -> str:
    if not export_request.execution_enabled:
        raise ValueError("real OML STL export requires execution_enabled=True")
    export_request.result_json_path.parent.mkdir(parents=True, exist_ok=True)
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind="export",
        inputs={
            "execution_mode": "real_execute_single_candidate",
            "geometry_provider_result_id": geometry_provider_result_id,
            "request": export_request.to_dict(),
            "timeout_seconds": timeout_seconds,
        },
    )
    log_event(
        connection,
        event_type="module.started",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message="Started real OML STL export adapter.",
    )

    try:
        completed = subprocess.run(
            export_request.command,
            cwd=export_request.exporter_working_directory,
            text=True,
            capture_output=True,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired as exc:
        failure_id = _record_execution_failure(
            connection,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            category="meshing.timeout",
            message=f"real OML STL export timed out after {timeout_seconds} seconds",
            metadata={
                "timeout_seconds": timeout_seconds,
                "stdout_tail": _tail(exc.stdout),
                "stderr_tail": _tail(exc.stderr),
            },
        )
        fail_module_attempt(
            connection,
            module_attempt_id=module_attempt_id,
            failure_id=failure_id,
            metrics={},
            metadata={"request": export_request.to_dict()},
            runtime_seconds=float(timeout_seconds),
        )
        return module_attempt_id

    if not export_request.result_json_path.exists():
        failure_id = _record_execution_failure(
            connection,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            category="artifact.missing_expected_artifact",
            message="real OML STL export did not produce the expected result JSON",
            metadata={
                "returncode": completed.returncode,
                "stdout_tail": _tail(completed.stdout),
                "stderr_tail": _tail(completed.stderr),
            },
        )
        fail_module_attempt(
            connection,
            module_attempt_id=module_attempt_id,
            failure_id=failure_id,
            metrics={},
            metadata={"request": export_request.to_dict()},
            runtime_seconds=0.0,
        )
        return module_attempt_id

    parsed = parse_oml_stl_export_result(export_request.result_json_path)
    artifact_ids = _register_real_export_artifacts(
        connection,
        artifact_root=artifact_root,
        parsed_artifacts=parsed.artifacts_dict(),
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
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
            message="Registered real OML STL export artifact.",
            payload={"artifact_id": artifact_id},
        )

    metadata: dict[str, Any] = {
        **parsed.metadata,
        "artifact_ids": artifact_ids,
        "real_export": True,
        "returncode": completed.returncode,
        "stdout_tail": _tail(completed.stdout),
        "stderr_tail": _tail(completed.stderr),
        "request": export_request.to_dict(),
    }
    runtime_seconds = float(parsed.metadata.get("wrapper_runtime_s") or 0.0)
    if parsed.failure is None and completed.returncode == 0:
        complete_module_attempt(
            connection,
            module_attempt_id=module_attempt_id,
            metrics=parsed.metrics_dict(),
            metadata=metadata,
            runtime_seconds=runtime_seconds,
        )
        log_event(
            connection,
            event_type="module.completed",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            actor="module",
            message="Completed real OML STL export adapter.",
        )
        return module_attempt_id

    failure = parsed.failure or FailureDraft(
        category="runtime.export_failed",
        stage="export",
        severity="recoverable",
        retryable=True,
        message=f"real OML STL export returned {completed.returncode}",
        metadata={"returncode": completed.returncode},
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
        message="Recorded real OML STL export failure.",
        payload={"failure_id": failure_id},
    )
    fail_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        failure_id=failure_id,
        metrics=parsed.metrics_dict(),
        metadata=metadata,
        runtime_seconds=runtime_seconds,
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
        message="Failed real OML STL export adapter.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id


def _register_real_export_artifacts(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    parsed_artifacts: list[dict[str, str]],
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
) -> list[str]:
    artifact_ids: list[str] = []
    for artifact in parsed_artifacts:
        artifact_type = artifact["kind"]
        source_path = Path(artifact["path"])
        if source_path.exists():
            artifact_ids.append(
                register_artifact(
                    connection,
                    artifact_root=artifact_root,
                    source_path=source_path,
                    artifact_type=artifact_type,
                    candidate_id=candidate_id,
                    evaluation_id=evaluation_id,
                    module_attempt_id=module_attempt_id,
                    producer_module=MODULE_NAME,
                    producer_version=MODULE_VERSION,
                    source_kind="real_export_copy",
                    source_original_path=str(source_path),
                    metadata={"real_export": True},
                )
            )
            continue
        artifact_ids.append(
            register_artifact_reference(
                connection,
                artifact_type=artifact_type,
                path=str(source_path),
                candidate_id=candidate_id,
                evaluation_id=evaluation_id,
                module_attempt_id=module_attempt_id,
                producer_module=MODULE_NAME,
                producer_version=MODULE_VERSION,
                source_kind="real_export_missing_reference",
                status="missing",
                source_original_path=str(source_path),
                metadata={"real_export": True},
            )
        )
    return artifact_ids


def _record_execution_failure(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
    category: str,
    message: str,
    metadata: dict[str, Any],
) -> str:
    failure_id = create_failure(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        failure=FailureDraft(
            category=category,
            stage="export",
            severity="recoverable",
            retryable=True,
            message=message,
            metadata=metadata,
        ),
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
        message=message,
        payload={"failure_id": failure_id},
    )
    return failure_id


def _tail(value: str | bytes | None, limit: int = 4000) -> str | None:
    if value is None:
        return None
    if isinstance(value, bytes):
        value = value.decode(errors="replace")
    return value[-limit:]
