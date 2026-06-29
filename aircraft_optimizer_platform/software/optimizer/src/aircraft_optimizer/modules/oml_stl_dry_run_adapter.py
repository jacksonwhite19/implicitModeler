from __future__ import annotations

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
from aircraft_optimizer.exporters import parse_oml_stl_export_result

MODULE_NAME = "oml_stl_export"
MODULE_VERSION = "0.1.0"


def persist_oml_stl_fixture_result(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    fixture_path: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    geometry_provider_result_id: str,
    requested_by: str = "test_fixture",
) -> str:
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind="export",
        inputs={
            "execution_mode": "fixture_parse_only",
            "fixture_path": str(fixture_path),
            "geometry_provider_result_id": geometry_provider_result_id,
            "requested_by": requested_by,
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
        message="Started OML STL fixture adapter.",
    )

    parsed = parse_oml_stl_export_result(fixture_path)
    artifact_ids = _register_parsed_artifacts(
        connection,
        artifact_root=artifact_root,
        fixture_path=fixture_path,
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
            message="Registered OML STL adapter artifact reference.",
            payload={"artifact_id": artifact_id},
        )

    metadata = {
        **parsed.metadata,
        "artifact_ids": artifact_ids,
        "fixture_parse_only": True,
    }
    if parsed.failure is None:
        complete_module_attempt(
            connection,
            module_attempt_id=module_attempt_id,
            metrics=parsed.metrics_dict(),
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
            message="Completed OML STL fixture adapter.",
        )
        return module_attempt_id

    failure_id = create_failure(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        failure=parsed.failure,
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
        message="Recorded OML STL fixture adapter failure.",
        payload={"failure_id": failure_id},
    )
    fail_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        failure_id=failure_id,
        metrics=parsed.metrics_dict(),
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
        message="Failed OML STL fixture adapter.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id


def _register_parsed_artifacts(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    fixture_path: Path,
    parsed_artifacts: list[dict[str, str]],
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
) -> list[str]:
    artifact_ids: list[str] = []
    for artifact in parsed_artifacts:
        artifact_type = artifact["kind"]
        artifact_path = artifact["path"]
        if artifact_type == "export_result_json":
            artifact_ids.append(
                register_artifact(
                    connection,
                    artifact_root=artifact_root,
                    source_path=fixture_path,
                    artifact_type=artifact_type,
                    candidate_id=candidate_id,
                    evaluation_id=evaluation_id,
                    module_attempt_id=module_attempt_id,
                    producer_module=MODULE_NAME,
                    producer_version=MODULE_VERSION,
                    source_kind="fixture_copy",
                    source_original_path=str(fixture_path),
                    metadata={"declared_result_json": artifact_path},
                )
            )
            continue

        artifact_ids.append(
            register_artifact_reference(
                connection,
                artifact_type=artifact_type,
                path=artifact_path,
                candidate_id=candidate_id,
                evaluation_id=evaluation_id,
                module_attempt_id=module_attempt_id,
                producer_module=MODULE_NAME,
                producer_version=MODULE_VERSION,
                source_kind="exporter_declared_path",
                status="referenced_unverified",
                source_original_path=artifact_path,
                metadata={"fixture_parse_only": True},
            )
        )
    return artifact_ids
