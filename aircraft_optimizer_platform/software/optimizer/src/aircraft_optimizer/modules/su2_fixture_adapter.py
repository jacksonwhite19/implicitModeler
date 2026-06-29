from __future__ import annotations

import sqlite3
from pathlib import Path

from aircraft_optimizer.artifacts.registry import register_artifact
from aircraft_optimizer.db.repositories import complete_module_attempt, create_module_attempt
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.external import parse_su2_history_metrics

MODULE_NAME = "su2_cfd_fixture"
MODULE_VERSION = "0.1.0"


def persist_su2_history_fixture_result(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    history_fixture_path: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    geometry_provider_result_id: str,
    export_module_attempt_id: str,
    case_builder_module_attempt_id: str | None = None,
) -> str:
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind="analysis",
        inputs={
            "execution_mode": "fixture_parse_only",
            "history_fixture_path": str(history_fixture_path),
            "geometry_provider_result_id": geometry_provider_result_id,
            "export_module_attempt_id": export_module_attempt_id,
            "case_builder_module_attempt_id": case_builder_module_attempt_id,
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
        message="Started SU2 history fixture adapter.",
    )
    artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=history_fixture_path,
        artifact_type="su2_history_csv",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        producer_module=MODULE_NAME,
        producer_version=MODULE_VERSION,
        source_kind="fixture_copy",
        source_original_path=str(history_fixture_path),
        metadata={"fixture_parse_only": True},
    )
    log_event(
        connection,
        event_type="artifact.created",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message="Registered SU2 history fixture artifact.",
        payload={"artifact_id": artifact_id},
    )
    complete_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        metrics=parse_su2_history_metrics(history_fixture_path),
        metadata={"fixture_parse_only": True, "artifact_ids": [artifact_id]},
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
        message="Completed SU2 history fixture adapter.",
    )
    return module_attempt_id
