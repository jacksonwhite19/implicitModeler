from __future__ import annotations

import sqlite3
from typing import Any

from aircraft_optimizer.db.repositories import (
    complete_module_attempt,
    create_module_attempt,
)
from aircraft_optimizer.events.event_log import log_event


def run_successful_module_attempt(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    module_name: str,
    module_version: str,
    module_kind: str,
    inputs: dict[str, Any],
    metrics: dict[str, Any],
    metadata: dict[str, Any],
    started_message: str,
    completed_message: str,
) -> str:
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=module_name,
        module_version=module_version,
        module_kind=module_kind,
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
        message=started_message,
    )
    complete_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        metrics=metrics,
        metadata=metadata,
    )
    log_event(
        connection,
        event_type="module.completed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message=completed_message,
    )
    return module_attempt_id
