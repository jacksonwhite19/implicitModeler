from __future__ import annotations

import sqlite3

from aircraft_optimizer.db.repositories import create_evaluation
from aircraft_optimizer.events.event_log import log_event


def start_evaluation_record(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    candidate_id: str,
    pipeline_id: str,
    pipeline_version: str,
    requested_by: str,
    reason: str,
    environment_fingerprint_id: str | None,
    message: str,
) -> str:
    evaluation_id = create_evaluation(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        pipeline_id=pipeline_id,
        pipeline_version=pipeline_version,
        requested_by=requested_by,
        reason=reason,
        environment_fingerprint_id=environment_fingerprint_id,
    )
    log_event(
        connection,
        event_type="evaluation.started",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        message=message,
    )
    return evaluation_id
