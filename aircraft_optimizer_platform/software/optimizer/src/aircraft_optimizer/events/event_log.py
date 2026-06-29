from __future__ import annotations

import sqlite3
from typing import Any

from aircraft_optimizer.ids import new_id
from aircraft_optimizer.jsonutil import dumps
from aircraft_optimizer.time import utc_now_iso


def log_event(
    connection: sqlite3.Connection,
    *,
    event_type: str,
    message: str,
    actor: str = "system",
    severity: str = "info",
    campaign_id: str | None = None,
    candidate_id: str | None = None,
    evaluation_id: str | None = None,
    module_attempt_id: str | None = None,
    payload: dict[str, Any] | None = None,
) -> str:
    event_id = new_id("event")
    connection.execute(
        """
        INSERT INTO events (
            event_id, campaign_id, candidate_id, evaluation_id, module_attempt_id,
            event_type, event_time, actor, severity, message, payload_json
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            event_id,
            campaign_id,
            candidate_id,
            evaluation_id,
            module_attempt_id,
            event_type,
            utc_now_iso(),
            actor,
            severity,
            message,
            dumps(payload) if payload is not None else None,
        ),
    )
    return event_id
