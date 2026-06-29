from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.db.repositories import create_candidate, create_candidate_lineage
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.records import CandidateSeed


@dataclass(frozen=True)
class CandidateRegistrationResult:
    candidate_id: str
    lineage_event_id: str


def register_candidate_with_lineage(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    variable_schema_id: str,
    candidate: CandidateSeed,
    parent_candidate_ids: list[str],
    operator: str,
    reason: str,
    mutation_summary: dict[str, Any],
    registration_message: str,
    lineage_message: str,
    operator_version: str = "0.1.0",
) -> CandidateRegistrationResult:
    candidate_id = create_candidate(
        connection,
        campaign_id=campaign_id,
        aircraft_family=candidate.aircraft_family,
        variable_schema_id=variable_schema_id,
        design_variables=candidate.design_variables_dict(),
        normalized_design_vector=candidate.normalized_design_vector,
        created_by=candidate.created_by,
        generation=candidate.generation,
        notes=candidate.notes,
    )
    log_event(
        connection,
        event_type="candidate.registered",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        actor="system",
        message=registration_message,
    )
    lineage_event_id = create_candidate_lineage(
        connection,
        campaign_id=campaign_id,
        child_candidate_id=candidate_id,
        parent_candidate_ids=parent_candidate_ids,
        operator=operator,
        operator_version=operator_version,
        reason=reason,
        mutation_summary=mutation_summary,
    )
    log_event(
        connection,
        event_type="lineage.recorded",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        actor="system",
        message=lineage_message,
        payload={"lineage_event_id": lineage_event_id},
    )
    return CandidateRegistrationResult(
        candidate_id=candidate_id,
        lineage_event_id=lineage_event_id,
    )
