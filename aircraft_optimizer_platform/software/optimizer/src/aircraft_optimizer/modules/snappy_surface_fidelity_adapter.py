from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

from aircraft_optimizer.artifacts.registry import register_artifact
from aircraft_optimizer.db.repositories import (
    complete_module_attempt,
    create_failure,
    create_module_attempt,
    fail_module_attempt,
)
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.records import FailureDraft, MetricValue


MODULE_NAME = "snappy_surface_fidelity_validation"
MODULE_VERSION = "0.1.0"
MODULE_KIND = "analysis"

DEFAULT_SNAPPY_SURFACE_FIDELITY_POLICY = {
    "acceptance_mode": "snappy_hifi_surface_development",
    "max_bidirectional_p95_mm": 0.25,
    "max_bidirectional_p99_mm": 2.5,
    "max_bidirectional_max_mm": 6.0,
    "min_area_ratio": 0.98,
    "max_area_ratio": 1.02,
}


def persist_snappy_surface_fidelity_result(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    surface_fidelity_report: Path,
    upstream_module_attempt_id: str | None = None,
    requested_by: str = "system",
    policy: dict[str, Any] | None = None,
) -> str:
    active_policy = {**DEFAULT_SNAPPY_SURFACE_FIDELITY_POLICY, **(policy or {})}
    inputs = {
        "surface_fidelity_report": str(surface_fidelity_report),
        "upstream_module_attempt_id": upstream_module_attempt_id,
        "requested_by": requested_by,
        "policy": active_policy,
        "scoring_allowed": False,
    }
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind=MODULE_KIND,
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
        message="Started snappy surface fidelity validation.",
        payload={"acceptance_mode": active_policy["acceptance_mode"]},
    )

    report = json.loads(surface_fidelity_report.read_text(encoding="utf-8"))
    failed_checks = evaluate_surface_fidelity_checks(report, active_policy)
    metrics = surface_fidelity_metrics(report, failed_checks)
    artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=surface_fidelity_report,
        artifact_type="snappy_surface_fidelity_report",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        producer_module=MODULE_NAME,
        producer_version=MODULE_VERSION,
        source_kind="snappy_surface_fidelity_input",
        source_original_path=str(surface_fidelity_report),
        metadata={
            "acceptance_mode": active_policy["acceptance_mode"],
            "scoring_allowed": False,
        },
    )
    log_event(
        connection,
        event_type="artifact.created",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message="Registered snappy surface fidelity artifact.",
        payload={"artifact_id": artifact_id},
    )

    metadata = {
        "module_name": MODULE_NAME,
        "module_version": MODULE_VERSION,
        "policy": active_policy,
        "failed_checks": failed_checks,
        "surface_fidelity_report": report,
        "artifact_ids": [artifact_id],
        "scoring_allowed": False,
        "analysis_role": "surface_capture_development_non_scoring",
    }
    if not failed_checks:
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
            message="Completed snappy surface fidelity validation.",
            payload={"scoring_allowed": False},
        )
        return module_attempt_id

    failure = FailureDraft(
        category="meshing.surface_fidelity_gate_failed",
        stage="snappy_surface_fidelity_validation",
        severity="recoverable",
        retryable=True,
        message="Snappy surface fidelity failed the configured development gate.",
        suggested_next_action="Inspect source-to-patch deviation report and feature closeups before using this mesh for solver development.",
        metadata={
            "failed_checks": failed_checks,
            "surface_fidelity_report": str(surface_fidelity_report),
            "scoring_allowed": False,
        },
    )
    failure_id = create_failure(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        failure=failure,
        evidence_artifact_ids=[artifact_id],
    )
    fail_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        failure_id=failure_id,
        metrics=metrics,
        warnings=[f"failed snappy surface fidelity check: {check}" for check in failed_checks],
        metadata=metadata,
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
        message="Failed snappy surface fidelity validation.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id


def evaluate_surface_fidelity_checks(
    report: dict[str, Any],
    policy: dict[str, Any],
) -> list[str]:
    failed: list[str] = []
    bidirectional = report.get("bidirectional_mm") or {}
    area_ratio = report.get("area_ratio_candidate_over_reference")
    if bidirectional.get("p95", float("inf")) > float(policy["max_bidirectional_p95_mm"]):
        failed.append("bidirectional_p95")
    if bidirectional.get("p99", float("inf")) > float(policy["max_bidirectional_p99_mm"]):
        failed.append("bidirectional_p99")
    if bidirectional.get("max", float("inf")) > float(policy["max_bidirectional_max_mm"]):
        failed.append("bidirectional_max")
    if area_ratio is None or area_ratio < float(policy["min_area_ratio"]):
        failed.append("area_ratio_min")
    if area_ratio is None or area_ratio > float(policy["max_area_ratio"]):
        failed.append("area_ratio_max")
    return failed


def surface_fidelity_metrics(
    report: dict[str, Any],
    failed_checks: list[str],
) -> dict[str, dict[str, object]]:
    bidirectional = report.get("bidirectional_mm") or {}
    ref_to_candidate = report.get("reference_to_candidate_mm") or {}
    candidate_to_ref = report.get("candidate_to_reference_mm") or {}
    return {
        "snappy_surface.bidirectional_p95_mm": _metric(bidirectional.get("p95"), "mm"),
        "snappy_surface.bidirectional_p99_mm": _metric(bidirectional.get("p99"), "mm"),
        "snappy_surface.bidirectional_max_mm": _metric(bidirectional.get("max"), "mm"),
        "snappy_surface.reference_to_candidate_p95_mm": _metric(
            ref_to_candidate.get("p95"),
            "mm",
        ),
        "snappy_surface.candidate_to_reference_p95_mm": _metric(
            candidate_to_ref.get("p95"),
            "mm",
        ),
        "snappy_surface.area_ratio": _metric(
            report.get("area_ratio_candidate_over_reference"),
            None,
        ),
        "snappy_surface.ready": _metric(0 if failed_checks else 1, None),
        "snappy_surface.failed_check_count": _metric(len(failed_checks), None),
    }


def _metric(value: float | int | None, unit: str | None) -> dict[str, object]:
    safe_value: float | int = 0 if value is None else value
    return MetricValue(
        value=round(safe_value, 12) if isinstance(safe_value, float) else safe_value,
        unit=unit,
        confidence=1.0,
        source=MODULE_NAME,
    ).to_dict()
