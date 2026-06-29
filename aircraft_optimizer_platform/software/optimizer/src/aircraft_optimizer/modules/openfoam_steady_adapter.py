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
from aircraft_optimizer.modules.openfoam_steady_result_validation import (
    MODULE_VERSION,
    validate_openfoam_steady_result,
)
from aircraft_optimizer.records import FailureDraft


MODULE_NAME = "openfoam_steady_validation"
MODULE_KIND = "analysis"


def persist_openfoam_steady_result(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    case_dir: Path,
    solver_log: Path,
    check_mesh_log: Path | None = None,
    force_coeffs_path: Path | None = None,
    yplus_path: Path | None = None,
    aircraft_iso_path: Path | None = None,
    upstream_module_attempt_id: str | None = None,
    policy: dict[str, Any] | None = None,
    requested_by: str = "system",
) -> str:
    inputs = {
        "case_dir": str(case_dir),
        "solver_log": str(solver_log),
        "check_mesh_log": str(check_mesh_log) if check_mesh_log else None,
        "force_coeffs_path": str(force_coeffs_path) if force_coeffs_path else None,
        "yplus_path": str(yplus_path) if yplus_path else None,
        "aircraft_iso_path": str(aircraft_iso_path) if aircraft_iso_path else None,
        "upstream_module_attempt_id": upstream_module_attempt_id,
        "acceptance_mode": (
            policy.get("acceptance_mode") if policy else "openfoam_steady_development"
        ),
        "scoring_tier": (
            policy.get("scoring_tier") if policy else "development_diagnostic"
        ),
        "score_usage": policy.get("score_usage") if policy else "diagnostic_only",
        "requested_by": requested_by,
        # Legacy alias: this means final/engineering scoring, not rough scoring.
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
        message="Started OpenFOAM steady validation.",
        payload={
            "scoring_tier": inputs["scoring_tier"],
            "score_usage": inputs["score_usage"],
            "scoring_allowed": False,
        },
    )

    result = validate_openfoam_steady_result(
        case_dir=case_dir,
        solver_log=solver_log,
        check_mesh_log=check_mesh_log,
        force_coeffs_path=force_coeffs_path,
        yplus_path=yplus_path,
        policy=policy,
    )
    artifact_ids = _register_openfoam_steady_artifacts(
        connection,
        artifact_root=artifact_root,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        result_payload={
            "passed": result.passed,
            "openfoam_steady_result": result.openfoam_steady_result,
            "metrics": result.metrics,
            "metadata": result.metadata,
            "warnings": result.warnings,
            "scoring": result.metadata["scoring"],
            "scoring_allowed": result.metadata["scoring_allowed"],
        },
        case_dir=case_dir,
        solver_log=solver_log,
        check_mesh_log=check_mesh_log,
        force_coeffs_path=force_coeffs_path,
        yplus_path=yplus_path,
        aircraft_iso_path=aircraft_iso_path,
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
            message="Registered OpenFOAM steady validation artifact.",
            payload={"artifact_id": artifact_id},
        )

    metadata = {
        **result.metadata,
        "artifact_ids": artifact_ids,
        "openfoam_steady_result": result.openfoam_steady_result,
        "scoring_allowed": result.metadata["scoring_allowed"],
        "analysis_role": _analysis_role(result.metadata["scoring"]),
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
            message="Completed OpenFOAM steady validation.",
            payload=result.metadata["scoring"],
        )
        return module_attempt_id

    failure = FailureDraft(
        category="cfd.openfoam_steady_validation_failed",
        stage="openfoam_steady_validation",
        severity="recoverable",
        retryable=True,
        message="OpenFOAM steady validation failed under the development policy.",
        suggested_next_action="Inspect solver residuals, force coefficients, and case dictionaries before rerunning.",
        metadata={
            "failed_checks": result.metadata["failed_checks"],
            "case_dir": str(case_dir),
            "scoring": result.metadata["scoring"],
            "scoring_allowed": result.metadata["scoring_allowed"],
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
        message="Recorded OpenFOAM steady validation failure.",
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
        message="Failed OpenFOAM steady validation.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id


def _register_openfoam_steady_artifacts(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
    result_payload: dict[str, Any],
    case_dir: Path,
    solver_log: Path,
    check_mesh_log: Path | None,
    force_coeffs_path: Path | None,
    yplus_path: Path | None,
    aircraft_iso_path: Path | None,
) -> list[str]:
    result_dir = artifact_root / "_generated_openfoam_steady" / evaluation_id
    result_dir.mkdir(parents=True, exist_ok=True)
    result_path = result_dir / "openfoam_steady_result.json"
    result_path.write_text(
        json.dumps(result_payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifact_ids = [
        register_artifact(
            connection,
            artifact_root=artifact_root,
            source_path=result_path,
            artifact_type="openfoam_steady_result_json",
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            producer_module=MODULE_NAME,
            producer_version=MODULE_VERSION,
            source_kind="generated_by_adapter",
            source_original_path=str(result_path),
            metadata={
                "schema_version": "0.1.0",
                "scoring": result_payload["scoring"],
                "scoring_allowed": result_payload["scoring_allowed"],
            },
        ),
        register_artifact_reference(
            connection,
            artifact_type="openfoam_case_dir",
            path=str(case_dir),
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            producer_module=MODULE_NAME,
            producer_version=MODULE_VERSION,
            source_kind="openfoam_steady_input",
            status="referenced_unverified",
            source_original_path=str(case_dir),
            metadata={"contains_volume_mesh": True},
        ),
    ]
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=solver_log,
        artifact_type="openfoam_steady_solver_log",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"solver": "foamRun -solver incompressibleFluid"},
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=check_mesh_log,
        artifact_type="openfoam_checkmesh_log",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"used_for_context": True},
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=force_coeffs_path,
        artifact_type="openfoam_force_coefficients",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={
            "scoring": result_payload["scoring"],
            "scoring_allowed": result_payload["scoring_allowed"],
        },
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=yplus_path,
        artifact_type="openfoam_yplus_field",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={
            "field": "yPlus",
            "scoring": result_payload["scoring"],
            "scoring_allowed": result_payload["scoring_allowed"],
        },
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=aircraft_iso_path,
        artifact_type="aircraft_iso_screenshot",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"patch_only": "aircraft"},
    )
    return artifact_ids


def _analysis_role(scoring: dict[str, Any]) -> str:
    tier = scoring.get("scoring_tier", "development_diagnostic")
    if tier == "rough_scoring":
        return "rough_scoring_optimizer_ranking"
    if tier == "confirmation_scoring_development":
        return "confirmation_scoring_development"
    if tier == "final_scoring_readiness":
        return "final_scoring_readiness_gate"
    return "steady_solver_development_diagnostic"


def _register_optional_file(
    artifact_ids: list[str],
    *,
    connection: sqlite3.Connection,
    artifact_root: Path,
    source_path: Path | None,
    artifact_type: str,
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
    metadata: dict[str, Any],
) -> None:
    if source_path is None or not source_path.exists():
        return
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
            source_kind="openfoam_steady_input",
            source_original_path=str(source_path),
            metadata=metadata,
        )
    )
