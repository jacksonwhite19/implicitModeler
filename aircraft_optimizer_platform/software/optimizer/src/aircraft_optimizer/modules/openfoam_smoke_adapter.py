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
from aircraft_optimizer.modules.openfoam_result_validation import (
    MODULE_VERSION,
    policy_for_acceptance_mode,
    validate_openfoam_result,
)
from aircraft_optimizer.records import FailureDraft


MODULE_NAME = "openfoam_smoke_validation"
MODULE_KIND = "analysis"


def persist_openfoam_smoke_result(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    case_dir: Path,
    check_mesh_log: Path,
    solver_log: Path,
    acceptance_mode: str = "strict",
    strict_check_mesh_log: Path | None = None,
    relaxed_check_mesh_log: Path | None = None,
    aircraft_iso_path: Path | None = None,
    upstream_module_attempt_id: str | None = None,
    requested_by: str = "system",
) -> str:
    policy = _policy_for_acceptance_mode(acceptance_mode)
    inputs = {
        "case_dir": str(case_dir),
        "check_mesh_log": str(check_mesh_log),
        "solver_log": str(solver_log),
        "acceptance_mode": acceptance_mode,
        "strict_check_mesh_log": str(strict_check_mesh_log) if strict_check_mesh_log else None,
        "relaxed_check_mesh_log": str(relaxed_check_mesh_log) if relaxed_check_mesh_log else None,
        "aircraft_iso_path": str(aircraft_iso_path) if aircraft_iso_path else None,
        "upstream_module_attempt_id": upstream_module_attempt_id,
        "requested_by": requested_by,
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
        message="Started OpenFOAM smoke validation.",
        payload={"acceptance_mode": acceptance_mode},
    )

    result = validate_openfoam_result(
        case_dir=case_dir,
        check_mesh_log=check_mesh_log,
        solver_log=solver_log,
        policy=policy,
    )
    artifact_ids = _register_openfoam_smoke_artifacts(
        connection,
        artifact_root=artifact_root,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        result_payload={
            "passed": result.passed,
            "openfoam_result": result.openfoam_result,
            "metrics": result.metrics,
            "metadata": result.metadata,
            "warnings": result.warnings,
            "scoring_allowed": False,
        },
        case_dir=case_dir,
        check_mesh_log=check_mesh_log,
        solver_log=solver_log,
        strict_check_mesh_log=strict_check_mesh_log,
        relaxed_check_mesh_log=relaxed_check_mesh_log,
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
            message="Registered OpenFOAM smoke validation artifact.",
            payload={"artifact_id": artifact_id},
        )

    metadata = {
        **result.metadata,
        "artifact_ids": artifact_ids,
        "openfoam_result": result.openfoam_result,
        "acceptance_mode": acceptance_mode,
        "scoring_allowed": False,
        "analysis_role": "solver_smoke_non_scoring",
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
            message="Completed OpenFOAM smoke validation.",
            payload={"acceptance_mode": acceptance_mode, "scoring_allowed": False},
        )
        return module_attempt_id

    failure = FailureDraft(
        category="cfd.openfoam_smoke_validation_failed",
        stage="openfoam_smoke_validation",
        severity="recoverable",
        retryable=True,
        message="OpenFOAM smoke validation failed under the configured acceptance policy.",
        suggested_next_action="Inspect strict and relaxed checkMesh logs, solver log, and aircraft screenshot before rerunning.",
        metadata={
            "failed_checks": result.metadata["failed_checks"],
            "case_dir": str(case_dir),
            "acceptance_mode": acceptance_mode,
            "scoring_allowed": False,
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
        message="Recorded OpenFOAM smoke validation failure.",
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
        message="Failed OpenFOAM smoke validation.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id


def _policy_for_acceptance_mode(mode: str) -> dict[str, Any]:
    return policy_for_acceptance_mode(mode)


def _register_openfoam_smoke_artifacts(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    candidate_id: str,
    evaluation_id: str,
    module_attempt_id: str,
    result_payload: dict[str, Any],
    case_dir: Path,
    check_mesh_log: Path,
    solver_log: Path,
    strict_check_mesh_log: Path | None,
    relaxed_check_mesh_log: Path | None,
    aircraft_iso_path: Path | None,
) -> list[str]:
    result_dir = artifact_root / "_generated_openfoam_smoke" / evaluation_id
    result_dir.mkdir(parents=True, exist_ok=True)
    result_path = result_dir / "openfoam_smoke_result.json"
    result_path.write_text(
        json.dumps(result_payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    artifact_ids = [
        register_artifact(
            connection,
            artifact_root=artifact_root,
            source_path=result_path,
            artifact_type="openfoam_smoke_result_json",
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            producer_module=MODULE_NAME,
            producer_version=MODULE_VERSION,
            source_kind="generated_by_adapter",
            source_original_path=str(result_path),
            metadata={"schema_version": "0.1.0", "scoring_allowed": False},
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
            source_kind="openfoam_smoke_input",
            status="referenced_unverified",
            source_original_path=str(case_dir),
            metadata={"contains_volume_mesh": True},
        ),
    ]
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=check_mesh_log,
        artifact_type="openfoam_checkmesh_log",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"used_for_acceptance": True},
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=solver_log,
        artifact_type="openfoam_solver_log",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"solver": "potentialFoam"},
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=strict_check_mesh_log,
        artifact_type="openfoam_strict_checkmesh_log",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"strict_default_check": True},
    )
    _register_optional_file(
        artifact_ids,
        connection=connection,
        artifact_root=artifact_root,
        source_path=relaxed_check_mesh_log,
        artifact_type="openfoam_relaxed_checkmesh_log",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        metadata={"skew_threshold": 6},
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
            source_kind="openfoam_smoke_input",
            source_original_path=str(source_path),
            metadata=metadata,
        )
    )
