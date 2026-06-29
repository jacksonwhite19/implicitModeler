from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.db.repositories import (
    complete_evaluation,
    create_failure,
    create_module_attempt,
    create_user_annotation,
    fail_evaluation,
    fail_module_attempt,
    update_candidate_status,
    upsert_candidate_runner_state,
)
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.modules.low_fidelity_aero import (
    MODULE_NAME as LOW_FIDELITY_MODULE_NAME,
    MODULE_VERSION as LOW_FIDELITY_MODULE_VERSION,
    low_fidelity_fixed_wing_metrics,
)
from aircraft_optimizer.modules.geometry_definition_screening import (
    MODULE_NAME as GEOMETRY_DEFINITION_SCREEN_MODULE_NAME,
    MODULE_VERSION as GEOMETRY_DEFINITION_SCREEN_MODULE_VERSION,
    screen_geometry_definition,
)
from aircraft_optimizer.modules.mock_module import mock_geometry_metrics
from aircraft_optimizer.modules.mock_scoring import fixture_scoring_metrics
from aircraft_optimizer.modules.oml_stl_dry_run_adapter import (
    persist_oml_stl_fixture_result,
)
from aircraft_optimizer.modules.su2_case_builder import persist_su2_dry_run_case
from aircraft_optimizer.modules.su2_fixture_adapter import (
    persist_su2_history_fixture_result,
)
from aircraft_optimizer.orchestration.candidate import register_candidate_with_lineage
from aircraft_optimizer.orchestration.evaluation import start_evaluation_record
from aircraft_optimizer.orchestration.geometry import persist_fixture_geometry
from aircraft_optimizer.orchestration.module_attempt import run_successful_module_attempt
from aircraft_optimizer.records import CandidateSeed, FailureDraft


@dataclass(frozen=True)
class CandidateEvaluationRequest:
    campaign_id: str
    variable_schema_id: str
    pipeline_id: str
    pipeline_version: str
    environment_fingerprint_id: str
    candidate: CandidateSeed
    parent_candidate_ids: list[str]
    lineage_operator: str
    lineage_reason: str
    mutation_summary: dict[str, Any]
    evaluation_reason: str
    registration_message: str
    lineage_message: str
    evaluation_started_message: str
    geometry_request_message: str
    geometry_completed_message: str
    export_fixture_path: Path
    annotation_tag: str | None = None
    annotation_comment: str | None = None
    annotation_metadata: dict[str, Any] | None = None
    run_scoring: bool = True
    run_low_fidelity_aero: bool = False
    run_su2_case_dry_run: bool = False
    run_su2_history_fixture: bool = False
    su2_history_fixture_path: Path | None = None
    mesh_reference_path: str = "outputs/direct_sdf_oml_A.stl"
    existing_candidate_id: str | None = None
    existing_evaluation_id: str | None = None
    preexisting_module_attempt_ids: list[str] | None = None
    optimizer_run_id: str | None = None
    optimizer_iteration_id: str | None = None
    runner_priority: int | None = None


@dataclass(frozen=True)
class CandidateEvaluationResult:
    candidate_id: str
    evaluation_id: str
    status: str
    geometry_provider_result_id: str
    module_attempt_ids: list[str]
    artifact_ids: list[str]
    failure_id: str | None


def evaluate_fixture_candidate(
    connection: sqlite3.Connection,
    *,
    platform_root: Path,
    workspace: Path,
    artifact_root: Path,
    variable_schema: Any,
    request: CandidateEvaluationRequest,
) -> CandidateEvaluationResult:
    request.candidate.validate()
    variable_schema.validate_candidate(request.candidate)

    if request.existing_candidate_id is None:
        registration = register_candidate_with_lineage(
            connection,
            campaign_id=request.campaign_id,
            variable_schema_id=request.variable_schema_id,
            candidate=request.candidate,
            parent_candidate_ids=request.parent_candidate_ids,
            operator=request.lineage_operator,
            reason=request.lineage_reason,
            mutation_summary=request.mutation_summary,
            registration_message=request.registration_message,
            lineage_message=request.lineage_message,
        )
        candidate_id = registration.candidate_id
    else:
        candidate_id = request.existing_candidate_id

    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=request.campaign_id,
        optimizer_run_id=request.optimizer_run_id,
        optimizer_iteration_id=request.optimizer_iteration_id,
        state="queued",
        stage="optimizer_proposed",
        priority=request.runner_priority,
        reason=request.lineage_reason,
        message="Candidate queued for evaluation.",
        metadata={"lineage_operator": request.lineage_operator},
    )

    if request.existing_evaluation_id is None:
        evaluation_id = start_evaluation_record(
            connection,
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            pipeline_id=request.pipeline_id,
            pipeline_version=request.pipeline_version,
            requested_by="system",
            reason=request.evaluation_reason,
            environment_fingerprint_id=request.environment_fingerprint_id,
            message=request.evaluation_started_message,
        )
    else:
        evaluation_id = request.existing_evaluation_id
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=request.campaign_id,
        optimizer_run_id=request.optimizer_run_id,
        optimizer_iteration_id=request.optimizer_iteration_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="geometry",
        active_module="fixture_geometry_provider",
        progress=0.10,
        priority=request.runner_priority,
        reason=request.evaluation_reason,
        message="Candidate evaluation is generating geometry.",
        metadata={"pipeline_id": request.pipeline_id},
    )
    geometry_record = persist_fixture_geometry(
        connection,
        platform_root=platform_root,
        workspace=workspace,
        artifact_root=artifact_root,
        campaign_id=request.campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        variable_schema_id=request.variable_schema_id,
        variable_schema=variable_schema,
        candidate=request.candidate,
        request_message=request.geometry_request_message,
        completed_message=request.geometry_completed_message,
    )
    module_attempt_ids: list[str] = list(request.preexisting_module_attempt_ids or [])
    artifact_ids = [
        geometry_record.geometry_artifact_id,
        geometry_record.trace_artifact_id,
    ]

    geometry_definition_screen = screen_geometry_definition(
        request.candidate,
        feature_name=geometry_record.feature_name,
        bbox_min_mm=geometry_record.bbox_min_mm,
        bbox_max_mm=geometry_record.bbox_max_mm,
    )
    geometry_definition_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=GEOMETRY_DEFINITION_SCREEN_MODULE_NAME,
        module_version=GEOMETRY_DEFINITION_SCREEN_MODULE_VERSION,
        module_kind="validation",
        inputs={
            "geometry_provider_result_id": geometry_record.geometry_provider_result_id,
            "feature_name": geometry_record.feature_name,
            "bbox_min_mm": geometry_record.bbox_min_mm,
            "bbox_max_mm": geometry_record.bbox_max_mm,
        },
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=request.campaign_id,
        optimizer_run_id=request.optimizer_run_id,
        optimizer_iteration_id=request.optimizer_iteration_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="pre_export_checks",
        active_module=GEOMETRY_DEFINITION_SCREEN_MODULE_NAME,
        progress=0.20,
        priority=request.runner_priority,
        reason="geometry definition screening",
        message="Candidate is running pre-export geometry checks.",
        metadata={"geometry_provider_result_id": geometry_record.geometry_provider_result_id},
    )
    log_event(
        connection,
        event_type="module.started",
        campaign_id=request.campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=geometry_definition_attempt_id,
        actor="module",
        message="Started geometry definition screening module.",
    )
    if geometry_definition_screen.passed:
        from aircraft_optimizer.db.repositories import complete_module_attempt

        complete_module_attempt(
            connection,
            module_attempt_id=geometry_definition_attempt_id,
            metrics=geometry_definition_screen.metrics,
            metadata=geometry_definition_screen.metadata,
            warnings=geometry_definition_screen.warnings,
        )
        log_event(
            connection,
            event_type="module.completed",
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=geometry_definition_attempt_id,
            actor="module",
            message="Completed geometry definition screening module.",
        )
    else:
        failure_id = create_failure(
            connection,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=geometry_definition_attempt_id,
            failure=FailureDraft(
                category="geometry.definition_screen_failed",
                stage="geometry_definition_screening",
                severity="recoverable",
                retryable=False,
                message="Candidate failed geometry definition screening before export.",
                suggested_next_action="Record result and let optimizer propose the next candidate.",
                metadata=geometry_definition_screen.metadata,
            ),
        )
        fail_module_attempt(
            connection,
            module_attempt_id=geometry_definition_attempt_id,
            failure_id=failure_id,
            metrics=geometry_definition_screen.metrics,
            warnings=geometry_definition_screen.warnings,
            metadata=geometry_definition_screen.metadata,
        )
        module_attempt_ids.append(geometry_definition_attempt_id)
        fail_evaluation(
            connection,
            evaluation_id=evaluation_id,
            failure_id=failure_id,
            summary={
                "status": "failed",
                "geometry_provider_result_id": geometry_record.geometry_provider_result_id,
                "module_attempt_ids": module_attempt_ids,
                "artifact_ids": artifact_ids,
                "failure_id": failure_id,
            },
        )
        log_event(
            connection,
            event_type="evaluation.failed",
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            actor="system",
            severity="warning",
            message="Failed geometry definition screening before export.",
            payload={"failure_id": failure_id},
        )
        update_candidate_status(connection, candidate_id=candidate_id, status="failed")
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=request.campaign_id,
            optimizer_run_id=request.optimizer_run_id,
            optimizer_iteration_id=request.optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="failed",
            stage="pre_export_checks",
            active_module=GEOMETRY_DEFINITION_SCREEN_MODULE_NAME,
            progress=1.0,
            priority=request.runner_priority,
            reason="geometry definition screen failed",
            message="Candidate failed pre-export geometry definition checks.",
            metadata={"failure_id": failure_id},
        )
        _maybe_record_annotation(connection, request, candidate_id, evaluation_id)
        return CandidateEvaluationResult(
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            status="failed",
            geometry_provider_result_id=geometry_record.geometry_provider_result_id,
            module_attempt_ids=module_attempt_ids,
            artifact_ids=artifact_ids,
            failure_id=failure_id,
        )
    module_attempt_ids.append(geometry_definition_attempt_id)

    geometry_metrics_attempt_id = run_successful_module_attempt(
        connection,
        campaign_id=request.campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_name="mock_geometry_metrics",
        module_version="0.1.0",
        module_kind="analysis",
        inputs={
            "geometry_provider_result_id": geometry_record.geometry_provider_result_id
        },
        metrics=mock_geometry_metrics(),
        metadata={"records_only": True},
        started_message="Started mock geometry metrics module.",
        completed_message="Completed mock geometry metrics module.",
    )
    module_attempt_ids.append(geometry_metrics_attempt_id)

    export_module_attempt_id = persist_oml_stl_fixture_result(
        connection,
        artifact_root=artifact_root,
        fixture_path=request.export_fixture_path,
        campaign_id=request.campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        geometry_provider_result_id=geometry_record.geometry_provider_result_id,
    )
    module_attempt_ids.append(export_module_attempt_id)
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=request.campaign_id,
        optimizer_run_id=request.optimizer_run_id,
        optimizer_iteration_id=request.optimizer_iteration_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="export",
        active_module="oml_stl_export",
        progress=0.40,
        priority=request.runner_priority,
        reason="OML export fixture",
        message="Candidate is recording OML export evidence.",
        metadata={"module_attempt_id": export_module_attempt_id},
    )
    export_status, export_failure_id = connection.execute(
        """
        SELECT status, failure_id
        FROM module_attempts
        WHERE module_attempt_id = ?
        """,
        (export_module_attempt_id,),
    ).fetchone()
    if export_status == "failed":
        fail_evaluation(
            connection,
            evaluation_id=evaluation_id,
            failure_id=export_failure_id,
            summary={
                "status": "failed",
                "geometry_provider_result_id": geometry_record.geometry_provider_result_id,
                "module_attempt_ids": module_attempt_ids,
                "artifact_ids": artifact_ids,
                "failure_id": export_failure_id,
            },
        )
        log_event(
            connection,
            event_type="evaluation.failed",
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            actor="system",
            severity="error",
            message="Failed fixture candidate evaluation.",
            payload={"failure_id": export_failure_id},
        )
        update_candidate_status(
            connection,
            candidate_id=candidate_id,
            status="failed",
        )
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=request.campaign_id,
            optimizer_run_id=request.optimizer_run_id,
            optimizer_iteration_id=request.optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="failed",
            stage="export",
            active_module="oml_stl_export",
            progress=1.0,
            priority=request.runner_priority,
            reason="export quality gate failed",
            message="Candidate failed OML export quality gate.",
            metadata={"failure_id": export_failure_id},
        )
        _maybe_record_annotation(connection, request, candidate_id, evaluation_id)
        return CandidateEvaluationResult(
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            status="failed",
            geometry_provider_result_id=geometry_record.geometry_provider_result_id,
            module_attempt_ids=module_attempt_ids,
            artifact_ids=artifact_ids,
            failure_id=export_failure_id,
        )

    if request.run_su2_case_dry_run:
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=request.campaign_id,
            optimizer_run_id=request.optimizer_run_id,
            optimizer_iteration_id=request.optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="cfd_running",
            stage="cfd_case",
            active_module="su2_case_dry_run",
            progress=0.55,
            priority=request.runner_priority,
            reason="SU2 case dry run",
            message="Candidate is recording dry-run CFD case evidence.",
            metadata={"export_module_attempt_id": export_module_attempt_id},
        )
        su2_case_attempt_id = persist_su2_dry_run_case(
            connection,
            artifact_root=artifact_root,
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            geometry_provider_result_id=geometry_record.geometry_provider_result_id,
            export_module_attempt_id=export_module_attempt_id,
            mesh_reference_path=request.mesh_reference_path,
        )
        module_attempt_ids.append(su2_case_attempt_id)
    else:
        su2_case_attempt_id = None

    if request.run_su2_history_fixture:
        if request.su2_history_fixture_path is None:
            raise ValueError("su2_history_fixture_path is required when SU2 history is enabled")
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=request.campaign_id,
            optimizer_run_id=request.optimizer_run_id,
            optimizer_iteration_id=request.optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="cfd_running",
            stage="cfd",
            active_module="su2_cfd_fixture",
            progress=0.65,
            priority=request.runner_priority,
            reason="SU2 history fixture",
            message="Candidate is recording fixture CFD results.",
            metadata={"case_builder_module_attempt_id": su2_case_attempt_id},
        )
        su2_history_attempt_id = persist_su2_history_fixture_result(
            connection,
            artifact_root=artifact_root,
            history_fixture_path=request.su2_history_fixture_path,
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            geometry_provider_result_id=geometry_record.geometry_provider_result_id,
            export_module_attempt_id=export_module_attempt_id,
            case_builder_module_attempt_id=su2_case_attempt_id,
        )
        module_attempt_ids.append(su2_history_attempt_id)

    if request.run_low_fidelity_aero:
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=request.campaign_id,
            optimizer_run_id=request.optimizer_run_id,
            optimizer_iteration_id=request.optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="running",
            stage="low_fidelity_aero",
            active_module=LOW_FIDELITY_MODULE_NAME,
            progress=0.75,
            priority=request.runner_priority,
            reason="low-fidelity aero scoring",
            message="Candidate is running low-fidelity aero analysis.",
            metadata={"method": "analytic_planform_screening"},
        )
        low_fidelity_attempt_id = run_successful_module_attempt(
            connection,
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_name=LOW_FIDELITY_MODULE_NAME,
            module_version=LOW_FIDELITY_MODULE_VERSION,
            module_kind="analysis",
            inputs={
                "geometry_provider_result_id": geometry_record.geometry_provider_result_id,
                "method": "analytic_planform_screening",
            },
            metrics=low_fidelity_fixed_wing_metrics(request.candidate),
            metadata={
                "low_fidelity": True,
                "screening_only": True,
                "not_cfd": True,
            },
            started_message="Started low-fidelity fixed-wing analysis module.",
            completed_message="Completed low-fidelity fixed-wing analysis module.",
        )
        module_attempt_ids.append(low_fidelity_attempt_id)

    if request.run_scoring:
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=request.campaign_id,
            optimizer_run_id=request.optimizer_run_id,
            optimizer_iteration_id=request.optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="running",
            stage="scoring",
            active_module="fixture_scoring",
            progress=0.85,
            priority=request.runner_priority,
            reason="fixture scoring",
            message="Candidate is running scoring module.",
            metadata={"export_module_attempt_id": export_module_attempt_id},
        )
        scoring_attempt_id = run_successful_module_attempt(
            connection,
            campaign_id=request.campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_name="fixture_scoring",
            module_version="0.1.0",
            module_kind="scoring",
            inputs={
                "geometry_module_attempt_id": geometry_metrics_attempt_id,
                "export_module_attempt_id": export_module_attempt_id,
            },
            metrics=fixture_scoring_metrics(request.candidate),
            metadata={"placeholder_scoring": True},
            started_message="Started fixture scoring module.",
            completed_message="Completed fixture scoring module.",
        )
        module_attempt_ids.append(scoring_attempt_id)

    complete_evaluation(
        connection,
        evaluation_id=evaluation_id,
        summary={
            "status": "complete",
            "geometry_provider_result_id": geometry_record.geometry_provider_result_id,
            "module_attempt_ids": module_attempt_ids,
            "artifact_ids": artifact_ids,
        },
    )
    log_event(
        connection,
        event_type="evaluation.completed",
        campaign_id=request.campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        message="Completed fixture candidate evaluation.",
    )
    update_candidate_status(
        connection,
        candidate_id=candidate_id,
        status="evaluated",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=request.campaign_id,
        optimizer_run_id=request.optimizer_run_id,
        optimizer_iteration_id=request.optimizer_iteration_id,
        evaluation_id=evaluation_id,
        state="scored",
        stage="scoring",
        active_module=None,
        progress=1.0,
        priority=request.runner_priority,
        reason="evaluation complete",
        message="Candidate evaluation completed and is ready for optimizer decision.",
        metadata={"module_attempt_ids": module_attempt_ids},
    )
    _maybe_record_annotation(connection, request, candidate_id, evaluation_id)
    return CandidateEvaluationResult(
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        status="complete",
        geometry_provider_result_id=geometry_record.geometry_provider_result_id,
        module_attempt_ids=module_attempt_ids,
        artifact_ids=artifact_ids,
        failure_id=None,
    )


def _maybe_record_annotation(
    connection: sqlite3.Connection,
    request: CandidateEvaluationRequest,
    candidate_id: str,
    evaluation_id: str,
) -> None:
    if request.annotation_tag is None:
        return
    annotation_id = create_user_annotation(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        user_label="system",
        tag=request.annotation_tag,
        comment=request.annotation_comment,
        metadata=request.annotation_metadata or {},
    )
    log_event(
        connection,
        event_type="annotation.created",
        campaign_id=request.campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        message="Recorded fixture candidate annotation.",
        payload={"annotation_id": annotation_id},
    )
