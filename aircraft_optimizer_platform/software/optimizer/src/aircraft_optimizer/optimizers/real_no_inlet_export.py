from __future__ import annotations

import sqlite3
from pathlib import Path
from typing import Any

from aircraft_optimizer.artifacts.registry import register_artifact
from aircraft_optimizer.db.connection import connect, initialize_database
from aircraft_optimizer.db.repositories import (
    complete_evaluation,
    create_campaign,
    create_environment_fingerprint,
    create_geometry_provider_result,
    create_user_annotation,
    create_variable_schema,
    fail_evaluation,
    update_candidate_status,
    upsert_candidate_runner_state,
)
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.external.real_adapter_boundary import (
    build_real_no_inlet_export_request,
)
from aircraft_optimizer.geometry.real_no_inlet_generator import (
    generate_real_no_inlet_candidate_geometry,
)
from aircraft_optimizer.modules.oml_stl_real_adapter import execute_real_oml_stl_export
from aircraft_optimizer.orchestration import (
    register_candidate_with_lineage,
    start_evaluation_record,
)
from aircraft_optimizer.pipelines import load_pipeline_config
from aircraft_optimizer.optimizers.ask_tell import HaltonAskTellPolicy, OptimizerObservation
from aircraft_optimizer.records import CandidateSeed, VariableValue
from aircraft_optimizer.schemas import load_variable_schema


def run_real_no_inlet_export_once(
    workspace: Path,
    *,
    platform_root: Path,
    timeout_seconds: int = 1200,
) -> dict[str, Any]:
    workspace = workspace.resolve()
    workspace.mkdir(parents=True, exist_ok=True)
    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "real_no_inlet_export" / "artifacts"
    connection = connect(db_path)
    initialize_database(connection)
    try:
        variable_schema = load_variable_schema(
            platform_root
            / "software"
            / "optimizer"
            / "schemas"
            / "fixed_wing_uav_reference.variables.v0_1.json"
        )
        pipeline_config = load_pipeline_config(
            platform_root
            / "software"
            / "optimizer"
            / "pipelines"
            / "v0_1_fixture.pipeline.json"
        )
        variable_schema_id = create_variable_schema(
            connection,
            aircraft_family=variable_schema.aircraft_family,
            schema_version=variable_schema.schema_version,
            schema=variable_schema.raw,
        )
        campaign_id = create_campaign(
            connection,
            name="real no-inlet OML export validation",
            description="Single controlled no-inlet geometry export; no aero or CFD analysis.",
            aircraft_family=variable_schema.aircraft_family,
            variable_schema_id=variable_schema_id,
            config={
                "pipeline_id": pipeline_config.pipeline_id,
                "pipeline_version": pipeline_config.pipeline_version,
                "real_exporter_execution": True,
                "real_cad_execution": False,
                "real_cfd_execution": False,
            },
        )
        log_event(
            connection,
            event_type="campaign.created",
            campaign_id=campaign_id,
            actor="system",
            message="Created real no-inlet export validation campaign.",
        )
        environment_fingerprint_id = create_environment_fingerprint(
            connection,
            tools={
                "geometry": {"mode": "curated_no_inlet_rhai"},
                "exporter": {"mode": "real_direct_sparse_oml_fast"},
                "analysis": {"enabled": False},
            },
            notes="Single real export validation; no analysis modules.",
        )
        candidate = CandidateSeed(
            aircraft_family=variable_schema.aircraft_family,
            design_variables={
                "wing.span_mm": VariableValue(700.0, "mm"),
                "wing.root_chord_mm": VariableValue(170.0, "mm"),
                "wing.tip_chord_mm": VariableValue(65.0, "mm"),
                "wing.sweep_deg": VariableValue(15.0, "deg"),
            },
            normalized_design_vector={
                "wing.span_mm": 0.5,
                "wing.root_chord_mm": 0.5,
                "wing.tip_chord_mm": 0.5,
                "wing.sweep_deg": 0.5,
            },
            created_by="real_no_inlet_export_validation",
            generation=0,
            notes="Controlled baseline no-inlet OML export validation candidate.",
        )
        variable_schema.validate_candidate(candidate)
        registration = register_candidate_with_lineage(
            connection,
            campaign_id=campaign_id,
            variable_schema_id=variable_schema_id,
            candidate=candidate,
            parent_candidate_ids=[],
            operator="real_no_inlet_export_validation",
            reason="single controlled real STL export validation",
            mutation_summary={"initial_seed": True, "real_export": True},
            registration_message="Registered real no-inlet export validation candidate.",
            lineage_message="Recorded real no-inlet export validation lineage.",
        )
        candidate_id = registration.candidate_id
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=campaign_id,
            state="queued",
            stage="export_validation",
            priority=0,
            reason="single controlled real STL export validation",
            message="Candidate queued for real no-inlet export validation.",
            metadata={"source": "real_no_inlet_export_once"},
        )
        evaluation_id = start_evaluation_record(
            connection,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            pipeline_id=pipeline_config.pipeline_id,
            pipeline_version=pipeline_config.pipeline_version,
            requested_by="system",
            reason="single controlled no-inlet OML STL export",
            environment_fingerprint_id=environment_fingerprint_id,
            message="Started real no-inlet export validation evaluation.",
        )
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=campaign_id,
            evaluation_id=evaluation_id,
            state="running",
            stage="geometry",
            active_module="manual_reference_geometry_provider",
            progress=0.20,
            priority=0,
            reason="manual reference geometry",
            message="Candidate is recording manual reference geometry.",
            metadata={"source": "real_no_inlet_export_once"},
        )
        geometry_result_id, geometry_artifact_id = _record_generated_geometry(
            connection,
            platform_root=platform_root,
            workspace=workspace,
            artifact_root=artifact_root,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            variable_schema_id=variable_schema_id,
            variable_schema=variable_schema,
            candidate=candidate,
        )
        export_request = build_real_no_inlet_export_request(
            platform_root=platform_root,
            workspace=workspace,
            execution_enabled=True,
            geometry_source_path=_geometry_script_path(connection, geometry_result_id),
            result_dir_name="real_no_inlet_export",
            result_json_name="no_inlet_oml_export_result.json",
            output_stem=f"direct_sdf_oml_{candidate_id[-8:]}",
        )
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=campaign_id,
            evaluation_id=evaluation_id,
            state="running",
            stage="export",
            active_module="oml_stl_real_export",
            progress=0.55,
            priority=0,
            reason="real direct sparse OML export",
            message="Candidate is running the real OML STL exporter.",
            metadata={"source": "real_no_inlet_export_once"},
        )
        module_attempt_id = execute_real_oml_stl_export(
            connection,
            artifact_root=artifact_root,
            export_request=export_request,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            geometry_provider_result_id=geometry_result_id,
            timeout_seconds=timeout_seconds,
        )
        module_status, failure_id = connection.execute(
            """
            SELECT status, failure_id
            FROM module_attempts
            WHERE module_attempt_id = ?
            """,
            (module_attempt_id,),
        ).fetchone()
        artifact_ids = [
            row[0]
            for row in connection.execute(
                "SELECT artifact_id FROM artifacts WHERE evaluation_id = ? ORDER BY created_at",
                (evaluation_id,),
            )
        ]
        if module_status == "success":
            complete_evaluation(
                connection,
                evaluation_id=evaluation_id,
                summary={
                    "status": "complete",
                    "geometry_provider_result_id": geometry_result_id,
                    "module_attempt_ids": [module_attempt_id],
                    "artifact_ids": artifact_ids,
                    "analysis_modules_run": False,
                },
            )
            log_event(
                connection,
                event_type="evaluation.completed",
                campaign_id=campaign_id,
                candidate_id=candidate_id,
                evaluation_id=evaluation_id,
                actor="system",
                message="Completed real no-inlet export validation evaluation.",
            )
            update_candidate_status(
                connection,
                candidate_id=candidate_id,
                status="evaluated",
            )
            upsert_candidate_runner_state(
                connection,
                candidate_id=candidate_id,
                campaign_id=campaign_id,
                evaluation_id=evaluation_id,
                state="scored",
                stage="export_validation",
                active_module=None,
                progress=1.0,
                priority=0,
                reason="real export validation complete",
                message="Candidate passed real no-inlet OML export validation.",
                metadata={
                    "source": "real_no_inlet_export_once",
                    "module_attempt_id": module_attempt_id,
                },
            )
        else:
            fail_evaluation(
                connection,
                evaluation_id=evaluation_id,
                failure_id=failure_id,
                summary={
                    "status": "failed",
                    "geometry_provider_result_id": geometry_result_id,
                    "module_attempt_ids": [module_attempt_id],
                    "artifact_ids": artifact_ids,
                    "failure_id": failure_id,
                    "analysis_modules_run": False,
                },
            )
            log_event(
                connection,
                event_type="evaluation.failed",
                campaign_id=campaign_id,
                candidate_id=candidate_id,
                evaluation_id=evaluation_id,
                actor="system",
                severity="error",
                message="Failed real no-inlet export validation evaluation.",
                payload={"failure_id": failure_id},
            )
            update_candidate_status(
                connection,
                candidate_id=candidate_id,
                status="failed",
            )
            upsert_candidate_runner_state(
                connection,
                candidate_id=candidate_id,
                campaign_id=campaign_id,
                evaluation_id=evaluation_id,
                state="failed",
                stage="export",
                active_module="oml_stl_real_export",
                progress=1.0,
                priority=0,
                reason="real export validation failed",
                message="Candidate failed real no-inlet OML export validation.",
                metadata={
                    "source": "real_no_inlet_export_once",
                    "module_attempt_id": module_attempt_id,
                    "failure_id": failure_id,
                },
            )
        annotation_id = create_user_annotation(
            connection,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            user_label="system",
            tag="real_export_validation",
            comment="Single controlled no-inlet OML STL export. No analysis modules run.",
            metadata={"module_attempt_id": module_attempt_id},
        )
        log_event(
            connection,
            event_type="annotation.created",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            actor="system",
            message="Recorded real export validation annotation.",
            payload={"annotation_id": annotation_id},
        )
        connection.commit()
        stl_artifacts = [
            row
            for row in connection.execute(
                """
                SELECT artifact_id, path, status
                FROM artifacts
                WHERE evaluation_id = ? AND artifact_type = 'oml_stl'
                ORDER BY created_at
                """,
                (evaluation_id,),
            )
        ]
        return {
            "workspace": str(workspace),
            "database": str(db_path),
            "campaign_id": campaign_id,
            "candidate_id": candidate_id,
            "evaluation_id": evaluation_id,
            "geometry_provider_result_id": geometry_result_id,
            "geometry_artifact_id": geometry_artifact_id,
            "module_attempt_id": module_attempt_id,
            "module_status": module_status,
            "failure_id": failure_id,
            "analysis_modules_run": False,
            "stl_artifacts": [
                {"artifact_id": row[0], "path": row[1], "status": row[2]}
                for row in stl_artifacts
            ],
        }
    finally:
        connection.close()


def run_real_no_inlet_export_batch(
    workspace: Path,
    *,
    platform_root: Path,
    iterations: int = 5,
    timeout_seconds: int = 1200,
) -> dict[str, Any]:
    if iterations < 1:
        raise ValueError("iterations must be at least 1")
    workspace = workspace.resolve()
    workspace.mkdir(parents=True, exist_ok=True)
    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "real_no_inlet_export_batch" / "artifacts"
    connection = connect(db_path)
    initialize_database(connection)
    try:
        variable_schema = load_variable_schema(
            platform_root
            / "software"
            / "optimizer"
            / "schemas"
            / "fixed_wing_uav_reference.variables.v0_1.json"
        )
        pipeline_config = load_pipeline_config(
            platform_root
            / "software"
            / "optimizer"
            / "pipelines"
            / "v0_1_fixture.pipeline.json"
        )
        variable_schema_id = create_variable_schema(
            connection,
            aircraft_family=variable_schema.aircraft_family,
            schema_version=variable_schema.schema_version,
            schema=variable_schema.raw,
        )
        campaign_id = create_campaign(
            connection,
            name="real no-inlet OML export batch validation",
            description="Controlled no-inlet geometry export batch; no aero, mesh, or CFD analysis.",
            aircraft_family=variable_schema.aircraft_family,
            variable_schema_id=variable_schema_id,
            config={
                "pipeline_id": pipeline_config.pipeline_id,
                "pipeline_version": pipeline_config.pipeline_version,
                "real_exporter_execution": True,
                "real_cad_execution": False,
                "real_cfd_execution": False,
                "iterations": iterations,
                "proposal_policy": "halton_ask_tell",
            },
        )
        log_event(
            connection,
            event_type="campaign.created",
            campaign_id=campaign_id,
            actor="system",
            message="Created real no-inlet export batch validation campaign.",
        )
        environment_fingerprint_id = create_environment_fingerprint(
            connection,
            tools={
                "geometry": {"mode": "generated_no_inlet_rhai"},
                "exporter": {"mode": "real_direct_sparse_oml_fast"},
                "analysis": {"enabled": False},
                "cfd": {"enabled": False},
            },
            notes="Real export batch validation; no meshing or CFD.",
        )
        policy = HaltonAskTellPolicy(variable_schema.raw)
        candidate_results: list[dict[str, Any]] = []
        for index in range(iterations):
            proposal = policy.ask(index)
            result = _run_real_export_candidate(
                connection=connection,
                platform_root=platform_root,
                workspace=workspace,
                artifact_root=artifact_root,
                variable_schema=variable_schema,
                variable_schema_id=variable_schema_id,
                pipeline_id=pipeline_config.pipeline_id,
                pipeline_version=pipeline_config.pipeline_version,
                environment_fingerprint_id=environment_fingerprint_id,
                campaign_id=campaign_id,
                candidate=proposal.candidate,
                parent_candidate_ids=proposal.parent_candidate_ids,
                lineage_reason=proposal.rationale,
                mutation_summary={
                    "proposal_name": proposal.name,
                    "policy": "halton_ask_tell",
                    "real_export_batch_index": index,
                },
                timeout_seconds=timeout_seconds,
                priority=index,
            )
            candidate_results.append(result)
            policy.tell(
                # The batch is not an optimizer scoring run; this only gives the
                # deterministic policy a parent for lineage continuity.
                OptimizerObservation(
                    candidate_id=result["candidate_id"],
                    status="complete" if result["module_status"] == "success" else "failed",
                    score=None,
                    failure_category=result["failure_id"],
                )
            )
            connection.commit()

        return {
            "workspace": str(workspace),
            "database": str(db_path),
            "campaign_id": campaign_id,
            "analysis_modules_run": False,
            "cfd_run": False,
            "candidate_results": candidate_results,
            "passed_export_count": sum(
                1 for item in candidate_results if item["module_status"] == "success"
            ),
            "failed_export_count": sum(
                1 for item in candidate_results if item["module_status"] != "success"
            ),
        }
    finally:
        connection.close()


def _run_real_export_candidate(
    *,
    connection: sqlite3.Connection,
    platform_root: Path,
    workspace: Path,
    artifact_root: Path,
    variable_schema: Any,
    variable_schema_id: str,
    pipeline_id: str,
    pipeline_version: str,
    environment_fingerprint_id: str,
    campaign_id: str,
    candidate: CandidateSeed,
    parent_candidate_ids: list[str],
    lineage_reason: str,
    mutation_summary: dict[str, Any],
    timeout_seconds: int,
    priority: int,
) -> dict[str, Any]:
    variable_schema.validate_candidate(candidate)
    registration = register_candidate_with_lineage(
        connection,
        campaign_id=campaign_id,
        variable_schema_id=variable_schema_id,
        candidate=candidate,
        parent_candidate_ids=parent_candidate_ids,
        operator="real_no_inlet_export_batch",
        reason=lineage_reason,
        mutation_summary=mutation_summary,
        registration_message="Registered real no-inlet export batch candidate.",
        lineage_message="Recorded real no-inlet export batch lineage.",
    )
    candidate_id = registration.candidate_id
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        state="queued",
        stage="export_validation",
        priority=priority,
        reason="real no-inlet export batch",
        message="Candidate queued for real no-inlet export batch validation.",
        metadata=mutation_summary,
    )
    evaluation_id = start_evaluation_record(
        connection,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        pipeline_id=pipeline_id,
        pipeline_version=pipeline_version,
        requested_by="system",
        reason="real no-inlet OML STL export batch",
        environment_fingerprint_id=environment_fingerprint_id,
        message="Started real no-inlet export batch evaluation.",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="geometry",
        active_module="real_no_inlet_candidate_geometry_generator",
        progress=0.20,
        priority=priority,
        reason="generated no-inlet geometry",
        message="Candidate is generating no-inlet Rhai geometry from design variables.",
        metadata=mutation_summary,
    )
    geometry_result_id, _geometry_artifact_id = _record_generated_geometry(
        connection,
        platform_root=platform_root,
        workspace=workspace,
        artifact_root=artifact_root,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        variable_schema_id=variable_schema_id,
        variable_schema=variable_schema,
        candidate=candidate,
    )
    export_request = build_real_no_inlet_export_request(
        platform_root=platform_root,
        workspace=workspace,
        execution_enabled=True,
        geometry_source_path=_geometry_script_path(connection, geometry_result_id),
        result_dir_name=f"real_no_inlet_export_batch/{candidate_id}",
        result_json_name="no_inlet_oml_export_result.json",
        output_stem=f"direct_sdf_oml_{priority:02d}_{candidate_id[-8:]}",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="export",
        active_module="oml_stl_real_export",
        progress=0.55,
        priority=priority,
        reason="real direct sparse OML export",
        message="Candidate is running the real OML STL exporter.",
        metadata=mutation_summary,
    )
    module_attempt_id = execute_real_oml_stl_export(
        connection,
        artifact_root=artifact_root,
        export_request=export_request,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        geometry_provider_result_id=geometry_result_id,
        timeout_seconds=timeout_seconds,
    )
    module_status, failure_id = connection.execute(
        """
        SELECT status, failure_id
        FROM module_attempts
        WHERE module_attempt_id = ?
        """,
        (module_attempt_id,),
    ).fetchone()
    artifact_ids = [
        row[0]
        for row in connection.execute(
            "SELECT artifact_id FROM artifacts WHERE evaluation_id = ? ORDER BY created_at",
            (evaluation_id,),
        )
    ]
    if module_status == "success":
        complete_evaluation(
            connection,
            evaluation_id=evaluation_id,
            summary={
                "status": "complete",
                "geometry_provider_result_id": geometry_result_id,
                "module_attempt_ids": [module_attempt_id],
                "artifact_ids": artifact_ids,
                "analysis_modules_run": False,
                "cfd_run": False,
            },
        )
        update_candidate_status(connection, candidate_id=candidate_id, status="evaluated")
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=campaign_id,
            evaluation_id=evaluation_id,
            state="scored",
            stage="export_validation",
            active_module=None,
            progress=1.0,
            priority=priority,
            reason="real export validation complete",
            message="Candidate passed real no-inlet OML export validation.",
            metadata=mutation_summary | {"module_attempt_id": module_attempt_id},
        )
    else:
        fail_evaluation(
            connection,
            evaluation_id=evaluation_id,
            failure_id=failure_id,
            summary={
                "status": "failed",
                "geometry_provider_result_id": geometry_result_id,
                "module_attempt_ids": [module_attempt_id],
                "artifact_ids": artifact_ids,
                "failure_id": failure_id,
                "analysis_modules_run": False,
                "cfd_run": False,
            },
        )
        update_candidate_status(connection, candidate_id=candidate_id, status="failed")
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=campaign_id,
            evaluation_id=evaluation_id,
            state="failed",
            stage="export",
            active_module="oml_stl_real_export",
            progress=1.0,
            priority=priority,
            reason="real export validation failed",
            message="Candidate failed real no-inlet OML export validation.",
            metadata=mutation_summary
            | {"module_attempt_id": module_attempt_id, "failure_id": failure_id},
        )
    annotation_id = create_user_annotation(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        user_label="system",
        tag="real_export_batch_validation",
        comment="Controlled no-inlet OML STL export batch. No analysis, mesh, or CFD modules run.",
        metadata={"module_attempt_id": module_attempt_id},
    )
    log_event(
        connection,
        event_type="annotation.created",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        message="Recorded real export batch validation annotation.",
        payload={"annotation_id": annotation_id},
    )
    return {
        "candidate_id": candidate_id,
        "evaluation_id": evaluation_id,
        "geometry_provider_result_id": geometry_result_id,
        "module_attempt_id": module_attempt_id,
        "module_status": module_status,
        "failure_id": failure_id,
        "result_json": str(export_request.result_json_path),
        "geometry_source_path": str(export_request.geometry_source_path),
    }


def _record_generated_geometry(
    connection: sqlite3.Connection,
    *,
    platform_root: Path,
    workspace: Path,
    artifact_root: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    variable_schema_id: str,
    variable_schema: Any,
    candidate: CandidateSeed,
) -> tuple[str, str]:
    log_event(
        connection,
        event_type="geometry.requested",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="module",
        message="Requested generated no-inlet candidate geometry.",
    )
    reference = generate_real_no_inlet_candidate_geometry(
        platform_root=platform_root,
        workspace=workspace,
        candidate_id=candidate_id,
        variable_schema_id=variable_schema_id,
        variable_schema=variable_schema,
        candidate=candidate,
    )
    geometry_artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=reference.script_path,
        artifact_type="geometry_source_rhai",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=None,
        producer_module="real_no_inlet_candidate_geometry_generator",
        producer_version="0.1.0",
        source_kind="generated_candidate_geometry_copy",
        source_original_path=str(reference.script_path),
        metadata=reference.metadata,
    )
    trace_artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=reference.trace_path,
        artifact_type="geometry_parameter_trace_json",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=None,
        producer_module="real_no_inlet_candidate_geometry_generator",
        producer_version="0.1.0",
        source_kind="generated_candidate_trace_copy",
        source_original_path=str(reference.trace_path),
        metadata=reference.metadata,
    )
    geometry_result_id = create_geometry_provider_result(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        provider_name="real_no_inlet_candidate_geometry_generator",
        provider_version="0.1.0",
        provider_kind="generated_candidate_rhai",
        representation="rhai_sdf",
        script_path=str(reference.script_path),
        feature_name=reference.feature_name,
        coordinate_frame=reference.coordinate_frame,
        bbox_min_mm=reference.bbox_min_mm,
        bbox_max_mm=reference.bbox_max_mm,
        source_hash=reference.source_hash,
        parameter_trace={
            "variable_schema_id": variable_schema_id,
            "design_variables": candidate.design_variables_dict(),
            "real_export_geometry": True,
        },
        metadata={
            **reference.metadata,
            "artifact_id": geometry_artifact_id,
            "trace_artifact_id": trace_artifact_id,
        },
    )
    log_event(
        connection,
        event_type="geometry.completed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="module",
        message="Generated no-inlet candidate geometry.",
        payload={"geometry_provider_result_id": geometry_result_id},
    )
    log_event(
        connection,
        event_type="artifact.created",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="module",
        message="Registered generated no-inlet geometry source artifact.",
        payload={"artifact_id": geometry_artifact_id},
    )
    return geometry_result_id, geometry_artifact_id


def _geometry_script_path(connection: sqlite3.Connection, geometry_result_id: str) -> Path:
    row = connection.execute(
        """
        SELECT script_path
        FROM geometry_provider_results
        WHERE geometry_provider_result_id = ?
        """,
        (geometry_result_id,),
    ).fetchone()
    if row is None:
        raise ValueError(f"geometry provider result not found: {geometry_result_id}")
    return Path(row[0])
