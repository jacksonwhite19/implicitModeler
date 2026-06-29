from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from aircraft_optimizer.db.connection import connect, initialize_database
from aircraft_optimizer.db.repositories import (
    complete_evaluation,
    complete_optimizer_iteration,
    complete_optimizer_run,
    create_campaign,
    create_environment_fingerprint,
    create_optimizer_iteration,
    create_optimizer_run,
    create_variable_schema,
    fail_evaluation,
    update_candidate_status,
    upsert_candidate_runner_state,
)
from aircraft_optimizer.db.report import optimizer_report
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.modules.pre_export_screening import (
    DEFAULT_SCREENING_POLICY,
    persist_pre_export_screening,
)
from aircraft_optimizer.optimizers.ask_tell import (
    HaltonAskTellPolicy,
    OptimizerObservation,
    OptimizerProposal,
)
from aircraft_optimizer.orchestration import (
    CandidateEvaluationRequest,
    evaluate_fixture_candidate,
    register_candidate_with_lineage,
    start_evaluation_record,
)
from aircraft_optimizer.pipelines import load_pipeline_config
from aircraft_optimizer.records import CandidateSeed
from aircraft_optimizer.schemas import load_variable_schema


def run_sequential_gated_study(
    workspace: Path,
    *,
    platform_root: Path,
    iterations: int = 3,
    virtual_components_config: Path | None = None,
) -> dict[str, Any]:
    if iterations < 1:
        raise ValueError("iterations must be at least 1")
    workspace.mkdir(parents=True, exist_ok=True)
    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "sequential_gated_v0_1" / "artifacts"
    connection = connect(db_path)
    initialize_database(connection)
    try:
        schema_path = (
            platform_root
            / "software"
            / "optimizer"
            / "schemas"
            / "fixed_wing_uav_reference.variables.v0_1.json"
        )
        variable_schema = load_variable_schema(schema_path)
        pipeline_path = (
            platform_root
            / "software"
            / "optimizer"
            / "pipelines"
            / "sequential_gated_v0_1.pipeline.json"
        )
        pipeline_config = load_pipeline_config(pipeline_path)
        virtual_component_assumptions = _load_virtual_component_assumptions(
            virtual_components_config
        )
        pre_export_policy = dict(DEFAULT_SCREENING_POLICY)
        if virtual_component_assumptions is not None:
            pre_export_policy["virtual_component_assumptions"] = virtual_component_assumptions
        variable_schema_id = create_variable_schema(
            connection,
            aircraft_family=variable_schema.aircraft_family,
            schema_version=variable_schema.schema_version,
            schema=variable_schema.raw,
        )
        campaign_id = create_campaign(
            connection,
            name="sequential gated optimizer v0.1",
            description="One-candidate-at-a-time gated optimizer skeleton with fixture export only.",
            aircraft_family=variable_schema.aircraft_family,
            variable_schema_id=variable_schema_id,
            config={
                "pipeline_id": pipeline_config.pipeline_id,
                "pipeline_version": pipeline_config.pipeline_version,
                "pipeline_config_path": str(pipeline_path),
                "pipeline": pipeline_config.raw,
                "real_exporter_execution": False,
                "real_cfd_execution": False,
                "pre_export_policy": pre_export_policy,
                "virtual_components_config_path": (
                    str(virtual_components_config) if virtual_components_config else None
                ),
            },
        )
        log_event(
            connection,
            event_type="campaign.created",
            campaign_id=campaign_id,
            actor="system",
            message="Created sequential gated optimizer campaign.",
        )
        environment_fingerprint_id = create_environment_fingerprint(
            connection,
            tools={
                "geometry": {"mode": "fixture_after_pre_export_gate"},
                "exporter": {"mode": "fixture_parse_only"},
                "cfd": {"checked_during_study": False},
            },
            notes="Sequential gated v0.1 uses cheap pre-export gates and fixture export only.",
        )
        optimizer_run_id = create_optimizer_run(
            connection,
            campaign_id=campaign_id,
            optimizer_name="sequential_gated_optimizer",
            optimizer_version="0.1.0",
            objective={
                "primary": "score.low_fidelity_total",
                "direction": "maximize",
                "scope": "sequential_gated_v0_1",
            },
            settings={
                "execution_model": "sequential",
                "candidate_batch_size": 1,
                "requested_iterations": iterations,
                "real_exporter_execution": False,
                "real_cfd_execution": False,
                "proposal_policy": "halton_ask_tell",
                "virtual_components_config_path": (
                    str(virtual_components_config) if virtual_components_config else None
                ),
            },
        )
        log_event(
            connection,
            event_type="optimizer.started",
            campaign_id=campaign_id,
            actor="optimizer",
            message="Started sequential gated optimizer run.",
            payload={"optimizer_run_id": optimizer_run_id},
        )

        completed_candidate_ids: list[str] = []
        failed_candidate_ids: list[str] = []
        all_candidate_ids: list[str] = []
        best_candidate_id: str | None = None
        policy = HaltonAskTellPolicy(variable_schema.raw)

        for index in range(iterations):
            proposal = policy.ask(index)
            candidate = proposal.candidate
            parent_ids = proposal.parent_candidate_ids
            iteration_id = create_optimizer_iteration(
                connection,
                optimizer_run_id=optimizer_run_id,
                campaign_id=campaign_id,
                iteration_index=index,
                parent_candidate_ids=parent_ids,
                proposed_candidate_ids=[],
                strategy="sequential_gated",
                rationale=proposal.rationale,
                optimizer_state=proposal.optimizer_state
                | {"design_variables": candidate.design_variables_dict()},
            )
            gate_result = _run_pre_export_gate(
                connection=connection,
                platform_root=platform_root,
                workspace=workspace,
                variable_schema=variable_schema,
                variable_schema_id=variable_schema_id,
                pipeline_id=pipeline_config.pipeline_id,
                pipeline_version=pipeline_config.pipeline_version,
                environment_fingerprint_id=environment_fingerprint_id,
                campaign_id=campaign_id,
                candidate=candidate,
                parent_candidate_ids=parent_ids,
                proposal=proposal,
                pre_export_policy=pre_export_policy,
                optimizer_run_id=optimizer_run_id,
                optimizer_iteration_id=iteration_id,
                runner_priority=index,
            )
            if gate_result["passed"]:
                result = evaluate_fixture_candidate(
                    connection,
                    platform_root=platform_root,
                    workspace=workspace,
                    artifact_root=artifact_root,
                    variable_schema=variable_schema,
                    request=CandidateEvaluationRequest(
                        campaign_id=campaign_id,
                        variable_schema_id=variable_schema_id,
                        pipeline_id=pipeline_config.pipeline_id,
                        pipeline_version=pipeline_config.pipeline_version,
                        environment_fingerprint_id=environment_fingerprint_id,
                        candidate=candidate,
                        parent_candidate_ids=parent_ids,
                        lineage_operator="sequential_gated",
                        lineage_reason=proposal.rationale,
                        mutation_summary={
                            "proposal_name": proposal.name,
                            "policy": "halton_ask_tell",
                        },
                        evaluation_reason=f"sequential gated evaluation {index}: {proposal.name}",
                        registration_message=f"Registered sequential gated candidate {proposal.name}.",
                        lineage_message=f"Recorded sequential gated lineage for {proposal.name}.",
                        evaluation_started_message=f"Started sequential gated evaluation for {proposal.name}.",
                        geometry_request_message=f"Requested fixture geometry after pre-export pass for {proposal.name}.",
                        geometry_completed_message=f"Generated fixture geometry for {proposal.name}.",
                        export_fixture_path=(
                            platform_root
                            / "software"
                            / "optimizer"
                            / "examples"
                            / "exporter_results"
                            / "oml_stl_passed.json"
                        ),
                        annotation_tag="sequential_gated",
                        annotation_comment=f"Sequential gated candidate: {proposal.name}.",
                        annotation_metadata={
                            "proposal_name": proposal.name,
                            "pre_export_gate": "passed",
                        },
                        run_low_fidelity_aero=True,
                        run_scoring=False,
                        existing_candidate_id=gate_result["candidate_id"],
                        existing_evaluation_id=gate_result["evaluation_id"],
                        preexisting_module_attempt_ids=[gate_result["module_attempt_id"]],
                        optimizer_run_id=optimizer_run_id,
                        optimizer_iteration_id=iteration_id,
                        runner_priority=index,
                    ),
                )
                candidate_id = result.candidate_id
                status = result.status
                if status == "complete":
                    completed_candidate_ids.append(candidate_id)
                    best_candidate_id = _best_candidate_id(connection, optimizer_run_id, fallback=candidate_id)
                    policy.tell(
                        OptimizerObservation(
                            candidate_id=candidate_id,
                            status=status,
                            score=_candidate_score(connection, candidate_id),
                            failure_category=None,
                        )
                    )
                else:
                    failed_candidate_ids.append(candidate_id)
                    policy.tell(
                        OptimizerObservation(
                            candidate_id=candidate_id,
                            status=status,
                            score=None,
                            failure_category="evaluation.failed",
                        )
                    )
                all_candidate_ids.append(candidate_id)
                iteration_state = {
                    "proposal_name": proposal.name,
                    "candidate_id": candidate_id,
                    "evaluation_id": result.evaluation_id,
                    "status": status,
                    "pre_export_gate": "passed",
                    "best_candidate_id_after_iteration": best_candidate_id,
                }
            else:
                candidate_id = gate_result["candidate_id"]
                failed_candidate_ids.append(candidate_id)
                all_candidate_ids.append(candidate_id)
                policy.tell(
                    OptimizerObservation(
                        candidate_id=candidate_id,
                        status="failed_pre_export",
                        score=None,
                        failure_category="optimization.pre_export_screen_failed",
                    )
                )
                iteration_state = {
                    "proposal_name": proposal.name,
                    "candidate_id": candidate_id,
                    "evaluation_id": gate_result["evaluation_id"],
                    "status": "failed_pre_export",
                    "pre_export_gate": "failed",
                    "failure_id": gate_result["failure_id"],
                    "best_candidate_id_after_iteration": best_candidate_id,
                }

            complete_optimizer_iteration(
                connection,
                optimizer_iteration_id=iteration_id,
                proposed_candidate_ids=[candidate_id],
                optimizer_state=iteration_state,
            )
            log_event(
                connection,
                event_type="optimizer.iteration.completed",
                campaign_id=campaign_id,
                candidate_id=candidate_id,
                evaluation_id=iteration_state["evaluation_id"],
                actor="optimizer",
                message=f"Completed sequential gated iteration for {proposal.name}.",
                payload={"optimizer_iteration_id": iteration_id},
            )
            connection.commit()

        report = optimizer_report(db_path, optimizer_run_id=optimizer_run_id)
        if report["ranking"]:
            best_candidate_id = report["ranking"][0]["candidate_id"]
        for candidate_id in all_candidate_ids:
            row = connection.execute(
                """
                SELECT evaluation_id
                FROM evaluations
                WHERE candidate_id = ?
                ORDER BY created_at DESC, rowid DESC
                LIMIT 1
                """,
                (candidate_id,),
            ).fetchone()
            if candidate_id in failed_candidate_ids:
                continue
            upsert_candidate_runner_state(
                connection,
                candidate_id=candidate_id,
                campaign_id=campaign_id,
                optimizer_run_id=optimizer_run_id,
                evaluation_id=row[0] if row else None,
                state="promoted" if candidate_id == best_candidate_id else "rejected",
                stage="optimizer_decision",
                progress=1.0,
                reason=(
                    "best candidate in sequential gated run"
                    if candidate_id == best_candidate_id
                    else "not selected after sequential gated run"
                ),
                message=(
                    "Candidate promoted by sequential gated optimizer."
                    if candidate_id == best_candidate_id
                    else "Candidate evaluated but not promoted by sequential gated optimizer."
                ),
                metadata={
                    "selection_basis": "score.low_fidelity_total",
                    "best_candidate_id": best_candidate_id,
                },
            )
        complete_optimizer_run(
            connection,
            optimizer_run_id=optimizer_run_id,
            summary={
                "status": "complete",
                "candidate_ids": all_candidate_ids,
                "evaluated_candidate_ids": completed_candidate_ids,
                "failed_candidate_ids": failed_candidate_ids,
                "best_candidate_id": best_candidate_id,
                "selection_basis": "score.low_fidelity_total",
                "execution_model": "sequential",
            },
        )
        log_event(
            connection,
            event_type="optimizer.completed",
            campaign_id=campaign_id,
            candidate_id=best_candidate_id,
            actor="optimizer",
            message="Completed sequential gated optimizer run.",
            payload={
                "optimizer_run_id": optimizer_run_id,
                "best_candidate_id": best_candidate_id,
            },
        )
        connection.commit()
        return {
            "workspace": str(workspace),
            "database": str(db_path),
            "campaign_id": campaign_id,
            "optimizer_run_id": optimizer_run_id,
            "candidate_ids": all_candidate_ids,
            "evaluated_candidate_ids": completed_candidate_ids,
            "failed_candidate_ids": failed_candidate_ids,
            "best_candidate_id": best_candidate_id,
            "execution_model": "sequential",
        }
    finally:
        connection.close()


def _run_pre_export_gate(
    *,
    connection: Any,
    platform_root: Path,
    workspace: Path,
    variable_schema: Any,
    variable_schema_id: str,
    pipeline_id: str,
    pipeline_version: str,
    environment_fingerprint_id: str,
    campaign_id: str,
    candidate: CandidateSeed,
    parent_candidate_ids: list[str],
    proposal: OptimizerProposal,
    pre_export_policy: dict[str, Any],
    optimizer_run_id: str | None = None,
    optimizer_iteration_id: str | None = None,
    runner_priority: int | None = None,
) -> dict[str, Any]:
    candidate.validate()
    variable_schema.validate_candidate(candidate)
    registration = register_candidate_with_lineage(
        connection,
        campaign_id=campaign_id,
        variable_schema_id=variable_schema_id,
        candidate=candidate,
        parent_candidate_ids=parent_candidate_ids,
        operator="sequential_gated_pre_export",
        reason=proposal.rationale,
        mutation_summary={
            "proposal_name": proposal.name,
            "policy": "halton_ask_tell",
        },
        registration_message=f"Registered pre-export candidate {proposal.name}.",
        lineage_message=f"Recorded pre-export lineage for {proposal.name}.",
    )
    candidate_id = registration.candidate_id
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=optimizer_iteration_id,
        state="queued",
        stage="optimizer_proposed",
        priority=runner_priority,
        reason=proposal.rationale,
        message="Candidate queued for sequential gated pre-export checks.",
        metadata={"proposal_name": proposal.name},
    )
    evaluation_id = start_evaluation_record(
        connection,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        pipeline_id=pipeline_id,
        pipeline_version=pipeline_version,
        requested_by="optimizer",
        reason=f"pre-export screening for {proposal.name}",
        environment_fingerprint_id=environment_fingerprint_id,
        message=f"Started pre-export screening evaluation for {proposal.name}.",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=optimizer_iteration_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="pre_export_checks",
        active_module="pre_export_screening",
        progress=0.10,
        priority=runner_priority,
        reason="pre-export screening",
        message="Candidate is running pre-export screening.",
        metadata={"proposal_name": proposal.name},
    )
    module_attempt_id, screening, failure_id = persist_pre_export_screening(
        connection,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        candidate=candidate,
        policy=pre_export_policy,
    )
    if screening.passed:
        update_candidate_status(connection, candidate_id=candidate_id, status="screened")
        upsert_candidate_runner_state(
            connection,
            candidate_id=candidate_id,
            campaign_id=campaign_id,
            optimizer_run_id=optimizer_run_id,
            optimizer_iteration_id=optimizer_iteration_id,
            evaluation_id=evaluation_id,
            state="running",
            stage="geometry",
            active_module=None,
            progress=0.20,
            priority=runner_priority,
            reason="pre-export screening passed",
            message="Candidate passed pre-export checks and will continue to geometry/export.",
            metadata={"proposal_name": proposal.name},
        )
        log_event(
            connection,
            event_type="evaluation.pre_export_passed",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            actor="system",
            message="Pre-export screening passed; continuing candidate evaluation.",
        )
        return {
            "passed": True,
            "candidate_id": candidate_id,
            "evaluation_id": evaluation_id,
            "failure_id": None,
            "module_attempt_id": module_attempt_id,
        }

    fail_evaluation(
        connection,
        evaluation_id=evaluation_id,
        failure_id=failure_id,
        summary={
            "status": "failed",
            "screening_only": True,
            "module_attempt_ids": [module_attempt_id],
            "failure_id": failure_id,
            "pre_export_gate": "failed",
        },
    )
    update_candidate_status(connection, candidate_id=candidate_id, status="failed")
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=optimizer_iteration_id,
        evaluation_id=evaluation_id,
        state="failed",
        stage="pre_export_checks",
        active_module="pre_export_screening",
        progress=1.0,
        priority=runner_priority,
        reason="pre-export screening failed",
        message="Candidate failed pre-export screening.",
        metadata={"proposal_name": proposal.name, "failure_id": failure_id},
    )
    log_event(
        connection,
        event_type="evaluation.failed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        severity="warning",
        message="Failed pre-export screening evaluation.",
        payload={"failure_id": failure_id},
    )
    return {
        "passed": False,
        "candidate_id": candidate_id,
        "evaluation_id": evaluation_id,
        "failure_id": failure_id,
        "module_attempt_id": module_attempt_id,
    }


def _best_candidate_id(connection: Any, optimizer_run_id: str, *, fallback: str) -> str:
    connection.commit()
    db_path = Path(connection.execute("PRAGMA database_list").fetchone()[2])
    report = optimizer_report(db_path, optimizer_run_id=optimizer_run_id)
    if report["ranking"]:
        return str(report["ranking"][0]["candidate_id"])
    return fallback


def _candidate_score(connection: Any, candidate_id: str) -> float | None:
    row = connection.execute(
        """
        SELECT metrics_json
        FROM module_attempts
        WHERE candidate_id = ?
          AND module_name = 'low_fidelity_fixed_wing'
          AND status = 'success'
        ORDER BY finished_at DESC
        LIMIT 1
        """,
        (candidate_id,),
    ).fetchone()
    if row is None:
        return None
    metrics = row[0]
    if isinstance(metrics, str):
        import json

        metrics = json.loads(metrics)
    score = metrics.get("score.low_fidelity_total")
    if not score:
        return None
    return float(score["value"])


def _load_virtual_component_assumptions(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("virtual component config must be a JSON object")
    return data
