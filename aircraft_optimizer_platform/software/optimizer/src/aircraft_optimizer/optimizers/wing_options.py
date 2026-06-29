from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.db.connection import connect, initialize_database
from aircraft_optimizer.db.repositories import (
    complete_optimizer_iteration,
    complete_optimizer_run,
    create_campaign,
    create_environment_fingerprint,
    create_optimizer_iteration,
    create_optimizer_run,
    create_variable_schema,
    upsert_candidate_runner_state,
)
from aircraft_optimizer.db.report import optimizer_report
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.orchestration import (
    CandidateEvaluationRequest,
    CandidateEvaluationResult,
    evaluate_fixture_candidate,
)
from aircraft_optimizer.pipelines import load_pipeline_config
from aircraft_optimizer.records import CandidateSeed, VariableValue
from aircraft_optimizer.schemas import load_variable_schema


@dataclass(frozen=True)
class WingOption:
    name: str
    span_mm: float
    root_chord_mm: float
    tip_chord_mm: float
    sweep_deg: float
    rationale: str


WING_OPTIONS = [
    WingOption(
        name="compact_low_sweep",
        span_mm=650.0,
        root_chord_mm=175.0,
        tip_chord_mm=80.0,
        sweep_deg=5.0,
        rationale="Compact lower-sweep reference option.",
    ),
    WingOption(
        name="high_aspect_moderate_sweep",
        span_mm=780.0,
        root_chord_mm=155.0,
        tip_chord_mm=58.0,
        sweep_deg=10.0,
        rationale="Higher aspect-ratio option with moderate sweep.",
    ),
    WingOption(
        name="baseline_swept_taper",
        span_mm=720.0,
        root_chord_mm=170.0,
        tip_chord_mm=65.0,
        sweep_deg=18.0,
        rationale="Baseline-like swept tapered option.",
    ),
    WingOption(
        name="broad_chord_stable",
        span_mm=680.0,
        root_chord_mm=205.0,
        tip_chord_mm=105.0,
        sweep_deg=12.0,
        rationale="Broader chord option that trades aspect ratio for planform area.",
    ),
    WingOption(
        name="long_high_sweep_taper",
        span_mm=850.0,
        root_chord_mm=150.0,
        tip_chord_mm=45.0,
        sweep_deg=25.0,
        rationale="Long highly tapered option with higher sweep penalty.",
    ),
]


def run_wing_options_study(workspace: Path, *, platform_root: Path) -> dict[str, Any]:
    workspace.mkdir(parents=True, exist_ok=True)
    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "wing_options_v0_1" / "artifacts"
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
            name="wing option sweep v0.1",
            description="Five deterministic no-inlet fixed-wing options evaluated through fixture adapters.",
            aircraft_family=variable_schema.aircraft_family,
            variable_schema_id=variable_schema_id,
            config={
                "pipeline_id": pipeline_config.pipeline_id,
                "pipeline_version": pipeline_config.pipeline_version,
                "pipeline": pipeline_config.raw,
                "real_cad_execution": False,
                "real_exporter_execution": False,
                "real_cfd_execution": False,
            },
        )
        log_event(
            connection,
            event_type="campaign.created",
            campaign_id=campaign_id,
            actor="system",
            message="Created wing option sweep campaign.",
        )
        environment_fingerprint_id = create_environment_fingerprint(
            connection,
            tools={
                "geometry": {"mode": "fixture_no_inlet_reference"},
                "exporter": {"mode": "fixture_parse_only"},
                "cfd": {"checked_during_study": False},
            },
            notes="Five-option wing sweep uses fixture adapters only.",
        )
        optimizer_run_id = create_optimizer_run(
            connection,
            campaign_id=campaign_id,
            optimizer_name="deterministic_wing_option_sweep",
            optimizer_version="0.1.0",
            objective={
                "primary": "score.low_fidelity_total",
                "direction": "maximize",
                "scope": "five_no_inlet_wing_options",
            },
            settings={
                "option_count": len(WING_OPTIONS),
                "proposal_policy": "deterministic_grid",
                "real_optimizer_algorithm": False,
            },
        )
        log_event(
            connection,
            event_type="optimizer.started",
            campaign_id=campaign_id,
            actor="optimizer",
            message="Started deterministic wing option sweep.",
            payload={"optimizer_run_id": optimizer_run_id},
        )

        results: list[CandidateEvaluationResult] = []
        for index, option in enumerate(WING_OPTIONS):
            candidate = _candidate_from_option(variable_schema.raw, option, generation=index)
            iteration_id = create_optimizer_iteration(
                connection,
                optimizer_run_id=optimizer_run_id,
                campaign_id=campaign_id,
                iteration_index=index,
                parent_candidate_ids=[],
                proposed_candidate_ids=[],
                strategy="deterministic_wing_option",
                rationale=option.rationale,
                optimizer_state={
                    "option_name": option.name,
                    "design_variables": candidate.design_variables_dict(),
                    "normalized_design_vector": candidate.normalized_design_vector,
                },
            )
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
                    parent_candidate_ids=[],
                    lineage_operator="deterministic_wing_option",
                    lineage_reason=option.rationale,
                    mutation_summary={
                        "option_name": option.name,
                        "wing.span_mm": option.span_mm,
                        "wing.root_chord_mm": option.root_chord_mm,
                        "wing.tip_chord_mm": option.tip_chord_mm,
                        "wing.sweep_deg": option.sweep_deg,
                    },
                    evaluation_reason=f"evaluate wing option {index + 1}: {option.name}",
                    registration_message=f"Registered wing option candidate {option.name}.",
                    lineage_message=f"Recorded wing option lineage for {option.name}.",
                    evaluation_started_message=f"Started wing option evaluation for {option.name}.",
                    geometry_request_message=f"Requested no-inlet fixture geometry for {option.name}.",
                    geometry_completed_message=f"Generated no-inlet fixture geometry for {option.name}.",
                    export_fixture_path=(
                        platform_root
                        / "software"
                        / "optimizer"
                        / "examples"
                        / "exporter_results"
                        / "oml_stl_passed.json"
                    ),
                    annotation_tag="wing_option",
                    annotation_comment=f"Deterministic wing option: {option.name}.",
                    annotation_metadata={
                        "option_name": option.name,
                        "study": "wing_options_v0_1",
                    },
                    run_low_fidelity_aero=True,
                    optimizer_run_id=optimizer_run_id,
                    optimizer_iteration_id=iteration_id,
                    runner_priority=index,
                ),
            )
            complete_optimizer_iteration(
                connection,
                optimizer_iteration_id=iteration_id,
                proposed_candidate_ids=[result.candidate_id],
                optimizer_state={
                    "option_name": option.name,
                    "candidate_id": result.candidate_id,
                    "evaluation_id": result.evaluation_id,
                    "status": result.status,
                },
            )
            log_event(
                connection,
                event_type="optimizer.iteration.completed",
                campaign_id=campaign_id,
                candidate_id=result.candidate_id,
                evaluation_id=result.evaluation_id,
                actor="optimizer",
                message=f"Completed wing option iteration for {option.name}.",
                payload={"optimizer_iteration_id": iteration_id},
            )
            results.append(result)

        connection.commit()
        report = optimizer_report(db_path, optimizer_run_id=optimizer_run_id)
        best_candidate_id = (
            report["ranking"][0]["candidate_id"] if report["ranking"] else None
        )
        for result in results:
            upsert_candidate_runner_state(
                connection,
                candidate_id=result.candidate_id,
                campaign_id=campaign_id,
                optimizer_run_id=optimizer_run_id,
                evaluation_id=result.evaluation_id,
                state="promoted" if result.candidate_id == best_candidate_id else "rejected",
                stage="optimizer_decision",
                progress=1.0,
                reason=(
                    "best candidate in deterministic wing option sweep"
                    if result.candidate_id == best_candidate_id
                    else "not selected after deterministic wing option sweep"
                ),
                message=(
                    "Candidate promoted by wing option sweep."
                    if result.candidate_id == best_candidate_id
                    else "Candidate evaluated but not promoted by wing option sweep."
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
                "candidate_ids": [result.candidate_id for result in results],
                "failed_candidate_ids": [
                    result.candidate_id for result in results if result.status == "failed"
                ],
                "best_candidate_id": best_candidate_id,
                "selection_basis": "score.low_fidelity_total",
            },
        )
        log_event(
            connection,
            event_type="optimizer.completed",
            campaign_id=campaign_id,
            candidate_id=best_candidate_id,
            actor="optimizer",
            message="Completed deterministic wing option sweep.",
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
            "candidate_ids": [result.candidate_id for result in results],
            "best_candidate_id": best_candidate_id,
        }
    finally:
        connection.close()


def _candidate_from_option(
    schema: dict[str, Any],
    option: WingOption,
    *,
    generation: int,
) -> CandidateSeed:
    values = {
        "wing.span_mm": (option.span_mm, "mm"),
        "wing.root_chord_mm": (option.root_chord_mm, "mm"),
        "wing.tip_chord_mm": (option.tip_chord_mm, "mm"),
        "wing.sweep_deg": (option.sweep_deg, "deg"),
    }
    return CandidateSeed(
        aircraft_family=schema["aircraft_family"],
        design_variables={
            name: VariableValue(value, unit) for name, (value, unit) in values.items()
        },
        normalized_design_vector={
            name: _normalize(schema, name, value) for name, (value, _) in values.items()
        },
        created_by="deterministic_wing_option_sweep",
        generation=generation,
        notes=f"Five-option no-inlet wing sweep candidate: {option.name}.",
    )


def _normalize(schema: dict[str, Any], variable_name: str, value: float) -> float:
    variable = schema["variables"][variable_name]
    minimum = float(variable["min"])
    maximum = float(variable["max"])
    return round((float(value) - minimum) / (maximum - minimum), 6)
