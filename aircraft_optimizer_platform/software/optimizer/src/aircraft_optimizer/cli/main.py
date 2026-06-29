from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from aircraft_optimizer.db.connection import connect, initialize_database
from aircraft_optimizer.db.repositories import (
    complete_evaluation,
    complete_campaign,
    complete_optimizer_iteration,
    complete_optimizer_run,
    create_campaign,
    create_environment_fingerprint,
    create_optimizer_iteration,
    create_optimizer_run,
    create_user_annotation,
    create_variable_schema,
    fail_evaluation,
    update_candidate_status,
    upsert_candidate_runner_state,
)
from aircraft_optimizer.db.report import (
    campaign_report,
    candidate_comparison_report,
    evaluation_report,
    layout_report,
    optimizer_report,
)
from aircraft_optimizer.db.summary import summarize_database
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.external import (
    check_wsl_cfd_tools,
    preflight_real_no_inlet_export_boundary,
)
from aircraft_optimizer.modules.mock_module import mock_geometry_metrics
from aircraft_optimizer.modules.mock_scoring import fixture_scoring_metrics
from aircraft_optimizer.modules.cfd_readiness import evaluate_cfd_readiness
from aircraft_optimizer.modules.cfd_surface_quality import evaluate_cfd_surface_quality
from aircraft_optimizer.modules.mesh_result_validation import validate_mesh_result
from aircraft_optimizer.modules.oml_stl_dry_run_adapter import (
    persist_oml_stl_fixture_result,
)
from aircraft_optimizer.modules.openfoam_result_validation import (
    policy_for_acceptance_mode,
    validate_openfoam_result,
)
from aircraft_optimizer.modules.openfoam_smoke_adapter import persist_openfoam_smoke_result
from aircraft_optimizer.modules.openfoam_steady_adapter import persist_openfoam_steady_result
from aircraft_optimizer.modules.openfoam_steady_result_validation import (
    OPENFOAM_STEADY_ACCEPTANCE_POLICIES,
    SNAPPY_HIFI_COEFFICIENT_DEVELOPMENT_POLICY,
    find_force_coeffs,
    validate_openfoam_steady_result,
)
from aircraft_optimizer.modules.rough_cfd_scoring import (
    MODULE_NAME as ROUGH_CFD_SCORING_MODULE_NAME,
    MODULE_VERSION as ROUGH_CFD_SCORING_MODULE_VERSION,
    stable_efficient_drone_rough_score,
)
from aircraft_optimizer.modules.snappy_surface_fidelity_adapter import (
    persist_snappy_surface_fidelity_result,
)
from aircraft_optimizer.modules.su2_case_builder import persist_su2_dry_run_case
from aircraft_optimizer.modules.su2_fixture_adapter import (
    persist_su2_history_fixture_result,
)
from aircraft_optimizer.exporters import parse_oml_stl_export_result
from aircraft_optimizer.orchestration import (
    persist_fixture_geometry,
    register_candidate_with_lineage,
    run_successful_module_attempt,
    start_evaluation_record,
)
from aircraft_optimizer.optimizers import (
    AdaptiveCfdPromotionPolicy,
    adaptive_cfd_promotion_report,
    decide_adaptive_cfd_promotion,
    run_real_no_inlet_export_batch,
    run_real_no_inlet_export_once,
    run_sequential_gated_study,
    run_wing_options_study,
)
from aircraft_optimizer.pipelines import load_pipeline_config
from aircraft_optimizer.records import CandidateSeed, VariableValue
from aircraft_optimizer.schemas import load_variable_schema


def find_platform_root() -> Path:
    return Path(__file__).resolve().parents[5]


def run_fixture(workspace: Path) -> None:
    platform_root = find_platform_root()
    workspace.mkdir(parents=True, exist_ok=True)
    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "v0_1_fixture" / "artifacts"

    connection = connect(db_path)
    initialize_database(connection)

    aircraft_family = "fixed_wing_uav_reference"
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
        / "v0_1_fixture.pipeline.json"
    )
    pipeline_config = load_pipeline_config(pipeline_path)
    variable_schema_id = create_variable_schema(
        connection,
        aircraft_family=aircraft_family,
        schema_version=variable_schema.schema_version,
        schema=variable_schema.raw,
    )
    campaign_id = create_campaign(
        connection,
        name="v0.1 fixture campaign",
        description="Records-only fixture; does not execute CAD or exporter tools.",
        aircraft_family=aircraft_family,
        variable_schema_id=variable_schema_id,
        config={
            "pipeline_id": pipeline_config.pipeline_id,
            "pipeline_version": pipeline_config.pipeline_version,
            "pipeline_config_path": str(pipeline_path),
            "pipeline": pipeline_config.raw,
        },
    )
    log_event(
        connection,
        event_type="campaign.created",
        campaign_id=campaign_id,
        actor="system",
        message="Created v0.1 fixture campaign.",
    )
    environment_fingerprint_id = create_environment_fingerprint(
        connection,
        tools={
            "cfd": {
                "environment": "aop-cfd",
                "checked_during_fixture": False,
                "reason": "fixture must not depend on external CFD tools",
            }
        },
        notes="v0.1 records-only fixture environment snapshot.",
    )
    optimizer_run_id = create_optimizer_run(
        connection,
        campaign_id=campaign_id,
        optimizer_name="deterministic_fixture_optimizer",
        optimizer_version="0.1.0",
        objective={
            "primary": "score.fixture_total",
            "direction": "maximize",
            "scope": "records_only_fixture",
        },
        settings={
            "proposal_count": 3,
            "real_optimizer_algorithm": False,
            "strategy": "seed_plus_deterministic_mutations",
        },
    )
    log_event(
        connection,
        event_type="optimizer.started",
        campaign_id=campaign_id,
        actor="optimizer",
        message="Started deterministic fixture optimizer run.",
        payload={"optimizer_run_id": optimizer_run_id},
    )

    candidate_seed = CandidateSeed(
        aircraft_family=aircraft_family,
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
        notes="Seed candidate for v0.1 records-only fixture.",
    )
    candidate_seed.validate()
    variable_schema.validate_candidate(candidate_seed)
    candidate_registration = register_candidate_with_lineage(
        campaign_id=campaign_id,
        connection=connection,
        variable_schema_id=variable_schema_id,
        candidate=candidate_seed,
        parent_candidate_ids=[],
        operator="seed",
        reason="v0.1 fixture seed candidate",
        mutation_summary={
            "initial_seed": True,
            "variable_schema_id": variable_schema_id,
        },
        registration_message="Registered fixture candidate.",
        lineage_message="Recorded fixture candidate lineage.",
    )
    candidate_id = candidate_registration.candidate_id
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        state="queued",
        stage="optimizer_proposed",
        priority=0,
        reason="seed candidate proposed",
        message="Candidate queued by deterministic fixture optimizer.",
        metadata={"source": "run_fixture", "role": "baseline"},
    )
    seed_iteration_id = create_optimizer_iteration(
        connection,
        optimizer_run_id=optimizer_run_id,
        campaign_id=campaign_id,
        iteration_index=0,
        parent_candidate_ids=[],
        proposed_candidate_ids=[candidate_id],
        strategy="seed",
        rationale="Create baseline candidate for v0.1 records-only campaign.",
        optimizer_state={
            "normalized_design_vector": candidate_seed.normalized_design_vector,
            "candidate_role": "baseline",
        },
    )
    complete_optimizer_iteration(
        connection,
        optimizer_iteration_id=seed_iteration_id,
        proposed_candidate_ids=[candidate_id],
        optimizer_state={"candidate_id": candidate_id, "status": "proposed"},
    )
    log_event(
        connection,
        event_type="optimizer.iteration.completed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        actor="optimizer",
        message="Recorded deterministic seed optimizer iteration.",
        payload={"optimizer_iteration_id": seed_iteration_id},
    )

    evaluation_id = start_evaluation_record(
        connection,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        pipeline_id=pipeline_config.pipeline_id,
        pipeline_version=pipeline_config.pipeline_version,
        requested_by="system",
        reason="v0.1 smoke fixture",
        environment_fingerprint_id=environment_fingerprint_id,
        message="Started fixture evaluation.",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=seed_iteration_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="geometry",
        active_module="fixture_geometry_provider",
        progress=0.15,
        reason="evaluation started",
        message="Candidate evaluation is generating fixture geometry.",
        metadata={"source": "run_fixture"},
    )

    geometry_record = persist_fixture_geometry(
        connection,
        platform_root=platform_root,
        workspace=workspace,
        artifact_root=artifact_root,
        campaign_id=campaign_id,
        variable_schema_id=variable_schema_id,
        variable_schema=variable_schema,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        candidate=candidate_seed,
        request_message="Requested manual reference geometry.",
        completed_message="Automatic fixture geometry generated.",
    )
    geometry_result_id = geometry_record.geometry_provider_result_id
    geometry_artifact_id = geometry_record.geometry_artifact_id
    trace_artifact_id = geometry_record.trace_artifact_id

    module_attempt_id = run_successful_module_attempt(
        connection,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_name="mock_geometry_metrics",
        module_version="0.1.0",
        module_kind="analysis",
        inputs={"geometry_provider_result_id": geometry_result_id},
        metrics=mock_geometry_metrics(),
        metadata={"records_only": True},
        started_message="Started mock module.",
        completed_message="Completed mock module.",
    )
    oml_fixture_path = (
        platform_root
        / "software"
        / "optimizer"
        / "examples"
        / "exporter_results"
        / "oml_stl_passed.json"
    )
    oml_module_attempt_id = persist_oml_stl_fixture_result(
        connection,
        artifact_root=artifact_root,
        fixture_path=oml_fixture_path,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        geometry_provider_result_id=geometry_result_id,
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=seed_iteration_id,
        evaluation_id=evaluation_id,
        state="running",
        stage="scoring",
        active_module="fixture_scoring",
        progress=0.75,
        reason="geometry/export complete",
        message="Candidate is ready for fixture scoring.",
        metadata={"source": "run_fixture"},
    )

    annotation_id = create_user_annotation(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        user_label="system",
        tag="fixture_baseline",
        comment="Fixture candidate for records-only platform validation.",
        metadata={"source": "run_fixture"},
    )
    log_event(
        connection,
        event_type="annotation.created",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        message="Recorded fixture annotation.",
        payload={"annotation_id": annotation_id},
    )

    child_candidate_seed = CandidateSeed(
        aircraft_family=aircraft_family,
        design_variables={
            "wing.span_mm": VariableValue(735.0, "mm"),
            "wing.root_chord_mm": VariableValue(166.0, "mm"),
            "wing.tip_chord_mm": VariableValue(70.0, "mm"),
            "wing.sweep_deg": VariableValue(18.0, "deg"),
        },
        normalized_design_vector={
            "wing.span_mm": 0.58,
            "wing.root_chord_mm": 0.46,
            "wing.tip_chord_mm": 0.56,
            "wing.sweep_deg": 0.58,
        },
        created_by="deterministic_mutation",
        generation=1,
        notes="Deterministic child candidate for lineage fixture.",
    )
    child_candidate_seed.validate()
    variable_schema.validate_candidate(child_candidate_seed)
    child_registration = register_candidate_with_lineage(
        campaign_id=campaign_id,
        connection=connection,
        variable_schema_id=variable_schema_id,
        candidate=child_candidate_seed,
        parent_candidate_ids=[candidate_id],
        operator="deterministic_mutation",
        reason="exercise parent-child lineage in v0.1 fixture",
        mutation_summary={
            "wing.span_mm": "+35.0 mm",
            "wing.root_chord_mm": "-4.0 mm",
            "wing.tip_chord_mm": "+5.0 mm",
            "wing.sweep_deg": "+3.0 deg",
        },
        registration_message="Registered deterministic child fixture candidate.",
        lineage_message="Recorded deterministic child fixture lineage.",
    )
    child_candidate_id = child_registration.candidate_id
    upsert_candidate_runner_state(
        connection,
        candidate_id=child_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        state="queued",
        stage="optimizer_proposed",
        priority=1,
        reason="mutation candidate proposed",
        message="Child candidate queued by deterministic fixture optimizer.",
        metadata={"source": "run_fixture", "role": "candidate_improvement"},
    )
    child_iteration_id = create_optimizer_iteration(
        connection,
        optimizer_run_id=optimizer_run_id,
        campaign_id=campaign_id,
        iteration_index=1,
        parent_candidate_ids=[candidate_id],
        proposed_candidate_ids=[child_candidate_id],
        strategy="deterministic_mutation",
        rationale="Explore higher aspect-ratio proxy direction in the fixture optimizer.",
        optimizer_state={
            "parent_candidate_id": candidate_id,
            "normalized_design_vector": child_candidate_seed.normalized_design_vector,
            "expected_role": "higher_fixture_score",
        },
    )
    complete_optimizer_iteration(
        connection,
        optimizer_iteration_id=child_iteration_id,
        proposed_candidate_ids=[child_candidate_id],
        optimizer_state={"candidate_id": child_candidate_id, "status": "proposed"},
    )
    log_event(
        connection,
        event_type="optimizer.iteration.completed",
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        actor="optimizer",
        message="Recorded deterministic child optimizer iteration.",
        payload={"optimizer_iteration_id": child_iteration_id},
    )

    child_evaluation_id = start_evaluation_record(
        connection,
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        pipeline_id=pipeline_config.pipeline_id,
        pipeline_version=pipeline_config.pipeline_version,
        requested_by="system",
        reason="v0.1 child lineage fixture",
        environment_fingerprint_id=environment_fingerprint_id,
        message="Started deterministic child fixture evaluation.",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=child_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=child_iteration_id,
        evaluation_id=child_evaluation_id,
        state="running",
        stage="geometry",
        active_module="fixture_geometry_provider",
        progress=0.15,
        reason="evaluation started",
        message="Child candidate evaluation is generating fixture geometry.",
        metadata={"source": "run_fixture"},
    )
    child_geometry_record = persist_fixture_geometry(
        connection,
        platform_root=platform_root,
        workspace=workspace,
        artifact_root=artifact_root,
        campaign_id=campaign_id,
        variable_schema_id=variable_schema_id,
        variable_schema=variable_schema,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        candidate=child_candidate_seed,
        request_message="Requested automatic fixture geometry for child candidate.",
        completed_message="Automatic fixture geometry generated for child candidate.",
    )
    child_geometry_result_id = child_geometry_record.geometry_provider_result_id
    child_geometry_artifact_id = child_geometry_record.geometry_artifact_id
    child_trace_artifact_id = child_geometry_record.trace_artifact_id

    child_module_attempt_id = run_successful_module_attempt(
        connection,
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        module_name="mock_geometry_metrics",
        module_version="0.1.0",
        module_kind="analysis",
        inputs={"geometry_provider_result_id": child_geometry_result_id},
        metrics=mock_geometry_metrics(),
        metadata={"records_only": True},
        started_message="Started child mock module.",
        completed_message="Completed child mock module.",
    )
    child_oml_module_attempt_id = persist_oml_stl_fixture_result(
        connection,
        artifact_root=artifact_root,
        fixture_path=oml_fixture_path,
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        geometry_provider_result_id=child_geometry_result_id,
    )
    child_su2_case_attempt_id = persist_su2_dry_run_case(
        connection,
        artifact_root=artifact_root,
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        geometry_provider_result_id=child_geometry_result_id,
        export_module_attempt_id=child_oml_module_attempt_id,
        mesh_reference_path="outputs/direct_sdf_oml_A.stl",
    )
    su2_history_fixture_path = (
        platform_root
        / "software"
        / "optimizer"
        / "examples"
        / "cfd_results"
        / "su2_history_fixture.csv"
    )
    child_su2_module_attempt_id = persist_su2_history_fixture_result(
        connection,
        artifact_root=artifact_root,
        history_fixture_path=su2_history_fixture_path,
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        geometry_provider_result_id=child_geometry_result_id,
        export_module_attempt_id=child_oml_module_attempt_id,
        case_builder_module_attempt_id=child_su2_case_attempt_id,
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=child_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=child_iteration_id,
        evaluation_id=child_evaluation_id,
        state="cfd_running",
        stage="cfd",
        active_module="su2_cfd_fixture",
        progress=0.65,
        reason="dry-run CFD fixture started",
        message="Child candidate is recording fixture CFD evidence.",
        metadata={"source": "run_fixture", "solver": "su2_fixture"},
    )

    scoring_attempt_id = run_successful_module_attempt(
        connection,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_name="fixture_scoring",
        module_version="0.1.0",
        module_kind="scoring",
        inputs={
            "geometry_module_attempt_id": module_attempt_id,
            "export_module_attempt_id": oml_module_attempt_id,
        },
        metrics=fixture_scoring_metrics(candidate_seed),
        metadata={"placeholder_scoring": True},
        started_message="Started fixture scoring module.",
        completed_message="Completed fixture scoring module.",
    )

    child_scoring_attempt_id = run_successful_module_attempt(
        connection,
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        module_name="fixture_scoring",
        module_version="0.1.0",
        module_kind="scoring",
        inputs={
            "geometry_module_attempt_id": child_module_attempt_id,
            "export_module_attempt_id": child_oml_module_attempt_id,
        },
        metrics=fixture_scoring_metrics(child_candidate_seed),
        metadata={"placeholder_scoring": True},
        started_message="Started child fixture scoring module.",
        completed_message="Completed child fixture scoring module.",
    )

    complete_evaluation(
        connection,
        evaluation_id=child_evaluation_id,
        summary={
            "status": "complete",
            "geometry_provider_result_id": child_geometry_result_id,
            "module_attempt_ids": [
                child_module_attempt_id,
                child_oml_module_attempt_id,
                child_su2_case_attempt_id,
                child_su2_module_attempt_id,
                child_scoring_attempt_id,
            ],
            "artifact_ids": [child_geometry_artifact_id, child_trace_artifact_id],
        },
    )
    log_event(
        connection,
        event_type="evaluation.completed",
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        actor="system",
        message="Completed deterministic child fixture evaluation.",
    )
    update_candidate_status(
        connection,
        candidate_id=child_candidate_id,
        status="evaluated",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=child_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=child_iteration_id,
        evaluation_id=child_evaluation_id,
        state="promoted",
        stage="optimizer_decision",
        active_module=None,
        progress=1.0,
        reason="highest fixture score",
        message="Candidate promoted by deterministic fixture optimizer.",
        metadata={"source": "run_fixture", "decision": "promote"},
    )
    child_annotation_id = create_user_annotation(
        connection,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        user_label="system",
        tag="fixture_child",
        comment="Deterministic child candidate for lineage validation.",
        metadata={"source": "run_fixture", "parent_candidate_id": candidate_id},
    )
    log_event(
        connection,
        event_type="annotation.created",
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        evaluation_id=child_evaluation_id,
        actor="system",
        message="Recorded child fixture annotation.",
        payload={"annotation_id": child_annotation_id},
    )
    complete_evaluation(
        connection,
        evaluation_id=evaluation_id,
        summary={
            "status": "complete",
            "geometry_provider_result_id": geometry_result_id,
            "module_attempt_ids": [
                module_attempt_id,
                oml_module_attempt_id,
                scoring_attempt_id,
            ],
            "artifact_ids": [geometry_artifact_id, trace_artifact_id],
        },
    )
    log_event(
        connection,
        event_type="evaluation.completed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="system",
        message="Completed fixture evaluation.",
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
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=seed_iteration_id,
        evaluation_id=evaluation_id,
        state="rejected",
        stage="optimizer_decision",
        active_module=None,
        progress=1.0,
        reason="superseded by promoted child",
        message="Baseline candidate evaluated but not promoted.",
        metadata={"source": "run_fixture", "decision": "reject_after_scoring"},
    )

    failed_candidate_seed = CandidateSeed(
        aircraft_family=aircraft_family,
        design_variables={
            "wing.span_mm": VariableValue(760.0, "mm"),
            "wing.root_chord_mm": VariableValue(180.0, "mm"),
            "wing.tip_chord_mm": VariableValue(58.0, "mm"),
            "wing.sweep_deg": VariableValue(25.0, "deg"),
        },
        normalized_design_vector={
            "wing.span_mm": 0.63,
            "wing.root_chord_mm": 0.55,
            "wing.tip_chord_mm": 0.22,
            "wing.sweep_deg": 0.75,
        },
        created_by="deterministic_failure_fixture",
        generation=1,
        notes="Deterministic candidate that fails the OML export quality gate fixture.",
    )
    failed_candidate_seed.validate()
    variable_schema.validate_candidate(failed_candidate_seed)
    failed_registration = register_candidate_with_lineage(
        campaign_id=campaign_id,
        connection=connection,
        variable_schema_id=variable_schema_id,
        candidate=failed_candidate_seed,
        parent_candidate_ids=[candidate_id],
        operator="deterministic_failure_fixture",
        reason="exercise campaign-level failure browsing in v0.1 fixture",
        mutation_summary={
            "wing.span_mm": "+60.0 mm",
            "wing.root_chord_mm": "+10.0 mm",
            "wing.tip_chord_mm": "-7.0 mm",
            "wing.sweep_deg": "+10.0 deg",
            "expected_failure": "OML fixture quality gate",
        },
        registration_message="Registered deterministic failed fixture candidate.",
        lineage_message="Recorded deterministic failed fixture lineage.",
    )
    failed_candidate_id = failed_registration.candidate_id
    upsert_candidate_runner_state(
        connection,
        candidate_id=failed_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        state="queued",
        stage="optimizer_proposed",
        priority=2,
        reason="failure fixture candidate proposed",
        message="Failure fixture candidate queued by deterministic optimizer.",
        metadata={"source": "run_fixture", "role": "failure_fixture"},
    )
    failed_iteration_id = create_optimizer_iteration(
        connection,
        optimizer_run_id=optimizer_run_id,
        campaign_id=campaign_id,
        iteration_index=2,
        parent_candidate_ids=[candidate_id],
        proposed_candidate_ids=[failed_candidate_id],
        strategy="deterministic_failure_fixture",
        rationale="Exercise failure preservation and campaign failure browsing.",
        optimizer_state={
            "parent_candidate_id": candidate_id,
            "normalized_design_vector": failed_candidate_seed.normalized_design_vector,
            "expected_failure": "OML fixture quality gate",
        },
    )
    complete_optimizer_iteration(
        connection,
        optimizer_iteration_id=failed_iteration_id,
        proposed_candidate_ids=[failed_candidate_id],
        optimizer_state={"candidate_id": failed_candidate_id, "status": "proposed"},
    )
    log_event(
        connection,
        event_type="optimizer.iteration.completed",
        campaign_id=campaign_id,
        candidate_id=failed_candidate_id,
        actor="optimizer",
        message="Recorded deterministic failed optimizer iteration.",
        payload={"optimizer_iteration_id": failed_iteration_id},
    )

    failed_evaluation_id = start_evaluation_record(
        connection,
        campaign_id=campaign_id,
        candidate_id=failed_candidate_id,
        pipeline_id=pipeline_config.pipeline_id,
        pipeline_version=pipeline_config.pipeline_version,
        requested_by="system",
        reason="v0.1 failed export fixture",
        environment_fingerprint_id=environment_fingerprint_id,
        message="Started deterministic failed fixture evaluation.",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=failed_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=failed_iteration_id,
        evaluation_id=failed_evaluation_id,
        state="running",
        stage="export",
        active_module="oml_stl_export",
        progress=0.45,
        reason="expected export quality gate failure fixture",
        message="Candidate is running export-quality gate fixture.",
        metadata={"source": "run_fixture"},
    )
    failed_geometry_record = persist_fixture_geometry(
        connection,
        platform_root=platform_root,
        workspace=workspace,
        artifact_root=artifact_root,
        campaign_id=campaign_id,
        variable_schema_id=variable_schema_id,
        variable_schema=variable_schema,
        candidate_id=failed_candidate_id,
        evaluation_id=failed_evaluation_id,
        candidate=failed_candidate_seed,
        request_message="Requested automatic fixture geometry for failed candidate.",
        completed_message="Automatic fixture geometry generated for failed candidate.",
    )
    failed_geometry_result_id = failed_geometry_record.geometry_provider_result_id
    failed_geometry_artifact_id = failed_geometry_record.geometry_artifact_id
    failed_trace_artifact_id = failed_geometry_record.trace_artifact_id

    failed_module_attempt_id = run_successful_module_attempt(
        connection,
        campaign_id=campaign_id,
        candidate_id=failed_candidate_id,
        evaluation_id=failed_evaluation_id,
        module_name="mock_geometry_metrics",
        module_version="0.1.0",
        module_kind="analysis",
        inputs={"geometry_provider_result_id": failed_geometry_result_id},
        metrics=mock_geometry_metrics(),
        metadata={"records_only": True},
        started_message="Started failed-candidate mock module.",
        completed_message="Completed failed-candidate mock module.",
    )
    failed_oml_fixture_path = (
        platform_root
        / "software"
        / "optimizer"
        / "examples"
        / "exporter_results"
        / "oml_stl_failed_quality_gate.json"
    )
    failed_oml_module_attempt_id = persist_oml_stl_fixture_result(
        connection,
        artifact_root=artifact_root,
        fixture_path=failed_oml_fixture_path,
        campaign_id=campaign_id,
        candidate_id=failed_candidate_id,
        evaluation_id=failed_evaluation_id,
        geometry_provider_result_id=failed_geometry_result_id,
    )
    failed_export_failure_id = connection.execute(
        """
        SELECT failure_id
        FROM module_attempts
        WHERE module_attempt_id = ?
        """,
        (failed_oml_module_attempt_id,),
    ).fetchone()[0]
    fail_evaluation(
        connection,
        evaluation_id=failed_evaluation_id,
        failure_id=failed_export_failure_id,
        summary={
            "status": "failed",
            "geometry_provider_result_id": failed_geometry_result_id,
            "module_attempt_ids": [failed_module_attempt_id, failed_oml_module_attempt_id],
            "artifact_ids": [failed_geometry_artifact_id, failed_trace_artifact_id],
            "failure_id": failed_export_failure_id,
        },
    )
    log_event(
        connection,
        event_type="evaluation.failed",
        campaign_id=campaign_id,
        candidate_id=failed_candidate_id,
        evaluation_id=failed_evaluation_id,
        actor="system",
        severity="error",
        message="Failed deterministic export-quality fixture evaluation.",
        payload={"failure_id": failed_export_failure_id},
    )
    update_candidate_status(
        connection,
        candidate_id=failed_candidate_id,
        status="failed",
    )
    upsert_candidate_runner_state(
        connection,
        candidate_id=failed_candidate_id,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
        optimizer_iteration_id=failed_iteration_id,
        evaluation_id=failed_evaluation_id,
        state="failed",
        stage="export",
        active_module="oml_stl_export",
        progress=1.0,
        reason="export quality gate failed",
        message="Candidate failed the OML export quality gate.",
        metadata={"source": "run_fixture", "failure_id": failed_export_failure_id},
    )
    failed_annotation_id = create_user_annotation(
        connection,
        candidate_id=failed_candidate_id,
        evaluation_id=failed_evaluation_id,
        user_label="system",
        tag="fixture_failed_export",
        comment="Deterministic failure fixture for campaign failure browsing.",
        metadata={"source": "run_fixture", "parent_candidate_id": candidate_id},
    )
    log_event(
        connection,
        event_type="annotation.created",
        campaign_id=campaign_id,
        candidate_id=failed_candidate_id,
        evaluation_id=failed_evaluation_id,
        actor="system",
        severity="warning",
        message="Recorded failed export fixture annotation.",
        payload={"annotation_id": failed_annotation_id},
    )
    complete_optimizer_run(
        connection,
        optimizer_run_id=optimizer_run_id,
        summary={
            "status": "complete",
            "candidate_ids": [candidate_id, child_candidate_id, failed_candidate_id],
            "evaluated_candidate_ids": [candidate_id, child_candidate_id],
            "failed_candidate_ids": [failed_candidate_id],
            "best_candidate_id": child_candidate_id,
            "selection_basis": "fixture_scoring_metrics",
        },
    )
    log_event(
        connection,
        event_type="optimizer.completed",
        campaign_id=campaign_id,
        candidate_id=child_candidate_id,
        actor="optimizer",
        message="Completed deterministic fixture optimizer run.",
        payload={
            "optimizer_run_id": optimizer_run_id,
            "best_candidate_id": child_candidate_id,
        },
    )
    connection.commit()
    connection.close()

    print(f"workspace={workspace}")
    print(f"database={db_path}")
    print(f"campaign_id={campaign_id}")
    print(f"candidate_id={candidate_id}")
    print(f"child_candidate_id={child_candidate_id}")
    print(f"failed_candidate_id={failed_candidate_id}")
    print(f"evaluation_id={evaluation_id}")
    print(f"child_evaluation_id={child_evaluation_id}")
    print(f"failed_evaluation_id={failed_evaluation_id}")


def summarize_workspace(workspace: Path) -> None:
    db_path = workspace / "optimizer.db"
    summary = summarize_database(db_path)
    print(json.dumps(summary, indent=2, sort_keys=True))


def report_adaptive_cfd_promotion(
    *,
    preset_path: Path | None,
    baseline_summaries: list[Path],
    candidate_alpha_summary: Path | None,
    variant_id: str | None,
) -> None:
    platform_root = find_platform_root()
    project_root = platform_root.parent
    preset_path = preset_path or (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "snappy_openfoam_external_aero_ranking.v0_1.json"
    )
    preset = json.loads(preset_path.read_text(encoding="utf-8-sig"))
    baseline_sweeps = _load_alpha_baseline_sweeps(
        project_root=project_root,
        summary_paths=baseline_summaries,
    )
    policy = _adaptive_cfd_promotion_policy_from_preset(preset)
    candidate_reports = _load_candidate_alpha_reports(
        project_root=project_root,
        summary_path=candidate_alpha_summary,
        variant_id=variant_id,
    )
    if not candidate_reports:
        report = adaptive_cfd_promotion_report(
            expanded_baseline_sweeps=baseline_sweeps,
            policy=policy,
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return

    reports = []
    for candidate_report in candidate_reports:
        report = adaptive_cfd_promotion_report(
            expanded_baseline_sweeps=baseline_sweeps,
            candidate_first_pass=candidate_report.get("aggregate", {}),
            policy=policy,
        )
        report["variant_id"] = candidate_report.get("variant_id")
        reports.append(report)
    print(
        json.dumps(
            {
                "schema": "adaptive_cfd_promotion_report.v0_1",
                "candidate_report_count": len(reports),
                "reports": reports,
            },
            indent=2,
            sort_keys=True,
        )
    )


def run_wing_options(workspace: Path) -> None:
    result = run_wing_options_study(workspace, platform_root=find_platform_root())
    print(json.dumps(result, indent=2, sort_keys=True))


def run_sequential_gated(
    workspace: Path,
    iterations: int,
    virtual_components_config: Path | None,
) -> None:
    result = run_sequential_gated_study(
        workspace,
        platform_root=find_platform_root(),
        iterations=iterations,
        virtual_components_config=virtual_components_config,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


def run_real_no_inlet_export(workspace: Path, timeout_seconds: int) -> None:
    result = run_real_no_inlet_export_once(
        workspace,
        platform_root=find_platform_root(),
        timeout_seconds=timeout_seconds,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


def run_real_no_inlet_export_batch_command(
    workspace: Path,
    iterations: int,
    timeout_seconds: int,
) -> None:
    result = run_real_no_inlet_export_batch(
        workspace,
        platform_root=find_platform_root(),
        iterations=iterations,
        timeout_seconds=timeout_seconds,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


def run_real_optimizer_pilot_command(
    workspace: Path,
    run_slug: str,
    initial_count: int,
    children_count: int,
    timeout_seconds: int,
    alpha_strategy: str,
    resume: bool,
) -> None:
    platform_root = find_platform_root()
    optimizer_root = platform_root / "software" / "optimizer"
    script_path = optimizer_root / "scripts" / "run_real_optimizer_pilot.py"
    env = dict(os.environ)
    env["PYTHONPATH"] = "src"
    command = [
        sys.executable,
        str(script_path),
        "--workspace",
        str(workspace),
        "--run-slug",
        run_slug,
        "--initial-count",
        str(initial_count),
        "--children-count",
        str(children_count),
        "--timeout-seconds",
        str(timeout_seconds),
        "--alpha-strategy",
        alpha_strategy,
    ]
    if resume:
        command.append("--resume")
    subprocess.run(command, cwd=optimizer_root, env=env, check=True)


def report_evaluation(workspace: Path, evaluation_id: str | None) -> None:
    db_path = workspace / "optimizer.db"
    report = evaluation_report(db_path, evaluation_id=evaluation_id)
    print(json.dumps(report, indent=2, sort_keys=True))


def report_campaign(workspace: Path, campaign_id: str | None, failed_only: bool) -> None:
    db_path = workspace / "optimizer.db"
    report = campaign_report(db_path, campaign_id=campaign_id, failed_only=failed_only)
    print(json.dumps(report, indent=2, sort_keys=True))


def report_comparison(
    workspace: Path,
    campaign_id: str | None,
    candidate_id: str | None,
    other_candidate_id: str | None,
) -> None:
    db_path = workspace / "optimizer.db"
    report = candidate_comparison_report(
        db_path,
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        other_candidate_id=other_candidate_id,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def report_optimizer_run(
    workspace: Path,
    campaign_id: str | None,
    optimizer_run_id: str | None,
) -> None:
    db_path = workspace / "optimizer.db"
    report = optimizer_report(
        db_path,
        campaign_id=campaign_id,
        optimizer_run_id=optimizer_run_id,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def report_layout(
    workspace: Path,
    candidate_id: str | None,
    evaluation_id: str | None,
) -> None:
    db_path = workspace / "optimizer.db"
    report = layout_report(
        db_path,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def check_cfd_tools() -> None:
    status = check_wsl_cfd_tools()
    print(json.dumps(status.to_dict(), indent=2, sort_keys=True))


def check_cfd_readiness(export_result: Path) -> None:
    parsed = parse_oml_stl_export_result(export_result)
    readiness = evaluate_cfd_readiness(parsed.metrics)
    print(
        json.dumps(
            {
                "export_result": str(export_result),
                "export_passed": parsed.passed,
                "cfd_ready": readiness.passed,
                "metrics": readiness.metrics,
                "metadata": readiness.metadata,
                "warnings": readiness.warnings,
            },
            indent=2,
            sort_keys=True,
        )
    )


def check_mesh_result(
    mesh: Path,
    mesh_format: str,
    su2_log: Path | None,
    aircraft_markers: list[str] | None,
    farfield_markers: list[str] | None,
    fluid_markers: list[str] | None,
) -> None:
    marker_map = None
    if aircraft_markers or farfield_markers or fluid_markers:
        marker_map = {
            "aircraft": aircraft_markers or [],
            "farfield": farfield_markers or [],
            "fluid": fluid_markers or [],
        }
    result = validate_mesh_result(
        mesh_path=mesh,
        mesh_format=mesh_format,
        su2_log_path=su2_log,
        semantic_to_solver_markers=marker_map,
    )
    print(
        json.dumps(
            {
                "passed": result.passed,
                "mesh_result": result.mesh_result,
                "metrics": result.metrics,
                "metadata": result.metadata,
                "warnings": result.warnings,
            },
            indent=2,
            sort_keys=True,
        )
    )


def check_cfd_surface(stl: Path) -> None:
    result = evaluate_cfd_surface_quality(stl)
    print(
        json.dumps(
            {
                "passed": result.passed,
                "surface_result": result.surface_result,
                "metrics": result.metrics,
                "metadata": result.metadata,
                "warnings": result.warnings,
            },
            indent=2,
            sort_keys=True,
        )
    )


def check_openfoam_result(
    case_dir: Path,
    check_mesh_log: Path,
    solver_log: Path,
    acceptance_mode: str,
) -> None:
    result = validate_openfoam_result(
        case_dir=case_dir,
        check_mesh_log=check_mesh_log,
        solver_log=solver_log,
        policy=policy_for_acceptance_mode(acceptance_mode),
    )
    print(
        json.dumps(
            {
                "passed": result.passed,
                "openfoam_result": result.openfoam_result,
                "metrics": result.metrics,
                "metadata": result.metadata,
                "warnings": result.warnings,
            },
            indent=2,
            sort_keys=True,
        )
    )


def persist_openfoam_smoke(
    workspace: Path,
    case_dir: Path,
    check_mesh_log: Path,
    solver_log: Path,
    acceptance_mode: str,
    strict_check_mesh_log: Path | None,
    relaxed_check_mesh_log: Path | None,
    aircraft_iso: Path | None,
    campaign_id: str | None,
    candidate_id: str | None,
    evaluation_id: str | None,
) -> None:
    db_path = workspace / "optimizer.db"
    connection = connect(db_path)
    try:
        selected = _select_evaluation_context(
            connection,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
        )
        artifact_root = workspace / "campaigns" / "openfoam_smoke" / "artifacts"
        module_attempt_id = persist_openfoam_smoke_result(
            connection,
            artifact_root=artifact_root,
            campaign_id=selected["campaign_id"],
            candidate_id=selected["candidate_id"],
            evaluation_id=selected["evaluation_id"],
            case_dir=case_dir,
            check_mesh_log=check_mesh_log,
            solver_log=solver_log,
            acceptance_mode=acceptance_mode,
            strict_check_mesh_log=strict_check_mesh_log,
            relaxed_check_mesh_log=relaxed_check_mesh_log,
            aircraft_iso_path=aircraft_iso,
            requested_by="cli",
        )
        connection.commit()
        print(
            json.dumps(
                {
                    "module_attempt_id": module_attempt_id,
                    "acceptance_mode": acceptance_mode,
                    "scoring_allowed": False,
                    **selected,
                },
                indent=2,
                sort_keys=True,
            )
        )
    finally:
        connection.close()


def check_openfoam_steady_result(
    case_dir: Path,
    solver_log: Path,
    check_mesh_log: Path | None,
    force_coeffs: Path | None,
    yplus: Path | None,
    acceptance_mode: str,
) -> None:
    policy = steady_policy_for_acceptance_mode(acceptance_mode)
    result = validate_openfoam_steady_result(
        case_dir=case_dir,
        solver_log=solver_log,
        check_mesh_log=check_mesh_log,
        force_coeffs_path=force_coeffs,
        yplus_path=yplus,
        policy=policy,
    )
    print(
        json.dumps(
            {
                "passed": result.passed,
                "openfoam_steady_result": result.openfoam_steady_result,
                "metrics": result.metrics,
                "metadata": result.metadata,
                "warnings": result.warnings,
            },
            indent=2,
            sort_keys=True,
        )
    )


def persist_openfoam_steady(
    workspace: Path,
    case_dir: Path,
    solver_log: Path,
    check_mesh_log: Path | None,
    force_coeffs: Path | None,
    yplus: Path | None,
    aircraft_iso: Path | None,
    acceptance_mode: str,
    campaign_id: str | None,
    candidate_id: str | None,
    evaluation_id: str | None,
) -> None:
    policy = steady_policy_for_acceptance_mode(acceptance_mode)
    db_path = workspace / "optimizer.db"
    connection = connect(db_path)
    try:
        selected = _select_evaluation_context(
            connection,
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
        )
        artifact_root = workspace / "campaigns" / "openfoam_steady" / "artifacts"
        module_attempt_id = persist_openfoam_steady_result(
            connection,
            artifact_root=artifact_root,
            campaign_id=selected["campaign_id"],
            candidate_id=selected["candidate_id"],
            evaluation_id=selected["evaluation_id"],
            case_dir=case_dir,
            solver_log=solver_log,
            check_mesh_log=check_mesh_log,
            force_coeffs_path=force_coeffs or find_force_coeffs(case_dir),
            yplus_path=yplus,
            aircraft_iso_path=aircraft_iso,
            policy=policy,
            requested_by="cli",
        )
        connection.commit()
        print(
            json.dumps(
                {
                    "module_attempt_id": module_attempt_id,
                    "analysis_role": (
                        "rough_scoring_optimizer_ranking"
                        if policy.get("scoring_tier") == "rough_scoring"
                        else "steady_solver_development_diagnostic"
                    ),
                    "scoring_tier": policy.get("scoring_tier"),
                    "score_usage": policy.get("score_usage"),
                    "scoring_allowed": False,
                    **selected,
                },
                indent=2,
                sort_keys=True,
            )
        )
    finally:
        connection.close()


def ingest_gmsh_bl_run(
    workspace: Path,
    summary_path: Path,
    variant_config_path: Path | None,
) -> None:
    platform_root = find_platform_root()
    project_root = platform_root.parent
    summary_path = _resolve_project_path(project_root, summary_path)
    variant_config_path = variant_config_path or (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "cfd_faired_cap_vehicle_variants.v0_1.json"
    )
    preset_path = (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "gmsh_openfoam_external_aero_bl.v0_2.json"
    )
    summary = json.loads(summary_path.read_text(encoding="utf-8-sig"))
    variant_config = json.loads(variant_config_path.read_text(encoding="utf-8-sig"))
    variants_by_id = {
        item["id"]: item
        for item in variant_config.get("variants", [])
    }

    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "gmsh_bl_openfoam_v0_2" / "artifacts"
    connection = connect(db_path)
    initialize_database(connection)
    try:
        aircraft_family = "fixed_wing_uav_faired_cap"
        schema_payload = {
            "schema": "gmsh_bl_faired_cap_variants.v0_1",
            "source_config": str(variant_config_path),
            "units": variant_config.get("units", "mm"),
            "variants": variant_config.get("variants", []),
        }
        variable_schema_id = create_variable_schema(
            connection,
            aircraft_family=aircraft_family,
            schema_version="gmsh_bl_faired_cap_variants.v0_1",
            schema=schema_payload,
        )
        campaign_id = create_campaign(
            connection,
            name="Gmsh BL OpenFOAM v0.2 evidence import",
            description=(
                "Imported five approved faired-cap Gmsh create-topology prism-layer "
                "OpenFOAM smoke cases. Non-scoring mesher/plumbing evidence only."
            ),
            aircraft_family=aircraft_family,
            variable_schema_id=variable_schema_id,
            config={
                "pipeline_id": "gmsh_openfoam_external_aero_bl",
                "pipeline_version": "0.2",
                "summary_path": str(summary_path),
                "variant_config_path": str(variant_config_path),
                "preset_path": str(preset_path),
                "scoring_allowed": False,
            },
        )
        log_event(
            connection,
            event_type="campaign.created",
            campaign_id=campaign_id,
            actor="system",
            message="Created Gmsh BL OpenFOAM evidence-import campaign.",
        )
        environment_fingerprint_id = create_environment_fingerprint(
            connection,
            tools={
                "gmsh_bl_selector": summary.get("strategy"),
                "openfoam": "OpenFOAM 13 via WSL",
                "solver_smoke": "potentialFoam -writep",
                "preset": str(preset_path),
            },
            notes="Imported existing Gmsh BL/OpenFOAM artifacts; did not rerun meshing or solver.",
        )

        imported = []
        for report in summary.get("reports", []):
            variant_id = report["variant_id"]
            variant = variants_by_id.get(variant_id, {})
            overrides = variant.get("overrides", {})
            candidate = CandidateSeed(
                aircraft_family=aircraft_family,
                design_variables={
                    key: VariableValue(value, _variant_unit(key))
                    for key, value in overrides.items()
                },
                normalized_design_vector={key: 0.5 for key in overrides},
                created_by="gmsh_bl_evidence_import",
                generation=0,
                notes=f"Imported Gmsh BL OpenFOAM evidence for {variant_id}.",
            )
            candidate.validate()
            registration = register_candidate_with_lineage(
                connection,
                campaign_id=campaign_id,
                variable_schema_id=variable_schema_id,
                candidate=candidate,
                parent_candidate_ids=[],
                operator="gmsh_bl_evidence_import",
                operator_version="0.2",
                reason="Import existing five-variant Gmsh BL/OpenFOAM smoke evidence.",
                mutation_summary={
                    "variant_id": variant_id,
                    "description": variant.get("description"),
                    "output_stem": variant.get("output_stem"),
                    "selector_decision": report.get("selector_decision"),
                    "raw_stl": report.get("raw_stl"),
                    "selected_surface": report.get("selected_surface"),
                },
                registration_message=f"Registered imported Gmsh BL candidate {variant_id}.",
                lineage_message=f"Recorded imported Gmsh BL lineage for {variant_id}.",
            )
            evaluation_id = start_evaluation_record(
                connection,
                campaign_id=campaign_id,
                candidate_id=registration.candidate_id,
                pipeline_id="gmsh_openfoam_external_aero_bl",
                pipeline_version="0.2",
                requested_by="cli",
                reason="Import existing Gmsh BL OpenFOAM smoke evidence.",
                environment_fingerprint_id=environment_fingerprint_id,
                message=f"Started imported Gmsh BL evaluation for {variant_id}.",
            )
            run_dir = _resolve_project_path(project_root, Path(report["run_dir"]))
            case_dir = run_dir / "gmsh_bl" / "openfoam_case"
            iso_path = run_dir / "aircraft_only_screenshots" / "iso.png"
            module_attempt_id = persist_openfoam_smoke_result(
                connection,
                artifact_root=artifact_root,
                campaign_id=campaign_id,
                candidate_id=registration.candidate_id,
                evaluation_id=evaluation_id,
                case_dir=case_dir,
                check_mesh_log=case_dir / "log.checkMesh",
                solver_log=case_dir / "log.potentialFoam",
                acceptance_mode="gmsh_bl_development",
                strict_check_mesh_log=case_dir / "log.checkMesh",
                aircraft_iso_path=iso_path if iso_path.exists() else None,
                requested_by="cli",
            )
            update_candidate_status(
                connection,
                candidate_id=registration.candidate_id,
                status="evaluated",
            )
            complete_evaluation(
                connection,
                evaluation_id=evaluation_id,
                summary={
                    "variant_id": variant_id,
                    "module_attempt_id": module_attempt_id,
                    "acceptance_mode": "gmsh_bl_development",
                    "scoring_allowed": False,
                    "openfoam": report.get("openfoam"),
                    "potential": report.get("potential"),
                    "selector_decision": report.get("selector_decision"),
                },
            )
            imported.append(
                {
                    "variant_id": variant_id,
                    "candidate_id": registration.candidate_id,
                    "evaluation_id": evaluation_id,
                    "module_attempt_id": module_attempt_id,
                }
            )

        complete_campaign(connection, campaign_id=campaign_id)
        connection.commit()
        print(
            json.dumps(
                {
                    "workspace": str(workspace),
                    "campaign_id": campaign_id,
                    "imported_count": len(imported),
                    "scoring_allowed": False,
                    "imported": imported,
                },
                indent=2,
                sort_keys=True,
            )
        )
    finally:
        connection.close()


def steady_policy_for_acceptance_mode(acceptance_mode: str) -> dict:
    try:
        return OPENFOAM_STEADY_ACCEPTANCE_POLICIES[acceptance_mode]
    except KeyError as exc:
        known = ", ".join(sorted(OPENFOAM_STEADY_ACCEPTANCE_POLICIES))
        raise SystemExit(
            f"Unknown OpenFOAM steady acceptance mode '{acceptance_mode}'. "
            f"Known modes: {known}"
        ) from exc


def _variant_unit(name: str) -> str:
    if name.endswith("_deg"):
        return "deg"
    return "mm"


def ingest_snappy_hifi_run(
    workspace: Path,
    run_root: Path,
    variant_config_path: Path | None,
    preset_path: Path | None,
    include_steady: bool,
    rough_scoring_config_path: Path | None = None,
    alpha_sweep_summary_name_override: str | None = None,
) -> None:
    platform_root = find_platform_root()
    project_root = platform_root.parent
    run_root = _resolve_project_path(project_root, run_root)
    summary_path = run_root / "comparison_summary.json"
    ladder_summary_path = select_ladder_summary_path(run_root)
    variant_config_path = variant_config_path or (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "cfd_faired_cap_vehicle_variants.v0_1.json"
    )
    preset_path = preset_path or (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "snappy_openfoam_external_aero_hifi_surface.v0_4.json"
    )
    preset = json.loads(preset_path.read_text(encoding="utf-8-sig"))
    optimizer_import = preset.get("optimizer_import", {})
    pipeline_id = optimizer_import.get(
        "pipeline_id", "snappy_openfoam_external_aero_hifi_surface"
    )
    pipeline_version = str(optimizer_import.get("pipeline_version", "0.4"))
    pipeline_version_slug = pipeline_version.replace(".", "_")
    surface_fidelity_report_name = optimizer_import.get(
        "surface_fidelity_report_name",
        "surface_fidelity_vtk_distance_1layer_candidate.json",
    )
    alpha_sweep_summary_name = (
        alpha_sweep_summary_name_override
        or optimizer_import.get(
            "alpha_sweep_summary_name",
            "alpha_sweep_summary_end60_np8_v2.json",
        )
    )
    acceptance_mode = optimizer_import.get(
        "acceptance_mode", "snappy_hifi_development"
    )
    steady_acceptance_mode = optimizer_import.get(
        "steady_acceptance_mode",
        acceptance_mode,
    )
    steady_policy = steady_policy_for_acceptance_mode(steady_acceptance_mode)
    default_rough_scoring_config_path = optimizer_import.get("rough_scoring_config")
    rough_scoring_config_path = rough_scoring_config_path or (
        _resolve_project_path(project_root, Path(default_rough_scoring_config_path))
        if default_rough_scoring_config_path
        else None
    )
    rough_scoring_config = (
        json.loads(rough_scoring_config_path.read_text(encoding="utf-8-sig"))
        if rough_scoring_config_path is not None
        else None
    )
    summary = json.loads(summary_path.read_text(encoding="utf-8-sig"))
    ladder_summary = (
        json.loads(ladder_summary_path.read_text(encoding="utf-8-sig"))
        if ladder_summary_path is not None
        else {"reports": []}
    )
    alpha_sweep_summary_path = run_root / alpha_sweep_summary_name
    alpha_sweep_summary = (
        json.loads(alpha_sweep_summary_path.read_text(encoding="utf-8-sig"))
        if alpha_sweep_summary_path.exists()
        else {"reports": []}
    )
    steady_by_variant = {
        report["variant_id"]: report for report in ladder_summary.get("reports", [])
    }
    alpha_sweep_by_variant = {
        report["variant_id"]: report for report in alpha_sweep_summary.get("reports", [])
    }
    variant_config = json.loads(variant_config_path.read_text(encoding="utf-8-sig"))
    variants_by_id = {
        item["id"]: item
        for item in variant_config.get("variants", [])
    }

    db_path = workspace / "optimizer.db"
    artifact_root = (
        workspace
        / "campaigns"
        / f"snappy_hifi_openfoam_v{pipeline_version_slug}"
        / "artifacts"
    )
    connection = connect(db_path)
    initialize_database(connection)
    try:
        aircraft_family = "fixed_wing_uav_faired_cap"
        schema_payload = {
            "schema": f"snappy_hifi_faired_cap_variants.v{pipeline_version}",
            "source_config": str(variant_config_path),
            "units": variant_config.get("units", "mm"),
            "variants": variant_config.get("variants", []),
        }
        variable_schema_id = create_variable_schema(
            connection,
            aircraft_family=aircraft_family,
            schema_version=f"snappy_hifi_faired_cap_variants.v{pipeline_version}",
            schema=schema_payload,
        )
        campaign_id = create_campaign(
            connection,
            name=f"snappy high-fidelity OpenFOAM v{pipeline_version} evidence import",
            description=(
                "Imported five approved faired-cap snappyHexMesh/OpenFOAM cases. "
                "High-fidelity mesher evidence plus qualified rough/confirmation scoring."
            ),
            aircraft_family=aircraft_family,
            variable_schema_id=variable_schema_id,
            config={
                "pipeline_id": pipeline_id,
                "pipeline_version": pipeline_version,
                "summary_path": str(summary_path),
                "ladder_summary_path": str(ladder_summary_path)
                if ladder_summary_path is not None
                else None,
                "alpha_sweep_summary_path": str(alpha_sweep_summary_path)
                if alpha_sweep_summary_path.exists()
                else None,
                "variant_config_path": str(variant_config_path),
                "preset_path": str(preset_path),
                "rough_scoring_config_path": str(rough_scoring_config_path)
                if rough_scoring_config_path is not None
                else None,
                "scoring_allowed": False,
            },
        )
        log_event(
            connection,
            event_type="campaign.created",
            campaign_id=campaign_id,
            actor="system",
            message=(
                f"Created snappy high-fidelity OpenFOAM v{pipeline_version} "
                "evidence-import campaign."
            ),
        )
        environment_fingerprint_id = create_environment_fingerprint(
            connection,
            tools={
                "mesher": "OpenFOAM snappyHexMesh",
                "openfoam": "OpenFOAM 13 via WSL",
                "solver_smoke": "potentialFoam -writep",
                "steady_solver": "foamRun -solver incompressibleFluid",
                "preset": str(preset_path),
            },
            notes=(
                f"Imported existing snappy v{pipeline_version}/OpenFOAM artifacts; "
                "did not rerun meshing or solver."
            ),
        )
        optimizer_run_id = create_optimizer_run(
            connection,
            campaign_id=campaign_id,
            optimizer_name="snappy_openfoam_import_ranker",
            optimizer_version=pipeline_version,
            objective={
                "primary": "score.rough_total",
                "direction": "maximize",
                "scope": pipeline_id,
                "scoring_config_path": str(rough_scoring_config_path)
                if rough_scoring_config_path is not None
                else None,
            },
            settings={
                "execution_model": "imported_existing_evidence",
                "candidate_batch_size": "all_imported",
                "steady_acceptance_mode": steady_acceptance_mode,
                "rough_scoring_enabled": include_steady,
            },
        )
        log_event(
            connection,
            event_type="optimizer.started",
            campaign_id=campaign_id,
            actor="optimizer",
            message="Started rough CFD import ranking run.",
            payload={"optimizer_run_id": optimizer_run_id},
        )

        imported = []
        for report in summary.get("reports", []):
            variant_id = report["variant_id"]
            variant = variants_by_id.get(variant_id, {})
            overrides = variant.get("overrides", {})
            design_variables = {
                key: VariableValue(value, _variant_unit(key))
                for key, value in overrides.items()
            }
            if not design_variables:
                design_variables = {
                    "source.imported_stl_index": VariableValue(
                        float(len(imported)),
                        "index",
                    )
                }
            candidate = CandidateSeed(
                aircraft_family=aircraft_family,
                design_variables=design_variables,
                normalized_design_vector={key: 0.5 for key in design_variables},
                created_by="snappy_hifi_evidence_import",
                generation=0,
                notes=(
                    f"Imported snappy hifi OpenFOAM v{pipeline_version} "
                    f"evidence for {variant_id}."
                ),
            )
            candidate.validate()
            run_dir = _resolve_project_path(project_root, Path(report["run_dir"]))
            case_dir = run_dir / "openfoam_case"
            surface_fidelity_path = run_dir / surface_fidelity_report_name
            registration = register_candidate_with_lineage(
                connection,
                campaign_id=campaign_id,
                variable_schema_id=variable_schema_id,
                candidate=candidate,
                parent_candidate_ids=[],
                operator="snappy_hifi_evidence_import",
                operator_version=pipeline_version,
                reason=(
                    "Import existing five-variant snappy/OpenFOAM "
                    f"v{pipeline_version} evidence."
                ),
                mutation_summary={
                    "variant_id": variant_id,
                    "description": variant.get("description"),
                    "output_stem": variant.get("output_stem"),
                    "input_stl": report.get("input_stl"),
                    "case_dir": str(case_dir),
                    "surface_fidelity_report": str(surface_fidelity_path)
                    if surface_fidelity_path.exists()
                    else None,
                },
                registration_message=f"Registered imported snappy hifi candidate {variant_id}.",
                lineage_message=f"Recorded imported snappy hifi lineage for {variant_id}.",
            )
            evaluation_id = start_evaluation_record(
                connection,
                campaign_id=campaign_id,
                candidate_id=registration.candidate_id,
                pipeline_id=pipeline_id,
                pipeline_version=pipeline_version,
                requested_by="cli",
                reason="Import existing snappy high-fidelity OpenFOAM evidence.",
                environment_fingerprint_id=environment_fingerprint_id,
                message=f"Started imported snappy hifi evaluation for {variant_id}.",
            )
            surface_attempt_id = None
            if surface_fidelity_path.exists():
                surface_attempt_id = persist_snappy_surface_fidelity_result(
                    connection,
                    artifact_root=artifact_root,
                    campaign_id=campaign_id,
                    candidate_id=registration.candidate_id,
                    evaluation_id=evaluation_id,
                    surface_fidelity_report=surface_fidelity_path,
                    requested_by="cli",
                )
            smoke_attempt_id = persist_openfoam_smoke_result(
                connection,
                artifact_root=artifact_root,
                campaign_id=campaign_id,
                candidate_id=registration.candidate_id,
                evaluation_id=evaluation_id,
                case_dir=case_dir,
                check_mesh_log=case_dir / "log.checkMesh",
                solver_log=case_dir / "log.potentialFoam",
                acceptance_mode=acceptance_mode,
                strict_check_mesh_log=case_dir / "log.checkMesh.strict",
                aircraft_iso_path=run_dir / "aircraft_iso.png",
                requested_by="cli",
            )
            steady_attempt_id = None
            rough_scoring_attempt_id = None
            steady_report = steady_by_variant.get(variant_id, {})
            if include_steady:
                rans_case = Path(
                    steady_report.get("rans_case")
                    or run_dir / "kOmegaSST_laminarStart_k002_om50_p01_u02_100"
                )
                rans_case = _resolve_project_path(project_root, rans_case)
                if (rans_case / "log.foamRun").exists():
                    steady_attempt_id = persist_openfoam_steady_result(
                        connection,
                        artifact_root=artifact_root,
                        campaign_id=campaign_id,
                        candidate_id=registration.candidate_id,
                        evaluation_id=evaluation_id,
                        case_dir=rans_case,
                        solver_log=rans_case / "log.foamRun",
                        check_mesh_log=rans_case / "log.checkMesh",
                        force_coeffs_path=(
                            rans_case
                            / "postProcessing"
                            / "aircraftForceCoeffs"
                            / "0"
                            / "forceCoeffs.dat"
                        ),
                        yplus_path=None,
                        aircraft_iso_path=run_dir / "aircraft_iso.png",
                        upstream_module_attempt_id=smoke_attempt_id,
                        policy=steady_policy,
                        requested_by="cli",
                    )
                    steady_payload = _module_attempt_payload(
                        connection,
                        steady_attempt_id,
                    )
                    surface_payload = (
                        _module_attempt_payload(connection, surface_attempt_id)
                        if surface_attempt_id is not None
                        else {"metrics": {}, "metadata": {}}
                    )
                    rough_metrics, rough_metadata, rough_warnings = (
                        stable_efficient_drone_rough_score(
                            steady_metrics=steady_payload["metrics"],
                            steady_metadata=steady_payload["metadata"],
                            surface_metrics=surface_payload["metrics"],
                            alpha_sweep=(
                                alpha_sweep_by_variant.get(variant_id, {}).get("aggregate")
                            ),
                            config=rough_scoring_config,
                        )
                    )
                    rough_scoring_attempt_id = run_successful_module_attempt(
                        connection,
                        campaign_id=campaign_id,
                        candidate_id=registration.candidate_id,
                        evaluation_id=evaluation_id,
                        module_name=ROUGH_CFD_SCORING_MODULE_NAME,
                        module_version=ROUGH_CFD_SCORING_MODULE_VERSION,
                        module_kind="scoring",
                        inputs={
                            "upstream_openfoam_steady_module_attempt_id": steady_attempt_id,
                            "upstream_surface_fidelity_module_attempt_id": surface_attempt_id,
                            "scoring_config_path": str(rough_scoring_config_path)
                            if rough_scoring_config_path is not None
                            else None,
                            "scoring_config_id": rough_metadata["scoring_config"]["id"],
                        },
                        metrics=rough_metrics,
                        metadata=rough_metadata,
                        started_message="Started rough CFD scoring.",
                        completed_message="Completed rough CFD scoring.",
                    )
            update_candidate_status(
                connection,
                candidate_id=registration.candidate_id,
                status="evaluated",
            )
            complete_evaluation(
                connection,
                evaluation_id=evaluation_id,
                summary={
                    "variant_id": variant_id,
                    "module_attempt_ids": [
                        item
                        for item in [
                            surface_attempt_id,
                            smoke_attempt_id,
                            steady_attempt_id,
                            rough_scoring_attempt_id,
                        ]
                        if item is not None
                    ],
                    "acceptance_mode": acceptance_mode,
                    "pipeline_id": pipeline_id,
                    "pipeline_version": pipeline_version,
                    "scoring_tier": steady_policy.get("scoring_tier")
                    if include_steady
                    else None,
                    "score_usage": steady_policy.get("score_usage") if include_steady else None,
                    "rough_scoring_allowed": bool(
                        include_steady and steady_policy.get("allows_rough_scoring")
                    ),
                    "final_scoring_allowed": bool(
                        include_steady and steady_policy.get("allows_final_scoring")
                    ),
                    "scoring_allowed": False,
                    "mesh_summary": report.get("mesh_summary"),
                    "potential_summary": report.get("potential_summary"),
                    "surface_fidelity_report": str(surface_fidelity_path)
                    if surface_fidelity_path.exists()
                    else None,
                    "steady_summary": steady_report,
                    "alpha_sweep_summary": alpha_sweep_by_variant.get(variant_id),
                    "rough_scoring_config_path": str(rough_scoring_config_path)
                    if rough_scoring_config_path is not None
                    else None,
                },
            )
            imported.append(
                {
                    "variant_id": variant_id,
                    "candidate_id": registration.candidate_id,
                    "evaluation_id": evaluation_id,
                    "surface_module_attempt_id": surface_attempt_id,
                    "smoke_module_attempt_id": smoke_attempt_id,
                    "steady_module_attempt_id": steady_attempt_id,
                    "rough_scoring_module_attempt_id": rough_scoring_attempt_id,
                }
            )

        best_candidate_id = _best_candidate_by_metric(
            connection,
            campaign_id=campaign_id,
            metric_key="score.rough_total",
        )
        complete_optimizer_run(
            connection,
            optimizer_run_id=optimizer_run_id,
            summary={
                "status": "complete",
                "selection_basis": "score.rough_total",
                "best_candidate_id": best_candidate_id,
                "imported_candidate_count": len(imported),
                "rough_scoring_enabled": include_steady,
            },
        )
        log_event(
            connection,
            event_type="optimizer.completed",
            campaign_id=campaign_id,
            candidate_id=best_candidate_id,
            actor="optimizer",
            message="Completed rough CFD import ranking run.",
            payload={
                "optimizer_run_id": optimizer_run_id,
                "best_candidate_id": best_candidate_id,
            },
        )
        complete_campaign(connection, campaign_id=campaign_id)
        connection.commit()
        print(
            json.dumps(
                {
                    "workspace": str(workspace),
                    "campaign_id": campaign_id,
                    "optimizer_run_id": optimizer_run_id,
                    "best_candidate_id": best_candidate_id,
                    "imported_count": len(imported),
                    "steady_import_enabled": include_steady,
                    "scoring_tier": steady_policy.get("scoring_tier")
                    if include_steady
                    else None,
                    "score_usage": steady_policy.get("score_usage") if include_steady else None,
                    "rough_scoring_allowed": bool(
                        include_steady and steady_policy.get("allows_rough_scoring")
                    ),
                    "final_scoring_allowed": bool(
                        include_steady and steady_policy.get("allows_final_scoring")
                    ),
                    "scoring_allowed": False,
                    "imported": imported,
                },
                indent=2,
                sort_keys=True,
            )
        )
    finally:
        connection.close()


def run_snappy_openfoam_backend(
    workspace: Path,
    run_root: Path,
    variant_ids: str,
    input_stl: Path | None,
    single_variant_id: str,
    input_stl_map: Path | None,
    variant_config_path: Path | None,
    preset_path: Path | None,
    rough_scoring_config_path: Path | None,
    alpha_sweep_summary_name: str | None,
    alpha_strategy: str,
    adaptive_baseline_summaries: list[Path] | None,
    skip_execution: bool,
) -> None:
    platform_root = find_platform_root()
    project_root = platform_root.parent
    run_root = _resolve_project_path(project_root, run_root)
    if input_stl is not None and input_stl_map is not None:
        raise SystemExit("Use only one of --input-stl or --input-stl-map.")
    input_stl_map = (
        _resolve_project_path(project_root, input_stl_map)
        if input_stl_map is not None
        else None
    )
    variant_config_path = (
        _resolve_project_path(project_root, variant_config_path)
        if variant_config_path is not None
        else None
    )
    preset_path = preset_path or (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "snappy_openfoam_external_aero_hifi_surface.v0_4.json"
    )
    preset = json.loads(preset_path.read_text(encoding="utf-8"))
    optimizer_import = preset.get("optimizer_import", {})
    selected_alpha_sweep_summary_name = alpha_sweep_summary_name or optimizer_import.get(
        "alpha_sweep_summary_name"
    )

    if not skip_execution:
        scripts_dir = project_root / "custom_cfd_mesher_experiment" / "scripts"
        mesher = preset["mesher"]
        layers = preset["layers"]
        smoke = preset["solver_smoke"]
        if run_root.exists():
            raise SystemExit(
                f"Run root already exists; choose a fresh path for execution: {run_root}"
            )
        run_root.parent.mkdir(parents=True, exist_ok=True)

        mesh_command = [
            sys.executable,
            str(scripts_dir / "run_snappy_layer_comparison.py"),
            "--run-root",
            str(run_root),
            "--velocity",
            ",".join(str(value) for value in smoke["freestream_mps"]),
            "--base-cells",
            mesher["base_cells"],
            "--pad",
            str(mesher["pad_m"]),
            "--surface-min-level",
            str(mesher["surface_min_level"]),
            "--surface-max-level",
            str(mesher["surface_max_level"]),
            "--feature-level",
            str(mesher["feature_level"]),
            "--n-cells-between-levels",
            str(mesher["n_cells_between_levels"]),
            "--snap-tolerance",
            str(mesher["snap_tolerance"]),
            "--n-smooth-patch",
            str(mesher["n_smooth_patch"]),
            "--n-surface-layers",
            str(layers["n_surface_layers"]),
            "--layer-relative-sizes",
            "true" if layers["layer_relative_sizes"] else "false",
            "--layer-expansion-ratio",
            str(layers["layer_expansion_ratio"]),
            "--final-layer-thickness",
            str(layers["final_layer_thickness"]),
            "--min-layer-thickness",
            str(layers["min_layer_thickness"]),
            "--layer-feature-angle",
            str(layers["layer_feature_angle_deg"]),
            "--n-layer-iter",
            str(layers["n_layer_iter"]),
            "--n-relaxed-iter",
            str(layers["n_relaxed_iter"]),
            "--max-global-cells",
            str(mesher["max_global_cells"]),
            "--max-local-cells",
            str(mesher["max_local_cells"]),
            "--feature-refinement-level",
            str(mesher["feature_refinement_level"]),
            "--feature-angle-deg",
            str(mesher["feature_angle_deg"]),
            "--feature-box-count",
            str(mesher["feature_box_count"]),
            "--feature-box-grid",
            mesher["feature_box_grid"],
            "--feature-box-padding-frac",
            str(mesher["feature_box_padding_frac"]),
            "--parallel-procs",
            str(mesher["parallel_procs"]),
            "--check-skew-threshold",
            str(mesher["check_skew_threshold"]),
            "--surface-fidelity-audit",
        ]
        if mesher.get("feature_refinement_boxes", True):
            mesh_command.append("--feature-refinement-boxes")
        if mesher.get("include_known_hotspot_boxes", True):
            mesh_command.append("--include-known-hotspot-boxes")
        if input_stl is not None:
            mesh_command.extend(
                [
                    "--input-stl",
                    str(_resolve_project_path(project_root, input_stl)),
                    "--single-variant-id",
                    single_variant_id,
                ]
            )
        if input_stl_map is not None:
            mesh_command.extend(["--input-stl-map", str(input_stl_map)])
        if variant_ids:
            mesh_command.extend(["--variant-ids", variant_ids])
        subprocess.run(mesh_command, check=True)

        rans_command = [
            sys.executable,
            str(scripts_dir / "run_no_slip_laminar_start_rans.py"),
            "--run-root",
            str(run_root),
            "--velocity",
            ",".join(str(value) for value in smoke["freestream_mps"]),
            "--end-time",
            str(smoke["rans_end_time"]),
            "--mag-u-inf",
            str(smoke["freestream_mps"][0]),
            "--wall-treatment",
            smoke.get("wall_treatment", "wall_function"),
            "--parallel-procs",
            str(smoke.get("parallel_procs", 1)),
        ]
        if smoke.get("lift_dir"):
            rans_command.extend(["--lift-dir", _vector_arg(smoke["lift_dir"])])
        if smoke.get("drag_dir"):
            rans_command.extend(["--drag-dir", _vector_arg(smoke["drag_dir"])])
        if smoke.get("pitch_axis"):
            rans_command.extend(["--pitch-axis", _vector_arg(smoke["pitch_axis"])])
        if "rans_timeout_s" in smoke:
            rans_command.extend(["--timeout-s", str(smoke["rans_timeout_s"])])
        if input_stl is not None:
            rans_command.extend(["--variant-ids", single_variant_id])
        if variant_ids:
            rans_command.extend(["--variant-ids", variant_ids])
        subprocess.run(rans_command, check=True)

        selected_alpha_sweep_summary_name = _execute_alpha_strategy(
            project_root=project_root,
            scripts_dir=scripts_dir,
            run_root=run_root,
            preset=preset,
            smoke=smoke,
            strategy=alpha_strategy,
            explicit_summary_name=alpha_sweep_summary_name,
            adaptive_baseline_summaries=adaptive_baseline_summaries or [],
            variant_ids=variant_ids,
            input_stl=input_stl,
            single_variant_id=single_variant_id,
        )

    ingest_snappy_hifi_run(
        workspace=workspace,
        run_root=run_root,
        variant_config_path=variant_config_path,
        preset_path=preset_path,
        include_steady=True,
        rough_scoring_config_path=rough_scoring_config_path,
        alpha_sweep_summary_name_override=selected_alpha_sweep_summary_name,
    )


def _execute_alpha_strategy(
    *,
    project_root: Path,
    scripts_dir: Path,
    run_root: Path,
    preset: dict[str, Any],
    smoke: dict[str, Any],
    strategy: str,
    explicit_summary_name: str | None,
    adaptive_baseline_summaries: list[Path],
    variant_ids: str,
    input_stl: Path | None,
    single_variant_id: str,
) -> str | None:
    if strategy == "none":
        return explicit_summary_name or preset.get("optimizer_import", {}).get(
            "alpha_sweep_summary_name"
        )
    first_stage = _alpha_stage_by_key(preset, "first_pass")
    survivor_stage = _alpha_stage_by_key(preset, "survivor_expanded")
    if strategy in {"first-pass", "survivor-expanded"}:
        stage = first_stage if strategy == "first-pass" else survivor_stage
        if stage is None:
            return explicit_summary_name
        _run_alpha_sweep_stage(
            scripts_dir=scripts_dir,
            run_root=run_root,
            smoke=smoke,
            stage=stage,
            variant_ids=variant_ids,
            input_stl=input_stl,
            single_variant_id=single_variant_id,
        )
        return str(stage["summary_name"])

    if explicit_summary_name:
        fixed_stage = _select_alpha_sweep_stage(
            preset=preset,
            summary_name=explicit_summary_name,
        )
        if fixed_stage is not None:
            _run_alpha_sweep_stage(
                scripts_dir=scripts_dir,
                run_root=run_root,
                smoke=smoke,
                stage=fixed_stage,
                variant_ids=variant_ids,
                input_stl=input_stl,
                single_variant_id=single_variant_id,
            )
        return explicit_summary_name

    if first_stage is None or survivor_stage is None:
        fallback = _select_alpha_sweep_stage(preset=preset, summary_name=None)
        if fallback is not None:
            _run_alpha_sweep_stage(
                scripts_dir=scripts_dir,
                run_root=run_root,
                smoke=smoke,
                stage=fallback,
                variant_ids=variant_ids,
                input_stl=input_stl,
                single_variant_id=single_variant_id,
            )
            return str(fallback["summary_name"])
        return None

    baseline_sweeps = _load_alpha_baseline_sweeps(
        project_root=project_root,
        summary_paths=adaptive_baseline_summaries,
    )
    policy = _adaptive_cfd_promotion_policy_from_preset(preset)
    print(
        json.dumps(
            {
                "event": "adaptive_cfd_promotion_baseline",
                **adaptive_cfd_promotion_report(
                    expanded_baseline_sweeps=baseline_sweeps,
                    policy=policy,
                ),
            },
            indent=2,
            sort_keys=True,
        )
    )
    if len(baseline_sweeps) < policy.min_expanded_baseline_candidates:
        _run_alpha_sweep_stage(
            scripts_dir=scripts_dir,
            run_root=run_root,
            smoke=smoke,
            stage=survivor_stage,
            variant_ids=variant_ids,
            input_stl=input_stl,
            single_variant_id=single_variant_id,
        )
        return str(survivor_stage["summary_name"])

    _run_alpha_sweep_stage(
        scripts_dir=scripts_dir,
        run_root=run_root,
        smoke=smoke,
        stage=first_stage,
        variant_ids=variant_ids,
        input_stl=input_stl,
        single_variant_id=single_variant_id,
    )
    first_summary_path = run_root / str(first_stage["summary_name"])
    first_summary = json.loads(first_summary_path.read_text(encoding="utf-8-sig"))
    decisions: dict[str, dict[str, Any]] = {}
    promote_variant_ids: list[str] = []
    for report in first_summary.get("reports", []):
        decision = decide_adaptive_cfd_promotion(
            candidate_first_pass=report.get("aggregate", {}),
            expanded_baseline_sweeps=baseline_sweeps,
            policy=policy,
        )
        decisions[report["variant_id"]] = _promotion_decision_payload(decision)
        print(
            json.dumps(
                {
                    "event": "adaptive_cfd_promotion_decision",
                    "variant_id": report["variant_id"],
                    **adaptive_cfd_promotion_report(
                        expanded_baseline_sweeps=baseline_sweeps,
                        candidate_first_pass=report.get("aggregate", {}),
                        policy=policy,
                    ),
                },
                indent=2,
                sort_keys=True,
            )
        )
        if decision.promote_to_expanded_sweep:
            promote_variant_ids.append(report["variant_id"])

    survivor_summary: dict[str, Any] | None = None
    if promote_variant_ids:
        _run_alpha_sweep_stage(
            scripts_dir=scripts_dir,
            run_root=run_root,
            smoke=smoke,
            stage=survivor_stage,
            variant_ids=",".join(promote_variant_ids),
            input_stl=input_stl,
            single_variant_id=single_variant_id,
        )
        survivor_summary = json.loads(
            (run_root / str(survivor_stage["summary_name"])).read_text(
                encoding="utf-8-sig"
            )
        )
    adaptive_summary_name = _adaptive_alpha_summary_name(smoke)
    _write_adaptive_alpha_summary(
        run_root=run_root,
        summary_name=adaptive_summary_name,
        first_summary=first_summary,
        survivor_summary=survivor_summary,
        decisions=decisions,
        policy=policy,
    )
    return adaptive_summary_name


def _vector_arg(values: Any) -> str:
    if isinstance(values, str):
        return values
    return ",".join(str(value) for value in values)


def _run_alpha_sweep_stage(
    *,
    scripts_dir: Path,
    run_root: Path,
    smoke: dict[str, Any],
    stage: dict[str, Any],
    variant_ids: str,
    input_stl: Path | None,
    single_variant_id: str,
) -> None:
    alpha_command = [
        sys.executable,
        str(scripts_dir / "run_no_slip_alpha_sweep.py"),
        "--run-root",
        str(run_root),
        f"--alphas-deg={','.join(str(value) for value in stage['angles_deg'])}",
        "--speed-mps",
        str(smoke["freestream_mps"][0]),
        "--end-time",
        str(smoke["rans_end_time"]),
        "--wall-treatment",
        smoke.get("wall_treatment", "wall_function"),
        "--parallel-procs",
        str(smoke.get("parallel_procs", 1)),
        "--summary-name",
        str(stage["summary_name"]),
        "--stage-name",
        str(stage["stage_name"]),
        "--stage-role",
        str(stage["stage_role"]),
    ]
    if "rans_timeout_s" in smoke:
        alpha_command.extend(["--timeout-s", str(smoke["rans_timeout_s"])])
    if smoke.get("vertical_axis"):
        alpha_command.extend(["--vertical-axis", str(smoke["vertical_axis"])])
    alpha_command.extend(_alpha_pruning_args(stage))
    if input_stl is not None:
        alpha_command.extend(["--variant-ids", single_variant_id])
    if variant_ids:
        alpha_command.extend(["--variant-ids", variant_ids])
    subprocess.run(alpha_command, check=True)


def _alpha_pruning_args(stage: dict[str, Any]) -> list[str]:
    policy = stage.get("early_pruning") or {}
    if not policy.get("enabled", False):
        return []
    args = ["--early-prune"]
    option_map = {
        "min_completed_alphas": "--early-prune-min-completed-alphas",
        "no_lift_cl_max": "--early-prune-no-lift-cl-max",
        "weak_lift_cl_max": "--early-prune-weak-lift-cl-max",
        "min_best_ld": "--early-prune-min-best-ld",
        "max_abs_coefficient": "--early-prune-max-abs-coefficient",
    }
    for key, option in option_map.items():
        if key in policy:
            args.extend([option, str(policy[key])])
    return args


def _load_alpha_baseline_sweeps(
    *,
    project_root: Path,
    summary_paths: list[Path],
) -> list[dict[str, Any]]:
    sweeps: list[dict[str, Any]] = []
    for summary_path in summary_paths:
        resolved = _resolve_project_path(project_root, summary_path)
        if not resolved.exists():
            raise FileNotFoundError(f"adaptive baseline summary not found: {resolved}")
        summary = json.loads(resolved.read_text(encoding="utf-8-sig"))
        for report in summary.get("reports", []):
            aggregate = report.get("aggregate")
            if aggregate:
                sweeps.append(aggregate)
    return sweeps


def _load_candidate_alpha_reports(
    *,
    project_root: Path,
    summary_path: Path | None,
    variant_id: str | None,
) -> list[dict[str, Any]]:
    if summary_path is None:
        return []
    resolved = _resolve_project_path(project_root, summary_path)
    if not resolved.exists():
        raise FileNotFoundError(f"candidate alpha summary not found: {resolved}")
    summary = json.loads(resolved.read_text(encoding="utf-8-sig"))
    reports = list(summary.get("reports", []))
    if variant_id:
        reports = [report for report in reports if report.get("variant_id") == variant_id]
    return reports


def _adaptive_cfd_promotion_policy_from_preset(
    preset: dict[str, Any],
) -> AdaptiveCfdPromotionPolicy:
    policy = (
        preset.get("solver_smoke", {})
        .get("alpha_sweep_policy", {})
        .get("adaptive_promotion_policy", {})
    )
    return AdaptiveCfdPromotionPolicy(
        min_expanded_baseline_candidates=int(
            policy.get("min_expanded_baseline_candidates", 5)
        ),
        first_pass_compare_alpha_deg=float(policy.get("compare_alpha_deg", 4.0)),
        max_lift_loss_fraction_vs_baseline_best=float(
            policy.get("max_lift_loss_fraction_vs_baseline_best", 0.65)
        ),
        max_abs_coefficient=float(policy.get("max_abs_coefficient", 10.0)),
        require_positive_lift_at_any_sampled_alpha=bool(
            policy.get("require_positive_lift_at_any_sampled_alpha", True)
        ),
    )


def _promotion_decision_payload(decision: Any) -> dict[str, Any]:
    return {
        "action": decision.action,
        "reason": decision.reason,
        "promote_to_expanded_sweep": decision.promote_to_expanded_sweep,
        "reject_candidate": decision.reject_candidate,
        "baseline": decision.baseline,
        "comparisons": decision.comparisons,
    }


def _write_adaptive_alpha_summary(
    *,
    run_root: Path,
    summary_name: str,
    first_summary: dict[str, Any],
    survivor_summary: dict[str, Any] | None,
    decisions: dict[str, dict[str, Any]],
    policy: AdaptiveCfdPromotionPolicy,
) -> None:
    survivor_by_variant = {
        report["variant_id"]: report
        for report in (survivor_summary or {}).get("reports", [])
    }
    reports = []
    for first_report in first_summary.get("reports", []):
        variant_id = first_report["variant_id"]
        decision = decisions.get(variant_id, {})
        selected_report = survivor_by_variant.get(variant_id, first_report)
        selected_report = json.loads(json.dumps(selected_report))
        selected_report["adaptive_promotion_decision"] = decision
        selected_report["adaptive_source_stage"] = (
            "survivor_expanded"
            if variant_id in survivor_by_variant
            else "first_pass"
        )
        if selected_report.get("aggregate") is not None:
            selected_report["aggregate"]["adaptive_promotion_decision"] = decision
            selected_report["aggregate"]["adaptive_source_stage"] = selected_report[
                "adaptive_source_stage"
            ]
        reports.append(selected_report)
    summary = {
        "schema": "adaptive_no_slip_alpha_sweep.v0_1",
        "stage_name": "adaptive_rough_scoring",
        "stage_role": "adaptive_candidate_screening_and_survivor_expansion",
        "run_root": first_summary.get("run_root"),
        "policy": {
            "min_expanded_baseline_candidates": policy.min_expanded_baseline_candidates,
            "first_pass_compare_alpha_deg": policy.first_pass_compare_alpha_deg,
            "max_lift_loss_fraction_vs_baseline_best": (
                policy.max_lift_loss_fraction_vs_baseline_best
            ),
            "max_abs_coefficient": policy.max_abs_coefficient,
            "require_positive_lift_at_any_sampled_alpha": (
                policy.require_positive_lift_at_any_sampled_alpha
            ),
        },
        "variant_count": len(reports),
        "reports": reports,
    }
    (run_root / summary_name).write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def _adaptive_alpha_summary_name(smoke: dict[str, Any]) -> str:
    return (
        f"alpha_sweep_summary_adaptive_end{int(smoke['rans_end_time'])}"
        f"_np{int(smoke.get('parallel_procs', 1))}.json"
    )


def _alpha_stage_by_key(
    preset: dict[str, Any],
    key: str,
) -> dict[str, Any] | None:
    return (
        preset.get("solver_smoke", {})
        .get("alpha_sweep_policy", {})
        .get("two_stage_policy", {})
        .get(key)
    )


def _select_alpha_sweep_stage(
    *,
    preset: dict[str, Any],
    summary_name: str | None,
) -> dict[str, Any] | None:
    policy = preset.get("solver_smoke", {}).get("alpha_sweep_policy", {})
    stages = policy.get("two_stage_policy", {})
    if not stages:
        return None
    selected = summary_name or preset.get("optimizer_import", {}).get(
        "alpha_sweep_summary_name"
    )
    for stage in stages.values():
        if stage.get("summary_name") == selected:
            return stage
    return stages.get("first_pass")


def _module_attempt_payload(
    connection,
    module_attempt_id: str,
) -> dict[str, dict]:
    row = connection.execute(
        """
        SELECT metrics_json, metadata_json
        FROM module_attempts
        WHERE module_attempt_id = ?
        """,
        (module_attempt_id,),
    ).fetchone()
    if row is None:
        return {"metrics": {}, "metadata": {}}
    metrics_json, metadata_json = row
    return {
        "metrics": json.loads(metrics_json or "{}"),
        "metadata": json.loads(metadata_json or "{}"),
    }


def _best_candidate_by_metric(
    connection,
    *,
    campaign_id: str,
    metric_key: str,
) -> str | None:
    rows = connection.execute(
        """
        SELECT m.candidate_id, m.metrics_json
        FROM module_attempts m
        JOIN evaluations e ON e.evaluation_id = m.evaluation_id
        WHERE e.campaign_id = ?
          AND m.status = 'success'
        ORDER BY m.finished_at DESC, m.rowid DESC
        """,
        (campaign_id,),
    ).fetchall()
    scores: dict[str, float] = {}
    for candidate_id, metrics_json in rows:
        metrics = json.loads(metrics_json or "{}")
        metric = metrics.get(metric_key)
        if isinstance(metric, dict) and metric.get("value") is not None:
            scores[candidate_id] = float(metric["value"])
    if not scores:
        return None
    return max(scores, key=scores.get)


def _resolve_project_path(project_root: Path, path: Path) -> Path:
    if path.is_absolute():
        return path
    candidate = project_root / path
    if candidate.exists():
        return candidate
    return path


def select_ladder_summary_path(run_root: Path) -> Path | None:
    canonical = run_root / "no_slip_ladder_summary.json"
    if canonical.exists():
        return canonical
    matches = sorted(
        run_root.glob("no_slip_ladder_summary_*.json"),
        key=lambda path: (path.stat().st_mtime, path.name),
        reverse=True,
    )
    return matches[0] if matches else None


def _select_evaluation_context(
    connection,
    *,
    campaign_id: str | None,
    candidate_id: str | None,
    evaluation_id: str | None,
) -> dict[str, str]:
    if evaluation_id is not None:
        row = connection.execute(
            """
            SELECT campaign_id, candidate_id, evaluation_id
            FROM evaluations
            WHERE evaluation_id = ?
            """,
            (evaluation_id,),
        ).fetchone()
    elif candidate_id is not None:
        row = connection.execute(
            """
            SELECT campaign_id, candidate_id, evaluation_id
            FROM evaluations
            WHERE candidate_id = ?
            ORDER BY started_at DESC
            LIMIT 1
            """,
            (candidate_id,),
        ).fetchone()
    elif campaign_id is not None:
        row = connection.execute(
            """
            SELECT campaign_id, candidate_id, evaluation_id
            FROM evaluations
            WHERE campaign_id = ?
            ORDER BY started_at DESC
            LIMIT 1
            """,
            (campaign_id,),
        ).fetchone()
    else:
        row = connection.execute(
            """
            SELECT campaign_id, candidate_id, evaluation_id
            FROM evaluations
            ORDER BY started_at DESC
            LIMIT 1
            """
        ).fetchone()
    if row is None:
        raise SystemExit("No matching evaluation found in optimizer workspace.")
    return {
        "campaign_id": row[0],
        "candidate_id": row[1],
        "evaluation_id": row[2],
    }


def preflight_real_adapters(workspace: Path) -> None:
    report = preflight_real_no_inlet_export_boundary(
        platform_root=find_platform_root(),
        workspace=workspace,
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser(prog="aircraft-optimizer")
    subparsers = parser.add_subparsers(dest="command", required=True)
    fixture = subparsers.add_parser("run-fixture")
    fixture.add_argument("--workspace", type=Path, required=True)
    wing_options = subparsers.add_parser("run-wing-options")
    wing_options.add_argument("--workspace", type=Path, required=True)
    sequential = subparsers.add_parser("run-sequential-gated")
    sequential.add_argument("--workspace", type=Path, required=True)
    sequential.add_argument("--iterations", type=int, default=3)
    sequential.add_argument("--virtual-components-config", type=Path, default=None)
    real_export = subparsers.add_parser("run-real-no-inlet-export")
    real_export.add_argument("--workspace", type=Path, required=True)
    real_export.add_argument("--timeout-seconds", type=int, default=1200)
    real_export_batch = subparsers.add_parser("run-real-no-inlet-export-batch")
    real_export_batch.add_argument("--workspace", type=Path, required=True)
    real_export_batch.add_argument("--iterations", type=int, default=5)
    real_export_batch.add_argument("--timeout-seconds", type=int, default=1200)
    real_optimizer_pilot = subparsers.add_parser("run-real-optimizer-pilot")
    real_optimizer_pilot.add_argument("--workspace", type=Path, required=True)
    real_optimizer_pilot.add_argument("--run-slug", required=True)
    real_optimizer_pilot.add_argument("--initial-count", type=int, default=6)
    real_optimizer_pilot.add_argument("--children-count", type=int, default=3)
    real_optimizer_pilot.add_argument("--timeout-seconds", type=int, default=1200)
    real_optimizer_pilot.add_argument(
        "--alpha-strategy",
        choices=["adaptive", "first-pass", "survivor-expanded", "none"],
        default="first-pass",
    )
    real_optimizer_pilot.add_argument(
        "--resume",
        action="store_true",
        help="Reuse completed export, mesh, alpha, and import artifacts when present.",
    )
    summary = subparsers.add_parser("summarize")
    summary.add_argument("--workspace", type=Path, required=True)
    adaptive_report_parser = subparsers.add_parser("report-adaptive-cfd-promotion")
    adaptive_report_parser.add_argument("--preset", type=Path, default=None)
    adaptive_report_parser.add_argument(
        "--baseline-summary",
        action="append",
        type=Path,
        default=[],
        help="Expanded/adaptive alpha summary to use as baseline. Can be repeated.",
    )
    adaptive_report_parser.add_argument(
        "--candidate-alpha-summary",
        type=Path,
        default=None,
        help="Optional first-pass/adaptive alpha summary for the candidate being judged.",
    )
    adaptive_report_parser.add_argument(
        "--variant-id",
        default=None,
        help="Optional variant id inside --candidate-alpha-summary.",
    )
    report = subparsers.add_parser("report-evaluation")
    report.add_argument("--workspace", type=Path, required=True)
    report.add_argument("--evaluation-id", type=str, default=None)
    campaign_report_parser = subparsers.add_parser("report-campaign")
    campaign_report_parser.add_argument("--workspace", type=Path, required=True)
    campaign_report_parser.add_argument("--campaign-id", type=str, default=None)
    campaign_report_parser.add_argument("--failed-only", action="store_true")
    comparison_report_parser = subparsers.add_parser("report-comparison")
    comparison_report_parser.add_argument("--workspace", type=Path, required=True)
    comparison_report_parser.add_argument("--campaign-id", type=str, default=None)
    comparison_report_parser.add_argument("--candidate-id", type=str, default=None)
    comparison_report_parser.add_argument("--other-candidate-id", type=str, default=None)
    optimizer_report_parser = subparsers.add_parser("report-optimizer")
    optimizer_report_parser.add_argument("--workspace", type=Path, required=True)
    optimizer_report_parser.add_argument("--campaign-id", type=str, default=None)
    optimizer_report_parser.add_argument("--optimizer-run-id", type=str, default=None)
    layout_report_parser = subparsers.add_parser("report-layout")
    layout_report_parser.add_argument("--workspace", type=Path, required=True)
    layout_report_parser.add_argument("--candidate-id", type=str, default=None)
    layout_report_parser.add_argument("--evaluation-id", type=str, default=None)
    real_adapter_parser = subparsers.add_parser("preflight-real-adapters")
    real_adapter_parser.add_argument("--workspace", type=Path, required=True)
    cfd_readiness_parser = subparsers.add_parser("check-cfd-readiness")
    cfd_readiness_parser.add_argument("--export-result", type=Path, required=True)
    cfd_surface_parser = subparsers.add_parser("check-cfd-surface")
    cfd_surface_parser.add_argument("--stl", type=Path, required=True)
    mesh_result_parser = subparsers.add_parser("check-mesh-result")
    mesh_result_parser.add_argument("--mesh", type=Path, required=True)
    mesh_result_parser.add_argument(
        "--mesh-format",
        choices=["su2", "cgns"],
        required=True,
    )
    mesh_result_parser.add_argument("--su2-log", type=Path, default=None)
    mesh_result_parser.add_argument(
        "--aircraft-marker",
        action="append",
        dest="aircraft_markers",
    )
    mesh_result_parser.add_argument(
        "--farfield-marker",
        action="append",
        dest="farfield_markers",
    )
    mesh_result_parser.add_argument(
        "--fluid-marker",
        action="append",
        dest="fluid_markers",
    )
    openfoam_result_parser = subparsers.add_parser("check-openfoam-result")
    openfoam_result_parser.add_argument("--case-dir", type=Path, required=True)
    openfoam_result_parser.add_argument("--check-mesh-log", type=Path, required=True)
    openfoam_result_parser.add_argument("--solver-log", type=Path, required=True)
    openfoam_result_parser.add_argument(
        "--acceptance-mode",
        choices=[
            "strict",
            "relaxed_development",
            "gmsh_bl_development",
            "snappy_hifi_development",
        ],
        default="strict",
    )
    openfoam_smoke_parser = subparsers.add_parser("persist-openfoam-smoke")
    openfoam_smoke_parser.add_argument("--workspace", type=Path, required=True)
    openfoam_smoke_parser.add_argument("--case-dir", type=Path, required=True)
    openfoam_smoke_parser.add_argument("--check-mesh-log", type=Path, required=True)
    openfoam_smoke_parser.add_argument("--solver-log", type=Path, required=True)
    openfoam_smoke_parser.add_argument(
        "--acceptance-mode",
        choices=[
            "strict",
            "relaxed_development",
            "gmsh_bl_development",
            "snappy_hifi_development",
        ],
        default="relaxed_development",
    )
    openfoam_smoke_parser.add_argument("--strict-check-mesh-log", type=Path, default=None)
    openfoam_smoke_parser.add_argument("--relaxed-check-mesh-log", type=Path, default=None)
    openfoam_smoke_parser.add_argument("--aircraft-iso", type=Path, default=None)
    openfoam_smoke_parser.add_argument("--campaign-id", type=str, default=None)
    openfoam_smoke_parser.add_argument("--candidate-id", type=str, default=None)
    openfoam_smoke_parser.add_argument("--evaluation-id", type=str, default=None)
    steady_check_parser = subparsers.add_parser("check-openfoam-steady-result")
    steady_check_parser.add_argument("--case-dir", type=Path, required=True)
    steady_check_parser.add_argument("--solver-log", type=Path, required=True)
    steady_check_parser.add_argument("--check-mesh-log", type=Path, default=None)
    steady_check_parser.add_argument("--force-coeffs", type=Path, default=None)
    steady_check_parser.add_argument("--yplus", type=Path, default=None)
    steady_check_parser.add_argument(
        "--acceptance-mode",
        type=str,
        default="openfoam_steady_development",
        choices=sorted(OPENFOAM_STEADY_ACCEPTANCE_POLICIES),
    )
    steady_persist_parser = subparsers.add_parser("persist-openfoam-steady")
    steady_persist_parser.add_argument("--workspace", type=Path, required=True)
    steady_persist_parser.add_argument("--case-dir", type=Path, required=True)
    steady_persist_parser.add_argument("--solver-log", type=Path, required=True)
    steady_persist_parser.add_argument("--check-mesh-log", type=Path, default=None)
    steady_persist_parser.add_argument("--force-coeffs", type=Path, default=None)
    steady_persist_parser.add_argument("--yplus", type=Path, default=None)
    steady_persist_parser.add_argument("--aircraft-iso", type=Path, default=None)
    steady_persist_parser.add_argument(
        "--acceptance-mode",
        type=str,
        default="openfoam_steady_development",
        choices=sorted(OPENFOAM_STEADY_ACCEPTANCE_POLICIES),
    )
    steady_persist_parser.add_argument("--campaign-id", type=str, default=None)
    steady_persist_parser.add_argument("--candidate-id", type=str, default=None)
    steady_persist_parser.add_argument("--evaluation-id", type=str, default=None)
    gmsh_bl_ingest_parser = subparsers.add_parser("ingest-gmsh-bl-run")
    gmsh_bl_ingest_parser.add_argument("--workspace", type=Path, required=True)
    gmsh_bl_ingest_parser.add_argument("--summary", type=Path, required=True)
    gmsh_bl_ingest_parser.add_argument("--variant-config", type=Path, default=None)
    snappy_ingest_parser = subparsers.add_parser("ingest-snappy-hifi-run")
    snappy_ingest_parser.add_argument("--workspace", type=Path, required=True)
    snappy_ingest_parser.add_argument("--run-root", type=Path, required=True)
    snappy_ingest_parser.add_argument("--variant-config", type=Path, default=None)
    snappy_ingest_parser.add_argument("--preset", type=Path, default=None)
    snappy_ingest_parser.add_argument(
        "--rough-scoring-config",
        type=Path,
        default=None,
        help="Optional JSON config overriding the rough CFD scoring weights and gates.",
    )
    snappy_ingest_parser.add_argument(
        "--alpha-sweep-summary-name",
        default=None,
        help=(
            "Optional alpha-sweep summary filename under the run root. "
            "Use this to switch between first-pass and full-survivor scoring."
        ),
    )
    snappy_ingest_parser.add_argument(
        "--skip-steady",
        action="store_true",
        help="Only import mesh and potentialFoam smoke evidence.",
    )
    snappy_backend_parser = subparsers.add_parser("run-snappy-openfoam-backend")
    snappy_backend_parser.add_argument("--workspace", type=Path, required=True)
    snappy_backend_parser.add_argument("--run-root", type=Path, required=True)
    snappy_backend_parser.add_argument(
        "--variant-ids",
        default="",
        help="Comma-separated faired-cap variant ids. Empty means all variants.",
    )
    snappy_backend_parser.add_argument(
        "--input-stl",
        type=Path,
        default=None,
        help="Run one arbitrary STL instead of the built-in faired-cap variants.",
    )
    snappy_backend_parser.add_argument(
        "--single-variant-id",
        default="custom_aircraft",
        help="Variant id to use with --input-stl.",
    )
    snappy_backend_parser.add_argument(
        "--input-stl-map",
        type=Path,
        default=None,
        help="JSON object/list mapping variant ids to raw STL paths for a batch run.",
    )
    snappy_backend_parser.add_argument(
        "--variant-config",
        type=Path,
        default=None,
        help="Variant metadata config to use when importing backend results.",
    )
    snappy_backend_parser.add_argument("--preset", type=Path, default=None)
    snappy_backend_parser.add_argument(
        "--rough-scoring-config",
        type=Path,
        default=None,
        help="Optional JSON config overriding the rough CFD scoring weights and gates.",
    )
    snappy_backend_parser.add_argument(
        "--alpha-sweep-summary-name",
        default=None,
        help=(
            "Optional alpha-sweep summary filename under the run root. "
            "Use this to switch between first-pass and full-survivor scoring."
        ),
    )
    snappy_backend_parser.add_argument(
        "--alpha-strategy",
        choices=["adaptive", "first-pass", "survivor-expanded", "none"],
        default="first-pass",
        help=(
            "Alpha execution strategy for fresh backend runs. The default "
            "first-pass runs the cheap 0/+4 degree rough-scoring stage. "
            "Adaptive is available explicitly when baseline-building or "
            "conditional survivor expansion is desired."
        ),
    )
    snappy_backend_parser.add_argument(
        "--adaptive-baseline-summary",
        action="append",
        type=Path,
        default=[],
        help=(
            "Prior expanded/adaptive alpha summary used as adaptive promotion "
            "baseline. Can be repeated."
        ),
    )
    snappy_backend_parser.add_argument(
        "--skip-execution",
        action="store_true",
        help="Only import an existing backend run root.",
    )
    subparsers.add_parser("check-cfd-tools")
    args = parser.parse_args()

    if args.command == "run-fixture":
        run_fixture(args.workspace)
    elif args.command == "run-wing-options":
        run_wing_options(args.workspace)
    elif args.command == "run-sequential-gated":
        run_sequential_gated(
            args.workspace,
            args.iterations,
            args.virtual_components_config,
        )
    elif args.command == "run-real-no-inlet-export":
        run_real_no_inlet_export(args.workspace, args.timeout_seconds)
    elif args.command == "run-real-no-inlet-export-batch":
        run_real_no_inlet_export_batch_command(
            args.workspace,
            args.iterations,
            args.timeout_seconds,
        )
    elif args.command == "run-real-optimizer-pilot":
        run_real_optimizer_pilot_command(
            args.workspace,
            args.run_slug,
            args.initial_count,
            args.children_count,
            args.timeout_seconds,
            args.alpha_strategy,
            args.resume,
        )
    elif args.command == "summarize":
        summarize_workspace(args.workspace)
    elif args.command == "report-adaptive-cfd-promotion":
        report_adaptive_cfd_promotion(
            preset_path=args.preset,
            baseline_summaries=args.baseline_summary,
            candidate_alpha_summary=args.candidate_alpha_summary,
            variant_id=args.variant_id,
        )
    elif args.command == "report-evaluation":
        report_evaluation(args.workspace, args.evaluation_id)
    elif args.command == "report-campaign":
        report_campaign(args.workspace, args.campaign_id, args.failed_only)
    elif args.command == "report-comparison":
        report_comparison(
            args.workspace,
            args.campaign_id,
            args.candidate_id,
            args.other_candidate_id,
        )
    elif args.command == "report-optimizer":
        report_optimizer_run(args.workspace, args.campaign_id, args.optimizer_run_id)
    elif args.command == "report-layout":
        report_layout(args.workspace, args.candidate_id, args.evaluation_id)
    elif args.command == "preflight-real-adapters":
        preflight_real_adapters(args.workspace)
    elif args.command == "check-cfd-readiness":
        check_cfd_readiness(args.export_result)
    elif args.command == "check-cfd-surface":
        check_cfd_surface(args.stl)
    elif args.command == "check-mesh-result":
        check_mesh_result(
            args.mesh,
            args.mesh_format,
            args.su2_log,
            args.aircraft_markers,
            args.farfield_markers,
            args.fluid_markers,
        )
    elif args.command == "check-cfd-tools":
        check_cfd_tools()
    elif args.command == "check-openfoam-result":
        check_openfoam_result(
            args.case_dir,
            args.check_mesh_log,
            args.solver_log,
            args.acceptance_mode,
        )
    elif args.command == "persist-openfoam-smoke":
        persist_openfoam_smoke(
            args.workspace,
            args.case_dir,
            args.check_mesh_log,
            args.solver_log,
            args.acceptance_mode,
            args.strict_check_mesh_log,
            args.relaxed_check_mesh_log,
            args.aircraft_iso,
            args.campaign_id,
            args.candidate_id,
            args.evaluation_id,
        )
    elif args.command == "check-openfoam-steady-result":
        check_openfoam_steady_result(
            args.case_dir,
            args.solver_log,
            args.check_mesh_log,
            args.force_coeffs,
            args.yplus,
            args.acceptance_mode,
        )
    elif args.command == "persist-openfoam-steady":
        persist_openfoam_steady(
            args.workspace,
            args.case_dir,
            args.solver_log,
            args.check_mesh_log,
            args.force_coeffs,
            args.yplus,
            args.aircraft_iso,
            args.acceptance_mode,
            args.campaign_id,
            args.candidate_id,
            args.evaluation_id,
        )
    elif args.command == "ingest-gmsh-bl-run":
        ingest_gmsh_bl_run(args.workspace, args.summary, args.variant_config)
    elif args.command == "ingest-snappy-hifi-run":
        ingest_snappy_hifi_run(
            args.workspace,
            args.run_root,
            args.variant_config,
            args.preset,
            include_steady=not args.skip_steady,
            rough_scoring_config_path=args.rough_scoring_config,
            alpha_sweep_summary_name_override=args.alpha_sweep_summary_name,
        )
    elif args.command == "run-snappy-openfoam-backend":
        run_snappy_openfoam_backend(
            args.workspace,
            args.run_root,
            args.variant_ids,
            args.input_stl,
            args.single_variant_id,
            args.input_stl_map,
            args.variant_config,
            args.preset,
            args.rough_scoring_config,
            args.alpha_sweep_summary_name,
            args.alpha_strategy,
            args.adaptive_baseline_summary,
            args.skip_execution,
        )

if __name__ == "__main__":
    main()
