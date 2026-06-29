from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

from aircraft_optimizer.artifacts.registry import register_artifact
from aircraft_optimizer.db.repositories import complete_module_attempt, create_module_attempt
from aircraft_optimizer.events.event_log import log_event

MODULE_NAME = "su2_case_dry_run"
MODULE_VERSION = "0.1.0"


def persist_su2_dry_run_case(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    geometry_provider_result_id: str,
    export_module_attempt_id: str,
    mesh_reference_path: str,
    freestream: dict[str, Any] | None = None,
) -> str:
    case_inputs = {
        "execution_mode": "dry_run_no_solver",
        "geometry_provider_result_id": geometry_provider_result_id,
        "export_module_attempt_id": export_module_attempt_id,
        "mesh_reference_path": mesh_reference_path,
        "freestream": freestream or default_freestream(),
    }
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind="analysis_setup",
        inputs=case_inputs,
    )
    log_event(
        connection,
        event_type="module.started",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message="Started SU2 dry-run case builder.",
    )

    case_dir = artifact_root / "_generated_su2_cases" / evaluation_id
    case_dir.mkdir(parents=True, exist_ok=True)
    config_path = case_dir / "su2_config.cfg"
    manifest_path = case_dir / "su2_case_manifest.json"
    config_path.write_text(render_su2_config(case_inputs), encoding="utf-8")
    manifest_path.write_text(
        json.dumps(
            {
                "case_schema_version": "0.1.0",
                "execution_mode": "dry_run_no_solver",
                "candidate_id": candidate_id,
                "evaluation_id": evaluation_id,
                "geometry_provider_result_id": geometry_provider_result_id,
                "export_module_attempt_id": export_module_attempt_id,
                "artifacts": {
                    "su2_config": str(config_path),
                    "mesh_reference": mesh_reference_path,
                },
                "freestream": case_inputs["freestream"],
                "solver_run_performed": False,
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    config_artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=config_path,
        artifact_type="su2_config_cfg",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        producer_module=MODULE_NAME,
        producer_version=MODULE_VERSION,
        source_kind="generated_by_adapter",
        source_original_path=str(config_path),
        metadata={"execution_mode": "dry_run_no_solver"},
    )
    manifest_artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=manifest_path,
        artifact_type="su2_case_manifest_json",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        producer_module=MODULE_NAME,
        producer_version=MODULE_VERSION,
        source_kind="generated_by_adapter",
        source_original_path=str(manifest_path),
        metadata={"execution_mode": "dry_run_no_solver"},
    )
    for artifact_id in [config_artifact_id, manifest_artifact_id]:
        log_event(
            connection,
            event_type="artifact.created",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            actor="module",
            message="Registered SU2 dry-run case artifact.",
            payload={"artifact_id": artifact_id},
        )

    complete_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        metrics={
            "cfd_case.ready_for_solver": {
                "value": True,
                "source": MODULE_NAME,
                "confidence": "configuration_only",
                "runtime_cost": "none",
            },
            "cfd_case.solver_run_performed": {
                "value": False,
                "source": MODULE_NAME,
                "confidence": "exact",
                "runtime_cost": "none",
            },
        },
        metadata={
            "execution_mode": "dry_run_no_solver",
            "artifact_ids": [config_artifact_id, manifest_artifact_id],
        },
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
        message="Completed SU2 dry-run case builder.",
    )
    return module_attempt_id


def default_freestream() -> dict[str, Any]:
    return {
        "mach": 0.04,
        "aoa_deg": 4.0,
        "reynolds_number": 350000.0,
        "reference_area_m2": 0.12,
        "reference_length_m": 0.25,
    }


def render_su2_config(inputs: dict[str, Any]) -> str:
    freestream = inputs["freestream"]
    lines = [
        "% Aircraft Optimizer Platform SU2 dry-run case",
        "% Generated for traceability only; solver execution is disabled.",
        "SOLVER= EULER",
        "KIND_TURB_MODEL= NONE",
        f"MACH_NUMBER= {freestream['mach']}",
        f"AOA= {freestream['aoa_deg']}",
        f"REYNOLDS_NUMBER= {freestream['reynolds_number']}",
        f"REF_AREA= {freestream['reference_area_m2']}",
        f"REF_LENGTH= {freestream['reference_length_m']}",
        f"MESH_FILENAME= {inputs['mesh_reference_path']}",
        "CONV_FILENAME= history",
        "VOLUME_FILENAME= flow",
        "SURFACE_FILENAME= surface_flow",
    ]
    return "\n".join(lines) + "\n"
