from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.artifacts.registry import register_artifact
from aircraft_optimizer.db.repositories import create_geometry_provider_result
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.geometry.fixture_generator import (
    generate_fixed_wing_fixture_geometry,
)
from aircraft_optimizer.geometry.manual_reference_provider import default_no_inlet_reference
from aircraft_optimizer.records import CandidateSeed


@dataclass(frozen=True)
class FixtureGeometryResult:
    geometry_provider_result_id: str
    geometry_artifact_id: str
    trace_artifact_id: str
    feature_name: str
    bbox_min_mm: list[float]
    bbox_max_mm: list[float]


def persist_fixture_geometry(
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
    request_message: str,
    completed_message: str,
) -> FixtureGeometryResult:
    log_event(
        connection,
        event_type="geometry.requested",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        actor="module",
        message=request_message,
    )
    reference_geometry = default_no_inlet_reference(platform_root)
    geometry = generate_fixed_wing_fixture_geometry(
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
        source_path=geometry.script_path,
        artifact_type="geometry_source_rhai",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=None,
        producer_module="fixed_wing_uav_fixture_generator",
        producer_version="0.1.0",
        source_kind="generated_fixture",
        source_original_path=str(reference_geometry.script_path),
        metadata={
            "feature_name": geometry.feature_name,
            "coordinate_frame": geometry.coordinate_frame,
            "trace_path": str(geometry.trace_path),
        },
    )
    trace_artifact_id = register_artifact(
        connection,
        artifact_root=artifact_root,
        source_path=geometry.trace_path,
        artifact_type="geometry_parameter_trace_json",
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=None,
        producer_module="fixed_wing_uav_fixture_generator",
        producer_version="0.1.0",
        source_kind="generated_fixture",
        source_original_path=str(geometry.trace_path),
        metadata={
            "geometry_source_artifact_id": geometry_artifact_id,
        },
    )
    geometry_provider_result_id = create_geometry_provider_result(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        provider_name="fixed_wing_uav_fixture_generator",
        provider_version="0.1.0",
        provider_kind="automatic_fixture",
        representation="rhai_sdf",
        script_path=str(geometry.script_path),
        feature_name=geometry.feature_name,
        coordinate_frame=geometry.coordinate_frame,
        bbox_min_mm=geometry.bbox_min_mm,
        bbox_max_mm=geometry.bbox_max_mm,
        source_hash=geometry.source_hash,
        parameter_trace={
            "aircraft_family": candidate.aircraft_family,
            "variable_schema_id": variable_schema_id,
            "design_variables": candidate.design_variables_dict(),
        },
        metadata={
            **geometry.metadata,
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
        message=completed_message,
        payload={"geometry_provider_result_id": geometry_provider_result_id},
    )
    for artifact_id, message in [
        (geometry_artifact_id, "Registered geometry source artifact."),
        (trace_artifact_id, "Registered geometry parameter trace artifact."),
    ]:
        log_event(
            connection,
            event_type="artifact.created",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            actor="module",
            message=message,
            payload={"artifact_id": artifact_id},
        )
    return FixtureGeometryResult(
        geometry_provider_result_id=geometry_provider_result_id,
        geometry_artifact_id=geometry_artifact_id,
        trace_artifact_id=trace_artifact_id,
        feature_name=geometry.feature_name,
        bbox_min_mm=geometry.bbox_min_mm,
        bbox_max_mm=geometry.bbox_max_mm,
    )
