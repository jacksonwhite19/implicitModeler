from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.modules.virtual_components import estimate_virtual_mass_properties
from aircraft_optimizer.records import CandidateSeed, MetricValue

MODULE_NAME = "geometry_definition_screening"
MODULE_VERSION = "0.1.0"


DEFAULT_GEOMETRY_DEFINITION_POLICY = {
    "required_feature_name": "aircraft_oml_native_mc",
    "max_bbox_length_mm": 1200.0,
    "max_bbox_width_mm": 1100.0,
    "max_bbox_height_mm": 450.0,
    "max_estimated_mesh_cells": 60000000,
    "max_narrow_feature_risk": 0.7,
    "min_internal_layout_wiggle_room_mm": 20.0,
}


@dataclass(frozen=True)
class GeometryDefinitionScreeningResult:
    passed: bool
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


def screen_geometry_definition(
    candidate: CandidateSeed,
    *,
    feature_name: str,
    bbox_min_mm: list[float],
    bbox_max_mm: list[float],
    policy: dict[str, Any] | None = None,
    virtual_component_assumptions: dict[str, Any] | None = None,
) -> GeometryDefinitionScreeningResult:
    active_policy = {**DEFAULT_GEOMETRY_DEFINITION_POLICY, **(policy or {})}
    bbox_length = float(bbox_max_mm[0]) - float(bbox_min_mm[0])
    bbox_width = float(bbox_max_mm[1]) - float(bbox_min_mm[1])
    bbox_height = float(bbox_max_mm[2]) - float(bbox_min_mm[2])
    mass_properties = estimate_virtual_mass_properties(
        candidate,
        assumptions=virtual_component_assumptions,
    )
    estimated_mesh_cells = _estimated_mesh_cells(bbox_length, bbox_width, bbox_height)
    narrow_feature_risk = _narrow_feature_risk(candidate)
    checks = {
        "required_feature_name": feature_name == active_policy["required_feature_name"],
        "max_bbox_length_mm": bbox_length <= float(active_policy["max_bbox_length_mm"]),
        "max_bbox_width_mm": bbox_width <= float(active_policy["max_bbox_width_mm"]),
        "max_bbox_height_mm": bbox_height <= float(active_policy["max_bbox_height_mm"]),
        "max_estimated_mesh_cells": estimated_mesh_cells <= int(active_policy["max_estimated_mesh_cells"]),
        "max_narrow_feature_risk": narrow_feature_risk <= float(active_policy["max_narrow_feature_risk"]),
        "internal_layout_feasible": mass_properties.layout_feasible,
        "min_internal_layout_wiggle_room_mm": mass_properties.layout_wiggle_room_mm >= float(active_policy["min_internal_layout_wiggle_room_mm"]),
    }
    failed_checks = [name for name, passed in checks.items() if not passed]
    metrics = {
        "geometry_definition.bbox_length_mm": _metric(bbox_length, "mm"),
        "geometry_definition.bbox_width_mm": _metric(bbox_width, "mm"),
        "geometry_definition.bbox_height_mm": _metric(bbox_height, "mm"),
        "geometry_definition.estimated_mesh_cells": _metric(estimated_mesh_cells, "cells"),
        "geometry_definition.narrow_feature_risk": _metric(narrow_feature_risk, None),
        "geometry_definition.failed_check_count": _metric(len(failed_checks), None),
    }
    return GeometryDefinitionScreeningResult(
        passed=not failed_checks,
        metrics=metrics,
        metadata={
            "module_name": MODULE_NAME,
            "module_version": MODULE_VERSION,
            "policy": active_policy,
            "checks": checks,
            "failed_checks": failed_checks,
            "feature_name": feature_name,
            "bbox_min_mm": bbox_min_mm,
            "bbox_max_mm": bbox_max_mm,
            "virtual_layout": mass_properties.metadata()["layout"],
        },
        warnings=[f"failed geometry-definition check: {check}" for check in failed_checks],
    )


def _estimated_mesh_cells(length_mm: float, width_mm: float, height_mm: float) -> int:
    volume_mm3 = max(0.0, length_mm * width_mm * height_mm)
    return int(round(volume_mm3 / (2.0 ** 3)))


def _narrow_feature_risk(candidate: CandidateSeed) -> float:
    variables = candidate.design_variables
    root_chord = float(variables["wing.root_chord_mm"].value)
    tip_chord = float(variables["wing.tip_chord_mm"].value)
    sweep = abs(float(variables["wing.sweep_deg"].value))
    taper_ratio = tip_chord / root_chord
    taper_penalty = max(0.0, (0.38 - taper_ratio) / 0.25)
    tip_penalty = max(0.0, (55.0 - tip_chord) / 35.0)
    sweep_penalty = max(0.0, (sweep - 22.0) / 18.0)
    return round(min(1.0, 0.45 * taper_penalty + 0.35 * tip_penalty + 0.2 * sweep_penalty), 6)


def _metric(value: float | int, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6) if isinstance(value, float) else value,
        unit=unit,
        confidence=0.3,
        source=MODULE_NAME,
    ).to_dict()
