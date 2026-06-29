from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.db.repositories import (
    complete_module_attempt,
    create_failure,
    create_module_attempt,
    fail_module_attempt,
)
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.records import CandidateSeed, FailureDraft, MetricValue
from aircraft_optimizer.modules.virtual_components import estimate_virtual_mass_properties

MODULE_NAME = "pre_export_screening"
MODULE_VERSION = "0.1.0"

BASE_FUSELAGE_LENGTH_MM = 700.0
HTAIL_LE_X_MM = 500.0
HTAIL_ROOT_CHORD_MM = 98.0
HTAIL_TIP_CHORD_MM = 80.0
VTAIL_LE_X_MM = 500.0
VTAIL_ROOT_CHORD_MM = 160.0
VTAIL_TIP_CHORD_MM = 66.0
VTAIL_SPAN_MM = 250.0
VTAIL_SWEEP_TAN = 0.70020753821
TAIL_END_CLEARANCE_MM = 25.0


@dataclass(frozen=True)
class ScreeningResult:
    passed: bool
    metrics: dict[str, dict[str, object]]
    warnings: list[str]
    failure: FailureDraft | None
    metadata: dict[str, Any]


DEFAULT_SCREENING_POLICY = {
    "estimated_mass_kg": 1.2,
    "min_planform_area_m2": 0.045,
    "max_wing_loading_n_m2": 260.0,
    "min_aspect_ratio": 4.0,
    "max_aspect_ratio": 13.5,
    "min_taper_ratio": 0.25,
    "max_taper_ratio": 0.85,
    "max_abs_sweep_deg": 32.0,
    "min_estimated_total_mass_kg": 0.75,
    "max_estimated_total_mass_kg": 2.1,
    "require_internal_layout_feasible": True,
    "max_layout_cg_error_mm": 55.0,
    "min_layout_wiggle_room_mm": 20.0,
    "min_cg_x_mm": 220.0,
    "max_cg_x_mm": 430.0,
    "max_abs_cg_y_mm": 8.0,
    "min_cg_z_mm": -40.0,
    "max_cg_z_mm": 45.0,
    "min_mean_chord_mm": 80.0,
    "max_mean_chord_mm": 210.0,
    "min_tip_chord_mm": 45.0,
    "max_span_root_chord_ratio": 7.0,
    "max_export_risk_score": 0.72,
    "min_tail_fuselage_end_clearance_mm": 20.0,
}


def screen_candidate_pre_export(
    candidate: CandidateSeed,
    policy: dict[str, Any] | None = None,
) -> ScreeningResult:
    active_policy = {**DEFAULT_SCREENING_POLICY, **(policy or {})}
    variables = candidate.design_variables
    span_mm = float(variables["wing.span_mm"].value)
    root_chord_mm = float(variables["wing.root_chord_mm"].value)
    tip_chord_mm = float(variables["wing.tip_chord_mm"].value)
    sweep_deg = float(variables["wing.sweep_deg"].value)

    span_m = span_mm / 1000.0
    mean_chord_m = ((root_chord_mm + tip_chord_mm) / 2.0) / 1000.0
    planform_area_m2 = span_m * mean_chord_m
    aspect_ratio = (span_m * span_m) / planform_area_m2
    taper_ratio = tip_chord_mm / root_chord_mm
    span_root_chord_ratio = span_mm / root_chord_mm
    export_risk_score = _export_risk_score(
        aspect_ratio=aspect_ratio,
        taper_ratio=taper_ratio,
        sweep_deg=sweep_deg,
        tip_chord_mm=tip_chord_mm,
        planform_area_m2=planform_area_m2,
    )
    mass_properties = estimate_virtual_mass_properties(
        candidate,
        assumptions=active_policy.get("virtual_component_assumptions"),
    )
    estimated_mass_kg = mass_properties.total_mass_kg
    wing_loading_n_m2 = (estimated_mass_kg * 9.80665) / planform_area_m2
    tail_clearance = _tail_fuselage_clearance_mm(candidate)

    checks = {
        "min_planform_area_m2": planform_area_m2 >= float(active_policy["min_planform_area_m2"]),
        "max_wing_loading_n_m2": wing_loading_n_m2 <= float(active_policy["max_wing_loading_n_m2"]),
        "min_aspect_ratio": aspect_ratio >= float(active_policy["min_aspect_ratio"]),
        "max_aspect_ratio": aspect_ratio <= float(active_policy["max_aspect_ratio"]),
        "min_taper_ratio": taper_ratio >= float(active_policy["min_taper_ratio"]),
        "max_taper_ratio": taper_ratio <= float(active_policy["max_taper_ratio"]),
        "max_abs_sweep_deg": abs(sweep_deg) <= float(active_policy["max_abs_sweep_deg"]),
        "min_estimated_total_mass_kg": mass_properties.total_mass_kg >= float(active_policy["min_estimated_total_mass_kg"]),
        "max_estimated_total_mass_kg": mass_properties.total_mass_kg <= float(active_policy["max_estimated_total_mass_kg"]),
        "internal_layout_feasible": mass_properties.layout_feasible or not bool(active_policy["require_internal_layout_feasible"]),
        "max_layout_cg_error_mm": abs(mass_properties.cg_error_mm) <= float(active_policy["max_layout_cg_error_mm"]),
        "min_layout_wiggle_room_mm": mass_properties.layout_wiggle_room_mm >= float(active_policy["min_layout_wiggle_room_mm"]),
        "min_cg_x_mm": mass_properties.cg_mm[0] >= float(active_policy["min_cg_x_mm"]),
        "max_cg_x_mm": mass_properties.cg_mm[0] <= float(active_policy["max_cg_x_mm"]),
        "max_abs_cg_y_mm": abs(mass_properties.cg_mm[1]) <= float(active_policy["max_abs_cg_y_mm"]),
        "min_cg_z_mm": mass_properties.cg_mm[2] >= float(active_policy["min_cg_z_mm"]),
        "max_cg_z_mm": mass_properties.cg_mm[2] <= float(active_policy["max_cg_z_mm"]),
        "min_mean_chord_mm": mean_chord_m * 1000.0 >= float(active_policy["min_mean_chord_mm"]),
        "max_mean_chord_mm": mean_chord_m * 1000.0 <= float(active_policy["max_mean_chord_mm"]),
        "min_tip_chord_mm": tip_chord_mm >= float(active_policy["min_tip_chord_mm"]),
        "max_span_root_chord_ratio": span_root_chord_ratio <= float(active_policy["max_span_root_chord_ratio"]),
        "max_export_risk_score": export_risk_score <= float(active_policy["max_export_risk_score"]),
        "min_tail_fuselage_end_clearance_mm": tail_clearance["clearance_mm"]
        >= float(active_policy["min_tail_fuselage_end_clearance_mm"]),
    }
    failed_checks = [name for name, passed in checks.items() if not passed]
    metrics = {
        "pre_export.planform_area_m2": _metric(planform_area_m2, "m^2"),
        "pre_export.aspect_ratio": _metric(aspect_ratio, None),
        "pre_export.taper_ratio": _metric(taper_ratio, None),
        "pre_export.estimated_mass_kg": _metric(estimated_mass_kg, "kg"),
        "pre_export.wing_loading_n_m2": _metric(wing_loading_n_m2, "N/m^2"),
        "pre_export.mean_chord_mm": _metric(mean_chord_m * 1000.0, "mm"),
        "pre_export.span_root_chord_ratio": _metric(span_root_chord_ratio, None),
        "pre_export.export_risk_score": _metric(export_risk_score, None),
        "pre_export.fuselage_length_mm": _metric(tail_clearance["fuselage_length_mm"], "mm"),
        "pre_export.tail_le_x_mm": _metric(tail_clearance["tail_le_x_mm"], "mm"),
        "pre_export.tail_aft_x_mm": _metric(tail_clearance["tail_aft_x_mm"], "mm"),
        "pre_export.tail_fuselage_end_clearance_mm": _metric(tail_clearance["clearance_mm"], "mm"),
        "pre_export.failed_check_count": _metric(len(failed_checks), None),
    }
    metrics.update(mass_properties.metrics())
    component_metadata = mass_properties.metadata()
    warnings = [f"failed pre-export check: {check}" for check in failed_checks]
    if failed_checks:
        return ScreeningResult(
            passed=False,
            metrics=metrics,
            warnings=warnings,
            failure=FailureDraft(
                category="optimization.pre_export_screen_failed",
                stage="pre_export_screening",
                severity="recoverable",
                retryable=False,
                message="Candidate failed cheap pre-export screening.",
                suggested_next_action="Record result and let optimizer propose the next candidate.",
                metadata={
                    "failed_checks": failed_checks,
                    "policy": active_policy,
                    "virtual_components": component_metadata["components"],
                },
            ),
            metadata={
                "screening_only": True,
                "policy": active_policy,
                "checks": checks,
                "failed_checks": failed_checks,
                "virtual_component_model": component_metadata,
            },
        )
    return ScreeningResult(
        passed=True,
        metrics=metrics,
        warnings=warnings,
        failure=None,
        metadata={
            "screening_only": True,
            "policy": active_policy,
            "checks": checks,
            "failed_checks": [],
            "virtual_component_model": component_metadata,
        },
    )


def persist_pre_export_screening(
    connection: sqlite3.Connection,
    *,
    campaign_id: str,
    candidate_id: str,
    evaluation_id: str,
    candidate: CandidateSeed,
    policy: dict[str, Any] | None = None,
) -> tuple[str, ScreeningResult, str | None]:
    result = screen_candidate_pre_export(candidate, policy=policy)
    module_attempt_id = create_module_attempt(
        connection,
        evaluation_id=evaluation_id,
        candidate_id=candidate_id,
        module_name=MODULE_NAME,
        module_version=MODULE_VERSION,
        module_kind="analysis",
        inputs={
            "candidate_variables": candidate.design_variables_dict(),
            "policy": result.metadata["policy"],
        },
    )
    log_event(
        connection,
        event_type="module.started",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        message="Started pre-export screening module.",
    )
    if result.passed:
        complete_module_attempt(
            connection,
            module_attempt_id=module_attempt_id,
            metrics=result.metrics,
            warnings=result.warnings,
            metadata=result.metadata,
        )
        log_event(
            connection,
            event_type="module.completed",
            campaign_id=campaign_id,
            candidate_id=candidate_id,
            evaluation_id=evaluation_id,
            module_attempt_id=module_attempt_id,
            actor="module",
            message="Completed pre-export screening module.",
        )
        return module_attempt_id, result, None

    if result.failure is None:
        raise ValueError("failed screening result must include a failure draft")
    failure_id = create_failure(
        connection,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        failure=result.failure,
    )
    fail_module_attempt(
        connection,
        module_attempt_id=module_attempt_id,
        failure_id=failure_id,
        metrics=result.metrics,
        warnings=result.warnings,
        metadata=result.metadata,
    )
    log_event(
        connection,
        event_type="module.failed",
        campaign_id=campaign_id,
        candidate_id=candidate_id,
        evaluation_id=evaluation_id,
        module_attempt_id=module_attempt_id,
        actor="module",
        severity="warning",
        message="Failed pre-export screening module.",
        payload={"failure_id": failure_id},
    )
    return module_attempt_id, result, failure_id

def _export_risk_score(
    *,
    aspect_ratio: float,
    taper_ratio: float,
    sweep_deg: float,
    tip_chord_mm: float,
    planform_area_m2: float,
) -> float:
    aspect_penalty = max(0.0, (aspect_ratio - 10.0) / 6.0)
    taper_penalty = max(0.0, (0.35 - taper_ratio) / 0.2)
    sweep_penalty = max(0.0, (abs(sweep_deg) - 20.0) / 20.0)
    tip_penalty = max(0.0, (55.0 - tip_chord_mm) / 30.0)
    area_penalty = max(0.0, (0.06 - planform_area_m2) / 0.04)
    return round(min(1.0, 0.25 * aspect_penalty + 0.25 * taper_penalty + 0.2 * sweep_penalty + 0.2 * tip_penalty + 0.1 * area_penalty), 6)


def _tail_fuselage_clearance_mm(candidate: CandidateSeed) -> dict[str, float]:
    length_delta = candidate.design_variables.get("fuselage.length_delta_mm")
    fuselage_length_mm = BASE_FUSELAGE_LENGTH_MM + (float(length_delta.value) if length_delta is not None else 0.0)
    tail_le_x = _tail_le_x_for_fuselage(fuselage_length_mm)
    htail_aft_x = max(
        tail_le_x + HTAIL_ROOT_CHORD_MM,
        tail_le_x + HTAIL_TIP_CHORD_MM,
    )
    vtail_tip_le_x = tail_le_x + VTAIL_SWEEP_TAN * (VTAIL_SPAN_MM * 0.5)
    vtail_aft_x = max(
        tail_le_x + VTAIL_ROOT_CHORD_MM,
        vtail_tip_le_x + VTAIL_TIP_CHORD_MM,
    )
    tail_aft_x = max(htail_aft_x, vtail_aft_x)
    return {
        "fuselage_length_mm": round(fuselage_length_mm, 6),
        "tail_le_x_mm": round(tail_le_x, 6),
        "tail_aft_x_mm": round(tail_aft_x, 6),
        "clearance_mm": round(fuselage_length_mm - tail_aft_x, 6),
    }


def _tail_le_x_for_fuselage(fuselage_length_mm: float) -> float:
    htail_aft_offset = max(HTAIL_ROOT_CHORD_MM, HTAIL_TIP_CHORD_MM)
    vtail_aft_offset = max(
        VTAIL_ROOT_CHORD_MM,
        VTAIL_SWEEP_TAN * (VTAIL_SPAN_MM * 0.5) + VTAIL_TIP_CHORD_MM,
    )
    max_tail_aft_offset = max(htail_aft_offset, vtail_aft_offset)
    max_allowed_le_x = fuselage_length_mm - TAIL_END_CLEARANCE_MM - max_tail_aft_offset
    return round(min(HTAIL_LE_X_MM, VTAIL_LE_X_MM, max_allowed_le_x), 6)


def _metric(value: float | int, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6) if isinstance(value, float) else value,
        unit=unit,
        confidence=0.3,
        source=MODULE_NAME,
    ).to_dict()
