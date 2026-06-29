from __future__ import annotations

import math

from aircraft_optimizer.records import CandidateSeed, MetricValue

MODULE_NAME = "low_fidelity_fixed_wing"
MODULE_VERSION = "0.2.0"

AIRFOIL_OPTIONS = {
    0: {
        "name": "thin_loiter",
        "cl_factor": 0.97,
        "cd0_delta": -0.0025,
        "stability_bonus": -0.02,
    },
    1: {
        "name": "balanced_uav",
        "cl_factor": 1.0,
        "cd0_delta": 0.0,
        "stability_bonus": 0.0,
    },
    2: {
        "name": "high_lift_stable",
        "cl_factor": 1.07,
        "cd0_delta": 0.0035,
        "stability_bonus": 0.08,
    },
}


def low_fidelity_fixed_wing_metrics(
    candidate: CandidateSeed,
) -> dict[str, dict[str, object]]:
    variables = candidate.design_variables
    span_m = float(variables["wing.span_mm"].value) / 1000.0
    root_chord_m = float(variables["wing.root_chord_mm"].value) / 1000.0
    tip_chord_m = float(variables["wing.tip_chord_mm"].value) / 1000.0
    sweep_deg = float(variables["wing.sweep_deg"].value)
    airfoil_index = _airfoil_index(_optional_variable(candidate, "wing.airfoil_selector", 1.0))
    airfoil = AIRFOIL_OPTIONS[airfoil_index]
    fuselage_length_delta_mm = _optional_variable(candidate, "fuselage.length_delta_mm", 0.0)
    nose_bluntness = _clamp(_optional_variable(candidate, "fuselage.nose_bluntness", 0.45), 0.0, 1.0)
    tail_bluntness = _clamp(_optional_variable(candidate, "fuselage.tail_bluntness", 0.65), 0.0, 1.0)

    area_m2 = span_m * (root_chord_m + tip_chord_m) / 2.0
    aspect_ratio = span_m * span_m / area_m2
    taper_ratio = tip_chord_m / root_chord_m

    efficiency = _oswald_efficiency(aspect_ratio, sweep_deg, taper_ratio)
    cruise_cl = 0.55 * float(airfoil["cl_factor"])
    fuselage_drag_delta = 0.000012 * abs(fuselage_length_delta_mm)
    nose_drag_delta = 0.008 * max(nose_bluntness - 0.45, 0.0) + 0.002 * max(0.20 - nose_bluntness, 0.0)
    tail_drag_delta = 0.010 * abs(tail_bluntness - 0.62)
    cd0 = (
        0.035
        + 0.0007 * abs(sweep_deg)
        + 0.010 * abs(taper_ratio - 0.45)
        + float(airfoil["cd0_delta"])
        + fuselage_drag_delta
        + nose_drag_delta
        + tail_drag_delta
    )
    induced_drag = cruise_cl * cruise_cl / (math.pi * efficiency * aspect_ratio)
    cd_total = cd0 + induced_drag
    lift_to_drag = cruise_cl / cd_total

    # Keep this intentionally modest: it is a screening score, not aircraft truth.
    static_layout_proxy = (
        0.15 * min(abs(fuselage_length_delta_mm) / 100.0, 1.0)
        + float(airfoil["stability_bonus"])
        - 0.20 * max(0.35 - tail_bluntness, 0.0)
    )
    manufacturability_proxy = -0.12 * max(0.12 - tail_bluntness, 0.0)
    low_fidelity_score = (
        lift_to_drag
        - 0.02 * abs(sweep_deg - 10.0)
        + static_layout_proxy
        + manufacturability_proxy
    )
    return {
        "geometry.wing_area_m2": _metric(area_m2, "m^2"),
        "geometry.aspect_ratio": _metric(aspect_ratio, None),
        "geometry.taper_ratio": _metric(taper_ratio, None),
        "geometry.airfoil_selector": _metric(float(airfoil_index), None),
        "geometry.fuselage_length_delta_mm": _metric(fuselage_length_delta_mm, "mm"),
        "geometry.nose_bluntness": _metric(nose_bluntness, None),
        "geometry.tail_bluntness": _metric(tail_bluntness, None),
        "aero.oswald_efficiency_estimate": _metric(efficiency, None),
        "aero.cd0_estimate": _metric(cd0, None),
        "aero.induced_drag_estimate": _metric(induced_drag, None),
        "aero.lift_to_drag_estimate": _metric(lift_to_drag, None),
        "score.static_layout_proxy": _metric(static_layout_proxy, None),
        "score.manufacturability_proxy": _metric(manufacturability_proxy, None),
        "score.low_fidelity_total": _metric(low_fidelity_score, None),
    }


def _oswald_efficiency(
    aspect_ratio: float,
    sweep_deg: float,
    taper_ratio: float,
) -> float:
    sweep_penalty = min(abs(sweep_deg) / 60.0, 0.35)
    taper_penalty = min(abs(taper_ratio - 0.45), 0.25)
    ar_penalty = 0.04 if aspect_ratio > 9.0 else 0.0
    return max(0.55, 0.82 - sweep_penalty - 0.20 * taper_penalty - ar_penalty)


def _metric(value: float, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6),
        unit=unit,
        confidence=0.35,
        source=MODULE_NAME,
    ).to_dict()


def _optional_variable(
    candidate: CandidateSeed,
    variable_name: str,
    default: float,
) -> float:
    variable = candidate.design_variables.get(variable_name)
    if variable is None:
        return default
    return float(variable.value)


def _airfoil_index(value: float) -> int:
    return int(_clamp(round(value), 0, 2))


def _clamp(value: float | int, minimum: float | int, maximum: float | int) -> float:
    return float(max(minimum, min(maximum, value)))
