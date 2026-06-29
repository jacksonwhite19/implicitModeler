from __future__ import annotations

from typing import Any

from aircraft_optimizer.records import MetricValue


MODULE_NAME = "rough_cfd_scoring"
MODULE_VERSION = "0.2.1"

DEFAULT_STABLE_EFFICIENT_DRONE_SCORING_CONFIG = {
    "id": "rough_cfd_stable_efficient_drone_v0_1",
    "version": "0.1",
    "goal": "Favor stable, efficient small fixed-wing UAV candidates using rough CFD ranking evidence.",
    "score_scale": 100.0,
    "weights": {
        "efficiency_l_over_d": 0.22,
        "mission_l_over_d": 0.24,
        "curve_consistency": 0.14,
        "low_drag": 0.14,
        "usable_lift": 0.10,
        "pitch_moment_trim": 0.03,
        "solver_confidence": 0.06,
        "mesh_confidence": 0.02,
    },
    "targets": {
        "ld_reference": 1.0,
        "cd_reference": 0.10,
        "cl_target": 0.09,
        "cl_min": 0.03,
        "cl_max": 0.18,
        "abs_cm_target": 0.08,
        "abs_cm_limit": 0.65,
        "surface_p95_reference_mm": 0.25,
        "surface_p99_reference_mm": 2.5,
        "polar_ld_reference": 4.0,
        "polar_cl_target": 0.4,
        "polar_cl_min": 0.05,
        "polar_cl_max": 0.8,
        "mission_speed_mps": 22.352,
        "air_density_kg_m3": 1.225,
        "level_flight_cl_margin": 0.05,
        "preferred_alpha_deg": 6.0,
        "max_preferred_alpha_deg": 10.0,
        "average_ld_reference": 3.5,
        "ld_curve_consistency_reference": 0.7,
    },
    "gates": {
        "require_rough_scoring_allowed": True,
        "require_steady_ready": True,
        "require_positive_lift": True,
        "max_abs_coefficient": 10.0,
    },
    "negative_detractors": {
        "high_abs_cm": {
            "weight": 3.0,
            "start": 0.35,
            "limit": 0.9,
        },
        "lift_outside_band": {
            "weight": 5.0,
        },
        "coefficient_instability": {
            "weight": 6.0,
            "cd_span_reference": 0.01,
            "cl_span_reference": 0.01,
            "cm_span_reference": 0.01,
        },
        "surface_deviation": {
            "weight": 4.0,
        },
        "solver_residual": {
            "weight": 4.0,
            "velocity_residual_reference": 0.001,
            "local_continuity_reference": 0.0001,
        },
        "high_alpha_required": {
            "weight": 10.0,
            "start_deg": 8.0,
            "limit_deg": 14.0,
        },
        "level_flight_lift_shortfall": {
            "weight": 14.0,
        },
        "weak_ld_curve_consistency": {
            "weight": 8.0,
            "target": 0.7,
        },
    },
}


def stable_efficient_drone_rough_score(
    *,
    steady_metrics: dict[str, Any],
    steady_metadata: dict[str, Any] | None = None,
    surface_metrics: dict[str, Any] | None = None,
    alpha_sweep: dict[str, Any] | None = None,
    config: dict[str, Any] | None = None,
) -> tuple[dict[str, dict[str, object]], dict[str, Any], list[str]]:
    active_config = _merge_config(
        DEFAULT_STABLE_EFFICIENT_DRONE_SCORING_CONFIG,
        config or {},
    )
    steady_metadata = steady_metadata or {}
    surface_metrics = surface_metrics or {}
    weights = active_config["weights"]
    targets = active_config["targets"]
    gates = active_config["gates"]
    detractor_config = active_config.get("negative_detractors", {})

    cd = _metric_number(steady_metrics, "openfoam_steady.cd_last")
    cl = _metric_number(steady_metrics, "openfoam_steady.cl_last")
    cm = _metric_number(steady_metrics, "openfoam_steady.cm_last")
    alpha_sweep = alpha_sweep or {}
    first_pass_screening = _is_first_pass_screening(alpha_sweep)
    polar_point = _polar_scoring_point(
        alpha_sweep,
        abs_cm_limit=float(targets["abs_cm_limit"]),
        screening_tolerant=first_pass_screening,
    )
    curve_metrics = _polar_curve_metrics(alpha_sweep)
    required_cl = _required_lift_coefficient(
        steady_metrics=steady_metrics,
        targets=targets,
    )
    mission_point = (
        _interpolated_point_at_cl(alpha_sweep, required_cl)
        if required_cl is not None
        else None
    )
    alpha_sweep_used = polar_point is not None
    if polar_point is not None:
        cd = float(polar_point["cd"])
        cl = float(polar_point["cl"])
        cm = float(polar_point["cm"])
    ready = _metric_number(steady_metrics, "openfoam_steady.ready")
    rough_allowed = _metric_number(
        steady_metrics,
        "openfoam_steady.rough_scoring_allowed",
    )
    failed_check_count = _metric_number(
        steady_metrics,
        "openfoam_steady.failed_check_count",
    )
    cd_span = _metric_number(steady_metrics, "openfoam_steady.cd_window_span")
    cl_span = _metric_number(steady_metrics, "openfoam_steady.cl_window_span")
    cm_span = _metric_number(steady_metrics, "openfoam_steady.cm_window_span")
    velocity_residual = _metric_number(
        steady_metrics,
        "openfoam_steady.velocity_final_residual",
    )
    local_continuity = _metric_number(
        steady_metrics,
        "openfoam_steady.local_continuity",
    )
    yplus_p95 = _metric_number(steady_metrics, "openfoam_steady.yplus_p95")
    surface_ready = _metric_number(surface_metrics, "snappy_surface.ready")
    surface_p95 = _metric_number(
        surface_metrics,
        "snappy_surface.bidirectional_p95_mm",
    )
    surface_p99 = _metric_number(
        surface_metrics,
        "snappy_surface.bidirectional_p99_mm",
    )

    warnings = _gate_warnings(
        cd=cd,
        cl=cl,
        cm=cm,
        ready=ready,
        rough_allowed=rough_allowed,
        gates=gates,
    )

    if cd is None or cl is None or cm is None or cd <= 0.0:
        components = _zero_components()
        total = 0.0
        warnings.append("missing_or_invalid_force_coefficients")
    else:
        ld = cl / cd
        components = {
            "efficiency_l_over_d": _clamp(ld / float(targets["ld_reference"]), 0.0, 1.5),
            "mission_l_over_d": _mission_ld_score(
                mission_point=mission_point,
                reference=float(targets["polar_ld_reference"]),
            ),
            "curve_consistency": _curve_consistency_score(
                curve_metrics=curve_metrics,
                average_reference=float(targets["average_ld_reference"]),
                consistency_reference=float(targets["ld_curve_consistency_reference"]),
            ),
            "low_drag": _clamp(float(targets["cd_reference"]) / cd, 0.0, 1.5),
            "usable_lift": _usable_lift_score(
                cl=cl,
                cl_min=float(
                    targets["polar_cl_min"] if alpha_sweep_used else targets["cl_min"]
                ),
                cl_target=float(
                    targets["polar_cl_target"] if alpha_sweep_used else targets["cl_target"]
                ),
                cl_max=float(
                    targets["polar_cl_max"] if alpha_sweep_used else targets["cl_max"]
                ),
            ),
            "pitch_moment_trim": _pitch_moment_score(
                cm=cm,
                target=float(targets["abs_cm_target"]),
                limit=float(targets["abs_cm_limit"]),
            ),
            "solver_confidence": _solver_confidence_score(
                failed_check_count=failed_check_count,
                cd_span=cd_span,
                cl_span=cl_span,
                cm_span=cm_span,
                velocity_residual=velocity_residual,
                local_continuity=local_continuity,
                yplus_p95=yplus_p95,
            ),
            "mesh_confidence": _mesh_confidence_score(
                surface_ready=surface_ready,
                surface_p95=surface_p95,
                surface_p99=surface_p99,
                p95_reference=float(targets["surface_p95_reference_mm"]),
                p99_reference=float(targets["surface_p99_reference_mm"]),
            ),
        }
        if alpha_sweep_used:
            components["efficiency_l_over_d"] = _clamp(
                ld / float(targets["polar_ld_reference"]),
                0.0,
                1.5,
            )
        weighted = sum(
            float(weights[name]) * components[name]
            for name in weights
        )
        positive_total = float(active_config["score_scale"]) * weighted / sum(
            float(value) for value in weights.values()
        )
        detractors = _negative_detractors(
            cl=cl,
            cm=cm,
            cl_min=float(targets["polar_cl_min"] if alpha_sweep_used else targets["cl_min"]),
            cl_max=float(targets["polar_cl_max"] if alpha_sweep_used else targets["cl_max"]),
            cd_span=cd_span,
            cl_span=cl_span,
            cm_span=cm_span,
            velocity_residual=velocity_residual,
            local_continuity=local_continuity,
            surface_p95=surface_p95,
            surface_p99=surface_p99,
            surface_p95_reference=float(targets["surface_p95_reference_mm"]),
            surface_p99_reference=float(targets["surface_p99_reference_mm"]),
            config=detractor_config,
        )
        detractors.update(
            _mission_curve_detractors(
                required_cl=required_cl,
                mission_point=mission_point,
                curve_metrics=curve_metrics,
                alpha_sweep=alpha_sweep,
                config=detractor_config,
            )
        )
        total = max(0.0, positive_total - sum(detractors.values()))
    if "detractors" not in locals():
        detractors = _zero_detractors()

    usable = not warnings
    confidence = _score_confidence(components, usable=usable)
    metrics = {
        "score.rough_total": _metric(total, None, confidence),
        "score.rough_efficiency_l_over_d_component": _metric(
            components["efficiency_l_over_d"],
            None,
            confidence,
        ),
        "score.rough_low_drag_component": _metric(
            components["low_drag"],
            None,
            confidence,
        ),
        "score.rough_mission_l_over_d_component": _metric(
            components.get("mission_l_over_d", 0.0),
            None,
            confidence,
        ),
        "score.rough_curve_consistency_component": _metric(
            components.get("curve_consistency", 0.0),
            None,
            confidence,
        ),
        "score.rough_usable_lift_component": _metric(
            components["usable_lift"],
            None,
            confidence,
        ),
        "score.rough_pitch_moment_trim_component": _metric(
            components["pitch_moment_trim"],
            None,
            confidence,
        ),
        "score.rough_solver_confidence_component": _metric(
            components["solver_confidence"],
            None,
            confidence,
        ),
        "score.rough_mesh_confidence_component": _metric(
            components["mesh_confidence"],
            None,
            confidence,
        ),
        "score.rough_positive_total_before_detractors": _metric(
            total + sum(detractors.values()),
            None,
            confidence,
        ),
        "score.rough_detractor_total": _metric(
            sum(detractors.values()),
            None,
            confidence,
        ),
        "score.rough_high_abs_cm_detractor": _metric(
            detractors["high_abs_cm"],
            None,
            confidence,
        ),
        "score.rough_lift_outside_band_detractor": _metric(
            detractors["lift_outside_band"],
            None,
            confidence,
        ),
        "score.rough_coefficient_instability_detractor": _metric(
            detractors["coefficient_instability"],
            None,
            confidence,
        ),
        "score.rough_surface_deviation_detractor": _metric(
            detractors["surface_deviation"],
            None,
            confidence,
        ),
        "score.rough_solver_residual_detractor": _metric(
            detractors["solver_residual"],
            None,
            confidence,
        ),
        "score.rough_high_alpha_required_detractor": _metric(
            detractors.get("high_alpha_required", 0.0),
            None,
            confidence,
        ),
        "score.rough_level_flight_lift_shortfall_detractor": _metric(
            detractors.get("level_flight_lift_shortfall", 0.0),
            None,
            confidence,
        ),
        "score.rough_weak_ld_curve_consistency_detractor": _metric(
            detractors.get("weak_ld_curve_consistency", 0.0),
            None,
            confidence,
        ),
        "score.rough_lift_to_drag": _metric(
            0.0 if cd is None or cl is None or cd <= 0.0 else cl / cd,
            None,
            confidence,
        ),
        "score.rough_alpha_sweep_used": _metric(1 if alpha_sweep_used else 0, None, confidence),
        "score.rough_alpha_screening_tolerant": _metric(
            1 if first_pass_screening else 0,
            None,
            confidence,
        ),
        "score.rough_scoring_alpha_deg": _metric(
            float(polar_point["alpha_deg"]) if polar_point is not None else 0.0,
            "deg",
            confidence,
        ),
        "score.rough_best_usable_ld": _metric(
            float(alpha_sweep.get("best_usable_ld") or 0.0),
            None,
            confidence,
        ),
        "score.rough_average_usable_ld": _metric(
            float(curve_metrics.get("average_positive_ld") or 0.0),
            None,
            confidence,
        ),
        "score.rough_ld_curve_consistency": _metric(
            float(curve_metrics.get("ld_consistency") or 0.0),
            None,
            confidence,
        ),
        "score.rough_required_cl": _metric(
            float(required_cl or 0.0),
            None,
            confidence,
        ),
        "score.rough_alpha_at_required_cl_deg": _metric(
            float(mission_point.get("alpha_deg", 0.0)) if mission_point else 0.0,
            "deg",
            confidence,
        ),
        "score.rough_ld_at_required_cl": _metric(
            float(mission_point.get("ld", 0.0)) if mission_point else 0.0,
            None,
            confidence,
        ),
        "score.rough_cl_alpha_per_deg": _metric(
            float(alpha_sweep.get("cl_alpha_per_deg") or 0.0),
            "1/deg",
            confidence,
        ),
        "score.rough_cm_alpha_per_deg": _metric(
            float(alpha_sweep.get("cm_alpha_per_deg") or 0.0),
            "1/deg",
            confidence,
        ),
        "score.rough_usable_for_optimizer": _metric(1 if usable else 0, None, confidence),
    }
    metadata = {
        "module_name": MODULE_NAME,
        "module_version": MODULE_VERSION,
        "scoring_config": active_config,
        "scoring_tier": "rough_scoring",
        "score_usage": "optimizer_relative_ranking",
        "rough_scoring_allowed": usable,
        "final_scoring_allowed": False,
        "scoring_allowed": False,
        "component_values": components,
        "detractor_values": detractors,
        "input_coefficients": {"cd": cd, "cl": cl, "cm": cm},
        "mission_required_cl": required_cl,
        "mission_scoring_point": mission_point,
        "polar_curve_metrics": curve_metrics,
        "alpha_sweep_used": alpha_sweep_used,
        "alpha_sweep_stage_name": alpha_sweep.get("stage_name"),
        "alpha_sweep_stage_role": alpha_sweep.get("stage_role"),
        "alpha_sweep_screening_tolerant": first_pass_screening,
        "alpha_sweep_aggregate": alpha_sweep,
        "polar_scoring_point": polar_point,
        "steady_scoring_metadata": steady_metadata.get("scoring"),
        "warnings": warnings,
    }
    return metrics, metadata, warnings


def _merge_config(default: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = {**default, **override}
    for key in ("weights", "targets", "gates"):
        merged[key] = {**default.get(key, {}), **override.get(key, {})}
    merged["negative_detractors"] = {
        name: {
            **default.get("negative_detractors", {}).get(name, {}),
            **override.get("negative_detractors", {}).get(name, {}),
        }
        for name in set(default.get("negative_detractors", {}))
        | set(override.get("negative_detractors", {}))
    }
    return merged


def _gate_warnings(
    *,
    cd: float | None,
    cl: float | None,
    cm: float | None,
    ready: float | None,
    rough_allowed: float | None,
    gates: dict[str, Any],
) -> list[str]:
    warnings: list[str] = []
    if gates.get("require_steady_ready", True) and ready != 1:
        warnings.append("steady_result_not_ready")
    if gates.get("require_rough_scoring_allowed", True) and rough_allowed != 1:
        warnings.append("rough_scoring_not_allowed_by_source")
    if gates.get("require_positive_lift", True) and (cl is None or cl <= 0.0):
        warnings.append("non_positive_lift")
    max_abs = float(gates.get("max_abs_coefficient", 10.0))
    for name, value in (("cd", cd), ("cl", cl), ("cm", cm)):
        if value is None:
            warnings.append(f"{name}_missing")
        elif abs(value) > max_abs:
            warnings.append(f"{name}_out_of_range")
    return warnings


def _polar_scoring_point(
    alpha_sweep: dict[str, Any],
    *,
    abs_cm_limit: float,
    screening_tolerant: bool = False,
) -> dict[str, Any] | None:
    points = [
        point
        for point in alpha_sweep.get("points", [])
        if point.get("cd") is not None
        and point.get("cl") is not None
        and point.get("cm") is not None
        and point.get("ld") is not None
        and float(point["cd"]) > 0.0
        and float(point["cl"]) > 0.0
    ]
    if not points:
        return None
    if screening_tolerant:
        selected = max(points, key=lambda item: float(item["ld"]))
        return {
            **selected,
            "exceeds_abs_cm_limit": abs(float(selected["cm"])) > abs_cm_limit,
            "selection_rule": "first_pass_screening_best_positive_lift_ld_soft_cm_review",
        }
    trimmed = [
        point
        for point in points
        if abs(float(point["cm"])) <= abs_cm_limit
    ]
    pool = trimmed or points
    selected = max(pool, key=lambda item: float(item["ld"]))
    return {
        **selected,
        "exceeds_abs_cm_limit": abs(float(selected["cm"])) > abs_cm_limit,
        "selection_rule": (
            "best_positive_lift_ld_within_abs_cm_limit"
            if trimmed
            else "fallback_best_positive_lift_ld_outside_abs_cm_limit"
        ),
    }


def _is_first_pass_screening(alpha_sweep: dict[str, Any]) -> bool:
    if alpha_sweep.get("stage_role") != "candidate_screening":
        return False
    point_count = len(alpha_sweep.get("points", []))
    completed = alpha_sweep.get("completed_alpha_count")
    if completed is None:
        completed = point_count
    return int(completed or 0) <= 2 and point_count <= 2


def _required_lift_coefficient(
    *,
    steady_metrics: dict[str, Any],
    targets: dict[str, Any],
) -> float | None:
    supplied = _metric_number(steady_metrics, "mission.required_cl")
    if supplied is not None and supplied > 0.0:
        return supplied
    mass_kg = _metric_number(steady_metrics, "pre_export.estimated_mass_kg")
    area_m2 = _metric_number(steady_metrics, "pre_export.planform_area_m2")
    if mass_kg is None or area_m2 is None or mass_kg <= 0.0 or area_m2 <= 0.0:
        return None
    speed = float(targets.get("mission_speed_mps", 22.352))
    rho = float(targets.get("air_density_kg_m3", 1.225))
    dynamic_pressure = 0.5 * rho * speed * speed
    if dynamic_pressure <= 0.0:
        return None
    return (mass_kg * 9.80665) / (dynamic_pressure * area_m2)


def _polar_curve_metrics(alpha_sweep: dict[str, Any]) -> dict[str, float | None]:
    points = _valid_positive_lift_points(alpha_sweep)
    if not points:
        return {
            "average_positive_ld": None,
            "max_positive_ld": None,
            "min_positive_ld": None,
            "ld_consistency": None,
            "best_alpha_deg": None,
            "ld_slope_to_best_per_deg": None,
        }
    lds = [float(point["ld"]) for point in points]
    average_ld = sum(lds) / len(lds)
    max_ld = max(lds)
    min_ld = min(lds)
    best = max(points, key=lambda point: float(point["ld"]))
    first = points[0]
    alpha_span = max(float(best["alpha_deg"]) - float(first["alpha_deg"]), 0.0)
    slope = (
        (float(best["ld"]) - float(first["ld"])) / alpha_span
        if alpha_span > 0.0
        else 0.0
    )
    return {
        "average_positive_ld": average_ld,
        "max_positive_ld": max_ld,
        "min_positive_ld": min_ld,
        "ld_consistency": average_ld / max_ld if max_ld > 0.0 else 0.0,
        "best_alpha_deg": float(best["alpha_deg"]),
        "ld_slope_to_best_per_deg": slope,
    }


def _interpolated_point_at_cl(
    alpha_sweep: dict[str, Any],
    required_cl: float,
) -> dict[str, float] | None:
    points = sorted(_valid_force_points(alpha_sweep), key=lambda point: float(point["cl"]))
    if not points:
        return None
    if required_cl < float(points[0]["cl"]) or required_cl > float(points[-1]["cl"]):
        return None
    for left, right in zip(points, points[1:]):
        left_cl = float(left["cl"])
        right_cl = float(right["cl"])
        if left_cl <= required_cl <= right_cl:
            span = right_cl - left_cl
            t = (required_cl - left_cl) / span if abs(span) > 1e-12 else 0.0
            alpha = _lerp(float(left["alpha_deg"]), float(right["alpha_deg"]), t)
            cd = _lerp(float(left["cd"]), float(right["cd"]), t)
            cm = _lerp(float(left["cm"]), float(right["cm"]), t)
            return {
                "alpha_deg": alpha,
                "cl": required_cl,
                "cd": cd,
                "cm": cm,
                "ld": required_cl / cd if cd > 0.0 else 0.0,
                "selection_rule": "interpolated_at_required_level_flight_cl",
            }
    return None


def _mission_ld_score(
    *,
    mission_point: dict[str, Any] | None,
    reference: float,
) -> float:
    if not mission_point:
        return 0.0
    return _clamp(float(mission_point["ld"]) / reference, 0.0, 1.5)


def _curve_consistency_score(
    *,
    curve_metrics: dict[str, float | None],
    average_reference: float,
    consistency_reference: float,
) -> float:
    average_ld = curve_metrics.get("average_positive_ld")
    consistency = curve_metrics.get("ld_consistency")
    if average_ld is None or consistency is None:
        return 0.0
    average_score = _clamp(float(average_ld) / average_reference, 0.0, 1.5)
    consistency_score = _clamp(float(consistency) / consistency_reference, 0.0, 1.2)
    return _clamp(0.65 * average_score + 0.35 * consistency_score, 0.0, 1.5)


def _mission_curve_detractors(
    *,
    required_cl: float | None,
    mission_point: dict[str, Any] | None,
    curve_metrics: dict[str, float | None],
    alpha_sweep: dict[str, Any],
    config: dict[str, Any],
) -> dict[str, float]:
    high_alpha = config.get("high_alpha_required", {})
    lift_shortfall = config.get("level_flight_lift_shortfall", {})
    weak_consistency = config.get("weak_ld_curve_consistency", {})

    high_alpha_penalty = 0.0
    if mission_point:
        high_alpha_penalty = _ramp_penalty(
            value=float(mission_point["alpha_deg"]),
            start=float(high_alpha.get("start_deg", 8.0)),
            limit=float(high_alpha.get("limit_deg", 14.0)),
            weight=float(high_alpha.get("weight", 0.0)),
        )

    shortfall_penalty = 0.0
    if required_cl is not None:
        points = _valid_force_points(alpha_sweep)
        max_cl = max((float(point["cl"]) for point in points), default=0.0)
        margin = max_cl - required_cl
        allowed_margin = max(0.0, 0.05 * required_cl)
        if mission_point is None or margin < allowed_margin:
            shortfall = max(required_cl + allowed_margin - max_cl, 0.0)
            shortfall_penalty = float(lift_shortfall.get("weight", 0.0)) * _clamp(
                shortfall / max(required_cl, 1e-12),
                0.0,
                1.0,
            )

    consistency = curve_metrics.get("ld_consistency")
    weak_consistency_penalty = 0.0
    if consistency is not None:
        target = float(weak_consistency.get("target", 0.7))
        if float(consistency) < target:
            weak_consistency_penalty = float(weak_consistency.get("weight", 0.0)) * _clamp(
                (target - float(consistency)) / max(target, 1e-12),
                0.0,
                1.0,
            )

    return {
        "high_alpha_required": high_alpha_penalty,
        "level_flight_lift_shortfall": shortfall_penalty,
        "weak_ld_curve_consistency": weak_consistency_penalty,
    }


def _valid_force_points(alpha_sweep: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        point
        for point in alpha_sweep.get("points", [])
        if point.get("alpha_deg") is not None
        and point.get("cd") is not None
        and point.get("cl") is not None
        and point.get("cm") is not None
        and float(point["cd"]) > 0.0
    ]


def _valid_positive_lift_points(alpha_sweep: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        point
        for point in _valid_force_points(alpha_sweep)
        if point.get("ld") is not None and float(point["cl"]) > 0.0
    ]


def _usable_lift_score(
    *,
    cl: float,
    cl_min: float,
    cl_target: float,
    cl_max: float,
) -> float:
    if cl <= 0.0:
        return 0.0
    if cl < cl_min:
        return _clamp(cl / cl_min, 0.0, 1.0)
    if cl <= cl_target:
        return 1.0
    if cl <= cl_max:
        return _clamp(1.0 - 0.25 * ((cl - cl_target) / (cl_max - cl_target)), 0.0, 1.0)
    return _clamp(0.75 - 0.5 * ((cl - cl_max) / cl_max), 0.0, 0.75)


def _pitch_moment_score(*, cm: float, target: float, limit: float) -> float:
    abs_cm = abs(cm)
    if abs_cm <= target:
        return 1.0
    return _clamp(1.0 - ((abs_cm - target) / max(limit - target, 1e-9)), 0.0, 1.0)


def _solver_confidence_score(
    *,
    failed_check_count: float | None,
    cd_span: float | None,
    cl_span: float | None,
    cm_span: float | None,
    velocity_residual: float | None,
    local_continuity: float | None,
    yplus_p95: float | None,
) -> float:
    score = 1.0
    score -= min(float(failed_check_count or 0) * 0.25, 0.75)
    score -= _soft_penalty(cd_span, 0.02, 0.25)
    score -= _soft_penalty(cl_span, 0.02, 0.25)
    score -= _soft_penalty(cm_span, 0.02, 0.25)
    score -= _soft_penalty(velocity_residual, 1e-3, 0.15)
    score -= _soft_penalty(local_continuity, 1e-4, 0.15)
    if yplus_p95 is None or yplus_p95 <= 0.0:
        score -= 0.1
    return _clamp(score, 0.0, 1.0)


def _mesh_confidence_score(
    *,
    surface_ready: float | None,
    surface_p95: float | None,
    surface_p99: float | None,
    p95_reference: float,
    p99_reference: float,
) -> float:
    score = 1.0 if surface_ready in (None, 1) else 0.5
    score -= _soft_penalty(surface_p95, p95_reference, 0.35)
    score -= _soft_penalty(surface_p99, p99_reference, 0.25)
    return _clamp(score, 0.0, 1.0)


def _negative_detractors(
    *,
    cl: float,
    cm: float,
    cl_min: float,
    cl_max: float,
    cd_span: float | None,
    cl_span: float | None,
    cm_span: float | None,
    velocity_residual: float | None,
    local_continuity: float | None,
    surface_p95: float | None,
    surface_p99: float | None,
    surface_p95_reference: float,
    surface_p99_reference: float,
    config: dict[str, Any],
) -> dict[str, float]:
    high_abs_cm = config.get("high_abs_cm", {})
    lift_outside_band = config.get("lift_outside_band", {})
    coefficient_instability = config.get("coefficient_instability", {})
    surface_deviation = config.get("surface_deviation", {})
    solver_residual = config.get("solver_residual", {})

    abs_cm = abs(cm)
    high_abs_cm_penalty = _ramp_penalty(
        value=abs_cm,
        start=float(high_abs_cm.get("start", 0.04)),
        limit=float(high_abs_cm.get("limit", 0.12)),
        weight=float(high_abs_cm.get("weight", 0.0)),
    )
    lift_penalty = 0.0
    if cl < cl_min:
        lift_penalty = float(lift_outside_band.get("weight", 0.0)) * _clamp(
            (cl_min - cl) / max(cl_min, 1e-12),
            0.0,
            1.0,
        )
    elif cl > cl_max:
        lift_penalty = float(lift_outside_band.get("weight", 0.0)) * _clamp(
            (cl - cl_max) / max(cl_max, 1e-12),
            0.0,
            1.0,
        )

    instability_reference = (
        float(coefficient_instability.get("cd_span_reference", 0.01))
        + float(coefficient_instability.get("cl_span_reference", 0.01))
        + float(coefficient_instability.get("cm_span_reference", 0.01))
    )
    instability_value = float(cd_span or 0.0) + float(cl_span or 0.0) + float(cm_span or 0.0)
    instability_penalty = float(coefficient_instability.get("weight", 0.0)) * _clamp(
        (instability_value - instability_reference) / max(instability_reference, 1e-12),
        0.0,
        1.0,
    )

    surface_excess = max(float(surface_p95 or 0.0) - surface_p95_reference, 0.0)
    surface_excess += max(float(surface_p99 or 0.0) - surface_p99_reference, 0.0)
    surface_reference = surface_p95_reference + surface_p99_reference
    surface_penalty = float(surface_deviation.get("weight", 0.0)) * _clamp(
        surface_excess / max(surface_reference, 1e-12),
        0.0,
        1.0,
    )

    residual_reference = float(
        solver_residual.get("velocity_residual_reference", 0.001)
    ) + float(solver_residual.get("local_continuity_reference", 0.0001))
    residual_value = float(velocity_residual or 0.0) + float(local_continuity or 0.0)
    residual_penalty = float(solver_residual.get("weight", 0.0)) * _clamp(
        (residual_value - residual_reference) / max(residual_reference, 1e-12),
        0.0,
        1.0,
    )

    return {
        "high_abs_cm": high_abs_cm_penalty,
        "lift_outside_band": lift_penalty,
        "coefficient_instability": instability_penalty,
        "surface_deviation": surface_penalty,
        "solver_residual": residual_penalty,
    }


def _ramp_penalty(*, value: float, start: float, limit: float, weight: float) -> float:
    if value <= start:
        return 0.0
    return weight * _clamp((value - start) / max(limit - start, 1e-12), 0.0, 1.0)


def _soft_penalty(value: float | None, reference: float, max_penalty: float) -> float:
    if value is None:
        return max_penalty * 0.5
    if value <= reference:
        return 0.0
    return min(((value - reference) / max(reference, 1e-12)) * max_penalty, max_penalty)


def _lerp(left: float, right: float, t: float) -> float:
    return left + (right - left) * t


def _score_confidence(components: dict[str, float], *, usable: bool) -> float:
    if not usable:
        return 0.2
    return round(
        0.35
        + 0.35 * components["solver_confidence"]
        + 0.20 * components["mesh_confidence"],
        6,
    )


def _zero_components() -> dict[str, float]:
    return {
        "efficiency_l_over_d": 0.0,
        "mission_l_over_d": 0.0,
        "curve_consistency": 0.0,
        "low_drag": 0.0,
        "usable_lift": 0.0,
        "pitch_moment_trim": 0.0,
        "solver_confidence": 0.0,
        "mesh_confidence": 0.0,
    }


def _zero_detractors() -> dict[str, float]:
    return {
        "high_abs_cm": 0.0,
        "lift_outside_band": 0.0,
        "coefficient_instability": 0.0,
        "surface_deviation": 0.0,
        "solver_residual": 0.0,
        "high_alpha_required": 0.0,
        "level_flight_lift_shortfall": 0.0,
        "weak_ld_curve_consistency": 0.0,
    }


def _metric_number(metrics: dict[str, Any], key: str) -> float | None:
    value = metrics.get(key)
    if isinstance(value, dict):
        value = value.get("value")
    if value is None:
        return None
    return float(value)


def _metric(value: float | int, unit: str | None, confidence: float) -> dict[str, object]:
    return MetricValue(
        value=round(value, 12) if isinstance(value, float) else value,
        unit=unit,
        confidence=confidence,
        source=MODULE_NAME,
    ).to_dict()


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
