from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.modules.openfoam_result_validation import parse_check_mesh_log
from aircraft_optimizer.records import MetricValue

MODULE_NAME = "openfoam_steady_result_validation"
MODULE_VERSION = "0.1.0"

DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY = {
    "acceptance_mode": "openfoam_steady_development",
    "scoring_tier": "development_diagnostic",
    "score_usage": "diagnostic_only",
    "allows_rough_scoring": False,
    "allows_final_scoring": False,
    "require_solver_completed": True,
    "allow_fatal_error": False,
    "max_local_continuity": 1e-2,
    "max_velocity_final_residual": 1e-2,
    "require_force_coefficients": True,
    "require_coefficient_stability": False,
    "max_cd_window_span": 0.25,
    "max_cl_window_span": 0.25,
    "max_cm_window_span": 0.25,
    "stability_window": 3,
    "require_yplus": False,
    "min_yplus_p50": None,
    "max_yplus_p50": None,
    "max_yplus_p95": None,
    "max_yplus_max": None,
}

SNAPPY_HIFI_COEFFICIENT_DEVELOPMENT_POLICY = {
    **DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY,
    "acceptance_mode": "snappy_hifi_coefficient_development",
    "scoring_tier": "confirmation_scoring_development",
    "score_usage": "survivor_confirmation_relative_ranking",
    "allows_rough_scoring": True,
    "require_coefficient_stability": True,
    "max_local_continuity": 1e-4,
    "max_velocity_final_residual": 1e-4,
    "max_cd_window_span": 0.005,
    "max_cl_window_span": 0.005,
    "max_cm_window_span": 0.005,
    "stability_window": 3,
    "require_yplus": True,
    "max_yplus_p95": 60.0,
    "max_yplus_max": 250.0,
}

SNAPPY_HIFI_SCORING_READINESS_POLICY = {
    **DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY,
    "acceptance_mode": "snappy_hifi_scoring_readiness",
    "scoring_tier": "final_scoring_readiness",
    "score_usage": "final_engineering_score_gate",
    "require_coefficient_stability": True,
    "max_local_continuity": 1e-5,
    "max_velocity_final_residual": 1e-5,
    "max_cd_window_span": 0.0025,
    "max_cl_window_span": 0.0025,
    "max_cm_window_span": 0.0025,
    "stability_window": 5,
    "require_yplus": True,
    "min_yplus_p50": 30.0,
    "max_yplus_p50": 100.0,
    "max_yplus_p95": 150.0,
    "max_yplus_max": 300.0,
}

SNAPPY_RANKING_CFD_DEVELOPMENT_POLICY = {
    **DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY,
    "acceptance_mode": "snappy_ranking_cfd_development",
    "scoring_tier": "rough_scoring",
    "score_usage": "optimizer_relative_ranking",
    "allows_rough_scoring": True,
    "require_coefficient_stability": True,
    "max_local_continuity": 1e-3,
    "max_velocity_final_residual": 1e-2,
    "max_cd_window_span": 0.05,
    "max_cl_window_span": 0.05,
    "max_cm_window_span": 0.05,
    "stability_window": 3,
    "require_yplus": True,
    "max_yplus_p95": 80.0,
    "max_yplus_max": 300.0,
}

OPENFOAM_STEADY_ACCEPTANCE_POLICIES = {
    DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY["acceptance_mode"]: (
        DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY
    ),
    SNAPPY_HIFI_COEFFICIENT_DEVELOPMENT_POLICY["acceptance_mode"]: (
        SNAPPY_HIFI_COEFFICIENT_DEVELOPMENT_POLICY
    ),
    SNAPPY_HIFI_SCORING_READINESS_POLICY["acceptance_mode"]: (
        SNAPPY_HIFI_SCORING_READINESS_POLICY
    ),
    SNAPPY_RANKING_CFD_DEVELOPMENT_POLICY["acceptance_mode"]: (
        SNAPPY_RANKING_CFD_DEVELOPMENT_POLICY
    ),
}


@dataclass(frozen=True)
class OpenFoamSteadyResultValidation:
    passed: bool
    openfoam_steady_result: dict[str, Any]
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


def validate_openfoam_steady_result(
    *,
    case_dir: Path,
    solver_log: Path,
    check_mesh_log: Path | None = None,
    force_coeffs_path: Path | None = None,
    yplus_path: Path | None = None,
    policy: dict[str, Any] | None = None,
) -> OpenFoamSteadyResultValidation:
    active_policy = {**DEFAULT_OPENFOAM_STEADY_DEVELOPMENT_POLICY, **(policy or {})}
    solver = parse_incompressible_fluid_log(solver_log)
    coeff_path = force_coeffs_path or find_force_coeffs(case_dir)
    coefficients = parse_force_coeffs(coeff_path) if coeff_path else None
    coefficient_summary = summarize_coefficients(
        coefficients or [],
        window=int(active_policy["stability_window"]),
    )
    yplus_field = yplus_path or find_yplus_field(case_dir)
    yplus_summary = parse_yplus_field(yplus_field) if yplus_field else {"available": False}
    mesh = parse_check_mesh_log(check_mesh_log) if check_mesh_log else None
    result = {
        "openfoam_steady_result_schema_version": "0.1.0",
        "case_dir": str(case_dir),
        "solver": "foamRun -solver incompressibleFluid",
        "case_units": {"length": "m", "velocity": "m/s"},
        "acceptance_mode": active_policy["acceptance_mode"],
        "mesh": mesh,
        "solver_result": solver,
        "force_coefficients": coefficient_summary,
        "yplus": yplus_summary,
        "artifacts": {
            "solver_log": str(solver_log),
            "check_mesh_log": str(check_mesh_log) if check_mesh_log else None,
            "force_coeffs": str(coeff_path) if coeff_path else None,
            "yplus": str(yplus_field) if yplus_field else None,
        },
        "scoring_allowed": False,
    }
    failed_checks = evaluate_steady_checks(result, active_policy)
    scoring = build_scoring_qualification(
        policy=active_policy,
        passed=not failed_checks,
    )
    result["scoring"] = scoring
    # Legacy alias: this means final/engineering scoring, not rough optimizer scoring.
    result["scoring_allowed"] = scoring["final_scoring_allowed"]
    metrics = {
        "openfoam_steady.completed": _metric(1 if solver["completed"] else 0, None),
        "openfoam_steady.fatal_error": _metric(1 if solver["fatal_error"] else 0, None),
        "openfoam_steady.last_time": _metric(solver["last_time"], "s"),
        "openfoam_steady.local_continuity": _metric(
            solver["last_continuity_local"], None
        ),
        "openfoam_steady.velocity_final_residual": _metric(
            solver["last_velocity_final_residual"], None
        ),
        "openfoam_steady.pressure_final_residual": _metric(
            solver["last_p_final_residual"], None
        ),
        "openfoam_steady.cd_last": _metric(coefficient_summary.get("cd_last"), None),
        "openfoam_steady.cl_last": _metric(coefficient_summary.get("cl_last"), None),
        "openfoam_steady.cm_last": _metric(coefficient_summary.get("cm_last"), None),
        "openfoam_steady.cd_window_span": _metric(
            coefficient_summary.get("cd_window_span"), None
        ),
        "openfoam_steady.cl_window_span": _metric(
            coefficient_summary.get("cl_window_span"), None
        ),
        "openfoam_steady.cm_window_span": _metric(
            coefficient_summary.get("cm_window_span"), None
        ),
        "openfoam_steady.coefficient_stable": _metric(
            1 if coefficient_summary.get("coefficient_stable") else 0,
            None,
        ),
        "openfoam_steady.yplus_available": _metric(
            1 if yplus_summary.get("available") else 0,
            None,
        ),
        "openfoam_steady.yplus_p50": _metric(yplus_summary.get("p50"), None),
        "openfoam_steady.yplus_p95": _metric(yplus_summary.get("p95"), None),
        "openfoam_steady.yplus_p99": _metric(yplus_summary.get("p99"), None),
        "openfoam_steady.yplus_max": _metric(yplus_summary.get("max"), None),
        "openfoam_steady.ready": _metric(0 if failed_checks else 1, None),
        "openfoam_steady.failed_check_count": _metric(len(failed_checks), None),
        "openfoam_steady.rough_scoring_allowed": _metric(
            1 if scoring["rough_scoring_allowed"] else 0,
            None,
        ),
        "openfoam_steady.final_scoring_allowed": _metric(
            1 if scoring["final_scoring_allowed"] else 0,
            None,
        ),
    }
    return OpenFoamSteadyResultValidation(
        passed=not failed_checks,
        openfoam_steady_result=result,
        metrics=metrics,
        metadata={
            "module_name": MODULE_NAME,
            "module_version": MODULE_VERSION,
            "policy": active_policy,
            "failed_checks": failed_checks,
            "scoring": scoring,
            "scoring_allowed": scoring["final_scoring_allowed"],
            "coefficient_stable": coefficient_summary.get("coefficient_stable"),
            "yplus_available": yplus_summary.get("available"),
        },
        warnings=[f"failed OpenFOAM steady check: {check}" for check in failed_checks],
    )


def build_scoring_qualification(
    *,
    policy: dict[str, Any],
    passed: bool,
) -> dict[str, Any]:
    rough_allowed = bool(passed and policy.get("allows_rough_scoring", False))
    final_allowed = bool(passed and policy.get("allows_final_scoring", False))
    return {
        "scoring_tier": policy.get("scoring_tier", "development_diagnostic"),
        "score_usage": policy.get("score_usage", "diagnostic_only"),
        "rough_scoring_allowed": rough_allowed,
        "final_scoring_allowed": final_allowed,
        "legacy_scoring_allowed_semantics": "final_scoring_allowed",
        "qualification_basis": policy.get("acceptance_mode"),
    }


def parse_incompressible_fluid_log(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    p_solves = parse_solves(text, "p")
    u_solves = (
        parse_solves(text, "Ux")
        + parse_solves(text, "Uy")
        + parse_solves(text, "Uz")
    )
    time_values = [
        float(value)
        for value in re.findall(r"^Time = ([0-9.eE+-]+)", text, flags=re.MULTILINE)
    ]
    continuity = parse_continuity(text)
    courant = parse_courant(text)
    return {
        "started": "Selecting solver incompressibleFluid" in text or "PIMPLE:" in text,
        "completed": "\nEnd\n" in text and "FOAM FATAL" not in text,
        "fatal_error": (
            "FOAM FATAL" in text
            or "sigFpe::sigHandler" in text
            or "Floating point exception" in text
        ),
        "floating_point_exception": (
            "sigFpe::sigHandler" in text or "Floating point exception" in text
        ),
        "warning_count": text.count("FOAM Warning"),
        "time_steps": len(time_values),
        "first_time": time_values[0] if time_values else None,
        "last_time": time_values[-1] if time_values else None,
        "last_p_initial_residual": p_solves[-1]["initial_residual"] if p_solves else None,
        "last_p_final_residual": p_solves[-1]["final_residual"] if p_solves else None,
        "last_velocity_initial_residual": (
            u_solves[-1]["initial_residual"] if u_solves else None
        ),
        "last_velocity_final_residual": (
            u_solves[-1]["final_residual"] if u_solves else None
        ),
        "last_continuity_local": continuity.get("last_local"),
        "last_continuity_global": continuity.get("last_global"),
        "last_continuity_cumulative": continuity.get("last_cumulative"),
        "courant": courant,
        "execution_time_s": _first_float(
            text,
            r"ExecutionTime = ([0-9.eE+-]+) s",
            last=True,
        ),
        "clock_time_s": _first_float(text, r"ClockTime = ([0-9.eE+-]+) s", last=True),
    }


def parse_force_coeffs(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        reader = csv.reader(
            (line for line in handle if line.strip() and not line.startswith("#")),
            delimiter="\t",
        )
        for raw in reader:
            parts = [item for item in raw if item.strip()]
            if len(parts) < 6:
                parts = raw[0].split() if raw else []
            if len(parts) < 6:
                continue
            rows.append(
                {
                    "time": float(parts[0]),
                    "cm": float(parts[1]),
                    "cd": float(parts[2]),
                    "cl": float(parts[3]),
                    "clf": float(parts[4]),
                    "clr": float(parts[5]),
                }
            )
    return rows


def summarize_coefficients(
    rows: list[dict[str, float]],
    *,
    window: int,
) -> dict[str, Any]:
    if not rows:
        return {
            "available": False,
            "count": 0,
            "coefficient_stable": False,
        }
    tail = rows[-max(window, 1) :]
    cd_values = [row["cd"] for row in tail]
    cl_values = [row["cl"] for row in tail]
    cm_values = [row["cm"] for row in tail]
    cd_span = max(cd_values) - min(cd_values)
    cl_span = max(cl_values) - min(cl_values)
    cm_span = max(cm_values) - min(cm_values)
    return {
        "available": True,
        "count": len(rows),
        "last": rows[-1],
        "window": len(tail),
        "cd_last": rows[-1]["cd"],
        "cl_last": rows[-1]["cl"],
        "cm_last": rows[-1]["cm"],
        "cd_window_min": min(cd_values),
        "cd_window_max": max(cd_values),
        "cd_window_span": cd_span,
        "cl_window_min": min(cl_values),
        "cl_window_max": max(cl_values),
        "cl_window_span": cl_span,
        "cm_window_min": min(cm_values),
        "cm_window_max": max(cm_values),
        "cm_window_span": cm_span,
        "coefficient_stable": cd_span <= 0.25 and cl_span <= 0.25 and cm_span <= 0.25,
    }


def evaluate_steady_checks(
    result: dict[str, Any],
    policy: dict[str, Any],
) -> list[str]:
    failed: list[str] = []
    solver = result["solver_result"]
    coefficients = result["force_coefficients"]
    if policy["require_solver_completed"] and not solver["completed"]:
        failed.append("solver_completed")
    if not policy["allow_fatal_error"] and solver["fatal_error"]:
        failed.append("fatal_error")
    if not _lte_optional(
        solver["last_continuity_local"],
        policy["max_local_continuity"],
    ):
        failed.append("local_continuity")
    if not _lte_optional(
        solver["last_velocity_final_residual"],
        policy["max_velocity_final_residual"],
    ):
        failed.append("velocity_final_residual")
    if policy["require_force_coefficients"] and not coefficients.get("available"):
        failed.append("force_coefficients")
    if policy["require_coefficient_stability"]:
        if not coefficients.get("available"):
            failed.append("coefficient_stability")
        if not _lte_optional(
            coefficients.get("cd_window_span"),
            policy["max_cd_window_span"],
        ):
            failed.append("cd_window_span")
        if not _lte_optional(
            coefficients.get("cl_window_span"),
            policy["max_cl_window_span"],
        ):
            failed.append("cl_window_span")
        if not _lte_optional(
            coefficients.get("cm_window_span"),
            policy["max_cm_window_span"],
        ):
            failed.append("cm_window_span")
    yplus = result["yplus"]
    if policy["require_yplus"] and not yplus.get("available"):
        failed.append("yplus_available")
    if policy.get("min_yplus_p50") is not None and not _gte_optional(
        yplus.get("p50"),
        policy["min_yplus_p50"],
    ):
        failed.append("yplus_p50_min")
    if policy.get("max_yplus_p50") is not None and not _lte_optional(
        yplus.get("p50"),
        policy["max_yplus_p50"],
    ):
        failed.append("yplus_p50_max")
    if policy.get("max_yplus_p95") is not None and not _lte_optional(
        yplus.get("p95"),
        policy["max_yplus_p95"],
    ):
        failed.append("yplus_p95")
    if policy.get("max_yplus_max") is not None and not _lte_optional(
        yplus.get("max"),
        policy["max_yplus_max"],
    ):
        failed.append("yplus_max")
    return failed


def find_force_coeffs(case_dir: Path) -> Path | None:
    matches = sorted(case_dir.glob("postProcessing/**/forceCoeffs.dat"))
    return matches[-1] if matches else None


def find_yplus_field(case_dir: Path) -> Path | None:
    matches = sorted(
        path
        for path in case_dir.glob("*/yPlus")
        if path.parent.name.replace(".", "", 1).isdigit()
    )
    return matches[-1] if matches else None


def parse_yplus_field(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r"value\s+nonuniform\s+List<scalar>\s+(\d+)\s*\((.*?)\)\s*;",
        text,
        flags=re.S,
    )
    if not match:
        return {"available": False, "path": str(path), "count": 0}
    values = [float(item) for item in match.group(2).split()]
    if not values:
        return {"available": False, "path": str(path), "count": 0}
    ordered = sorted(values)
    return {
        "available": True,
        "path": str(path),
        "count": len(ordered),
        "min": ordered[0],
        "mean": sum(ordered) / len(ordered),
        "p50": percentile(ordered, 50.0),
        "p90": percentile(ordered, 90.0),
        "p95": percentile(ordered, 95.0),
        "p99": percentile(ordered, 99.0),
        "max": ordered[-1],
        "lt1": sum(1 for value in ordered if value < 1.0),
        "gt30": sum(1 for value in ordered if value > 30.0),
        "gt100": sum(1 for value in ordered if value > 100.0),
    }


def percentile(ordered_values: list[float], percent: float) -> float:
    if len(ordered_values) == 1:
        return ordered_values[0]
    index = (len(ordered_values) - 1) * percent / 100.0
    lower = int(index)
    upper = min(lower + 1, len(ordered_values) - 1)
    fraction = index - lower
    return ordered_values[lower] * (1.0 - fraction) + ordered_values[upper] * fraction


def parse_solves(text: str, field: str) -> list[dict[str, float | int | str]]:
    pattern = (
        rf"Solving for {re.escape(field)}, Initial residual = ([0-9.eE+-]+), "
        rf"Final residual = ([0-9.eE+-]+), No Iterations ([0-9]+)"
    )
    return [
        {
            "field": field,
            "initial_residual": float(match.group(1)),
            "final_residual": float(match.group(2)),
            "iterations": int(match.group(3)),
        }
        for match in re.finditer(pattern, text)
    ]


def parse_continuity(text: str) -> dict[str, float | None]:
    matches = re.findall(
        r"time step continuity errors : sum local = ([0-9.eE+-]+), "
        r"global = ([0-9.eE+-]+), cumulative = ([0-9.eE+-]+)",
        text,
    )
    if not matches:
        return {"last_local": None, "last_global": None, "last_cumulative": None}
    local, global_, cumulative = matches[-1]
    return {
        "last_local": float(local),
        "last_global": float(global_),
        "last_cumulative": float(cumulative),
    }


def parse_courant(text: str) -> dict[str, float | None]:
    matches = re.findall(
        r"Courant Number mean: ([0-9.eE+-]+) max: ([0-9.eE+-]+)",
        text,
    )
    if not matches:
        return {"last_mean": None, "last_max": None}
    mean, maximum = matches[-1]
    return {"last_mean": float(mean), "last_max": float(maximum)}


def _lte_optional(value: float | None, limit: Any) -> bool:
    return value is not None and value <= float(limit)


def _gte_optional(value: float | None, limit: Any) -> bool:
    return value is not None and value >= float(limit)


def _first_float(text: str, pattern: str, *, last: bool = False) -> float | None:
    matches = re.findall(pattern, text)
    if not matches:
        return None
    return float(matches[-1] if last else matches[0])


def _metric(value: float | int | None, unit: str | None) -> dict[str, object]:
    safe_value: float | int = 0 if value is None else value
    return MetricValue(
        value=round(safe_value, 12) if isinstance(safe_value, float) else safe_value,
        unit=unit,
        confidence=1.0,
        source=MODULE_NAME,
    ).to_dict()
