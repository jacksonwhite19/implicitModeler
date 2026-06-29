from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.records import MetricValue

MODULE_NAME = "openfoam_result_validation"
MODULE_VERSION = "0.1.0"

DEFAULT_OPENFOAM_RESULT_POLICY = {
    "acceptance_mode": "strict",
    "require_mesh_ok": True,
    "min_cells": 1000,
    "min_aircraft_faces": 1,
    "min_farfield_faces": 1,
    "max_non_orthogonality": 75.0,
    "max_skewness": 4.0,
    "max_continuity_error": 1e-3,
    "max_interpolated_velocity_error": 1e-2,
}

RELAXED_DEVELOPMENT_OPENFOAM_RESULT_POLICY = {
    **DEFAULT_OPENFOAM_RESULT_POLICY,
    "acceptance_mode": "relaxed_development",
    "require_mesh_ok": False,
    "max_skewness": 6.0,
}

GMSH_BL_DEVELOPMENT_OPENFOAM_RESULT_POLICY = {
    **DEFAULT_OPENFOAM_RESULT_POLICY,
    "acceptance_mode": "gmsh_bl_development",
    "require_mesh_ok": True,
    "max_non_orthogonality": 90.0,
    "max_skewness": 4.0,
}

SNAPPY_HIFI_DEVELOPMENT_OPENFOAM_RESULT_POLICY = {
    **DEFAULT_OPENFOAM_RESULT_POLICY,
    "acceptance_mode": "snappy_hifi_development",
    "require_mesh_ok": True,
    "min_aircraft_faces": 50000,
    "max_non_orthogonality": 70.0,
    "max_skewness": 12.0,
}

SNAPPY_RANKING_CFD_DEVELOPMENT_OPENFOAM_RESULT_POLICY = {
    **SNAPPY_HIFI_DEVELOPMENT_OPENFOAM_RESULT_POLICY,
    "acceptance_mode": "snappy_ranking_cfd_development",
}

OPENFOAM_ACCEPTANCE_POLICIES = {
    "strict": DEFAULT_OPENFOAM_RESULT_POLICY,
    "relaxed_development": RELAXED_DEVELOPMENT_OPENFOAM_RESULT_POLICY,
    "gmsh_bl_development": GMSH_BL_DEVELOPMENT_OPENFOAM_RESULT_POLICY,
    "snappy_hifi_development": SNAPPY_HIFI_DEVELOPMENT_OPENFOAM_RESULT_POLICY,
    "snappy_ranking_cfd_development": (
        SNAPPY_RANKING_CFD_DEVELOPMENT_OPENFOAM_RESULT_POLICY
    ),
}


@dataclass(frozen=True)
class OpenFoamResultValidation:
    passed: bool
    openfoam_result: dict[str, Any]
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


def validate_openfoam_result(
    *,
    case_dir: Path,
    check_mesh_log: Path,
    solver_log: Path,
    policy: dict[str, Any] | None = None,
) -> OpenFoamResultValidation:
    active_policy = {**DEFAULT_OPENFOAM_RESULT_POLICY, **(policy or {})}
    check_mesh = parse_check_mesh_log(check_mesh_log)
    solver = parse_potential_foam_log(solver_log)
    result = {
        "openfoam_result_schema_version": "0.1.0",
        "case_dir": str(case_dir),
        "solver": "potentialFoam",
        "case_units": {"length": "m", "velocity": "m/s"},
        "acceptance_mode": active_policy["acceptance_mode"],
        "mesh": check_mesh,
        "solver_result": solver,
        "artifacts": {
            "check_mesh_log": str(check_mesh_log),
            "solver_log": str(solver_log),
            "vtk_dir": str(case_dir / "VTK"),
        },
    }
    failed_checks = evaluate_openfoam_checks(result, active_policy)
    metrics = {
        "openfoam.mesh_ok": _metric(1 if check_mesh["mesh_ok"] else 0, None),
        "openfoam.cells": _metric(check_mesh["cells"], None),
        "openfoam.points": _metric(check_mesh["points"], None),
        "openfoam.aircraft_faces": _metric(
            check_mesh["patches"].get("aircraft", {}).get("faces", 0),
            None,
        ),
        "openfoam.farfield_faces": _metric(
            check_mesh["patches"].get("farfield", {}).get("faces", 0),
            None,
        ),
        "openfoam.max_non_orthogonality": _metric(
            check_mesh["quality"]["max_non_orthogonality"],
            None,
        ),
        "openfoam.max_skewness": _metric(check_mesh["quality"]["max_skewness"], None),
        "openfoam.continuity_error": _metric(solver["continuity_error"], None),
        "openfoam.interpolated_velocity_error": _metric(
            solver["interpolated_velocity_error"],
            None,
        ),
        "openfoam.ready": _metric(0 if failed_checks else 1, None),
        "openfoam.failed_check_count": _metric(len(failed_checks), None),
    }
    return OpenFoamResultValidation(
        passed=not failed_checks,
        openfoam_result=result,
        metrics=metrics,
        metadata={
            "module_name": MODULE_NAME,
            "module_version": MODULE_VERSION,
            "policy": active_policy,
            "failed_checks": failed_checks,
        },
        warnings=[f"failed OpenFOAM check: {check}" for check in failed_checks],
    )


def policy_for_acceptance_mode(mode: str) -> dict[str, Any]:
    try:
        return OPENFOAM_ACCEPTANCE_POLICIES[mode]
    except KeyError as exc:
        raise ValueError(f"unknown OpenFOAM acceptance mode: {mode}") from exc


def parse_check_mesh_log(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    patches = {}
    for name, faces in re.findall(
        r"^\s*(farfield|aircraft)\s+(\d+)\s+\d+\s+ok\b",
        text,
        re.M,
    ):
        patches[name] = {"faces": int(faces)}
    for name, faces in re.findall(
        r"^\s*\d+\s+(farfield|aircraft)\s+\S+\s+(\d+)\s*$",
        text,
        re.M,
    ):
        patches.setdefault(name, {"faces": int(faces)})

    max_non_ortho = _first_float(
        text,
        r"Mesh non-orthogonality Max:\s*([0-9.eE+-]+)",
    )
    avg_non_ortho = _first_float(
        text,
        r"Mesh non-orthogonality Max:\s*[0-9.eE+-]+\s+average:\s*([0-9.eE+-]+)",
    )
    max_skewness = _first_float(text, r"Max skewness =\s*([0-9.eE+-]+)")
    return {
        "mesh_ok": "Mesh OK." in text or "Mesh OK" in text,
        "points": _first_int(text, r"^\s*points:\s*(\d+)", re.M) or 0,
        "faces": _first_int(text, r"^\s*faces:\s*(\d+)", re.M) or 0,
        "cells": _first_int(text, r"^\s*cells:\s*(\d+)", re.M) or 0,
        "patches": patches,
        "quality": {
            "max_non_orthogonality": max_non_ortho,
            "average_non_orthogonality": avg_non_ortho,
            "max_skewness": max_skewness,
        },
    }


def parse_potential_foam_log(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    return {
        "execution_completed": "End" in text,
        "continuity_error": _first_float(text, r"Continuity error =\s*([0-9.eE+-]+)"),
        "interpolated_velocity_error": _first_float(
            text,
            r"Interpolated velocity error =\s*([0-9.eE+-]+)",
        ),
        "execution_time_s": _first_float(text, r"ExecutionTime =\s*([0-9.eE+-]+) s"),
    }


def evaluate_openfoam_checks(
    result: dict[str, Any],
    policy: dict[str, Any],
) -> list[str]:
    failed: list[str] = []
    mesh = result["mesh"]
    solver = result["solver_result"]
    if policy["require_mesh_ok"] and not mesh["mesh_ok"]:
        failed.append("mesh_ok")
    if mesh["cells"] < int(policy["min_cells"]):
        failed.append("min_cells")
    if mesh["patches"].get("aircraft", {}).get("faces", 0) < int(
        policy["min_aircraft_faces"]
    ):
        failed.append("aircraft_faces")
    if mesh["patches"].get("farfield", {}).get("faces", 0) < int(
        policy["min_farfield_faces"]
    ):
        failed.append("farfield_faces")
    if not _lte_optional(
        mesh["quality"]["max_non_orthogonality"],
        policy["max_non_orthogonality"],
    ):
        failed.append("max_non_orthogonality")
    if not _lte_optional(mesh["quality"]["max_skewness"], policy["max_skewness"]):
        failed.append("max_skewness")
    if not solver["execution_completed"]:
        failed.append("solver_completed")
    if not _lte_optional(solver["continuity_error"], policy["max_continuity_error"]):
        failed.append("continuity_error")
    if not _lte_optional(
        solver["interpolated_velocity_error"],
        policy["max_interpolated_velocity_error"],
    ):
        failed.append("interpolated_velocity_error")
    return failed


def _lte_optional(value: float | None, limit: Any) -> bool:
    return value is not None and value <= float(limit)


def _first_int(text: str, pattern: str, flags: int = 0) -> int | None:
    match = re.search(pattern, text, flags)
    return int(match.group(1)) if match else None


def _first_float(text: str, pattern: str, flags: int = 0) -> float | None:
    match = re.search(pattern, text, flags)
    return float(match.group(1)) if match else None


def _metric(value: float | int | None, unit: str | None) -> dict[str, object]:
    safe_value: float | int = 0 if value is None else value
    return MetricValue(
        value=round(safe_value, 12) if isinstance(safe_value, float) else safe_value,
        unit=unit,
        confidence=1.0,
        source=MODULE_NAME,
    ).to_dict()
