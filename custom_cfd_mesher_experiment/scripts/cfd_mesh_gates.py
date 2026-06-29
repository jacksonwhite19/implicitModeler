from __future__ import annotations

import argparse
import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import trimesh

from cfd_surface_mesher import mesh_report


@dataclass(frozen=True)
class SourceSurfaceLimits:
    max_boundary_edges: int = 0
    max_nonmanifold_edges: int = 0
    max_body_count: int = 1
    require_watertight: bool = True
    max_sliver_q_lt_0p10: int | None = None


@dataclass(frozen=True)
class MeshResultLimits:
    max_unexpected_openfoam_patches: int = 0
    max_default_faces: int = 0
    max_failed_checks: int = 0
    max_highly_skew_faces: int = 0
    max_severe_nonorth_faces: int = 250
    max_nonorth_deg: float = 89.0
    max_skewness: float = 4.0
    max_aspect_ratio: float = 250.0
    require_su2_valid: bool = True
    require_openfoam_mesh_ok: bool = True


EXPECTED_SPLIT_PATCHES = {
    "aircraft",
    "inlet",
    "outlet",
    "side_ymin",
    "side_ymax",
    "side_zmin",
    "side_zmax",
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate CFD mesh pipeline gates.")
    sub = parser.add_subparsers(dest="command", required=True)

    source = sub.add_parser("source")
    source.add_argument("--input-stl", type=Path, required=True)
    source.add_argument("--report", type=Path, required=True)
    source.add_argument("--max-boundary-edges", type=int, default=0)
    source.add_argument("--max-nonmanifold-edges", type=int, default=0)
    source.add_argument("--max-body-count", type=int, default=1)
    source.add_argument("--allow-open", action="store_true")
    source.add_argument("--max-sliver-q-lt-0p10", type=int, default=None)

    mesh = sub.add_parser("mesh")
    mesh.add_argument("--pipeline-report", type=Path, required=True)
    mesh.add_argument("--case-dir", type=Path, required=True)
    mesh.add_argument("--report", type=Path, required=True)
    mesh.add_argument("--expected-patches", default=",".join(sorted(EXPECTED_SPLIT_PATCHES)))
    mesh.add_argument("--expected-optimizers", default="default,Netgen,Relocate3D")
    mesh.add_argument("--max-unexpected-openfoam-patches", type=int, default=0)
    mesh.add_argument("--max-default-faces", type=int, default=0)
    mesh.add_argument("--max-failed-checks", type=int, default=0)
    mesh.add_argument("--max-highly-skew-faces", type=int, default=0)
    mesh.add_argument("--max-severe-nonorth-faces", type=int, default=250)
    mesh.add_argument("--max-nonorth-deg", type=float, default=89.0)
    mesh.add_argument("--max-skewness", type=float, default=4.0)
    mesh.add_argument("--max-aspect-ratio", type=float, default=250.0)
    mesh.add_argument("--allow-invalid-su2", action="store_true")
    mesh.add_argument("--allow-openfoam-check-fail", action="store_true")

    axis = sub.add_parser("axis-reference")
    axis.add_argument("--pipeline-report", type=Path, required=True)
    axis.add_argument("--report", type=Path, required=True)
    axis.add_argument("--drag-dir", default="1,0,0")
    axis.add_argument("--lift-dir", default="0,0,1")
    axis.add_argument("--pitch-axis", default="0,1,0")
    axis.add_argument("--a-ref", type=float, required=True)
    axis.add_argument("--l-ref", type=float, required=True)
    axis.add_argument("--cofr", default="0,0,0")
    axis.add_argument("--min-x-span", type=float, default=0.05)
    axis.add_argument("--min-y-span", type=float, default=0.05)
    axis.add_argument("--min-z-span", type=float, default=0.01)
    axis.add_argument("--max-z-over-planform-span", type=float, default=0.6)
    axis.add_argument("--cofr-margin-fraction", type=float, default=0.25)

    yplus = sub.add_parser("yplus")
    yplus.add_argument("--log", type=Path, required=True)
    yplus.add_argument("--report", type=Path, required=True)
    yplus.add_argument("--patch", default="aircraft")
    yplus.add_argument(
        "--policy",
        choices=["low_re", "wall_function", "custom", "observe"],
        default="wall_function",
    )
    yplus.add_argument("--min-yplus", type=float, default=None)
    yplus.add_argument("--max-yplus", type=float, default=None)
    yplus.add_argument("--max-average-yplus", type=float, default=None)

    args = parser.parse_args()
    if args.command == "source":
        limits = SourceSurfaceLimits(
            max_boundary_edges=args.max_boundary_edges,
            max_nonmanifold_edges=args.max_nonmanifold_edges,
            max_body_count=args.max_body_count,
            require_watertight=not args.allow_open,
            max_sliver_q_lt_0p10=args.max_sliver_q_lt_0p10,
        )
        report = evaluate_source_surface(args.input_stl, limits)
    elif args.command == "mesh":
        limits = MeshResultLimits(
            max_unexpected_openfoam_patches=args.max_unexpected_openfoam_patches,
            max_default_faces=args.max_default_faces,
            max_failed_checks=args.max_failed_checks,
            max_highly_skew_faces=args.max_highly_skew_faces,
            max_severe_nonorth_faces=args.max_severe_nonorth_faces,
            max_nonorth_deg=args.max_nonorth_deg,
            max_skewness=args.max_skewness,
            max_aspect_ratio=args.max_aspect_ratio,
            require_su2_valid=not args.allow_invalid_su2,
            require_openfoam_mesh_ok=not args.allow_openfoam_check_fail,
        )
        expected = {part.strip() for part in args.expected_patches.split(",") if part.strip()}
        expected_optimizers = [part.strip() for part in args.expected_optimizers.split(",") if part.strip()]
        report = evaluate_mesh_result(args.pipeline_report, args.case_dir, limits, expected, expected_optimizers)
    elif args.command == "axis-reference":
        report = evaluate_axis_reference(
            args.pipeline_report,
            drag_dir=parse_vector(args.drag_dir),
            lift_dir=parse_vector(args.lift_dir),
            pitch_axis=parse_vector(args.pitch_axis),
            a_ref=args.a_ref,
            l_ref=args.l_ref,
            cofr=parse_vector(args.cofr),
            min_x_span=args.min_x_span,
            min_y_span=args.min_y_span,
            min_z_span=args.min_z_span,
            max_z_over_planform_span=args.max_z_over_planform_span,
            cofr_margin_fraction=args.cofr_margin_fraction,
        )
    elif args.command == "yplus":
        report = evaluate_yplus(
            args.log,
            patch=args.patch,
            policy=args.policy,
            min_yplus=args.min_yplus,
            max_yplus=args.max_yplus,
            max_average_yplus=args.max_average_yplus,
        )

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["pass_gate"]:
        raise SystemExit(2)


def evaluate_source_surface(path: Path, limits: SourceSurfaceLimits) -> dict[str, Any]:
    mesh = trimesh.load(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh: {path}")
    mesh.merge_vertices(digits_vertex=8)
    report = mesh_report(mesh)
    failures: list[str] = []
    if limits.require_watertight and not report["watertight"]:
        failures.append("surface is not watertight")
    if int(report["boundary_edges"]) > limits.max_boundary_edges:
        failures.append(f"boundary_edges={report['boundary_edges']} exceeds {limits.max_boundary_edges}")
    if int(report["nonmanifold_edges"]) > limits.max_nonmanifold_edges:
        failures.append(f"nonmanifold_edges={report['nonmanifold_edges']} exceeds {limits.max_nonmanifold_edges}")
    if int(report["body_count"]) > limits.max_body_count:
        failures.append(f"body_count={report['body_count']} exceeds {limits.max_body_count}")
    if limits.max_sliver_q_lt_0p10 is not None and int(report["sliver_faces_q_lt_0p10"]) > limits.max_sliver_q_lt_0p10:
        failures.append(
            f"sliver_faces_q_lt_0p10={report['sliver_faces_q_lt_0p10']} exceeds {limits.max_sliver_q_lt_0p10}"
        )
    return {
        "gate": "source_surface",
        "input_stl": str(path),
        "pass_gate": not failures,
        "failures": failures,
        "limits": asdict(limits),
        "surface": report,
    }


def evaluate_mesh_result(
    pipeline_report: Path,
    case_dir: Path,
    limits: MeshResultLimits,
    expected_patches: set[str],
    expected_optimizers: list[str],
) -> dict[str, Any]:
    pipeline = json.loads(pipeline_report.read_text(encoding="utf-8"))
    openfoam = pipeline.get("openfoam") or {}
    su2 = pipeline.get("su2_validation") or {}
    boundary = parse_openfoam_boundary(case_dir / "constant" / "polyMesh" / "boundary")
    actual = set(boundary)
    unexpected = sorted(actual - expected_patches)
    missing = sorted(expected_patches - actual)
    default_faces = int(boundary.get("defaultFaces", {}).get("nFaces", 0))
    actual_optimizers = list((pipeline.get("gmsh_report") or {}).get("optimizers") or [])

    failures: list[str] = []
    if limits.require_su2_valid and not su2.get("valid"):
        failures.append("SU2 validation failed")
    if limits.require_openfoam_mesh_ok and not openfoam.get("mesh_ok"):
        failures.append("OpenFOAM checkMesh did not report Mesh OK")
    if expected_optimizers and actual_optimizers != expected_optimizers:
        failures.append(f"gmsh optimizers {actual_optimizers} did not match expected {expected_optimizers}")
    if missing:
        failures.append(f"missing expected OpenFOAM patches: {', '.join(missing)}")
    if len(unexpected) > limits.max_unexpected_openfoam_patches:
        failures.append(f"unexpected OpenFOAM patches {unexpected} exceeds {limits.max_unexpected_openfoam_patches}")
    if default_faces > limits.max_default_faces:
        failures.append(f"defaultFaces={default_faces} exceeds {limits.max_default_faces}")
    failed_checks = int(openfoam.get("failed_checks") or 0)
    if failed_checks > limits.max_failed_checks:
        failures.append(f"failed_checks={failed_checks} exceeds {limits.max_failed_checks}")
    highly_skew = int(openfoam.get("highly_skew_faces") or 0)
    if highly_skew > limits.max_highly_skew_faces:
        failures.append(f"highly_skew_faces={highly_skew} exceeds {limits.max_highly_skew_faces}")
    severe = int(openfoam.get("severely_non_orthogonal_faces") or 0)
    if severe > limits.max_severe_nonorth_faces:
        failures.append(f"severely_non_orthogonal_faces={severe} exceeds {limits.max_severe_nonorth_faces}")
    max_nonorth = float(openfoam.get("max_non_orthogonality") or 0.0)
    if max_nonorth > limits.max_nonorth_deg:
        failures.append(f"max_non_orthogonality={max_nonorth} exceeds {limits.max_nonorth_deg}")
    max_skew = float(openfoam.get("max_skewness") or 0.0)
    if max_skew > limits.max_skewness:
        failures.append(f"max_skewness={max_skew} exceeds {limits.max_skewness}")
    max_aspect = float(openfoam.get("max_aspect_ratio") or 0.0)
    if max_aspect > limits.max_aspect_ratio:
        failures.append(f"max_aspect_ratio={max_aspect} exceeds {limits.max_aspect_ratio}")

    return {
        "gate": "mesh_result",
        "pipeline_report": str(pipeline_report),
        "case_dir": str(case_dir),
        "pass_gate": not failures,
        "failures": failures,
        "limits": asdict(limits),
        "expected_patches": sorted(expected_patches),
        "expected_optimizers": expected_optimizers,
        "actual_optimizers": actual_optimizers,
        "openfoam_patches": boundary,
        "unexpected_openfoam_patches": unexpected,
        "missing_openfoam_patches": missing,
        "default_faces": default_faces,
        "mesh_summary": {
            "cells": openfoam.get("cells"),
            "points": openfoam.get("points"),
            "mesh_ok": openfoam.get("mesh_ok"),
            "max_aspect_ratio": openfoam.get("max_aspect_ratio"),
            "max_skewness": openfoam.get("max_skewness"),
            "max_non_orthogonality": openfoam.get("max_non_orthogonality"),
            "severely_non_orthogonal_faces": openfoam.get("severely_non_orthogonal_faces"),
            "su2_valid": su2.get("valid"),
        },
    }


def evaluate_axis_reference(
    pipeline_report: Path,
    *,
    drag_dir: tuple[float, float, float],
    lift_dir: tuple[float, float, float],
    pitch_axis: tuple[float, float, float],
    a_ref: float,
    l_ref: float,
    cofr: tuple[float, float, float],
    min_x_span: float,
    min_y_span: float,
    min_z_span: float,
    max_z_over_planform_span: float,
    cofr_margin_fraction: float,
) -> dict[str, Any]:
    pipeline = json.loads(pipeline_report.read_text(encoding="utf-8"))
    bounds = extract_aircraft_bounds(pipeline)
    lo = [float(value) for value in bounds[0]]
    hi = [float(value) for value in bounds[1]]
    span = [hi[index] - lo[index] for index in range(3)]
    planform_span = max(span[0], span[1])

    failures: list[str] = []
    if span[0] < min_x_span:
        failures.append(f"x_span={span[0]} below {min_x_span}")
    if span[1] < min_y_span:
        failures.append(f"y_span={span[1]} below {min_y_span}")
    if span[2] < min_z_span:
        failures.append(f"z_span={span[2]} below {min_z_span}")
    if planform_span > 0.0 and span[2] / planform_span > max_z_over_planform_span:
        failures.append(
            f"z_span/planform_span={span[2] / planform_span} exceeds {max_z_over_planform_span}"
        )

    expected = {
        "drag_dir": (1.0, 0.0, 0.0),
        "lift_dir": (0.0, 0.0, 1.0),
        "pitch_axis": (0.0, 1.0, 0.0),
    }
    actual = {
        "drag_dir": drag_dir,
        "lift_dir": lift_dir,
        "pitch_axis": pitch_axis,
    }
    for name, vector in actual.items():
        if not vector_close(vector, expected[name]):
            failures.append(f"{name}={vector} does not match expected {expected[name]}")
        norm = vector_norm(vector)
        if abs(norm - 1.0) > 1e-6:
            failures.append(f"{name} is not unit length: {norm}")
    for first, second in (("drag_dir", "lift_dir"), ("drag_dir", "pitch_axis"), ("lift_dir", "pitch_axis")):
        dot = vector_dot(actual[first], actual[second])
        if abs(dot) > 1e-6:
            failures.append(f"{first} and {second} are not orthogonal: dot={dot}")

    if a_ref <= 0.0:
        failures.append("a_ref must be positive")
    if l_ref <= 0.0:
        failures.append("l_ref must be positive")

    margin = [max(component * cofr_margin_fraction, 1e-9) for component in span]
    for index, axis_name in enumerate(("x", "y", "z")):
        if cofr[index] < lo[index] - margin[index] or cofr[index] > hi[index] + margin[index]:
            failures.append(
                f"cofr_{axis_name}={cofr[index]} outside bounds plus margin "
                f"[{lo[index] - margin[index]}, {hi[index] + margin[index]}]"
            )

    return {
        "gate": "axis_reference",
        "pipeline_report": str(pipeline_report),
        "pass_gate": not failures,
        "failures": failures,
        "axis_convention": {
            "x": "longitudinal_flow",
            "y": "span",
            "z": "vertical",
            "drag_dir": drag_dir,
            "lift_dir": lift_dir,
            "pitch_axis": pitch_axis,
        },
        "reference_values": {
            "a_ref": a_ref,
            "l_ref": l_ref,
            "cofr": cofr,
        },
        "geometry": {
            "bounds_m": [lo, hi],
            "span_m": {
                "x": span[0],
                "y": span[1],
                "z": span[2],
            },
            "z_over_planform_span": span[2] / planform_span if planform_span else None,
        },
    }


def evaluate_yplus(
    log_path: Path,
    *,
    patch: str,
    policy: str,
    min_yplus: float | None,
    max_yplus: float | None,
    max_average_yplus: float | None,
) -> dict[str, Any]:
    text = log_path.read_text(encoding="utf-8", errors="replace")
    values = parse_yplus_log(text, patch)
    if policy == "low_re":
        policy_min = 0.0
        policy_max = 5.0
        policy_avg = 2.0
    elif policy == "wall_function":
        policy_min = 30.0
        policy_max = 300.0
        policy_avg = 150.0
    elif policy == "observe":
        policy_min = None
        policy_max = None
        policy_avg = None
    else:
        policy_min = None
        policy_max = None
        policy_avg = None

    effective_min = min_yplus if min_yplus is not None else policy_min
    effective_max = max_yplus if max_yplus is not None else policy_max
    effective_avg = max_average_yplus if max_average_yplus is not None else policy_avg

    failures: list[str] = []
    if values is None:
        failures.append(f"no yPlus summary found for patch {patch}")
    else:
        if effective_min is not None and values["min"] < effective_min:
            failures.append(f"min_yplus={values['min']} below {effective_min}")
        if effective_max is not None and values["max"] > effective_max:
            failures.append(f"max_yplus={values['max']} exceeds {effective_max}")
        if effective_avg is not None and values["average"] > effective_avg:
            failures.append(f"average_yplus={values['average']} exceeds {effective_avg}")

    return {
        "gate": "yplus",
        "log": str(log_path),
        "patch": patch,
        "policy": policy,
        "pass_gate": not failures,
        "failures": failures,
        "limits": {
            "min_yplus": effective_min,
            "max_yplus": effective_max,
            "max_average_yplus": effective_avg,
        },
        "summary": values,
    }


def extract_aircraft_bounds(pipeline: dict[str, Any]) -> list[list[float]]:
    gmsh_report = pipeline.get("gmsh_report") or {}
    farfield_domain = gmsh_report.get("farfield_domain") or {}
    if farfield_domain.get("aircraft_bounds"):
        return farfield_domain["aircraft_bounds"]
    prepared_surface = gmsh_report.get("prepared_surface") or {}
    if prepared_surface.get("bounds"):
        return prepared_surface["bounds"]
    surface_fidelity = pipeline.get("surface_fidelity") or {}
    if surface_fidelity.get("candidate_bounds_m"):
        return surface_fidelity["candidate_bounds_m"]
    raise ValueError("Could not find aircraft bounds in pipeline report")


def parse_yplus_log(text: str, patch: str) -> dict[str, float] | None:
    pattern = (
        rf"patch\s+{re.escape(patch)}\s+y\+\s*:\s*min\s*=\s*([0-9.eE+-]+),\s*"
        rf"max\s*=\s*([0-9.eE+-]+),\s*average\s*=\s*([0-9.eE+-]+)"
    )
    match = re.search(pattern, text)
    if not match:
        return None
    return {
        "min": float(match.group(1)),
        "max": float(match.group(2)),
        "average": float(match.group(3)),
    }


def parse_vector(value: str) -> tuple[float, float, float]:
    parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError(f"Expected vector as x,y,z: {value}")
    return parts[0], parts[1], parts[2]


def vector_norm(vector: tuple[float, float, float]) -> float:
    return sum(component * component for component in vector) ** 0.5


def vector_dot(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> float:
    return sum(left[index] * right[index] for index in range(3))


def vector_close(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> bool:
    return all(abs(left[index] - right[index]) <= 1e-9 for index in range(3))


def parse_openfoam_boundary(path: Path) -> dict[str, dict[str, int | str]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    patches: dict[str, dict[str, int | str]] = {}
    lines = [line.strip() for line in text.splitlines()]
    i = 0
    while i < len(lines):
        name = lines[i]
        if not name or name.startswith("//") or name in {"(", ")"} or name.isdigit() or name.startswith("FoamFile"):
            i += 1
            continue
        if i + 1 < len(lines) and lines[i + 1] == "{":
            patch: dict[str, int | str] = {}
            i += 2
            while i < len(lines) and lines[i] != "}":
                parts = lines[i].rstrip(";").split()
                if len(parts) >= 2:
                    value: int | str
                    try:
                        value = int(parts[1])
                    except ValueError:
                        value = parts[1]
                    patch[parts[0]] = value
                i += 1
            patches[name] = patch
        i += 1
    return patches


if __name__ == "__main__":
    main()
