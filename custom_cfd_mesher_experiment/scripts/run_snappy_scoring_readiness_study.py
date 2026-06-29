from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SCRIPTS_DIR = Path(__file__).resolve().parent

LEVELS: dict[str, dict[str, Any]] = {
    "v0_4_medium": {
        "base_cells": "44,28,44",
        "surface_min_level": 4,
        "surface_max_level": 5,
        "feature_level": 5,
        "feature_refinement_level": 5,
        "feature_box_count": 18,
        "feature_box_grid": "10,10,6",
        "max_global_cells": 6_000_000,
        "max_local_cells": 3_000_000,
    },
    "v0_4_fine_surface": {
        "base_cells": "56,36,56",
        "surface_min_level": 5,
        "surface_max_level": 6,
        "feature_level": 6,
        "feature_refinement_level": 6,
        "feature_box_count": 22,
        "feature_box_grid": "12,12,8",
        "max_global_cells": 9_000_000,
        "max_local_cells": 5_000_000,
    },
}

SURFACE_LIMITS = {
    "p95_mm": 0.25,
    "p99_mm": 2.5,
    "max_mm": 6.0,
}

YPLUS_WALL_FUNCTION_TARGET = {
    "p50_min": 30.0,
    "p50_max": 100.0,
    "p95_max": 150.0,
    "max_max": 300.0,
}

CONVERGENCE_LIMITS = {
    "cd_relative_delta_max": 0.05,
    "cl_relative_delta_max": 0.05,
    "cm_absolute_delta_max": 0.01,
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a bounded snappy/OpenFOAM CFD scoring-readiness study."
    )
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--variant-id", default="scoring_readiness_aircraft")
    parser.add_argument(
        "--levels",
        default="v0_4_medium,v0_4_fine_surface",
        help="Comma-separated study levels. Known: " + ",".join(LEVELS),
    )
    parser.add_argument("--velocity", default="22.352,0,0")
    parser.add_argument("--end-time", type=int, default=100)
    parser.add_argument("--timeout-s", type=int, default=1200)
    parser.add_argument("--parallel-procs", type=int, default=12)
    parser.add_argument("--surface-fidelity-samples", type=int, default=40000)
    parser.add_argument("--reuse-existing", action="store_true")
    args = parser.parse_args()

    selected_levels = [item.strip() for item in args.levels.split(",") if item.strip()]
    unknown = [level for level in selected_levels if level not in LEVELS]
    if unknown:
        raise SystemExit(f"Unknown study level(s): {', '.join(unknown)}")

    args.run_root.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    level_reports = []
    for level in selected_levels:
        level_reports.append(run_level(level, LEVELS[level], args))

    summary = {
        "schema": "snappy_scoring_readiness_study.v0_1",
        "input_stl": str(args.input_stl),
        "run_root": str(args.run_root),
        "variant_id": args.variant_id,
        "levels": level_reports,
        "runtime_s": time.perf_counter() - started,
    }
    summary["readiness"] = evaluate_scoring_readiness(level_reports)
    write_outputs(args.run_root, summary)
    print(json.dumps(summary, indent=2, sort_keys=True))


def run_level(level: str, config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    requested_level_root = args.run_root / level
    requested_level_root.mkdir(parents=True, exist_ok=True)
    level_root = find_actual_level_root(requested_level_root)
    if not (args.reuse_existing and (level_root / "comparison_summary.json").exists()):
        level_root = requested_level_root

    if not (args.reuse_existing and (level_root / "comparison_summary.json").exists()):
        mesh_command = [
            sys.executable,
            str(SCRIPTS_DIR / "run_snappy_layer_comparison.py"),
            "--run-root",
            str(level_root),
            "--input-stl",
            str(args.input_stl),
            "--single-variant-id",
            args.variant_id,
            "--velocity",
            args.velocity,
            "--base-cells",
            config["base_cells"],
            "--pad",
            "0.8",
            "--surface-min-level",
            str(config["surface_min_level"]),
            "--surface-max-level",
            str(config["surface_max_level"]),
            "--feature-level",
            str(config["feature_level"]),
            "--n-cells-between-levels",
            "4",
            "--snap-tolerance",
            "0.5",
            "--n-smooth-patch",
            "10",
            "--n-surface-layers",
            "1",
            "--layer-relative-sizes",
            "true",
            "--layer-expansion-ratio",
            "1.2",
            "--final-layer-thickness",
            "0.75",
            "--min-layer-thickness",
            "0.12",
            "--layer-feature-angle",
            "70.0",
            "--n-layer-iter",
            "50",
            "--n-relaxed-iter",
            "20",
            "--max-global-cells",
            str(config["max_global_cells"]),
            "--max-local-cells",
            str(config["max_local_cells"]),
            "--feature-refinement-level",
            str(config["feature_refinement_level"]),
            "--feature-angle-deg",
            "24.0",
            "--feature-box-count",
            str(config["feature_box_count"]),
            "--feature-box-grid",
            config["feature_box_grid"],
            "--feature-box-padding-frac",
            "0.03",
            "--parallel-procs",
            str(args.parallel_procs),
            "--check-skew-threshold",
            "12.0",
            "--feature-refinement-boxes",
            "--include-known-hotspot-boxes",
            "--surface-fidelity-audit",
            "--surface-fidelity-samples",
            str(args.surface_fidelity_samples),
        ]
        run_command(mesh_command, level_root / "log.mesh_level_driver.txt")
        level_root = find_actual_level_root(requested_level_root)

    if not (args.reuse_existing and rans_summary_is_complete(level_root, args.variant_id)):
        rans_command = [
            sys.executable,
            str(SCRIPTS_DIR / "run_no_slip_laminar_start_rans.py"),
            "--run-root",
            str(level_root),
            "--variant-ids",
            args.variant_id,
            "--velocity",
            args.velocity,
            "--end-time",
            str(args.end_time),
            "--mag-u-inf",
            args.velocity.split(",")[0],
            "--wall-treatment",
            "wall_function",
            "--timeout-s",
            str(args.timeout_s),
        ]
        run_command(rans_command, level_root / "log.rans_level_driver.txt")
    return summarize_level(level, level_root, args.variant_id, reused=args.reuse_existing)


def find_actual_level_root(requested_level_root: Path) -> Path:
    if (requested_level_root / "comparison_summary.json").exists():
        return requested_level_root
    parent = requested_level_root.parent
    matches = sorted(
        parent.glob(f"{requested_level_root.name}*"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for match in matches:
        if (match / "comparison_summary.json").exists():
            return match
    return requested_level_root


def rans_summary_is_complete(level_root: Path, variant_id: str) -> bool:
    summary_path = level_root / "no_slip_ladder_summary.json"
    if not summary_path.exists():
        return False
    data = read_json(summary_path)
    for report in data.get("reports", []):
        if report.get("variant_id") == variant_id:
            return bool(report.get("rans", {}).get("completed"))
    return False


def summarize_level(level: str, level_root: Path, variant_id: str, *, reused: bool) -> dict[str, Any]:
    comparison = read_json(level_root / "comparison_summary.json")
    ladder = read_json(level_root / "no_slip_ladder_summary.json")
    mesh_report = next(
        item for item in comparison.get("reports", []) if item.get("variant_id") == variant_id
    )
    steady_report = next(
        item for item in ladder.get("reports", []) if item.get("variant_id") == variant_id
    )
    variant_dir = level_root / variant_id
    surface_report_path = variant_dir / "surface_fidelity_vtk_distance_1layer_candidate.json"
    surface = read_json(surface_report_path) if surface_report_path.exists() else {}
    check_mesh = mesh_report.get("mesh_summary", {}).get("check_mesh", {})
    rans = steady_report.get("rans", {})
    force = rans.get("force_last") or {}
    yplus = rans.get("yplus") or {}
    bidir = surface.get("bidirectional_mm") or {}
    return {
        "level": level,
        "level_root": str(level_root),
        "variant_dir": str(variant_dir),
        "reused": reused,
        "mesh_verdict": mesh_report.get("verdict"),
        "cells": check_mesh.get("cells"),
        "points": check_mesh.get("points"),
        "aircraft_patch_faces": check_mesh.get("aircraft_patch_faces"),
        "max_non_orthogonality": check_mesh.get("max_non_orthogonality"),
        "max_skewness": check_mesh.get("max_skewness"),
        "potential_completed": mesh_report.get("potential_summary", {}).get("completed"),
        "surface_p95_mm": bidir.get("p95"),
        "surface_p99_mm": bidir.get("p99"),
        "surface_max_mm": bidir.get("max"),
        "surface_report": str(surface_report_path) if surface_report_path.exists() else None,
        "laminar_completed": steady_report.get("laminar", {}).get("completed"),
        "rans_completed": rans.get("completed"),
        "rans_fatal_error": rans.get("fatal_error"),
        "rans_velocity_residual": rans.get("last_velocity_final_residual"),
        "rans_pressure_residual": rans.get("last_p_final_residual"),
        "force_stable_for_scoring_gate": rans.get("force_stable_for_scoring_gate"),
        "cd": force.get("Cd"),
        "cl": force.get("Cl"),
        "cm": force.get("Cm"),
        "yplus_available": bool(yplus.get("available")),
        "yplus_p50": yplus.get("p50"),
        "yplus_p95": yplus.get("p95"),
        "yplus_max": yplus.get("max"),
        "yplus_wall_function_gate": rans.get("yplus_wall_function_gate"),
    }


def evaluate_scoring_readiness(level_reports: list[dict[str, Any]]) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []
    for report in level_reports:
        prefix = report["level"]
        if report.get("mesh_verdict") != "pass_for_layered_plumbing":
            blockers.append(f"{prefix}: mesh/potential plumbing failed")
        if not report.get("rans_completed"):
            blockers.append(f"{prefix}: kOmegaSST did not complete")
        if report.get("rans_fatal_error"):
            blockers.append(f"{prefix}: kOmegaSST fatal error")
        if not report.get("force_stable_for_scoring_gate"):
            blockers.append(f"{prefix}: force coefficients failed stability gate")
        if not within(report.get("surface_p95_mm"), SURFACE_LIMITS["p95_mm"]):
            blockers.append(f"{prefix}: surface p95 exceeds {SURFACE_LIMITS['p95_mm']} mm")
        if not within(report.get("surface_p99_mm"), SURFACE_LIMITS["p99_mm"]):
            blockers.append(f"{prefix}: surface p99 exceeds {SURFACE_LIMITS['p99_mm']} mm")
        if not within(report.get("surface_max_mm"), SURFACE_LIMITS["max_mm"]):
            blockers.append(f"{prefix}: surface max exceeds {SURFACE_LIMITS['max_mm']} mm")
        if not report.get("yplus_available"):
            blockers.append(f"{prefix}: yPlus missing")
        elif not wall_function_yplus_ready(report):
            blockers.append(f"{prefix}: yPlus wall-function target not met")

    convergence = compare_finest_pair(level_reports)
    if convergence is None:
        blockers.append("mesh convergence not tested")
    else:
        if not convergence["cd_relative_delta_ok"]:
            blockers.append("Cd mesh sensitivity exceeds limit")
        if not convergence["cl_relative_delta_ok"]:
            blockers.append("Cl mesh sensitivity exceeds limit")
        if not convergence["cm_absolute_delta_ok"]:
            blockers.append("Cm mesh sensitivity exceeds limit")
        if convergence["cell_ratio"] < 1.25:
            warnings.append("finest two meshes may be too close in size for a strong convergence claim")

    return {
        "scoring_allowed": False,
        "ready_for_scoring": not blockers,
        "blockers": blockers,
        "warnings": warnings,
        "convergence": convergence,
        "surface_limits": SURFACE_LIMITS,
        "yplus_wall_function_target": YPLUS_WALL_FUNCTION_TARGET,
        "convergence_limits": CONVERGENCE_LIMITS,
        "decision_note": (
            "Even if ready_for_scoring becomes true, optimizer scoring should stay disabled "
            "until a human reviews coefficient plausibility and reference-area policy."
        ),
    }


def compare_finest_pair(level_reports: list[dict[str, Any]]) -> dict[str, Any] | None:
    usable = [
        report
        for report in level_reports
        if report.get("cells") and report.get("cd") is not None and report.get("cl") is not None
    ]
    if len(usable) < 2:
        return None
    ordered = sorted(usable, key=lambda item: int(item["cells"]))
    coarse, fine = ordered[-2], ordered[-1]
    cd_delta = abs(float(fine["cd"]) - float(coarse["cd"]))
    cl_delta = abs(float(fine["cl"]) - float(coarse["cl"]))
    cm_delta = abs(float(fine.get("cm") or 0.0) - float(coarse.get("cm") or 0.0))
    cd_rel = relative_delta(cd_delta, fine["cd"])
    cl_rel = relative_delta(cl_delta, fine["cl"])
    return {
        "coarse_level": coarse["level"],
        "fine_level": fine["level"],
        "coarse_cells": coarse["cells"],
        "fine_cells": fine["cells"],
        "cell_ratio": float(fine["cells"]) / float(coarse["cells"]),
        "cd_coarse": coarse["cd"],
        "cd_fine": fine["cd"],
        "cd_absolute_delta": cd_delta,
        "cd_relative_delta": cd_rel,
        "cd_relative_delta_ok": cd_rel <= CONVERGENCE_LIMITS["cd_relative_delta_max"],
        "cl_coarse": coarse["cl"],
        "cl_fine": fine["cl"],
        "cl_absolute_delta": cl_delta,
        "cl_relative_delta": cl_rel,
        "cl_relative_delta_ok": cl_rel <= CONVERGENCE_LIMITS["cl_relative_delta_max"],
        "cm_coarse": coarse.get("cm"),
        "cm_fine": fine.get("cm"),
        "cm_absolute_delta": cm_delta,
        "cm_absolute_delta_ok": cm_delta <= CONVERGENCE_LIMITS["cm_absolute_delta_max"],
    }


def wall_function_yplus_ready(report: dict[str, Any]) -> bool:
    p50 = report.get("yplus_p50")
    p95 = report.get("yplus_p95")
    max_value = report.get("yplus_max")
    if p50 is None or p95 is None or max_value is None:
        return False
    return (
        YPLUS_WALL_FUNCTION_TARGET["p50_min"] <= float(p50) <= YPLUS_WALL_FUNCTION_TARGET["p50_max"]
        and float(p95) <= YPLUS_WALL_FUNCTION_TARGET["p95_max"]
        and float(max_value) <= YPLUS_WALL_FUNCTION_TARGET["max_max"]
    )


def within(value: Any, limit: float) -> bool:
    return value is not None and float(value) <= limit


def relative_delta(delta: float, reference: Any) -> float:
    value = abs(float(reference))
    if value < 1e-9:
        return float("inf")
    return delta / value


def write_outputs(run_root: Path, summary: dict[str, Any]) -> None:
    (run_root / "scoring_readiness_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    lines = [
        "# Snappy Scoring Readiness Study",
        "",
        f"Input STL: `{summary['input_stl']}`",
        f"Variant ID: `{summary['variant_id']}`",
        "",
        "## Verdict",
        "",
        f"- Ready for scoring: `{summary['readiness']['ready_for_scoring']}`",
        f"- Scoring allowed in optimizer: `{summary['readiness']['scoring_allowed']}`",
        "",
        "## Blockers",
        "",
    ]
    blockers = summary["readiness"]["blockers"]
    lines.extend([f"- {item}" for item in blockers] or ["- none"])
    lines.extend(
        [
            "",
            "## Level Metrics",
            "",
            "| Level | Cells | Faces | Surf p95 mm | Cd | Cl | Cm | y+ p50 | y+ p95 | y+ max |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for report in summary["levels"]:
        lines.append(
            f"| {report['level']} | {fmt(report.get('cells'))} | {fmt(report.get('aircraft_patch_faces'))} | "
            f"{fmt(report.get('surface_p95_mm'), 4)} | {fmt(report.get('cd'), 5)} | "
            f"{fmt(report.get('cl'), 5)} | {fmt(report.get('cm'), 5)} | "
            f"{fmt(report.get('yplus_p50'), 2)} | {fmt(report.get('yplus_p95'), 2)} | "
            f"{fmt(report.get('yplus_max'), 2)} |"
        )
    lines.extend(["", "## Convergence", "", "```json", json.dumps(summary["readiness"]["convergence"], indent=2, sort_keys=True), "```", ""])
    (run_root / "scoring_readiness_summary.md").write_text(
        "\n".join(lines),
        encoding="utf-8",
    )


def fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, bool):
        return str(value)
    return f"{float(value):.{digits}f}"


def run_command(command: list[str], log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    completed = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    log_path.write_text(
        json.dumps(
            {
                "command": command,
                "returncode": completed.returncode,
                "runtime_s": time.perf_counter() - started,
                "output_tail": completed.stdout[-12000:],
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    if completed.returncode != 0:
        raise subprocess.CalledProcessError(completed.returncode, command, completed.stdout)


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8-sig"))


if __name__ == "__main__":
    main()
