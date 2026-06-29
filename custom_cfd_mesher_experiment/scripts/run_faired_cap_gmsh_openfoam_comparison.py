from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent

VARIANTS = [
    (
        "fcv01_long_glider",
        REPO_ROOT
        / "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl",
    ),
    (
        "fcv02_short_swept",
        REPO_ROOT
        / "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv02_short_swept_faired_cap_spacing_1p0.stl",
    ),
    (
        "fcv03_high_aspect_mild",
        REPO_ROOT
        / "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv03_high_aspect_mild_faired_cap_spacing_1p0.stl",
    ),
    (
        "fcv04_compact_wide_tail",
        REPO_ROOT
        / "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv04_compact_wide_tail_faired_cap_spacing_1p0.stl",
    ),
    (
        "fcv05_aft_wing_fast",
        REPO_ROOT
        / "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv05_aft_wing_fast_faired_cap_spacing_1p0.stl",
    ),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the five faired-cap inlet variants through Gmsh/OpenFOAM.")
    parser.add_argument("--run-root", type=Path, default=EXPERIMENT_ROOT / "runs" / "faired_cap_gmsh_openfoam_20260623")
    parser.add_argument("--target-edge-mm", type=float, default=4.0)
    parser.add_argument("--surface-size", type=float, default=0.004)
    parser.add_argument("--max-surface-distance-mm", type=float, default=0.15)
    parser.add_argument("--remesh-iterations", type=int, default=6)
    parser.add_argument("--smooth-remesh", action="store_true")
    parser.add_argument("--velocity", default="22.352,0,0")
    args = parser.parse_args()

    run_root = unique_run_root(args.run_root)
    run_root.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    reports = []
    for variant_id, stl_path in VARIANTS:
        reports.append(run_variant(variant_id, stl_path, run_root, args))

    summary = {
        "status": "ready",
        "run_root": str(run_root),
        "runtime_s": time.perf_counter() - started,
        "variant_count": len(reports),
        "reports": reports,
    }
    (run_root / "comparison_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    (run_root / "comparison_summary.md").write_text(render_summary_markdown(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


def unique_run_root(path: Path) -> Path:
    if not path.exists():
        return path
    index = 2
    while True:
        candidate = path.with_name(f"{path.name}_v{index:02d}")
        if not candidate.exists():
            return candidate
        index += 1


def run_variant(variant_id: str, stl_path: Path, run_root: Path, args: argparse.Namespace) -> dict[str, Any]:
    variant_dir = run_root / variant_id
    variant_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    report: dict[str, Any] = {
        "variant_id": variant_id,
        "input_stl": str(stl_path),
        "run_dir": str(variant_dir),
        "commands": [],
        "status": "running",
        "stl_conditioning": {
            "source_stl_modified": False,
            "conditioning": "pymeshlab_isotropic_remesh_for_cfd_surface_input",
            "target_edge_mm": args.target_edge_mm,
            "surface_size_m": args.surface_size,
            "max_surface_distance_mm": args.max_surface_distance_mm,
            "smooth_remesh": args.smooth_remesh,
        },
    }
    try:
        if not stl_path.exists():
            raise FileNotFoundError(stl_path)

        mesh_run_dir = variant_dir / "gmsh_mesh"
        command = [
            sys.executable,
            str(SCRIPTS_DIR / "run_gated_mesh_smoke.py"),
            "--input-stl",
            str(stl_path),
            "--run-dir",
            str(mesh_run_dir),
            "--skip-solver",
            "--target-edge-mm",
            str(args.target_edge_mm),
            "--surface-size",
            str(args.surface_size),
            "--max-surface-distance-mm",
            str(args.max_surface_distance_mm),
            "--remesh-iterations",
            str(args.remesh_iterations),
            "--feature-deg",
            "25",
            "--farfield-policy",
            "dynamic",
            "--farfield-size-policy",
            "dynamic",
            "--farfield-size-fraction",
            "0.15",
            "--min-farfield-size",
            "0.04",
            "--max-farfield-size",
            "0.35",
        ]
        if args.smooth_remesh:
            command.append("--smooth-remesh")
        run_logged(command, variant_dir / "log.run_gated_mesh_smoke.txt", report, check=False)

        runner_report = load_json(mesh_run_dir / "runner_report.json")
        pipeline_report_path = mesh_run_dir / "gmsh" / "pipeline_report.json"
        mesh_gate_path = mesh_run_dir / "mesh_gate.json"
        pipeline = load_json(pipeline_report_path) if pipeline_report_path.exists() else {}
        mesh_gate = load_json(mesh_gate_path) if mesh_gate_path.exists() else {}
        openfoam = pipeline.get("openfoam") or {}
        su2_validation = pipeline.get("su2_validation") or {}

        screenshot_dir = variant_dir / "screenshots"
        prepared_stl = mesh_run_dir / "gmsh" / "aircraft_prepared_for_gmsh.stl"
        if prepared_stl.exists():
            render_cmd = [
                sys.executable,
                str(SCRIPTS_DIR / "render_stl_view_set.py"),
                "--input-stl",
                str(prepared_stl),
                "--output-dir",
                str(screenshot_dir),
                "--edges",
            ]
            run_logged(render_cmd, variant_dir / "log.render_view_set.txt", report, check=False)

        potential_report = run_potential_foam(variant_dir, mesh_run_dir, args.velocity, report, bool(openfoam.get("mesh_ok")))

        report.update(
            collect_variant_metrics(
                runner_report=runner_report,
                pipeline=pipeline,
                mesh_gate=mesh_gate,
                potential_report=potential_report,
                mesh_run_dir=mesh_run_dir,
                screenshot_dir=screenshot_dir,
                runtime_s=time.perf_counter() - started,
            )
        )
        report["status"] = "ready"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        report["runtime_s"] = time.perf_counter() - started

    report["verdict"] = classify_verdict(report)
    report["verdict_explanation"] = explain_verdict(report)
    (variant_dir / "variant_report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return report


def run_potential_foam(
    variant_dir: Path,
    mesh_run_dir: Path,
    velocity: str,
    report: dict[str, Any],
    mesh_ok: bool,
) -> dict[str, Any]:
    case_dir = mesh_run_dir / "gmsh" / "openfoam_case"
    potential_report = {
        "status": "skipped",
        "reason": "OpenFOAM mesh was not usable enough for potentialFoam",
        "case_dir": str(case_dir),
    }
    if not mesh_ok or not case_dir.exists():
        return potential_report

    setup_cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "setup_potential_foam_smoke.py"),
        "--case-dir",
        str(case_dir),
        "--velocity",
        velocity,
    ]
    setup_result = run_logged(setup_cmd, variant_dir / "log.setup_potential_foam.txt", report, check=False)
    if setup_result.returncode != 0:
        return {"status": "setup_failed", "case_dir": str(case_dir)}

    case_wsl = windows_path_to_wsl(case_dir.resolve())
    foam_script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "potentialFoam -writep > log.potentialFoam 2>&1"
    )
    run_logged(["wsl", "bash", "-lc", foam_script], variant_dir / "log.potential_foam_driver.txt", report, check=False)

    summary_path = case_dir / "potential_summary.json"
    summarize_cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "summarize_potential_foam_smoke.py"),
        "--log",
        str(case_dir / "log.potentialFoam"),
        "--report",
        str(summary_path),
    ]
    run_logged(summarize_cmd, variant_dir / "log.summarize_potential_foam.txt", report, check=False)
    if summary_path.exists():
        return load_json(summary_path)
    return {"status": "summary_missing", "case_dir": str(case_dir), "log": str(case_dir / "log.potentialFoam")}


def collect_variant_metrics(
    *,
    runner_report: dict[str, Any],
    pipeline: dict[str, Any],
    mesh_gate: dict[str, Any],
    potential_report: dict[str, Any],
    mesh_run_dir: Path,
    screenshot_dir: Path,
    runtime_s: float,
) -> dict[str, Any]:
    openfoam = pipeline.get("openfoam") or {}
    su2_validation = pipeline.get("su2_validation") or {}
    gmsh_report = pipeline.get("gmsh_report") or {}
    patches = mesh_gate.get("openfoam_patches") or {}
    aircraft_patch = patches.get("aircraft") or {}
    farfield_faces = sum(
        int((patches.get(name) or {}).get("nFaces", 0))
        for name in ("farfield", "inlet", "outlet", "side_ymin", "side_ymax", "side_zmin", "side_zmax")
    )
    remesh_report = load_json(mesh_run_dir / "remesh" / "remesh_report.json") if (mesh_run_dir / "remesh" / "remesh_report.json").exists() else {}
    return {
        "runtime_s": runtime_s,
        "meshing_tool": "Gmsh",
        "meshing_tool_version": ((pipeline.get("tool_versions") or {}).get("gmsh_python_api")),
        "openfoam_version": ((pipeline.get("tool_versions") or {}).get("openfoam")),
        "mesh_case_dir": str(mesh_run_dir / "gmsh" / "openfoam_case"),
        "mesh_format_path": str(mesh_run_dir / "gmsh" / "mesh.msh"),
        "su2_mesh_path": str(mesh_run_dir / "gmsh" / "mesh.su2"),
        "patch_names": sorted(patches.keys()),
        "cells": openfoam.get("cells"),
        "points": openfoam.get("points"),
        "aircraft_patch_faces": aircraft_patch.get("nFaces"),
        "farfield_patch_faces": farfield_faces,
        "boundary_layer_prism_status": "none_tetrahedral_slip_wall_smoke",
        "min_cell_volume": parse_checkmesh_value(mesh_run_dir / "gmsh" / "openfoam_case" / "log.checkMesh", "Min volume = "),
        "max_cell_volume": parse_checkmesh_value(mesh_run_dir / "gmsh" / "openfoam_case" / "log.checkMesh", "Max volume = "),
        "max_skewness": openfoam.get("max_skewness"),
        "max_non_orthogonality": openfoam.get("max_non_orthogonality"),
        "severe_nonorth_faces": openfoam.get("severely_non_orthogonal_faces"),
        "failed_checkmesh_checks": openfoam.get("failed_checks"),
        "strict_checkmesh_verdict": "pass" if openfoam.get("mesh_ok") else "fail",
        "mesh_gate_pass": mesh_gate.get("pass_gate"),
        "mesh_gate_failures": mesh_gate.get("failures", []),
        "su2_valid": su2_validation.get("valid"),
        "potential_foam": potential_report,
        "surface_deviation_metrics": {
            "source_to_remesh_mm": (remesh_report.get("surface_fidelity_mm") or {}),
            "remesh_to_prepared_mm": (pipeline.get("surface_fidelity") or {}),
        },
        "geometry_capture_visual_assessment": geometry_assessment(pipeline, remesh_report),
        "aircraft_iso_screenshot": str(mesh_run_dir / "gmsh" / "aircraft_iso.png"),
        "screenshot_folder": str(screenshot_dir),
        "logs_and_reports": {
            "runner_report": str(mesh_run_dir / "runner_report.json"),
            "pipeline_report": str(mesh_run_dir / "gmsh" / "pipeline_report.json"),
            "mesh_gate": str(mesh_run_dir / "mesh_gate.json"),
            "source_gate": str(mesh_run_dir / "source_gate.json"),
            "remesh_report": str(mesh_run_dir / "remesh" / "remesh_report.json"),
            "potential_log": str(mesh_run_dir / "gmsh" / "openfoam_case" / "log.potentialFoam"),
            "potential_summary": str(mesh_run_dir / "gmsh" / "openfoam_case" / "potential_summary.json"),
            "nonortho_localization": str(mesh_run_dir / "nonortho_localization.json"),
        },
        "runner_status": runner_report.get("status"),
        "runner_error": runner_report.get("error"),
    }


def geometry_assessment(pipeline: dict[str, Any], remesh_report: dict[str, Any]) -> str:
    source_fidelity = (remesh_report.get("surface_fidelity_mm") or {}).get("bidirectional") or {}
    p95 = source_fidelity.get("p95")
    p99 = source_fidelity.get("p99")
    max_dist = source_fidelity.get("max")
    faces = ((pipeline.get("gmsh_report") or {}).get("prepared_surface") or {}).get("faces")
    if p95 is None:
        return "Surface deviation unavailable; inspect screenshots before using for CFD development."
    if p95 <= 0.08 and (p99 or 0.0) <= 0.15:
        return (
            f"Good proxy capture for development: aircraft patch has {faces} faces; "
            f"source-to-remesh bidirectional p95={p95:.4f} mm, p99={p99:.4f} mm, max={max_dist:.4f} mm. "
            "Screenshots still required for inlet lip, cap, LE/TE, tips, root blend, tail, and fuselage curvature."
        )
    return (
        f"Conditional geometry capture: source-to-remesh bidirectional p95={p95:.4f} mm, "
        f"p99={p99:.4f} mm, max={max_dist:.4f} mm. Inspect closeups before treating this as development CFD input."
    )


def classify_verdict(report: dict[str, Any]) -> str:
    if report.get("status") != "ready":
        return "fail_openfoam_quality"
    if report.get("strict_checkmesh_verdict") == "fail":
        return "fail_openfoam_quality"
    potential = report.get("potential_foam") or {}
    if potential.get("status") not in (None, "skipped") and potential.get("completed") is False:
        return "fail_solver_smoke"
    if not potential.get("completed"):
        return "fail_solver_smoke"
    if report.get("mesh_gate_pass") is True:
        return "pass_for_mesher_plumbing"
    return "conditional_development_only"


def explain_verdict(report: dict[str, Any]) -> str:
    verdict = report.get("verdict")
    if verdict == "pass_for_mesher_plumbing":
        return "Strict mesh gate passed and potentialFoam completed; still solver-plumbing only because aircraft wall is slip."
    if verdict == "conditional_development_only":
        return "OpenFOAM mesh is usable and potentialFoam completed, but strict mesh gate failed or remained marginal."
    if verdict == "fail_solver_smoke":
        return "Mesh reached OpenFOAM but potentialFoam did not complete."
    return "OpenFOAM strict quality or automation failed before a usable solver smoke result."


def parse_checkmesh_value(path: Path, prefix: str) -> float | None:
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8", errors="replace")
    if prefix == "Min volume = ":
        match = re.search(r"Min volume = ([0-9.eE+-]+)", text)
    elif prefix == "Max volume = ":
        match = re.search(r"Max volume = ([0-9.eE+-]+)", text)
    else:
        match = None
    if not match:
        marker = text.find(prefix)
        if marker < 0:
            return None
        raw = text[marker + len(prefix) :].split()[0]
    else:
        raw = match.group(1)
    try:
        return float(raw.rstrip("."))
    except ValueError:
        return None


def run_logged(command: list[str], log_path: Path, report: dict[str, Any], *, check: bool) -> subprocess.CompletedProcess[str]:
    started = time.perf_counter()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.write_text("$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr, encoding="utf-8")
    report.setdefault("commands", []).append(
        {
            "command": command,
            "log": str(log_path),
            "returncode": result.returncode,
            "runtime_s": time.perf_counter() - started,
        }
    )
    if check and result.returncode != 0:
        raise RuntimeError(f"command failed with exit code {result.returncode}; see {log_path}")
    return result


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def windows_path_to_wsl(path: Path) -> str:
    text = str(path)
    drive, rest = text[0], text[2:]
    rest = rest.replace("\\", "/")
    return f"/mnt/{drive.lower()}{rest}"


def render_summary_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# Faired-Cap Gmsh/OpenFOAM Comparison",
        "",
        f"Run root: `{summary['run_root']}`",
        f"Runtime: `{summary['runtime_s']:.1f} s`",
        "",
        "| Variant | Verdict | Cells | Aircraft faces | Max skew | Max nonorth | Severe >70 | checkMesh | potentialFoam | p95 mm | p99 mm | max mm | Runtime s | Risk |",
        "|---|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|---:|---|",
    ]
    for report in summary["reports"]:
        fidelity = ((report.get("surface_deviation_metrics") or {}).get("source_to_remesh_mm") or {}).get("bidirectional") or {}
        potential = report.get("potential_foam") or {}
        lines.append(
            "| {variant} | {verdict} | {cells} | {aircraft_faces} | {skew} | {nonorth} | {severe} | {check} | {potential} | {p95} | {p99} | {maxd} | {runtime} | {risk} |".format(
                variant=report.get("variant_id"),
                verdict=report.get("verdict"),
                cells=fmt(report.get("cells")),
                aircraft_faces=fmt(report.get("aircraft_patch_faces")),
                skew=fmt(report.get("max_skewness")),
                nonorth=fmt(report.get("max_non_orthogonality")),
                severe=fmt(report.get("severe_nonorth_faces")),
                check=report.get("strict_checkmesh_verdict"),
                potential="complete" if potential.get("completed") else potential.get("status", "failed"),
                p95=fmt(fidelity.get("p95")),
                p99=fmt(fidelity.get("p99")),
                maxd=fmt(fidelity.get("max")),
                runtime=fmt(report.get("runtime_s")),
                risk="; ".join(report.get("mesh_gate_failures") or []) or report.get("verdict_explanation", ""),
            )
        )
    lines.append("")
    return "\n".join(lines)


def fmt(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


if __name__ == "__main__":
    main()
