from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from generate_gmsh_followup_report import (
    render_feature_closeups,
    run_and_localize_nonortho,
    windows_path_to_wsl,
)


VARIANTS = {
    "fcv01_long_glider": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl"
    ),
    "fcv02_short_swept": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv02_short_swept_faired_cap_spacing_1p0.stl"
    ),
    "fcv03_high_aspect_mild": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv03_high_aspect_mild_faired_cap_spacing_1p0.stl"
    ),
    "fcv04_compact_wide_tail": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv04_compact_wide_tail_faired_cap_spacing_1p0.stl"
    ),
    "fcv05_aft_wing_fast": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv05_aft_wing_fast_faired_cap_spacing_1p0.stl"
    ),
}


V02_PRESET = {
    "name": "gmsh_openfoam_external_aero_v0_2",
    "description": "Bounded all-variant Gmsh/OpenFOAM plumbing preset with 4.5 mm remesh/surface target and Netgen disabled for unattended stability.",
    "mesher": "Gmsh",
    "openfoam_solver_smoke": "potentialFoam -writep",
    "parameters": {
        "target_edge_mm": 4.5,
        "surface_size_m": 0.0045,
        "max_surface_distance_mm": 0.18,
        "remesh_iterations": 6,
        "feature_deg": 25.0,
        "smooth_remesh": False,
        "algorithm3d": 4,
        "optimize": "default,Relocate3D",
        "farfield_policy": "dynamic",
        "upstream_lengths": 1.5,
        "downstream_lengths": 3.0,
        "side_y_spans": 2.0,
        "side_z_spans": 2.0,
        "min_farfield_padding": 0.15,
        "farfield_size_policy": "dynamic",
        "farfield_size_fraction": 0.15,
        "min_farfield_size": 0.04,
        "max_farfield_size": 0.35,
        "farfield_patches": "split",
        "velocity_mps": [22.352, 0.0, 0.0],
    },
    "acceptance_gates": {
        "require_strict_checkmesh": True,
        "require_potential_foam_complete": True,
        "max_skewness": 4.0,
        "max_non_orthogonality_development": 89.0,
        "max_severe_nonorth_faces_development": 250,
        "require_nonzero_aircraft_patch_faces": True,
        "require_nonzero_farfield_patch_faces": True,
        "scoring_allowed": False,
    },
    "known_limitations": [
        "No prism or boundary-layer cells.",
        "potentialFoam smoke only; no trusted lift or drag.",
        "Severe non-orthogonal faces remain near aircraft features.",
        "Netgen optimization is intentionally disabled after a bounded 4.5 mm probe crashed on fcv01.",
        "Promotion to scoring CFD requires no-slip RANS and wall-treatment validation.",
    ],
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    started = time.perf_counter()
    args.run_root.mkdir(parents=True, exist_ok=True)
    scripts_dir = Path(__file__).resolve().parent

    preset_dir = args.run_root / V02_PRESET["name"]
    preset_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for variant_id, stl in VARIANTS.items():
        result = run_variant(
            variant_id=variant_id,
            input_stl=stl,
            preset_dir=preset_dir,
            scripts_dir=scripts_dir,
            force=args.force,
        )
        results.append(result)
        write_variant_report(preset_dir / variant_id / "mesh_quality_report.json", result)
        write_variant_report_md(preset_dir / variant_id / "mesh_quality_report.md", result)

    baseline = load_baseline(args.baseline_root)
    matrix = {
        "status": "ready",
        "run_root": str(args.run_root),
        "baseline_root": str(args.baseline_root),
        "preset": V02_PRESET,
        "baseline_results": baseline,
        "candidate_results": results,
        "runtime_s": time.perf_counter() - started,
        "recommendation": recommendation(baseline, results),
    }
    write_outputs(args.run_root, matrix)
    print(json.dumps(matrix, indent=2, sort_keys=True))


def run_variant(
    *,
    variant_id: str,
    input_stl: Path,
    preset_dir: Path,
    scripts_dir: Path,
    force: bool,
) -> dict[str, Any]:
    variant_dir = preset_dir / variant_id
    mesh_dir = variant_dir / "gmsh_mesh"
    if force and variant_dir.exists():
        shutil.rmtree(variant_dir)
    variant_dir.mkdir(parents=True, exist_ok=True)
    commands: list[dict[str, Any]] = []

    runner_report = mesh_dir / "runner_report.json"
    if not runner_report.exists():
        cmd = [
            sys.executable,
            str(scripts_dir / "run_gated_mesh_smoke.py"),
            "--input-stl",
            str(input_stl),
            "--run-dir",
            str(mesh_dir),
            "--skip-solver",
            "--target-edge-mm",
            str(V02_PRESET["parameters"]["target_edge_mm"]),
            "--surface-size",
            str(V02_PRESET["parameters"]["surface_size_m"]),
            "--max-surface-distance-mm",
            str(V02_PRESET["parameters"]["max_surface_distance_mm"]),
            "--remesh-iterations",
            str(V02_PRESET["parameters"]["remesh_iterations"]),
            "--feature-deg",
            str(V02_PRESET["parameters"]["feature_deg"]),
            "--algorithm3d",
            str(V02_PRESET["parameters"]["algorithm3d"]),
            "--optimize",
            str(V02_PRESET["parameters"]["optimize"]),
            "--farfield-policy",
            str(V02_PRESET["parameters"]["farfield_policy"]),
            "--upstream-lengths",
            str(V02_PRESET["parameters"]["upstream_lengths"]),
            "--downstream-lengths",
            str(V02_PRESET["parameters"]["downstream_lengths"]),
            "--side-y-spans",
            str(V02_PRESET["parameters"]["side_y_spans"]),
            "--side-z-spans",
            str(V02_PRESET["parameters"]["side_z_spans"]),
            "--min-farfield-padding",
            str(V02_PRESET["parameters"]["min_farfield_padding"]),
            "--farfield-size-policy",
            str(V02_PRESET["parameters"]["farfield_size_policy"]),
            "--farfield-size-fraction",
            str(V02_PRESET["parameters"]["farfield_size_fraction"]),
            "--min-farfield-size",
            str(V02_PRESET["parameters"]["min_farfield_size"]),
            "--max-farfield-size",
            str(V02_PRESET["parameters"]["max_farfield_size"]),
            "--velocity",
            "22.352,0,0",
        ]
        commands.append(run_logged(cmd, variant_dir / "log.run_gated_mesh_smoke.txt"))

    case_dir = mesh_dir / "gmsh" / "openfoam_case"
    setup_cmd = [
        sys.executable,
        str(scripts_dir / "setup_potential_foam_smoke.py"),
        "--case-dir",
        str(case_dir),
        "--velocity",
        "22.352,0,0",
    ]
    commands.append(run_logged(setup_cmd, variant_dir / "log.setup_potential_foam.txt"))

    potential_log = case_dir / "log.potentialFoam"
    potential_script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{windows_path_to_wsl(case_dir.resolve())}' && "
        "potentialFoam -writep > log.potentialFoam 2>&1"
    )
    commands.append(
        run_logged(["wsl", "bash", "-lc", potential_script], variant_dir / "log.potentialFoam_driver.txt")
    )
    summarize_cmd = [
        sys.executable,
        str(scripts_dir / "summarize_potential_foam_smoke.py"),
        "--log",
        str(potential_log),
        "--report",
        str(case_dir / "potential_summary.json"),
    ]
    commands.append(run_logged(summarize_cmd, variant_dir / "log.summarize_potential_foam.txt"))

    screenshots_dir = variant_dir / "screenshots"
    render_cmd = [
        sys.executable,
        str(scripts_dir / "render_stl_view_set.py"),
        "--input-stl",
        str(mesh_dir / "gmsh" / "aircraft_prepared_for_gmsh.stl"),
        "--output-dir",
        str(screenshots_dir),
        "--edges",
    ]
    commands.append(run_logged(render_cmd, variant_dir / "log.render_view_set.txt"))

    pipeline = load_json(mesh_dir / "gmsh" / "pipeline_report.json")
    aircraft_bounds = np.asarray(pipeline["gmsh_report"]["farfield_domain"]["aircraft_bounds"], dtype=float)
    closeups = render_feature_closeups(
        mesh_dir / "gmsh" / "aircraft_prepared_for_gmsh.stl",
        variant_dir / "feature_closeups",
        width=1600,
        height=1200,
    )
    nonortho = run_and_localize_nonortho(
        case_dir=case_dir,
        aircraft_stl=mesh_dir / "gmsh" / "aircraft_prepared_for_gmsh.stl",
        aircraft_bounds=aircraft_bounds,
        out_dir=variant_dir / "nonortho",
        width=1600,
        height=1200,
    )

    result = collect_variant_result(
        variant_id=variant_id,
        input_stl=input_stl,
        variant_dir=variant_dir,
        mesh_dir=mesh_dir,
        commands=commands,
        closeups=closeups,
        nonortho=nonortho,
    )
    return result


def collect_variant_result(
    *,
    variant_id: str,
    input_stl: Path,
    variant_dir: Path,
    mesh_dir: Path,
    commands: list[dict[str, Any]],
    closeups: dict[str, Any],
    nonortho: dict[str, Any],
) -> dict[str, Any]:
    runner = load_json(mesh_dir / "runner_report.json")
    pipeline = load_json(mesh_dir / "gmsh" / "pipeline_report.json")
    remesh = load_json(mesh_dir / "remesh" / "remesh_report.json")
    mesh_gate = load_json(mesh_dir / "mesh_gate.json")
    potential = load_json(mesh_dir / "gmsh" / "openfoam_case" / "potential_summary.json")
    openfoam = pipeline.get("openfoam") or {}
    gmsh_report = pipeline.get("gmsh_report") or {}
    prepared = gmsh_report.get("prepared_surface") or {}
    check_text = (mesh_dir / "gmsh" / "openfoam_case" / "log.checkMesh").read_text(
        encoding="utf-8",
        errors="replace",
    )
    boundary = parse_boundary_faces(mesh_dir / "gmsh" / "openfoam_case" / "constant" / "polyMesh" / "boundary")
    return {
        "variant_id": variant_id,
        "preset": V02_PRESET["name"],
        "input_stl": str(input_stl),
        "run_dir": str(variant_dir),
        "case_dir": str(mesh_dir / "gmsh" / "openfoam_case"),
        "mesh_format_path": str(mesh_dir / "gmsh" / "mesh.msh"),
        "su2_mesh_path": str(mesh_dir / "gmsh" / "mesh.su2"),
        "commands": commands + runner.get("commands", []),
        "runtime_s": runner.get("runtime_s"),
        "cells": openfoam.get("cells"),
        "points": openfoam.get("points"),
        "aircraft_patch_faces": prepared.get("faces"),
        "farfield_patch_faces": sum(
            int(boundary.get(name, {}).get("nFaces", 0))
            for name in ("inlet", "outlet", "side_ymin", "side_ymax", "side_zmin", "side_zmax")
        ),
        "patch_names": sorted(boundary),
        "min_cell_volume": parse_checkmesh_value(check_text, r"Min volume = ([0-9.eE+\-.]+)"),
        "max_cell_volume": parse_checkmesh_value(check_text, r"Max volume = ([0-9.eE+\-.]+)"),
        "max_skewness": openfoam.get("max_skewness"),
        "max_non_orthogonality": openfoam.get("max_non_orthogonality"),
        "severe_nonorth_faces": nonortho.get("nonortho_face_count"),
        "strict_checkmesh_pass": bool(openfoam.get("mesh_ok")),
        "mesh_gate_pass": bool(mesh_gate.get("pass_gate")),
        "potential_foam": potential,
        "surface_fidelity_mm": remesh.get("surface_fidelity_mm", {}).get("bidirectional", {}),
        "remesh_report": str(mesh_dir / "remesh" / "remesh_report.json"),
        "mesh_gate_report": str(mesh_dir / "mesh_gate.json"),
        "pipeline_report": str(mesh_dir / "gmsh" / "pipeline_report.json"),
        "aircraft_iso_screenshot": str(screenshots_path(variant_dir, "iso")),
        "screenshots_dir": str(variant_dir / "screenshots"),
        "feature_closeups": closeups,
        "nonortho_localization": nonortho,
        "verdict": variant_verdict(openfoam, mesh_gate, potential),
    }


def variant_verdict(openfoam: dict[str, Any], mesh_gate: dict[str, Any], potential: dict[str, Any]) -> str:
    if not openfoam.get("mesh_ok") or not mesh_gate.get("pass_gate"):
        return "fail_openfoam_quality"
    if not potential.get("completed"):
        return "fail_solver_smoke"
    return "pass_for_mesher_plumbing"


def load_baseline(root: Path) -> list[dict[str, Any]]:
    comparison = load_json(root / "comparison_summary.json")
    reports = comparison.get("reports", [])
    followup = root / "followup_gmsh_geometry_nonortho"
    output = []
    for report in reports:
        variant_id = report["variant_id"]
        non_path = followup / variant_id / "nonortho" / "nonortho_localization_summary.json"
        non = load_json(non_path) if non_path.exists() else {}
        output.append(
            {
                "variant_id": variant_id,
                "preset": "gmsh_openfoam_external_aero_v0_1_baseline",
                "input_stl": report.get("input_stl"),
                "run_dir": report.get("run_dir"),
                "runtime_s": report.get("runtime_s"),
                "cells": report.get("cells"),
                "points": report.get("points"),
                "aircraft_patch_faces": report.get("aircraft_patch_faces"),
                "farfield_patch_faces": report.get("farfield_patch_faces"),
                "max_skewness": report.get("max_skewness"),
                "max_non_orthogonality": report.get("max_non_orthogonality"),
                "severe_nonorth_faces": report.get("severe_nonorth_faces"),
                "strict_checkmesh_pass": report.get("strict_checkmesh_verdict") == "pass",
                "potential_foam": report.get("potential_foam"),
                "surface_fidelity_mm": (
                    report.get("surface_deviation_metrics", {})
                    .get("source_to_remesh_mm", {})
                    .get("bidirectional", {})
                ),
                "nonortho_localization": non,
                "verdict": report.get("verdict"),
            }
        )
    return output


def recommendation(baseline: list[dict[str, Any]], candidate: list[dict[str, Any]]) -> dict[str, Any]:
    all_pass = all(row.get("verdict") == "pass_for_mesher_plumbing" for row in candidate)
    baseline_severe = sum(int(row.get("severe_nonorth_faces") or 0) for row in baseline)
    candidate_severe = sum(int(row.get("severe_nonorth_faces") or 0) for row in candidate)
    baseline_runtime = sum(float(row.get("runtime_s") or 0.0) for row in baseline)
    candidate_runtime = sum(float(row.get("runtime_s") or 0.0) for row in candidate)
    if all_pass and candidate_severe < baseline_severe:
        decision = "adopt_conditional_preset"
        reason = (
            "Candidate passes strict checkMesh and potentialFoam on all five variants "
            "and reduces aggregate severe non-orthogonal faces, but remains non-scoring due to no boundary layers."
        )
    elif all_pass:
        decision = "keep_current_preset"
        reason = (
            "Candidate passes all plumbing gates but does not improve severe non-orthogonality enough to replace "
            "the current baseline."
        )
    else:
        decision = "keep_current_preset"
        reason = "Candidate did not pass all required gates."
    return {
        "decision": decision,
        "reason": reason,
        "baseline_total_severe_nonorth_faces": baseline_severe,
        "candidate_total_severe_nonorth_faces": candidate_severe,
        "baseline_total_runtime_s": baseline_runtime,
        "candidate_total_runtime_s": candidate_runtime,
        "scoring_allowed": False,
    }


def write_outputs(run_root: Path, matrix: dict[str, Any]) -> None:
    (run_root / "gmsh_optimizer_preset_v0_2.json").write_text(
        json.dumps(V02_PRESET, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    (run_root / "comparison_matrix.json").write_text(
        json.dumps(matrix, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    write_csv(run_root / "comparison_matrix.csv", matrix)
    (run_root / "GMSH_NEXT_STEP_REPORT.md").write_text(render_report(matrix), encoding="utf-8")


def write_csv(path: Path, matrix: dict[str, Any]) -> None:
    rows = []
    for group_name, group in (("baseline", matrix["baseline_results"]), ("candidate", matrix["candidate_results"])):
        for row in group:
            fidelity = row.get("surface_fidelity_mm") or {}
            potential = row.get("potential_foam") or {}
            rows.append(
                {
                    "group": group_name,
                    "preset": row.get("preset"),
                    "variant_id": row.get("variant_id"),
                    "verdict": row.get("verdict"),
                    "runtime_s": row.get("runtime_s"),
                    "cells": row.get("cells"),
                    "points": row.get("points"),
                    "aircraft_patch_faces": row.get("aircraft_patch_faces"),
                    "farfield_patch_faces": row.get("farfield_patch_faces"),
                    "max_skewness": row.get("max_skewness"),
                    "max_non_orthogonality": row.get("max_non_orthogonality"),
                    "severe_nonorth_faces": row.get("severe_nonorth_faces"),
                    "strict_checkmesh_pass": row.get("strict_checkmesh_pass"),
                    "potential_completed": potential.get("completed"),
                    "potential_continuity_error": potential.get("continuity_error"),
                    "potential_interpolated_velocity_error": potential.get("interpolated_velocity_error"),
                    "potential_last_phi_final_residual": potential.get("last_phi_final_residual"),
                    "potential_last_p_final_residual": potential.get("last_p_final_residual"),
                    "surface_p95_mm": fidelity.get("p95"),
                    "surface_p99_mm": fidelity.get("p99"),
                    "surface_max_mm": fidelity.get("max"),
                }
            )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def render_report(matrix: dict[str, Any]) -> str:
    lines = [
        "# GMSH Next Step Report",
        "",
        "## Verdict",
        "",
        f"Decision: `{matrix['recommendation']['decision']}`.",
        "",
        matrix["recommendation"]["reason"],
        "",
        "This remains OpenFOAM mesher/plumbing quality only. No boundary-layer/prism mesh exists yet, and no lift/drag scoring is claimed.",
        "",
        "## Preset",
        "",
        f"Recommended preset file: `gmsh_optimizer_preset_v0_2.json`",
        "",
        "```json",
        json.dumps(V02_PRESET["parameters"], indent=2, sort_keys=True),
        "```",
        "",
        "## Comparison Matrix",
        "",
        "| Variant | Baseline severe | v0.2 severe | Baseline cells | v0.2 cells | v0.2 checkMesh | v0.2 potentialFoam | v0.2 runtime s |",
        "|---|---:|---:|---:|---:|---|---|---:|",
    ]
    baseline_by_id = {row["variant_id"]: row for row in matrix["baseline_results"]}
    for row in matrix["candidate_results"]:
        base = baseline_by_id[row["variant_id"]]
        potential = row.get("potential_foam") or {}
        lines.append(
            f"| `{row['variant_id']}` | {base.get('severe_nonorth_faces')} | {row.get('severe_nonorth_faces')} | "
            f"{base.get('cells'):,} | {row.get('cells'):,} | {row.get('strict_checkmesh_pass')} | "
            f"{potential.get('completed')} | {float(row.get('runtime_s') or 0.0):.1f} |"
        )
    lines.extend(
        [
            "",
            "## Non-Orthogonality Localization",
            "",
            "Each candidate variant writes `nonOrthoFaces.vtk`, `nonOrthoFace_centroids.csv`, top/ISO overlays, and a JSON localization summary under `<preset>/<variant>/nonortho/`.",
            "",
            "| Variant | Severe >70 | Max non-orth | Dominant location | Worst-region bbox note |",
            "|---|---:|---:|---|---|",
        ]
    )
    for row in matrix["candidate_results"]:
        non = row.get("nonortho_localization") or {}
        regions = non.get("feature_regions") or {}
        dominant = non.get("dominant_location")
        bbox = regions.get(dominant, {}).get("centroid_bounds_m") if dominant else None
        lines.append(
            f"| `{row['variant_id']}` | {row.get('severe_nonorth_faces')} | "
            f"{float(row.get('max_non_orthogonality') or 0.0):.2f} | {dominant} | `{bbox}` |"
        )
    lines.extend(
        [
            "",
            "## OpenFOAM Plumbing",
            "",
            "All candidate cases must keep `potentialFoam -writep` as the required smoke test. These runs use the same 50 mph freestream and split farfield patches. The aircraft wall is still not a scoring wall treatment.",
            "",
            "## First Scoring-CFD Step Draft",
            "",
            "- Solver: start with `simpleFoam` for steady incompressible RANS coefficient plumbing. Use `pimpleFoam` only if convergence or unsteady behavior requires it.",
            "- Turbulence model: `kOmegaSST` as the first robust default for small-UAV external flow.",
            "- Aircraft wall: `noSlip` for scoring attempts. Current slip/plumbing cases are not scoring CFD.",
            "- Upstream/farfield inlet: fixed freestream velocity `(22.352 0 0)`, zero-gradient pressure, and fixed/inlet turbulence quantities.",
            "- Downstream outlet: zero-gradient velocity, fixed reference pressure, and outlet-compatible turbulence quantities.",
            "- Side/top/bottom farfield: freestream-style BCs or slip/symmetry for early plumbing; freestream/open boundary behavior is preferred for coefficient work.",
            "- Reference policy: store `Aref`, `Lref`, `CofR`, lift axis `+Z`, drag axis `+X`, pitch axis `+Y` per candidate from geometry metadata.",
            "- Force reporting: use `forceCoeffs` plus raw `forces`, with residuals, continuity, coefficient histories, and final-window averages.",
            "- Convergence checks: residual reduction, bounded continuity errors, stable force coefficients over a final time window, no floating-point exceptions, and no runaway turbulence variables.",
            "- Mesh blocker: trusted drag/lift requires a boundary-layer/prism strategy and wall-treatment/y+ policy. Without layers, pressure trends may be useful but drag is not scoring-grade.",
            "",
            "## Automation Gates",
            "",
            "- Fail fast on source topology failure, Gmsh crash, missing patches, negative/zero cell volume, strict `checkMesh` failure, severe farfield-transition clusters, excessive surface deviation, or `potentialFoam` fatal errors.",
            "- Store tool versions, STL identity, preset name/settings, runtime, cell/point/patch counts, min/max volume, skew/non-orth metrics, severe-face localization, surface fidelity, solver residuals, warnings, screenshots, VTK/CSV artifact paths, and final verdict in the optimizer database.",
            "- Keep geometry-capture acceptance and scoring-CFD promotion human-review-only until no-slip RANS and boundary-layer validation are working.",
        ]
    )
    return "\n".join(lines)


def run_logged(command: list[str], log_path: Path) -> dict[str, Any]:
    started = time.perf_counter()
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(
        "$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr,
        encoding="utf-8",
    )
    record = {
        "command": command,
        "log": str(log_path),
        "returncode": result.returncode,
        "runtime_s": time.perf_counter() - started,
    }
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {result.returncode}; see {log_path}")
    return record


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_variant_report(path: Path, report: dict[str, Any]) -> None:
    path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")


def write_variant_report_md(path: Path, report: dict[str, Any]) -> None:
    non = report.get("nonortho_localization") or {}
    potential = report.get("potential_foam") or {}
    fidelity = report.get("surface_fidelity_mm") or {}
    lines = [
        f"# {report['variant_id']} Mesh Quality Report",
        "",
        f"- Verdict: `{report['verdict']}`",
        f"- Cells: `{report['cells']}`",
        f"- Points: `{report['points']}`",
        f"- Aircraft patch faces: `{report['aircraft_patch_faces']}`",
        f"- Farfield patch faces: `{report['farfield_patch_faces']}`",
        f"- Max skewness: `{report['max_skewness']}`",
        f"- Max non-orthogonality: `{report['max_non_orthogonality']}`",
        f"- Severe non-orthogonal faces >70 deg: `{report['severe_nonorth_faces']}`",
        f"- Dominant non-orth location: `{non.get('dominant_location')}`",
        f"- PotentialFoam complete: `{potential.get('completed')}`",
        f"- Continuity error: `{potential.get('continuity_error')}`",
        f"- Interpolated velocity error: `{potential.get('interpolated_velocity_error')}`",
        f"- Surface fidelity p95/p99/max mm: `{fidelity.get('p95')}` / `{fidelity.get('p99')}` / `{fidelity.get('max')}`",
        f"- Aircraft ISO screenshot: `{report['aircraft_iso_screenshot']}`",
        f"- Non-orth VTK: `{non.get('nonortho_vtk')}`",
        f"- Non-orth centroid CSV: `{non.get('nonortho_centroid_csv')}`",
        f"- Non-orth overlay: `{non.get('overlay_iso')}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_checkmesh_value(text: str, pattern: str) -> float | int | None:
    import re

    match = re.search(pattern, text)
    if not match:
        return None
    raw = match.group(1).rstrip(".")
    return float(raw) if any(char in raw for char in ".eE") else int(raw)


def parse_boundary_faces(path: Path) -> dict[str, dict[str, int | str]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = [line.strip() for line in text.splitlines()]
    patches: dict[str, dict[str, int | str]] = {}
    i = 0
    while i < len(lines):
        name = lines[i]
        if i + 1 < len(lines) and lines[i + 1] == "{":
            patch: dict[str, int | str] = {}
            i += 2
            while i < len(lines) and lines[i] != "}":
                parts = lines[i].rstrip(";").split()
                if len(parts) >= 2:
                    try:
                        value: int | str = int(parts[1])
                    except ValueError:
                        value = parts[1]
                    patch[parts[0]] = value
                i += 1
            patches[name] = patch
        i += 1
    return patches


def screenshots_path(variant_dir: Path, name: str) -> Path:
    return variant_dir / "screenshots" / f"{name}.png"


if __name__ == "__main__":
    main()
