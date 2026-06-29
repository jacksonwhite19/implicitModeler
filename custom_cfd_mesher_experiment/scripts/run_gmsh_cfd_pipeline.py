from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--target-faces", type=int, default=8000)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--padding", default="0.7,0.5,0.5")
    parser.add_argument("--farfield-policy", choices=["fixed-padding", "dynamic"], default="dynamic")
    parser.add_argument("--upstream-lengths", type=float, default=1.5)
    parser.add_argument("--downstream-lengths", type=float, default=3.0)
    parser.add_argument("--side-y-spans", type=float, default=2.0)
    parser.add_argument("--side-z-spans", type=float, default=2.0)
    parser.add_argument("--min-farfield-padding", type=float, default=0.15)
    parser.add_argument("--surface-size", type=float, default=0.008)
    parser.add_argument("--feature-size", type=float, default=0.0)
    parser.add_argument("--feature-distance-min", type=float, default=0.0)
    parser.add_argument("--feature-distance-max", type=float, default=0.0)
    parser.add_argument("--refinement-boxes", default="")
    parser.add_argument("--farfield-size", type=float, default=0.12)
    parser.add_argument("--farfield-size-policy", choices=["fixed", "dynamic"], default="dynamic")
    parser.add_argument("--farfield-size-fraction", type=float, default=0.15)
    parser.add_argument("--min-farfield-size", type=float, default=0.04)
    parser.add_argument("--max-farfield-size", type=float, default=0.35)
    parser.add_argument(
        "--farfield-patches",
        choices=["single", "split"],
        default="single",
    )
    parser.add_argument("--angle-deg", type=float, default=40.0)
    parser.add_argument("--curve-angle-deg", type=float, default=180.0)
    parser.add_argument("--algorithm3d", type=int, default=10)
    parser.add_argument("--geometry-mode", default="create-topology")
    parser.add_argument("--optimize", default="default,Netgen,Relocate3D")
    parser.add_argument("--fidelity-samples", type=int, default=25000)
    parser.add_argument("--skip-openfoam", action="store_true")
    parser.add_argument(
        "--gmsh-to-foam-keep-orientation",
        action="store_true",
        help="Pass -keepOrientation to gmshToFoam for prism/hex-containing Gmsh meshes.",
    )
    args = parser.parse_args()

    started = time.perf_counter()
    scripts_dir = Path(__file__).resolve().parent
    args.run_dir.mkdir(parents=True, exist_ok=True)
    step_times: dict[str, float] = {}

    gmsh_cmd = [
        sys.executable,
        str(scripts_dir / "gmsh_external_flow_mesher.py"),
        "--input-stl",
        str(args.input_stl),
        "--run-dir",
        str(args.run_dir),
        "--scale",
        str(args.scale),
        "--target-faces",
        str(args.target_faces),
        "--padding",
        args.padding,
        "--farfield-policy",
        args.farfield_policy,
        "--upstream-lengths",
        str(args.upstream_lengths),
        "--downstream-lengths",
        str(args.downstream_lengths),
        "--side-y-spans",
        str(args.side_y_spans),
        "--side-z-spans",
        str(args.side_z_spans),
        "--min-farfield-padding",
        str(args.min_farfield_padding),
        "--surface-size",
        str(args.surface_size),
        "--feature-size",
        str(args.feature_size),
        "--feature-distance-min",
        str(args.feature_distance_min),
        "--feature-distance-max",
        str(args.feature_distance_max),
        "--refinement-boxes",
        args.refinement_boxes,
        "--farfield-size",
        str(args.farfield_size),
        "--farfield-size-policy",
        args.farfield_size_policy,
        "--farfield-size-fraction",
        str(args.farfield_size_fraction),
        "--min-farfield-size",
        str(args.min_farfield_size),
        "--max-farfield-size",
        str(args.max_farfield_size),
        "--farfield-patches",
        args.farfield_patches,
        "--angle-deg",
        str(args.angle_deg),
        "--curve-angle-deg",
        str(args.curve_angle_deg),
        "--algorithm3d",
        str(args.algorithm3d),
        "--geometry-mode",
        args.geometry_mode,
        "--optimize",
        args.optimize,
    ]
    step_times["gmsh_mesh_s"] = timed_run_logged(gmsh_cmd, args.run_dir / "log.pipeline.gmsh.txt")

    su2_report = args.run_dir / "su2_validation.json"
    su2_cmd = [
        sys.executable,
        str(scripts_dir / "validate_su2_mesh.py"),
        "--mesh",
        str(args.run_dir / "mesh.su2"),
        "--report",
        str(su2_report),
    ]
    step_times["su2_validation_s"] = timed_run_logged(su2_cmd, args.run_dir / "log.pipeline.su2_validation.txt")

    fidelity_report = args.run_dir / "surface_fidelity.json"
    fidelity_cmd = [
        sys.executable,
        str(scripts_dir / "surface_fidelity_audit.py"),
        "--reference-stl",
        str(args.input_stl),
        "--candidate-stl",
        str(args.run_dir / "aircraft_prepared_for_gmsh.stl"),
        "--report",
        str(fidelity_report),
        "--scale",
        str(args.scale),
        "--samples",
        str(args.fidelity_samples),
    ]
    step_times["surface_fidelity_s"] = timed_run_logged(
        fidelity_cmd,
        args.run_dir / "log.pipeline.surface_fidelity.txt",
    )

    openfoam_report = None
    if not args.skip_openfoam:
        openfoam_started = time.perf_counter()
        openfoam_report = run_openfoam_checks(
            args.run_dir / "openfoam_case",
            keep_orientation=args.gmsh_to_foam_keep_orientation,
        )
        step_times["openfoam_conversion_check_s"] = time.perf_counter() - openfoam_started

    screenshot = args.run_dir / "aircraft_iso.png"
    render_cmd = [
        sys.executable,
        str(scripts_dir / "render_stl_iso_screenshot.py"),
        "--input-stl",
        str(args.run_dir / "aircraft_prepared_for_gmsh.stl"),
        "--output-png",
        str(screenshot),
        "--edges",
    ]
    step_times["render_s"] = timed_run_logged(render_cmd, args.run_dir / "log.pipeline.render.txt")
    step_times["total_s"] = time.perf_counter() - started

    report = {
        "status": "ready" if report_passed(su2_report, openfoam_report, screenshot) else "failed",
        "input_stl": str(args.input_stl),
        "run_dir": str(args.run_dir),
        "gmsh_report": load_json(args.run_dir / "gmsh_report.json"),
        "su2_validation": load_json(su2_report),
        "surface_fidelity": load_json(fidelity_report),
        "openfoam": openfoam_report,
        "step_times_s": step_times,
        "tool_versions": collect_versions(),
        "screenshot": str(screenshot),
    }
    (args.run_dir / "pipeline_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    if report["status"] != "ready":
        raise SystemExit(1)


def timed_run_logged(command: list[str], log_path: Path) -> float:
    started = time.perf_counter()
    run_logged(command, log_path)
    return time.perf_counter() - started


def run_logged(command: list[str], log_path: Path) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.write_text(
        "$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr,
        encoding="utf-8",
    )
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {result.returncode}; see {log_path}")
    return result


def run_openfoam_checks(case_dir: Path, *, keep_orientation: bool = False) -> dict[str, object]:
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    gmsh_to_foam = "gmshToFoam -keepOrientation ../mesh.msh" if keep_orientation else "gmshToFoam ../mesh.msh"
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf constant/polyMesh VTK log.gmshToFoam log.checkMesh log.foamToVTK && "
        f"{gmsh_to_foam} > log.gmshToFoam 2>&1 && "
        "checkMesh > log.checkMesh 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (case_dir / "log.pipeline.openfoam_stdout.txt").write_text(
        result.stdout + result.stderr,
        encoding="utf-8",
    )
    check_log = case_dir / "log.checkMesh"
    summary = parse_check_mesh(check_log.read_text(encoding="utf-8", errors="replace") if check_log.exists() else "")
    summary["returncode"] = result.returncode
    summary["case_dir"] = str(case_dir)
    summary["gmsh_to_foam_keep_orientation"] = keep_orientation
    summary["gmsh_to_foam_log"] = str(case_dir / "log.gmshToFoam")
    summary["check_mesh_log"] = str(check_log)
    if result.returncode != 0:
        summary["status"] = "failed"
        return summary
    summary["status"] = "ready" if summary.get("mesh_ok") else "failed"
    return summary


def collect_versions() -> dict[str, object]:
    return {
        "python": run_version([sys.executable, "--version"]),
        "gmsh_python_api": run_version([sys.executable, "-c", "import gmsh; print(gmsh.__version__)"]),
        "trimesh": run_version([sys.executable, "-c", "import trimesh; print(trimesh.__version__)"]),
        "openfoam": run_version(["wsl", "bash", "-lc", "source /opt/openfoam13/etc/bashrc && foamVersion"]),
    }


def run_version(command: list[str]) -> str:
    result = subprocess.run(command, text=True, capture_output=True)
    text = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        return f"unavailable: {text}"
    return text


def parse_check_mesh(text: str) -> dict[str, object]:
    def number(pattern: str) -> float | int | None:
        match = re.search(pattern, text)
        if not match:
            return None
        raw = match.group(1)
        return float(raw) if any(char in raw for char in ".eE") else int(raw)

    return {
        "mesh_ok": "Mesh OK." in text,
        "points": number(r"points:\s+(\d+)"),
        "faces": number(r"faces:\s+(\d+)"),
        "internal_faces": number(r"internal faces:\s+(\d+)"),
        "cells": number(r"cells:\s+(\d+)"),
        "tetrahedra": number(r"tetrahedra:\s+(\d+)"),
        "prisms": number(r"prisms:\s+(\d+)"),
        "pyramids": number(r"pyramids:\s+(\d+)"),
        "polyhedra": number(r"polyhedra:\s+(\d+)"),
        "boundary_patches": number(r"boundary patches:\s+(\d+)"),
        "max_aspect_ratio": number(r"Max aspect ratio = ([0-9.eE+-]+)"),
        "high_aspect_cells": number(r"Max aspect ratio: [0-9.eE+-]+, number of cells (\d+)"),
        "invalid_vertex_faces": number(r"Faces with invalid vertex labels found,\s+number of faces: (\d+)"),
        "zero_area_faces": number(r"Zero or negative face area detected\.[\s\S]*?Writing (\d+) zero area faces"),
        "negative_volume_cells": number(r"Number of negative volume cells: (\d+)"),
        "max_non_orthogonality": number(r"Mesh non-orthogonality Max: ([0-9.eE+-]+)"),
        "non_orthogonality_errors": number(r"Number of non-orthogonality errors: (\d+)"),
        "severely_non_orthogonal_faces": number(r"Number of severely non-orthogonal \(> 70 degrees\) faces: (\d+)"),
        "wrong_oriented_faces": number(r"Error in face pyramids: (\d+) faces are incorrectly oriented"),
        "max_skewness": number(r"Max skewness = ([0-9.eE+-]+)"),
        "highly_skew_faces": number(r"Max skewness = [0-9.eE+-]+, (\d+) highly skew faces"),
        "failed_checks": number(r"Failed (\d+) mesh checks"),
    }


def windows_path_to_wsl(path: Path) -> str:
    text = str(path)
    drive, rest = text[0], text[2:]
    rest = rest.replace("\\", "/")
    return f"/mnt/{drive.lower()}{rest}"


def report_passed(su2_report: Path, openfoam_report: dict[str, object] | None, screenshot: Path) -> bool:
    su2 = load_json(su2_report)
    if not su2.get("valid"):
        return False
    if openfoam_report is not None and not openfoam_report.get("mesh_ok"):
        return False
    return screenshot.exists() and screenshot.stat().st_size > 0


def load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
