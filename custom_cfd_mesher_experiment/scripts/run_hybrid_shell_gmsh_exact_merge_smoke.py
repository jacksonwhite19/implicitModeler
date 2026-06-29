from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from run_gmsh_cfd_pipeline import parse_check_mesh, windows_path_to_wsl


VARIANTS = {
    "fcv01_long_glider": Path(
        "custom_cfd_mesher_experiment/runs/direct_prism_shell_five_variant_v0_1_20260624_fcv01_remesh_tuning/fd45/aircraft_surface_fd45.stl"
    ),
    "fcv02_short_swept": Path(
        "custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv02_short_swept/base_remesh/aircraft_surface_base_4mm_fd25.stl"
    ),
    "fcv03_high_aspect_mild": Path(
        "custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv03_high_aspect_mild/selected_remesh/aircraft_surface_fd45.stl"
    ),
    "fcv04_compact_wide_tail": Path(
        "custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv04_compact_wide_tail/selected_remesh/aircraft_surface_fd15.stl"
    ),
    "fcv05_aft_wing_fast": Path(
        "custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv05_aft_wing_fast/base_remesh/aircraft_surface_base_4mm_fd25.stl"
    ),
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--variant", choices=sorted(VARIANTS), action="append")
    parser.add_argument("--velocity", default="22.352,0,0")
    parser.add_argument("--surface-size", type=float, default=0.004)
    parser.add_argument("--farfield-size", type=float, default=0.11)
    parser.add_argument("--farfield-patches", choices=["single", "split"], default="single")
    parser.add_argument("--optimize", default="default,Netgen")
    parser.add_argument("--match-tolerance", type=float, default=5e-8)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    scripts = Path(__file__).resolve().parent
    args.run_root.mkdir(parents=True, exist_ok=True)

    selected = args.variant or sorted(VARIANTS)
    results = []
    for variant in selected:
        started = time.perf_counter()
        variant_dir = args.run_root / variant
        shell_dir = variant_dir / "shell"
        outer_dir = variant_dir / "outer"
        merged_case = variant_dir / "merged_openfoam_case"
        variant_dir.mkdir(parents=True, exist_ok=True)

        input_stl = (repo_root / VARIANTS[variant]).resolve()
        commands: list[list[str]] = []

        shell_cmd = [
            sys.executable,
            str(scripts / "direct_prism_shell_mesher.py"),
            "--input-stl",
            str(input_stl),
            "--run-dir",
            str(shell_dir),
            "--scale",
            "0.001",
            "--offsets",
            "0.0002,0.0005,0.001",
            "--adaptive-offset-caps",
            "--offset-cap-factor",
            "0.25",
            "--write-openfoam",
        ]
        commands.append(shell_cmd)
        run_logged(shell_cmd, variant_dir / "log.shell.txt")

        outer_cmd = [
            sys.executable,
            str(scripts / "gmsh_external_flow_mesher.py"),
            "--input-stl",
            str(shell_dir / "surfaces" / "outer_shell.stl"),
            "--run-dir",
            str(outer_dir),
            "--scale",
            "1.0",
            "--target-faces",
            "0",
            "--padding",
            "1.14003,1.6,0.38598",
            "--farfield-policy",
            "fixed-padding",
            "--surface-size",
            str(args.surface_size),
            "--farfield-size",
            str(args.farfield_size),
            "--farfield-patches",
            args.farfield_patches,
            "--angle-deg",
            "80",
            "--curve-angle-deg",
            "180",
            "--algorithm3d",
            "4",
            "--geometry-mode",
            "create-topology",
            "--optimize",
            args.optimize,
        ]
        commands.append(outer_cmd)
        run_logged(outer_cmd, variant_dir / "log.outer_gmsh.txt")
        run_openfoam_gmsh_to_foam(outer_dir / "openfoam_case", variant_dir / "log.outer_gmshToFoam_driver.txt")

        merge_cmd = [
            sys.executable,
            str(scripts / "merge_prism_shell_gmsh_outer.py"),
            "--shell-case",
            str(shell_dir / "openfoam_case"),
            "--outer-case",
            str(outer_dir / "openfoam_case"),
            "--output-case",
            str(merged_case),
            "--match-tolerance",
            str(args.match_tolerance),
        ]
        if args.farfield_patches == "split":
            merge_cmd.extend(["--outer-boundary-patches", "auto"])
        commands.append(merge_cmd)
        run_logged(merge_cmd, variant_dir / "log.exact_merge.txt")

        setup_cmd = [
            sys.executable,
            str(scripts / "setup_potential_foam_smoke.py"),
            "--case-dir",
            str(merged_case),
            "--velocity",
            args.velocity,
        ]
        commands.append(setup_cmd)
        run_logged(setup_cmd, variant_dir / "log.setup_potential.txt")

        check_summary = run_check_mesh(merged_case)
        potential_summary = run_potential_foam(merged_case)
        elapsed = time.perf_counter() - started

        result = {
            "variant": variant,
            "input_stl": str(input_stl),
            "variant_dir": str(variant_dir),
            "shell_dir": str(shell_dir),
            "outer_dir": str(outer_dir),
            "merged_case": str(merged_case),
            "commands": commands,
            "check_mesh": check_summary,
            "potential_foam": potential_summary,
            "runtime_s": elapsed,
            "status": "ready"
            if check_summary.get("mesh_ok") and potential_summary.get("completed")
            else "failed",
        }
        (variant_dir / "hybrid_result.json").write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
        results.append(result)

    summary = {"run_root": str(args.run_root), "results": results}
    (args.run_root / "hybrid_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if any(item["status"] != "ready" for item in results):
        raise SystemExit(1)


def run_logged(command: list[str], log_path: Path) -> None:
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.write_text("$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {result.returncode}; see {log_path}")


def run_openfoam_gmsh_to_foam(case_dir: Path, log_path: Path) -> None:
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf constant/polyMesh log.gmshToFoam && "
        "gmshToFoam ../mesh.msh > log.gmshToFoam 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"gmshToFoam failed; see {log_path}")


def run_check_mesh(case_dir: Path) -> dict[str, object]:
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf postProcessing VTK constant/polyMesh/sets log.checkMesh && "
        "checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk > log.checkMesh 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (case_dir / "log.checkMesh_driver.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
    log = case_dir / "log.checkMesh"
    text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else ""
    summary = parse_check_mesh(text)
    summary["returncode"] = result.returncode
    summary["log"] = str(log)
    return summary


def run_potential_foam(case_dir: Path) -> dict[str, object]:
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf postProcessing 1 log.potentialFoam && "
        "potentialFoam -writep > log.potentialFoam 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (case_dir / "log.potentialFoam_driver.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
    log = case_dir / "log.potentialFoam"
    text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else ""
    return {
        "returncode": result.returncode,
        "completed": result.returncode == 0 and "End" in text,
        "log": str(log),
        "continuity_error": extract_float(text, "Continuity error = "),
        "interpolated_velocity_error": extract_float(text, "Interpolated velocity error = "),
    }


def extract_float(text: str, prefix: str) -> float | None:
    for line in text.splitlines():
        if prefix in line:
            try:
                return float(line.split(prefix, 1)[1].strip().split()[0])
            except ValueError:
                return None
    return None


if __name__ == "__main__":
    main()
