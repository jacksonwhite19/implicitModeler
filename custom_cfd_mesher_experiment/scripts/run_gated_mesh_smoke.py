from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the gated candidate mesh and optional solver smoke pipeline.")
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--skip-solver", action="store_true")
    parser.add_argument(
        "--prep-source-cap-loop-span-mm",
        type=float,
        default=None,
        help="Optional source-prep cap limit. Default keeps strict no-repair source gating.",
    )
    parser.add_argument("--target-edge-mm", type=float, default=7.2)
    parser.add_argument("--remesh-iterations", type=int, default=5)
    parser.add_argument("--feature-deg", type=float, default=25.0)
    parser.add_argument("--max-surface-distance-mm", type=float, default=0.25)
    parser.add_argument("--smooth-remesh", action="store_true")
    parser.add_argument("--surface-size", type=float, default=0.0072)
    parser.add_argument("--farfield-size", type=float, default=0.11)
    parser.add_argument("--farfield-policy", choices=["fixed-padding", "dynamic"], default="dynamic")
    parser.add_argument("--upstream-lengths", type=float, default=1.5)
    parser.add_argument("--downstream-lengths", type=float, default=3.0)
    parser.add_argument("--side-y-spans", type=float, default=2.0)
    parser.add_argument("--side-z-spans", type=float, default=2.0)
    parser.add_argument("--min-farfield-padding", type=float, default=0.15)
    parser.add_argument("--farfield-size-policy", choices=["fixed", "dynamic"], default="dynamic")
    parser.add_argument("--farfield-size-fraction", type=float, default=0.15)
    parser.add_argument("--min-farfield-size", type=float, default=0.04)
    parser.add_argument("--max-farfield-size", type=float, default=0.35)
    parser.add_argument("--padding", default="0.7,0.5,0.5")
    parser.add_argument("--algorithm3d", type=int, default=4)
    parser.add_argument("--optimize", default="default,Netgen,Relocate3D")
    parser.add_argument("--solver-end-time", type=int, default=60)
    parser.add_argument("--solver-write-interval", type=int, default=20)
    parser.add_argument("--velocity", default="22.352,0,0")
    parser.add_argument("--rho-inf", type=float, default=1.225)
    parser.add_argument("--mag-u-inf", type=float, default=22.352)
    parser.add_argument("--l-ref", type=float, default=0.7111267)
    parser.add_argument("--a-ref", type=float, default=0.120)
    parser.add_argument("--cofr", default="0.338,0,0")
    parser.add_argument("--lift-dir", default="0,0,1")
    parser.add_argument("--drag-dir", default="1,0,0")
    parser.add_argument("--pitch-axis", default="0,1,0")
    args = parser.parse_args()

    scripts_dir = Path(__file__).resolve().parent
    run_dir = args.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()

    report: dict[str, Any] = {
        "status": "running",
        "input_stl": str(args.input_stl),
        "run_dir": str(run_dir),
        "steps": {},
        "commands": [],
        "settings": {
            "target_edge_mm": args.target_edge_mm,
            "remesh_iterations": args.remesh_iterations,
            "feature_deg": args.feature_deg,
            "max_surface_distance_mm": args.max_surface_distance_mm,
            "smooth_remesh": args.smooth_remesh,
            "surface_size": args.surface_size,
            "farfield_size": args.farfield_size,
            "farfield_policy": args.farfield_policy,
            "upstream_lengths": args.upstream_lengths,
            "downstream_lengths": args.downstream_lengths,
            "side_y_spans": args.side_y_spans,
            "side_z_spans": args.side_z_spans,
            "min_farfield_padding": args.min_farfield_padding,
            "farfield_size_policy": args.farfield_size_policy,
            "farfield_size_fraction": args.farfield_size_fraction,
            "min_farfield_size": args.min_farfield_size,
            "max_farfield_size": args.max_farfield_size,
            "padding": args.padding,
            "algorithm3d": args.algorithm3d,
            "optimize": args.optimize,
            "solver_end_time": args.solver_end_time,
            "solver_write_interval": args.solver_write_interval,
            "velocity": args.velocity,
            "rho_inf": args.rho_inf,
            "mag_u_inf": args.mag_u_inf,
            "l_ref": args.l_ref,
            "a_ref": args.a_ref,
            "cofr": args.cofr,
            "lift_dir": args.lift_dir,
            "drag_dir": args.drag_dir,
            "pitch_axis": args.pitch_axis,
            "prep_source_cap_loop_span_mm": args.prep_source_cap_loop_span_mm,
        },
    }

    try:
        pipeline_input_stl = args.input_stl
        if args.prep_source_cap_loop_span_mm is not None:
            prep_dir = run_dir / "source_prep"
            pipeline_input_stl = prep_dir / "aircraft_capped.stl"
            run_step(
                [
                    sys.executable,
                    str(scripts_dir / "cfd_surface_mesher.py"),
                    "--input-stl",
                    str(args.input_stl),
                    "--output-stl",
                    str(pipeline_input_stl),
                    "--report",
                    str(prep_dir / "source_prep_report.json"),
                    "--max-cap-loop-span-mm",
                    str(args.prep_source_cap_loop_span_mm),
                ],
                run_dir / "log.source_prep.txt",
                report,
                "source_prep",
            )
            report["prepared_input_stl"] = str(pipeline_input_stl)

        source_gate = run_dir / "source_gate.json"
        run_step(
            [
                sys.executable,
                str(scripts_dir / "cfd_mesh_gates.py"),
                "source",
                "--input-stl",
                str(pipeline_input_stl),
                "--report",
                str(source_gate),
            ],
            run_dir / "log.source_gate.txt",
            report,
            "source_gate",
        )

        remesh_dir = run_dir / "remesh"
        remeshed_stl = remesh_dir / "aircraft_surface_iso.stl"
        remesh_report = remesh_dir / "remesh_report.json"
        remesh_command = [
            sys.executable,
            str(scripts_dir / "remesh_aircraft_surface_pymeshlab.py"),
            "--input-stl",
            str(pipeline_input_stl),
            "--output-stl",
            str(remeshed_stl),
            "--report",
            str(remesh_report),
            "--target-edge-mm",
            str(args.target_edge_mm),
            "--iterations",
            str(args.remesh_iterations),
            "--feature-deg",
            str(args.feature_deg),
            "--max-surface-distance-mm",
            str(args.max_surface_distance_mm),
            "--samples",
            "12000",
        ]
        if not args.smooth_remesh:
            remesh_command.append("--no-smooth")
        run_step(
            remesh_command,
            run_dir / "log.remesh.txt",
            report,
            "remesh",
        )

        remesh_gate = run_dir / "remeshed_source_gate.json"
        run_step(
            [
                sys.executable,
                str(scripts_dir / "cfd_mesh_gates.py"),
                "source",
                "--input-stl",
                str(remeshed_stl),
                "--report",
                str(remesh_gate),
            ],
            run_dir / "log.remeshed_source_gate.txt",
            report,
            "remeshed_source_gate",
        )

        mesh_dir = run_dir / "gmsh"
        run_step(
            [
                sys.executable,
                str(scripts_dir / "run_gmsh_cfd_pipeline.py"),
                "--input-stl",
                str(remeshed_stl),
                "--run-dir",
                str(mesh_dir),
                "--target-faces",
                "0",
                "--surface-size",
                str(args.surface_size),
                "--farfield-size",
                str(args.farfield_size),
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
                "--algorithm3d",
                str(args.algorithm3d),
                "--optimize",
                args.optimize,
                "--farfield-size-policy",
                args.farfield_size_policy,
                "--farfield-size-fraction",
                str(args.farfield_size_fraction),
                "--min-farfield-size",
                str(args.min_farfield_size),
                "--max-farfield-size",
                str(args.max_farfield_size),
                "--fidelity-samples",
                "8000",
                "--farfield-patches",
                "split",
            ],
            run_dir / "log.gmsh_pipeline.txt",
            report,
            "gmsh_pipeline",
        )

        mesh_gate = run_dir / "mesh_gate.json"
        try:
            run_step(
                [
                    sys.executable,
                    str(scripts_dir / "cfd_mesh_gates.py"),
                    "mesh",
                    "--pipeline-report",
                    str(mesh_dir / "pipeline_report.json"),
                    "--case-dir",
                    str(mesh_dir / "openfoam_case"),
                    "--report",
                    str(mesh_gate),
                    "--expected-optimizers",
                    args.optimize,
                ],
                run_dir / "log.mesh_gate.txt",
                report,
                "mesh_gate",
            )
        except Exception:
            run_checkmesh_diagnostics(
                scripts_dir=scripts_dir,
                run_dir=run_dir,
                case_dir=mesh_dir / "openfoam_case",
                pipeline_report=mesh_dir / "pipeline_report.json",
                report=report,
            )
            raise

        if not args.skip_solver:
            solver_dir = run_dir / "openfoam_case_incompressible_default_60_forces"
            run_step(
                [
                    sys.executable,
                    str(scripts_dir / "setup_incompressible_fluid_smoke.py"),
                    "--source-case-dir",
                    str(mesh_dir / "openfoam_case"),
                    "--case-dir",
                    str(solver_dir),
                    "--velocity",
                    args.velocity,
                    "--farfield-mode",
                    "split",
                    "--end-time",
                    str(args.solver_end_time),
                    "--write-interval",
                    str(args.solver_write_interval),
                    "--force-coeffs",
                    "--rho-inf",
                    str(args.rho_inf),
                    "--mag-u-inf",
                    str(args.mag_u_inf),
                    "--l-ref",
                    str(args.l_ref),
                    "--a-ref",
                    str(args.a_ref),
                    "--cofr",
                    args.cofr,
                    "--lift-dir",
                    args.lift_dir,
                    "--drag-dir",
                    args.drag_dir,
                    "--pitch-axis",
                    args.pitch_axis,
                    "--force-write-interval",
                    "1",
                ],
                run_dir / "log.setup_solver.txt",
                report,
                "setup_solver",
            )
            run_openfoam_solver(solver_dir, run_dir / "log.openfoam_solver_driver.txt", report)
            run_step(
                [
                    sys.executable,
                    str(scripts_dir / "summarize_incompressible_fluid_smoke.py"),
                    "--case-dir",
                    str(solver_dir),
                    "--log",
                    str(solver_dir / "log.incompressibleFluid"),
                    "--report",
                    str(solver_dir / "incompressible_summary.json"),
                    "--quiet",
                ],
                run_dir / "log.solver_summary.txt",
                report,
                "solver_summary",
            )
            force_path = find_force_coeffs(solver_dir)
            run_step(
                [
                    sys.executable,
                    str(scripts_dir / "summarize_force_coeffs.py"),
                    "--force-coeffs",
                    str(force_path),
                    "--report",
                    str(solver_dir / "force_coeffs_summary.json"),
                ],
                run_dir / "log.force_summary.txt",
                report,
                "force_summary",
            )

        report["status"] = "ready"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
        write_report(run_dir, report, started)
        print(json.dumps(report, indent=2, sort_keys=True))
        raise SystemExit(1) from exc

    write_report(run_dir, report, started)
    print(json.dumps(report, indent=2, sort_keys=True))


def run_step(command: list[str], log_path: Path, report: dict[str, Any], step: str) -> None:
    started = time.perf_counter()
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.write_text("$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr, encoding="utf-8")
    report["commands"].append({"step": step, "command": command, "log": str(log_path), "returncode": result.returncode})
    report["steps"][step] = {
        "status": "ready" if result.returncode == 0 else "failed",
        "runtime_s": time.perf_counter() - started,
        "log": str(log_path),
    }
    if result.returncode != 0:
        raise RuntimeError(f"{step} failed with exit code {result.returncode}; see {log_path}")


def run_openfoam_solver(solver_dir: Path, log_path: Path, report: dict[str, Any]) -> None:
    started = time.perf_counter()
    solver_wsl = windows_path_to_wsl(solver_dir.resolve())
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{solver_wsl}' && "
        "checkMesh > log.checkMesh 2>&1 && "
        "foamRun -solver incompressibleFluid > log.incompressibleFluid 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    report["commands"].append(
        {
            "step": "openfoam_solver",
            "command": ["wsl", "bash", "-lc", script],
            "log": str(log_path),
            "returncode": result.returncode,
        }
    )
    report["steps"]["openfoam_solver"] = {
        "status": "ready" if result.returncode == 0 else "failed",
        "runtime_s": time.perf_counter() - started,
        "log": str(log_path),
    }
    if result.returncode != 0:
        raise RuntimeError(f"openfoam_solver failed with exit code {result.returncode}; see {log_path}")


def run_checkmesh_diagnostics(
    *,
    scripts_dir: Path,
    run_dir: Path,
    case_dir: Path,
    pipeline_report: Path,
    report: dict[str, Any],
) -> None:
    started = time.perf_counter()
    diagnostic: dict[str, Any] = {
        "status": "running",
        "case_dir": str(case_dir),
        "pipeline_report": str(pipeline_report),
    }
    try:
        pipeline = json.loads(pipeline_report.read_text(encoding="utf-8"))
        aircraft_bounds = (
            pipeline.get("gmsh_report", {})
            .get("farfield_domain", {})
            .get("aircraft_bounds")
        )
        if not aircraft_bounds:
            raise ValueError("pipeline report does not include gmsh_report.farfield_domain.aircraft_bounds")

        case_wsl = windows_path_to_wsl(case_dir.resolve())
        script = (
            "source /opt/openfoam13/etc/bashrc && "
            f"cd '{case_wsl}' && "
            "rm -rf postProcessing/checkMesh constant/polyMesh/sets/nonOrthoFaces && "
            "checkMesh -writeSurfaces -writeSets -nonOrthThreshold 70 > log.checkMesh.writeSurfaces 2>&1"
        )
        result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
        log_path = case_dir / "log.checkMesh.writeSurfaces.driver.txt"
        log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
        diagnostic["write_surfaces"] = {
            "command": ["wsl", "bash", "-lc", script],
            "returncode": result.returncode,
            "driver_log": str(log_path),
            "openfoam_log": str(case_dir / "log.checkMesh.writeSurfaces"),
        }

        vtk_path = case_dir / "postProcessing" / "checkMesh" / "constant" / "nonOrthoFaces.vtk"
        diagnostic["nonortho_vtk"] = str(vtk_path)
        if result.returncode != 0 or not vtk_path.exists():
            diagnostic["status"] = "failed"
            report["mesh_quality_diagnostics"] = diagnostic
            return

        flat_bounds = ",".join(str(value) for point in aircraft_bounds for value in point)
        localization_report = run_dir / "nonortho_localization.json"
        command = [
            sys.executable,
            str(scripts_dir / "localize_checkmesh_faces.py"),
            "--vtk",
            str(vtk_path),
            f"--aircraft-bounds={flat_bounds}",
            "--report",
            str(localization_report),
        ]
        result = subprocess.run(command, text=True, capture_output=True)
        localization_log = run_dir / "log.nonortho_localization.txt"
        localization_log.write_text(
            "$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr,
            encoding="utf-8",
        )
        diagnostic["localization"] = {
            "command": command,
            "returncode": result.returncode,
            "log": str(localization_log),
            "report": str(localization_report),
        }
        diagnostic["status"] = "ready" if result.returncode == 0 else "failed"
    except Exception as exc:
        diagnostic["status"] = "failed"
        diagnostic["error"] = str(exc)
    finally:
        diagnostic["runtime_s"] = time.perf_counter() - started
        report["mesh_quality_diagnostics"] = diagnostic


def find_force_coeffs(solver_dir: Path) -> Path:
    matches = list(solver_dir.glob("postProcessing/**/forceCoeffs.dat"))
    if not matches:
        raise FileNotFoundError(f"No forceCoeffs.dat under {solver_dir}")
    return matches[0]


def windows_path_to_wsl(path: Path) -> str:
    text = str(path)
    drive, rest = text[0], text[2:]
    rest = rest.replace("\\", "/")
    return f"/mnt/{drive.lower()}{rest}"


def write_report(run_dir: Path, report: dict[str, Any], started: float) -> None:
    report["runtime_s"] = time.perf_counter() - started
    (run_dir / "runner_report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    main()
