from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from run_gmsh_cfd_pipeline import parse_check_mesh, windows_path_to_wsl


FCV01 = Path(
    "custom_cfd_mesher_experiment/runs/faired_cap_gmsh_openfoam_20260623/"
    "fcv01_long_glider/gmsh_mesh/remesh/aircraft_surface_iso.stl"
)


CASES = [
    {
        "name": "full_1layer_50um_algo4_noopt",
        "args": [
            "--target-faces",
            "0",
            "--surface-size",
            "0.004",
            "--farfield-size",
            "0.11",
            "--padding",
            "1.14003,1.6,0.38598",
            "--layer-heights",
            "0.00005",
            "--layer-elements",
            "1",
            "--angle-deg",
            "80",
            "--curve-angle-deg",
            "180",
            "--algorithm3d",
            "4",
            "--geometry-mode",
            "create-geometry",
            "--optimize",
            "none",
        ],
    },
    {
        "name": "full_1layer_25um_algo4_noopt",
        "args": [
            "--target-faces",
            "0",
            "--surface-size",
            "0.004",
            "--farfield-size",
            "0.11",
            "--padding",
            "1.14003,1.6,0.38598",
            "--layer-heights",
            "0.000025",
            "--layer-elements",
            "1",
            "--angle-deg",
            "80",
            "--curve-angle-deg",
            "180",
            "--algorithm3d",
            "4",
            "--geometry-mode",
            "create-geometry",
            "--optimize",
            "none",
        ],
    },
    {
        "name": "full_1layer_100um_algo4_noopt",
        "args": [
            "--target-faces",
            "0",
            "--surface-size",
            "0.004",
            "--farfield-size",
            "0.11",
            "--padding",
            "1.14003,1.6,0.38598",
            "--layer-heights",
            "0.0001",
            "--layer-elements",
            "1",
            "--angle-deg",
            "80",
            "--curve-angle-deg",
            "180",
            "--algorithm3d",
            "4",
            "--geometry-mode",
            "create-geometry",
            "--optimize",
            "none",
        ],
    },
    {
        "name": "full_1layer_50um_norecombine",
        "args": [
            "--target-faces",
            "0",
            "--surface-size",
            "0.004",
            "--farfield-size",
            "0.11",
            "--padding",
            "1.14003,1.6,0.38598",
            "--layer-heights",
            "0.00005",
            "--layer-elements",
            "1",
            "--angle-deg",
            "80",
            "--curve-angle-deg",
            "180",
            "--algorithm3d",
            "4",
            "--geometry-mode",
            "create-geometry",
            "--optimize",
            "none",
            "--no-recombine",
        ],
    },
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--input-stl", type=Path, default=FCV01)
    args = parser.parse_args()

    root = args.run_root
    root.mkdir(parents=True, exist_ok=True)
    scripts_dir = Path(__file__).resolve().parent
    reports = []
    for case in CASES:
        started = time.perf_counter()
        case_dir = root / case["name"]
        case_dir.mkdir(parents=True, exist_ok=True)
        mesh_cmd = [
            sys.executable,
            str(scripts_dir / "gmsh_boundary_layer_mesher.py"),
            "--input-stl",
            str(args.input_stl),
            "--run-dir",
            str(case_dir),
            *case["args"],
        ]
        mesh_result = run_logged(mesh_cmd, case_dir / "log.mesh_driver.txt")
        openfoam = run_openfoam(case_dir / "openfoam_case", keep_orientation=True)
        report = {
            "name": case["name"],
            "input_stl": str(args.input_stl),
            "case_dir": str(case_dir),
            "mesh_command": mesh_cmd,
            "mesh_returncode": mesh_result.returncode,
            "openfoam": openfoam,
            "gmsh_report": load_json(case_dir / "gmsh_boundary_layer_report.json"),
            "runtime_s": time.perf_counter() - started,
        }
        (case_dir / "pilot_report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        reports.append(report)
        print(summary_line(report), flush=True)

    summary = {
        "run_root": str(root),
        "variant": "fcv01_long_glider",
        "reports": reports,
    }
    (root / "gmsh_bl_pilot_matrix_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def run_logged(command: list[str], log_path: Path) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.write_text(
        "$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr,
        encoding="utf-8",
    )
    return result


def run_openfoam(case_dir: Path, *, keep_orientation: bool) -> dict[str, object]:
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    gmsh_to_foam = "gmshToFoam -keepOrientation ../mesh.msh" if keep_orientation else "gmshToFoam ../mesh.msh"
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf constant/polyMesh postProcessing VTK log.gmshToFoam log.checkMesh && "
        f"{gmsh_to_foam} > log.gmshToFoam 2>&1 && "
        "checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk > log.checkMesh 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (case_dir / "log.openfoam_driver.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
    check_log = case_dir / "log.checkMesh"
    text = check_log.read_text(encoding="utf-8", errors="replace") if check_log.exists() else ""
    summary = parse_check_mesh(text)
    summary["returncode"] = result.returncode
    summary["mesh_ok"] = bool(summary.get("mesh_ok"))
    summary["status"] = "ready" if summary["mesh_ok"] else "failed"
    summary["check_mesh_log"] = str(check_log)
    summary["gmsh_to_foam_log"] = str(case_dir / "log.gmshToFoam")
    return summary


def summary_line(report: dict[str, object]) -> str:
    foam = report["openfoam"]
    return (
        f"{report['name']}: {foam.get('status')} cells={foam.get('cells')} "
        f"prisms={foam.get('prisms')} negVol={foam.get('negative_volume_cells')} "
        f"nonOrtho={foam.get('max_non_orthogonality')} skew={foam.get('max_skewness')} "
        f"failed={foam.get('failed_checks')} runtime={report['runtime_s']:.1f}s"
    )


def load_json(path: Path) -> dict[str, object]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
