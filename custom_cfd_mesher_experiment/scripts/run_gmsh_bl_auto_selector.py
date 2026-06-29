from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from run_gmsh_cfd_pipeline import parse_check_mesh, windows_path_to_wsl


VARIANTS = {
    "fcv01_long_glider": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/"
        "direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl"
    ),
    "fcv02_short_swept": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/"
        "direct_sdf_oml_fcv02_short_swept_faired_cap_spacing_1p0.stl"
    ),
    "fcv03_high_aspect_mild": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/"
        "direct_sdf_oml_fcv03_high_aspect_mild_faired_cap_spacing_1p0.stl"
    ),
    "fcv04_compact_wide_tail": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/"
        "direct_sdf_oml_fcv04_compact_wide_tail_faired_cap_spacing_1p0.stl"
    ),
    "fcv05_aft_wing_fast": Path(
        "dual_contouring/direct_sparse_sdf_mc_experiment/stl/"
        "direct_sdf_oml_fcv05_aft_wing_fast_faired_cap_spacing_1p0.stl"
    ),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run first-pass selected Gmsh BL prism meshes for five aircraft variants.")
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--variant", action="append", choices=sorted(VARIANTS), default=None)
    parser.add_argument("--skip-potential", action="store_true")
    args = parser.parse_args()

    started = time.perf_counter()
    scripts_dir = Path(__file__).resolve().parent
    run_root = args.run_root
    run_root.mkdir(parents=True, exist_ok=True)
    selected_variants = args.variant or list(VARIANTS)

    reports = []
    for variant_id in selected_variants:
        try:
            report = run_variant(
                variant_id,
                VARIANTS[variant_id],
                run_root / variant_id,
                scripts_dir=scripts_dir,
                skip_potential=args.skip_potential,
            )
        except Exception as exc:
            report = {
                "variant_id": variant_id,
                "raw_stl": str(VARIANTS[variant_id]),
                "run_dir": str(run_root / variant_id),
                "fatal_error": str(exc),
                "openfoam": {"mesh_ok": False},
                "potential": {"completed": False},
            }
        reports.append(report)
        openfoam = report.get("openfoam") or {}
        potential = report.get("potential") or {}
        decision = report.get("selector_decision") or {"feature_deg": None, "layer_height_m": 0.0}
        print(
            f"{variant_id}: mesh={openfoam.get('mesh_ok')} cells={openfoam.get('cells')} "
            f"prisms={openfoam.get('prisms')} skew={openfoam.get('max_skewness')} "
            f"nonorth={openfoam.get('max_non_orthogonality')} potential={potential.get('completed')} "
            f"feature={decision['feature_deg']} layer_um={decision['layer_height_m'] * 1e6:.0f}",
            flush=True,
        )

    summary = {
        "run_root": str(run_root),
        "strategy": "gmsh_bl_auto_selector_v0_2",
        "runtime_s": time.perf_counter() - started,
        "variant_count": len(reports),
        "pass_count": sum(1 for item in reports if ((item.get("openfoam") or {}).get("mesh_ok"))),
        "potential_pass_count": sum(1 for item in reports if ((item.get("potential") or {}).get("completed"))),
        "reports": reports,
    }
    (run_root / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    if summary["pass_count"] != len(reports):
        raise SystemExit(2)
    if not args.skip_potential and summary["potential_pass_count"] != len(reports):
        raise SystemExit(3)


def run_variant(
    variant_id: str,
    raw_stl: Path,
    run_dir: Path,
    *,
    scripts_dir: Path,
    skip_potential: bool,
) -> dict[str, Any]:
    started = time.perf_counter()
    run_dir.mkdir(parents=True, exist_ok=True)

    base_surface = run_dir / "base_remesh" / "aircraft_surface_base_4mm_fd25.stl"
    base_remesh_report = run_dir / "base_remesh" / "remesh_report.json"
    run_remesh(
        raw_stl,
        base_surface,
        base_remesh_report,
        scripts_dir=scripts_dir,
        feature_deg=25.0,
        target_edge_mm=4.0,
        iterations=8,
        smooth=False,
        log_path=run_dir / "log.base_remesh.txt",
    )
    base_gate_report = run_dir / "base_remesh" / "bl_surface_gate.json"
    base_bad_csv = run_dir / "base_remesh" / "bad_triangles.csv"
    run_bl_surface_gate(
        base_surface,
        base_gate_report,
        base_bad_csv,
        scripts_dir=scripts_dir,
        log_path=run_dir / "log.base_bl_gate.txt",
    )
    base_gate = load_json(base_gate_report)
    decision = select_preset(base_gate)

    if decision["feature_deg"] == 25.0:
        selected_surface = base_surface
        selected_remesh_report = base_remesh_report
        selected_gate = base_gate
    else:
        selected_surface = run_dir / "selected_remesh" / f"aircraft_surface_fd{int(decision['feature_deg'])}.stl"
        selected_remesh_report = run_dir / "selected_remesh" / "remesh_report.json"
        run_remesh(
            raw_stl,
            selected_surface,
            selected_remesh_report,
            scripts_dir=scripts_dir,
            feature_deg=decision["feature_deg"],
            target_edge_mm=4.0,
            iterations=8,
            smooth=False,
            log_path=run_dir / "log.selected_remesh.txt",
        )
        selected_gate_report = run_dir / "selected_remesh" / "bl_surface_gate.json"
        selected_bad_csv = run_dir / "selected_remesh" / "bad_triangles.csv"
        run_bl_surface_gate(
            selected_surface,
            selected_gate_report,
            selected_bad_csv,
            scripts_dir=scripts_dir,
            log_path=run_dir / "log.selected_bl_gate.txt",
        )
        selected_gate = load_json(selected_gate_report)

    gmsh_dir = run_dir / "gmsh_bl"
    mesh_cmd = [
        sys.executable,
        str(scripts_dir / "gmsh_boundary_layer_mesher.py"),
        "--input-stl",
        str(selected_surface),
        "--run-dir",
        str(gmsh_dir),
        "--target-faces",
        "0",
        "--surface-size",
        "0.004",
        "--farfield-size",
        "0.11",
        "--padding",
        "1.14003,1.6,0.38598",
        "--layer-heights",
        str(decision["layer_height_m"]),
        "--layer-elements",
        "1",
        "--angle-deg",
        "80",
        "--curve-angle-deg",
        "180",
        "--algorithm3d",
        "4",
        "--geometry-mode",
        "create-topology",
        "--optimize",
        "none",
    ]
    mesh_result = run_logged(mesh_cmd, run_dir / "log.gmsh_bl.txt")

    openfoam = {}
    potential = {"completed": False, "skipped": "mesh_not_ready"}
    if mesh_result.returncode == 0:
        openfoam = run_openfoam_check(gmsh_dir / "openfoam_case")
        if openfoam.get("mesh_ok") and not skip_potential:
            potential = run_potential(gmsh_dir / "openfoam_case", scripts_dir=scripts_dir)

    report = {
        "variant_id": variant_id,
        "raw_stl": str(raw_stl),
        "run_dir": str(run_dir),
        "base_surface": str(base_surface),
        "selected_surface": str(selected_surface),
        "selector_decision": decision,
        "base_gate": base_gate,
        "selected_gate": selected_gate,
        "gmsh_report": load_json(gmsh_dir / "gmsh_boundary_layer_report.json"),
        "mesh_returncode": mesh_result.returncode,
        "openfoam": openfoam,
        "potential": potential,
        "runtime_s": time.perf_counter() - started,
    }
    (run_dir / "variant_report.json").write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return report


def select_preset(base_gate: dict[str, Any]) -> dict[str, Any]:
    surface = base_gate["surface"]
    counts = surface["counts"]
    clusters = surface.get("bad_triangle_clusters") or []
    top = clusters[0] if clusters else {"region_label": "", "count": 0}
    top_region = str(top.get("region_label") or "")
    top_count = int(top.get("count") or 0)
    aspect_max = float(surface["triangle_aspect_ratio"]["max"] or 0.0)
    q_min = float(surface["triangle_quality"]["min"] or 0.0)
    aspect_gt60 = int(counts["aspect_gt_60"])
    angle_lt_2p5 = int(counts["min_angle_lt_2p5deg"])
    faces = int(surface["faces"])

    reasons: list[str] = []
    feature_deg = 25.0
    layer_height_m = 0.00005

    if top_region.startswith("tail/") and aspect_max > 600.0:
        feature_deg = 15.0
        layer_height_m = 0.0001
        reasons.append("tail-cluster/high-aspect surface risk; use lower feature angle and thicker layer")
    elif q_min < 0.0038 and aspect_max > 500.0 and top_region.startswith("mid/"):
        feature_deg = 45.0
        layer_height_m = 0.00005
        reasons.append("mid-span high-aspect/low-quality cluster; use higher feature angle remesh")
    elif q_min < 0.004 and top_region.startswith("mid/") and top_count >= 15:
        feature_deg = 45.0
        layer_height_m = 0.00005
        reasons.append("mid-span concentrated low-quality cluster; use higher feature angle remesh")
    elif faces < 70000 and aspect_gt60 <= 20 and top_region.startswith("aft_mid/left_tip"):
        layer_height_m = 0.00015
        reasons.append("low-face short-swept transition risk; keep base remesh and thicken first layer")
    elif angle_lt_2p5 >= 80 and not top_region.startswith("tail/"):
        reasons.append("many small-angle triangles but prior evidence favors base layer on non-tail cluster")
    else:
        reasons.append("base remesh and nominal layer selected")

    return {
        "selector": "gmsh_bl_auto_selector_v0_2",
        "feature_deg": feature_deg,
        "target_edge_mm": 4.0,
        "remesh_iterations": 8,
        "smooth": False,
        "layer_height_m": layer_height_m,
        "surface_size_m": 0.004,
        "geometry_mode": "create-topology",
        "algorithm3d": 4,
        "reasons": reasons,
        "inputs": {
            "faces": faces,
            "top_cluster": top_region,
            "top_cluster_count": top_count,
            "triangle_quality_min": q_min,
            "triangle_aspect_ratio_max": aspect_max,
            "aspect_gt_60": aspect_gt60,
            "min_angle_lt_2p5deg": angle_lt_2p5,
        },
    }


def run_remesh(
    input_stl: Path,
    output_stl: Path,
    report: Path,
    *,
    scripts_dir: Path,
    feature_deg: float,
    target_edge_mm: float,
    iterations: int,
    smooth: bool,
    log_path: Path,
) -> None:
    cmd = [
        sys.executable,
        str(scripts_dir / "remesh_aircraft_surface_pymeshlab.py"),
        "--input-stl",
        str(input_stl),
        "--output-stl",
        str(output_stl),
        "--report",
        str(report),
        "--target-edge-mm",
        str(target_edge_mm),
        "--iterations",
        str(iterations),
        "--feature-deg",
        str(feature_deg),
        "--max-surface-distance-mm",
        "0.15",
        "--samples",
        "8000",
    ]
    if not smooth:
        cmd.append("--no-smooth")
    result = run_logged(cmd, log_path)
    if result.returncode != 0:
        raise RuntimeError(f"Remesh failed; see {log_path}")


def run_bl_surface_gate(input_stl: Path, report: Path, csv_path: Path, *, scripts_dir: Path, log_path: Path) -> None:
    cmd = [
        sys.executable,
        str(scripts_dir / "bl_surface_gate.py"),
        "--input-stl",
        str(input_stl),
        "--report",
        str(report),
        "--bad-triangles-csv",
        str(csv_path),
    ]
    result = run_logged(cmd, log_path)
    if result.returncode not in (0, 2):
        raise RuntimeError(f"BL surface gate failed; see {log_path}")


def run_openfoam_check(case_dir: Path) -> dict[str, Any]:
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    script = (
        "source /opt/openfoam13/etc/bashrc && "
        f"cd '{case_wsl}' && "
        "rm -rf constant/polyMesh postProcessing VTK log.gmshToFoam log.checkMesh && "
        "gmshToFoam -keepOrientation ../mesh.msh > log.gmshToFoam 2>&1 && "
        "checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk > log.checkMesh 2>&1"
    )
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (case_dir / "log.openfoam_driver.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
    check_log = case_dir / "log.checkMesh"
    text = check_log.read_text(encoding="utf-8", errors="replace") if check_log.exists() else ""
    summary = parse_check_mesh(text)
    summary["returncode"] = result.returncode
    summary["status"] = "ready" if summary.get("mesh_ok") else "failed"
    summary["check_mesh_log"] = str(check_log)
    return summary


def run_potential(case_dir: Path, *, scripts_dir: Path) -> dict[str, Any]:
    setup_cmd = [
        sys.executable,
        str(scripts_dir / "setup_potential_foam_smoke.py"),
        "--case-dir",
        str(case_dir),
        "--velocity",
        "22.352,0,0",
    ]
    setup = run_logged(setup_cmd, case_dir / "log.potential_setup_driver.txt")
    if setup.returncode != 0:
        return {"completed": False, "setup_returncode": setup.returncode}
    case_wsl = windows_path_to_wsl(case_dir.resolve())
    script = f"source /opt/openfoam13/etc/bashrc && cd '{case_wsl}' && potentialFoam -writep > log.potentialFoam 2>&1"
    result = subprocess.run(["wsl", "bash", "-lc", script], text=True, capture_output=True)
    (case_dir / "log.potential_driver.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
    summary_path = case_dir / "potential_summary.json"
    summary_cmd = [
        sys.executable,
        str(scripts_dir / "summarize_potential_foam_smoke.py"),
        "--log",
        str(case_dir / "log.potentialFoam"),
        "--report",
        str(summary_path),
    ]
    run_logged(summary_cmd, case_dir / "log.potential_summary_driver.txt")
    summary = load_json(summary_path)
    summary["returncode"] = result.returncode
    return summary


def run_logged(command: list[str], log_path: Path) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, text=True, capture_output=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(
        "$ " + " ".join(command) + "\n\n" + result.stdout + result.stderr,
        encoding="utf-8",
    )
    return result


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
