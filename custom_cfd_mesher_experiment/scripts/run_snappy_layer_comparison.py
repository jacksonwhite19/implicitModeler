from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import trimesh


EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[2]

PREPARED_ROOT = EXPERIMENT_ROOT / "runs" / "faired_cap_gmsh_openfoam_20260623"
VARIANTS = [
    ("fcv01_long_glider", PREPARED_ROOT / "fcv01_long_glider/gmsh_mesh/remesh/aircraft_surface_iso.stl"),
    ("fcv02_short_swept", PREPARED_ROOT / "fcv02_short_swept/gmsh_mesh/remesh/aircraft_surface_iso.stl"),
    (
        "fcv03_high_aspect_mild",
        PREPARED_ROOT / "fcv03_high_aspect_mild_retry_5mm/gmsh_mesh/remesh/aircraft_surface_iso.stl",
    ),
    ("fcv04_compact_wide_tail", PREPARED_ROOT / "fcv04_compact_wide_tail/gmsh_mesh/remesh/aircraft_surface_iso.stl"),
    ("fcv05_aft_wing_fast", PREPARED_ROOT / "fcv05_aft_wing_fast/gmsh_mesh/remesh/aircraft_surface_iso.stl"),
]

EXTRA_REFINEMENT_BOXES = {
    "fcv02_short_swept": [
        (-1.00002288819e-05, 0.027319708354780283, -0.025914432780249963, 0.02368556721975007, -0.007259893913205084, 0.006578947944705073, 4),
    ],
    "fcv03_high_aspect_mild": [
        (0.5133986996002197, 0.5956988003997804, 0.17233074999999992, 0.244438, 0.0006298880175781285, 0.01597838448242187, 4),
        (0.5132944496002197, 0.5956583003997803, -0.244439, -0.17227075000000003, 0.0006309780175781307, 0.015991374482421876, 4),
    ],
    "fcv04_compact_wide_tail": [
        (0.4783596996002196, 0.5345808003997803, -0.23503075000000004, -0.174152, 0.004251305787353513, 0.02160208671264647, 4),
        (0.47964719960021973, 0.5356488003997804, 0.17414775000000005, 0.23494775000000012, 0.0035828007873535198, 0.01842212421264649, 4),
        (0.4180394496002197, 0.4740410503997803, 0.340882, 0.3800000000000002, -0.007433801962646494, 0.007405521462646475, 4),
        (0.35053869960021966, 0.4266050503997803, 0.34715799999999997, 0.3800000000000002, -0.001333564212646493, 0.014302069212646491, 5),
        (0.3963701996002197, 0.4561158003997803, 0.34782674999999996, 0.3800000000000002, -0.0036901392126464927, 0.011482023712646484, 5),
        (0.4899654496002196, 0.5459670503997802, 0.1842442499999999, 0.24504424999999996, 0.001067728287353513, 0.015907051712646482, 5),
    ],
    "fcv05_aft_wing_fast": [
        (0.5100534193267822, 0.5721395806732178, 0.31428725, 0.3500000000000001, -0.009146023885986329, 0.00629346738598633, 4),
        (0.48390941932678216, 0.5459955806732179, 0.32007725, 0.3500000000000001, -0.010495345635986336, 0.00494414563598633, 5),
    ],
}

FEATURE_REFINEMENT_ZONES_XSV = {
    # Fractions in x=length, s=span, v=vertical/thickness coordinates.
    "nose": (0.00, 0.24, 0.34, 0.66, 0.00, 1.00),
    "wing_root_blend": (0.24, 0.66, 0.30, 0.70, 0.10, 0.78),
    "left_wing_le": (0.16, 0.62, 0.00, 0.36, 0.10, 0.78),
    "right_wing_le": (0.16, 0.62, 0.64, 1.00, 0.10, 0.78),
    "left_wing_te": (0.32, 0.84, 0.00, 0.36, 0.10, 0.82),
    "right_wing_te": (0.32, 0.84, 0.64, 1.00, 0.10, 0.82),
    "left_wingtip": (0.24, 0.86, 0.00, 0.16, 0.08, 0.84),
    "right_wingtip": (0.24, 0.86, 0.84, 1.00, 0.08, 0.84),
    "tail_root_blend": (0.60, 0.98, 0.32, 0.68, 0.10, 0.94),
    "tail_edges": (0.62, 1.00, 0.00, 1.00, 0.10, 1.00),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run fast snappy layer mesh/potentialFoam comparison.")
    parser.add_argument(
        "--run-root",
        type=Path,
        default=EXPERIMENT_ROOT / "runs" / "snappy_layer_five_variant_v0_1_20260623",
    )
    parser.add_argument("--velocity", default="22.352,0,0")
    parser.add_argument("--base-cells", default="44,28,44")
    parser.add_argument("--pad", type=float, default=0.8)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--surface-min-level", type=int, default=2)
    parser.add_argument("--surface-max-level", type=int, default=3)
    parser.add_argument("--feature-level", type=int, default=2)
    parser.add_argument("--n-cells-between-levels", type=int, default=4)
    parser.add_argument("--snap-tolerance", type=float, default=0.5)
    parser.add_argument("--n-smooth-patch", type=int, default=10)
    parser.add_argument("--n-surface-layers", type=int, default=2)
    parser.add_argument(
        "--layer-relative-sizes",
        choices=["true", "false"],
        default="true",
        help="Pass-through for snappyHexMesh addLayersControls.relativeSizes.",
    )
    parser.add_argument("--final-layer-thickness", type=float, default=0.20)
    parser.add_argument("--min-layer-thickness", type=float, default=0.03)
    parser.add_argument("--layer-expansion-ratio", type=float, default=1.2)
    parser.add_argument("--layer-feature-angle", type=float, default=70.0)
    parser.add_argument("--n-layer-iter", type=int, default=40)
    parser.add_argument("--n-relaxed-iter", type=int, default=15)
    parser.add_argument("--max-global-cells", type=int, default=2_500_000)
    parser.add_argument("--max-local-cells", type=int, default=1_500_000)
    parser.add_argument("--tip-refinement-level", type=int, default=4)
    parser.add_argument("--feature-refinement-boxes", action="store_true")
    parser.add_argument("--feature-refinement-level", type=int, default=4)
    parser.add_argument("--feature-angle-deg", type=float, default=28.0)
    parser.add_argument("--feature-box-count", type=int, default=12)
    parser.add_argument("--feature-box-grid", default="8,8,5")
    parser.add_argument("--feature-box-padding-frac", type=float, default=0.035)
    parser.add_argument("--include-known-hotspot-boxes", action="store_true")
    parser.add_argument(
        "--variant-ids",
        default="",
        help="Comma-separated variant ids to run. Empty means all variants.",
    )
    parser.add_argument(
        "--input-stl",
        type=Path,
        default=None,
        help="Run one arbitrary STL instead of the built-in faired-cap variant set.",
    )
    parser.add_argument(
        "--single-variant-id",
        default="custom_aircraft",
        help="Variant id to use with --input-stl.",
    )
    parser.add_argument(
        "--input-stl-map",
        type=Path,
        default=None,
        help="JSON object or list mapping variant ids to STL paths for a raw batch run.",
    )
    parser.add_argument("--parallel-procs", type=int, default=1)
    parser.add_argument("--check-skew-threshold", type=float, default=None)
    parser.add_argument("--only-failed-from", type=Path, default=None)
    parser.add_argument(
        "--skip-solver",
        action="store_true",
        help="Stop after mesh generation, checkMesh, screenshots, and optional surface-fidelity audit.",
    )
    parser.add_argument("--skip-screenshots", action="store_true")
    parser.add_argument(
        "--surface-fidelity-audit",
        action="store_true",
        help="Convert the final aircraft patch to STL and run source-to-patch fidelity audit.",
    )
    parser.add_argument("--surface-fidelity-samples", type=int, default=40000)
    args = parser.parse_args()

    run_root = unique_run_root(args.run_root)
    run_root.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    variants = selected_variants(
        args.only_failed_from,
        args.variant_ids,
        input_stl=args.input_stl,
        single_variant_id=args.single_variant_id,
        input_stl_map=args.input_stl_map,
    )
    reports = []
    for variant_id, input_stl in variants:
        reports.append(run_variant(variant_id, input_stl, run_root, args))

    summary = {
        "run_root": str(run_root),
        "runtime_s": time.perf_counter() - started,
        "variant_count": len(reports),
        "pass_count": sum(
            1
            for report in reports
            if report.get("verdict") in {"pass_for_layered_plumbing", "pass_for_mesh_only"}
        ),
        "reports": reports,
    }
    (run_root / "comparison_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
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


def write_decompose_dict(case_dir: Path, subdomains: int) -> None:
    if subdomains <= 1:
        return
    system_dir = case_dir / "system"
    system_dir.mkdir(parents=True, exist_ok=True)
    (system_dir / "decomposeParDict").write_text(
        """FoamFile
{
    format      ascii;
    class       dictionary;
    object      decomposeParDict;
}

"""
        + f"""numberOfSubdomains {subdomains};

method          scotch;

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def selected_variants(
    only_failed_from: Path | None,
    variant_ids: str,
    *,
    input_stl: Path | None,
    single_variant_id: str,
    input_stl_map: Path | None,
) -> list[tuple[str, Path]]:
    def resolve_input_path(path: str | Path) -> Path:
        resolved = Path(path)
        if resolved.is_absolute():
            return resolved
        return REPO_ROOT / resolved

    if input_stl is not None and input_stl_map is not None:
        raise ValueError("Use only one of --input-stl or --input-stl-map.")
    if input_stl is not None:
        return [(single_variant_id, resolve_input_path(input_stl))]
    if input_stl_map is not None:
        raw = json.loads(input_stl_map.read_text(encoding="utf-8-sig"))
        items = raw.items() if isinstance(raw, dict) else ((item["id"], item["stl"]) for item in raw)
        selected = {item.strip() for item in variant_ids.split(",") if item.strip()}
        return [
            (str(variant_id), resolve_input_path(path))
            for variant_id, path in items
            if not selected or str(variant_id) in selected
        ]
    selected = {item.strip() for item in variant_ids.split(",") if item.strip()}
    variants = [(variant_id, path) for variant_id, path in VARIANTS if not selected or variant_id in selected]
    if only_failed_from is None:
        return variants
    data = json.loads(only_failed_from.read_text(encoding="utf-8"))
    failed = {
        report["variant_id"]
        for report in data.get("reports", [])
        if report.get("verdict") != "pass_for_layered_plumbing"
    }
    return [(variant_id, path) for variant_id, path in variants if variant_id in failed]


def run_variant(variant_id: str, input_stl: Path, run_root: Path, args: argparse.Namespace) -> dict[str, Any]:
    variant_dir = run_root / variant_id
    case_dir = variant_dir / "openfoam_case"
    variant_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    report: dict[str, Any] = {
        "variant_id": variant_id,
        "input_stl": str(input_stl),
        "run_dir": str(variant_dir),
        "case_dir": str(case_dir),
        "commands": [],
        "status": "running",
    }
    try:
        if not input_stl.exists():
            raise FileNotFoundError(input_stl)
        refinement_boxes, refinement_box_metadata = automatic_tip_refinement_boxes(
            variant_id,
            input_stl,
            scale=args.scale,
            level=args.tip_refinement_level,
            include_known_hotspots=args.include_known_hotspot_boxes,
            include_feature_boxes=args.feature_refinement_boxes,
            feature_level=args.feature_refinement_level,
            feature_angle_deg=args.feature_angle_deg,
            feature_box_count=args.feature_box_count,
            feature_box_grid=parse_grid(args.feature_box_grid),
            feature_box_padding_frac=args.feature_box_padding_frac,
        )
        report["refinement_boxes"] = refinement_box_metadata
        make_command = [
            sys.executable,
            str(SCRIPTS_DIR / "make_openfoam_case.py"),
            "--input-stl",
            str(input_stl),
            "--case-dir",
            str(case_dir),
            "--scale",
            str(args.scale),
            "--pad",
            str(args.pad),
            "--base-cells",
            args.base_cells,
            "--surface-feature-angle",
            "150",
            "--feature-level",
            str(args.feature_level),
            "--surface-min-level",
            str(args.surface_min_level),
            "--surface-max-level",
            str(args.surface_max_level),
            "--n-cells-between-levels",
            str(args.n_cells_between_levels),
            "--snap-tolerance",
            str(args.snap_tolerance),
            "--n-smooth-patch",
            str(args.n_smooth_patch),
            "--max-local-cells",
            str(args.max_local_cells),
            "--max-global-cells",
            str(args.max_global_cells),
            "--add-layers",
            "--n-surface-layers",
            str(args.n_surface_layers),
            "--layer-relative-sizes",
            args.layer_relative_sizes,
            "--layer-expansion-ratio",
            str(args.layer_expansion_ratio),
            "--final-layer-thickness",
            str(args.final_layer_thickness),
            "--min-layer-thickness",
            str(args.min_layer_thickness),
            "--layer-feature-angle",
            str(args.layer_feature_angle),
            "--n-layer-iter",
            str(args.n_layer_iter),
            "--n-relaxed-iter",
            str(args.n_relaxed_iter),
            "--refinement-boxes",
            refinement_boxes,
        ]
        run_logged(make_command, variant_dir / "log.make_case.txt", report)

        case_wsl = to_wsl_path(case_dir.resolve())
        if args.parallel_procs > 1:
            write_decompose_dict(case_dir, args.parallel_procs)
        check_mesh_command = "checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk"
        if args.check_skew_threshold is not None:
            check_mesh_command = (
                f"checkMesh -skewThreshold {args.check_skew_threshold} "
                "-writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk"
            )
        if args.parallel_procs > 1:
            mesh_script = (
                "source /opt/openfoam13/etc/bashrc && "
                f"cd '{case_wsl}' && "
                "rm -rf constant/polyMesh processor* 1 2 3 VTK postProcessing "
                "log.surfaceCheck log.blockMesh log.surfaceFeatures log.decomposePar "
                "log.snappyHexMesh log.reconstructPar log.checkMesh.strict log.checkMesh log.foamToVTK && "
                "surfaceCheck constant/geometry/aircraft.stl > log.surfaceCheck 2>&1 && "
                "blockMesh > log.blockMesh 2>&1 && "
                "surfaceFeatures > log.surfaceFeatures 2>&1 && "
                "decomposePar > log.decomposePar 2>&1 && "
                f"mpirun -np {args.parallel_procs} snappyHexMesh -parallel -overwrite > log.snappyHexMesh 2>&1 && "
                "reconstructPar -constant -noFields > log.reconstructPar 2>&1 && "
                "rm -rf processor* && "
                "checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk > log.checkMesh.strict 2>&1 || true && "
                f"{check_mesh_command} > log.checkMesh 2>&1"
            )
        else:
            mesh_script = (
                "source /opt/openfoam13/etc/bashrc && "
                f"cd '{case_wsl}' && "
                "rm -rf constant/polyMesh 1 2 3 VTK postProcessing log.surfaceCheck log.blockMesh "
                "log.surfaceFeatures log.snappyHexMesh log.checkMesh.strict log.checkMesh log.foamToVTK && "
                "surfaceCheck constant/geometry/aircraft.stl > log.surfaceCheck 2>&1 && "
                "blockMesh > log.blockMesh 2>&1 && "
                "surfaceFeatures > log.surfaceFeatures 2>&1 && "
                "snappyHexMesh -overwrite > log.snappyHexMesh 2>&1 && "
                "checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk > log.checkMesh.strict 2>&1 || true && "
                f"{check_mesh_command} > log.checkMesh 2>&1"
            )
        mesh_started = time.perf_counter()
        run_logged(["wsl", "bash", "-lc", mesh_script], variant_dir / "log.openfoam_mesh_driver.txt", report)
        report["mesh_driver_runtime_s"] = time.perf_counter() - mesh_started

        run_logged(
            [
                sys.executable,
                str(SCRIPTS_DIR / "summarize_openfoam_run.py"),
                "--run-dir",
                str(variant_dir),
            ],
            variant_dir / "log.summarize_mesh.txt",
            report,
        )
        mesh_summary = read_json(variant_dir / "run_summary.json")
        report["mesh_summary"] = mesh_summary

        if args.skip_solver:
            potential_summary = {
                "status": "skipped",
                "reason": "mesh-only run requested with --skip-solver",
            }
            report["potential_summary"] = potential_summary
        else:
            run_logged(
                [
                    sys.executable,
                    str(SCRIPTS_DIR / "setup_potential_foam_smoke.py"),
                    "--case-dir",
                    str(case_dir),
                    "--velocity",
                    args.velocity,
                ],
                variant_dir / "log.setup_potential.txt",
                report,
            )
            potential_script = (
                "source /opt/openfoam13/etc/bashrc && "
                f"cd '{case_wsl}' && "
                "potentialFoam -writep > log.potentialFoam 2>&1"
            )
            run_logged(["wsl", "bash", "-lc", potential_script], variant_dir / "log.potential_driver.txt", report)
            run_logged(
                [
                    sys.executable,
                    str(SCRIPTS_DIR / "summarize_potential_foam_smoke.py"),
                    "--log",
                    str(case_dir / "log.potentialFoam"),
                    "--report",
                    str(variant_dir / "potential_summary.json"),
                ],
                variant_dir / "log.summarize_potential.txt",
                report,
            )
            potential_summary = read_json(variant_dir / "potential_summary.json")
            report["potential_summary"] = potential_summary

        aircraft_vtk = case_dir / "VTK/aircraft/aircraft_0.vtk"
        if not args.skip_screenshots or args.surface_fidelity_audit:
            try:
                vtk_script = (
                    "source /opt/openfoam13/etc/bashrc && "
                    f"cd '{case_wsl}' && "
                    "foamToVTK -constant -noInternal -excludePatches '(farfield)' > log.foamToVTK 2>&1"
                )
                run_logged(["wsl", "bash", "-lc", vtk_script], variant_dir / "log.foam_to_vtk_driver.txt", report)
            except Exception as exc:  # Screenshots should not hide mesh results.
                report["foam_to_vtk_error"] = str(exc)

        if not args.skip_screenshots and aircraft_vtk.exists():
            screenshot_path = variant_dir / "aircraft_iso.png"
            try:
                run_pvpython(
                    SCRIPTS_DIR / "render_aircraft_iso_screenshot.py",
                    [
                        "--aircraft-vtk",
                        str(aircraft_vtk),
                        "--output-png",
                        str(screenshot_path),
                        "--representation",
                        "Surface With Edges",
                    ],
                    variant_dir / "log.render_iso.txt",
                    report,
                )
                report["aircraft_iso_png"] = str(screenshot_path)
            except Exception as exc:  # Screenshots should not hide mesh results.
                report["screenshot_error"] = str(exc)

        if args.surface_fidelity_audit and aircraft_vtk.exists():
            patch_stl = variant_dir / "aircraft_patch_from_vtk.stl"
            fidelity_report = variant_dir / "surface_fidelity_vtk_distance_1layer_candidate.json"
            try:
                run_pvpython(
                    SCRIPTS_DIR / "convert_vtk_surface_to_stl.py",
                    [
                        "--input-vtk",
                        str(aircraft_vtk),
                        "--output-stl",
                        str(patch_stl),
                    ],
                    variant_dir / "log.convert_aircraft_vtk_to_stl.txt",
                    report,
                )
                run_logged(
                    [
                        sys.executable,
                        str(SCRIPTS_DIR / "surface_fidelity_audit.py"),
                        "--reference-stl",
                        str(input_stl),
                        "--candidate-stl",
                        str(patch_stl),
                        "--report",
                        str(fidelity_report),
                        "--scale",
                        str(args.scale),
                        "--samples",
                        str(args.surface_fidelity_samples),
                    ],
                    variant_dir / "log.surface_fidelity_audit.txt",
                    report,
                )
                report["aircraft_patch_stl"] = str(patch_stl)
                report["surface_fidelity_report"] = str(fidelity_report)
                report["surface_fidelity"] = read_json(fidelity_report)
            except Exception as exc:
                report["surface_fidelity_error"] = str(exc)

        mesh_ok = bool(mesh_summary.get("check_mesh", {}).get("mesh_ok"))
        potential_ok = bool(potential_summary.get("completed"))
        if mesh_ok and args.skip_solver:
            report["verdict"] = "pass_for_mesh_only"
        elif mesh_ok and potential_ok:
            report["verdict"] = "pass_for_layered_plumbing"
        elif not mesh_ok:
            report["verdict"] = "fail_openfoam_quality"
        else:
            report["verdict"] = "fail_potential_smoke"
        report["status"] = "completed"
    except Exception as exc:
        report["status"] = "failed"
        report["verdict"] = "fail_pipeline"
        report["error"] = str(exc)
    report["runtime_s"] = time.perf_counter() - started
    (variant_dir / "variant_summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return report


def automatic_tip_refinement_boxes(
    variant_id: str,
    input_stl: Path,
    *,
    scale: float,
    level: int,
    include_known_hotspots: bool,
    include_feature_boxes: bool,
    feature_level: int,
    feature_angle_deg: float,
    feature_box_count: int,
    feature_box_grid: tuple[int, int, int],
    feature_box_padding_frac: float,
) -> tuple[str, dict[str, Any]]:
    mesh = trimesh.load(input_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh: {input_stl}")
    mesh = mesh.copy()
    mesh.vertices *= scale
    bounds = mesh.bounds
    axes = infer_aircraft_axes(bounds)
    box_records: list[dict[str, Any]] = []
    boxes: list[tuple[float, float, float, float, float, float, int]] = []
    tip_zones = [
        ("left_tip_seed", (0.28, 0.82, 0.00, 0.09, 0.08, 0.84)),
        ("right_tip_seed", (0.28, 0.82, 0.91, 1.00, 0.08, 0.84)),
    ]
    for name, zone in tip_zones:
        box = xsv_fraction_box_to_xyz(bounds, axes, zone, level)
        boxes.append(box)
        box_records.append({"source": "tip_seed", "name": name, "box": box})
    if include_known_hotspots:
        for index, box in enumerate(EXTRA_REFINEMENT_BOXES.get(variant_id, [])):
            boxes.append(box)
            box_records.append({"source": "known_hotspot", "name": f"{variant_id}_{index}", "box": box})
    if include_feature_boxes:
        feature_boxes, feature_records = surface_feature_refinement_boxes(
            mesh,
            bounds,
            axes=axes,
            level=feature_level,
            feature_angle_deg=feature_angle_deg,
            max_boxes=feature_box_count,
            grid=feature_box_grid,
            padding_frac=feature_box_padding_frac,
        )
        boxes.extend(feature_boxes)
        box_records.extend(feature_records)
    return (
        ";".join(",".join(f"{value:.9g}" for value in box) for box in boxes),
        {
            "axis_mapping": axes,
            "box_count": len(boxes),
            "feature_zone_names": list(FEATURE_REFINEMENT_ZONES_XSV),
            "boxes": [
                {
                    **record,
                    "box": [round(float(value), 9) for value in record["box"]],
                }
                for record in box_records
            ],
        },
    )


def parse_grid(value: str) -> tuple[int, int, int]:
    parts = [int(part) for part in value.split(",") if part.strip()]
    if len(parts) != 3 or any(part <= 0 for part in parts):
        raise argparse.ArgumentTypeError("--feature-box-grid must be three positive integers")
    return (parts[0], parts[1], parts[2])


def find_windows_pvpython() -> Path | None:
    candidates = [
        Path(r"C:\Program Files\ParaView 5.10.1-Windows-Python3.9-msvc2017-AMD64\bin\pvpython.exe"),
        Path(r"C:\Program Files\ParaView 5.13.0\bin\pvpython.exe"),
        Path(r"C:\Program Files\ParaView 5.12.0\bin\pvpython.exe"),
        Path(r"C:\Program Files\ParaView 5.11.0\bin\pvpython.exe"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def run_pvpython(
    script_path: Path,
    args: list[str],
    log_path: Path,
    report: dict[str, Any],
) -> None:
    windows_pvpython = find_windows_pvpython()
    if windows_pvpython is not None:
        run_logged([str(windows_pvpython), str(script_path), *args], log_path, report)
        return
    wsl_args = " ".join(shlex.quote(to_wsl_path(Path(arg).resolve())) if looks_like_path_arg(arg) else shlex.quote(arg) for arg in args)
    command = f"pvpython {shlex.quote(to_wsl_path(script_path.resolve()))} {wsl_args}"
    run_logged(["wsl", "bash", "-lc", command], log_path, report)


def looks_like_path_arg(value: str) -> bool:
    if value.startswith("--"):
        return False
    return (
        "\\" in value
        or "/" in value
        or value.lower().endswith((".vtk", ".stl", ".png", ".py"))
    )


def surface_feature_refinement_boxes(
    mesh: trimesh.Trimesh,
    bounds: Any,
    *,
    axes: dict[str, int],
    level: int,
    feature_angle_deg: float,
    max_boxes: int,
    grid: tuple[int, int, int],
    padding_frac: float,
) -> tuple[list[tuple[float, float, float, float, float, float, int]], list[dict[str, Any]]]:
    if max_boxes <= 0:
        return [], []
    adjacency = getattr(mesh, "face_adjacency", None)
    angles = getattr(mesh, "face_adjacency_angles", None)
    edges = getattr(mesh, "face_adjacency_edges", None)
    if adjacency is None or angles is None or edges is None or len(edges) == 0:
        return [], []
    threshold = feature_angle_deg * 3.141592653589793 / 180.0
    mask = angles >= threshold
    if not bool(mask.any()):
        return [], []
    vertices = mesh.vertices
    feature_edges = edges[mask]
    centers = vertices[feature_edges].mean(axis=1)
    lo = bounds[0]
    hi = bounds[1]
    span = hi - lo
    norm = (centers - lo) / span
    inside = (norm >= 0.0).all(axis=1) & (norm <= 1.0).all(axis=1)
    centers = centers[inside]
    norm = norm[inside]
    zone_labels = [feature_zone_for_norm(values, axes) for values in norm]
    zone_mask = [label is not None for label in zone_labels]
    centers = centers[zone_mask]
    norm = norm[zone_mask]
    zone_labels = [label for label in zone_labels if label is not None]
    if len(centers) == 0:
        return [], []

    nx, ny, nz = grid
    bins: dict[tuple[str, int, int, int], list[int]] = {}
    for row, values in enumerate(norm):
        key = (
            str(zone_labels[row]),
            min(max(int(values[0] * nx), 0), nx - 1),
            min(max(int(values[1] * ny), 0), ny - 1),
            min(max(int(values[2] * nz), 0), nz - 1),
        )
        bins.setdefault(key, []).append(row)

    ranked = sorted(bins.values(), key=len, reverse=True)
    boxes: list[tuple[float, float, float, float, float, float, int]] = []
    records: list[dict[str, Any]] = []
    min_fraction = (0.04, 0.04, 0.04)
    for rows in ranked[:max_boxes]:
        local = centers[rows]
        box_lo = local.min(axis=0) - span * padding_frac
        box_hi = local.max(axis=0) + span * padding_frac
        for axis, frac in enumerate(min_fraction):
            minimum_width = span[axis] * frac
            width = box_hi[axis] - box_lo[axis]
            if width < minimum_width:
                center = 0.5 * (box_lo[axis] + box_hi[axis])
                box_lo[axis] = center - 0.5 * minimum_width
                box_hi[axis] = center + 0.5 * minimum_width
        box_lo = box_lo.clip(lo, hi)
        box_hi = box_hi.clip(lo, hi)
        if (box_hi > box_lo).all():
            box = (
                float(box_lo[0]),
                float(box_hi[0]),
                float(box_lo[1]),
                float(box_hi[1]),
                float(box_lo[2]),
                float(box_hi[2]),
                int(level),
            )
            labels = sorted({str(zone_labels[row]) for row in rows})
            boxes.append(box)
            records.append(
                {
                    "source": "feature_zone",
                    "name": "+".join(labels),
                    "edge_count": len(rows),
                    "box": box,
                }
            )
    return boxes, records


def infer_aircraft_axes(bounds: Any) -> dict[str, int]:
    span = bounds[1] - bounds[0]
    span_axis = 1 if span[1] >= span[2] else 2
    vertical_axis = 2 if span_axis == 1 else 1
    return {"length": 0, "span": int(span_axis), "vertical": int(vertical_axis)}


def xsv_fraction_box_to_xyz(
    bounds: Any,
    axes: dict[str, int],
    fractions: tuple[float, float, float, float, float, float],
    level: int,
) -> tuple[float, float, float, float, float, float, int]:
    lo = bounds[0]
    span = bounds[1] - bounds[0]
    output = [0.0] * 6
    ranges = {
        axes["length"]: (fractions[0], fractions[1]),
        axes["span"]: (fractions[2], fractions[3]),
        axes["vertical"]: (fractions[4], fractions[5]),
    }
    for axis, (start, end) in ranges.items():
        output[2 * axis] = float(lo[axis] + start * span[axis])
        output[2 * axis + 1] = float(lo[axis] + end * span[axis])
    return (
        output[0],
        output[1],
        output[2],
        output[3],
        output[4],
        output[5],
        int(level),
    )


def feature_zone_for_norm(values: Any, axes: dict[str, int]) -> str | None:
    x = float(values[axes["length"]])
    s = float(values[axes["span"]])
    v = float(values[axes["vertical"]])
    for name, box in FEATURE_REFINEMENT_ZONES_XSV.items():
        if box[0] <= x <= box[1] and box[2] <= s <= box[3] and box[4] <= v <= box[5]:
            return name
    return None


def run_logged(command: list[str], log_path: Path, report: dict[str, Any]) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    record = {"command": command, "log": str(log_path)}
    report["commands"].append(record)
    with log_path.open("w", encoding="utf-8", errors="replace") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    record["returncode"] = result.returncode
    record["runtime_s"] = time.perf_counter() - started
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit {result.returncode}: {' '.join(command)}")


def to_wsl_path(path: Path) -> str:
    text = str(path)
    if text[1:3] == ":\\":
        drive = text[0].lower()
        return f"/mnt/{drive}/" + text[3:].replace("\\", "/")
    return text.replace("\\", "/")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
