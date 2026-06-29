from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SCRIPTS_DIR = Path(__file__).resolve().parent
EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]


WALL_FUNCTION_PROBE_CONFIGS = [
    {
        "id": "rel_3layer_0p55_0p08",
        "n_surface_layers": 3,
        "layer_relative_sizes": "true",
        "final_layer_thickness": 0.55,
        "min_layer_thickness": 0.08,
        "layer_expansion_ratio": 1.2,
        "layer_feature_angle": 70.0,
        "wall_treatment": "wall_function",
    },
    {
        "id": "rel_1layer_0p75_0p12",
        "n_surface_layers": 1,
        "layer_relative_sizes": "true",
        "final_layer_thickness": 0.75,
        "min_layer_thickness": 0.12,
        "layer_expansion_ratio": 1.2,
        "layer_feature_angle": 70.0,
        "wall_treatment": "wall_function",
    },
]


RESOLVED_WALL_PROBE_CONFIGS = [
    {
        "id": "abs_4layer_0p18mm_0p015mm",
        "n_surface_layers": 4,
        "layer_relative_sizes": "false",
        "final_layer_thickness": 0.00018,
        "min_layer_thickness": 0.000015,
        "layer_expansion_ratio": 1.18,
        "layer_feature_angle": 70.0,
        "wall_treatment": "low_re",
    },
    {
        "id": "abs_5layer_0p12mm_0p010mm",
        "n_surface_layers": 5,
        "layer_relative_sizes": "false",
        "final_layer_thickness": 0.00012,
        "min_layer_thickness": 0.000010,
        "layer_expansion_ratio": 1.15,
        "layer_feature_angle": 70.0,
        "wall_treatment": "low_re",
    },
]


STUDY_MODES = {
    "wall-function-probe": WALL_FUNCTION_PROBE_CONFIGS,
    "resolved-wall-probe": RESOLVED_WALL_PROBE_CONFIGS,
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a bounded snappy layer/y+ study for one faired-cap variant."
    )
    parser.add_argument(
        "--run-root",
        type=Path,
        default=EXPERIMENT_ROOT / "runs" / "snappy_yplus_study_v0_1",
    )
    parser.add_argument("--variant-id", default="fcv04_compact_wide_tail")
    parser.add_argument(
        "--study-mode",
        choices=sorted(STUDY_MODES),
        default="wall-function-probe",
    )
    parser.add_argument(
        "--config-ids",
        default="",
        help="Comma-separated config ids to run within the selected study mode.",
    )
    parser.add_argument("--end-time", type=int, default=100)
    parser.add_argument("--timeout-s", type=int, default=900)
    args = parser.parse_args()

    if args.run_root.exists():
        existing_cases = [path for path in args.run_root.iterdir() if path.is_dir()]
        if existing_cases or (args.run_root / "summary.json").exists():
            raise SystemExit(f"Run root already has study output; choose a fresh path: {args.run_root}")
    args.run_root.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    selected_config_ids = {item.strip() for item in args.config_ids.split(",") if item.strip()}
    configs = [
        config
        for config in STUDY_MODES[args.study_mode]
        if not selected_config_ids or config["id"] in selected_config_ids
    ]
    missing_config_ids = selected_config_ids - {config["id"] for config in configs}
    if missing_config_ids:
        raise SystemExit(f"Unknown config ids for {args.study_mode}: {sorted(missing_config_ids)}")
    reports = [run_config(config, args) for config in configs]
    summary = {
        "study": "snappy_yplus_study_v0_2",
        "study_mode": args.study_mode,
        "variant_id": args.variant_id,
        "run_root": str(args.run_root),
        "runtime_s": time.perf_counter() - started,
        "reports": reports,
    }
    (args.run_root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    write_csv(args.run_root / "summary.csv", reports)
    write_markdown(args.run_root / "summary.md", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))


def run_config(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    config_root = args.run_root / config["id"]
    mesh_root = config_root / "mesh_run"
    started = time.perf_counter()
    report: dict[str, Any] = {
        "config_id": config["id"],
        "variant_id": args.variant_id,
        "run_root": str(config_root),
        "mesh_run_root": str(mesh_root),
        "layer_settings": config,
        "commands": [],
    }
    try:
        mesh_command = [
            sys.executable,
            str(SCRIPTS_DIR / "run_snappy_layer_comparison.py"),
            "--run-root",
            str(mesh_root),
            "--variant-ids",
            args.variant_id,
            "--velocity",
            "22.352,0,0",
            "--base-cells",
            "44,28,44",
            "--pad",
            "0.8",
            "--surface-min-level",
            "4",
            "--surface-max-level",
            "5",
            "--feature-level",
            "5",
            "--n-cells-between-levels",
            "4",
            "--snap-tolerance",
            "0.5",
            "--n-smooth-patch",
            "10",
            "--n-surface-layers",
            str(config["n_surface_layers"]),
            "--layer-relative-sizes",
            str(config["layer_relative_sizes"]),
            "--layer-expansion-ratio",
            str(config["layer_expansion_ratio"]),
            "--final-layer-thickness",
            str(config["final_layer_thickness"]),
            "--min-layer-thickness",
            str(config["min_layer_thickness"]),
            "--layer-feature-angle",
            str(config["layer_feature_angle"]),
            "--n-layer-iter",
            "50",
            "--n-relaxed-iter",
            "20",
            "--max-global-cells",
            "6000000",
            "--max-local-cells",
            "3000000",
            "--feature-refinement-level",
            "5",
            "--feature-angle-deg",
            "24.0",
            "--feature-box-count",
            "18",
            "--feature-box-grid",
            "10,10,6",
            "--feature-box-padding-frac",
            "0.03",
            "--parallel-procs",
            "12",
            "--check-skew-threshold",
            "12.0",
            "--feature-refinement-boxes",
            "--include-known-hotspot-boxes",
            "--skip-screenshots",
        ]
        run_logged(mesh_command, config_root / "log.mesh_command.txt", report)
        rans_command = [
            sys.executable,
            str(SCRIPTS_DIR / "run_no_slip_laminar_start_rans.py"),
            "--run-root",
            str(mesh_root),
            "--variant-ids",
            args.variant_id,
            "--end-time",
            str(args.end_time),
            "--timeout-s",
            str(args.timeout_s),
            "--wall-treatment",
            str(config["wall_treatment"]),
        ]
        run_logged(rans_command, config_root / "log.rans_command.txt", report)
        report.update(extract_metrics(mesh_root, args.variant_id))
        report["status"] = "complete"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = str(exc)
    report["runtime_s"] = time.perf_counter() - started
    (config_root / "study_case_summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return report


def extract_metrics(mesh_root: Path, variant_id: str) -> dict[str, Any]:
    variant_root = mesh_root / variant_id
    comparison = read_json(mesh_root / "comparison_summary.json")
    ladder = read_json(mesh_root / "no_slip_ladder_summary.json")
    mesh_report = (comparison.get("reports") or [{}])[0]
    mesh_summary = mesh_report.get("mesh_summary") or {}
    check_mesh = mesh_summary.get("check_mesh") or {}
    potential = mesh_report.get("potential_summary") or {}
    ladder_report = (ladder.get("reports") or [{}])[0]
    rans = ladder_report.get("rans") or {}
    yplus = rans.get("yplus") or {}
    force_last = rans.get("force_last") or {}
    return {
        "variant_root": str(variant_root),
        "mesh_ok": bool(check_mesh.get("mesh_ok")),
        "potential_completed": bool(potential.get("completed")),
        "rans_completed": bool(rans.get("completed")),
        "cells": check_mesh.get("cells"),
        "points": check_mesh.get("points"),
        "aircraft_faces": check_mesh.get("aircraft_patch_faces")
        or ((check_mesh.get("patches") or {}).get("aircraft") or {}).get("faces"),
        "max_skewness": check_mesh.get("max_skewness")
        or ((check_mesh.get("quality") or {}).get("max_skewness")),
        "max_non_orthogonality": check_mesh.get("max_non_orthogonality")
        or ((check_mesh.get("quality") or {}).get("max_non_orthogonality")),
        "cd_last": force_last.get("cd"),
        "cl_last": force_last.get("cl"),
        "cm_last": force_last.get("cm"),
        "yplus_p50": yplus.get("p50"),
        "yplus_p95": yplus.get("p95"),
        "yplus_max": yplus.get("max"),
        "yplus_lt1": yplus.get("lt1"),
        "yplus_gt30": yplus.get("gt30"),
        "yplus_gt100": yplus.get("gt100"),
        "wall_function_gate": rans.get("yplus_wall_function_gate"),
        "mesh_summary_path": str(variant_root / "run_summary.json"),
        "ladder_summary_path": str(variant_root / "no_slip_ladder_summary.json"),
    }


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


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_csv(path: Path, reports: list[dict[str, Any]]) -> None:
    fields = [
        "config_id",
        "status",
        "mesh_ok",
        "potential_completed",
        "rans_completed",
        "cells",
        "points",
        "aircraft_faces",
        "max_skewness",
        "max_non_orthogonality",
        "cd_last",
        "cl_last",
        "cm_last",
        "yplus_p50",
        "yplus_p95",
        "yplus_max",
        "yplus_lt1",
        "yplus_gt30",
        "yplus_gt100",
        "runtime_s",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for report in reports:
            writer.writerow({field: report.get(field) for field in fields})


def write_markdown(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Snappy y+ Study v0.2",
        "",
        f"Variant: `{summary['variant_id']}`",
        f"Study mode: `{summary['study_mode']}`",
        "",
        "| config | mesh | RANS | cells | aircraft faces | skew | non-ortho | y+ p50 | y+ p95 | y+ max |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for report in summary["reports"]:
        lines.append(
            "| {config} | {mesh} | {rans} | {cells} | {faces} | {skew} | {nonortho} | {p50} | {p95} | {maxy} |".format(
                config=report.get("config_id"),
                mesh=report.get("mesh_ok"),
                rans=report.get("rans_completed"),
                cells=report.get("cells"),
                faces=report.get("aircraft_faces"),
                skew=fmt(report.get("max_skewness")),
                nonortho=fmt(report.get("max_non_orthogonality")),
                p50=fmt(report.get("yplus_p50")),
                p95=fmt(report.get("yplus_p95")),
                maxy=fmt(report.get("yplus_max")),
            )
        )
    lines.append("")
    lines.append("All cases are development evidence only; coefficients remain non-scoring.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


if __name__ == "__main__":
    main()
