from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--case-dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    case_dir = args.case_dir or args.run_dir / "openfoam_case"
    output = args.output or args.run_dir / "run_summary.json"
    summary = summarize(args.run_dir, case_dir)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


def summarize(run_dir: Path, case_dir: Path) -> dict[str, Any]:
    surface_report = read_json(run_dir / "surface_report.json")
    case_report = read_json(case_dir / "case_report.json")
    surface_check = read_text(case_dir / "log.surfaceCheck")
    check_mesh = read_text(case_dir / "log.checkMesh")
    snappy = read_text(case_dir / "log.snappyHexMesh")

    return {
        "run_dir": str(run_dir),
        "case_dir": str(case_dir),
        "surface": {
            "watertight": surface_report.get("final", {}).get("watertight"),
            "boundary_edges": surface_report.get("final", {}).get("boundary_edges"),
            "nonmanifold_edges": surface_report.get("final", {}).get("nonmanifold_edges"),
            "faces": surface_report.get("final", {}).get("faces"),
            "vertices": surface_report.get("final", {}).get("vertices"),
            "sliver_faces_q_lt_0p05": surface_report.get("final", {}).get("sliver_faces_q_lt_0p05"),
            "triangle_quality": surface_report.get("final", {}).get("triangle_quality"),
            "caps": surface_report.get("caps", []),
        },
        "openfoam_case": {
            "base_cells": case_report.get("base_cells"),
            "surface_min_level": case_report.get("surface_min_level"),
            "surface_max_level": case_report.get("surface_max_level"),
            "feature_level": case_report.get("feature_level"),
            "n_cells_between_levels": case_report.get("n_cells_between_levels"),
            "snap_tolerance": case_report.get("snap_tolerance"),
            "n_smooth_patch": case_report.get("n_smooth_patch"),
        },
        "surface_check": parse_surface_check(surface_check),
        "snappy": parse_snappy(snappy),
        "check_mesh": parse_check_mesh(check_mesh),
        "artifacts": {
            "surface_stl": str(run_dir / "aircraft_surface.stl"),
            "openfoam_stl": str(case_dir / "constant" / "geometry" / "aircraft.stl"),
            "aircraft_vtk": str(case_dir / "VTK" / "aircraft" / "aircraft_0.vtk"),
            "aircraft_iso_png": str(run_dir / "aircraft_iso.png"),
        },
    }


def parse_surface_check(text: str) -> dict[str, Any]:
    return {
        "closed": "Surface is closed. All edges connected to two faces." in text,
        "illegal_triangles": not ("Surface has no illegal triangles." in text),
        "unconnected_parts": int_match(text, r"Number of unconnected parts\s*:\s*(\d+)"),
        "normal_zones": int_match(text, r"Number of zones \(connected area with consistent normal\)\s*:\s*(\d+)"),
        "min_triangle_quality": float_match(text, r"min\s+([0-9.eE+-]+)\s+for triangle"),
    }


def parse_snappy(text: str) -> dict[str, Any]:
    return {
        "finished_without_errors": "Finished meshing without any errors" in text,
        "runtime_s": float_match(text, r"Finished meshing in =\s*([0-9.eE+-]+)\s*s"),
        "snapped_cells": int_match(text, r"Snapped mesh : cells:(\d+)"),
        "snapped_faces": int_match(text, r"Snapped mesh : cells:\d+\s+faces:(\d+)"),
        "snapped_points": int_match(text, r"Snapped mesh : cells:\d+\s+faces:\d+\s+points:(\d+)"),
    }


def parse_check_mesh(text: str) -> dict[str, Any]:
    return {
        "mesh_ok": "Mesh OK." in text,
        "failed_checks": int_match(text, r"Failed\s+(\d+)\s+mesh checks"),
        "points": int_match(text, r"points:\s*(\d+)"),
        "faces": int_match(text, r"faces:\s*(\d+)"),
        "internal_faces": int_match(text, r"internal faces:\s*(\d+)"),
        "cells": int_match(text, r"cells:\s*(\d+)"),
        "aircraft_patch_faces": int_match(text, r"^\s*aircraft\s+(\d+)\s+\d+\s+.+$", re.MULTILINE),
        "farfield_patch_faces": int_match(text, r"^\s*farfield\s+(\d+)\s+\d+\s+.+$", re.MULTILINE),
        "max_aspect_ratio": float_match(text, r"Max aspect ratio =\s*([0-9.eE+-]+)"),
        "max_non_orthogonality": float_match(text, r"Mesh non-orthogonality Max:\s*([0-9.eE+-]+)"),
        "avg_non_orthogonality": float_match(text, r"Mesh non-orthogonality Max:\s*[0-9.eE+-]+\s+average:\s*([0-9.eE+-]+)"),
        "max_skewness": float_match(text, r"Max skewness =\s*([0-9.eE+-]+)"),
    }


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def read_text(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def int_match(text: str, pattern: str, flags: int = 0) -> int | None:
    match = re.search(pattern, text, flags)
    if not match:
        return None
    return int(match.group(1))


def float_match(text: str, pattern: str) -> float | None:
    match = re.search(pattern, text)
    if not match:
        return None
    return float(match.group(1))


if __name__ == "__main__":
    main()
