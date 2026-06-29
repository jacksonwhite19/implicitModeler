from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.records import MetricValue

MODULE_NAME = "cfd_mesh_result_validation"
MODULE_VERSION = "0.1.0"


@dataclass(frozen=True)
class MeshResultValidation:
    passed: bool
    mesh_result: dict[str, Any]
    metrics: dict[str, dict[str, object]]
    metadata: dict[str, Any]
    warnings: list[str]


def validate_mesh_result(
    *,
    mesh_path: Path,
    mesh_format: str,
    su2_log_path: Path | None = None,
    semantic_to_solver_markers: dict[str, list[str]] | None = None,
) -> MeshResultValidation:
    normalized_format = mesh_format.lower()
    if normalized_format == "su2":
        parsed = parse_native_su2_mesh(mesh_path)
        marker_map = semantic_to_solver_markers or infer_semantic_marker_map(
            parsed["solver_markers"]
        )
    elif normalized_format == "cgns":
        if su2_log_path is None:
            raise ValueError("CGNS validation requires su2_log_path")
        parsed = parse_su2_mesh_preprocessing_log(su2_log_path)
        marker_map = semantic_to_solver_markers or infer_semantic_marker_map(
            parsed["solver_markers"]
        )
    else:
        raise ValueError(f"Unsupported mesh format: {mesh_format}")

    mesh_result = build_mesh_result(
        mesh_path=mesh_path,
        mesh_format=normalized_format,
        parsed=parsed,
        marker_map=marker_map,
    )
    failed_checks = evaluate_mesh_checks(mesh_result)
    metrics = {
        "cfd_mesh.nodes": _metric(mesh_result["cell_counts"]["nodes"], None),
        "cfd_mesh.volume_cells": _metric(
            mesh_result["cell_counts"]["volume_cells"], None
        ),
        "cfd_mesh.aircraft_faces": _metric(
            mesh_result["boundary_counts"]["aircraft_faces"], None
        ),
        "cfd_mesh.farfield_faces": _metric(
            mesh_result["boundary_counts"]["farfield_faces"], None
        ),
        "cfd_mesh.ready": _metric(0 if failed_checks else 1, None),
        "cfd_mesh.failed_check_count": _metric(len(failed_checks), None),
    }
    return MeshResultValidation(
        passed=not failed_checks,
        mesh_result=mesh_result,
        metrics=metrics,
        metadata={
            "module_name": MODULE_NAME,
            "module_version": MODULE_VERSION,
            "failed_checks": failed_checks,
            "solver_markers": parsed["solver_markers"],
        },
        warnings=[f"failed mesh-result check: {check}" for check in failed_checks],
    )


def parse_native_su2_mesh(path: Path) -> dict[str, Any]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    ndime = _find_int_assignment(lines, "NDIME")
    nelem_line_index, volume_cells = _find_assignment_with_index(lines, "NELEM")
    npoin_line_index, nodes = _find_assignment_with_index(lines, "NPOIN")
    nmark = _find_int_assignment(lines, "NMARK")

    volume_type_counts = _count_su2_volume_types(
        lines[nelem_line_index + 1 : npoin_line_index]
    )
    markers = _parse_su2_marker_sections(lines)
    surface_faces = sum(marker["face_count"] for marker in markers.values())
    return {
        "source": "native_su2_mesh",
        "ndime": ndime,
        "nodes": nodes,
        "volume_cells": volume_cells,
        "surface_faces": surface_faces,
        "nmark": nmark,
        "solver_markers": markers,
        "volume_type_counts": volume_type_counts,
    }


def parse_su2_mesh_preprocessing_log(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    zone_match = re.search(
        r"Zone\s+\d+,\s+.+?:\s+(\d+)\s+total vertices,\s+(\d+)\s+total elements",
        text,
    )
    if zone_match:
        nodes = int(zone_match.group(1))
        total_elements = int(zone_match.group(2))
    else:
        nodes = _first_int_match(text, r"(\d+)\s+grid points\.")
        total_elements = _first_int_match(text, r"(\d+)\s+volume elements\.")

    section_counts: dict[str, dict[str, Any]] = {}
    for section, count, element_type in re.findall(
        r"Section\s+(\S+)\s+contains\s+(\d+)\s+elements of type\s+([A-Za-z]+)",
        text,
    ):
        section_counts[section] = {
            "face_count": int(count) if element_type.lower() != "tetrahedron" else 0,
            "element_count": int(count),
            "element_type": element_type,
        }

    solver_markers: dict[str, dict[str, Any]] = {}
    for count, marker in re.findall(
        r"(\d+)\s+boundary elements in index\s+\d+\s+\(Marker = ([^)]+)\)\.",
        text,
    ):
        marker = marker.strip()
        element_type = section_counts.get(marker, {}).get("element_type", "surface")
        solver_markers[marker] = {
            "face_count": int(count),
            "element_type": element_type,
        }

    tetrahedra = _first_int_match(text, r"(\d+)\s+tetrahedra\.")
    volume_cells = tetrahedra or total_elements
    volume_type_counts = {
        "tetrahedra": tetrahedra,
        "hexahedra": _first_int_match(text, r"(\d+)\s+hexahedra\."),
        "prisms": _first_int_match(text, r"(\d+)\s+prisms\."),
        "pyramids": _first_int_match(text, r"(\d+)\s+pyramids\."),
    }
    surface_faces = sum(marker["face_count"] for marker in solver_markers.values())
    return {
        "source": "su2_mesh_preprocessing_log",
        "ndime": 3 if "Three dimensional problem." in text else None,
        "nodes": nodes,
        "volume_cells": volume_cells,
        "surface_faces": surface_faces,
        "nmark": len(solver_markers),
        "solver_markers": solver_markers,
        "volume_type_counts": volume_type_counts,
    }


def infer_semantic_marker_map(
    solver_markers: dict[str, dict[str, Any]]
) -> dict[str, list[str]]:
    marker_names = set(solver_markers)
    return {
        "aircraft": ["aircraft"] if "aircraft" in marker_names else [],
        "farfield": ["farfield"] if "farfield" in marker_names else [],
        "fluid": ["fluid"] if "fluid" in marker_names else [],
    }


def build_mesh_result(
    *,
    mesh_path: Path,
    mesh_format: str,
    parsed: dict[str, Any],
    marker_map: dict[str, list[str]],
) -> dict[str, Any]:
    solver_markers = parsed["solver_markers"]
    fluid_markers = list(marker_map.get("fluid", []))
    if not fluid_markers and mesh_format == "su2" and (parsed["volume_cells"] or 0) > 0:
        fluid_markers = ["implicit_single_volume"]
    aircraft_faces = _marker_face_sum(solver_markers, marker_map.get("aircraft", []))
    farfield_faces = _marker_face_sum(solver_markers, marker_map.get("farfield", []))
    volume_counts = parsed["volume_type_counts"]
    return {
        "mesh_result_schema_version": "0.1.0",
        "mesh_format": mesh_format,
        "mesh_path": str(mesh_path),
        "case_units": {"length": "m", "velocity": "m/s"},
        "coordinate_frame": {"x": "forward", "y": "right", "z": "up"},
        "semantic_to_solver_markers": {
            "fluid": fluid_markers,
            "aircraft": list(marker_map.get("aircraft", [])),
            "farfield": list(marker_map.get("farfield", [])),
        },
        "cell_counts": {
            "nodes": parsed["nodes"] or 0,
            "volume_cells": parsed["volume_cells"] or 0,
            "surface_faces": parsed["surface_faces"] or 0,
            "tetrahedra": volume_counts.get("tetrahedra"),
            "hexahedra": volume_counts.get("hexahedra"),
            "prisms": volume_counts.get("prisms"),
            "pyramids": volume_counts.get("pyramids"),
        },
        "boundary_counts": {
            "aircraft_faces": aircraft_faces,
            "farfield_faces": farfield_faces,
        },
        "quality": {
            "min_quality": None,
            "max_skewness": None,
            "invalid_surface_elements": 0,
            "invalid_volume_elements": 0,
        },
        "solver_markers": solver_markers,
    }


def evaluate_mesh_checks(mesh_result: dict[str, Any]) -> list[str]:
    failed: list[str] = []
    cell_counts = mesh_result["cell_counts"]
    boundary_counts = mesh_result["boundary_counts"]
    marker_map = mesh_result["semantic_to_solver_markers"]
    if mesh_result["mesh_format"] not in {"su2", "cgns"}:
        failed.append("mesh_format_supported")
    if cell_counts["nodes"] <= 0:
        failed.append("nodes_positive")
    if cell_counts["volume_cells"] <= 0:
        failed.append("volume_cells_positive")
    if not marker_map["aircraft"]:
        failed.append("aircraft_marker_mapped")
    if not marker_map["farfield"]:
        failed.append("farfield_marker_mapped")
    if not marker_map["fluid"] and mesh_result["mesh_format"] != "su2":
        failed.append("fluid_marker_mapped")
    if boundary_counts["aircraft_faces"] <= 0:
        failed.append("aircraft_faces_positive")
    if boundary_counts["farfield_faces"] <= 0:
        failed.append("farfield_faces_positive")
    return failed


def _parse_su2_marker_sections(lines: list[str]) -> dict[str, dict[str, Any]]:
    markers: dict[str, dict[str, Any]] = {}
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if line.startswith("MARKER_TAG"):
            marker = _assignment_value(line)
            elems_line = lines[index + 1].strip()
            face_count = int(_assignment_value(elems_line))
            markers[marker] = {"face_count": face_count, "element_type": "surface"}
            index += 2 + face_count
        else:
            index += 1
    return markers


def _count_su2_volume_types(element_lines: list[str]) -> dict[str, int | None]:
    counts = {"tetrahedra": 0, "hexahedra": 0, "prisms": 0, "pyramids": 0}
    type_map = {
        10: "tetrahedra",
        12: "hexahedra",
        13: "prisms",
        14: "pyramids",
    }
    for line in element_lines:
        parts = line.strip().split()
        if not parts:
            continue
        try:
            element_type = int(parts[0])
        except ValueError:
            continue
        key = type_map.get(element_type)
        if key:
            counts[key] += 1
    return counts


def _marker_face_sum(
    solver_markers: dict[str, dict[str, Any]], marker_names: list[str]
) -> int:
    return sum(int(solver_markers.get(marker, {}).get("face_count", 0)) for marker in marker_names)


def _find_int_assignment(lines: list[str], key: str) -> int:
    _, value = _find_assignment_with_index(lines, key)
    return value


def _find_assignment_with_index(lines: list[str], key: str) -> tuple[int, int]:
    prefix = f"{key}="
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith(prefix):
            return index, int(_assignment_value(stripped))
    raise ValueError(f"Missing SU2 assignment: {key}")


def _assignment_value(line: str) -> str:
    return line.split("=", 1)[1].strip()


def _first_int_match(text: str, pattern: str) -> int | None:
    match = re.search(pattern, text)
    return int(match.group(1)) if match else None


def _metric(value: float | int, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6) if isinstance(value, float) else value,
        unit=unit,
        confidence=1.0,
        source=MODULE_NAME,
    ).to_dict()
