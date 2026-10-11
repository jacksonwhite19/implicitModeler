"""Generic two-sided schedule-aware ridge-sector metric targets.

The input policy contains persistent curve and sheet names plus certified
directional target sizes.  It contains no coordinate regions or transient face
descriptors.  Stock MMGS remains the only topology engine.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def _unit(value: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=np.float64)
    length = float(np.linalg.norm(value))
    if not length > 0.0:
        raise ValueError("zero directional metric vector")
    return value / length


def _replace_directional_size(matrix: np.ndarray, direction: np.ndarray,
                              target_size: float) -> np.ndarray:
    """Set one directional metric size while preserving the orthogonal block."""
    if not target_size > 0.0:
        raise ValueError("two-sided target size must be positive")
    q0 = _unit(direction)
    reference = np.asarray([1.0, 0.0, 0.0])
    if abs(float(np.dot(q0, reference))) > 0.85:
        reference = np.asarray([0.0, 1.0, 0.0])
    q1 = _unit(reference - np.dot(reference, q0) * q0)
    q2 = _unit(np.cross(q0, q1))
    basis = np.column_stack((q0, q1, q2))
    local = basis.T @ np.asarray(matrix, dtype=np.float64) @ basis
    result_local = np.zeros((3, 3), dtype=np.float64)
    result_local[0, 0] = 1.0 / float(target_size * target_size)
    result_local[1:, 1:] = local[1:, 1:]
    result = basis @ result_local @ basis.T
    result = 0.5 * (result + result.T)
    if not np.all(np.linalg.eigvalsh(result) > 0.0):
        raise RuntimeError("two-sided directional replacement is not SPD")
    return result


def load_policy(path: str | Path) -> dict:
    policy = json.loads(Path(path).read_text(encoding="ascii"))
    if policy.get("schema") != "V3TwoSidedScheduleAwareMetricPolicy/v1":
        raise ValueError("unsupported two-sided metric policy")
    if not policy.get("all_required_admissible_sets_nonempty"):
        raise ValueError("two-sided policy is not authorized for MMGS")
    return policy


def apply_two_sided_policy(metrics: list[np.ndarray], vertices, faces, edges,
                           authority, policy: dict) -> tuple[list[np.ndarray], dict]:
    """Apply persistent ridge-sector directional targets to seed metrics."""
    curve_by_name = {name: ref for ref, name in authority.curve_names.items()}
    sheet_by_name = {name: ref for ref, name in authority.sheet_names.items()}
    entries = []
    for raw in policy["entries"]:
        curve_names = tuple(sorted(raw["boundary_curves"]))
        missing_curves = [name for name in curve_names if name not in curve_by_name]
        if missing_curves:
            raise ValueError(f"unknown persistent curves: {missing_curves}")
        if raw["sheet"] not in sheet_by_name:
            raise ValueError(f"unknown persistent sheet: {raw['sheet']}")
        entries.append({
            **raw,
            "curve_refs": tuple(curve_by_name[name] for name in curve_names),
            "sheet_ref": sheet_by_name[raw["sheet"]],
        })
    entries.sort(key=lambda row: (row["mode"], row["sheet"], tuple(row["boundary_curves"])))

    vertex_curves = [set() for _ in vertices]
    curve_directions: dict[tuple[int, int], list[np.ndarray]] = {}
    for edge in edges:
        vertex_curves[edge.vertices[0]].add(edge.curve_ref)
        vertex_curves[edge.vertices[1]].add(edge.curve_ref)
        direction = _unit(vertices[edge.vertices[1]].point - vertices[edge.vertices[0]].point)
        curve_directions.setdefault((edge.vertices[0], edge.curve_ref), []).append(direction)
        curve_directions.setdefault((edge.vertices[1], edge.curve_ref), []).append(direction)

    adjusted = [np.asarray(metric, dtype=np.float64).copy() for metric in metrics]
    touched_vertices: set[int] = set()
    report_entries = []
    for entry in entries:
        matched_faces = 0
        entry_vertices: set[int] = set()
        if entry["mode"] == "two_curve_bridge":
            # A topology seed need not yet contain the output bridge triangle.
            # Express its directional band through the exact persistent curve
            # tangents and every incident target-sheet face.  No coordinate
            # proximity or raw descriptor is used.
            for face in faces:
                if face.sheet_ref != entry["sheet_ref"]:
                    continue
                selected = [
                    vertex_index for vertex_index in face.vertices
                    if vertex_curves[vertex_index].intersection(entry["curve_refs"])
                ]
                if not selected:
                    continue
                directions = []
                for vertex_index in selected:
                    for curve_ref in sorted(vertex_curves[vertex_index].intersection(entry["curve_refs"])):
                        directions.extend(curve_directions.get((vertex_index, curve_ref), []))
                if not directions:
                    continue
                reference = directions[0]
                aligned = [direction if np.dot(direction, reference) >= 0.0 else -direction
                           for direction in directions]
                direction = _unit(np.sum(aligned, axis=0))
                for vertex_index in face.vertices:
                    adjusted[vertex_index] = _replace_directional_size(
                        adjusted[vertex_index], direction, float(entry["target_size_m"])
                    )
                    touched_vertices.add(vertex_index)
                    entry_vertices.add(vertex_index)
                matched_faces += 1
            if matched_faces == 0:
                raise RuntimeError(
                    f"two-curve bridge entry matched no persistent curve sectors: "
                    f"{entry['sheet']} {entry['boundary_curves']}"
                )
            report_entries.append({
                "mode": entry["mode"],
                "sheet": entry["sheet"],
                "boundary_curves": list(entry["boundary_curves"]),
                "certified_lower_m": entry["certified_lower_m"],
                "certified_upper_m": entry["certified_upper_m"],
                "target_size_m": entry["target_size_m"],
                "matched_seed_faces": matched_faces,
                "adjusted_seed_vertices": len(entry_vertices),
                "seed_representation": "persistent_curve_tangent_sectors",
            })
            continue
        for face in faces:
            if face.sheet_ref != entry["sheet_ref"]:
                continue
            local_curve_sets = [vertex_curves[v] for v in face.vertices]
            selected = [
                i for i, refs in enumerate(local_curve_sets)
                if refs.intersection(entry["curve_refs"])
            ]
            if entry["mode"] == "same_curve_sector":
                if len(entry["curve_refs"]) != 1 or len(selected) != 2:
                    continue
                if not all(entry["curve_refs"][0] in local_curve_sets[i] for i in selected):
                    continue
            else:
                raise ValueError(f"unknown two-sided mode: {entry['mode']}")
            interior = [i for i in range(3) if i not in selected]
            if len(interior) != 1:
                continue
            i, j = selected
            k = interior[0]
            boundary_midpoint = 0.5 * (
                vertices[face.vertices[i]].point + vertices[face.vertices[j]].point
            )
            raw_direction = vertices[face.vertices[k]].point - boundary_midpoint
            sheet_normal = authority.exact_normal(
                face.sheet_ref, vertices[face.vertices[k]].point
            )
            direction = raw_direction - np.dot(raw_direction, sheet_normal) * sheet_normal
            if np.linalg.norm(direction) <= 1.0e-14:
                continue
            direction = _unit(direction)
            for vertex_index in face.vertices:
                adjusted[vertex_index] = _replace_directional_size(
                    adjusted[vertex_index], direction, float(entry["target_size_m"])
                )
                touched_vertices.add(vertex_index)
                entry_vertices.add(vertex_index)
            matched_faces += 1
        if matched_faces == 0:
            raise RuntimeError(
                f"two-sided policy entry matched no seed faces: {entry['sheet']} {entry['boundary_curves']}"
            )
        report_entries.append({
            "mode": entry["mode"],
            "sheet": entry["sheet"],
            "boundary_curves": list(entry["boundary_curves"]),
            "certified_lower_m": entry["certified_lower_m"],
            "certified_upper_m": entry["certified_upper_m"],
            "target_size_m": entry["target_size_m"],
            "matched_seed_faces": matched_faces,
            "adjusted_seed_vertices": len(entry_vertices),
        })

    return adjusted, {
        "enabled": True,
        "policy_schema": policy["schema"],
        "policy_sha256": policy["policy_sha256"],
        "entries": report_entries,
        "adjusted_seed_vertices": len(touched_vertices),
        "raw_face_ids_used": False,
        "coordinate_regions_used": False,
    }
