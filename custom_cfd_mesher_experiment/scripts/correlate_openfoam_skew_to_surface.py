from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from bl_surface_gate import (
    cluster_bad_triangles,
    triangle_angles_degrees,
    triangle_area,
    triangle_aspect_ratio,
    triangle_edge_lengths,
    triangle_min_altitude,
    triangle_quality_from_edges,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Map OpenFOAM problem-face VTK centers to nearest STL surface triangles.")
    parser.add_argument("--surface-stl", type=Path, required=True)
    parser.add_argument("--vtk", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--scale-vtk-to-mm", type=float, default=1000.0)
    args = parser.parse_args()

    report = correlate(args.surface_stl, args.vtk, scale_vtk_to_mm=args.scale_vtk_to_mm)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def correlate(surface_stl: Path, vtk: Path, *, scale_vtk_to_mm: float) -> dict[str, Any]:
    mesh = trimesh.load(surface_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh: {surface_stl}")
    mesh.merge_vertices(digits_vertex=8)
    tri = mesh.vertices[mesh.faces]
    centers = tri.mean(axis=1)
    edges = triangle_edge_lengths(tri)
    area = triangle_area(tri)
    quality = triangle_quality_from_edges(edges, area)
    altitude = triangle_min_altitude(edges, area)
    aspect = triangle_aspect_ratio(edges, altitude)
    angles = triangle_angles_degrees(edges)
    min_angle = np.min(angles, axis=1)
    max_angle = np.max(angles, axis=1)

    vtk_centers = read_vtk_cell_centers(vtk) * scale_vtk_to_mm
    tree = cKDTree(centers)
    distances, indices = tree.query(vtk_centers, k=1)
    nearest = np.asarray(indices, dtype=int)
    records = []
    for row, tri_idx in enumerate(nearest):
        records.append(
            {
                "problem_face_index": int(row),
                "problem_center_mm": vtk_centers[row].tolist(),
                "nearest_surface_face": int(tri_idx),
                "distance_mm": float(distances[row]),
                "surface_center_mm": centers[tri_idx].tolist(),
                "quality": float(quality[tri_idx]),
                "aspect_ratio": float(aspect[tri_idx]),
                "min_angle_deg": float(min_angle[tri_idx]),
                "max_angle_deg": float(max_angle[tri_idx]),
                "min_altitude_mm": float(altitude[tri_idx]),
                "edge_lengths_mm": edges[tri_idx].tolist(),
            }
        )

    unique_indices = np.unique(nearest)
    return {
        "surface_stl": str(surface_stl),
        "vtk": str(vtk),
        "problem_faces": int(len(vtk_centers)),
        "unique_nearest_surface_faces": int(len(unique_indices)),
        "nearest_distance_mm": summarize(distances),
        "nearest_surface_triangle_quality": summarize(quality[nearest]),
        "nearest_surface_aspect_ratio": summarize(aspect[nearest]),
        "nearest_surface_min_angle_deg": summarize(min_angle[nearest]),
        "nearest_surface_min_altitude_mm": summarize(altitude[nearest]),
        "clusters": cluster_bad_triangles(vtk_centers, mesh.bounds, grid=(8, 8, 5)),
        "records": records,
    }


def read_vtk_cell_centers(path: Path) -> np.ndarray:
    text = path.read_text(encoding="utf-8", errors="replace")
    points_match = re.search(r"POINTS\s+(\d+)\s+\w+\s+(.+?)(?:\nPOLYGONS|\nCELLS)", text, re.S)
    if not points_match:
        raise ValueError(f"Could not parse POINTS from {path}")
    point_count = int(points_match.group(1))
    points = np.fromstring(points_match.group(2), sep=" ", dtype=float).reshape((point_count, 3))

    cell_match = re.search(r"(?:POLYGONS|CELLS)\s+(\d+)\s+\d+\s+(.+?)(?:\nCELL_TYPES|\nPOINT_DATA|\nCELL_DATA|$)", text, re.S)
    if not cell_match:
        raise ValueError(f"Could not parse cells from {path}")
    cell_count = int(cell_match.group(1))
    values = np.fromstring(cell_match.group(2), sep=" ", dtype=int)
    centers = []
    cursor = 0
    for _ in range(cell_count):
        n = int(values[cursor])
        cursor += 1
        indices = values[cursor : cursor + n]
        cursor += n
        centers.append(points[indices].mean(axis=0))
    return np.asarray(centers, dtype=float)


def summarize(values: np.ndarray) -> dict[str, float | None]:
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        return {"min": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "min": float(np.min(finite)),
        "p50": float(np.percentile(finite, 50)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "max": float(np.max(finite)),
    }


if __name__ == "__main__":
    main()
