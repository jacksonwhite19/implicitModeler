from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
import trimesh.repair


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--max-cap-loop-span-mm", type=float, default=8.0)
    parser.add_argument("--merge-digits", type=int, default=8)
    args = parser.parse_args()

    mesh = load_mesh(args.input_stl)
    initial = mesh_report(mesh)

    cap_reports = cap_small_boundary_loops(
        mesh,
        max_loop_span_mm=args.max_cap_loop_span_mm,
    )
    cleanup_mesh(mesh, merge_digits=args.merge_digits)
    trimesh.repair.fix_normals(mesh, multibody=True)
    cleanup_mesh(mesh, merge_digits=args.merge_digits)
    final = mesh_report(mesh)

    args.output_stl.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.output_stl)

    report = {
        "input_stl": str(args.input_stl),
        "output_stl": str(args.output_stl),
        "strategy": "preserve_surface_cap_small_boundary_loops",
        "max_cap_loop_span_mm": args.max_cap_loop_span_mm,
        "merge_digits": args.merge_digits,
        "caps": cap_reports,
        "initial": initial,
        "final": final,
    }
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def load_mesh(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one STL mesh, got {type(mesh)!r}")
    mesh = mesh.copy()
    cleanup_mesh(mesh, merge_digits=8)
    return mesh


def cleanup_mesh(mesh: trimesh.Trimesh, *, merge_digits: int) -> None:
    if hasattr(mesh, "remove_duplicate_faces"):
        mesh.remove_duplicate_faces()
    if hasattr(mesh, "remove_degenerate_faces"):
        mesh.remove_degenerate_faces()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices(digits_vertex=merge_digits)
    mesh.remove_unreferenced_vertices()


def mesh_report(mesh: trimesh.Trimesh) -> dict[str, Any]:
    quality = triangle_quality(mesh)
    edge_counts = unique_edge_counts(mesh)
    boundary_edges = int(np.count_nonzero(edge_counts == 1))
    nonmanifold_edges = int(np.count_nonzero(edge_counts > 2))
    edge_lengths = mesh.edges_unique_length if len(mesh.edges_unique) else np.array([])
    return {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "euler_number": int(mesh.euler_number),
        "body_count": int(len(mesh.split(only_watertight=False))),
        "boundary_edges": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "bounds_mm": mesh.bounds.tolist(),
        "area_mm2": float(mesh.area),
        "volume_mm3_signed": float(mesh.volume),
        "edge_length_mm": percentile_report(edge_lengths),
        "triangle_quality": percentile_report(quality),
        "sliver_faces_q_lt_0p05": int(np.count_nonzero(quality < 0.05)),
        "sliver_faces_q_lt_0p10": int(np.count_nonzero(quality < 0.10)),
    }


def triangle_quality(mesh: trimesh.Trimesh) -> np.ndarray:
    if len(mesh.faces) == 0:
        return np.array([], dtype=float)
    tri = mesh.vertices[mesh.faces]
    ab = np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1)
    bc = np.linalg.norm(tri[:, 2] - tri[:, 1], axis=1)
    ca = np.linalg.norm(tri[:, 0] - tri[:, 2], axis=1)
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    denom = ab * ab + bc * bc + ca * ca
    return np.divide(
        4.0 * np.sqrt(3.0) * area,
        denom,
        out=np.zeros_like(area),
        where=denom > 0.0,
    )


def unique_edge_counts(mesh: trimesh.Trimesh) -> np.ndarray:
    counts = np.zeros(len(mesh.edges_unique), dtype=np.int32)
    for edge_index in mesh.edges_unique_inverse:
        counts[int(edge_index)] += 1
    return counts


def percentile_report(values: np.ndarray) -> dict[str, float | None]:
    if len(values) == 0:
        return {"min": None, "p01": None, "p05": None, "p50": None, "p95": None, "max": None}
    return {
        "min": float(np.min(values)),
        "p01": float(np.percentile(values, 1)),
        "p05": float(np.percentile(values, 5)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def cap_small_boundary_loops(
    mesh: trimesh.Trimesh,
    *,
    max_loop_span_mm: float,
) -> list[dict[str, Any]]:
    loops = boundary_loops(mesh)
    cap_reports: list[dict[str, Any]] = []
    new_vertices: list[np.ndarray] = []
    new_faces: list[list[int]] = []

    for loop in loops:
        points = mesh.vertices[loop]
        span = points.max(axis=0) - points.min(axis=0)
        perimeter = loop_perimeter(points)
        if float(span.max()) > max_loop_span_mm:
            cap_reports.append(
                {
                    "loop_vertices": len(loop),
                    "span_mm": span.tolist(),
                    "perimeter_mm": perimeter,
                    "capped": False,
                    "reason": "loop_exceeds_max_span",
                }
            )
            continue
        center_index = len(mesh.vertices) + len(new_vertices)
        center = points.mean(axis=0)
        new_vertices.append(center)
        for i, vertex in enumerate(loop):
            new_faces.append([int(vertex), int(loop[(i + 1) % len(loop)]), int(center_index)])
        cap_reports.append(
            {
                "loop_vertices": len(loop),
                "span_mm": span.tolist(),
                "perimeter_mm": perimeter,
                "capped": True,
                "new_faces": len(loop),
            }
        )

    if new_vertices:
        mesh.vertices = np.vstack([mesh.vertices, np.asarray(new_vertices)])
        mesh.faces = np.vstack([mesh.faces, np.asarray(new_faces, dtype=np.int64)])
    return cap_reports


def loop_perimeter(points: np.ndarray) -> float:
    shifted = np.roll(points, -1, axis=0)
    return float(np.linalg.norm(shifted - points, axis=1).sum())


def boundary_loops(mesh: trimesh.Trimesh) -> list[list[int]]:
    edge_counts: dict[tuple[int, int], int] = {}
    for face in mesh.faces:
        for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge = tuple(sorted((int(a), int(b))))
            edge_counts[edge] = edge_counts.get(edge, 0) + 1
    boundary_edges = [edge for edge, count in edge_counts.items() if count == 1]

    adjacency: dict[int, list[int]] = {}
    for a, b in boundary_edges:
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)

    loops: list[list[int]] = []
    visited_edges: set[tuple[int, int]] = set()
    for start_a, start_b in boundary_edges:
        start_edge = tuple(sorted((start_a, start_b)))
        if start_edge in visited_edges:
            continue
        loop = [start_a, start_b]
        visited_edges.add(start_edge)
        previous = start_a
        current = start_b
        while True:
            candidates = [vertex for vertex in adjacency[current] if vertex != previous]
            if not candidates:
                break
            next_vertex = candidates[0]
            next_edge = tuple(sorted((current, next_vertex)))
            if next_edge in visited_edges:
                break
            visited_edges.add(next_edge)
            if next_vertex == loop[0]:
                break
            loop.append(next_vertex)
            previous, current = current, next_vertex
        loops.append(loop)
    return loops


if __name__ == "__main__":
    main()
