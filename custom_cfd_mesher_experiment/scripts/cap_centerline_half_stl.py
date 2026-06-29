from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import mapbox_earcut as earcut
import numpy as np
import trimesh
try:
    import triangle as triangle_lib
except ImportError:  # pragma: no cover - optional experiment dependency
    triangle_lib = None


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def main() -> None:
    parser = argparse.ArgumentParser(description="Cap open half-aircraft STL loops on a symmetry plane.")
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    parser.add_argument("--report-json", type=Path, required=True)
    parser.add_argument("--axis", choices=sorted(AXIS_INDEX), default="y")
    parser.add_argument("--plane", type=float, default=0.0)
    parser.add_argument("--plane-tolerance", type=float, default=1e-4)
    parser.add_argument("--triangulator", choices=["earcut", "triangle"], default="earcut")
    parser.add_argument(
        "--max-cap-area",
        type=float,
        default=25.0,
        help="Maximum centerline cap triangle area in input STL units squared when using --triangulator triangle.",
    )
    parser.add_argument(
        "--allow-boundary-steiner",
        action="store_true",
        help="Allow Triangle to insert extra points on the cap boundary. Usually off so the cap welds to the STL loop.",
    )
    args = parser.parse_args()

    mesh = trimesh.load(args.input_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected a single mesh: {args.input_stl}")
    mesh = mesh.copy()
    mesh.merge_vertices(digits_vertex=9)

    axis = AXIS_INDEX[args.axis]
    pre = mesh_summary(mesh)
    boundary_edges = find_boundary_edges(mesh.faces)
    loops = build_boundary_loops(boundary_edges)
    cap_faces: list[list[int]] = []
    loop_reports = []
    for loop in loops:
        coords = mesh.vertices[np.asarray(loop, dtype=np.int64)]
        plane_dist = np.abs(coords[:, axis] - args.plane)
        on_plane = bool(np.max(plane_dist) <= args.plane_tolerance)
        report = {
            "vertices": len(loop),
            "on_plane": on_plane,
            "max_plane_distance": float(np.max(plane_dist)),
            "capped": False,
            "new_faces": 0,
        }
        if on_plane and len(loop) >= 3:
            new_vertices, faces = triangulate_loop(
                mesh.vertices,
                loop,
                axis=axis,
                plane=args.plane,
                triangulator=args.triangulator,
                max_area=args.max_cap_area,
                allow_boundary_steiner=args.allow_boundary_steiner,
            )
            if len(new_vertices):
                first_new_index = len(mesh.vertices)
                mesh.vertices = np.vstack([mesh.vertices, new_vertices])
                faces = [
                    [first_new_index + (-int(vertex) - 1) if int(vertex) < 0 else int(vertex) for vertex in face]
                    for face in faces
                ]
            cap_faces.extend(faces)
            report["capped"] = True
            report["new_faces"] = len(faces)
            report["new_vertices"] = int(len(new_vertices))
        loop_reports.append(report)

    if cap_faces:
        mesh.faces = np.vstack([mesh.faces, np.asarray(cap_faces, dtype=np.int64)])
        if hasattr(mesh, "remove_degenerate_faces"):
            mesh.remove_degenerate_faces()
        else:
            mesh.update_faces(mesh.nondegenerate_faces())
        if hasattr(mesh, "remove_duplicate_faces"):
            mesh.remove_duplicate_faces()
        else:
            mesh.update_faces(mesh.unique_faces())
        mesh.merge_vertices(digits_vertex=9)
        trimesh.repair.fix_normals(mesh, multibody=True)
        if mesh.volume < 0:
            mesh.invert()

    args.output_stl.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.output_stl)
    report = {
        "input_stl": str(args.input_stl),
        "output_stl": str(args.output_stl),
        "axis": args.axis,
        "plane": args.plane,
        "plane_tolerance": args.plane_tolerance,
        "triangulator": args.triangulator,
        "max_cap_area": args.max_cap_area if args.triangulator == "triangle" else None,
        "allow_boundary_steiner": args.allow_boundary_steiner if args.triangulator == "triangle" else None,
        "pre": pre,
        "post": mesh_summary(mesh),
        "boundary_loop_count": len(loops),
        "capped_loop_count": sum(1 for row in loop_reports if row["capped"]),
        "cap_face_count": int(sum(row["new_faces"] for row in loop_reports)),
        "loops": loop_reports,
    }
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def mesh_summary(mesh: trimesh.Trimesh) -> dict[str, object]:
    return {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "euler_number": int(mesh.euler_number),
        "bounds": mesh.bounds.tolist(),
        "volume": float(mesh.volume),
    }


def find_boundary_edges(faces: np.ndarray) -> list[tuple[int, int]]:
    counts: dict[tuple[int, int], int] = defaultdict(int)
    for tri in faces:
        a, b, c = [int(v) for v in tri]
        for u, v in ((a, b), (b, c), (c, a)):
            counts[tuple(sorted((u, v)))] += 1
    return [edge for edge, count in counts.items() if count == 1]


def build_boundary_loops(edges: list[tuple[int, int]]) -> list[list[int]]:
    adjacency: dict[int, list[int]] = defaultdict(list)
    for a, b in edges:
        adjacency[a].append(b)
        adjacency[b].append(a)
    loops: list[list[int]] = []
    used: set[tuple[int, int]] = set()
    for start_a, start_b in edges:
        edge_key = tuple(sorted((start_a, start_b)))
        if edge_key in used:
            continue
        loop = [start_a, start_b]
        used.add(edge_key)
        prev = start_a
        current = start_b
        while current != start_a:
            candidates = [v for v in adjacency[current] if v != prev]
            next_vertex = None
            for candidate in candidates:
                candidate_key = tuple(sorted((current, candidate)))
                if candidate_key not in used:
                    next_vertex = candidate
                    break
            if next_vertex is None:
                break
            loop.append(next_vertex)
            used.add(tuple(sorted((current, next_vertex))))
            prev, current = current, next_vertex
            if len(loop) > len(edges) + 1:
                raise RuntimeError("Boundary loop traversal did not terminate")
        if len(loop) > 2 and loop[-1] == loop[0]:
            loop = loop[:-1]
        loops.append(loop)
    return loops


def triangulate_loop(
    vertices: np.ndarray,
    loop: list[int],
    *,
    axis: int,
    plane: float,
    triangulator: str,
    max_area: float,
    allow_boundary_steiner: bool,
) -> tuple[np.ndarray, list[list[int]]]:
    projected_axes = [idx for idx in range(3) if idx != axis]
    points = vertices[np.asarray(loop, dtype=np.int64)][:, projected_axes].astype(np.float64)
    if signed_area(points) < 0.0:
        loop = list(reversed(loop))
        points = points[::-1]
    if triangulator == "triangle":
        return triangulate_loop_with_triangle(
            vertices,
            loop,
            points,
            axis=axis,
            plane=plane,
            max_area=max_area,
            allow_boundary_steiner=allow_boundary_steiner,
        )
    ring_ends = np.asarray([len(points)], dtype=np.uint32)
    indices = earcut.triangulate_float64(points, ring_ends)
    faces = [
        [loop[int(indices[i])], loop[int(indices[i + 1])], loop[int(indices[i + 2])]]
        for i in range(0, len(indices), 3)
    ]
    return np.empty((0, 3), dtype=np.float64), faces


def triangulate_loop_with_triangle(
    vertices: np.ndarray,
    loop: list[int],
    points: np.ndarray,
    *,
    axis: int,
    plane: float,
    max_area: float,
    allow_boundary_steiner: bool,
) -> tuple[np.ndarray, list[list[int]]]:
    if triangle_lib is None:
        raise RuntimeError("The triangle package is required for --triangulator triangle")
    if max_area <= 0.0:
        raise ValueError("--max-cap-area must be positive")
    segments = np.column_stack(
        [
            np.arange(len(points), dtype=np.int32),
            np.roll(np.arange(len(points), dtype=np.int32), -1),
        ]
    )
    switches = f"pq30a{max_area}"
    if not allow_boundary_steiner:
        switches += "Y"
    result = triangle_lib.triangulate({"vertices": points, "segments": segments}, switches)
    output_points = np.asarray(result["vertices"], dtype=np.float64)
    output_triangles = np.asarray(result["triangles"], dtype=np.int64)
    new_projected = output_points[len(points) :]
    projected_axes = [idx for idx in range(3) if idx != axis]
    new_vertices = np.zeros((len(new_projected), 3), dtype=np.float64)
    if len(new_projected):
        new_vertices[:, axis] = plane
        new_vertices[:, projected_axes] = new_projected

    faces: list[list[int]] = []
    for tri in output_triangles:
        face: list[int] = []
        for local_index in tri:
            local_index = int(local_index)
            if local_index < len(loop):
                face.append(int(loop[local_index]))
            else:
                face.append(-(local_index - len(loop) + 1))
        faces.append(face)
    return new_vertices, faces


def signed_area(points: np.ndarray) -> float:
    x = points[:, 0]
    y = points[:, 1]
    return float(0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


if __name__ == "__main__":
    main()
