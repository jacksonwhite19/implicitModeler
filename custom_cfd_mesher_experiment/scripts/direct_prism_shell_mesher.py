from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
import trimesh.repair

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from direct_cartesian_cfd_mesher import (  # noqa: E402
    FaceRecord,
    foam_face_list,
    foam_header,
    foam_label_list,
    foam_vector_list,
    write_openfoam_control,
    write_text,
)


@dataclass(frozen=True)
class PrismCell:
    inner: tuple[int, int, int]
    outer: tuple[int, int, int]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--offset", type=float, default=0.35)
    parser.add_argument("--offsets", default="")
    parser.add_argument("--target-faces", type=int, default=0)
    parser.add_argument("--min-triangle-height", type=float, default=1e-6)
    parser.add_argument("--adaptive-offset-caps", action="store_true")
    parser.add_argument("--offset-cap-factor", type=float, default=0.5)
    parser.add_argument("--offset-cap-floor", type=float, default=1e-5)
    parser.add_argument("--outer-mode", choices=["normal", "radial-box", "radial-sphere"], default="normal")
    parser.add_argument("--farfield-padding", default="0.7,0.5,0.5")
    parser.add_argument("--farfield-fractions", default="0.25,0.55,1.0")
    parser.add_argument("--write-openfoam", action="store_true")
    parser.add_argument("--write-su2", action="store_true")
    args = parser.parse_args()

    if not args.write_openfoam and not args.write_su2:
        args.write_openfoam = True

    source, cleanup = load_surface(
        args.input_stl,
        scale=args.scale,
        target_faces=args.target_faces,
        min_triangle_height=args.min_triangle_height,
    )
    offsets = parse_offsets(args.offsets, fallback=args.offset)
    farfield_padding = parse_vector3(args.farfield_padding, "--farfield-padding")
    farfield_fractions = parse_fractions(args.farfield_fractions)
    points, cells, faces, build_report = build_prism_shell(
        source,
        offsets=offsets,
        adaptive_offset_caps=args.adaptive_offset_caps,
        offset_cap_factor=args.offset_cap_factor,
        offset_cap_floor=args.offset_cap_floor,
        outer_mode=args.outer_mode,
        farfield_padding=farfield_padding,
        farfield_fractions=farfield_fractions,
    )
    args.run_dir.mkdir(parents=True, exist_ok=True)

    if args.write_openfoam:
        write_openfoam_shell(args.run_dir / "openfoam_case" / "constant" / "polyMesh", points, faces)
        write_openfoam_control(args.run_dir / "openfoam_case" / "system")
    if args.write_su2:
        write_su2_prism_shell(args.run_dir / "mesh.su2", points, cells, faces)
    patch_surfaces = write_patch_surfaces(args.run_dir, points, faces)

    report = {
        "input_stl": str(args.input_stl),
        "strategy": "direct_body_fitted_prism_shell_v0",
        "scale": args.scale,
        "offset": args.offset,
        "offsets": offsets,
        "adaptive_offset_caps": args.adaptive_offset_caps,
        "offset_cap_factor": args.offset_cap_factor,
        "offset_cap_floor": args.offset_cap_floor,
        "outer_mode": args.outer_mode,
        "farfield_padding": farfield_padding,
        "farfield_fractions": farfield_fractions,
        "build_report": build_report,
        "target_faces": args.target_faces or None,
        "min_triangle_height": args.min_triangle_height,
        "surface_cleanup": cleanup,
        "source_vertices": int(len(source.vertices)),
        "source_faces": int(len(source.faces)),
        "source_watertight": bool(source.is_watertight),
        "points": int(len(points)),
        "prism_cells": int(len(cells)),
        "faces_total": int(len(faces)),
        "internal_faces": int(sum(1 for face in faces if face.neighbour is not None)),
        "aircraft_faces": int(sum(1 for face in faces if face.patch == "aircraft")),
        "farfield_faces": int(sum(1 for face in faces if face.patch == "farfield")),
        "patch_surfaces": patch_surfaces,
        "bounds": {
            "aircraft": source.bounds.tolist(),
            "mesh": np.array([points.min(axis=0), points.max(axis=0)]).tolist(),
        },
        "known_limitations": [
            "Farfield is currently an offset shell, not a rectangular/tunnel domain.",
            "Radial-box mode is experimental and keeps surface topology while projecting outer layers to a padded box.",
            "Layer growth is normal-based and does not yet include local rollback/smoothing.",
            "Offset can self-intersect on concave geometry if the requested distance is too large.",
        ],
    }
    (args.run_dir / "direct_prism_shell_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def load_surface(
    path: Path,
    *,
    scale: float,
    target_faces: int,
    min_triangle_height: float,
) -> tuple[trimesh.Trimesh, dict[str, int | bool]]:
    mesh = trimesh.load(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("Expected a single triangle mesh")
    mesh = mesh.copy()
    mesh.vertices *= scale
    mesh.merge_vertices(digits_vertex=9)
    trimesh.repair.fix_normals(mesh, multibody=True)
    if mesh.volume < 0:
        mesh.invert()
    if target_faces and len(mesh.faces) > target_faces:
        mesh = mesh.simplify_quadric_decimation(face_count=target_faces, aggression=3)
        mesh.merge_vertices(digits_vertex=9)
        trimesh.repair.fix_normals(mesh, multibody=True)
        if mesh.volume < 0:
            mesh.invert()
    before_cleanup_faces = int(len(mesh.faces))
    keep = mesh.nondegenerate_faces(height=min_triangle_height)
    removed_degenerate_faces = int((~keep).sum())
    if removed_degenerate_faces:
        mesh.update_faces(keep)
        mesh.remove_unreferenced_vertices()
        mesh.merge_vertices(digits_vertex=9)
        trimesh.repair.fix_normals(mesh, multibody=True)
        if mesh.volume < 0:
            mesh.invert()
    if not mesh.is_watertight:
        raise ValueError("Prism shell requires a watertight source surface")
    return mesh, {
        "faces_before_cleanup": before_cleanup_faces,
        "removed_degenerate_faces": removed_degenerate_faces,
        "faces_after_cleanup": int(len(mesh.faces)),
        "watertight_after_cleanup": bool(mesh.is_watertight),
    }


def write_patch_surfaces(run_dir: Path, points: np.ndarray, faces: list[FaceRecord]) -> dict[str, str]:
    surfaces_dir = run_dir / "surfaces"
    surfaces_dir.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, str] = {}
    for patch_name, filename in {
        "aircraft": "aircraft_wall.stl",
        "farfield": "outer_shell.stl",
    }.items():
        triangles = [
            tuple(int(vertex) for vertex in face.vertices)
            for face in faces
            if face.patch == patch_name and len(face.vertices) == 3
        ]
        if not triangles:
            continue
        path = surfaces_dir / filename
        write_ascii_stl(path, points, triangles, solid_name=patch_name)
        outputs[patch_name] = str(path)
    return outputs


def write_ascii_stl(
    path: Path,
    points: np.ndarray,
    triangles: list[tuple[int, int, int]],
    *,
    solid_name: str,
) -> None:
    lines = [f"solid {solid_name}"]
    for triangle in triangles:
        p0, p1, p2 = (points[index] for index in triangle)
        normal = np.cross(p1 - p0, p2 - p0)
        length = float(np.linalg.norm(normal))
        if length > 1e-20:
            normal = normal / length
        else:
            normal = np.zeros(3, dtype=float)
        lines.append(f"  facet normal {normal[0]:.17g} {normal[1]:.17g} {normal[2]:.17g}")
        lines.append("    outer loop")
        for vertex in (p0, p1, p2):
            lines.append(f"      vertex {vertex[0]:.17g} {vertex[1]:.17g} {vertex[2]:.17g}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {solid_name}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def parse_offsets(value: str, *, fallback: float) -> list[float]:
    if not value.strip():
        return [float(fallback)]
    offsets = [float(part) for part in value.split(",") if part.strip()]
    if not offsets or any(offset <= 0.0 for offset in offsets):
        raise ValueError("--offsets must contain positive distances")
    if offsets != sorted(offsets):
        raise ValueError("--offsets must be cumulative and increasing")
    return offsets


def parse_vector3(value: str, label: str) -> list[float]:
    parts = [float(part) for part in value.split(",") if part.strip()]
    if len(parts) != 3 or any(part <= 0.0 for part in parts):
        raise ValueError(f"{label} must contain three positive numbers")
    return parts


def parse_fractions(value: str) -> list[float]:
    fractions = [float(part) for part in value.split(",") if part.strip()]
    if not fractions or any(frac <= 0.0 or frac > 1.0 for frac in fractions):
        raise ValueError("--farfield-fractions must contain values in (0, 1]")
    if fractions != sorted(fractions) or fractions[-1] != 1.0:
        raise ValueError("--farfield-fractions must be increasing and end at 1.0")
    return fractions


def build_prism_shell(
    mesh: trimesh.Trimesh,
    *,
    offsets: list[float],
    adaptive_offset_caps: bool,
    offset_cap_factor: float,
    offset_cap_floor: float,
    outer_mode: str,
    farfield_padding: list[float],
    farfield_fractions: list[float],
) -> tuple[np.ndarray, list[PrismCell], list[FaceRecord], dict[str, object]]:
    inner_points = np.asarray(mesh.vertices, dtype=float)
    normals = np.asarray(mesh.vertex_normals, dtype=float)
    normal_lengths = np.linalg.norm(normals, axis=1)
    normals = normals / np.maximum(normal_lengths[:, None], 1e-12)
    caps = compute_vertex_offset_caps(
        mesh,
        max_offset=offsets[-1],
        factor=offset_cap_factor,
        floor=offset_cap_floor,
    )
    if adaptive_offset_caps:
        effective_max = np.minimum(offsets[-1], caps)
        layers = [inner_points]
        for offset in offsets:
            layer_offsets = effective_max * (offset / offsets[-1])
            layers.append(inner_points + layer_offsets[:, None] * normals)
    else:
        effective_max = np.full(len(inner_points), offsets[-1], dtype=float)
        layers = [inner_points] + [inner_points + offset * normals for offset in offsets]
    if outer_mode in {"radial-box", "radial-sphere"}:
        if outer_mode == "radial-box":
            outer_targets = radial_box_targets(inner_points, normals, mesh.bounds, farfield_padding)
        else:
            outer_targets = radial_sphere_targets(inner_points, normals, mesh.bounds, farfield_padding)
        transition_start = layers[-1]
        layers.extend(
            transition_start * (1.0 - fraction) + outer_targets * fraction
            for fraction in farfield_fractions
        )
    points = np.vstack(layers)
    layer_count = len(layers)
    vertices_per_layer = len(inner_points)
    face_count = len(mesh.faces)

    cells: list[PrismCell] = []
    for layer_index in range(layer_count - 1):
        lower_shift = layer_index * vertices_per_layer
        upper_shift = (layer_index + 1) * vertices_per_layer
        for face in mesh.faces:
            cells.append(
                PrismCell(
                    inner=(int(face[0] + lower_shift), int(face[1] + lower_shift), int(face[2] + lower_shift)),
                    outer=(int(face[0] + upper_shift), int(face[1] + upper_shift), int(face[2] + upper_shift)),
                )
            )

    centroids = np.array(
        [
            np.vstack([points[list(cell.inner)], points[list(cell.outer)]]).mean(axis=0)
            for cell in cells
        ]
    )

    edge_to_faces: dict[tuple[int, int], list[int]] = {}
    for face_index, face in enumerate(mesh.faces):
        for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge = tuple(sorted((int(a), int(b))))
            edge_to_faces.setdefault(edge, []).append(face_index)

    internal: list[FaceRecord] = []
    aircraft: list[FaceRecord] = []
    farfield: list[FaceRecord] = []

    for layer_index in range(layer_count - 1):
        lower_shift = layer_index * vertices_per_layer
        upper_shift = (layer_index + 1) * vertices_per_layer
        cell_shift = layer_index * face_count
        for edge, adjacent in edge_to_faces.items():
            if len(adjacent) != 2:
                raise ValueError(f"Expected manifold edge with two adjacent faces, got {len(adjacent)}")
            owner_face, neighbour_face = sorted((int(adjacent[0]), int(adjacent[1])))
            owner = cell_shift + owner_face
            neighbour = cell_shift + neighbour_face
            a, b = edge
            raw = (a + lower_shift, b + lower_shift, b + upper_shift, a + upper_shift)
            oriented = orient_face(points, raw, centroids[neighbour] - centroids[owner])
            internal.append(FaceRecord(oriented, owner, neighbour, None))

    for layer_index in range(layer_count - 2):
        lower_cell_shift = layer_index * face_count
        upper_cell_shift = (layer_index + 1) * face_count
        interface_shift = (layer_index + 1) * vertices_per_layer
        for face_index, face in enumerate(mesh.faces):
            owner = lower_cell_shift + face_index
            neighbour = upper_cell_shift + face_index
            raw = tuple(int(vertex + interface_shift) for vertex in face)
            oriented = orient_face(points, raw, centroids[neighbour] - centroids[owner])
            internal.append(FaceRecord(oriented, owner, neighbour, None))

    for face_index, face in enumerate(mesh.faces):
        cell_index = face_index
        centroid = centroids[cell_index]
        inner_raw = tuple(int(vertex) for vertex in face)
        inner_center = points[list(inner_raw)].mean(axis=0)
        inner = orient_face(points, inner_raw, inner_center - centroid)
        aircraft.append(FaceRecord(inner, cell_index, None, "aircraft"))

    farfield_cell_shift = (layer_count - 2) * face_count
    farfield_vertex_shift = (layer_count - 1) * vertices_per_layer
    for face_index, face in enumerate(mesh.faces):
        cell_index = farfield_cell_shift + face_index
        centroid = centroids[cell_index]
        outer_raw = tuple(int(vertex + farfield_vertex_shift) for vertex in face)
        outer_center = points[list(outer_raw)].mean(axis=0)
        outer = orient_face(points, outer_raw, outer_center - centroid)
        farfield.append(FaceRecord(outer, cell_index, None, "farfield"))

    internal.sort(key=lambda face: (face.owner, -1 if face.neighbour is None else face.neighbour))
    farfield.sort(key=lambda face: face.owner)
    aircraft.sort(key=lambda face: face.owner)
    build_report = {
        "offset_cap_stats": percentile_report(caps),
        "effective_max_offset_stats": percentile_report(effective_max),
        "capped_vertices_at_final_offset": int(np.count_nonzero(caps < offsets[-1])),
        "total_vertices": int(len(inner_points)),
        "outer_mode": outer_mode,
        "layer_count": int(len(layers) - 1),
    }
    return points, cells, internal + farfield + aircraft, build_report


def radial_box_targets(
    points: np.ndarray,
    normals: np.ndarray,
    bounds: np.ndarray,
    padding: list[float],
) -> np.ndarray:
    pad = np.asarray(padding, dtype=float)
    box_min = bounds[0] - pad
    box_max = bounds[1] + pad
    center = 0.5 * (bounds[0] + bounds[1])
    targets = np.zeros_like(points)
    for index, point in enumerate(points):
        direction = point - center
        if float(np.linalg.norm(direction)) < 1e-12:
            direction = normals[index]
        candidates = []
        for axis in range(3):
            if direction[axis] > 0.0:
                candidates.append((box_max[axis] - center[axis]) / direction[axis])
            elif direction[axis] < 0.0:
                candidates.append((box_min[axis] - center[axis]) / direction[axis])
        positive = [value for value in candidates if value > 1.0]
        scale = min(positive) if positive else 1.0
        targets[index] = center + scale * direction
    return targets


def radial_sphere_targets(
    points: np.ndarray,
    normals: np.ndarray,
    bounds: np.ndarray,
    padding: list[float],
) -> np.ndarray:
    center = 0.5 * (bounds[0] + bounds[1])
    radius = 0.5 * float(np.linalg.norm(bounds[1] - bounds[0])) + max(padding)
    targets = np.zeros_like(points)
    for index, point in enumerate(points):
        direction = point - center
        length = float(np.linalg.norm(direction))
        if length < 1e-12:
            direction = normals[index]
            length = float(np.linalg.norm(direction))
        targets[index] = center + radius * direction / max(length, 1e-12)
    return targets


def compute_vertex_offset_caps(
    mesh: trimesh.Trimesh,
    *,
    max_offset: float,
    factor: float,
    floor: float,
) -> np.ndarray:
    vertices = np.asarray(mesh.vertices, dtype=float)
    caps = np.full(len(vertices), max_offset, dtype=float)
    for face in mesh.faces:
        points = vertices[face]
        ab = float(np.linalg.norm(points[1] - points[0]))
        bc = float(np.linalg.norm(points[2] - points[1]))
        ca = float(np.linalg.norm(points[0] - points[2]))
        longest = max(ab, bc, ca)
        area = 0.5 * float(np.linalg.norm(np.cross(points[1] - points[0], points[2] - points[0])))
        if longest <= 0.0:
            local_cap = floor
        else:
            min_altitude = 2.0 * area / longest
            local_cap = max(floor, factor * min_altitude)
        for vertex in face:
            caps[int(vertex)] = min(caps[int(vertex)], local_cap)
    return caps


def percentile_report(values: np.ndarray) -> dict[str, float]:
    return {
        "min": float(np.min(values)),
        "p01": float(np.percentile(values, 1)),
        "p05": float(np.percentile(values, 5)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def orient_face(points: np.ndarray, vertices: tuple[int, ...], desired: np.ndarray) -> tuple[int, ...]:
    normal = face_normal(points[list(vertices)])
    if float(np.dot(normal, desired)) < 0.0:
        return tuple(reversed(vertices))
    return vertices


def face_normal(vertices: np.ndarray) -> np.ndarray:
    normal = np.zeros(3, dtype=float)
    for current, nxt in zip(vertices, np.roll(vertices, -1, axis=0)):
        normal[0] += (current[1] - nxt[1]) * (current[2] + nxt[2])
        normal[1] += (current[2] - nxt[2]) * (current[0] + nxt[0])
        normal[2] += (current[0] - nxt[0]) * (current[1] + nxt[1])
    return normal


def write_openfoam_shell(poly_dir: Path, points: np.ndarray, faces: list[FaceRecord]) -> None:
    poly_dir.mkdir(parents=True, exist_ok=True)
    internal_count = sum(1 for face in faces if face.neighbour is not None)
    farfield_count = sum(1 for face in faces if face.patch == "farfield")
    aircraft_count = sum(1 for face in faces if face.patch == "aircraft")
    write_text(poly_dir / "points", foam_header("vectorField", "points") + foam_vector_list(points))
    write_text(poly_dir / "faces", foam_header("faceList", "faces") + foam_face_list([face.vertices for face in faces]))
    write_text(poly_dir / "owner", foam_header("labelList", "owner") + foam_label_list([face.owner for face in faces]))
    write_text(
        poly_dir / "neighbour",
        foam_header("labelList", "neighbour") + foam_label_list([face.neighbour for face in faces if face.neighbour is not None]),
    )
    boundary = (
        foam_header("polyBoundaryMesh", "boundary")
        + "2\n(\n"
        + f"    farfield\n    {{\n        type            patch;\n        nFaces          {farfield_count};\n        startFace       {internal_count};\n    }}\n\n"
        + f"    aircraft\n    {{\n        type            wall;\n        nFaces          {aircraft_count};\n        startFace       {internal_count + farfield_count};\n    }}\n"
        + ")\n\n// ************************************************************************* //\n"
    )
    write_text(poly_dir / "boundary", boundary)


def write_su2_prism_shell(path: Path, points: np.ndarray, cells: list[PrismCell], faces: list[FaceRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    farfield = [face for face in faces if face.patch == "farfield"]
    aircraft = [face for face in faces if face.patch == "aircraft"]
    lines = ["NDIME= 3", f"NPOIN= {len(points)}"]
    lines.extend(f"{x:.12g} {y:.12g} {z:.12g} {idx}" for idx, (x, y, z) in enumerate(points))
    lines.append(f"NELEM= {len(cells)}")
    for idx, cell in enumerate(cells):
        vertices = cell.inner + cell.outer
        lines.append("13 " + " ".join(str(vertex) for vertex in vertices) + f" {idx}")
    lines.append("NMARK= 2")
    append_su2_tri_marker(lines, "farfield", farfield)
    append_su2_tri_marker(lines, "aircraft", aircraft)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def append_su2_tri_marker(lines: list[str], name: str, faces: list[FaceRecord]) -> None:
    lines.append(f"MARKER_TAG= {name}")
    lines.append(f"MARKER_ELEMS= {len(faces)}")
    for face in faces:
        if len(face.vertices) != 3:
            raise ValueError("Prism shell boundary markers should be triangles")
        lines.append("5 " + " ".join(str(vertex) for vertex in face.vertices))


if __name__ == "__main__":
    main()
