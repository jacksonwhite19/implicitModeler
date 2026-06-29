from __future__ import annotations

import argparse
import json
import re
import shutil
from dataclasses import dataclass
from math import dist
from pathlib import Path


@dataclass
class BoundaryPatch:
    name: str
    type: str
    n_faces: int
    start_face: int
    extra: dict[str, str]


@dataclass
class PolyMesh:
    points: list[tuple[float, float, float]]
    faces: list[list[int]]
    owner: list[int]
    neighbour: list[int]
    patches: list[BoundaryPatch]
    cell_count: int


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Exact merge a direct prism shell OpenFOAM mesh with a Gmsh outer volume mesh."
    )
    parser.add_argument("--shell-case", type=Path, required=True)
    parser.add_argument("--outer-case", type=Path, required=True)
    parser.add_argument("--output-case", type=Path, required=True)
    parser.add_argument("--shell-interface-patch", default="farfield")
    parser.add_argument("--outer-interface-patch", default="aircraft")
    parser.add_argument("--aircraft-patch", default="aircraft")
    parser.add_argument("--farfield-patch", default="farfield")
    parser.add_argument(
        "--outer-boundary-patches",
        default="",
        help=(
            "Comma-separated outer boundary patches to preserve. "
            "Use 'auto' to preserve every outer patch except the interface. "
            "Defaults to --farfield-patch for backward compatibility."
        ),
    )
    parser.add_argument("--coord-digits", type=int, default=11)
    parser.add_argument("--match-tolerance", type=float, default=5.0e-8)
    args = parser.parse_args()

    shell = read_polymesh(args.shell_case / "constant" / "polyMesh")
    outer = read_polymesh(args.outer_case / "constant" / "polyMesh")
    result, report = merge_meshes(
        shell,
        outer,
        shell_interface_patch=args.shell_interface_patch,
        outer_interface_patch=args.outer_interface_patch,
        aircraft_patch=args.aircraft_patch,
        farfield_patch=args.farfield_patch,
        outer_boundary_patches=args.outer_boundary_patches,
        coord_digits=args.coord_digits,
        match_tolerance=args.match_tolerance,
    )

    if args.output_case.exists():
        shutil.rmtree(args.output_case)
    shutil.copytree(args.shell_case, args.output_case)
    poly_dir = args.output_case / "constant" / "polyMesh"
    if poly_dir.exists():
        shutil.rmtree(poly_dir)
    poly_dir.mkdir(parents=True)
    write_polymesh(poly_dir, result)
    (args.output_case / "merge_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def merge_meshes(
    shell: PolyMesh,
    outer: PolyMesh,
    *,
    shell_interface_patch: str,
    outer_interface_patch: str,
    aircraft_patch: str,
    farfield_patch: str,
    outer_boundary_patches: str,
    coord_digits: int,
    match_tolerance: float,
) -> tuple[PolyMesh, dict[str, object]]:
    shell_interface = require_patch(shell, shell_interface_patch)
    outer_interface = require_patch(outer, outer_interface_patch)
    shell_aircraft = require_patch(shell, aircraft_patch)
    outer_boundary_names = select_outer_boundary_names(
        outer,
        outer_interface_patch=outer_interface_patch,
        farfield_patch=farfield_patch,
        outer_boundary_patches=outer_boundary_patches,
    )
    outer_boundary_patch_records = [require_patch(outer, name) for name in outer_boundary_names]

    shell_interface_face_ids = patch_face_ids(shell_interface)
    outer_interface_face_ids = patch_face_ids(outer_interface)
    if len(shell_interface_face_ids) != len(outer_interface_face_ids):
        raise ValueError("Interface patch face counts differ")

    coord_to_shell_vertex: dict[tuple[float, float, float], int] = {}
    shell_interface_vertices: set[int] = set()
    for face_id in shell_interface_face_ids:
        for vertex in shell.faces[face_id]:
            shell_interface_vertices.add(vertex)
            key = coord_key(shell.points[vertex], coord_digits)
            existing = coord_to_shell_vertex.get(key)
            if existing is not None and existing != vertex:
                raise ValueError(f"Duplicate shell interface vertex coordinate at {key}")
            coord_to_shell_vertex[key] = vertex

    shell_vertices_sorted = sorted(shell_interface_vertices)
    shell_interface_points = [shell.points[vertex] for vertex in shell_vertices_sorted]
    outer_to_merged: dict[int, int] = {}
    unmatched_vertices: list[dict[str, object]] = []
    reused_shell_vertices: dict[int, int] = {}
    max_match_distance = 0.0
    for face_id in outer_interface_face_ids:
        for vertex in outer.faces[face_id]:
            if vertex in outer_to_merged:
                continue
            key = coord_key(outer.points[vertex], coord_digits)
            mapped = coord_to_shell_vertex.get(key)
            distance = 0.0
            if mapped is None:
                mapped, distance = nearest_vertex(
                    outer.points[vertex],
                    shell_vertices_sorted,
                    shell_interface_points,
                    tolerance=match_tolerance,
                )
            if mapped is None:
                unmatched_vertices.append({"outer_vertex": vertex, "coordinate": outer.points[vertex]})
            else:
                existing_outer = reused_shell_vertices.get(mapped)
                if existing_outer is not None and existing_outer != vertex:
                    raise ValueError(
                        f"Outer interface vertices {existing_outer} and {vertex} both map to shell vertex {mapped}"
                    )
                reused_shell_vertices[mapped] = vertex
                outer_to_merged[vertex] = mapped
                max_match_distance = max(max_match_distance, distance)
    if unmatched_vertices:
        raise ValueError(f"{len(unmatched_vertices)} outer interface vertices did not map to shell interface")

    merged_points = list(shell.points)
    for index, point in enumerate(outer.points):
        if index not in outer_to_merged:
            outer_to_merged[index] = len(merged_points)
            merged_points.append(point)

    shell_boundary_ids = set()
    for patch in shell.patches:
        shell_boundary_ids.update(patch_face_ids(patch))
    outer_boundary_ids = set()
    for patch in outer.patches:
        outer_boundary_ids.update(patch_face_ids(patch))

    merged_faces: list[list[int]] = []
    merged_owner: list[int] = []
    merged_neighbour: list[int] = []
    outer_cell_offset = shell.cell_count

    # Existing shell internal faces.
    shell_internal_count = min(patch.start_face for patch in shell.patches) if shell.patches else len(shell.faces)
    for face_id in range(shell_internal_count):
        merged_faces.append(shell.faces[face_id])
        merged_owner.append(shell.owner[face_id])
        merged_neighbour.append(shell.neighbour[face_id])

    # Existing outer internal faces.
    outer_internal_count = min(patch.start_face for patch in outer.patches) if outer.patches else len(outer.faces)
    for face_id in range(outer_internal_count):
        merged_faces.append([outer_to_merged[vertex] for vertex in outer.faces[face_id]])
        merged_owner.append(outer.owner[face_id] + outer_cell_offset)
        merged_neighbour.append(outer.neighbour[face_id] + outer_cell_offset)

    # Shared shell/outer interface becomes internal. Use the shell face orientation
    # and pair by exact sorted mapped vertex set.
    outer_interface_by_key: dict[tuple[int, ...], int] = {}
    for face_id in outer_interface_face_ids:
        mapped_face = [outer_to_merged[vertex] for vertex in outer.faces[face_id]]
        key = tuple(sorted(mapped_face))
        if key in outer_interface_by_key:
            raise ValueError(f"Duplicate outer interface face key: {key}")
        outer_interface_by_key[key] = face_id

    missing_outer_faces = []
    for shell_face_id in shell_interface_face_ids:
        shell_face = shell.faces[shell_face_id]
        key = tuple(sorted(shell_face))
        outer_face_id = outer_interface_by_key.get(key)
        if outer_face_id is None:
            missing_outer_faces.append(shell_face_id)
            continue
        merged_faces.append(shell_face)
        merged_owner.append(shell.owner[shell_face_id])
        merged_neighbour.append(outer.owner[outer_face_id] + outer_cell_offset)
    if missing_outer_faces:
        raise ValueError(f"{len(missing_outer_faces)} shell interface faces did not map to outer interface")

    used_outer_interface = {
        tuple(sorted(outer_to_merged[vertex] for vertex in outer.faces[face_id]))
        for face_id in outer_interface_face_ids
    }
    if len(used_outer_interface) != len(shell_interface_face_ids):
        raise ValueError("Outer interface face map did not cover the shell interface exactly")

    boundary_start = len(merged_faces)
    patches: list[BoundaryPatch] = []

    aircraft_start = len(merged_faces)
    for face_id in patch_face_ids(shell_aircraft):
        merged_faces.append(shell.faces[face_id])
        merged_owner.append(shell.owner[face_id])
    patches.append(
        BoundaryPatch(
            name=aircraft_patch,
            type=shell_aircraft.type or "wall",
            n_faces=len(merged_faces) - aircraft_start,
            start_face=aircraft_start,
            extra={},
        )
    )

    for outer_patch in outer_boundary_patch_records:
        patch_start = len(merged_faces)
        for face_id in patch_face_ids(outer_patch):
            merged_faces.append([outer_to_merged[vertex] for vertex in outer.faces[face_id]])
            merged_owner.append(outer.owner[face_id] + outer_cell_offset)
        patches.append(
            BoundaryPatch(
                name=outer_patch.name,
                type=outer_patch.type or "patch",
                n_faces=len(merged_faces) - patch_start,
                start_face=patch_start,
                extra={"physicalType": "patch"} if outer_patch.extra.get("physicalType") else {},
            )
        )

    result = PolyMesh(
        points=merged_points,
        faces=merged_faces,
        owner=merged_owner,
        neighbour=merged_neighbour,
        patches=patches,
        cell_count=shell.cell_count + outer.cell_count,
    )
    report = {
        "status": "ready",
        "coord_digits": coord_digits,
        "match_tolerance": match_tolerance,
        "max_interface_vertex_match_distance": max_match_distance,
        "points": len(result.points),
        "faces": len(result.faces),
        "internal_faces": len(result.neighbour),
        "boundary_faces": len(result.faces) - len(result.neighbour),
        "cells": result.cell_count,
        "shell_cells": shell.cell_count,
        "outer_cells": outer.cell_count,
        "interface_faces": len(shell_interface_face_ids),
        "interface_vertices": len(coord_to_shell_vertex),
        "outer_boundary_patches": outer_boundary_names,
        "outer_vertices_reused_on_interface": len({v for v in outer_to_merged if v < len(outer.points) and outer_to_merged[v] < len(shell.points)}),
        "boundary_start": boundary_start,
        "patches": [
            {"name": patch.name, "type": patch.type, "nFaces": patch.n_faces, "startFace": patch.start_face}
            for patch in patches
        ],
    }
    return result, report


def read_polymesh(poly_dir: Path) -> PolyMesh:
    points = parse_points((poly_dir / "points").read_text(encoding="utf-8", errors="replace"))
    faces = parse_faces((poly_dir / "faces").read_text(encoding="utf-8", errors="replace"))
    owner = parse_labels((poly_dir / "owner").read_text(encoding="utf-8", errors="replace"))
    neighbour_path = poly_dir / "neighbour"
    neighbour = parse_labels(neighbour_path.read_text(encoding="utf-8", errors="replace")) if neighbour_path.exists() else []
    patches = parse_boundary((poly_dir / "boundary").read_text(encoding="utf-8", errors="replace"))
    cell_count = max(owner + neighbour) + 1 if owner or neighbour else 0
    return PolyMesh(points=points, faces=faces, owner=owner, neighbour=neighbour, patches=patches, cell_count=cell_count)


def write_polymesh(poly_dir: Path, mesh: PolyMesh) -> None:
    write_points(poly_dir / "points", mesh.points)
    write_faces(poly_dir / "faces", mesh.faces)
    write_labels(poly_dir / "owner", mesh.owner, "owner")
    write_labels(poly_dir / "neighbour", mesh.neighbour, "neighbour")
    write_boundary(poly_dir / "boundary", mesh.patches)


def parse_points(text: str) -> list[tuple[float, float, float]]:
    body = list_body(text)
    points = []
    for match in re.finditer(r"\(([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\)", body):
        points.append((float(match.group(1)), float(match.group(2)), float(match.group(3))))
    return points


def parse_faces(text: str) -> list[list[int]]:
    body = list_body(text)
    faces = []
    for match in re.finditer(r"(\d+)\(([^()]*)\)", body):
        count = int(match.group(1))
        values = [int(part) for part in match.group(2).split()]
        if len(values) != count:
            raise ValueError("Face vertex count does not match")
        faces.append(values)
    return faces


def parse_labels(text: str) -> list[int]:
    body = list_body(text)
    return [int(match.group(0)) for match in re.finditer(r"-?\d+", body)]


def parse_boundary(text: str) -> list[BoundaryPatch]:
    patches: list[BoundaryPatch] = []
    for match in re.finditer(r"\n\s*([A-Za-z0-9_]+)\s*\n\s*\{([^}]*)\}", text):
        name = match.group(1)
        if name == "FoamFile":
            continue
        body = match.group(2)
        type_match = re.search(r"\btype\s+([^;]+);", body)
        n_faces = re.search(r"\bnFaces\s+(\d+);", body)
        start_face = re.search(r"\bstartFace\s+(\d+);", body)
        extra = {}
        physical = re.search(r"\bphysicalType\s+([^;]+);", body)
        if physical:
            extra["physicalType"] = physical.group(1).strip()
        if not n_faces or not start_face:
            raise ValueError(f"Patch {name} missing nFaces/startFace")
        patches.append(
            BoundaryPatch(
                name=name,
                type=type_match.group(1).strip() if type_match else "patch",
                n_faces=int(n_faces.group(1)),
                start_face=int(start_face.group(1)),
                extra=extra,
            )
        )
    return patches


def list_body(text: str) -> str:
    count_match = re.search(r"\n\s*(\d+)\s*\n\s*\(", text)
    if not count_match:
        raise ValueError("Could not find OpenFOAM list body")
    start = count_match.end()
    end = text.rfind(")")
    if end <= start:
        raise ValueError("Could not find OpenFOAM list end")
    return text[start:end]


def write_points(path: Path, points: list[tuple[float, float, float]]) -> None:
    lines = [foam_header("vectorField", "points"), str(len(points)), "("]
    lines.extend(f"({x:.17g} {y:.17g} {z:.17g})" for x, y, z in points)
    lines.extend([")", ""])
    path.write_text("\n".join(lines), encoding="ascii")


def write_faces(path: Path, faces: list[list[int]]) -> None:
    lines = [foam_header("faceList", "faces"), str(len(faces)), "("]
    lines.extend(f"{len(face)}({' '.join(str(vertex) for vertex in face)})" for face in faces)
    lines.extend([")", ""])
    path.write_text("\n".join(lines), encoding="ascii")


def write_labels(path: Path, labels: list[int], name: str) -> None:
    lines = [foam_header("labelList", name), str(len(labels)), "("]
    lines.extend(str(label) for label in labels)
    lines.extend([")", ""])
    path.write_text("\n".join(lines), encoding="ascii")


def write_boundary(path: Path, patches: list[BoundaryPatch]) -> None:
    lines = [foam_header("polyBoundaryMesh", "boundary"), str(len(patches)), "("]
    for patch in patches:
        lines.extend(
            [
                f"    {patch.name}",
                "    {",
                f"        type            {patch.type};",
            ]
        )
        for key, value in patch.extra.items():
            lines.append(f"        {key}    {value};")
        lines.extend(
            [
                f"        nFaces          {patch.n_faces};",
                f"        startFace       {patch.start_face};",
                "    }",
            ]
        )
    lines.extend([")", ""])
    path.write_text("\n".join(lines), encoding="ascii")


def foam_header(class_name: str, object_name: str) -> str:
    return f"""/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM
   \\\\    /   O peration     |
    \\\\  /    A nd           |
     \\\\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       {class_name};
    object      {object_name};
}}
"""


def require_patch(mesh: PolyMesh, name: str) -> BoundaryPatch:
    for patch in mesh.patches:
        if patch.name == name:
            return patch
    raise ValueError(f"Missing patch {name}")


def select_outer_boundary_names(
    outer: PolyMesh,
    *,
    outer_interface_patch: str,
    farfield_patch: str,
    outer_boundary_patches: str,
) -> list[str]:
    if outer_boundary_patches.strip().lower() == "auto":
        names = [patch.name for patch in outer.patches if patch.name != outer_interface_patch]
    elif outer_boundary_patches.strip():
        names = [part.strip() for part in outer_boundary_patches.split(",") if part.strip()]
    else:
        names = [farfield_patch]
    if not names:
        raise ValueError("No outer boundary patches selected")
    missing = [name for name in names if not any(patch.name == name for patch in outer.patches)]
    if missing:
        raise ValueError(f"Missing selected outer boundary patch(es): {missing}")
    if outer_interface_patch in names:
        raise ValueError("Outer interface patch cannot also be preserved as an outer boundary")
    return names


def patch_face_ids(patch: BoundaryPatch) -> range:
    return range(patch.start_face, patch.start_face + patch.n_faces)


def coord_key(point: tuple[float, float, float], digits: int) -> tuple[float, float, float]:
    return tuple(round(value, digits) for value in point)


def nearest_vertex(
    point: tuple[float, float, float],
    vertex_ids: list[int],
    points: list[tuple[float, float, float]],
    *,
    tolerance: float,
) -> tuple[int | None, float]:
    # Prefer a KD-tree when scipy is available. Fall back to a linear search so
    # the script remains usable in minimal Python environments.
    global _KD_CACHE
    try:
        cache = _KD_CACHE
    except NameError:
        cache = _KD_CACHE = {}

    key = id(points)
    if key not in cache:
        try:
            import numpy as np
            from scipy.spatial import cKDTree

            cache[key] = ("scipy", cKDTree(np.asarray(points, dtype=float)))
        except Exception:
            cache[key] = ("brute", None)

    mode, tree = cache[key]
    if mode == "scipy":
        distance, index = tree.query(point, k=1)
        distance = float(distance)
        if distance <= tolerance:
            return vertex_ids[int(index)], distance
        return None, distance

    best_index = None
    best_distance = float("inf")
    for index, candidate in enumerate(points):
        candidate_distance = dist(point, candidate)
        if candidate_distance < best_distance:
            best_index = index
            best_distance = candidate_distance
    if best_index is not None and best_distance <= tolerance:
        return vertex_ids[best_index], best_distance
    return None, best_distance


if __name__ == "__main__":
    main()
