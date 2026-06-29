from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh


DIRS = {
    "xmin": np.array([-1.0, 0.0, 0.0]),
    "xmax": np.array([1.0, 0.0, 0.0]),
    "ymin": np.array([0.0, -1.0, 0.0]),
    "ymax": np.array([0.0, 1.0, 0.0]),
    "zmin": np.array([0.0, 0.0, -1.0]),
    "zmax": np.array([0.0, 0.0, 1.0]),
}


@dataclass(frozen=True)
class FaceRecord:
    vertices: tuple[int, ...]
    owner: int
    neighbour: int | None
    patch: str | None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--cells", default="44,32,44")
    parser.add_argument("--pad-x", type=float, default=0.85)
    parser.add_argument("--pad-y", type=float, default=0.65)
    parser.add_argument("--pad-z", type=float, default=0.65)
    parser.add_argument("--write-openfoam", action="store_true")
    parser.add_argument("--write-su2", action="store_true")
    parser.add_argument("--classifier", choices=["ray-x", "contains"], default="ray-x")
    args = parser.parse_args()

    if not args.write_openfoam and not args.write_su2:
        args.write_openfoam = True

    nx, ny, nz = parse_cells(args.cells)
    mesh = load_surface(args.input_stl, scale=args.scale)
    bounds = padded_bounds(mesh.bounds, args.pad_x, args.pad_y, args.pad_z)
    x = np.linspace(bounds[0, 0], bounds[1, 0], nx + 1)
    y = np.linspace(bounds[0, 1], bounds[1, 1], ny + 1)
    z = np.linspace(bounds[0, 2], bounds[1, 2], nz + 1)

    if args.classifier == "ray-x":
        inside = classify_inside_ray_x(mesh, x, y, z)
    else:
        cell_centers = make_cell_centers(x, y, z)
        inside = mesh.contains(cell_centers).reshape((nx, ny, nz))
    exterior = ~inside
    cell_ids = -np.ones((nx, ny, nz), dtype=np.int64)
    cell_ids[exterior] = np.arange(int(exterior.sum()))

    points = make_grid_points(x, y, z)
    cells = make_cells(nx, ny, nz, cell_ids)
    faces = make_faces(nx, ny, nz, cell_ids)

    args.run_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "input_stl": str(args.input_stl),
        "strategy": "direct_cartesian_cutout_v0",
        "scale": args.scale,
        "cells_requested": [nx, ny, nz],
        "classifier": args.classifier,
        "points": int(len(points)),
        "fluid_cells": int(len(cells)),
        "solid_cutout_cells": int(inside.sum()),
        "faces_total": int(len(faces)),
        "internal_faces": int(sum(1 for face in faces if face.neighbour is not None)),
        "farfield_faces": int(sum(1 for face in faces if face.patch == "farfield")),
        "aircraft_faces": int(sum(1 for face in faces if face.patch == "aircraft")),
        "bounds": bounds.tolist(),
        "surface_source": {
            "vertices": int(len(mesh.vertices)),
            "faces": int(len(mesh.faces)),
            "watertight": bool(mesh.is_watertight),
            "bounds": mesh.bounds.tolist(),
        },
        "known_limitations": [
            "Aircraft boundary is currently a Cartesian cut-out, not body-fitted to the STL triangles.",
            "This validates direct OpenFOAM/SU2 volume output before implementing cut-cell/body-fitted boundary faces.",
        ],
    }

    if args.write_openfoam:
        write_openfoam_poly_mesh(args.run_dir / "openfoam_case" / "constant" / "polyMesh", points, faces)
        write_openfoam_control(args.run_dir / "openfoam_case" / "system")
        report["openfoam_polyMesh"] = str(args.run_dir / "openfoam_case" / "constant" / "polyMesh")
    if args.write_su2:
        write_su2(args.run_dir / "mesh.su2", points, cells, faces)
        report["su2_mesh"] = str(args.run_dir / "mesh.su2")

    (args.run_dir / "direct_mesh_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))


def parse_cells(value: str) -> tuple[int, int, int]:
    parts = [int(part) for part in value.split(",")]
    if len(parts) != 3 or any(part < 2 for part in parts):
        raise argparse.ArgumentTypeError("--cells must be three integers >= 2")
    return parts[0], parts[1], parts[2]


def load_surface(path: Path, *, scale: float) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("Expected a single triangle mesh")
    mesh = mesh.copy()
    mesh.vertices *= scale
    mesh.merge_vertices(digits_vertex=9)
    if not mesh.is_watertight:
        raise ValueError("Input surface must be watertight before direct volume meshing")
    return mesh


def padded_bounds(bounds: np.ndarray, pad_x: float, pad_y: float, pad_z: float) -> np.ndarray:
    pad = np.array([pad_x, pad_y, pad_z])
    result = bounds.copy()
    result[0] -= pad
    result[1] += pad
    return result


def make_cell_centers(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    xc = 0.5 * (x[:-1] + x[1:])
    yc = 0.5 * (y[:-1] + y[1:])
    zc = 0.5 * (z[:-1] + z[1:])
    xx, yy, zz = np.meshgrid(xc, yc, zc, indexing="ij")
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])


def classify_inside_ray_x(mesh: trimesh.Trimesh, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    xc = 0.5 * (x[:-1] + x[1:])
    yc = 0.5 * (y[:-1] + y[1:])
    zc = 0.5 * (z[:-1] + z[1:])
    origins = []
    ray_keys = []
    x0 = float(x[0] - 0.1 * (x[-1] - x[0]) - 1e-6)
    for j, yv in enumerate(yc):
        for k, zv in enumerate(zc):
            origins.append([x0, float(yv), float(zv)])
            ray_keys.append((j, k))
    origins_arr = np.asarray(origins, dtype=float)
    directions = np.tile(np.array([[1.0, 0.0, 0.0]], dtype=float), (len(origins_arr), 1))

    locations, ray_indices, _ = mesh.ray.intersects_location(
        ray_origins=origins_arr,
        ray_directions=directions,
        multiple_hits=True,
    )
    hits_by_ray: list[list[float]] = [[] for _ in range(len(origins_arr))]
    for location, ray_index in zip(locations, ray_indices):
        hits_by_ray[int(ray_index)].append(float(location[0]))

    inside = np.zeros((len(xc), len(yc), len(zc)), dtype=bool)
    for ray_index, hits in enumerate(hits_by_ray):
        if not hits:
            continue
        j, k = ray_keys[ray_index]
        clustered = cluster_hits(sorted(hits), tol=max(1e-7, 1e-5 * float(np.min(np.diff(x)))))
        if len(clustered) < 2:
            continue
        for start, end in zip(clustered[0::2], clustered[1::2]):
            inside[:, j, k] |= (xc > start) & (xc < end)
    return inside


def cluster_hits(values: list[float], *, tol: float) -> list[float]:
    clusters: list[list[float]] = []
    for value in values:
        if not clusters or abs(value - clusters[-1][-1]) > tol:
            clusters.append([value])
        else:
            clusters[-1].append(value)
    return [float(np.mean(cluster)) for cluster in clusters]


def point_id(i: int, j: int, k: int, ny: int, nz: int) -> int:
    return (i * (ny + 1) + j) * (nz + 1) + k


def make_grid_points(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    xx, yy, zz = np.meshgrid(x, y, z, indexing="ij")
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])


def cell_vertices(i: int, j: int, k: int, ny: int, nz: int) -> tuple[int, ...]:
    p000 = point_id(i, j, k, ny, nz)
    p100 = point_id(i + 1, j, k, ny, nz)
    p110 = point_id(i + 1, j + 1, k, ny, nz)
    p010 = point_id(i, j + 1, k, ny, nz)
    p001 = point_id(i, j, k + 1, ny, nz)
    p101 = point_id(i + 1, j, k + 1, ny, nz)
    p111 = point_id(i + 1, j + 1, k + 1, ny, nz)
    p011 = point_id(i, j + 1, k + 1, ny, nz)
    return (p000, p100, p110, p010, p001, p101, p111, p011)


def make_cells(nx: int, ny: int, nz: int, cell_ids: np.ndarray) -> list[tuple[int, ...]]:
    cells: list[tuple[int, ...]] = []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if cell_ids[i, j, k] >= 0:
                    cells.append(cell_vertices(i, j, k, ny, nz))
    return cells


def make_faces(nx: int, ny: int, nz: int, cell_ids: np.ndarray) -> list[FaceRecord]:
    internal: list[FaceRecord] = []
    farfield: list[FaceRecord] = []
    aircraft: list[FaceRecord] = []

    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                owner = int(cell_ids[i, j, k])
                if owner < 0:
                    continue
                for name, offset in (
                    ("xmin", (-1, 0, 0)),
                    ("xmax", (1, 0, 0)),
                    ("ymin", (0, -1, 0)),
                    ("ymax", (0, 1, 0)),
                    ("zmin", (0, 0, -1)),
                    ("zmax", (0, 0, 1)),
                ):
                    ni, nj, nk = i + offset[0], j + offset[1], k + offset[2]
                    vertices = oriented_face_vertices(name, i, j, k, ny, nz)
                    if 0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz:
                        neighbour = int(cell_ids[ni, nj, nk])
                        if neighbour >= 0:
                            if owner < neighbour:
                                internal.append(FaceRecord(vertices, owner, neighbour, None))
                            continue
                        aircraft.append(FaceRecord(vertices, owner, None, "aircraft"))
                    else:
                        farfield.append(FaceRecord(vertices, owner, None, "farfield"))

    internal.sort(key=lambda face: (face.owner, -1 if face.neighbour is None else face.neighbour))
    farfield.sort(key=lambda face: face.owner)
    aircraft.sort(key=lambda face: face.owner)
    return internal + farfield + aircraft


def oriented_face_vertices(name: str, i: int, j: int, k: int, ny: int, nz: int) -> tuple[int, ...]:
    p = cell_vertices(i, j, k, ny, nz)
    raw = {
        "xmin": (p[0], p[4], p[7], p[3]),
        "xmax": (p[1], p[2], p[6], p[5]),
        "ymin": (p[0], p[1], p[5], p[4]),
        "ymax": (p[3], p[7], p[6], p[2]),
        "zmin": (p[0], p[3], p[2], p[1]),
        "zmax": (p[4], p[5], p[6], p[7]),
    }[name]
    return raw


def write_openfoam_poly_mesh(poly_dir: Path, points: np.ndarray, faces: list[FaceRecord]) -> None:
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


def write_openfoam_control(system_dir: Path) -> None:
    system_dir.mkdir(parents=True, exist_ok=True)
    control = (
        foam_header("dictionary", "controlDict")
        + """application     simpleFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         1;
deltaT          1;
writeControl    timeStep;
writeInterval   1;
purgeWrite      0;
writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;

// ************************************************************************* //
"""
    )
    write_text(system_dir / "controlDict", control)


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


def foam_vector_list(points: np.ndarray) -> str:
    lines = [str(len(points)), "("]
    lines.extend(f"({x:.12g} {y:.12g} {z:.12g})" for x, y, z in points)
    lines.extend([")", "", "// ************************************************************************* //", ""])
    return "\n".join(lines)


def foam_face_list(faces: Iterable[tuple[int, ...]]) -> str:
    face_list = list(faces)
    lines = [str(len(face_list)), "("]
    lines.extend(f"{len(face)}(" + " ".join(str(v) for v in face) + ")" for face in face_list)
    lines.extend([")", "", "// ************************************************************************* //", ""])
    return "\n".join(lines)


def foam_label_list(values: Iterable[int | None]) -> str:
    value_list = [int(value) for value in values if value is not None]
    lines = [str(len(value_list)), "("]
    lines.extend(str(value) for value in value_list)
    lines.extend([")", "", "// ************************************************************************* //", ""])
    return "\n".join(lines)


def write_su2(path: Path, points: np.ndarray, cells: list[tuple[int, ...]], faces: list[FaceRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    boundary_faces = [face for face in faces if face.patch is not None]
    farfield = [face for face in boundary_faces if face.patch == "farfield"]
    aircraft = [face for face in boundary_faces if face.patch == "aircraft"]

    lines = ["NDIME= 3", f"NPOIN= {len(points)}"]
    lines.extend(f"{x:.12g} {y:.12g} {z:.12g} {idx}" for idx, (x, y, z) in enumerate(points))
    lines.append(f"NELEM= {len(cells)}")
    for idx, cell in enumerate(cells):
        lines.append("12 " + " ".join(str(v) for v in cell) + f" {idx}")
    lines.append("NMARK= 2")
    append_su2_marker(lines, "farfield", farfield)
    append_su2_marker(lines, "aircraft", aircraft)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def append_su2_marker(lines: list[str], name: str, faces: list[FaceRecord]) -> None:
    lines.append(f"MARKER_TAG= {name}")
    lines.append(f"MARKER_ELEMS= {len(faces)}")
    for face in faces:
        if len(face.vertices) != 4:
            raise ValueError("Cartesian SU2 writer currently expects quad boundary faces")
        lines.append("9 " + " ".join(str(v) for v in face.vertices))


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
