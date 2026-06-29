from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def main() -> None:
    parser = argparse.ArgumentParser(description="Write a two-region ASCII STL separating centerline cap faces.")
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    parser.add_argument("--report-json", type=Path, required=True)
    parser.add_argument("--axis", choices=sorted(AXIS_INDEX), default="y")
    parser.add_argument("--plane", type=float, default=0.0)
    parser.add_argument("--plane-tolerance", type=float, default=0.004)
    parser.add_argument("--aircraft-region", default="aircraft")
    parser.add_argument("--cap-region", default="centerline_cap")
    args = parser.parse_args()

    mesh = trimesh.load(args.input_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected a single mesh: {args.input_stl}")
    mesh = mesh.copy()
    mesh.merge_vertices(digits_vertex=9)
    axis = AXIS_INDEX[args.axis]
    face_vertices = mesh.vertices[mesh.faces]
    cap_mask = np.max(np.abs(face_vertices[:, :, axis] - args.plane), axis=1) <= args.plane_tolerance

    args.output_stl.parent.mkdir(parents=True, exist_ok=True)
    write_region_ascii_stl(
        args.output_stl,
        mesh.vertices,
        mesh.faces,
        cap_mask,
        aircraft_region=args.aircraft_region,
        cap_region=args.cap_region,
    )
    report = {
        "input_stl": str(args.input_stl),
        "output_stl": str(args.output_stl),
        "axis": args.axis,
        "plane": args.plane,
        "plane_tolerance": args.plane_tolerance,
        "aircraft_region": args.aircraft_region,
        "cap_region": args.cap_region,
        "face_count": int(len(mesh.faces)),
        "aircraft_face_count": int(np.count_nonzero(~cap_mask)),
        "cap_face_count": int(np.count_nonzero(cap_mask)),
        "watertight": bool(mesh.is_watertight),
        "bounds": mesh.bounds.tolist(),
    }
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def write_region_ascii_stl(
    path: Path,
    vertices: np.ndarray,
    faces: np.ndarray,
    cap_mask: np.ndarray,
    *,
    aircraft_region: str,
    cap_region: str,
) -> None:
    with path.open("w", encoding="ascii", newline="\n") as handle:
        write_solid(handle, aircraft_region, vertices, faces[~cap_mask])
        write_solid(handle, cap_region, vertices, faces[cap_mask])


def write_solid(handle, name: str, vertices: np.ndarray, faces: np.ndarray) -> None:
    handle.write(f"solid {name}\n")
    for face in faces:
        tri = vertices[face]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = float(np.linalg.norm(normal))
        if norm > 0.0:
            normal = normal / norm
        else:
            normal = np.zeros(3)
        handle.write(f"  facet normal {normal[0]:.9e} {normal[1]:.9e} {normal[2]:.9e}\n")
        handle.write("    outer loop\n")
        for vertex in tri:
            handle.write(f"      vertex {vertex[0]:.9e} {vertex[1]:.9e} {vertex[2]:.9e}\n")
        handle.write("    endloop\n")
        handle.write("  endfacet\n")
    handle.write(f"endsolid {name}\n")


if __name__ == "__main__":
    main()
