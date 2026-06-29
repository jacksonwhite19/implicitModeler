from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh

from cfd_surface_mesher import mesh_report


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a deterministic affine STL geometry variant.")
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--scale", default="1.0,1.0,1.0", help="Scale x,y,z about the selected center.")
    parser.add_argument("--translate-mm", default="0,0,0")
    parser.add_argument("--center", choices=["bounds", "origin"], default="bounds")
    parser.add_argument("--label", default="affine_variant")
    args = parser.parse_args()

    mesh = trimesh.load(args.input_stl, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh: {args.input_stl}")
    mesh.merge_vertices(digits_vertex=8)
    initial = mesh_report(mesh.copy())

    scale = parse_vec(args.scale)
    translate = parse_vec(args.translate_mm)
    center = np.zeros(3, dtype=float)
    if args.center == "bounds":
        center = 0.5 * (mesh.bounds[0] + mesh.bounds[1])

    vertices = np.asarray(mesh.vertices, dtype=float)
    mesh.vertices = (vertices - center) * scale + center + translate
    mesh.remove_unreferenced_vertices()
    final = mesh_report(mesh.copy())

    args.output_stl.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.output_stl)
    report = {
        "label": args.label,
        "input_stl": str(args.input_stl),
        "output_stl": str(args.output_stl),
        "operation": "affine_scale_translate",
        "center_mode": args.center,
        "center_mm": center.tolist(),
        "scale": scale.tolist(),
        "translate_mm": translate.tolist(),
        "initial": initial,
        "final": final,
    }
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def parse_vec(text: str) -> np.ndarray:
    values = np.asarray([float(part) for part in text.split(",")], dtype=float)
    if values.shape != (3,):
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return values


if __name__ == "__main__":
    main()
