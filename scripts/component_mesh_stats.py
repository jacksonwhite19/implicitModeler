import json
import pathlib
import sys

import numpy as np
import trimesh


def connected_component_count(mesh: trimesh.Trimesh) -> int:
    cc = trimesh.graph.connected_components(mesh.face_adjacency)
    if hasattr(cc, "shape"):
        return int(cc.shape[0])
    return int(len(cc))


def boundary_and_non_manifold_edges(mesh: trimesh.Trimesh) -> tuple[int, int]:
    unique, counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
    boundary_edges = int((counts == 1).sum())
    non_manifold_edges = int((counts != 2).sum())
    return boundary_edges, non_manifold_edges


def edge_lengths(mesh: trimesh.Trimesh) -> tuple[float, float, float]:
    lengths = mesh.edges_unique_length
    if len(lengths) == 0:
        return 0.0, 0.0, 0.0
    return float(lengths.mean()), float(lengths.min()), float(lengths.max())


def mesh_stats(path: pathlib.Path) -> dict:
    mesh = trimesh.load(path, force="mesh")
    boundary_edges, non_manifold_edges = boundary_and_non_manifold_edges(mesh)
    edge_mean, edge_min, edge_max = edge_lengths(mesh)
    bounds = mesh.bounds if len(mesh.vertices) else np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    return {
        "file": str(path),
        "triangle_count": int(len(mesh.faces)),
        "vertex_count": int(len(mesh.vertices)),
        "boundary_edges": boundary_edges,
        "non_manifold_edges": non_manifold_edges,
        "connected_components": connected_component_count(mesh),
        "watertight": bool(mesh.is_watertight),
        "winding_consistent": bool(mesh.is_winding_consistent),
        "bounds_min": [float(x) for x in bounds[0]],
        "bounds_max": [float(x) for x in bounds[1]],
        "average_edge_length": edge_mean,
        "minimum_edge_length": edge_min,
        "maximum_edge_length": edge_max,
    }


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: python scripts/component_mesh_stats.py <mesh-or-dir> [<mesh-or-dir> ...]")
        return 2

    for arg in sys.argv[1:]:
        path = pathlib.Path(arg)
        if path.is_dir():
            for stl in sorted(path.glob("*.stl")):
                print(json.dumps(mesh_stats(stl)))
        else:
            print(json.dumps(mesh_stats(path)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
