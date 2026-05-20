import trimesh
import numpy as np
import sys, json, datetime, pathlib

stl_path = sys.argv[1]
log_path = pathlib.Path("mesh_quality_log.jsonl")

m = trimesh.load(stl_path, force='mesh')

unique, counts = np.unique(m.edges_sorted, axis=0, return_counts=True)
non_manifold_edges = int((counts != 2).sum())
boundary_edges = int((counts == 1).sum())

record = {
    "timestamp": datetime.datetime.utcnow().isoformat(),
    "file": stl_path,
    "non_manifold_edges": non_manifold_edges,
    "boundary_edges": boundary_edges,
    "non_manifold_vertices": int(len(m.vertices) - len(m.as_open3d.vertices)) if hasattr(m, 'as_open3d') else "n/a",
    "is_watertight": bool(m.is_watertight),
    "is_winding_consistent": bool(m.is_winding_consistent),
    "connected_components": int(trimesh.graph.connected_components(m.face_adjacency).shape[0] if hasattr(trimesh.graph.connected_components(m.face_adjacency), 'shape') else len(trimesh.graph.connected_components(m.face_adjacency))),
    "n_triangles": len(m.faces),
    "n_vertices": len(m.vertices),
}

with log_path.open("a") as f:
    f.write(json.dumps(record) + "\n")

for k, v in record.items():
    print(f"{k}: {v}")

edges_sorted = m.edges_sorted
unique, counts = np.unique(edges_sorted, axis=0, return_counts=True)
nm_edge_indices = np.where(counts != 2)[0]
nm_edges = unique[nm_edge_indices]
nm_verts = m.vertices[nm_edges.flatten()].reshape(-1, 2, 3)
nm_centers = nm_verts.mean(axis=1)

bounds_min = m.vertices.min(axis=0)
bounds_max = m.vertices.max(axis=0)
grid = np.floor(
    (nm_centers - bounds_min) / (bounds_max - bounds_min + 1e-9) * 10
).astype(int).clip(0, 9)
keys = grid[:, 0] * 100 + grid[:, 1] * 10 + grid[:, 2]
unique_keys, bin_counts = np.unique(keys, return_counts=True)
top10 = np.argsort(bin_counts)[-10:][::-1]
print("non_manifold_edge_hotspots:")
for idx in top10:
    k = unique_keys[idx]
    ix, iy, iz = k // 100, (k % 100) // 10, k % 10
    cell_min = bounds_min + (bounds_max - bounds_min) * np.array([ix, iy, iz]) / 10
    cell_max = bounds_min + (bounds_max - bounds_min) * np.array([ix + 1, iy + 1, iz + 1]) / 10
    print(
        f"  [{ix},{iy},{iz}] count={bin_counts[idx]} "
        f"x={cell_min[0]:.1f}..{cell_max[0]:.1f} "
        f"y={cell_min[1]:.1f}..{cell_max[1]:.1f} "
        f"z={cell_min[2]:.1f}..{cell_max[2]:.1f}"
    )
