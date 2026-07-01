import json
import math
import pathlib
import re
import subprocess
import sys
from collections import defaultdict

import numpy as np
import trimesh


ROOT = pathlib.Path(__file__).resolve().parents[1]
EXE = ROOT / "target" / "release" / "implicit-cad.exe"
OUTPUT_MD = ROOT / "output.md"

COMPONENT_SCRIPT = ROOT / "tmp_topology_debug_components.rhai"
CONTROL_SCRIPT = ROOT / "tmp_topology_debug_controls.rhai"

STAGE_RE = re.compile(
    r"aero-cleanup stage=(?P<stage>\S+) triangles=(?P<tri>\d+) vertices=(?P<vert>\d+) "
    r"boundary_edges=(?P<boundary>\d+) non_manifold_edges=(?P<nonmanifold>\d+) "
    r"connected_components=(?P<components>\d+) degenerate_removed=(?P<deg>\d+) "
    r"sliver_removed=(?P<sliver>\d+) vertices_welded=(?P<welded>\d+) "
    r"tiny_component_deleted=(?P<tiny>\S+) watertight=(?P<watertight>\S+)"
)
MC_STAGE_RE = re.compile(
    r"adaptive-mc boundary_checkpoint: stage=(?P<stage>\S+) boundary_edges=(?P<boundary>\d+) "
    r"triangles=(?P<tri>\d+) vertices=(?P<vert>\d+)"
)
MC_WELD_STAGE_RE = re.compile(
    r"adaptive-mc boundary_checkpoint: stage=(?P<stage>\S+) boundary_edges=(?P<boundary>\d+) "
    r"triangles=(?P<tri>\d+) vertices=(?P<vert>\d+) weld_epsilon=(?P<eps>[0-9.]+) "
    r"vertices_merged=(?P<welded>\d+) new_boundary_edges_introduced=(?P<new_boundary>\d+)"
)
DEGENERATE_RE = re.compile(
    r"adaptive-mc degenerate_removal_diagnostic: .*triangles_before=(?P<before>\d+) "
    r"triangles_removed=(?P<removed>\d+)"
)
PATCH_START_RE = re.compile(r"\[aero\] meshing patch '(?P<patch>[^']+)'")


def run_export(script: pathlib.Path, out_dir: pathlib.Path, cell_size: float, max_voxels: int | None):
    if out_dir.exists():
        for child in out_dir.iterdir():
            if child.is_file():
                child.unlink()
            else:
                import shutil
                shutil.rmtree(child)
    cmd = [
        str(EXE),
        "--headless",
        "--script",
        str(script),
        "--format",
        "aero",
        "--output",
        str(out_dir),
        "--resolution",
        "32",
        "--aero-mode",
        "external",
        "--aero-uniform-reference",
        "--aero-min-cell-mm",
        str(cell_size),
        "--aero-timeout-seconds",
        "1800",
    ]
    if max_voxels is not None:
        cmd.extend(["--aero-max-patch-voxels", str(max_voxels)])
    proc = subprocess.run(
        cmd,
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    return proc


def parse_stage_logs(text: str):
    per_patch: dict[str, dict[str, dict]] = defaultdict(dict)
    current_patch = None
    pending_deg_removed = None
    for line in text.splitlines():
        patch_match = PATCH_START_RE.search(line)
        if patch_match:
            current_patch = patch_match.group("patch")
            continue
        if current_patch is None:
            continue

        m = DEGENERATE_RE.search(line)
        if m:
            pending_deg_removed = int(m.group("removed"))
            continue

        m = MC_WELD_STAGE_RE.search(line)
        if m:
            per_patch[current_patch][m.group("stage")] = {
                "triangles": int(m.group("tri")),
                "vertices": int(m.group("vert")),
                "boundary_edges": int(m.group("boundary")),
                "vertices_welded": int(m.group("welded")),
                "new_boundary_edges_introduced": int(m.group("new_boundary")),
            }
            continue

        m = MC_STAGE_RE.search(line)
        if m:
            entry = {
                "triangles": int(m.group("tri")),
                "vertices": int(m.group("vert")),
                "boundary_edges": int(m.group("boundary")),
            }
            if m.group("stage") == "post_degenerate" and pending_deg_removed is not None:
                entry["degenerate_removed"] = pending_deg_removed
            per_patch[current_patch][m.group("stage")] = entry
            continue

        m = STAGE_RE.search(line)
        if m:
            per_patch[current_patch][m.group("stage")] = {
                "triangles": int(m.group("tri")),
                "vertices": int(m.group("vert")),
                "boundary_edges": int(m.group("boundary")),
                "non_manifold_edges": int(m.group("nonmanifold")),
                "connected_components": int(m.group("components")),
                "degenerate_removed": int(m.group("deg")),
                "sliver_removed": int(m.group("sliver")),
                "vertices_welded": int(m.group("welded")),
                "tiny_component_deleted": m.group("tiny") == "true",
                "watertight": m.group("watertight") == "true",
            }
    return per_patch


def boundary_and_non_manifold_edges(mesh: trimesh.Trimesh):
    unique, counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
    boundary_edges = int((counts == 1).sum())
    non_manifold_edges = int((counts != 2).sum())
    return boundary_edges, non_manifold_edges, unique, counts


def connected_component_count(mesh: trimesh.Trimesh):
    cc = trimesh.graph.connected_components(mesh.face_adjacency)
    if hasattr(cc, "shape"):
        return int(cc.shape[0])
    return int(len(cc))


def edge_lengths(mesh: trimesh.Trimesh):
    lengths = mesh.edges_unique_length
    if len(lengths) == 0:
        return 0.0, 0.0, 0.0
    return float(lengths.mean()), float(lengths.min()), float(lengths.max())


def mesh_stats(path: pathlib.Path):
    mesh = trimesh.load(path, force="mesh")
    boundary_edges, non_manifold_edges, unique_edges, counts = boundary_and_non_manifold_edges(mesh)
    edge_mean, edge_min, edge_max = edge_lengths(mesh)
    bounds = mesh.bounds if len(mesh.vertices) else np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    return {
        "mesh": mesh,
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
        "unique_edges": unique_edges,
        "edge_counts": counts,
    }


def cluster_boundary_edges(mesh: trimesh.Trimesh, bounds_min, bounds_max):
    _, _, unique_edges, counts = boundary_and_non_manifold_edges(mesh)
    boundary_mask = counts == 1
    if not np.any(boundary_mask):
        return []
    boundary_edges = unique_edges[np.where(boundary_mask)[0]]
    verts = mesh.vertices[boundary_edges.flatten()].reshape(-1, 2, 3)
    centers = verts.mean(axis=1)
    extent = bounds_max - bounds_min + 1e-9
    grid = np.floor((centers - bounds_min) / extent * 10).astype(int).clip(0, 9)
    bins: dict[tuple[int, int, int], list[np.ndarray]] = defaultdict(list)
    for cell, center in zip(grid, centers):
        bins[tuple(int(v) for v in cell)].append(center)
    clusters = []
    for idx, (cell, pts) in enumerate(sorted(bins.items(), key=lambda item: len(item[1]), reverse=True), start=1):
        pts_arr = np.array(pts)
        clusters.append({
            "cluster_id": idx,
            "count": len(pts),
            "cell": cell,
            "bbox_min": pts_arr.min(axis=0).tolist(),
            "bbox_max": pts_arr.max(axis=0).tolist(),
        })
    return clusters


def classify_feature(part_name: str, cluster, bounds_min, bounds_max):
    cmin = np.array(cluster["bbox_min"])
    cmax = np.array(cluster["bbox_max"])
    center = 0.5 * (cmin + cmax)
    extent = bounds_max - bounds_min
    tol = np.maximum(extent * 0.08, 2.0)
    near_bounds = (
        np.any(np.abs(cmin - bounds_min) <= tol) or np.any(np.abs(bounds_max - cmax) <= tol)
    )
    if near_bounds:
        return "export bounds edge"
    name = part_name.lower()
    if "wing" in name:
        if center[0] > bounds_min[0] + 0.8 * extent[0]:
            return "trailing edge"
        if center[0] < bounds_min[0] + 0.15 * extent[0]:
            return "leading edge"
        if abs(center[1]) < 0.15 * extent[1]:
            return "root"
        if abs(center[1]) > 0.42 * extent[1]:
            return "tip"
    if "vtail" in name:
        if center[2] > bounds_min[2] + 0.8 * extent[2]:
            return "tip"
        if center[0] > bounds_min[0] + 0.7 * extent[0]:
            return "trailing edge"
        if center[0] < bounds_min[0] + 0.2 * extent[0]:
            return "leading edge"
        if abs(center[1]) < 0.2 * extent[1]:
            return "blend/junction"
    if "fuselage" in name:
        if center[0] < bounds_min[0] + 0.15 * extent[0]:
            return "root"
        if center[0] > bounds_min[0] + 0.85 * extent[0]:
            return "tip"
        if center[2] > bounds_min[2] + 0.7 * extent[2]:
            return "blend/junction"
    if "inlet" in name:
        if center[0] < bounds_min[0] + 0.2 * extent[0]:
            return "leading edge"
        if center[0] > bounds_min[0] + 0.8 * extent[0]:
            return "trailing edge"
    return "unknown"


def component_diagnostics(mesh: trimesh.Trimesh):
    face_components = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(mesh.faces)))
    if hasattr(face_components, "tolist"):
        face_components = face_components.tolist()
    result = []
    for idx, faces in enumerate(face_components, start=1):
        sub = mesh.submesh([list(faces)], append=True, repair=False)
        stats = mesh_stats_from_mesh(sub)
        stats["component_id"] = idx
        result.append(stats)
    result.sort(key=lambda row: row["triangle_count"], reverse=True)
    return result


def mesh_stats_from_mesh(mesh: trimesh.Trimesh):
    boundary_edges, non_manifold_edges, _, _ = boundary_and_non_manifold_edges(mesh)
    edge_mean, edge_min, edge_max = edge_lengths(mesh)
    bounds = mesh.bounds if len(mesh.vertices) else np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    return {
        "triangle_count": int(len(mesh.faces)),
        "vertex_count": int(len(mesh.vertices)),
        "boundary_edges": boundary_edges,
        "non_manifold_edges": non_manifold_edges,
        "connected_components": connected_component_count(mesh),
        "watertight": bool(mesh.is_watertight),
        "bounds_min": [float(x) for x in bounds[0]],
        "bounds_max": [float(x) for x in bounds[1]],
        "average_edge_length": edge_mean,
        "minimum_edge_length": edge_min,
        "maximum_edge_length": edge_max,
    }


def manifest_for(out_dir: pathlib.Path):
    return json.loads((out_dir / "manifest.json").read_text())


def main():
    OUTPUT_MD.write_text("# Topology Debug Output\n\n")
    OUTPUT_MD.write_text("Initial run pending.\n", append=False) if False else None

    cell_plan = [
        ("components", COMPONENT_SCRIPT, [3.0, 2.0, 1.5, 1.0], {1.5: 200_000_000, 1.0: 600_000_000}),
        ("controls", CONTROL_SCRIPT, [3.0, 2.0, 1.5, 1.0], {}),
    ]

    lines = ["# Topology Debug Output", ""]
    for label, script, cells, voxel_overrides in cell_plan:
        lines.append(f"## Run Group: {label}")
        for cell in cells:
            out_dir = ROOT / f"tmp_topology_{label}_{str(cell).replace('.', 'p')}"
            proc = run_export(script, out_dir, cell, voxel_overrides.get(cell))
            lines.append(f"### Cell {cell:.1f} mm")
            lines.append("")
            lines.append(f"- exit_code: `{proc.returncode}`")
            if proc.returncode != 0:
                lines.append("- status: failed")
            else:
                lines.append("- status: completed")
            lines.append("")
            lines.append("```text")
            lines.append(proc.stdout.strip())
            lines.append(proc.stderr.strip())
            lines.append("```")
            lines.append("")
            if proc.returncode != 0 or not (out_dir / "manifest.json").exists():
                continue
            manifest = manifest_for(out_dir)
            stage_data = parse_stage_logs(proc.stdout + "\n" + proc.stderr)
            lines.append("#### Patches")
            for patch in manifest["patches"]:
                patch_name = patch["name"]
                stl_path = out_dir / patch["file"]
                stats = mesh_stats(stl_path)
                bounds_min = np.array(stats["bounds_min"])
                bounds_max = np.array(stats["bounds_max"])
                lines.append(f"- `{patch_name}` final: tris={stats['triangle_count']} verts={stats['vertex_count']} boundary={stats['boundary_edges']} non_manifold={stats['non_manifold_edges']} components={stats['connected_components']} watertight={stats['watertight']}")
                if patch_name in stage_data:
                    lines.append(f"  stages: `{json.dumps(stage_data[patch_name], sort_keys=True)}`")
                clusters = cluster_boundary_edges(stats["mesh"], bounds_min, bounds_max)[:5]
                if clusters:
                    for cluster in clusters:
                        feature = classify_feature(patch_name, cluster, bounds_min, bounds_max)
                        lines.append(
                            f"  boundary_cluster_{cluster['cluster_id']}: count={cluster['count']} bbox={cluster['bbox_min']}..{cluster['bbox_max']} feature={feature}"
                        )
                comp_stats = component_diagnostics(stats["mesh"])[:5]
                for comp in comp_stats:
                    lines.append(
                        f"  connected_component_{comp['component_id']}: tris={comp['triangle_count']} verts={comp['vertex_count']} boundary={comp['boundary_edges']} non_manifold={comp['non_manifold_edges']} watertight={comp['watertight']} bbox={comp['bounds_min']}..{comp['bounds_max']}"
                    )
            lines.append("")
    OUTPUT_MD.write_text("\n".join(lines))
    print(f"Wrote {OUTPUT_MD}")


if __name__ == "__main__":
    main()
