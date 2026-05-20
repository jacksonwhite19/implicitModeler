import argparse
import json
import math
import subprocess
import sys
from collections import Counter, defaultdict, deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from plot_sdf_section_csv import load_grid
from plot_stl_section import load_stl_triangles


def q3(v, tol):
    return tuple(int(round(c / tol)) for c in v)


def vec3_from_q(q, tol):
    return tuple(c * tol for c in q)


def bbox_from_points(points):
    arr = np.asarray(points, dtype=float)
    return {
        "xmin": float(arr[:, 0].min()),
        "xmax": float(arr[:, 0].max()),
        "ymin": float(arr[:, 1].min()),
        "ymax": float(arr[:, 1].max()),
        "zmin": float(arr[:, 2].min()),
        "zmax": float(arr[:, 2].max()),
    }


def bbox_from_points_2d(points):
    arr = np.asarray(points, dtype=float)
    return {
        "amin": float(arr[:, 0].min()),
        "amax": float(arr[:, 0].max()),
        "bmin": float(arr[:, 1].min()),
        "bmax": float(arr[:, 1].max()),
    }


def centroid_from_points(points):
    arr = np.asarray(points, dtype=float)
    return {
        "x": float(arr[:, 0].mean()),
        "y": float(arr[:, 1].mean()),
        "z": float(arr[:, 2].mean()),
    }


def analyze_mesh(triangles, tol=1e-4):
    triangle_keys = []
    triangle_points = []
    edge_to_tris = defaultdict(list)
    vertex_to_tris = defaultdict(set)

    for tidx, tri in enumerate(triangles):
        verts = [q3(v, tol) for v in tri]
        triangle_keys.append(verts)
        triangle_points.append([vec3_from_q(v, tol) for v in verts])
        for v in verts:
            vertex_to_tris[v].add(tidx)
        edges = ((verts[0], verts[1]), (verts[1], verts[2]), (verts[2], verts[0]))
        for a, b in edges:
            if a == b:
                continue
            edge_to_tris[tuple(sorted((a, b)))].append(tidx)

    tri_adj = defaultdict(set)
    for tri_ids in edge_to_tris.values():
        if len(tri_ids) < 2:
            continue
        for i in tri_ids:
            tri_adj[i].update(t for t in tri_ids if t != i)

    seen = set()
    components = []
    for tidx in range(len(triangle_keys)):
        if tidx in seen:
            continue
        q = deque([tidx])
        seen.add(tidx)
        tris = []
        verts = set()
        while q:
            cur = q.popleft()
            tris.append(cur)
            verts.update(triangle_keys[cur])
            for nxt in tri_adj[cur]:
                if nxt in seen:
                    continue
                seen.add(nxt)
                q.append(nxt)
        pts = [vec3_from_q(v, tol) for v in verts]
        components.append({
            "triangle_count": len(tris),
            "bbox_mm": bbox_from_points(pts),
            "centroid_mm": centroid_from_points(pts),
        })
    components.sort(key=lambda c: c["triangle_count"], reverse=True)

    boundary_edges = []
    nonmanifold_edges = []
    for edge, tri_ids in edge_to_tris.items():
        if len(tri_ids) == 1:
            boundary_edges.append(edge)
        elif len(tri_ids) > 2:
            nonmanifold_edges.append((edge, len(tri_ids)))

    def edge_midpoint(edge):
        a = np.asarray(vec3_from_q(edge[0], tol))
        b = np.asarray(vec3_from_q(edge[1], tol))
        mid = 0.5 * (a + b)
        return tuple(float(x) for x in mid)

    boundary_graph = defaultdict(set)
    for a, b in boundary_edges:
        boundary_graph[a].add(b)
        boundary_graph[b].add(a)

    visited_edges = set()
    boundary_loops = []
    boundary_chains = []
    for a, nbrs in list(boundary_graph.items()):
        for b in list(nbrs):
            edge_key = tuple(sorted((a, b)))
            if edge_key in visited_edges:
                continue
            path = [a, b]
            visited_edges.add(edge_key)
            prev = a
            cur = b
            closed = False
            while True:
                nexts = [n for n in boundary_graph[cur] if n != prev]
                if not nexts:
                    break
                nxt = nexts[0]
                e = tuple(sorted((cur, nxt)))
                if e in visited_edges:
                    if nxt == path[0]:
                        closed = True
                    break
                visited_edges.add(e)
                path.append(nxt)
                prev, cur = cur, nxt
                if cur == path[0]:
                    closed = True
                    break
            pts = [vec3_from_q(v, tol) for v in path]
            rec = {
                "edge_count": len(path) - 1 if closed else len(path),
                "bbox_mm": bbox_from_points(pts),
                "centroid_mm": centroid_from_points(pts),
            }
            if closed:
                boundary_loops.append(rec)
            else:
                boundary_chains.append(rec)
    boundary_loops.sort(key=lambda r: r["edge_count"], reverse=True)
    boundary_chains.sort(key=lambda r: r["edge_count"], reverse=True)

    hotspot_bins = Counter()
    for edge, mult in nonmanifold_edges:
        mid = edge_midpoint(edge)
        cell = (round(mid[0] / 20.0), round(mid[1] / 20.0), round(mid[2] / 20.0))
        hotspot_bins[cell] += mult - 2
    hotspots = []
    for cell, score in hotspot_bins.most_common(12):
        hotspots.append({
            "score": int(score),
            "approx_center_mm": {
                "x": float(cell[0] * 20.0),
                "y": float(cell[1] * 20.0),
                "z": float(cell[2] * 20.0),
            },
        })

    watertight_topology = len(boundary_edges) == 0 and len(nonmanifold_edges) == 0 and len(components) == 1
    return {
        "connected_components": {
            "count": len(components),
            "components": components[:20],
        },
        "boundary_edges": {
            "count": len(boundary_edges),
            "loop_count": len(boundary_loops),
            "open_chain_count": len(boundary_chains),
            "loops": boundary_loops[:20],
            "open_chains": boundary_chains[:20],
        },
        "non_manifold_edges": {
            "open_edge_count": len(boundary_edges),
            "gt2_face_edge_count": len(nonmanifold_edges),
            "hotspots": hotspots,
        },
        "watertightness_probe": {
            "topology_consistent_closed_shell": watertight_topology,
        },
    }


def edge_plane_intersection(a, b, axis, coord, eps):
    av = a[axis] - coord
    bv = b[axis] - coord
    if abs(av) <= eps and abs(bv) <= eps:
        return None
    if abs(av) <= eps:
        return a
    if abs(bv) <= eps:
        return b
    if av * bv > 0.0:
        return None
    t = av / (av - bv)
    return (
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
    )


def slice_triangle_const_x(tri, plane_x, eps):
    pts = []
    for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
        p = edge_plane_intersection(a, b, 0, plane_x, eps)
        if p is None:
            continue
        duplicate = False
        for q in pts:
            if abs(p[1] - q[1]) < eps and abs(p[2] - q[2]) < eps:
                duplicate = True
                break
        if not duplicate:
            pts.append(p)
    if len(pts) == 2:
        return pts[0], pts[1]
    return None


def collect_segments_const_x(triangles, plane_x, eps):
    segments = []
    for tri in triangles:
        seg = slice_triangle_const_x(tri, plane_x, eps)
        if seg is not None:
            segments.append(seg)
    return segments


def q2(v, tol):
    return tuple(int(round(c / tol)) for c in v)


def signed_area(poly):
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def polygon_centroid(poly):
    area = signed_area(poly)
    if abs(area) < 1e-9:
        return {"a": float(poly[:, 0].mean()), "b": float(poly[:, 1].mean())}
    x = poly[:, 0]
    y = poly[:, 1]
    cross = x * np.roll(y, -1) - np.roll(x, -1) * y
    cx = np.sum((x + np.roll(x, -1)) * cross) / (6.0 * area)
    cy = np.sum((y + np.roll(y, -1)) * cross) / (6.0 * area)
    return {"a": float(cx), "b": float(cy)}


def stitch_segments_to_loops(segments, tol=0.05):
    graph = defaultdict(set)
    edge_mult = Counter()
    node_points = {}
    for a, b in segments:
        qa = q2((a[1], a[2]), tol)
        qb = q2((b[1], b[2]), tol)
        if qa == qb:
            continue
        graph[qa].add(qb)
        graph[qb].add(qa)
        edge_mult[tuple(sorted((qa, qb)))] += 1
        node_points.setdefault(qa, (qa[0] * tol, qa[1] * tol))
        node_points.setdefault(qb, (qb[0] * tol, qb[1] * tol))

    loops = []
    fragments = []
    visited = set()
    for start in graph:
        if len(graph[start]) != 2:
            continue
        for nxt in graph[start]:
            e0 = tuple(sorted((start, nxt)))
            if e0 in visited:
                continue
            cycle = [start, nxt]
            visited.add(e0)
            prev = start
            cur = nxt
            closed = False
            while True:
                nbrs = [n for n in graph[cur] if n != prev]
                if not nbrs:
                    break
                nxt2 = nbrs[0]
                e = tuple(sorted((cur, nxt2)))
                if e in visited:
                    if nxt2 == cycle[0]:
                        closed = True
                    break
                visited.add(e)
                cycle.append(nxt2)
                prev, cur = cur, nxt2
                if cur == cycle[0]:
                    closed = True
                    break
            pts = np.asarray([node_points[q] for q in cycle], dtype=float)
            if closed and len(pts) >= 4:
                if not np.allclose(pts[0], pts[-1]):
                    pts = np.vstack([pts, pts[0]])
                area = abs(signed_area(pts))
                loops.append({
                    "point_count": int(len(pts)),
                    "area_mm2": float(area),
                    "centroid_mm": polygon_centroid(pts),
                    "bbox_mm": bbox_from_points_2d(pts),
                })
            else:
                fragments.append({
                    "node_count": int(len(cycle)),
                    "bbox_mm": bbox_from_points_2d(pts),
                })

    for start in graph:
        if len(graph[start]) == 2:
            continue
        pts = [node_points[start]]
        fragments.append({
            "node_count": 1,
            "bbox_mm": bbox_from_points_2d(np.asarray(pts, dtype=float)),
        })

    loops.sort(key=lambda r: r["area_mm2"], reverse=True)
    fragments.sort(key=lambda r: r["node_count"], reverse=True)
    return loops, fragments


def run_sample_section(script, x_station, out_csv):
    cmd = [
        "cargo", "run", "--release", "--bin", "sample_section", "--",
        f"--script={script}",
        "--plane=yz",
        f"--coord={x_station}",
        "--amin=-90.0",
        "--amax=90.0",
        "--bmin=-20.0",
        "--bmax=140.0",
        "--na=721",
        "--nb=641",
        "--backend=cpu",
        f"--out={out_csv}",
    ]
    subprocess.run(cmd, check=True)


def loops_from_sdf_csv(csv_path):
    a_name, b_name, aa, bb, d = load_grid(csv_path)
    import matplotlib.pyplot as plt
    A, B = np.meshgrid(aa, bb)
    fig, ax = plt.subplots()
    cs = ax.contour(A, B, d, levels=[0.0])
    plt.close(fig)
    loops = []
    for seg in cs.allsegs[0]:
        pts = np.asarray(seg, dtype=float)
        if len(pts) < 4:
            continue
        if not np.allclose(pts[0], pts[-1]):
            pts = np.vstack([pts, pts[0]])
        area = abs(signed_area(pts))
        loops.append({
            "point_count": int(len(pts)),
            "area_mm2": float(area),
            "centroid_mm": polygon_centroid(pts),
            "bbox_mm": bbox_from_points_2d(pts),
        })
    loops.sort(key=lambda r: r["area_mm2"], reverse=True)
    return loops


def match_loops(stl_loops, sdf_loops):
    sdf_major = [l for l in sdf_loops if l["area_mm2"] >= 20.0]
    stl_major = [l for l in stl_loops if l["area_mm2"] >= 20.0]
    matches = []
    used = set()
    for sidx, sdf in enumerate(sdf_major):
        best = None
        best_score = None
        for tidx, stl in enumerate(stl_major):
            if tidx in used:
                continue
            area_ratio = abs(stl["area_mm2"] - sdf["area_mm2"]) / max(sdf["area_mm2"], 1.0)
            dc = math.hypot(
                stl["centroid_mm"]["a"] - sdf["centroid_mm"]["a"],
                stl["centroid_mm"]["b"] - sdf["centroid_mm"]["b"],
            )
            score = area_ratio * 100.0 + dc
            if best_score is None or score < best_score:
                best = tidx
                best_score = score
        if best is not None:
            used.add(best)
            stl = stl_major[best]
            matches.append({
                "sdf_area_mm2": sdf["area_mm2"],
                "stl_area_mm2": stl["area_mm2"],
                "area_ratio_error": abs(stl["area_mm2"] - sdf["area_mm2"]) / max(sdf["area_mm2"], 1.0),
                "centroid_delta_mm": math.hypot(
                    stl["centroid_mm"]["a"] - sdf["centroid_mm"]["a"],
                    stl["centroid_mm"]["b"] - sdf["centroid_mm"]["b"],
                ),
            })
    excessive_fragmentation = len([l for l in stl_loops if l["area_mm2"] < 20.0]) > 8
    missing_major = len(matches) < len(sdf_major)
    bad_match = any(m["area_ratio_error"] > 0.25 or m["centroid_delta_mm"] > 5.0 for m in matches)
    passed = not excessive_fragmentation and not missing_major and not bad_match
    return {
        "sdf_loop_count": len(sdf_loops),
        "stl_loop_count": len(stl_loops),
        "sdf_major_loop_count": len(sdf_major),
        "stl_major_loop_count": len(stl_major),
        "matches": matches,
        "excessive_fragmentation": excessive_fragmentation,
        "missing_major_loops": missing_major,
        "bad_major_match": bad_match,
        "pass": passed,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--script", type=Path, required=True)
    ap.add_argument("--stl", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    args = ap.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    triangles = load_stl_triangles(args.stl)
    mesh_report = analyze_mesh(triangles)

    section_results = []
    for name, x_station in [("inlet_mouth", 250.0), ("inlet_transition", 300.0), ("exhaust", 660.0)]:
        csv_path = out_dir / f"{name}_yz.csv"
        run_sample_section(args.script, x_station, csv_path)
        sdf_loops = loops_from_sdf_csv(csv_path)
        segments = collect_segments_const_x(triangles, x_station, 1e-4)
        stl_loops, stl_fragments = stitch_segments_to_loops(segments, tol=0.05)
        section_results.append({
            "name": name,
            "x_station_mm": x_station,
            "segment_count": len(segments),
            "stl_loops": stl_loops[:30],
            "stl_fragment_count": len(stl_fragments),
            "stl_fragments": stl_fragments[:20],
            "sdf_loops": sdf_loops[:20],
            "validation": match_loops(stl_loops, sdf_loops),
        })

    report = {
        "script": str(args.script),
        "stl": str(args.stl),
        "mesh_integrity": mesh_report,
        "sections": section_results,
        "refinement_generalization": {
            "recommended_structure": "multiple refinement paths plus optional shared envelope regions",
            "notes": [
                "Represent refinement input as a list of path objects, each with points, radii, forward/aft extension, and local target cell.",
                "Permit overlapping regions from separate paths and merge them by taking the minimum local target cell at subdivision time.",
                "Keep field-level refinement only; do not split meshes or stitch local remeshes.",
            ],
        },
    }
    json_path = out_dir / "mesh_section_report.json"
    json_path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print(f"report={json_path}")


if __name__ == "__main__":
    main()
