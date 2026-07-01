import argparse
import json
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from plot_sdf_section_csv import load_grid
from plot_stl_section import load_stl_triangles


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


def rasterize_stl_mask(ys, zs, segments, eps):
    mask = np.zeros((len(zs), len(ys)), dtype=bool)
    if not segments:
        return mask
    for zi, z in enumerate(zs):
        intersections = []
        for a, b in segments:
            z0 = a[2]
            z1 = b[2]
            if abs(z1 - z0) <= eps:
                continue
            if z < min(z0, z1) - eps or z > max(z0, z1) + eps:
                continue
            t = (z - z0) / (z1 - z0)
            if t < -eps or t > 1.0 + eps:
                continue
            y = a[1] + t * (b[1] - a[1])
            intersections.append(y)
        if len(intersections) < 2:
            continue
        intersections.sort()
        for i in range(0, len(intersections) - 1, 2):
            y0 = intersections[i]
            y1 = intersections[i + 1]
            if y1 < y0:
                y0, y1 = y1, y0
            inside = (ys >= y0 - eps) & (ys <= y1 + eps)
            mask[zi, inside] = True
    return mask


def component_count(mask):
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    count = 0
    for z in range(h):
        for y in range(w):
            if not mask[z, y] or seen[z, y]:
                continue
            count += 1
            stack = [(z, y)]
            seen[z, y] = True
            while stack:
                cz, cy = stack.pop()
                for dz, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nz = cz + dz
                    ny = cy + dy
                    if nz < 0 or nz >= h or ny < 0 or ny >= w:
                        continue
                    if seen[nz, ny] or not mask[nz, ny]:
                        continue
                    seen[nz, ny] = True
                    stack.append((nz, ny))
    return int(count)


def bounds_from_mask(mask, ys, zs):
    if not mask.any():
        return None
    idx = np.argwhere(mask)
    z_idx = idx[:, 0]
    y_idx = idx[:, 1]
    return {
        "ymin": float(ys[y_idx].min()),
        "ymax": float(ys[y_idx].max()),
        "zmin": float(zs[z_idx].min()),
        "zmax": float(zs[z_idx].max()),
    }


def contour_bbox_delta(a, b):
    if a is None or b is None:
        return float("inf")
    return max(
        abs(a["ymin"] - b["ymin"]),
        abs(a["ymax"] - b["ymax"]),
        abs(a["zmin"] - b["zmin"]),
        abs(a["zmax"] - b["zmax"]),
    )


def plot_overlay(csv_path, segments, out_path, title):
    a_name, b_name, ys, zs, d = load_grid(csv_path)
    Y, Z = np.meshgrid(ys, zs)
    fig, ax = plt.subplots(figsize=(8, 6), dpi=180)
    ax.contour(Y, Z, d, levels=[0.0], colors=["#111111"], linewidths=1.2)
    first = True
    for a, b in segments:
        ax.plot([a[1], b[1]], [a[2], b[2]], color="#c0392b", linewidth=0.8,
                alpha=0.9, label="STL" if first else None)
        first = False
    ax.plot([], [], color="#111111", linewidth=1.2, label="SDF")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"{a_name.upper()} (mm)")
    ax.set_ylabel(f"{b_name.upper()} (mm)")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.legend()
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def run_sample_section(script, x_station, out_csv, y_bounds, z_bounds, ny, nz):
    cmd = [
        "cargo", "run", "--release", "--bin", "sample_section", "--",
        f"--script={script}",
        "--plane=yz",
        f"--coord={x_station}",
        f"--amin={y_bounds[0]}",
        f"--amax={y_bounds[1]}",
        f"--bmin={z_bounds[0]}",
        f"--bmax={z_bounds[1]}",
        f"--na={ny}",
        f"--nb={nz}",
        "--backend=cpu",
        f"--out={out_csv}",
    ]
    subprocess.run(cmd, check=True)


def evaluate_station(csv_path, stl_path, x_station, eps, out_png):
    a_name, b_name, ys, zs, d = load_grid(csv_path)
    if a_name.lower() != "y" or b_name.lower() != "z":
        raise ValueError(f"Expected YZ section CSV, got {a_name},{b_name}")
    sdf_mask = d <= 0.0
    triangles = load_stl_triangles(stl_path)
    segments = collect_segments_const_x(triangles, x_station, eps)
    stl_mask = rasterize_stl_mask(ys, zs, segments, eps)
    relevant = sdf_mask | stl_mask
    mismatch = sdf_mask ^ stl_mask
    mismatch_ratio = float(mismatch.sum() / max(int(relevant.sum()), 1))
    sdf_components = component_count(sdf_mask)
    stl_components = component_count(stl_mask)
    sdf_bbox = bounds_from_mask(sdf_mask, ys, zs)
    stl_bbox = bounds_from_mask(stl_mask, ys, zs)
    bbox_delta_mm = contour_bbox_delta(sdf_bbox, stl_bbox)
    passed = mismatch_ratio <= 0.12 and sdf_components == stl_components and bbox_delta_mm <= 6.0
    plot_overlay(csv_path, segments, out_png, f"YZ section at X = {x_station:.1f} mm")
    return {
        "x_station_mm": x_station,
        "sdf_csv": str(csv_path),
        "overlay_png": str(out_png),
        "segment_count": len(segments),
        "mismatch_ratio": mismatch_ratio,
        "sdf_components": sdf_components,
        "stl_components": stl_components,
        "sdf_bbox_mm": sdf_bbox,
        "stl_bbox_mm": stl_bbox,
        "bbox_delta_mm": bbox_delta_mm,
        "pass": passed,
    }


def main():
    ap = argparse.ArgumentParser(description="Validate a frozen STL export against direct SDF YZ sections at fixed X stations.")
    ap.add_argument("--script", type=Path, required=True)
    ap.add_argument("--stl", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    args = ap.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    stations = [
        ("inlet_mouth", 250.0),
        ("inlet_transition", 300.0),
        ("exhaust", 660.0),
    ]
    y_bounds = (-90.0, 90.0)
    z_bounds = (-20.0, 140.0)
    ny = 721
    nz = 641
    eps = 1e-4

    results = []
    for name, x_station in stations:
        csv_path = out_dir / f"{name}_yz_x{x_station:g}.csv"
        png_path = out_dir / f"{name}_yz_x{x_station:g}_overlay.png"
        run_sample_section(args.script, x_station, csv_path, y_bounds, z_bounds, ny, nz)
        results.append(evaluate_station(csv_path, args.stl, x_station, eps, png_path))

    summary = {
        "script": str(args.script),
        "stl": str(args.stl),
        "sampling": {
            "plane": "yz",
            "y_bounds_mm": list(y_bounds),
            "z_bounds_mm": list(z_bounds),
            "ny": ny,
            "nz": nz,
            "backend": "cpu",
        },
        "stations": results,
        "all_pass": all(r["pass"] for r in results),
    }
    json_path = out_dir / "validation_summary.json"
    json_path.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"summary={json_path}")


if __name__ == "__main__":
    main()
