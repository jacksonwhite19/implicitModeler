import argparse
import math
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_stl_triangles(path: Path):
    with path.open("rb") as f:
        header = f.read(80)
        if len(header) < 80:
            raise ValueError("STL file too short")
        tri_count_bytes = f.read(4)
        if len(tri_count_bytes) < 4:
            raise ValueError("Missing triangle count")
        tri_count = struct.unpack("<I", tri_count_bytes)[0]
        expected_size = 84 + tri_count * 50
        actual_size = path.stat().st_size

        if actual_size == expected_size:
            triangles = []
            for _ in range(tri_count):
                rec = f.read(50)
                if len(rec) < 50:
                    break
                vals = struct.unpack("<12fH", rec)
                v1 = (vals[3], vals[4], vals[5])
                v2 = (vals[6], vals[7], vals[8])
                v3 = (vals[9], vals[10], vals[11])
                triangles.append((v1, v2, v3))
            return triangles

    text = path.read_text(errors="ignore")
    triangles = []
    verts = []
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("vertex"):
            parts = line.split()
            if len(parts) >= 4:
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
                if len(verts) == 3:
                    triangles.append((verts[0], verts[1], verts[2]))
                    verts = []
    return triangles


def edge_plane_intersection(a, b, plane_y, eps):
    ay = a[1] - plane_y
    by = b[1] - plane_y
    if abs(ay) <= eps and abs(by) <= eps:
        return None
    if abs(ay) <= eps:
        return a
    if abs(by) <= eps:
        return b
    if ay * by > 0.0:
        return None
    t = ay / (ay - by)
    return (
        a[0] + t * (b[0] - a[0]),
        plane_y,
        a[2] + t * (b[2] - a[2]),
    )


def slice_triangle(tri, plane_y, eps):
    pts = []
    for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
        p = edge_plane_intersection(a, b, plane_y, eps)
        if p is None:
            continue
        duplicate = False
        for q in pts:
            if (abs(p[0] - q[0]) < eps) and (abs(p[2] - q[2]) < eps):
                duplicate = True
                break
        if not duplicate:
            pts.append(p)
    if len(pts) == 2:
        return pts[0], pts[1]
    return None


def collect_segments(triangles, plane_y, eps):
    segments = []
    for tri in triangles:
        seg = slice_triangle(tri, plane_y, eps)
        if seg is not None:
            segments.append(seg)
    return segments


def plot_segments(segments, out_png: Path, plane_y: float, title: str | None):
    fig, ax = plt.subplots(figsize=(10, 6), dpi=180)
    for (a, b) in segments:
        ax.plot([a[0], b[0]], [a[2], b[2]], color="black", linewidth=0.8)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_title(title or f"STL section at Y = {plane_y:.3f} mm")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Slice an STL with a Y=constant plane and plot the XZ section.")
    ap.add_argument("stl", type=Path)
    ap.add_argument("--y", type=float, default=0.0, help="Section plane Y position in mm")
    ap.add_argument("--eps", type=float, default=1e-4, help="Numerical tolerance")
    ap.add_argument("--out", type=Path, default=None, help="Output PNG path")
    ap.add_argument("--title", type=str, default=None)
    args = ap.parse_args()

    triangles = load_stl_triangles(args.stl)
    segments = collect_segments(triangles, args.y, args.eps)
    if not segments:
        raise SystemExit(f"No section segments found for Y={args.y}")

    out_png = args.out or args.stl.with_suffix(f".section_y{args.y:g}.png")
    plot_segments(segments, out_png, args.y, args.title)
    print(f"triangles={len(triangles)}")
    print(f"segments={len(segments)}")
    print(f"saved={out_png}")


if __name__ == "__main__":
    main()
