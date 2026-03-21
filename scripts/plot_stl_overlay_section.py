import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plot_stl_section import load_stl_triangles, collect_segments


def main():
    ap = argparse.ArgumentParser(description="Overlay multiple STL XZ sections at a single Y plane.")
    ap.add_argument("stls", nargs="+", type=Path)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--eps", type=float, default=1e-4)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", type=str, default=None)
    args = ap.parse_args()

    colors = ["#000000", "#c0392b", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e"]
    fig, ax = plt.subplots(figsize=(10, 6), dpi=180)

    for i, stl in enumerate(args.stls):
        triangles = load_stl_triangles(stl)
        segments = collect_segments(triangles, args.y, args.eps)
        color = colors[i % len(colors)]
        label = stl.stem
        first = True
        for a, b in segments:
            ax.plot([a[0], b[0]], [a[2], b[2]],
                    color=color, linewidth=0.9, alpha=0.9,
                    label=label if first else None)
            first = False

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_title(args.title or f"Overlay section at Y = {args.y:.3f} mm")
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.out)
    plt.close(fig)
    print(f"saved={args.out}")


if __name__ == "__main__":
    main()
