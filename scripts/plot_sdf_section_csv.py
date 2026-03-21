import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_grid(path: Path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    names = data.dtype.names
    if names is None or len(names) < 3:
        raise ValueError(f"{path} does not have expected CSV headers")
    a_name, b_name = names[0], names[1]
    xs = np.unique(data[a_name])
    zs = np.unique(data[b_name])
    nx = len(xs)
    nz = len(zs)
    d = data["d"].reshape((nz, nx))
    return a_name, b_name, xs, zs, d


def main():
    ap = argparse.ArgumentParser(description="Plot zero contours from one or more SDF section CSV grids.")
    ap.add_argument("csvs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", type=str, default="SDF section")
    args = ap.parse_args()

    colors = ["#000000", "#c0392b", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e"]
    fig, ax = plt.subplots(figsize=(10, 6), dpi=180)

    for i, csv_path in enumerate(args.csvs):
        a_name, b_name, xs, zs, d = load_grid(csv_path)
        X, Z = np.meshgrid(xs, zs)
        cs = ax.contour(
            X, Z, d,
            levels=[0.0],
            colors=[colors[i % len(colors)]],
            linewidths=1.2,
        )
        if getattr(cs, "allsegs", None) and cs.allsegs and cs.allsegs[0]:
            ax.plot([], [], color=colors[i % len(colors)], linewidth=1.2, label=csv_path.stem)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"{a_name.upper()} (mm)")
    ax.set_ylabel(f"{b_name.upper()} (mm)")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.legend()
    ax.set_title(args.title)
    fig.tight_layout()
    fig.savefig(args.out)
    plt.close(fig)
    print(f"saved={args.out}")


if __name__ == "__main__":
    main()
