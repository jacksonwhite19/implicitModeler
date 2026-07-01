import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_sdf_section_csv import load_grid


def symmetric_limit(values: np.ndarray, percentile: float = 98.0) -> float:
    finite = np.abs(values[np.isfinite(values)])
    if finite.size == 0:
        return 1.0
    limit = float(np.percentile(finite, percentile))
    return max(limit, 1.0e-6)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot canonical vs conditioned section contours and distance delta."
    )
    parser.add_argument("--canonical", type=Path, required=True)
    parser.add_argument("--conditioned", type=Path, required=True)
    parser.add_argument("--delta", type=Path, required=True)
    parser.add_argument("--metrics", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--title", default="Conditioned SDF section comparison")
    args = parser.parse_args()

    a_name, b_name, a, b, canonical = load_grid(args.canonical)
    _, _, a2, b2, conditioned = load_grid(args.conditioned)
    _, _, a3, b3, delta = load_grid(args.delta)
    if not (np.array_equal(a, a2) and np.array_equal(b, b2) and np.array_equal(a, a3) and np.array_equal(b, b3)):
        raise SystemExit("CSV grids do not share the same coordinates")

    metrics = None
    if args.metrics and args.metrics.exists():
        metrics = json.loads(args.metrics.read_text())

    aa, bb = np.meshgrid(a, b)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), dpi=180)

    ax = axes[0]
    ax.contour(aa, bb, canonical, levels=[0.0], colors=["#111111"], linewidths=1.4)
    ax.contour(aa, bb, conditioned, levels=[0.0], colors=["#d62728"], linewidths=1.1, linestyles="--")
    ax.plot([], [], color="#111111", linewidth=1.4, label="canonical")
    ax.plot([], [], color="#d62728", linewidth=1.1, linestyle="--", label="conditioned")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.set_xlabel(f"{a_name.upper()} (mm)")
    ax.set_ylabel(f"{b_name.upper()} (mm)")
    ax.set_title("Zero contour overlay")
    ax.legend(loc="best")

    ax = axes[1]
    limit = symmetric_limit(delta)
    image = ax.imshow(
        delta,
        origin="lower",
        extent=[float(a[0]), float(a[-1]), float(b[0]), float(b[-1])],
        aspect="equal",
        cmap="coolwarm",
        vmin=-limit,
        vmax=limit,
    )
    ax.contour(aa, bb, canonical, levels=[0.0], colors=["#111111"], linewidths=0.8)
    ax.set_xlabel(f"{a_name.upper()} (mm)")
    ax.set_ylabel(f"{b_name.upper()} (mm)")
    ax.set_title("Conditioned - canonical distance")
    fig.colorbar(image, ax=ax, shrink=0.86, label="mm")

    subtitle = ""
    if metrics:
        runtime = metrics.get("runtime_vs_canonical", {})
        cache = metrics.get("raw_cache_vs_canonical", {})
        guard = metrics.get("published_guard", {})
        raw_drift = metrics.get("raw_storage_zero_contour_drift", {})
        guard_pct = 100.0 * float(guard.get("inferred_guarded_fraction", 0))
        subtitle = (
            f"published near p95={runtime.get('near_abs_delta_p95', 0):.3g} mm, "
            f"published near sign flips={runtime.get('sign_mismatch_near_interface_count', 0)}, "
            f"guard inferred={guard_pct:.1f}%, "
            f"raw-storage near p95={cache.get('near_abs_delta_p95', 0):.3g} mm, "
            f"raw zero-drift p95={raw_drift.get('canonical_to_comparison_p95', 0):.3g} mm"
        )
    fig.suptitle(args.title if not subtitle else f"{args.title}\n{subtitle}")
    fig.tight_layout()
    fig.savefig(args.out)
    plt.close(fig)
    print(f"saved={args.out}")


if __name__ == "__main__":
    main()
