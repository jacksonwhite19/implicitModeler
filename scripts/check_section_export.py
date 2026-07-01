import argparse
import json
import sys
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from plot_sdf_section_csv import load_grid  # noqa: E402
from plot_stl_section import load_stl_triangles, collect_segments  # noqa: E402


def nearest_index(values: np.ndarray, target: float) -> int:
    return int(np.abs(values - target).argmin())


def point_to_segment_distance(px: float, pz: float, a, b) -> float:
    ax, az = a[0], a[2]
    bx, bz = b[0], b[2]
    vx = bx - ax
    vz = bz - az
    vv = vx * vx + vz * vz
    if vv <= 1e-12:
        return float(((px - ax) ** 2 + (pz - az) ** 2) ** 0.5)
    t = ((px - ax) * vx + (pz - az) * vz) / vv
    t = max(0.0, min(1.0, t))
    qx = ax + t * vx
    qz = az + t * vz
    return float(((px - qx) ** 2 + (pz - qz) ** 2) ** 0.5)


def nearest_segment_distance(px: float, pz: float, segments) -> float:
    if not segments:
        return float("inf")
    return min(point_to_segment_distance(px, pz, a, b) for a, b in segments)


def count_components(mask: np.ndarray) -> int:
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    components = 0
    for z in range(h):
        for x in range(w):
            if not mask[z, x] or seen[z, x]:
                continue
            components += 1
            q = deque([(z, x)])
            seen[z, x] = True
            while q:
                cz, cx = q.popleft()
                for dz, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nz = cz + dz
                    nx = cx + dx
                    if nz < 0 or nz >= h or nx < 0 or nx >= w:
                        continue
                    if seen[nz, nx] or not mask[nz, nx]:
                        continue
                    seen[nz, nx] = True
                    q.append((nz, nx))
    return components


def component_regions(mask: np.ndarray, xs: np.ndarray, zs: np.ndarray, limit: int = 12) -> list[dict]:
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    regions = []
    for z in range(h):
        for x in range(w):
            if not mask[z, x] or seen[z, x]:
                continue
            q = deque([(z, x)])
            seen[z, x] = True
            pixels = []
            while q:
                cz, cx = q.popleft()
                pixels.append((cz, cx))
                for dz, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nz = cz + dz
                    nx = cx + dx
                    if nz < 0 or nz >= h or nx < 0 or nx >= w:
                        continue
                    if seen[nz, nx] or not mask[nz, nx]:
                        continue
                    seen[nz, nx] = True
                    q.append((nz, nx))
            z_idx = np.array([p[0] for p in pixels], dtype=int)
            x_idx = np.array([p[1] for p in pixels], dtype=int)
            x_vals = xs[x_idx]
            z_vals = zs[z_idx]
            regions.append({
                "pixel_count": int(len(pixels)),
                "bbox": {
                    "xmin": float(x_vals.min()),
                    "xmax": float(x_vals.max()),
                    "zmin": float(z_vals.min()),
                    "zmax": float(z_vals.max()),
                },
                "centroid": {
                    "x": float(x_vals.mean()),
                    "z": float(z_vals.mean()),
                },
            })
    regions.sort(key=lambda r: r["pixel_count"], reverse=True)
    return regions[:limit]


def rasterize_stl_mask(
    xs: np.ndarray,
    zs: np.ndarray,
    stl_paths: list[Path],
    plane_y: float,
    eps: float,
) -> tuple[np.ndarray, list[tuple[tuple[float, float, float], tuple[float, float, float]]]]:
    mask = np.zeros((len(zs), len(xs)), dtype=bool)
    all_segments = []
    for stl_path in stl_paths:
        triangles = load_stl_triangles(stl_path)
        all_segments.extend(collect_segments(triangles, plane_y, eps))

    if not all_segments:
        return mask, all_segments

    for zi, z in enumerate(zs):
        intersections = []
        for a, b in all_segments:
            z0 = a[2]
            z1 = b[2]
            if abs(z1 - z0) <= eps:
                continue
            if z < min(z0, z1) - eps or z > max(z0, z1) + eps:
                continue
            t = (z - z0) / (z1 - z0)
            if t < -eps or t > 1.0 + eps:
                continue
            x = a[0] + t * (b[0] - a[0])
            intersections.append(x)
        if len(intersections) < 2:
            continue
        intersections.sort()
        for i in range(0, len(intersections) - 1, 2):
            x0 = intersections[i]
            x1 = intersections[i + 1]
            if x1 < x0:
                x0, x1 = x1, x0
            inside = (xs >= x0 - eps) & (xs <= x1 + eps)
            mask[zi, inside] = True

    return mask, all_segments


def build_diff_image(
    xs: np.ndarray,
    zs: np.ndarray,
    sdf_mask: np.ndarray,
    stl_mask: np.ndarray,
    stl_segments,
    probes,
    sdf_only_regions,
    stl_only_regions,
    out_path: Path,
    title: str,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
    diff = np.zeros((*sdf_mask.shape, 3), dtype=float)
    both = sdf_mask & stl_mask
    sdf_only = sdf_mask & ~stl_mask
    stl_only = stl_mask & ~sdf_mask
    diff[both] = np.array([0.92, 0.92, 0.92])
    diff[sdf_only] = np.array([0.85, 0.20, 0.20])
    diff[stl_only] = np.array([0.20, 0.35, 0.85])
    extent = [float(xs[0]), float(xs[-1]), float(zs[0]), float(zs[-1])]
    ax.imshow(diff, origin="lower", extent=extent, aspect="equal", alpha=0.85)

    for a, b in stl_segments:
        ax.plot([a[0], b[0]], [a[2], b[2]], color="#1f4fa3", linewidth=0.5, alpha=0.85)

    X, Z = np.meshgrid(xs, zs)
    sdf_scalar = np.where(sdf_mask, -1.0, 1.0)
    ax.contour(X, Z, sdf_scalar, levels=[0.0], colors=["#111111"], linewidths=1.0)

    for probe in probes:
        color = "#2ca02c" if probe["match"] else "#ff7f0e"
        marker = "o" if probe["match"] else "x"
        ax.scatter([probe["x"]], [probe["z"]], c=color, s=30, marker=marker, zorder=5)
        ax.text(probe["x"] + 3.0, probe["z"] + 3.0, probe["label"], color=color, fontsize=7)

    for idx, region in enumerate(sdf_only_regions[:5], start=1):
        bbox = region["bbox"]
        rect = plt.Rectangle(
            (bbox["xmin"], bbox["zmin"]),
            bbox["xmax"] - bbox["xmin"],
            bbox["zmax"] - bbox["zmin"],
            fill=False,
            edgecolor="#c62828",
            linewidth=1.0,
            linestyle="--",
            alpha=0.9,
        )
        ax.add_patch(rect)
        ax.text(bbox["xmin"], bbox["zmax"] + 2.0, f"SDF-only {idx}", color="#c62828", fontsize=7)

    for idx, region in enumerate(stl_only_regions[:5], start=1):
        bbox = region["bbox"]
        rect = plt.Rectangle(
            (bbox["xmin"], bbox["zmin"]),
            bbox["xmax"] - bbox["xmin"],
            bbox["zmax"] - bbox["zmin"],
            fill=False,
            edgecolor="#1565c0",
            linewidth=1.0,
            linestyle=":",
            alpha=0.9,
        )
        ax.add_patch(rect)
        ax.text(bbox["xmin"], bbox["zmin"] - 4.0, f"STL-only {idx}", color="#1565c0", fontsize=7)

    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare an SDF section CSV against sliced STL sections.")
    ap.add_argument("--sdf-csv", type=Path, required=True)
    ap.add_argument("--stl", dest="stls", type=Path, nargs="+", required=True)
    ap.add_argument("--plane-y", type=float, default=0.0)
    ap.add_argument("--eps", type=float, default=1e-4)
    ap.add_argument("--out-prefix", type=Path, required=True)
    ap.add_argument("--max-mismatch-ratio", type=float, default=0.02)
    ap.add_argument(
        "--probe",
        action="append",
        nargs=4,
        metavar=("X", "Z", "EXPECT", "LABEL"),
        help="Probe a point on the section. EXPECT is one of solid|void.",
    )
    ap.add_argument(
        "--rule",
        action="append",
        nargs=5,
        metavar=("NAME", "KIND", "X", "Z", "EXPECT"),
        help=(
            "Named rule evaluated at a section point. "
            "KIND is one of probe|clearance|surface. "
            "For probe, EXPECT is solid|void. "
            "For clearance/surface, EXPECT is a distance in mm."
        ),
    )
    args = ap.parse_args()

    a_name, b_name, xs, zs, d = load_grid(args.sdf_csv)
    if a_name.lower() != "x" or b_name.lower() != "z":
        raise SystemExit(f"Expected XZ CSV, got headers {a_name},{b_name}")
    sdf_mask = d <= 0.0
    stl_mask, stl_segments = rasterize_stl_mask(xs, zs, args.stls, args.plane_y, args.eps)

    relevant = sdf_mask | stl_mask
    mismatch = sdf_mask ^ stl_mask
    sdf_only = sdf_mask & ~stl_mask
    stl_only = stl_mask & ~sdf_mask
    relevant_count = int(relevant.sum())
    mismatch_count = int(mismatch.sum())
    mismatch_ratio = float(mismatch_count / max(relevant_count, 1))
    sdf_only_regions = component_regions(sdf_only, xs, zs)
    stl_only_regions = component_regions(stl_only, xs, zs)

    probes = []
    failed_probes = []
    rules = []
    failed_rules = []
    for raw in args.probe or []:
        x = float(raw[0])
        z = float(raw[1])
        expect = raw[2].strip().lower()
        label = raw[3]
        if expect not in {"solid", "void"}:
            raise SystemExit(f"Unsupported probe expectation '{expect}'")
        xi = nearest_index(xs, x)
        zi = nearest_index(zs, z)
        sdf_state = "solid" if sdf_mask[zi, xi] else "void"
        stl_state = "solid" if stl_mask[zi, xi] else "void"
        expected_match = (sdf_state == expect)
        if not expected_match:
            raise SystemExit(
                f"Probe '{label}' expectation is inconsistent with source SDF: expected {expect}, got {sdf_state}"
            )
        match = sdf_state == stl_state
        record = {
            "label": label,
            "x": x,
            "z": z,
            "expected": expect,
            "sdf": sdf_state,
            "stl": stl_state,
            "match": match,
        }
        probes.append(record)
        if not match:
            failed_probes.append(record)

    for raw in args.rule or []:
        name = raw[0]
        kind = raw[1].strip().lower()
        x = float(raw[2])
        z = float(raw[3])
        xi = nearest_index(xs, x)
        zi = nearest_index(zs, z)
        sdf_state = "solid" if sdf_mask[zi, xi] else "void"
        stl_state = "solid" if stl_mask[zi, xi] else "void"

        if kind == "probe":
            expect = raw[4].strip().lower()
            if expect not in {"solid", "void"}:
                raise SystemExit(f"Unsupported probe rule expectation '{expect}' for rule '{name}'")
            if sdf_state != expect:
                raise SystemExit(
                    f"Rule '{name}' expectation is inconsistent with source SDF: expected {expect}, got {sdf_state}"
                )
            passed = stl_state == expect
            failure_mode = None
            if not passed:
                if expect == "void" and stl_state == "solid":
                    failure_mode = "false_closure_or_extra_material"
                elif expect == "solid" and stl_state == "void":
                    failure_mode = "lost_wall_or_missing_surface"
                else:
                    failure_mode = "unexpected_state_mismatch"
            record = {
                "name": name,
                "kind": kind,
                "x": x,
                "z": z,
                "expected": expect,
                "sdf": sdf_state,
                "stl": stl_state,
                "pass": passed,
                "failure_mode": failure_mode,
            }
        elif kind in {"clearance", "surface"}:
            tol_mm = float(raw[4])
            if tol_mm <= 0.0:
                raise SystemExit(f"Rule '{name}' must use a positive tolerance")
            nearest_mm = nearest_segment_distance(x, z, stl_segments)
            if kind == "clearance":
                passed = nearest_mm >= tol_mm
                failure_mode = None if passed else "surface_intrudes_into_required_open_region"
            else:
                passed = nearest_mm <= tol_mm
                failure_mode = None if passed else "expected_surface_missing_near_probe"
            record = {
                "name": name,
                "kind": kind,
                "x": x,
                "z": z,
                "tolerance_mm": tol_mm,
                "nearest_segment_distance_mm": nearest_mm,
                "pass": passed,
                "failure_mode": failure_mode,
            }
        else:
            raise SystemExit(f"Unsupported rule kind '{kind}' for rule '{name}'")
        rules.append(record)
        if not passed:
            failed_rules.append(record)

    repair_hints = []
    for rule in failed_rules:
        if rule["failure_mode"] == "false_closure_or_extra_material":
            repair_hints.append({
                "rule": rule["name"],
                "kind": "geometry_opening",
                "priority": "high",
                "location": {"x": rule["x"], "z": rule["z"], "plane_y": args.plane_y},
                "suggestion": (
                    "Source SDF is open here but exported STL is closed. "
                    "Inspect the aero-export outer patch near this station for a local cap or bridging surface. "
                    "Prefer a localized mouth/transition cut or a dedicated flow-opening export SDF over subtracting the full duct void."
                ),
            })
        elif rule["failure_mode"] == "lost_wall_or_missing_surface":
            repair_hints.append({
                "rule": rule["name"],
                "kind": "wall_preservation",
                "priority": "high",
                "location": {"x": rule["x"], "z": rule["z"], "plane_y": args.plane_y},
                "suggestion": (
                    "Source SDF is solid here but exported STL is missing it. "
                    "Inspect the export cutter/boolean around this station and tighten it to avoid removing load-bearing wall geometry."
                ),
            })
        elif rule["failure_mode"] == "surface_intrudes_into_required_open_region":
            repair_hints.append({
                "rule": rule["name"],
                "kind": "clearance_violation",
                "priority": "high",
                "location": {"x": rule["x"], "z": rule["z"], "plane_y": args.plane_y},
                "suggestion": (
                    "A section segment lies too close to a point that should remain open. "
                    "Trim or relocate the local export opening/cap geometry so the passage retains clearance."
                ),
            })
        elif rule["failure_mode"] == "expected_surface_missing_near_probe":
            repair_hints.append({
                "rule": rule["name"],
                "kind": "surface_absence",
                "priority": "high",
                "location": {"x": rule["x"], "z": rule["z"], "plane_y": args.plane_y},
                "suggestion": (
                    "No section segment was found close to a location that should lie on a wall. "
                    "Restore local shell geometry or move the cut so the wall survives export."
                ),
            })

    for region in sdf_only_regions[:5]:
        repair_hints.append({
            "kind": "lost_surface_region",
            "priority": "medium",
            "bbox": region["bbox"],
            "centroid": region["centroid"],
            "suggestion": (
                "Export lost solid surface in this region. "
                "Check whether a mouth-opening subtraction or patch filtering is overcutting adjacent structure."
            ),
        })
    for region in stl_only_regions[:5]:
        repair_hints.append({
            "kind": "false_closure_region",
            "priority": "medium",
            "bbox": region["bbox"],
            "centroid": region["centroid"],
            "suggestion": (
                "Export added solid material in this region. "
                "Check for caps, bridge surfaces, or STL patch overlap that closes a passage or fills a void."
            ),
        })

    mismatch_ok = args.max_mismatch_ratio < 0.0 or mismatch_ratio <= args.max_mismatch_ratio

    report = {
        "sdf_csv": str(args.sdf_csv),
        "stls": [str(p) for p in args.stls],
        "plane_y": args.plane_y,
        "grid": {
            "nx": int(len(xs)),
            "nz": int(len(zs)),
            "xmin": float(xs[0]),
            "xmax": float(xs[-1]),
            "zmin": float(zs[0]),
            "zmax": float(zs[-1]),
        },
        "metrics": {
            "relevant_pixels": relevant_count,
            "mismatch_pixels": mismatch_count,
            "mismatch_ratio": mismatch_ratio,
            "sdf_components": count_components(sdf_mask),
            "stl_components": count_components(stl_mask),
            "sdf_only_components": count_components(sdf_only),
            "stl_only_components": count_components(stl_only),
            "segment_count": len(stl_segments),
        },
        "actionable_regions": {
            "sdf_only": [
                {
                    **region,
                    "meaning": "Source SDF contains solid here, but exported STL does not. Likely lost wall/surface during export."
                }
                for region in sdf_only_regions
            ],
            "stl_only": [
                {
                    **region,
                    "meaning": "Exported STL contains solid here, but source SDF is void. Likely false closure, cap, or extra material."
                }
                for region in stl_only_regions
            ],
        },
        "probes": probes,
        "rules": rules,
        "repair_hints": repair_hints,
        "pass": mismatch_ok and not failed_probes and not failed_rules,
    }

    json_path = args.out_prefix.with_suffix(".json")
    png_path = args.out_prefix.with_suffix(".png")
    build_diff_image(
        xs,
        zs,
        sdf_mask,
        stl_mask,
        stl_segments,
        probes,
        sdf_only_regions,
        stl_only_regions,
        png_path,
        "SDF vs STL Section Check",
    )
    json_path.write_text(json.dumps(report, indent=2))

    print(json.dumps(report, indent=2))
    print(f"report={json_path}")
    print(f"overlay={png_path}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
