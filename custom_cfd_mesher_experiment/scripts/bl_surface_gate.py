from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate prism boundary-layer suitability of an aircraft STL surface.")
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--bad-triangles-csv", type=Path, default=None)
    parser.add_argument("--max-bad-triangles", type=int, default=500)
    parser.add_argument("--max-aspect-ratio", type=float, default=60.0)
    parser.add_argument("--min-quality", type=float, default=0.05)
    parser.add_argument("--min-angle-deg", type=float, default=2.5)
    parser.add_argument("--min-altitude-mm", type=float, default=0.02)
    parser.add_argument("--max-sliver-count", type=int, default=0)
    parser.add_argument("--max-short-altitude-count", type=int, default=0)
    parser.add_argument("--max-small-angle-count", type=int, default=0)
    parser.add_argument("--grid", default="8,8,5")
    args = parser.parse_args()

    mesh = load_mesh(args.input_stl)
    metrics = evaluate_surface(mesh, grid=parse_grid(args.grid))
    failures = gate_failures(
        metrics,
        max_aspect_ratio=args.max_aspect_ratio,
        min_quality=args.min_quality,
        min_angle_deg=args.min_angle_deg,
        min_altitude_mm=args.min_altitude_mm,
        max_sliver_count=args.max_sliver_count,
        max_short_altitude_count=args.max_short_altitude_count,
        max_small_angle_count=args.max_small_angle_count,
    )

    report = {
        "gate": "boundary_layer_surface",
        "input_stl": str(args.input_stl),
        "pass_gate": not failures,
        "failures": failures,
        "limits": {
            "max_aspect_ratio": args.max_aspect_ratio,
            "min_quality": args.min_quality,
            "min_angle_deg": args.min_angle_deg,
            "min_altitude_mm": args.min_altitude_mm,
            "max_sliver_count": args.max_sliver_count,
            "max_short_altitude_count": args.max_short_altitude_count,
            "max_small_angle_count": args.max_small_angle_count,
        },
        "surface": metrics,
    }

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    if args.bad_triangles_csv:
        write_bad_triangles_csv(
            args.bad_triangles_csv,
            metrics["bad_triangles"][: max(args.max_bad_triangles, 0)],
        )
    print(json.dumps(report, indent=2, sort_keys=True))
    if failures:
        raise SystemExit(2)


def load_mesh(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh: {path}")
    mesh = mesh.copy()
    mesh.merge_vertices(digits_vertex=8)
    mesh.remove_unreferenced_vertices()
    return mesh


def evaluate_surface(mesh: trimesh.Trimesh, *, grid: tuple[int, int, int]) -> dict[str, Any]:
    tri = mesh.vertices[mesh.faces]
    edges = triangle_edge_lengths(tri)
    area = triangle_area(tri)
    quality = triangle_quality_from_edges(edges, area)
    min_altitude = triangle_min_altitude(edges, area)
    angles = triangle_angles_degrees(edges)
    min_angle = np.min(angles, axis=1) if len(angles) else np.array([], dtype=float)
    max_angle = np.max(angles, axis=1) if len(angles) else np.array([], dtype=float)
    aspect = triangle_aspect_ratio(edges, min_altitude)
    centers = tri.mean(axis=1) if len(tri) else np.empty((0, 3))

    bad_mask = (
        (quality < 0.10)
        | (aspect > 40.0)
        | (min_angle < 3.0)
        | (min_altitude < 0.03)
    )
    bad_indices = np.where(bad_mask)[0]
    ranked = sorted(
        bad_indices.tolist(),
        key=lambda idx: (
            min(float(quality[idx]) / 0.10, 1.0),
            min(float(min_angle[idx]) / 3.0, 1.0),
            min(float(min_altitude[idx]) / 0.03, 1.0),
            -float(aspect[idx]),
        ),
    )
    bad_records = [
        {
            "face_index": int(idx),
            "center_mm": centers[idx].tolist(),
            "quality": float(quality[idx]),
            "aspect_ratio": float(aspect[idx]),
            "min_angle_deg": float(min_angle[idx]),
            "max_angle_deg": float(max_angle[idx]),
            "min_altitude_mm": float(min_altitude[idx]),
            "edge_lengths_mm": edges[idx].tolist(),
        }
        for idx in ranked
    ]

    return {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "bounds_mm": mesh.bounds.tolist(),
        "edge_length_mm": percentile_report(edges.reshape(-1)),
        "triangle_quality": percentile_report(quality),
        "triangle_aspect_ratio": percentile_report(aspect),
        "triangle_min_altitude_mm": percentile_report(min_altitude),
        "triangle_min_angle_deg": percentile_report(min_angle),
        "triangle_max_angle_deg": percentile_report(max_angle),
        "counts": {
            "quality_lt_0p02": int(np.count_nonzero(quality < 0.02)),
            "quality_lt_0p05": int(np.count_nonzero(quality < 0.05)),
            "quality_lt_0p10": int(np.count_nonzero(quality < 0.10)),
            "aspect_gt_40": int(np.count_nonzero(aspect > 40.0)),
            "aspect_gt_60": int(np.count_nonzero(aspect > 60.0)),
            "aspect_gt_100": int(np.count_nonzero(aspect > 100.0)),
            "min_angle_lt_1deg": int(np.count_nonzero(min_angle < 1.0)),
            "min_angle_lt_2p5deg": int(np.count_nonzero(min_angle < 2.5)),
            "min_angle_lt_5deg": int(np.count_nonzero(min_angle < 5.0)),
            "min_altitude_lt_0p01mm": int(np.count_nonzero(min_altitude < 0.01)),
            "min_altitude_lt_0p02mm": int(np.count_nonzero(min_altitude < 0.02)),
            "min_altitude_lt_0p05mm": int(np.count_nonzero(min_altitude < 0.05)),
            "bad_triangle_count": int(len(bad_records)),
        },
        "bad_triangle_clusters": cluster_bad_triangles(centers[bad_indices], mesh.bounds, grid=grid),
        "bad_triangles": bad_records,
    }


def triangle_edge_lengths(tri: np.ndarray) -> np.ndarray:
    if len(tri) == 0:
        return np.empty((0, 3), dtype=float)
    return np.column_stack(
        [
            np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1),
            np.linalg.norm(tri[:, 2] - tri[:, 1], axis=1),
            np.linalg.norm(tri[:, 0] - tri[:, 2], axis=1),
        ]
    )


def triangle_area(tri: np.ndarray) -> np.ndarray:
    if len(tri) == 0:
        return np.array([], dtype=float)
    return 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)


def triangle_quality_from_edges(edges: np.ndarray, area: np.ndarray) -> np.ndarray:
    denom = np.sum(edges * edges, axis=1)
    return np.divide(4.0 * np.sqrt(3.0) * area, denom, out=np.zeros_like(area), where=denom > 0.0)


def triangle_min_altitude(edges: np.ndarray, area: np.ndarray) -> np.ndarray:
    max_edge = np.max(edges, axis=1) if len(edges) else np.array([], dtype=float)
    return np.divide(2.0 * area, max_edge, out=np.zeros_like(area), where=max_edge > 0.0)


def triangle_aspect_ratio(edges: np.ndarray, min_altitude: np.ndarray) -> np.ndarray:
    max_edge = np.max(edges, axis=1) if len(edges) else np.array([], dtype=float)
    return np.divide(max_edge, min_altitude, out=np.full_like(max_edge, np.inf), where=min_altitude > 0.0)


def triangle_angles_degrees(edges: np.ndarray) -> np.ndarray:
    if len(edges) == 0:
        return np.empty((0, 3), dtype=float)
    a, b, c = edges[:, 0], edges[:, 1], edges[:, 2]
    angles = np.column_stack(
        [
            law_of_cosines_angle(c, a, b),
            law_of_cosines_angle(a, b, c),
            law_of_cosines_angle(b, c, a),
        ]
    )
    return np.degrees(angles)


def law_of_cosines_angle(adjacent_a: np.ndarray, adjacent_b: np.ndarray, opposite: np.ndarray) -> np.ndarray:
    denom = 2.0 * adjacent_a * adjacent_b
    cosine = np.divide(
        adjacent_a * adjacent_a + adjacent_b * adjacent_b - opposite * opposite,
        denom,
        out=np.ones_like(opposite),
        where=denom > 0.0,
    )
    return np.arccos(np.clip(cosine, -1.0, 1.0))


def percentile_report(values: np.ndarray) -> dict[str, float | None]:
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        return {"min": None, "p01": None, "p05": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "min": float(np.min(finite)),
        "p01": float(np.percentile(finite, 1)),
        "p05": float(np.percentile(finite, 5)),
        "p50": float(np.percentile(finite, 50)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "max": float(np.max(finite)),
    }


def cluster_bad_triangles(centers: np.ndarray, bounds: np.ndarray, *, grid: tuple[int, int, int]) -> list[dict[str, Any]]:
    if len(centers) == 0:
        return []
    lo, hi = bounds
    span = np.maximum(hi - lo, 1.0e-9)
    norm = np.clip((centers - lo) / span, 0.0, np.nextafter(1.0, 0.0))
    grid_array = np.asarray(grid, dtype=int)
    indices = np.floor(norm * grid_array).astype(int)
    bins: dict[tuple[int, int, int], list[int]] = {}
    for row, key in enumerate(map(tuple, indices)):
        bins.setdefault(key, []).append(row)
    records = []
    for key, rows in sorted(bins.items(), key=lambda item: len(item[1]), reverse=True)[:12]:
        pts = centers[np.asarray(rows, dtype=int)]
        mean_norm = norm[np.asarray(rows, dtype=int)].mean(axis=0)
        records.append(
            {
                "voxel_index": list(map(int, key)),
                "count": int(len(rows)),
                "center_mean_mm": pts.mean(axis=0).tolist(),
                "center_bounds_mm": [pts.min(axis=0).tolist(), pts.max(axis=0).tolist()],
                "normalized_center_mean": mean_norm.tolist(),
                "region_label": region_label(mean_norm),
            }
        )
    return records


def region_label(norm_mean: np.ndarray) -> str:
    x = axis_bucket(float(norm_mean[0]), ("nose", "front_mid", "mid", "aft_mid", "tail"))
    y = axis_bucket(float(norm_mean[1]), ("left_tip", "left_mid", "center", "right_mid", "right_tip"))
    z = axis_bucket(float(norm_mean[2]), ("low", "lower_mid", "mid", "upper_mid", "high"))
    return f"{x}/{y}/{z}"


def axis_bucket(value: float, names: tuple[str, str, str, str, str]) -> str:
    index = min(max(int(value * len(names)), 0), len(names) - 1)
    return names[index]


def gate_failures(
    metrics: dict[str, Any],
    *,
    max_aspect_ratio: float,
    min_quality: float,
    min_angle_deg: float,
    min_altitude_mm: float,
    max_sliver_count: int,
    max_short_altitude_count: int,
    max_small_angle_count: int,
) -> list[str]:
    counts = metrics["counts"]
    failures = []
    if not metrics["watertight"]:
        failures.append("surface is not watertight")
    if (metrics["triangle_aspect_ratio"]["max"] or 0.0) > max_aspect_ratio:
        failures.append(f"triangle_aspect_ratio.max exceeds {max_aspect_ratio}")
    if (metrics["triangle_quality"]["min"] or 0.0) < min_quality:
        failures.append(f"triangle_quality.min below {min_quality}")
    if (metrics["triangle_min_angle_deg"]["min"] or 0.0) < min_angle_deg:
        failures.append(f"triangle_min_angle_deg.min below {min_angle_deg}")
    if (metrics["triangle_min_altitude_mm"]["min"] or 0.0) < min_altitude_mm:
        failures.append(f"triangle_min_altitude_mm.min below {min_altitude_mm}")
    if int(counts["quality_lt_0p05"]) > max_sliver_count:
        failures.append(f"quality_lt_0p05={counts['quality_lt_0p05']} exceeds {max_sliver_count}")
    if int(counts["min_altitude_lt_0p02mm"]) > max_short_altitude_count:
        failures.append(f"min_altitude_lt_0p02mm={counts['min_altitude_lt_0p02mm']} exceeds {max_short_altitude_count}")
    if int(counts["min_angle_lt_2p5deg"]) > max_small_angle_count:
        failures.append(f"min_angle_lt_2p5deg={counts['min_angle_lt_2p5deg']} exceeds {max_small_angle_count}")
    return failures


def write_bad_triangles_csv(path: Path, records: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "face_index",
        "center_x_mm",
        "center_y_mm",
        "center_z_mm",
        "quality",
        "aspect_ratio",
        "min_angle_deg",
        "max_angle_deg",
        "min_altitude_mm",
        "edge_a_mm",
        "edge_b_mm",
        "edge_c_mm",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            center = record["center_mm"]
            edges = record["edge_lengths_mm"]
            writer.writerow(
                {
                    "face_index": record["face_index"],
                    "center_x_mm": center[0],
                    "center_y_mm": center[1],
                    "center_z_mm": center[2],
                    "quality": record["quality"],
                    "aspect_ratio": record["aspect_ratio"],
                    "min_angle_deg": record["min_angle_deg"],
                    "max_angle_deg": record["max_angle_deg"],
                    "min_altitude_mm": record["min_altitude_mm"],
                    "edge_a_mm": edges[0],
                    "edge_b_mm": edges[1],
                    "edge_c_mm": edges[2],
                }
            )


def parse_grid(value: str) -> tuple[int, int, int]:
    parts = [int(part) for part in value.split(",")]
    if len(parts) != 3 or any(part <= 0 for part in parts):
        raise ValueError("Expected grid as positive nx,ny,nz")
    return parts[0], parts[1], parts[2]


if __name__ == "__main__":
    main()
