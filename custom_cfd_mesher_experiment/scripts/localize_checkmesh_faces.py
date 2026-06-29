from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vtk", type=Path, required=True)
    parser.add_argument("--aircraft-bounds", required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--grid", type=int, default=5)
    parser.add_argument("--top-voxels", type=int, default=12)
    parser.add_argument("--refinement-padding-frac", type=float, default=0.04)
    args = parser.parse_args()

    faces = read_vtk_polygons(args.vtk)
    bounds = parse_bounds(args.aircraft_bounds)
    report = localize_faces(
        faces,
        bounds,
        grid=max(args.grid, 1),
        top_voxels=max(args.top_voxels, 0),
        refinement_padding_frac=max(args.refinement_padding_frac, 0.0),
    )
    report["vtk"] = str(args.vtk)
    report["aircraft_bounds_m"] = bounds.tolist()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def read_vtk_polygons(path: Path) -> np.ndarray:
    text = path.read_text(encoding="utf-8", errors="replace")
    points_match = re.search(r"POINTS\s+(\d+)\s+\w+\s+(.+?)(?:\nPOLYGONS|\nCELLS)", text, re.S)
    if not points_match:
        raise ValueError(f"Could not parse POINTS from {path}")
    point_count = int(points_match.group(1))
    point_values = np.fromstring(points_match.group(2), sep=" ", dtype=float)
    points = point_values.reshape((point_count, 3))

    poly_match = re.search(r"(?:POLYGONS|CELLS)\s+(\d+)\s+\d+\s+(.+?)(?:\nCELL_TYPES|\nPOINT_DATA|\nCELL_DATA|$)", text, re.S)
    if not poly_match:
        raise ValueError(f"Could not parse polygons/cells from {path}")
    face_count = int(poly_match.group(1))
    values = np.fromstring(poly_match.group(2), sep=" ", dtype=int)
    faces = []
    cursor = 0
    for _ in range(face_count):
        n = int(values[cursor])
        cursor += 1
        indices = values[cursor : cursor + n]
        cursor += n
        faces.append(points[indices].mean(axis=0))
    return np.asarray(faces)


def parse_bounds(value: str) -> np.ndarray:
    parts = [float(part) for part in value.split(",")]
    if len(parts) != 6:
        raise ValueError("Expected aircraft bounds as xmin,ymin,zmin,xmax,ymax,zmax")
    return np.asarray([[parts[0], parts[1], parts[2]], [parts[3], parts[4], parts[5]]], dtype=float)


def localize_faces(
    centers: np.ndarray,
    bounds: np.ndarray,
    *,
    grid: int,
    top_voxels: int,
    refinement_padding_frac: float,
) -> dict[str, object]:
    lo, hi = bounds
    span = hi - lo
    norm = (centers - lo) / span
    inside = np.all((norm >= 0.0) & (norm <= 1.0), axis=1)
    counts = {
        "total": int(len(centers)),
        "inside_aircraft_bounds": int(np.sum(inside)),
        "outside_aircraft_bounds": int(np.sum(~inside)),
    }
    bins = {
        "nose_x_0_20": norm[:, 0] < 0.20,
        "front_mid_x_20_40": (norm[:, 0] >= 0.20) & (norm[:, 0] < 0.40),
        "mid_x_40_60": (norm[:, 0] >= 0.40) & (norm[:, 0] < 0.60),
        "aft_mid_x_60_80": (norm[:, 0] >= 0.60) & (norm[:, 0] < 0.80),
        "tail_x_80_100": norm[:, 0] >= 0.80,
        "left_wing_y_0_20": norm[:, 1] < 0.20,
        "right_wing_y_80_100": norm[:, 1] > 0.80,
        "center_y_35_65": (norm[:, 1] >= 0.35) & (norm[:, 1] <= 0.65),
        "low_z_0_25": norm[:, 2] < 0.25,
        "mid_z_25_75": (norm[:, 2] >= 0.25) & (norm[:, 2] <= 0.75),
        "high_z_75_100": norm[:, 2] > 0.75,
        "wing_plane_band": (norm[:, 2] >= 0.35) & (norm[:, 2] <= 0.75),
    }
    counts["normalized_region_counts"] = {name: int(np.sum(mask)) for name, mask in bins.items()}
    counts["centroid_bounds_m"] = [centers.min(axis=0).tolist(), centers.max(axis=0).tolist()]
    counts["normalized_centroid_bounds"] = [norm.min(axis=0).tolist(), norm.max(axis=0).tolist()]
    counts["normalized_quantiles"] = {
        axis: {
            "p05": float(np.percentile(norm[:, index], 5)),
            "p25": float(np.percentile(norm[:, index], 25)),
            "p50": float(np.percentile(norm[:, index], 50)),
            "p75": float(np.percentile(norm[:, index], 75)),
            "p95": float(np.percentile(norm[:, index], 95)),
        }
        for index, axis in enumerate(("x", "y", "z"))
    }
    counts["top_normalized_voxels"] = top_voxel_report(
        centers,
        norm,
        bounds,
        grid=grid,
        limit=top_voxels,
        padding_frac=refinement_padding_frac,
    )
    return counts


def top_voxel_report(
    centers: np.ndarray,
    norm: np.ndarray,
    bounds: np.ndarray,
    *,
    grid: int,
    limit: int,
    padding_frac: float,
) -> list[dict[str, object]]:
    if len(centers) == 0 or limit == 0:
        return []
    clipped = np.clip(norm, 0.0, np.nextafter(1.0, 0.0))
    indices = np.floor(clipped * grid).astype(int)
    bins: dict[tuple[int, int, int], list[int]] = {}
    for row, key in enumerate(map(tuple, indices)):
        bins.setdefault(key, []).append(row)

    lo, hi = bounds
    span = hi - lo
    ranked = sorted(bins.items(), key=lambda item: len(item[1]), reverse=True)
    records: list[dict[str, object]] = []
    for key, rows in ranked[:limit]:
        row_idx = np.asarray(rows, dtype=int)
        local_norm = norm[row_idx]
        local_centers = centers[row_idx]
        local_min = np.clip(local_norm.min(axis=0) - padding_frac, 0.0, 1.0)
        local_max = np.clip(local_norm.max(axis=0) + padding_frac, 0.0, 1.0)
        box_min = lo + span * local_min
        box_max = lo + span * local_max
        records.append(
            {
                "voxel_index": list(map(int, key)),
                "count": int(len(rows)),
                "normalized_centroid_mean": local_norm.mean(axis=0).tolist(),
                "centroid_bounds_m": [local_centers.min(axis=0).tolist(), local_centers.max(axis=0).tolist()],
                "suggested_refinement_box_m": [box_min.tolist(), box_max.tolist()],
                "suggested_refinement_box_fractions": [local_min.tolist(), local_max.tolist()],
                "region_label": region_label(local_norm.mean(axis=0)),
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


if __name__ == "__main__":
    main()
