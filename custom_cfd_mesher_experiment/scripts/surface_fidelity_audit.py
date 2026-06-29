from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
import trimesh.proximity


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-stl", type=Path, required=True)
    parser.add_argument("--candidate-stl", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=0.001)
    parser.add_argument("--samples", type=int, default=25000)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    report = audit_surface_fidelity(
        args.reference_stl,
        args.candidate_stl,
        scale=args.scale,
        samples=args.samples,
        seed=args.seed,
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def audit_surface_fidelity(
    reference_stl: Path,
    candidate_stl: Path,
    *,
    scale: float,
    samples: int,
    seed: int,
) -> dict[str, object]:
    reference = load_mesh(reference_stl)
    candidate = load_mesh(candidate_stl)
    reference.vertices *= scale

    ref_points, _ = trimesh.sample.sample_surface(reference, samples, seed=seed)
    cand_points, _ = trimesh.sample.sample_surface(candidate, samples, seed=seed + 1)

    ref_to_cand = nearest_distances(candidate, ref_points)
    cand_to_ref = nearest_distances(reference, cand_points)
    bidirectional = np.concatenate([ref_to_cand, cand_to_ref])

    return {
        "reference_stl": str(reference_stl),
        "candidate_stl": str(candidate_stl),
        "scale": scale,
        "samples_per_direction": samples,
        "reference_faces": int(len(reference.faces)),
        "candidate_faces": int(len(candidate.faces)),
        "reference_watertight": bool(reference.is_watertight),
        "candidate_watertight": bool(candidate.is_watertight),
        "reference_area_m2": float(reference.area),
        "candidate_area_m2": float(candidate.area),
        "area_ratio_candidate_over_reference": float(candidate.area / reference.area) if reference.area else None,
        "reference_bounds_m": reference.bounds.tolist(),
        "candidate_bounds_m": candidate.bounds.tolist(),
        "reference_to_candidate_mm": summarize_mm(ref_to_cand),
        "candidate_to_reference_mm": summarize_mm(cand_to_ref),
        "bidirectional_mm": summarize_mm(bidirectional),
    }


def load_mesh(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected a single triangle mesh: {path}")
    mesh = mesh.copy()
    mesh.merge_vertices(digits_vertex=9)
    return mesh


def nearest_distances(mesh: trimesh.Trimesh, points: np.ndarray) -> np.ndarray:
    closest, _, _ = trimesh.proximity.closest_point(mesh, points)
    return np.linalg.norm(points - closest, axis=1)


def summarize_mm(values_m: np.ndarray) -> dict[str, float]:
    values = values_m * 1000.0
    return {
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


if __name__ == "__main__":
    main()
