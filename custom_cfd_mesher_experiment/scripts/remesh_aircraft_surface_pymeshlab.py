from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pymeshlab as ml
import trimesh
import trimesh.proximity

from cfd_surface_mesher import mesh_report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a cleaner aircraft surface triangulation for CFD meshing."
    )
    parser.add_argument("--input-stl", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--target-edge-mm", type=float, required=True)
    parser.add_argument("--iterations", type=int, default=5)
    parser.add_argument("--feature-deg", type=float, default=25.0)
    parser.add_argument("--max-surface-distance-mm", type=float, default=0.35)
    parser.add_argument("--samples", type=int, default=12000)
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--no-smooth", action="store_true")
    args = parser.parse_args()

    args.output_stl.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)

    original = load_trimesh(args.input_stl)
    initial_report = mesh_report(original.copy())

    meshset = ml.MeshSet()
    meshset.load_new_mesh(str(args.input_stl))
    meshset.meshing_isotropic_explicit_remeshing(
        iterations=args.iterations,
        adaptive=False,
        selectedonly=False,
        targetlen=ml.PureValue(args.target_edge_mm),
        featuredeg=args.feature_deg,
        checksurfdist=True,
        maxsurfdist=ml.PureValue(args.max_surface_distance_mm),
        splitflag=True,
        collapseflag=True,
        swapflag=True,
        smoothflag=not args.no_smooth,
        reprojectflag=True,
    )
    meshset.save_current_mesh(str(args.output_stl), binary=True)

    remeshed = load_trimesh(args.output_stl)
    remeshed.merge_vertices(digits_vertex=8)
    remeshed.remove_unreferenced_vertices()
    remeshed.export(args.output_stl)

    final_report = mesh_report(remeshed.copy())
    fidelity = surface_fidelity(
        reference=original,
        candidate=remeshed,
        samples=args.samples,
        seed=args.seed,
    )
    report: dict[str, Any] = {
        "strategy": "pymeshlab_isotropic_explicit_remeshing",
        "input_stl": str(args.input_stl),
        "output_stl": str(args.output_stl),
        "target_edge_mm": args.target_edge_mm,
        "iterations": args.iterations,
        "feature_deg": args.feature_deg,
        "max_surface_distance_mm": args.max_surface_distance_mm,
        "smooth": not args.no_smooth,
        "initial": initial_report,
        "final": final_report,
        "surface_fidelity_mm": fidelity,
    }
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def load_trimesh(path: Path) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh: {path}")
    mesh = mesh.copy()
    mesh.merge_vertices(digits_vertex=8)
    mesh.remove_unreferenced_vertices()
    return mesh


def surface_fidelity(
    *,
    reference: trimesh.Trimesh,
    candidate: trimesh.Trimesh,
    samples: int,
    seed: int,
) -> dict[str, Any]:
    ref_points, _ = trimesh.sample.sample_surface(reference, samples, seed=seed)
    cand_points, _ = trimesh.sample.sample_surface(candidate, samples, seed=seed + 1)
    ref_to_cand = nearest_distances(candidate, ref_points)
    cand_to_ref = nearest_distances(reference, cand_points)
    bidirectional = np.concatenate([ref_to_cand, cand_to_ref])
    return {
        "samples_per_direction": samples,
        "reference_to_candidate": summarize(ref_to_cand),
        "candidate_to_reference": summarize(cand_to_ref),
        "bidirectional": summarize(bidirectional),
        "area_ratio_candidate_over_reference": (
            float(candidate.area / reference.area) if reference.area else None
        ),
        "reference_bounds_mm": reference.bounds.tolist(),
        "candidate_bounds_mm": candidate.bounds.tolist(),
    }


def nearest_distances(mesh: trimesh.Trimesh, points: np.ndarray) -> np.ndarray:
    closest, _, _ = trimesh.proximity.closest_point(mesh, points)
    return np.linalg.norm(points - closest, axis=1)


def summarize(values: np.ndarray) -> dict[str, float]:
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
