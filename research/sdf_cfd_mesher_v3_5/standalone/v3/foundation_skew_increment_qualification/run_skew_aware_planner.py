#!/usr/bin/env python3
"""Add exact Foundation skewness and a no-catch-up increment guard."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import time

import numpy as np


HERE = Path(__file__).resolve().parent
V3 = HERE.parent
PRIOR = V3 / "foundation_aware_prism_shell_qualification"
SOURCE = V3 / "prism_aware_surface_co_design/naca/C/prism_host"
BASE_PLAN = PRIOR / "naca_plan/FOUNDATION_AWARE_PLAN_A.npz"
LAYERS = 20
HARD_SKEW = 4.0
QUALIFICATION_FACTOR = 1.10
QUAL_SKEW = HARD_SKEW / QUALIFICATION_FACTOR
HARD_WEIGHT = 0.05
QUAL_WEIGHT = 0.055
GUIDANCE_RATIO = 1.20
NOMINAL_R = 1.163359674
H1 = 0.000166527069

sys.path.insert(0, str(PRIOR))
import run_weight_aware_planner as WEIGHT  # noqa: E402
from foundation_geometry import (  # noqa: E402
    layer_geometry,
    layer_subset_geometry,
    unique_manifold_edges,
)

BASE = WEIGHT.BASE
PM = WEIGHT.PM


def distribution(values: np.ndarray) -> dict:
    x = np.asarray(values, dtype=np.float64).reshape(-1)
    q = np.quantile(x, [0.01, 0.05, 0.5, 0.95, 0.99])
    return {
        "count": int(len(x)), "minimum": float(x.min()),
        "p01": float(q[0]), "p05": float(q[1]), "median": float(q[2]),
        "p95": float(q[3]), "p99": float(q[4]), "maximum": float(x.max()),
    }


def hash_array(values: np.ndarray) -> str:
    x = np.ascontiguousarray(values)
    h = hashlib.sha256()
    h.update(str(x.dtype).encode("ascii"))
    h.update(struct.pack("<Q", x.ndim))
    h.update(struct.pack("<" + "Q" * x.ndim, *x.shape))
    h.update(x.tobytes())
    return h.hexdigest()


def complete_face_audit(wall, faces, trajectories, increments, edges, edge_faces, *, accepted_state=None):
    lower = np.zeros(len(wall), dtype=np.float64)
    positions = None if accepted_state is None else accepted_state.positions(increments, wall, faces)
    by_layer = []
    qualification_edges: set[int] = set()
    hard_edges: set[int] = set()
    all_skew = []
    all_weight = []
    first_qualification = LAYERS + 1
    for layer in range(LAYERS):
        upper = lower + increments[:, layer]
        geometry = layer_geometry(
            wall, faces, trajectories, lower, upper, edges, edge_faces
        )
        if positions is not None:
            lo, hi = positions[layer:layer+2]
            geometry = layer_geometry(lo, faces, hi-lo, np.zeros(len(wall)), np.ones(len(wall)), edges, edge_faces)
        skew = geometry["side_skewness"]
        weight = geometry["side_weights"]
        hard = skew > HARD_SKEW
        qualification = skew > QUAL_SKEW
        hard_edges.update(map(int, np.flatnonzero(hard)))
        qualification_edges.update(map(int, np.flatnonzero(qualification)))
        if np.any(qualification):
            first_qualification = min(first_qualification, layer + 1)
        all_skew.append(skew); all_weight.append(weight)
        by_layer.append({
            "layer": layer + 1,
            "skewness": distribution(skew),
            "hard_skew_failures": int(np.count_nonzero(hard)),
            "qualification_skew_failures": int(np.count_nonzero(qualification)),
            "minimum_weight": float(weight.min()),
            "hard_weight_failures": int(np.count_nonzero(weight < HARD_WEIGHT)),
            "qualification_weight_failures": int(np.count_nonzero(weight < QUAL_WEIGHT)),
        })
        lower = upper
    return {
        "by_layer": by_layer,
        "skewness": distribution(np.concatenate(all_skew)),
        "weight": distribution(np.concatenate(all_weight)),
        "hard_skew_failures": sum(row["hard_skew_failures"] for row in by_layer),
        "qualification_skew_failures": sum(row["qualification_skew_failures"] for row in by_layer),
        "hard_weight_failures": sum(row["hard_weight_failures"] for row in by_layer),
        "qualification_weight_failures": sum(row["qualification_weight_failures"] for row in by_layer),
        "qualification_edges": np.asarray(sorted(qualification_edges), dtype=np.int64),
        "hard_edges": np.asarray(sorted(hard_edges), dtype=np.int64),
        "first_qualification_layer": first_qualification,
    }


def local_audit(wall, faces, trajectories, increments, edges, edge_faces,
                required_faces, affected_edges, *, accepted_state=None):
    lower = np.zeros(len(wall), dtype=np.float64)
    positions = None if accepted_state is None else accepted_state.positions(increments, wall, faces)
    minimum_weight = 1.0
    maximum_skew = 0.0
    weight_failures = skew_failures = wedge_failures = 0
    for layer in range(LAYERS):
        upper = lower + increments[:, layer]
        geometry = layer_subset_geometry(
            wall, faces, trajectories, lower, upper,
            required_faces, affected_edges, edges, edge_faces,
        )
        if positions is not None:
            lo, hi = positions[layer:layer+2]
            geometry = layer_subset_geometry(lo, faces, hi-lo, np.zeros(len(wall)), np.ones(len(wall)), required_faces, affected_edges, edges, edge_faces)
        weight = geometry["side_weights"]
        skew = geometry["side_skewness"]
        minimum_weight = min(minimum_weight, float(weight.min()))
        maximum_skew = max(maximum_skew, float(skew.max()))
        weight_failures += int(np.count_nonzero(weight < QUAL_WEIGHT))
        skew_failures += int(np.count_nonzero(skew > QUAL_SKEW))
        incidence = faces[required_faces]
        lo = wall + lower[:, None] * trajectories
        hi = wall + upper[:, None] * trajectories
        if positions is not None:
            lo, hi = positions[layer:layer+2]
        prism = np.concatenate((lo[incidence], hi[incidence]), axis=1)
        jacobian, volume = PM._exact_wedge_jacobian_and_volume(prism)
        determinant = PM._foundation_determinant(
            prism, lower_cap_is_uncoupled_wall=(layer == 0)
        )
        if accepted_state is not None:
            determinant = accepted_state.determinant(prism, layer, PM)
        orientation = np.einsum(
            "ij,ij->i",
            np.cross(prism[:, 4] - prism[:, 3], prism[:, 5] - prism[:, 3]),
            np.cross(prism[:, 1] - prism[:, 0], prism[:, 2] - prism[:, 0]),
        )
        wedge_failures += int(np.count_nonzero(
            (determinant < BASE.FOUNDATION_QUAL) | (jacobian <= 0.0)
            | (volume <= 0.0) | (orientation <= 0.0)
        ))
        lower = upper
    return {
        "pass": weight_failures == 0 and skew_failures == 0 and wedge_failures == 0,
        "weight_failures": weight_failures,
        "skew_failures": skew_failures,
        "wedge_failures": wedge_failures,
        "minimum_weight": minimum_weight,
        "maximum_skewness": maximum_skew,
    }


def apply_field(base, distance, radius, scale, start_layer):
    factor = WEIGHT.factor_field(distance, radius, scale)
    proposal = base.copy()
    proposal[:, start_layer - 1:] = np.maximum(
        proposal[:, start_layer - 1:] * factor[:, None], H1
    )
    return proposal


def select_field(wall, faces, trajectories, base, edges, edge_faces,
                 controller_edges, neighbors, vertex_area, *, accepted_state=None,
                 start_layers=None):
    controller_faces = np.unique(edge_faces[controller_edges])
    seeds = set(map(int, faces[controller_faces].ravel()))
    candidates = []
    scale_grid = np.asarray((0.0, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0))
    # The first qualification warning is at layer 19.  Candidate starts are
    # derived from its bounded nine-layer look-ahead horizon, not from a face
    # identity or coordinate region.
    # Omitted input retains the historical qualified search exactly. The V3.5
    # caller supplies its explicit finite horizon and measured warning window.
    starts = range(10, 20) if start_layers is None else tuple(start_layers)
    if any(type(k) is not int or not 2 <= k <= LAYERS for k in starts):
        raise ValueError('InvalidSkewCompressionStartLayers')
    for start_layer in starts:
        for radius in range(1, 9):
            distance = BASE.graph_distances(seeds, neighbors, radius)
            changed = np.flatnonzero(distance < radius)
            required_faces, affected_edges = WEIGHT.affected_neighborhood(
                changed, faces, edge_faces
            )
            valid = []
            audits = []
            for scale in scale_grid:
                proposal = apply_field(base, distance, radius, float(scale), start_layer)
                audit = local_audit(
                    wall, faces, trajectories, proposal, edges, edge_faces,
                    required_faces, affected_edges, accepted_state=accepted_state,
                )
                valid.append(audit["pass"]); audits.append(audit)
            components = BASE.passing_scale_components(np.asarray(valid), scale_grid)
            if not components:
                continue
            begin, end = max(components, key=lambda pair: scale_grid[pair[1]])
            lower_scale = float(scale_grid[begin])
            upper_scale = float(scale_grid[end])
            if end + 1 < len(scale_grid):
                lo, hi = upper_scale, float(scale_grid[end + 1])
                for _ in range(20):
                    midpoint = 0.5 * (lo + hi)
                    proposal = apply_field(base, distance, radius, midpoint, start_layer)
                    audit = local_audit(
                        wall, faces, trajectories, proposal, edges, edge_faces,
                        required_faces, affected_edges, accepted_state=accepted_state,
                    )
                    if audit["pass"]:
                        lo = midpoint
                    else:
                        hi = midpoint
                upper_scale = lo
            selected_scale = upper_scale - BASE.INTERIOR_FRACTION * (
                upper_scale - lower_scale
            )
            proposal = apply_field(
                base, distance, radius, selected_scale, start_layer
            )
            audit = local_audit(
                wall, faces, trajectories, proposal, edges, edge_faces,
                required_faces, affected_edges, accepted_state=accepted_state,
            )
            if not audit["pass"]:
                continue
            total = proposal.sum(axis=1)
            candidates.append({
                "start_layer": start_layer,
                "radius_rings": radius,
                "scale_lower_boundary": lower_scale,
                "scale_upper_boundary": upper_scale,
                "selected_scale": selected_scale,
                "changed_vertices": changed,
                "affected_faces": required_faces,
                "affected_edges": affected_edges,
                "minimum_local_weight": audit["minimum_weight"],
                "maximum_local_skewness": audit["maximum_skewness"],
                "retained_area_height_integral": float(np.dot(vertex_area, total)),
                "proposal": proposal,
            })
    if not candidates:
        raise RuntimeError("PrismSkewnessPlanningFailure")
    selected = max(candidates, key=lambda row: (
        row["retained_area_height_integral"],
        -len(row["changed_vertices"]), -row["radius_rings"], row["start_layer"],
    ))
    selected["candidate_count"] = len(candidates)
    return selected


def increment_guard_audit(increments, wall, faces, trajectories, *, accepted_state=None):
    ratio = increments[:, 1:] / increments[:, :-1]
    above = np.argwhere(ratio > GUIDANCE_RATIO + 1.0e-14)
    clipped = increments.copy()
    clipped[:, 1:] = np.minimum(clipped[:, 1:], GUIDANCE_RATIO * clipped[:, :-1])
    clipped_wedge = BASE.audit_faces(wall, faces, trajectories, clipped, accepted_state=accepted_state)
    return {
        "observed_adjacent_ratio": distribution(ratio),
        "soft_guidance_ratio": GUIDANCE_RATIO,
        "vertices_layers_above_guidance": int(len(above)),
        "first_above_guidance": [
            {"persistent_vertex": int(v), "physical_layer": int(k + 2), "ratio": float(ratio[v, k])}
            for v, k in above[:16]
        ],
        "blind_global_clip_is_valid": bool(clipped_wedge["pass"]),
        "blind_global_clip_layer_failures": [
            row for row in clipped_wedge["by_layer"]
            if row["foundation_qualification_failures"]
        ],
        "frozen_hard_ratio": None,
        "policy": (
            "No target-debt catch-up. The desired increment never exceeds the frozen nominal schedule. "
            "A 1.20 adjacent-layer ratio is a soft CFD guide. If exact shared-neighborhood conditioning "
            "requires a larger lower bound, the smallest certified feasible increment is allowed. All "
            "increments remain capped by exact quality, SDF normal-map, branch, and clearance envelopes."
        ),
    }


def run(output: Path, replay: str) -> int:
    output.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    wall = np.load(SOURCE / "wall_points.npy")
    faces = np.load(SOURCE / "wall_faces.npy")
    trajectories = np.load(SOURCE / "trajectory.npy")
    base = np.load(BASE_PLAN)["increments"]
    edges, edge_faces = unique_manifold_edges(faces)
    _, neighbors = BASE.build_incidence(faces, len(wall))
    p = wall[faces]
    face_area = 0.5 * np.linalg.norm(
        np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1
    )
    vertex_area = np.zeros(len(wall), dtype=np.float64)
    for corner in range(3):
        np.add.at(vertex_area, faces[:, corner], face_area / 3.0)

    historical = complete_face_audit(
        wall, faces, trajectories, base, edges, edge_faces
    )
    if historical["hard_skew_failures"] != 6:
        raise RuntimeError("HistoricalSixSkewReplayMismatch")

    selection_started = time.perf_counter()
    selected = select_field(
        wall, faces, trajectories, base, edges, edge_faces,
        historical["qualification_edges"], neighbors, vertex_area,
    )
    selection_seconds = time.perf_counter() - selection_started
    increments = selected.pop("proposal")

    exact_started = time.perf_counter()
    wedge = BASE.audit_faces(wall, faces, trajectories, increments)
    face_audit = complete_face_audit(
        wall, faces, trajectories, increments, edges, edge_faces
    )
    exact_seconds = time.perf_counter() - exact_started
    if not wedge["pass"]:
        raise RuntimeError("CorrectedSkewPlanWedgeFailure")
    if face_audit["qualification_skew_failures"] or face_audit["qualification_weight_failures"]:
        raise RuntimeError("CorrectedSkewPlanFaceFailure")

    base_total = base.sum(axis=1)
    total = increments.sum(axis=1)
    changed = np.any(np.abs(increments - base) > 1.0e-15, axis=1)
    changed_faces = np.any(changed[faces], axis=1)
    ratio = increments[:, 1:] / increments[:, :-1]
    selected_summary = {
        key: (int(len(value)) if isinstance(value, np.ndarray) else value)
        for key, value in selected.items()
        if key not in ("changed_vertices", "affected_faces", "affected_edges")
    }
    selected_summary.update({
        "changed_vertices": int(len(selected["changed_vertices"])),
        "affected_faces": int(len(selected["affected_faces"])),
        "affected_edges": int(len(selected["affected_edges"])),
    })
    plan = output / f"FOUNDATION_SKEW_AWARE_PLAN_{replay}.npz"
    np.savez(
        plan, increments=increments, cumulative=np.cumsum(increments, axis=1),
        modified_vertices=np.flatnonzero(changed),
        historical_skew_risk_edges=historical["qualification_edges"],
        edges=edges, edge_faces=edge_faces,
    )
    report = {
        "schema": "V3FoundationSkewAwareStagedNacaPlan/v1",
        "replay": replay,
        "policy": {
            "foundation_hard_skewness": HARD_SKEW,
            "foundation_qualification_skewness": QUAL_SKEW,
            "qualification_derivation": "Hard upper bound divided by the frozen 1.10 relative qualification factor.",
            "foundation_hard_weight": HARD_WEIGHT,
            "foundation_qualification_weight": QUAL_WEIGHT,
            "increment_guard": increment_guard_audit(increments, wall, faces, trajectories),
            "candidate_start_layers": [10, 19],
            "candidate_taper_rings": [1, 8],
            "selection": [
                "maximize area-integrated retained thickness",
                "minimize changed persistent vertices",
                "minimize taper radius",
                "prefer later correction start",
            ],
        },
        "historical": {
            "hard_skew_failures": historical["hard_skew_failures"],
            "qualification_skew_failures": historical["qualification_skew_failures"],
            "first_qualification_layer": historical["first_qualification_layer"],
            "by_layer": historical["by_layer"],
        },
        "selection": selected_summary,
        "field_change": {
            "modified_persistent_vertices": int(np.count_nonzero(changed)),
            "modified_wall_faces": int(np.count_nonzero(changed_faces)),
            "modified_wall_area_m2": float(np.sum(face_area[changed_faces])),
            "modified_wall_area_fraction": float(np.sum(face_area[changed_faces]) / np.sum(face_area)),
            "outer_cap_displacement_difference_m": distribution(np.abs(total - base_total)),
            "total_height_m": distribution(total),
            "layer_20_increment_m": distribution(increments[:, 19]),
            "adjacent_layer_growth_ratio": distribution(ratio),
            "fraction_wall_area_reaching_20mm": float(np.sum(face_area[np.all(total[faces] >= 0.020 - 1e-12, axis=1)]) / np.sum(face_area)),
            "fraction_wall_area_reaching_15mm": float(np.sum(face_area[np.all(total[faces] >= 0.015, axis=1)]) / np.sum(face_area)),
            "fraction_wall_area_reaching_10mm": float(np.sum(face_area[np.all(total[faces] >= 0.010, axis=1)]) / np.sum(face_area)),
            "h1_minimum_m": float(increments[:, 0].min()),
            "h1_maximum_m": float(increments[:, 0].max()),
        },
        "complete_prospective": {
            "prisms": len(faces) * LAYERS,
            "wedge_by_layer": wedge["by_layer"],
            "face_by_layer": face_audit["by_layer"],
            "minimum_determinant": min(row["minimum_foundation_determinant"] for row in wedge["by_layer"]),
            "minimum_jacobian": min(row["minimum_exact_jacobian"] for row in wedge["by_layer"]),
            "minimum_volume_m3": min(row["minimum_signed_volume_m3"] for row in wedge["by_layer"]),
            "minimum_weight": face_audit["weight"]["minimum"],
            "maximum_skewness": face_audit["skewness"]["maximum"],
            "skew_hard_failures": face_audit["hard_skew_failures"],
            "skew_qualification_failures": face_audit["qualification_skew_failures"],
            "weight_hard_failures": face_audit["hard_weight_failures"],
            "weight_qualification_failures": face_audit["qualification_weight_failures"],
        },
        "hashes": {
            "increments": hash_array(increments),
            "base_increments": hash_array(base),
        },
        "timing_seconds": {
            "shared_front_solve": selection_seconds,
            "complete_prospective_audit": exact_seconds,
            "total": time.perf_counter() - started,
        },
    }
    report["pass"] = (
        report["complete_prospective"]["skew_qualification_failures"] == 0
        and report["complete_prospective"]["weight_qualification_failures"] == 0
        and wedge["pass"]
        and report["field_change"]["h1_minimum_m"] == H1
        and report["field_change"]["h1_maximum_m"] == H1
    )
    (output / f"FOUNDATION_SKEW_AWARE_PLAN_{replay}.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    canonical = dict(report); canonical.pop("replay"); canonical.pop("timing_seconds")
    (output / f"FOUNDATION_SKEW_AWARE_PLAN_{replay}_CANONICAL.json").write_text(
        json.dumps(canonical, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "pass": report["pass"], "replay": replay,
        "changed_vertices": report["field_change"]["modified_persistent_vertices"],
        "maximum_skewness": report["complete_prospective"]["maximum_skewness"],
        "minimum_weight": report["complete_prospective"]["minimum_weight"],
        "increments_sha256": report["hashes"]["increments"],
    }, sort_keys=True))
    return 0 if report["pass"] else 3


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=HERE / "naca_plan")
    parser.add_argument("--replay", choices=("A", "A2"), required=True)
    args = parser.parse_args()
    return run(args.output, args.replay)


if __name__ == "__main__":
    raise SystemExit(main())
