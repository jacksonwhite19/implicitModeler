#!/usr/bin/env python3
"""Extend the staged NACA plan with exact Foundation side-face authority."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import time

import numpy as np

from foundation_geometry import (
    face_area_centre_tri,
    layer_geometry,
    layer_subset_geometry,
    unique_manifold_edges,
)


HERE = Path(__file__).resolve().parent
V3 = HERE.parent
SOURCE = V3 / "prism_aware_surface_co_design/naca/C"
WALL_SOURCE = SOURCE / "prism_host"
HISTORICAL_PLANNER = V3 / "sdf_guided_advancing_front_qualification/run_naca_staged_planner.py"
HARD_WEIGHT = 0.05
# Foundation's determinant qualification adds ten percent to the hard limit
# (0.001 -> 0.0011).  The same frozen relative margin is applied to weights.
QUAL_WEIGHT = 0.055
PROXY_RISK_WEIGHT = 0.08
LAYERS = 20


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


BASE = load_module("weight_base_staged_planner", HISTORICAL_PLANNER)
PM = BASE.PM


def hash_array(values: np.ndarray) -> str:
    values = np.ascontiguousarray(values)
    digest = hashlib.sha256()
    digest.update(str(values.dtype).encode("ascii"))
    digest.update(struct.pack("<Q", values.ndim))
    digest.update(struct.pack("<" + "Q" * values.ndim, *values.shape))
    digest.update(values.tobytes())
    return digest.hexdigest()


def distribution(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    q = np.quantile(values, [0.01, 0.05, 0.5, 0.95, 0.99])
    return {
        "count": int(len(values)), "minimum": float(values.min()),
        "p01": float(q[0]), "p05": float(q[1]), "median": float(q[2]),
        "p95": float(q[3]), "p99": float(q[4]), "maximum": float(values.max()),
    }


def cheap_weights(wall, faces, trajectories, lower, upper, edges, edge_faces):
    lo = wall + lower[:, None] * trajectories
    hi = wall + upper[:, None] * trajectories
    centres = (np.sum(lo[faces], axis=1) + np.sum(hi[faces], axis=1)) / 6.0
    side = np.stack((lo[edges[:, 0]], lo[edges[:, 1]], hi[edges[:, 1]], hi[edges[:, 0]]), axis=1)
    centre = np.mean(side, axis=1)
    area = np.cross(side[:, 2] - side[:, 0], side[:, 3] - side[:, 1])
    da = np.abs(np.einsum("ij,ij->i", area, centre - centres[edge_faces[:, 0]]))
    db = np.abs(np.einsum("ij,ij->i", area, centres[edge_faces[:, 1]] - centre))
    return np.minimum(da, db) / np.maximum(da + db, np.finfo(float).tiny)


def complete_weight_audit(wall, faces, trajectories, increments, edges, edge_faces, *, accepted_state=None):
    lower = np.zeros(len(wall), dtype=np.float64)
    positions = None if accepted_state is None else accepted_state.positions(increments, wall, faces)
    previous_centres = None
    by_layer = []
    side_all = []
    horizontal_all = []
    proxy_false_safe = []
    exact_failure_edges: set[int] = set()
    exact_qualification_edges: set[int] = set()
    first_qualification_layer = LAYERS + 1
    cell_centres_by_layer = []
    for layer in range(LAYERS):
        upper = lower + increments[:, layer]
        result = layer_geometry(wall, faces, trajectories, lower, upper, edges, edge_faces)
        if positions is not None:
            lo, hi = positions[layer:layer+2]
            result = layer_geometry(lo, faces, hi-lo, np.zeros(len(wall)), np.ones(len(wall)), edges, edge_faces)
        exact = result["side_weights"]
        proxy = cheap_weights(wall, faces, trajectories, lower, upper, edges, edge_faces)
        if positions is not None:
            proxy = cheap_weights(lo, faces, hi-lo, np.zeros(len(wall)), np.ones(len(wall)), edges, edge_faces)
        risk = proxy < PROXY_RISK_WEIGHT
        false_safe = np.flatnonzero((exact < QUAL_WEIGHT) & ~risk)
        if len(false_safe):
            proxy_false_safe.extend((layer + 1, int(edge)) for edge in false_safe)
        hard = exact < HARD_WEIGHT
        qualification = exact < QUAL_WEIGHT
        exact_failure_edges.update(map(int, np.flatnonzero(hard)))
        exact_qualification_edges.update(map(int, np.flatnonzero(qualification)))
        if np.any(qualification):
            first_qualification_layer = min(first_qualification_layer, layer + 1)

        horizontal = np.empty(0, dtype=np.float64)
        if previous_centres is not None:
            front = wall + lower[:, None] * trajectories
            if positions is not None:
                front = positions[layer]
            area, centre = face_area_centre_tri(front[faces])
            da = np.abs(np.einsum("ij,ij->i", area, centre - previous_centres))
            db = np.abs(np.einsum("ij,ij->i", area, result["centres"] - centre))
            horizontal = np.minimum(da, db) / np.maximum(
                da + db, np.finfo(float).tiny
            )
            horizontal_all.append(horizontal)
        side_all.append(exact)
        cell_centres_by_layer.append(result["centres"])
        by_layer.append({
            "layer": layer + 1,
            "side_weight": distribution(exact),
            "side_hard_failures": int(hard.sum()),
            "side_qualification_failures": int(qualification.sum()),
            "horizontal_weight": distribution(horizontal) if len(horizontal) else None,
            "horizontal_hard_failures": int(np.count_nonzero(horizontal < HARD_WEIGHT)),
            "horizontal_qualification_failures": int(np.count_nonzero(horizontal < QUAL_WEIGHT)),
            "cheap_risk_edges": int(np.count_nonzero(risk)),
            "cheap_false_safe_edges": int(len(false_safe)),
        })
        previous_centres = result["centres"]
        lower = upper
    return {
        "by_layer": by_layer,
        "side_weight": distribution(np.concatenate(side_all)),
        "horizontal_weight": distribution(np.concatenate(horizontal_all)),
        "side_hard_failures": int(sum(row["side_hard_failures"] for row in by_layer)),
        "side_qualification_failures": int(sum(row["side_qualification_failures"] for row in by_layer)),
        "horizontal_hard_failures": int(sum(row["horizontal_hard_failures"] for row in by_layer)),
        "horizontal_qualification_failures": int(sum(row["horizontal_qualification_failures"] for row in by_layer)),
        "qualification_edges": np.asarray(sorted(exact_qualification_edges), dtype=np.int64),
        "hard_failure_edges": np.asarray(sorted(exact_failure_edges), dtype=np.int64),
        "first_qualification_layer": first_qualification_layer,
        "cheap_false_safe": proxy_false_safe,
        "cell_centres": np.stack(cell_centres_by_layer, axis=0),
    }


def affected_neighborhood(changed, faces, edge_faces):
    changed_mask = np.zeros(int(faces.max()) + 1, dtype=bool)
    changed_mask[np.asarray(changed, dtype=np.int64)] = True
    face_mask = np.any(changed_mask[faces], axis=1)
    affected_edges = np.flatnonzero(np.any(face_mask[edge_faces], axis=1))
    required_faces = np.unique(edge_faces[affected_edges])
    return required_faces, affected_edges


class _CandidateLayerCache:
    """Exact layer-summary reuse, scoped to one invocation of the selector.

    Content keys include a conservative superset of every evaluated dependency.
    A changed guard, ownership state or predicate implementation cannot reuse a
    previous result. No candidate is removed and no predicate is approximated.
    """

    def __init__(self):
        self.values = {}
        self.last_access = []

    def begin(self, wall, faces, trajectories, edges, edge_faces,
              required_faces, affected_edges, accepted_state):
        self.context = tuple(hash_array(a) for a in (
            wall, faces, trajectories, edges, edge_faces,
            required_faces, affected_edges,
        )) + (
            None if accepted_state is None else accepted_state.ownership,
            None if accepted_state is None else type(accepted_state),
            None if accepted_state is None else accepted_state.determinant.__func__,
            PM._foundation_determinant, PM._exact_wedge_jacobian_and_volume,
            layer_subset_geometry, BASE.FOUNDATION_QUAL, QUAL_WEIGHT, LAYERS,
        )
        self.last_access = []

    def lookup(self, layer, lower, upper):
        key = (self.context, layer, hash_array(lower), hash_array(upper))
        hit = key in self.values
        self.last_access.append({"key": key, "layer": layer, "hit": hit})
        return key, self.values.get(key)


def audit_candidate(wall, faces, trajectories, increments, edges, edge_faces,
                    required_faces, affected_edges, *, accepted_state=None,
                    _layer_cache=None):
    lower = np.zeros(len(wall), dtype=np.float64)
    positions = None if accepted_state is None else accepted_state.positions(increments, wall, faces)
    if _layer_cache is not None:
        _layer_cache.begin(wall, faces, trajectories, edges, edge_faces,
                           required_faces, affected_edges, accepted_state)
    minimum_weight = 1.0
    weight_failures = wedge_failures = 0
    for layer in range(LAYERS):
        upper = lower + increments[:, layer]
        if _layer_cache is not None:
            key, cached = _layer_cache.lookup(
                layer, lower if positions is None else positions[layer],
                upper if positions is None else positions[layer + 1],
            )
            if cached is not None:
                layer_wedge_failures, layer_weight_failures, layer_minimum = cached
                wedge_failures += layer_wedge_failures
                weight_failures += layer_weight_failures
                minimum_weight = min(minimum_weight, layer_minimum)
                lower = upper
                continue
        result = layer_subset_geometry(
            wall, faces, trajectories, lower, upper,
            required_faces, affected_edges, edges, edge_faces,
        )
        incidence = faces[required_faces]
        prism = np.concatenate((
            (wall + lower[:, None] * trajectories)[incidence],
            (wall + upper[:, None] * trajectories)[incidence],
        ), axis=1)
        if positions is not None:
            lo, hi = positions[layer:layer+2]
            result = layer_subset_geometry(lo, faces, hi-lo, np.zeros(len(wall)), np.ones(len(wall)), required_faces, affected_edges, edges, edge_faces)
            prism = np.concatenate((lo[incidence], hi[incidence]), axis=1)
        jacobian, volume = PM._exact_wedge_jacobian_and_volume(prism)
        determinant = PM._foundation_determinant(prism, lower_cap_is_uncoupled_wall=(layer == 0))
        if accepted_state is not None:
            determinant = accepted_state.determinant(prism, layer, PM)
        top, base = prism[:, 3:], prism[:, :3]
        orientation = np.einsum(
            "ij,ij->i",
            np.cross(top[:, 1] - top[:, 0], top[:, 2] - top[:, 0]),
            np.cross(base[:, 1] - base[:, 0], base[:, 2] - base[:, 0]),
        )
        layer_wedge_failures = int(np.count_nonzero(
            (determinant < BASE.FOUNDATION_QUAL) | (jacobian <= 0.0)
            | (volume <= 0.0) | (orientation <= 0.0)
        ))
        layer_weight_failures = int(np.count_nonzero(result["side_weights"] < QUAL_WEIGHT))
        layer_minimum = float(result["side_weights"].min())
        wedge_failures += layer_wedge_failures
        weight_failures += layer_weight_failures
        minimum_weight = min(minimum_weight, layer_minimum)
        if _layer_cache is not None:
            _layer_cache.values[key] = (layer_wedge_failures, layer_weight_failures, layer_minimum)
        lower = upper
    return {
        "pass": weight_failures == 0 and wedge_failures == 0,
        "weight_failures": weight_failures,
        "wedge_failures": wedge_failures,
        "minimum_weight": minimum_weight,
    }


def factor_field(distance, radius, scale):
    factor = np.ones(len(distance), dtype=np.float64)
    inside = distance < radius
    factor[inside] = scale + (1.0 - scale) * distance[inside] / radius
    return factor


def select_correction(wall, faces, trajectories, base_increments, edges, edge_faces,
                      qualification_edges, first_failure_layer, incident, neighbors,
                      vertex_area, *, accepted_state=None):
    controller_faces = np.unique(edge_faces[qualification_edges])
    seeds = set(map(int, faces[controller_faces].ravel()))
    start_layer = max(2, first_failure_layer - 2)
    candidates = []
    layer_cache = _CandidateLayerCache()
    frozen_grid = np.asarray([0.0, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.0])
    for radius in range(1, 9):
        distance = BASE.graph_distances(seeds, neighbors, radius)
        changed = np.flatnonzero(distance < radius)
        required_faces, affected_edges = affected_neighborhood(changed, faces, edge_faces)
        valid = []
        for scale in frozen_grid:
            factor = factor_field(distance, radius, float(scale))
            proposal = base_increments.copy()
            proposal[:, start_layer - 1:] = np.maximum(
                proposal[:, start_layer - 1:] * factor[:, None], BASE.H1
            )
            valid.append(audit_candidate(
                wall, faces, trajectories, proposal, edges, edge_faces,
                required_faces, affected_edges, accepted_state=accepted_state,
                _layer_cache=layer_cache,
            )["pass"])
        components = BASE.passing_scale_components(np.asarray(valid), frozen_grid)
        if not components:
            continue
        begin, end = max(components, key=lambda pair: frozen_grid[pair[1]])
        lower_scale = float(frozen_grid[begin]); upper_scale = float(frozen_grid[end])
        if end + 1 < len(frozen_grid):
            lo, hi = upper_scale, float(frozen_grid[end + 1])
            # The selected point is then moved two percent into the certified
            # component.  Eighteen bisections make the bracket error far
            # smaller than that explicit policy margin.
            for _ in range(18):
                midpoint = 0.5 * (lo + hi)
                factor = factor_field(distance, radius, midpoint)
                proposal = base_increments.copy()
                proposal[:, start_layer - 1:] = np.maximum(
                    proposal[:, start_layer - 1:] * factor[:, None], BASE.H1
                )
                if audit_candidate(
                    wall, faces, trajectories, proposal, edges, edge_faces,
                    required_faces, affected_edges, accepted_state=accepted_state,
                    _layer_cache=layer_cache,
                )["pass"]:
                    lo = midpoint
                else:
                    hi = midpoint
            upper_scale = lo
        selected_scale = upper_scale - BASE.INTERIOR_FRACTION * (upper_scale - lower_scale)
        factor = factor_field(distance, radius, selected_scale)
        proposal = base_increments.copy()
        proposal[:, start_layer - 1:] = np.maximum(
            proposal[:, start_layer - 1:] * factor[:, None], BASE.H1
        )
        audit = audit_candidate(
            wall, faces, trajectories, proposal, edges, edge_faces,
            required_faces, affected_edges, accepted_state=accepted_state,
            _layer_cache=layer_cache,
        )
        if not audit["pass"]:
            continue
        total = proposal.sum(axis=1)
        candidates.append({
            "start_layer": start_layer, "radius_rings": radius,
            "scale_lower_boundary": lower_scale,
            "scale_upper_boundary": upper_scale,
            "selected_scale": selected_scale,
            "changed_vertices": changed,
            "affected_faces": required_faces,
            "affected_edges": affected_edges,
            "minimum_local_weight": audit["minimum_weight"],
            "retained_area_height_integral": float(np.dot(vertex_area, total)),
            "proposal": proposal,
        })
    if not candidates:
        raise RuntimeError("PrismSideWeightPathInfeasible")
    selected = max(candidates, key=lambda row: (
        row["retained_area_height_integral"],
        -len(row["changed_vertices"]), -row["radius_rings"],
    ))
    selected["candidate_count"] = len(candidates)
    return selected


def mechanism_counterfactual(wall, faces, trajectories, increments, edges, edge_faces, audit):
    rows = []
    lower = np.zeros(len(wall), dtype=np.float64)
    cumulative = np.cumsum(increments, axis=1)
    for layer in range(LAYERS):
        upper = cumulative[:, layer]
        actual = layer_geometry(wall, faces, trajectories, lower, upper, edges, edge_faces)
        failed = np.flatnonzero(actual["side_weights"] < HARD_WEIGHT)
        for edge_id in failed:
            face_ids = edge_faces[edge_id]
            vertices = np.unique(faces[face_ids].ravel())
            common = np.mean(trajectories[vertices], axis=0)
            common /= np.linalg.norm(common)
            parallel = trajectories.copy(); parallel[vertices] = common
            # Evaluate only the copied local controller at the same offsets.
            local_faces = np.asarray(face_ids, dtype=np.int64)
            local_edges = np.asarray([edge_id], dtype=np.int64)
            parallel_result = layer_subset_geometry(
                wall, faces, parallel, lower, upper,
                local_faces, local_edges, edges, edge_faces,
            )["side_weights"][0]
            zero_result = layer_subset_geometry(
                wall, faces, trajectories,
                np.zeros(len(wall)), increments[:, layer],
                local_faces, local_edges, edges, edge_faces,
            )["side_weights"][0]
            rows.append({
                "edge": int(edge_id), "layer": layer + 1,
                "actual_weight": float(actual["side_weights"][edge_id]),
                "parallel_trajectory_weight": float(parallel_result),
                "zero_accumulated_offset_weight": float(zero_result),
                "shared_edge_increment_difference_m": float(abs(
                    increments[edges[edge_id, 0], layer] - increments[edges[edge_id, 1], layer]
                )),
                "shared_edge_cumulative_difference_m": float(abs(
                    upper[edges[edge_id, 0]] - upper[edges[edge_id, 1]]
                )),
                "trajectory_spread_degrees": float(np.degrees(np.arccos(np.clip(
                    np.dot(trajectories[edges[edge_id, 0]], trajectories[edges[edge_id, 1]]), -1, 1
                )))),
            })
        lower = upper
    return {
        "failed_rows": len(rows),
        "actual_weight": distribution([row["actual_weight"] for row in rows]),
        "parallel_trajectory_weight": distribution([row["parallel_trajectory_weight"] for row in rows]),
        "zero_accumulated_offset_weight": distribution([row["zero_accumulated_offset_weight"] for row in rows]),
        "increment_difference_m": distribution([row["shared_edge_increment_difference_m"] for row in rows]),
        "cumulative_difference_m": distribution([row["shared_edge_cumulative_difference_m"] for row in rows]),
        "trajectory_spread_degrees": distribution([row["trajectory_spread_degrees"] for row in rows]),
        "interpretation": "Accumulated trajectory shear amplifies pre-existing adjacent-triangle altitude asymmetry; unequal requested increments are not the controller.",
    }


def run(output: Path, replay: str):
    output.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    # Regenerate the prior qualified staged plan from the frozen wall.  This is
    # not a replay of its materialized shell and uses no historical bad edge id.
    base_dir = output / f"base_{replay}"
    base_result = BASE.run(base_dir, replay)
    base_increments = np.load(base_result["plan"])["increments"]
    wall = np.load(WALL_SOURCE / "wall_points.npy")
    faces = np.load(WALL_SOURCE / "wall_faces.npy")
    trajectories = np.load(WALL_SOURCE / "trajectory.npy")
    edges, edge_faces = unique_manifold_edges(faces)
    incident, neighbors = BASE.build_incidence(faces, len(wall))
    p = wall[faces]
    area = 0.5 * np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1)
    vertex_area = np.zeros(len(wall), dtype=np.float64)
    for corner in range(3):
        np.add.at(vertex_area, faces[:, corner], area / 3.0)

    historical_start = time.perf_counter()
    historical_weights = complete_weight_audit(
        wall, faces, trajectories, base_increments, edges, edge_faces
    )
    historical_seconds = time.perf_counter() - historical_start
    if historical_weights["side_hard_failures"] != 367:
        raise RuntimeError("Historical367FailureReplayMismatch")
    if historical_weights["cheap_false_safe"]:
        raise RuntimeError("WeightPredictorFalseSafeOnHistoricalShell")

    selection_start = time.perf_counter()
    selected = select_correction(
        wall, faces, trajectories, base_increments, edges, edge_faces,
        historical_weights["qualification_edges"],
        historical_weights["first_qualification_layer"],
        incident, neighbors, vertex_area,
    )
    selection_seconds = time.perf_counter() - selection_start
    increments = selected.pop("proposal")

    wedge_start = time.perf_counter()
    wedge = BASE.audit_faces(wall, faces, trajectories, increments)
    weight = complete_weight_audit(wall, faces, trajectories, increments, edges, edge_faces)
    exact_seconds = time.perf_counter() - wedge_start
    if not wedge["pass"]:
        raise RuntimeError("CorrectedPlanWedgeFailure")
    if (weight["side_qualification_failures"] or weight["horizontal_qualification_failures"]
            or weight["cheap_false_safe"]):
        raise RuntimeError("CorrectedPlanWeightFailure")

    total = increments.sum(axis=1)
    historical_total = base_increments.sum(axis=1)
    modified = np.any(np.abs(increments - base_increments) > 1.0e-15, axis=1)
    modified_faces = np.any(modified[faces], axis=1)
    outer_area = area
    report_selection = {
        key: (int(len(value)) if isinstance(value, np.ndarray) else value)
        for key, value in selected.items()
        if key not in ("changed_vertices", "affected_faces", "affected_edges")
    }
    report_selection.update({
        "changed_vertex_count": int(len(selected["changed_vertices"])),
        "affected_face_count": int(len(selected["affected_faces"])),
        "affected_edge_count": int(len(selected["affected_edges"])),
    })
    plan_path = output / f"FOUNDATION_AWARE_PLAN_{replay}.npz"
    np.savez(
        plan_path, increments=increments, cumulative=np.cumsum(increments, axis=1),
        weight_risk_edges=historical_weights["qualification_edges"],
        modified_vertices=np.flatnonzero(modified), edges=edges, edge_faces=edge_faces,
    )
    mechanism = mechanism_counterfactual(
        wall, faces, trajectories, base_increments, edges, edge_faces, historical_weights
    )
    report = {
        "schema": "V3FoundationAwareStagedNacaPlan/v1",
        "replay": replay,
        "policy": {
            "foundation_hard_weight": HARD_WEIGHT,
            "foundation_qualification_weight": QUAL_WEIGHT,
            "qualification_derivation": "Ten percent relative margin, identical to frozen determinant margin policy.",
            "cheap_proxy_risk_weight": PROXY_RISK_WEIGHT,
            "compression_start_rule": "two physical layers before first exact future qualification failure",
            "increment_floor_m": BASE.H1,
            "taper_radius_candidates_rings": [1, 8],
            "scale_grid": [0.0, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.0],
            "selection_order": [
                "maximize area-integrated retained total thickness",
                "minimize changed vertices", "minimize taper radius",
            ],
        },
        "historical_replay": {
            "side_hard_failures": historical_weights["side_hard_failures"],
            "side_qualification_failures": historical_weights["side_qualification_failures"],
            "first_qualification_failure_layer": historical_weights["first_qualification_layer"],
            "by_layer": historical_weights["by_layer"],
            "cheap_false_safe_count": len(historical_weights["cheap_false_safe"]),
        },
        "mechanism": mechanism,
        "selection": report_selection,
        "field_change": {
            "modified_persistent_vertices": int(np.count_nonzero(modified)),
            "modified_wall_faces": int(np.count_nonzero(modified_faces)),
            "modified_wall_area_m2": float(np.sum(area[modified_faces])),
            "modified_wall_area_fraction": float(np.sum(area[modified_faces]) / np.sum(area)),
            "outer_vertex_displacement_difference_m": distribution(np.abs(total - historical_total)),
            "total_height_m": distribution(total),
            "fraction_wall_area_reaching_20mm": float(np.sum(outer_area[np.all(total[faces] >= 0.020 - 1e-12, axis=1)]) / np.sum(outer_area)),
            "fraction_wall_area_reaching_15mm": float(np.sum(outer_area[np.all(total[faces] >= 0.015, axis=1)]) / np.sum(outer_area)),
            "fraction_wall_area_reaching_10mm": float(np.sum(outer_area[np.all(total[faces] >= 0.010, axis=1)]) / np.sum(outer_area)),
            "h1_minimum_m": float(increments[:, 0].min()),
            "h1_maximum_m": float(increments[:, 0].max()),
        },
        "complete_prospective": {
            "prisms": len(faces) * LAYERS,
            "lateral_faces": len(edges) * LAYERS,
            "horizontal_faces": len(faces) * (LAYERS - 1),
            "wedge_by_layer": wedge["by_layer"],
            "side_weight_by_layer": weight["by_layer"],
            "minimum_side_weight": weight["side_weight"]["minimum"],
            "minimum_horizontal_weight": weight["horizontal_weight"]["minimum"],
            "side_hard_failures": weight["side_hard_failures"],
            "side_qualification_failures": weight["side_qualification_failures"],
            "horizontal_hard_failures": weight["horizontal_hard_failures"],
            "horizontal_qualification_failures": weight["horizontal_qualification_failures"],
            "foundation_determinant_failures": int(sum(row["foundation_hard_failures"] for row in wedge["by_layer"])),
            "foundation_determinant_qualification_failures": int(sum(row["foundation_qualification_failures"] for row in wedge["by_layer"])),
            "jacobian_failures": int(sum(row["nonpositive_exact_jacobians"] for row in wedge["by_layer"])),
            "volume_failures": int(sum(row["nonpositive_signed_volumes"] for row in wedge["by_layer"])),
            "orientation_failures": int(sum(row["orientation_failures"] for row in wedge["by_layer"])),
            "cheap_weight_false_safe_count": len(weight["cheap_false_safe"]),
        },
        "hashes": {
            "increments": hash_array(increments), "base_increments": hash_array(base_increments),
            "wall_points": BASE.sha256(WALL_SOURCE / "wall_points.npy"),
            "wall_faces": BASE.sha256(WALL_SOURCE / "wall_faces.npy"),
            "trajectories": BASE.sha256(WALL_SOURCE / "trajectory.npy"),
        },
        "timing_seconds": {
            "historical_weight_replay": historical_seconds,
            "shared_front_solve": selection_seconds,
            "complete_prospective_audit": exact_seconds,
            "total": time.perf_counter() - started,
        },
    }
    report["pass"] = (
        report["complete_prospective"]["side_qualification_failures"] == 0
        and report["complete_prospective"]["horizontal_qualification_failures"] == 0
        and report["complete_prospective"]["foundation_determinant_qualification_failures"] == 0
        and report["complete_prospective"]["jacobian_failures"] == 0
        and report["complete_prospective"]["volume_failures"] == 0
        and report["complete_prospective"]["orientation_failures"] == 0
        and report["complete_prospective"]["cheap_weight_false_safe_count"] == 0
    )
    canonical = dict(report); canonical.pop("replay"); canonical.pop("timing_seconds")
    (output / f"FOUNDATION_AWARE_PLAN_{replay}.json").write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output / f"FOUNDATION_AWARE_PLAN_{replay}_CANONICAL.json").write_text(
        json.dumps(canonical, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "pass": report["pass"], "replay": replay,
        "modified_vertices": report["field_change"]["modified_persistent_vertices"],
        "minimum_side_weight": report["complete_prospective"]["minimum_side_weight"],
        "minimum_horizontal_weight": report["complete_prospective"]["minimum_horizontal_weight"],
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
