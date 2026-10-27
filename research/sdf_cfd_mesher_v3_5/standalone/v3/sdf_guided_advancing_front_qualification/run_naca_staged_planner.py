#!/usr/bin/env python3
"""Authoritative persistent-key staged planner for the frozen reduced NACA wall.

The script never creates layer topology.  It proposes one shared displacement
per current-front vertex, validates the exact unequal-height wedges, and writes
the immutable twenty-stage plan consumed later by the stock Foundation kernel.
No transient face descriptor is an input to controller discovery or policy.
"""

from __future__ import annotations

import argparse
import csv
from collections import deque
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import time

import numpy as np


HERE = Path(__file__).resolve().parent
V3 = HERE.parent
SOURCE = V3 / "prism_aware_surface_co_design" / "naca" / "C"
PRISM_SOURCE = SOURCE / "prism_host"

H1 = 0.000166527069
LAYERS = 20
R_NOMINAL = 1.163359674
FOUNDATION_HARD = 0.001
FOUNDATION_QUAL = 0.0011
RISK_CONDITION_LIMIT = 0.25
RISK_TRAJECTORY_SPREAD_DEG = 5.0
INTERIOR_FRACTION = 0.02
MAX_TAPER_RINGS = 20


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


PM = load_module(
    "v3_staged_prism_math",
    V3 / "prism_feasibility_contract" / "prospective_commit_adapter.py",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def hash_array(values: np.ndarray) -> str:
    values = np.ascontiguousarray(values)
    digest = hashlib.sha256()
    digest.update(str(values.dtype).encode("ascii"))
    digest.update(struct.pack("<Q", values.ndim))
    digest.update(struct.pack("<" + "Q" * values.ndim, *values.shape))
    digest.update(values.tobytes())
    return digest.hexdigest()


def json_scalar(value):
    """Convert only NumPy scalar wrappers; arrays must be made explicit."""
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def nominal_increments() -> np.ndarray:
    return H1 * np.power(R_NOMINAL, np.arange(LAYERS, dtype=np.float64))


def load_tsv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream, delimiter="\t"))


def face_key(face_id: int, faces: np.ndarray, face_rows: list[dict[str, str]],
             vertex_rows: list[dict[str, str]]) -> dict:
    incidence = [int(value) for value in faces[face_id]]
    curve_union = sorted({
        curve
        for vertex in incidence
        for curve in vertex_rows[vertex]["curves"].split(",")
        if curve
    })
    corner_union = sorted({
        int(vertex_rows[vertex]["corner"])
        for vertex in incidence
        if int(vertex_rows[vertex]["corner"]) != 0
    })
    return {
        "persistent_sheet": face_rows[face_id]["persistent_sheet"],
        "sorted_persistent_vertex_ids": sorted(incidence),
        "oriented_persistent_vertex_ids": incidence,
        "persistent_curves": curve_union,
        "persistent_corners": corner_union,
    }


def key_string(key: dict) -> str:
    return json.dumps(key, sort_keys=True, separators=(",", ":"))


def build_incidence(faces: np.ndarray, vertex_count: int):
    incident: list[list[int]] = [[] for _ in range(vertex_count)]
    neighbors: list[set[int]] = [set() for _ in range(vertex_count)]
    for face_id, face in enumerate(faces):
        a, b, c = (int(value) for value in face)
        incident[a].append(face_id); incident[b].append(face_id); incident[c].append(face_id)
        neighbors[a].update((b, c)); neighbors[b].update((a, c)); neighbors[c].update((a, b))
    return incident, [tuple(sorted(row)) for row in neighbors]


def connected_face_components(selected: np.ndarray, faces: np.ndarray,
                              incident: list[list[int]]) -> list[list[int]]:
    selected_set = {int(value) for value in selected}
    components: list[list[int]] = []
    while selected_set:
        root = min(selected_set)
        selected_set.remove(root)
        queue = [root]
        component = []
        while queue:
            face_id = queue.pop()
            component.append(face_id)
            for vertex in faces[face_id]:
                for neighbor_face in incident[int(vertex)]:
                    if neighbor_face in selected_set:
                        selected_set.remove(neighbor_face)
                        queue.append(neighbor_face)
        components.append(sorted(component))
    return components


def graph_distances(seeds: set[int], neighbors: list[tuple[int, ...]],
                    maximum: int) -> np.ndarray:
    distance = np.full(len(neighbors), maximum + 1, dtype=np.int16)
    queue: deque[int] = deque()
    for vertex in sorted(seeds):
        distance[vertex] = 0
        queue.append(vertex)
    while queue:
        vertex = queue.popleft()
        if distance[vertex] >= maximum:
            continue
        for adjacent in neighbors[vertex]:
            if distance[adjacent] > distance[vertex] + 1:
                distance[adjacent] = distance[vertex] + 1
                queue.append(adjacent)
    return distance


def face_geometry(points: np.ndarray, faces: np.ndarray, trajectories: np.ndarray) -> dict:
    p0, p1, p2 = points[faces[:, 0]], points[faces[:, 1]], points[faces[:, 2]]
    e1, e2 = p1 - p0, p2 - p0
    d1 = trajectories[faces[:, 1]] - trajectories[faces[:, 0]]
    d2 = trajectories[faces[:, 2]] - trajectories[faces[:, 0]]
    base_matrix = np.stack((e1, e2), axis=2)
    direction_matrix = np.stack((d1, d2), axis=2)
    base_singular = np.linalg.svd(base_matrix, compute_uv=False)
    direction_singular = np.linalg.svd(direction_matrix, compute_uv=False)
    area2 = np.linalg.norm(np.cross(e1, e2), axis=1)
    edge = np.stack((
        np.linalg.norm(e1, axis=1),
        np.linalg.norm(p2 - p1, axis=1),
        np.linalg.norm(e2, axis=1),
    ), axis=1)
    spreads = []
    for first, second in ((0, 1), (1, 2), (2, 0)):
        dot = np.einsum("ij,ij->i", trajectories[faces[:, first]], trajectories[faces[:, second]])
        spreads.append(np.degrees(np.arccos(np.clip(dot, -1.0, 1.0))))
    return {
        "area": 0.5 * area2,
        "edge_min": edge.min(axis=1),
        "edge_max": edge.max(axis=1),
        "base_sigma_min": base_singular[:, 1],
        "direction_sigma_max": direction_singular[:, 0],
        "trajectory_spread_deg": np.max(np.stack(spreads, axis=1), axis=1),
    }


def naca_curvature_and_clearance(points: np.ndarray, faces: np.ndarray,
                                 face_rows: list[dict[str, str]]) -> tuple[np.ndarray, np.ndarray]:
    """Cheap source-authority queries used only for risk screening."""
    center = points[faces].mean(axis=1)
    x = np.clip(center[:, 0], 1.0e-12, 1.0)
    d1 = 0.6 * (0.2969 / (2.0 * np.sqrt(x)) - 0.1260 - 0.7032 * x
                + 0.8529 * x * x - 0.4060 * x ** 3)
    d2 = 0.6 * (-0.2969 / (4.0 * x ** 1.5) - 0.7032
                + 1.7058 * x - 1.2180 * x * x)
    curvature = np.abs(d2) / np.power(1.0 + d1 * d1, 1.5)
    # Branch distance is a conservative local semantic scale, not an extrusion
    # authorization.  Exact branch progression is certified later.
    thickness = 0.6 * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x * x
                       + 0.2843 * x ** 3 - 0.1015 * x ** 4)
    branch_scale = np.minimum.reduce((
        np.maximum(2.0 * thickness, np.finfo(float).tiny),
        np.full(len(faces), 0.5),
        np.maximum(1.0 - x, np.finfo(float).tiny),
    ))
    for face_id, row in enumerate(face_rows):
        if row["persistent_sheet"] != "airfoil.skin":
            curvature[face_id] = 0.0
            branch_scale[face_id] = min(0.5, max(1.0e-12, 1.0 - center[face_id, 0]))
    return curvature, branch_scale


def cheap_risk_screen(points: np.ndarray, faces: np.ndarray, trajectories: np.ndarray,
                      face_rows: list[dict[str, str]], vertex_rows: list[dict[str, str]],
                      geometry: dict) -> dict:
    desired = nominal_increments()
    cumulative = np.cumsum(desired)
    condition_by_layer = (
        cumulative[:, None] * geometry["direction_sigma_max"][None, :]
        / np.maximum(geometry["base_sigma_min"][None, :], np.finfo(float).tiny)
    )
    first_condition = np.argmax(condition_by_layer >= RISK_CONDITION_LIMIT, axis=0) + 1
    never_condition = ~np.any(condition_by_layer >= RISK_CONDITION_LIMIT, axis=0)
    first_condition[never_condition] = LAYERS + 1
    curvature, branch_scale = naca_curvature_and_clearance(points, faces, face_rows)
    focal_by_layer = cumulative[:, None] * curvature[None, :]
    first_focal = np.argmax(focal_by_layer >= 0.70, axis=0) + 1
    never_focal = ~np.any(focal_by_layer >= 0.70, axis=0)
    first_focal[never_focal] = LAYERS + 1
    first_branch = np.argmax(cumulative[:, None] >= 0.70 * branch_scale[None, :], axis=0) + 1
    never_branch = ~np.any(cumulative[:, None] >= 0.70 * branch_scale[None, :], axis=0)
    first_branch[never_branch] = LAYERS + 1
    feature = np.zeros(len(faces), dtype=bool)
    for face_id, face in enumerate(faces):
        feature[face_id] = any(
            bool(vertex_rows[int(vertex)]["curves"])
            or int(vertex_rows[int(vertex)]["corner"]) != 0
            for vertex in face
        )
    spread_risk = geometry["trajectory_spread_deg"] >= RISK_TRAJECTORY_SPREAD_DEG
    risk = (
        (first_condition <= LAYERS)
        | (first_focal <= LAYERS)
        | (first_branch <= LAYERS)
        | spread_risk
        | feature
    )
    first_risk = np.minimum.reduce((first_condition, first_focal, first_branch))
    first_risk[spread_risk | feature] = np.minimum(first_risk[spread_risk | feature], 2)
    risk_class = np.full(len(faces), "NominalSafe", dtype="U32")
    risk_class[first_condition <= LAYERS] = "NormalMapRisk"
    risk_class[first_focal <= LAYERS] = "NormalMapRisk"
    risk_class[first_branch <= LAYERS] = "BranchMergeRisk"
    risk_class[spread_risk] = "FeatureTransitionRisk"
    risk_class[feature] = "FeatureTransitionRisk"
    return {
        "risk": risk,
        "first_risk_layer": first_risk,
        "risk_class": risk_class,
        "normal_condition_at_nominal_total": condition_by_layer[-1],
        "curvature_1_per_m": curvature,
        "branch_scale_m": branch_scale,
        "feature_incident": feature,
    }


def audit_faces(points: np.ndarray, faces: np.ndarray, trajectories: np.ndarray,
                increments: np.ndarray, selected_faces: np.ndarray | list[int] | None = None,
                *, collect_failures: bool = True, accepted_state=None) -> dict:
    if selected_faces is None:
        selected = np.arange(len(faces), dtype=np.int64)
    else:
        selected = np.asarray(selected_faces, dtype=np.int64)
    incidence = faces[selected]
    base = points[incidence]
    directions = trajectories[incidence]
    base_normal = np.cross(base[:, 1] - base[:, 0], base[:, 2] - base[:, 0])
    cumulative = np.zeros((len(points),), dtype=np.float64)
    positions = None if accepted_state is None else accepted_state.positions(increments, points, faces)
    by_layer = []
    failures: list[dict] = []
    for layer in range(LAYERS):
        lower_height = cumulative[incidence]
        cumulative += increments[:, layer]
        upper_height = cumulative[incidence]
        lower = base + lower_height[:, :, None] * directions
        upper = base + upper_height[:, :, None] * directions
        if positions is not None:
            lower = positions[layer, incidence]
            upper = positions[layer + 1, incidence]
        prism = np.concatenate((lower, upper), axis=1)
        jacobian, volume = PM._exact_wedge_jacobian_and_volume(prism)
        determinant = PM._foundation_determinant(
            prism, lower_cap_is_uncoupled_wall=(layer == 0)
        )
        if accepted_state is not None:
            determinant = accepted_state.determinant(prism, layer, PM)
        top_normal = np.cross(upper[:, 1] - upper[:, 0], upper[:, 2] - upper[:, 0])
        orientation = np.einsum("ij,ij->i", top_normal, base_normal)
        hard = determinant < FOUNDATION_HARD
        qual = determinant < FOUNDATION_QUAL
        jac_fail = jacobian <= 0.0
        vol_fail = volume <= 0.0
        orient_fail = orientation <= 0.0
        fail = qual | jac_fail | vol_fail | orient_fail
        by_layer.append({
            "layer": layer + 1,
            "minimum_foundation_determinant": float(determinant.min()),
            "foundation_hard_failures": int(hard.sum()),
            "foundation_qualification_failures": int(qual.sum()),
            "minimum_exact_jacobian": float(jacobian.min()),
            "nonpositive_exact_jacobians": int(jac_fail.sum()),
            "minimum_signed_volume_m3": float(volume.min()),
            "nonpositive_signed_volumes": int(vol_fail.sum()),
            "minimum_orientation": float(orientation.min()),
            "orientation_failures": int(orient_fail.sum()),
        })
        if collect_failures and np.any(fail):
            for local in np.flatnonzero(fail):
                failures.append({
                    "face": int(selected[local]),
                    "layer": layer + 1,
                    "foundation_determinant": float(determinant[local]),
                    "exact_jacobian": float(jacobian[local]),
                    "signed_volume_m3": float(volume[local]),
                    "orientation": float(orientation[local]),
                })
    return {
        "faces_evaluated": int(len(selected)),
        "prisms_evaluated": int(len(selected) * LAYERS),
        "by_layer": by_layer,
        "failures": failures,
        "pass": not failures if collect_failures else all(
            row["foundation_qualification_failures"] == 0
            and row["nonpositive_exact_jacobians"] == 0
            and row["nonpositive_signed_volumes"] == 0
            and row["orientation_failures"] == 0
            for row in by_layer
        ),
    }


def impacted_faces(vertices: np.ndarray | list[int], incident: list[list[int]]) -> np.ndarray:
    result: set[int] = set()
    for vertex in vertices:
        result.update(incident[int(vertex)])
    return np.asarray(sorted(result), dtype=np.int64)


def local_pass(points, faces, trajectories, increments, face_ids, *, accepted_state=None) -> bool:
    return bool(audit_faces(
        points, faces, trajectories, increments, face_ids, collect_failures=False, accepted_state=accepted_state
    )["pass"])


def select_layer2_component(component: list[int], points: np.ndarray, faces: np.ndarray,
                            trajectories: np.ndarray, increments: np.ndarray,
                            incident: list[list[int]], *, accepted_state=None) -> dict:
    vertices = np.unique(faces[np.asarray(component, dtype=np.int64)].ravel())
    check_faces = impacted_faces(vertices, incident)
    nominal = float(nominal_increments()[1])
    upper = float(nominal_increments()[2])
    grid = np.linspace(nominal, upper, 161)
    passing = []
    for value in grid:
        proposal = increments.copy()
        proposal[vertices, 1] = value
        passing.append(local_pass(points, faces, trajectories, proposal, check_faces, accepted_state=accepted_state))
    passing = np.asarray(passing, dtype=bool)
    if not np.any(passing):
        return {"pass": False, "reason": "EmptyLayer2IncrementBand"}
    first = int(np.flatnonzero(passing)[0])
    lo = float(grid[max(0, first - 1)])
    hi = float(grid[first])
    for _ in range(64):
        midpoint = 0.5 * (lo + hi)
        proposal = increments.copy(); proposal[vertices, 1] = midpoint
        if local_pass(points, faces, trajectories, proposal, check_faces, accepted_state=accepted_state):
            hi = midpoint
        else:
            lo = midpoint
    # Leave two percent of the remaining declared bracket as an interior margin.
    selected = hi + INTERIOR_FRACTION * (upper - hi)
    proposal = increments.copy(); proposal[vertices, 1] = selected
    return {
        "pass": local_pass(points, faces, trajectories, proposal, check_faces, accepted_state=accepted_state),
        "vertices": vertices,
        "checked_faces": check_faces,
        "lower_boundary_m": hi,
        "upper_bracket_m": upper,
        "selected_increment_m": selected,
        "proposal": proposal,
    }


def scale_field(distance: np.ndarray, radius: int, core_scale: float) -> np.ndarray:
    factor = np.ones(len(distance), dtype=np.float64)
    inside = distance < radius
    factor[inside] = core_scale + (1.0 - core_scale) * distance[inside] / radius
    return factor


def passing_scale_components(valid: np.ndarray, grid: np.ndarray) -> list[tuple[int, int]]:
    components = []
    begin = None
    for index, status in enumerate(valid):
        if status and begin is None:
            begin = index
        if begin is not None and (not status or index == len(valid) - 1):
            end = index if status else index - 1
            components.append((begin, end))
            begin = None
    return components


def select_late_component(component: list[int], points: np.ndarray, faces: np.ndarray,
                          trajectories: np.ndarray, increments: np.ndarray,
                          incident: list[list[int]], neighbors: list[tuple[int, ...]],
                          vertex_area: np.ndarray, compression_start_layer: int, *, accepted_state=None) -> dict:
    core_vertices = set(int(value) for value in faces[np.asarray(component)].ravel())
    candidates = []
    nominal = nominal_increments()
    for radius in range(1, MAX_TAPER_RINGS + 1):
        distance = graph_distances(core_vertices, neighbors, radius)
        changed = np.flatnonzero(distance < radius)
        check_faces = impacted_faces(changed, incident)
        grid = np.linspace(0.05, 1.0, 39)
        valid = []
        for core_scale in grid:
            factor = scale_field(distance, radius, float(core_scale))
            proposal = increments.copy()
            proposal[:, compression_start_layer - 1:] = np.minimum(
                proposal[:, compression_start_layer - 1:],
                nominal[None, compression_start_layer - 1:] * factor[:, None],
            )
            valid.append(local_pass(points, faces, trajectories, proposal, check_faces, accepted_state=accepted_state))
        valid = np.asarray(valid, dtype=bool)
        components = passing_scale_components(valid, grid)
        if not components:
            continue
        # Select the passing component closest to nominal, then tighten its upper
        # edge by bisection.  This is a bounded feasibility search, not tuning.
        begin, end = max(components, key=lambda pair: (grid[pair[1]], pair[1] - pair[0]))
        lower_boundary = float(grid[begin])
        upper_boundary = float(grid[end])
        if end + 1 < len(grid):
            lo, hi = float(grid[end]), float(grid[end + 1])
            for _ in range(60):
                midpoint = 0.5 * (lo + hi)
                factor = scale_field(distance, radius, midpoint)
                proposal = increments.copy()
                proposal[:, compression_start_layer - 1:] = np.minimum(
                    proposal[:, compression_start_layer - 1:],
                    nominal[None, compression_start_layer - 1:] * factor[:, None],
                )
                if local_pass(points, faces, trajectories, proposal, check_faces, accepted_state=accepted_state):
                    lo = midpoint
                else:
                    hi = midpoint
            upper_boundary = lo
        selected_scale = upper_boundary - INTERIOR_FRACTION * (upper_boundary - lower_boundary)
        factor = scale_field(distance, radius, selected_scale)
        proposal = increments.copy()
        proposal[:, compression_start_layer - 1:] = np.minimum(
            proposal[:, compression_start_layer - 1:],
            nominal[None, compression_start_layer - 1:] * factor[:, None],
        )
        if not local_pass(points, faces, trajectories, proposal, check_faces, accepted_state=accepted_state):
            continue
        total = proposal.sum(axis=1)
        retained_integral = float(np.dot(vertex_area, total))
        candidates.append({
            "radius_rings": radius,
            "core_scale_lower_boundary": lower_boundary,
            "core_scale_upper_boundary": upper_boundary,
            "selected_core_scale": selected_scale,
            "changed_vertices": changed,
            "checked_faces": check_faces,
            "retained_area_height_integral": retained_integral,
            "proposal": proposal,
        })
    if not candidates:
        return _select_independent_late_candidate(
            component, points, faces, trajectories, increments, incident,
            vertex_area, compression_start_layer, accepted_state)
    selected = max(candidates, key=lambda row: (
        row["retained_area_height_integral"],
        -len(row["changed_vertices"]),
        -row["radius_rings"],
    ))
    selected["pass"] = True
    selected["candidate_count"] = len(candidates)
    return selected


def _select_independent_late_candidate(component, points, faces, trajectories,
                                     increments, incident, vertex_area,
                                     compression_start_layer, accepted_state):
    """Bounded per-vertex/per-layer compression in the existing schedule.

    Called only when every existing one-field suffix candidate is rejected.
    Old candidate ordering and selection remain unchanged. No direction changes.
    """
    if accepted_state is None:
        return {"pass": False, "reason": "IndependentLateRequiresAcceptedFrontState"}
    import ast
    import casadi as ca
    FG = load_module("independent_late_foundation_geometry", V3 / "foundation_aware_prism_shell_qualification/foundation_geometry.py")
    started = time.perf_counter()
    state = accepted_state
    positions = state.positions(increments, points, faces)
    edges, edge_faces = FG.unique_manifold_edges(faces)
    weight_module = load_module("v3_independent_late_weight_predicates",
                               V3 / "foundation_aware_prism_shell_qualification/run_weight_aware_planner.py")
    weight_audit = weight_module.complete_weight_audit(
        points, faces, trajectories, increments, edges, edge_faces, accepted_state=state)
    cells = set(component)
    cells.update(r["face"] for r in audit_faces(points, faces, trajectories, increments, accepted_state=state)["failures"] if r["layer"] >= compression_start_layer)
    cells.update(int(v) for v in edge_faces[weight_audit["qualification_edges"]].ravel())
    # Both already-qualified ownership contracts apply to the final shell.
    last = np.concatenate((positions[-2, faces], positions[-1, faces]), axis=1)
    vectors = PM._face_vectors(last)
    mask = np.array([1., 0., 1., 1., 1.])
    avg = np.sum(np.linalg.norm(vectors, axis=2) * mask, axis=1) / 4.
    scaled = vectors / avg[:, None, None]
    isolated = PM._det_symm(np.einsum('nfi,nfj,f->nij', scaled, scaled, mask))
    cells.update(map(int, np.flatnonzero(isolated < FOUNDATION_QUAL)))
    selected = sorted(set(map(int, faces[sorted(cells)].ravel())),
                      key=lambda v: state.vertex_order[v])
    affected = impacted_faces(selected, incident)
    side_ids = np.flatnonzero(np.any(np.isin(edge_faces, affected), axis=1))
    columns = np.union1d(affected, edge_faces[side_ids].ravel())
    vertices = np.unique(faces[columns])
    vl = {int(v): i for i, v in enumerate(vertices)}
    cl = {int(v): i for i, v in enumerate(columns)}
    si = {int(v): i for i, v in enumerate(selected)}
    local_faces = np.array([[vl[int(v)] for v in row] for row in faces[columns]])
    local_edges = np.array([[vl[int(v)] for v in row] for row in edges[side_ids]])
    owners = np.array([[cl[int(v)] for v in row] for row in edge_faces[side_ids]])
    # Reuse the existing exact symbolic prism/face infrastructure. The only
    # additional expression is the frozen final-isolated ownership mask.
    source = V3 / "complete_short_wing_continuation_2026-09-06/continuation_model.py"
    node = next(n for n in ast.parse(source.read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name == "functions")
    ns = dict(ca=ca, V3=V3)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), ns)
    _, cell, side, horizontal = ns["functions"]()
    src = (V3 / "finite_wing_terminal_prism_feasibility_2026-09-06/symbolic_prism.py").read_text()
    src = src.replace("coupled=newell[1:];a=sum(norm(s) for s in coupled)/4",
                      "coupled=[newell[i] for i in [0,2,3,4]];a=sum(norm(s) for s in coupled)/4")
    isolated_ns = {}; exec(compile(src, "existing_symbolic_prism:final_mask", "exec"), isolated_ns)
    isolated_cell, _ = isolated_ns["make_functions"]()
    cm, sm, hm, im = cell.map(len(columns)), side.map(len(side_ids)), horizontal.map(len(columns)), isolated_cell.map(len(columns))
    start = compression_start_layer
    count = LAYERS - start + 1
    z = ca.MX.sym("late_factors", len(selected) * count)
    lower_factors = np.minimum(1., .05 * nominal_increments()[None, start - 1:] / increments[selected, start - 1:]).ravel()
    lo = ca.DM(positions[start - 1, vertices].T)
    previous = ca.DM(FG.prism_cell_centres(np.concatenate(
        (positions[start - 2, faces[columns]], positions[start - 1, faces[columns]]), axis=1))[0].T)
    wall = points[faces[columns]]
    area2 = np.linalg.norm(np.cross(wall[:, 1] - wall[:, 0], wall[:, 2] - wall[:, 0]), axis=1)
    js = area2 * H1; oscale = area2 ** 2
    quality = []; retained = 0
    for j, k in enumerate(range(start, LAYERS + 1)):
        field = ca.horzcat(*[z[si[int(v)] * count + j] if int(v) in si else 1. for v in vertices])
        hi = lo + ca.DM(state.vectors[k - 2, vertices].T) * ca.repmat(field * ca.DM(increments[vertices, k - 1]).T, 3, 1)
        pp = ca.horzcat(*[ca.horzcat(lo[:, list(row)], hi[:, list(row)]) for row in local_faces])
        dt, jj, vv, oo, cent = cm(pp)
        quality.extend([dt.T / FOUNDATION_QUAL - 1.000001,
                        jj.T / ca.DM(js) - 1e-8, vv.T / ca.DM(js / 2) - 1e-8,
                        oo.T / ca.DM(oscale) - 1e-8])
        if k == LAYERS:
            quality.append(im(pp)[0].T / FOUNDATION_QUAL - 1.000001)
        quad = ca.horzcat(*[ca.horzcat(lo[:, int(a)], lo[:, int(b)], hi[:, int(b)], hi[:, int(a)]) for a, b in local_edges])
        sw, sk = sm(quad, cent[:, owners[:, 0].tolist()], cent[:, owners[:, 1].tolist()])
        tri = ca.horzcat(*[lo[:, list(row)] for row in local_faces])
        hw, hs = hm(tri, previous, cent)
        quality.extend([sw.T / .055 - 1.000001, 1 - sk.T / (4 / 1.1),
                        hw.T / .055 - 1.000001, 1 - hs.T / (4 / 1.1)])
        retained += sum(vertex_area[v] * increments[v, k - 1] * z[si[v] * count + j] for v in selected)
        previous, lo = cent, hi
    g = ca.vertcat(*quality)
    gf = ca.Function("late_quality", [z], [g])
    slack = ca.MX.sym("slack")
    options = {"print_time": False, "ipopt.print_level": 0, "ipopt.sb": "yes",
               "ipopt.max_cpu_time": 25., "ipopt.max_iter": 350,
               "ipopt.tol": 1e-8, "ipopt.constr_viol_tol": 1e-8,
               "ipopt.bound_relax_factor": 0., "ipopt.mu_strategy": "adaptive",
               "ipopt.hessian_approximation": "limited-memory"}
    feasible = ca.nlpsol("late_feasibility", "ipopt", dict(x=ca.vertcat(z, slack), f=slack, g=g + slack), options)
    objective = ca.nlpsol("late_retained_length", "ipopt", dict(x=z, f=-retained / max(float(np.dot(vertex_area, increments.sum(axis=1))), 1e-30), g=g), options)
    evaluator_source = V3 / "short_wing_displacement_contract_2026-09-06/evaluate.py"
    evaluator_tree = ast.parse(evaluator_source.read_text())
    evaluator_ns = dict(np=np, PM=PM, FG=FG, DET=FOUNDATION_QUAL,
                        WEIGHT=weight_module.QUAL_WEIGHT, SKEW=4 / 1.1)
    for node in evaluator_tree.body:
        if isinstance(node, ast.FunctionDef):
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(evaluator_source), "exec"), evaluator_ns)
    candidates = []; attempts = []
    for label, value in [("nominal", 1.), ("half", .5)]:
        if time.perf_counter() - started > 90:
            break
        initial = np.maximum(lower_factors, value)
        sl = max(0., -float(np.array(gf(initial)).min())) + 1e-5
        sol = feasible(x0=np.r_[initial, sl], lbx=np.r_[lower_factors, 0.],
                       ubx=np.r_[np.ones(z.numel()), 1e6], lbg=0., ubg=np.inf)
        proposal_z = np.array(sol["x"]).ravel()[:-1]
        if not np.all(np.isfinite(proposal_z)):
            attempts.append(dict(start=label, reason="NonfiniteProposal"))
            continue
        if float(np.array(gf(proposal_z)).min()) >= -1e-8:
            optimum = objective(x0=proposal_z, lbx=lower_factors, ubx=1., lbg=0., ubg=np.inf)
            optimized = np.array(optimum["x"]).ravel()
            if float(np.array(gf(optimized)).min()) >= -1e-8:
                proposal_z = optimized
        shortfall = max(0., -float(np.array(gf(proposal_z)).min()))
        attempts.append(dict(start=label,scaled_shortfall=shortfall,solver_status=feasible.stats()["return_status"]))
        if shortfall > 1e-8:
            continue
        if not (np.all(np.isfinite(proposal_z)) and np.all(proposal_z >= lower_factors)
                and np.all(proposal_z <= 1.)):
            attempts[-1]["bound_contract_pass"] = False
            continue
        proposal = increments.copy()
        proposal[selected, start - 1:] *= proposal_z.reshape(len(selected), count)
        if not audit_faces(points, faces, trajectories, proposal, columns, accepted_state=state)["pass"]:
            continue
        candidates.append(dict(**{'pass': True}, proposal=proposal, independent_vertex_layer_fallback=True,
                               changed_vertices=np.asarray(selected), checked_faces=columns,
                               affected_edges=side_ids,
                               retained_area_height_integral=float(np.dot(vertex_area, proposal.sum(axis=1))),
                               radius_rings=1, scalar_variables=int(z.numel())))
    if not candidates:
        return dict(**{'pass': False},reason="IndependentLateSearchUnresolved",attempts=attempts)
    best = max(candidates, key=lambda q:(q["retained_area_height_integral"],-len(q["changed_vertices"])))
    best.update(candidate_count=len(candidates),attempts=attempts,seconds=time.perf_counter()-started)
    return best



def select_staged_schedule(points, faces, trajectories, increments, *,
                           accepted_state=None, late_start_layer=None, observer=None):
    """Dispatch the existing selectors from the actual current schedule.

    No early component is required. The unchanged input schedule is the current
    state until a selector supplies an accepted proposal. Exact look-ahead callers
    use the established first-failure-minus-one compression start; callers with
    an earlier qualified risk prediction may supply their existing start rule.
    This function changes no candidate, objective, predicate, or vector meaning.
    """
    current = np.array(increments, dtype=np.float64, copy=True)
    incident, neighbors = build_incidence(faces, len(points))
    triangles = points[faces]
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1) / 2
    vertex_area = np.zeros(len(points), dtype=np.float64)
    for corner in range(3):
        np.add.at(vertex_area, faces[:, corner], area / 3)
    selections = []

    def evaluate():
        return audit_faces(points, faces, trajectories, current,
                           accepted_state=accepted_state)

    def emit(kind, **data):
        if observer is not None:
            observer(kind, data)

    nominal = evaluate()
    early_faces = sorted({r['face'] for r in nominal['failures'] if r['layer'] <= 2})
    early_components = connected_face_components(early_faces, faces, incident)
    emit('initial', audit=nominal, early_components=early_components)
    for index, component in enumerate(early_components):
        emit('early_begin', index=index, component=component)
        selected = select_layer2_component(component, points, faces, trajectories,
                                            current, incident, accepted_state=accepted_state)
        if not selected['pass']:
            return dict(pass_gate=False, phase='early', component=component,
                        reason=selected['reason'], selections=selections, audit=evaluate())
        current = selected['proposal']
        detail = {k: v for k, v in selected.items() if k != 'proposal'}
        selections.append(dict(phase='early', component=component, **detail))
        emit('early_selected', index=index, selection=detail, increments=current)

    # Re-audit selected early coordinates. Nominal late failures may have moved,
    # disappeared, or changed first failing layer after an early correction.
    after_early = evaluate()
    if any(r['layer'] <= 2 for r in after_early['failures']):
        return dict(pass_gate=False, phase='early', reason='SelectedEarlyStateFails',
                    selections=selections, audit=after_early)
    late_faces = sorted({r['face'] for r in after_early['failures']})
    late_components = connected_face_components(late_faces, faces, incident)
    emit('late_detection', audit=after_early, late_components=late_components,
         increments=current)
    for index, component in enumerate(late_components):
        actual = evaluate()
        records = [r for r in actual['failures'] if r['face'] in component]
        if not records:
            continue
        first = min(r['layer'] for r in records)
        start = max(3, first - 1) if late_start_layer is None else late_start_layer(component, records)
        emit('late_begin', index=index, component=component, records=records,
             compression_start_layer=start)
        selected = select_late_component(component, points, faces, trajectories,
                                          current, incident, neighbors, vertex_area,
                                          start, accepted_state=accepted_state)
        if not selected['pass']:
            return dict(pass_gate=False, phase='late', component=component,
                        reason=selected['reason'], selections=selections, audit=actual)
        current = selected['proposal']
        detail = {k: v for k, v in selected.items() if k != 'proposal'}
        selections.append(dict(phase='late', component=component,
                               compression_start_layer=start, **detail))
        emit('late_selected', index=index, selection=detail, increments=current)
    final = evaluate()
    if not final['pass']:
        return dict(pass_gate=False, phase='final', reason='SelectedFieldFailsFullExactAudit',
                    selections=selections, audit=final)
    emit('complete', audit=final, increments=current)
    return dict(pass_gate=True, proposal=current, selections=selections,
                nominal_audit=nominal, audit=final,
                early_component_count=len(early_components),
                late_component_count=len(late_components))


def distribution(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    return {
        "minimum": float(np.min(values)),
        "p05": float(np.quantile(values, 0.05)),
        "median": float(np.quantile(values, 0.50)),
        "p95": float(np.quantile(values, 0.95)),
        "maximum": float(np.max(values)),
    }


def serializable_controller(face_id: int, failure_rows: list[dict], faces: np.ndarray,
                            face_rows: list[dict[str, str]], vertex_rows: list[dict[str, str]],
                            geometry: dict, risk: dict) -> dict:
    rows = [row for row in failure_rows if row["face"] == face_id]
    return {
        "persistent_key": face_key(face_id, faces, face_rows, vertex_rows),
        "forensic_transient_face": face_id,
        "failing_layers": sorted({row["layer"] for row in rows}),
        "first_failure_layer": min(row["layer"] for row in rows),
        "minimum_foundation_determinant": min(row["foundation_determinant"] for row in rows),
        "minimum_exact_jacobian": min(row["exact_jacobian"] for row in rows),
        "minimum_signed_volume_m3": min(row["signed_volume_m3"] for row in rows),
        "minimum_orientation": min(row["orientation"] for row in rows),
        "trajectory_spread_deg": float(geometry["trajectory_spread_deg"][face_id]),
        "normal_condition_at_nominal_total": float(risk["normal_condition_at_nominal_total"][face_id]),
        "cheap_first_risk_layer": int(risk["first_risk_layer"][face_id]),
        "cheap_risk_class": str(risk["risk_class"][face_id]),
    }


def run(output: Path, replay: str) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    points_path = PRISM_SOURCE / "wall_points.npy"
    faces_path = PRISM_SOURCE / "wall_faces.npy"
    trajectory_path = PRISM_SOURCE / "trajectory.npy"
    face_ledger_path = SOURCE / "faces_C.tsv"
    vertex_ledger_path = SOURCE / "vertices_C.tsv"
    points = np.load(points_path)
    faces = np.load(faces_path)
    trajectories = np.load(trajectory_path)
    face_rows = load_tsv(face_ledger_path)
    vertex_rows = load_tsv(vertex_ledger_path)
    if points.shape != trajectories.shape or faces.shape != (96810, 3):
        raise RuntimeError("FrozenNacaAuthorityMismatch")
    incident, neighbors = build_incidence(faces, len(points))
    geometry = face_geometry(points, faces, trajectories)
    vertex_area = np.zeros(len(points), dtype=np.float64)
    for corner in range(3):
        np.add.at(vertex_area, faces[:, corner], geometry["area"] / 3.0)

    look_start = time.perf_counter()
    risk = cheap_risk_screen(points, faces, trajectories, face_rows, vertex_rows, geometry)
    look_seconds = time.perf_counter() - look_start
    risk_faces = np.flatnonzero(risk["risk"])

    increments = np.tile(nominal_increments(), (len(points), 1))
    exact_start = time.perf_counter()
    nominal_risk_audit = audit_faces(points, faces, trajectories, increments, risk_faces)
    exact_risk_seconds = time.perf_counter() - exact_start
    failed_risk_faces = np.asarray(sorted({row["face"] for row in nominal_risk_audit["failures"]}), dtype=np.int64)
    controller_records = [
        serializable_controller(face_id, nominal_risk_audit["failures"], faces,
                                face_rows, vertex_rows, geometry, risk)
        for face_id in failed_risk_faces
    ]
    # Raw IDs are retained only in forensic evidence.  Policy order is the
    # persistent semantic key.
    controller_records.sort(key=lambda row: key_string(row["persistent_key"]))
    controller_by_key = {key_string(row["persistent_key"]): row for row in controller_records}

    lower_faces = np.asarray([
        row["forensic_transient_face"] for row in controller_records
        if row["first_failure_layer"] <= 2
    ], dtype=np.int64)
    late_faces = np.asarray([
        row["forensic_transient_face"] for row in controller_records
        if row["first_failure_layer"] > 2
    ], dtype=np.int64)
    selections = []
    for component in connected_face_components(lower_faces, faces, incident):
        keys = sorted(key_string(face_key(face_id, faces, face_rows, vertex_rows)) for face_id in component)
        selected = select_layer2_component(component, points, faces, trajectories, increments, incident)
        if not selected["pass"]:
            raise RuntimeError(f"Layer2BandEmpty:{keys}")
        increments = selected.pop("proposal")
        selections.append({
            "kind": "Layer2ConditioningIncrease",
            "controller_keys": [json.loads(key) for key in keys],
            "selected_increment_m": selected["selected_increment_m"],
            "lower_boundary_m": selected["lower_boundary_m"],
            "upper_bracket_m": selected["upper_bracket_m"],
            "changed_vertex_count": int(len(selected["vertices"])),
            "checked_face_count": int(len(selected["checked_faces"])),
        })

    for component in connected_face_components(late_faces, faces, incident):
        records = [controller_by_key[key_string(face_key(face_id, faces, face_rows, vertex_rows))]
                   for face_id in component]
        first_failure = min(row["first_failure_layer"] for row in records)
        predicted = min(row["cheap_first_risk_layer"] for row in records)
        compression_start = max(3, min(first_failure - 1, predicted - 1))
        keys = sorted(key_string(row["persistent_key"]) for row in records)
        selected = select_late_component(
            component, points, faces, trajectories, increments, incident, neighbors,
            vertex_area, compression_start,
        )
        if not selected["pass"]:
            raise RuntimeError(f"LatePathEmpty:{keys}")
        increments = selected.pop("proposal")
        selections.append({
            "kind": "FutureRiskEarlyCompression",
            "controller_keys": [json.loads(key) for key in keys],
            "nominal_first_failure_layer": first_failure,
            "cheap_first_risk_layer": predicted,
            "compression_start_layer": compression_start,
            "radius_rings": selected["radius_rings"],
            "core_scale_lower_boundary": selected["core_scale_lower_boundary"],
            "core_scale_upper_boundary": selected["core_scale_upper_boundary"],
            "selected_core_scale": selected["selected_core_scale"],
            "changed_vertex_count": int(len(selected["changed_vertices"])),
            "checked_face_count": int(len(selected["checked_faces"])),
            "candidate_count": selected["candidate_count"],
        })

    # Independent full-wall falsification and acceptance authority.
    final_exact_start = time.perf_counter()
    final_audit = audit_faces(points, faces, trajectories, increments)
    final_exact_seconds = time.perf_counter() - final_exact_start
    if not final_audit["pass"]:
        raise RuntimeError("SelectedFieldFailsFullExactAudit")

    # Replay the nominal schedule everywhere once to falsify the cheap screen.
    nominal_full_start = time.perf_counter()
    nominal_full = audit_faces(
        points, faces, trajectories,
        np.tile(nominal_increments(), (len(points), 1)),
    )
    nominal_full_seconds = time.perf_counter() - nominal_full_start
    nominal_failure_faces = {row["face"] for row in nominal_full["failures"]}
    false_safe = sorted(face for face in nominal_failure_faces if not risk["risk"][face])
    if false_safe:
        raise RuntimeError(f"SDFLookAheadFalseSafe:{false_safe[:16]}")

    total = increments.sum(axis=1)
    compressed = np.any(increments < nominal_increments()[None, :] - 1.0e-15, axis=1)
    increased = np.any(increments > nominal_increments()[None, :] + 1.0e-15, axis=1)
    per_layer = []
    cumulative = np.zeros(len(points), dtype=np.float64)
    for layer in range(LAYERS):
        cumulative += increments[:, layer]
        row = final_audit["by_layer"][layer]
        per_layer.append({
            **row,
            "increment_m": distribution(increments[:, layer]),
            "cumulative_height_m": distribution(cumulative),
            "vertices_at_nominal_increment": int(np.sum(np.abs(increments[:, layer] - nominal_increments()[layer]) <= 1.0e-15)),
            "future_viability": "CertifiedRemainingTwentyLayerPath",
        })

    plan_path = output / f"NACA_STAGED_PLAN_{replay}.npz"
    np.savez(
        plan_path,
        increments=np.asarray(increments, dtype=np.float64),
        cumulative=np.cumsum(increments, axis=1),
        risk=np.asarray(risk["risk"], dtype=np.uint8),
        first_risk_layer=np.asarray(risk["first_risk_layer"], dtype=np.int16),
    )
    semantic_report = {
        "schema": "V3SDFFutureGuidedNacaStagedPlan/v1",
        "replay": replay,
        "frozen_inputs": {
            "vertices": int(len(points)),
            "faces": int(len(faces)),
            "h1_m": H1,
            "maximum_layers": LAYERS,
            "nominal_growth": R_NOMINAL,
            "desired_total_height_m": float(nominal_increments().sum()),
            "foundation_hard": FOUNDATION_HARD,
            "foundation_qualification": FOUNDATION_QUAL,
            "wall_points_sha256": sha256(points_path),
            "wall_faces_sha256": sha256(faces_path),
            "trajectories_sha256": sha256(trajectory_path),
            "face_ledger_sha256": sha256(face_ledger_path),
            "vertex_ledger_sha256": sha256(vertex_ledger_path),
        },
        "cheap_lookahead": {
            "inputs": [
                "front tangent singular scale",
                "trajectory-map singular scale",
                "analytic NACA curvature",
                "procedural branch scale",
                "persistent feature incidence",
                "remaining requested cumulative schedule",
            ],
            "normal_map_condition_limit": RISK_CONDITION_LIMIT,
            "trajectory_spread_risk_degrees": RISK_TRAJECTORY_SPREAD_DEG,
            "risk_faces": int(len(risk_faces)),
            "risk_face_fraction": float(len(risk_faces) / len(faces)),
            "safe_faces": int(len(faces) - len(risk_faces)),
            "exact_checks_avoided_initial_fraction": float((len(faces) - len(risk_faces)) / len(faces)),
            "false_safe_faces": false_safe,
            "false_safe_count": len(false_safe),
            "false_risk_note": "Conservative risk is accepted; exact escalation remains authority.",
        },
        "nominal_controllers": controller_records,
        "selections": selections,
        "selected_field": {
            "increments_sha256": hash_array(increments),
            "plan_npz_sha256": sha256(plan_path),
            "increment_by_layer": [distribution(increments[:, layer]) for layer in range(LAYERS)],
            "total_height_m": distribution(total),
            "fraction_vertices_compressed": float(np.mean(compressed)),
            "fraction_vertices_with_layer2_conditioning_increase": float(np.mean(increased)),
            "fraction_vertices_reaching_20mm": float(np.mean(total >= 0.020 - 1.0e-12)),
            "fraction_vertices_reaching_15mm": float(np.mean(total >= 0.015)),
            "fraction_vertices_reaching_10mm": float(np.mean(total >= 0.010)),
            "minimum_first_height_m": float(increments[:, 0].min()),
            "maximum_first_height_m": float(increments[:, 0].max()),
        },
        "exact_prospective": {
            "nominal_risk_audit_faces": int(len(risk_faces)),
            "nominal_full_falsification_prisms": int(len(faces) * LAYERS),
            "selected_full_prisms": int(len(faces) * LAYERS),
            "by_layer": per_layer,
            "foundation_hard_failures": int(sum(row["foundation_hard_failures"] for row in final_audit["by_layer"])),
            "foundation_qualification_failures": int(sum(row["foundation_qualification_failures"] for row in final_audit["by_layer"])),
            "nonpositive_exact_jacobians": int(sum(row["nonpositive_exact_jacobians"] for row in final_audit["by_layer"])),
            "nonpositive_signed_volumes": int(sum(row["nonpositive_signed_volumes"] for row in final_audit["by_layer"])),
            "orientation_failures": int(sum(row["orientation_failures"] for row in final_audit["by_layer"])),
            "pass": final_audit["pass"],
        },
        "timing_seconds": {
            "cheap_lookahead": look_seconds,
            "risk_local_exact_nominal": exact_risk_seconds,
            "selected_full_exact_validation": final_exact_seconds,
            "nominal_full_falsification": nominal_full_seconds,
            "total_planning": time.perf_counter() - started,
        },
        "pass": final_audit["pass"] and len(false_safe) == 0,
    }
    canonical = dict(semantic_report)
    canonical.pop("replay", None)
    canonical.pop("timing_seconds", None)
    canonical["selected_field"] = dict(canonical["selected_field"])
    canonical["selected_field"].pop("plan_npz_sha256", None)
    semantic_encoded = json.dumps(
        semantic_report, indent=2, sort_keys=True, allow_nan=False, default=json_scalar
    ) + "\n"
    canonical_encoded = json.dumps(
        canonical, sort_keys=True, separators=(",", ":"), allow_nan=False,
        default=json_scalar,
    ) + "\n"
    report_path = output / f"NACA_STAGED_PLAN_{replay}.json"
    canonical_path = output / f"NACA_STAGED_PLAN_{replay}_CANONICAL.json"
    report_path.write_text(semantic_encoded, encoding="utf-8")
    canonical_path.write_text(canonical_encoded, encoding="utf-8")
    return {
        "report": report_path,
        "canonical": canonical_path,
        "plan": plan_path,
        "report_sha256": sha256(report_path),
        "canonical_sha256": sha256(canonical_path),
        "plan_sha256": sha256(plan_path),
        "increments_sha256": hash_array(increments),
        "pass": semantic_report["pass"],
        "summary": semantic_report,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=HERE / "naca_plan")
    parser.add_argument("--replay", choices=("A", "A2"), required=True)
    args = parser.parse_args()
    result = run(args.output, args.replay)
    print(json.dumps({
        "pass": result["pass"],
        "replay": args.replay,
        "report": str(result["report"]),
        "report_sha256": result["report_sha256"],
        "canonical_sha256": result["canonical_sha256"],
        "increments_sha256": result["increments_sha256"],
        "minimum_total_height_m": result["summary"]["selected_field"]["total_height_m"]["minimum"],
        "fraction_reaching_20mm": result["summary"]["selected_field"]["fraction_vertices_reaching_20mm"],
    }, sort_keys=True))
    return 0 if result["pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
