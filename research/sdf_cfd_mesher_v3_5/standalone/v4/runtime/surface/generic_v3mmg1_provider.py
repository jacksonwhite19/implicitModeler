"""Geometry-neutral live-authority to V3MMG1 provider and post-MMGS projector."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
from collections import defaultdict

import numpy as np

from procedural_authorities import make_authority
from prism_host_metric import PrismMetricPolicy, intersect_tangent_metric
from two_sided_metric import apply_two_sided_policy, load_policy as load_two_sided_policy
from shape_aware_metric import (ShapeMetricPolicy, apply_shape_metric,
                                generic_geometry_metric,
                                generic_geometry_metric_from_differential)
from seed_support_metric import (SeedSupportEnvelopePolicy,
                                 apply_seed_support_envelopes)
from shape_operator_support_envelope import (OperatorSupportPolicy,
                                             apply_shape_operator_support_envelopes)
from principal_direction_cone_support import apply_principal_cone_support
from adaptive_sdf_carrier import (CarrierOraclePolicy, refine_metric_carriers,
                                  promote_face_probe_metrics)


def spd_intersection(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Deterministic Loewner majorant of two SPD tensors."""
    values, vectors = np.linalg.eigh(a)
    root = (vectors * np.sqrt(values)) @ vectors.T
    inv_root = (vectors * (1.0 / np.sqrt(values))) @ vectors.T
    relative = inv_root @ b @ inv_root
    relative_values, relative_vectors = np.linalg.eigh(0.5 * (relative + relative.T))
    capped = (relative_vectors * np.maximum(relative_values, 1.0)) @ relative_vectors.T
    return 0.5 * (root @ capped @ root + (root @ capped @ root).T)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def write_contract(path: Path, vertices, faces, edges, authority, multiplier: float,
                   prism_policy: PrismMetricPolicy | None = None,
                   two_sided_policy: dict | None = None,
                   shape_policy: ShapeMetricPolicy | None = None,
                   support_policy: SeedSupportEnvelopePolicy | None = None,
                   operator_support_policy: OperatorSupportPolicy | None = None,
                   principal_cone_depth: int | None = None,
                   metric_override: list[np.ndarray] | None = None) -> dict[str, object]:
    vertex_curves = [set() for _ in vertices]
    for edge in edges:
        vertex_curves[edge.vertices[0]].add(edge.curve_ref)
        vertex_curves[edge.vertices[1]].add(edge.curve_ref)
    metric_statistics = {
        "vertices": len(vertices),
        "prism_metric_active_vertices": 0,
        "one_tangent_direction_refined_vertices": 0,
        "two_tangent_directions_refined_vertices": 0,
        "prism_tangent_cap_m": None,
        "wall_tangent_to_height_ratio": None,
        "owned_layer_tangent_to_height_ratio": None,
        "wall_tangent_cap_m": None,
        "owned_layer_tangent_cap_m": None,
        "reference_tangent_cap_m": None,
        "metric_response_factor": None,
    }
    metrics = []
    shape_rows: list[dict] = []
    shape_statistics = {"enabled": shape_policy is not None, "active_vertices": 0,
                        "anisotropy_before": [], "anisotropy_after": [],
                        "long_scale_ratio": [], "active_vertex_mask": []}
    for i, vertex in enumerate(vertices):
        principal = None
        if shape_policy is None:
            metric = authority.metric(vertex, vertex_curves[i], multiplier)
        else:
            metric, principal = generic_geometry_metric(authority, vertex, multiplier)
            if vertex_curves[i]:
                metric = spd_intersection(metric,
                                          authority.metric(vertex, vertex_curves[i], multiplier))
        if prism_policy is not None:
            metric, prism = intersect_tangent_metric(
                metric,
                authority.exact_normal(vertex.sheet_ref, vertex.point),
                prism_policy,
            )
            directions = int(prism["directions_refined"])
            metric_statistics["prism_metric_active_vertices"] += int(directions > 0)
            metric_statistics["one_tangent_direction_refined_vertices"] += int(directions == 1)
            metric_statistics["two_tangent_directions_refined_vertices"] += int(directions == 2)
            metric_statistics["prism_tangent_cap_m"] = prism["prism_tangent_cap_m"]
            metric_statistics["wall_tangent_to_height_ratio"] = prism["wall_tangent_to_height_ratio"]
            metric_statistics["owned_layer_tangent_to_height_ratio"] = prism["owned_layer_tangent_to_height_ratio"]
            metric_statistics["wall_tangent_cap_m"] = prism["wall_tangent_cap_m"]
            metric_statistics["owned_layer_tangent_cap_m"] = prism["owned_layer_tangent_cap_m"]
            metric_statistics["reference_tangent_cap_m"] = prism["reference_tangent_cap_m"]
            metric_statistics["metric_response_factor"] = prism["metric_response_factor"]
        if shape_policy is not None:
            metric, shape = apply_shape_metric(metric, principal, shape_policy)
            shape_rows.append(shape)
            shape_statistics["active_vertices"] += int(shape["active"])
            shape_statistics["active_vertex_mask"].append(bool(shape["active"]))
            shape_statistics["anisotropy_before"].append(shape["anisotropy_before"])
            shape_statistics["anisotropy_after"].append(shape["anisotropy_after"])
            shape_statistics["long_scale_ratio"].append(
                shape["long_length_after_m"] / shape["long_length_before_m"])
        metrics.append(metric)
    two_sided_statistics = {"enabled": False}
    if two_sided_policy is not None:
        metrics, two_sided_statistics = apply_two_sided_policy(
            metrics, vertices, faces, edges, authority, two_sided_policy
        )
    support_statistics = {"enabled": False}
    if support_policy is not None:
        if shape_policy is None:
            raise ValueError("seed-support envelope requires --shape-aware")
        metrics, support_statistics = apply_seed_support_envelopes(
            metrics, vertices, faces, authority, multiplier, prism_policy,
            shape_policy, support_policy, shape_rows)
        if not support_statistics.get("mmgs_materialization_authorized", False):
            raise RuntimeError(
                "seed-support metric envelope contains Unresolved supports")
    operator_support_statistics = {"enabled": False}
    if operator_support_policy is not None:
        if shape_policy is None:
            raise ValueError("shape-operator support envelope requires --shape-aware")
        if support_policy is not None:
            raise ValueError("principal and operator support envelopes are mutually exclusive")
        metrics, operator_support_statistics = apply_shape_operator_support_envelopes(
            metrics, vertices, faces, authority, multiplier, shape_policy,
            operator_support_policy)
        if not operator_support_statistics.get("mmgs_materialization_authorized", False):
            raise RuntimeError("shape-operator support envelope contains Unresolved supports")
    principal_cone_statistics = {"enabled": False}
    if principal_cone_depth is not None:
        if shape_policy is None:
            raise ValueError("principal cone support requires --shape-aware")
        if support_policy is not None or operator_support_policy is not None:
            raise ValueError("support metric authorities are mutually exclusive")
        metrics, principal_cone_statistics = apply_principal_cone_support(
            metrics, vertices, faces, authority, shape_policy,
            principal_cone_depth)

    if metric_override is not None:
        if len(metric_override) != len(vertices):
            raise ValueError("metric override does not match vertex inventory")
        metrics = [np.asarray(value, dtype=float) for value in metric_override]

    with path.open("wb") as out:
        out.write(b"V3MMG1\0\0")
        out.write(struct.pack("<QQQ", len(vertices), len(faces), len(edges)))
        for i, vertex in enumerate(vertices):
            metric = metrics[i]
            eig = np.linalg.eigvalsh(metric)
            if not np.all(eig > 0.0):
                raise RuntimeError(f"non-SPD metric at vertex {i}: {eig}")
            # A unique corner token remains transient; semantic corner identity is
            # reconstructed from persistent curve incidence after adaptation.
            ref = vertex.corner_ref if vertex.corner_ref else vertex.sheet_ref
            out.write(struct.pack("<ddd i BB dddddd", *vertex.point, ref,
                                  int(bool(vertex.corner_ref)), int(bool(vertex.corner_ref)),
                                  metric[0, 0], metric[0, 1], metric[0, 2],
                                  metric[1, 1], metric[1, 2], metric[2, 2]))
        for face in faces:
            out.write(struct.pack("<iiii", face.vertices[0] + 1, face.vertices[1] + 1,
                                  face.vertices[2] + 1, face.sheet_ref))
        for edge in edges:
            out.write(struct.pack("<iiiBB", edge.vertices[0] + 1, edge.vertices[1] + 1,
                                  edge.curve_ref, int(edge.ridge), 0))
    metric_statistics["two_sided_schedule_metric"] = two_sided_statistics
    if shape_policy is not None:
        active_mask = shape_statistics.pop("active_vertex_mask")
        total_area = 0.0
        active_area = 0.0
        active_faces = 0
        for face in faces:
            a, b, c = (vertices[index].point for index in face.vertices)
            area = 0.5 * float(np.linalg.norm(np.cross(b - a, c - a)))
            total_area += area
            if any(active_mask[index] for index in face.vertices):
                active_faces += 1
                active_area += area
        shape_statistics["active_seed_faces"] = active_faces
        shape_statistics["active_seed_area_m2"] = active_area
        shape_statistics["total_seed_area_m2"] = total_area
        shape_statistics["active_seed_area_fraction"] = (
            active_area / total_area if total_area > 0.0 else 0.0)
        for key in ("anisotropy_before", "anisotropy_after", "long_scale_ratio"):
            values = np.asarray(shape_statistics[key], dtype=float)
            shape_statistics[key] = {"minimum": float(values.min()),
                                     "median": float(np.quantile(values, .5)),
                                     "p95": float(np.quantile(values, .95)),
                                     "maximum": float(values.max())}
        shape_statistics["normal_qualification_fraction"] = shape_policy.normal_qualification_fraction
        shape_statistics["mmgs_response_reserve"] = shape_policy.mmgs_response_reserve
    metric_statistics["shape_aware_tangent_conditioning"] = shape_statistics
    metric_statistics["seed_support_shape_envelope"] = support_statistics
    metric_statistics["shape_operator_support_envelope"] = operator_support_statistics
    metric_statistics["principal_direction_cone_support"] = principal_cone_statistics
    return metric_statistics


def parse_medit(path: Path):
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    sections = {}; i = 0
    names = {"Vertices", "Triangles", "Edges", "Corners", "RequiredVertices", "RequiredEdges"}
    while i < len(lines):
        if lines[i] in names:
            name = lines[i]; n = int(lines[i + 1]); sections[name] = lines[i + 2:i + 2 + n]; i += 2 + n
        else:
            i += 1
    vertices = np.array([[float(x) for x in row.split()[:3]] for row in sections["Vertices"]])
    vertex_ref = np.array([int(row.split()[3]) for row in sections["Vertices"]], dtype=np.int64)
    faces = np.array([[int(x) - 1 for x in row.split()[:3]] for row in sections["Triangles"]], dtype=np.int64)
    face_ref = np.array([int(row.split()[3]) for row in sections["Triangles"]], dtype=np.int64)
    edges = np.array([[int(x) - 1 for x in row.split()[:2]] for row in sections.get("Edges", [])], dtype=np.int64)
    edge_ref = np.array([int(row.split()[2]) for row in sections.get("Edges", [])], dtype=np.int64)
    corners = {int(row) - 1 for row in sections.get("Corners", [])}
    return vertices, vertex_ref, faces, face_ref, edges, edge_ref, corners


def canonicalize_and_project(authority, raw, faces, face_ref, edges, edge_ref, corners):
    batch = getattr(authority, 'project_sheet_batch', None)
    active = getattr(authority, 'construction_active', None)
    use_batch = callable(batch) and callable(active) and active()
    def input_binding():
        return (tuple((np.asarray(v).dtype.str,np.asarray(v).shape,
                       hashlib.sha256(np.asarray(v).tobytes()).hexdigest())
                      for v in (raw,faces,face_ref,edges,edge_ref)),tuple(sorted(corners)))
    captured = input_binding() if use_batch else None
    incident_faces = [set() for _ in raw]
    incident_curves = [set() for _ in raw]
    for tri, ref in zip(faces, face_ref):
        for v in tri: incident_faces[int(v)].add(int(ref))
    curve_edges = []
    for e, ref in zip(edges, edge_ref):
        if int(ref) not in authority.curve_names:
            continue
        curve_edges.append((int(e[0]), int(e[1]), int(ref)))
        incident_curves[int(e[0])].add(int(ref)); incident_curves[int(e[1])].add(int(ref))
    projected = raw.copy()
    ambiguous = []
    sheet_groups = {}
    for i, p in enumerate(raw):
        if incident_curves[i]:
            # All incident persistent curve authorities meet at a corner when more
            # than one is present. Project in stable ref order and verify agreement.
            candidates = [authority.project_curve(ref, p) for ref in sorted(incident_curves[i])]
            q = candidates[0]
            if any(np.linalg.norm(c - q) > 1.0e-9 for c in candidates[1:]):
                ambiguous.append(i); continue
            projected[i] = q
        elif len(incident_faces[i]) == 1:
            ref = next(iter(incident_faces[i]))
            if use_batch:sheet_groups.setdefault(ref, []).append(i)
            else:projected[i] = authority.project_sheet(ref, p)
        else:
            ambiguous.append(i)
    if ambiguous:
        raise RuntimeError(f"ambiguous authority vertices: {ambiguous[:20]}")
    for ref, indices in sheet_groups.items():
        proposed = batch(ref, raw[indices])
        if np.shape(proposed) != (len(indices),3) or not np.isfinite(proposed).all():
            raise ValueError('InvalidConstructionProjectionBatch')
        projected[indices] = proposed
    if use_batch and input_binding() != captured:
        raise ValueError('ChangedConstructionProviderInputs')
    return projected, incident_faces, incident_curves, curve_edges


def topology_report(points, faces, face_ref, curve_edges, incident_curves, authority):
    incidence = defaultdict(list); unique = set(); duplicates = 0
    for fi, tri in enumerate(faces):
        key = tuple(sorted(map(int, tri)))
        if key in unique: duplicates += 1
        unique.add(key)
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            incidence[tuple(sorted((int(a), int(b))))].append(fi)
    open_edges = sum(len(v) == 1 for v in incidence.values())
    overfull = sum(len(v) > 2 for v in incidence.values())
    curve_graph = defaultdict(lambda: defaultdict(int))
    for a, b, ref in curve_edges:
        curve_graph[ref][a] += 1; curve_graph[ref][b] += 1
    curve_failures = 0
    for ref, degrees in curve_graph.items():
        endpoints = sum(d == 1 for d in degrees.values())
        bad = sum(d not in (1, 2) for d in degrees.values())
        # Persistent curves may be open (2 endpoints) or closed (0 endpoints).
        curve_failures += bad + (endpoints not in (0, 2))
    orientation_failures = 0
    for tri, ref in zip(faces, face_ref):
        a, b, c = points[tri]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) == 0 or np.dot(n, authority.exact_normal(int(ref), (a + b + c) / 3.0)) <= 0:
            orientation_failures += 1
    return {"open_edges": open_edges, "overfull_edges": overfull,
            "duplicate_faces": duplicates, "orientation_failures": orientation_failures,
            "curve_graph_failures": curve_failures,
            "persistent_curve_count": len(curve_graph),
            "topology_pass": not any((open_edges, overfull, duplicates, orientation_failures, curve_failures))}


def write_outputs(output: Path, replay: str, points, raw, vertex_ref, faces, face_ref,
                  curve_edges, incident_faces, incident_curves, authority):
    off = output / f"surface_{replay}.off"
    with off.open("w", encoding="utf-8") as stream:
        stream.write(f"OFF\n{len(points)} {len(faces)} 0\n")
        for p in points: stream.write("%.17g %.17g %.17g\n" % tuple(p))
        for f in faces: stream.write(f"3 {f[0]} {f[1]} {f[2]}\n")
    with (output / f"vertices_{replay}.tsv").open("w", encoding="utf-8") as stream:
        stream.write("vertex\tx\ty\tz\traw_x\traw_y\traw_z\tbranches\tcurves\tcorner\tcorner_ref\tpersistent_corner\n")
        for i, (p, r) in enumerate(zip(points, raw)):
            branches = ",".join(authority.branch_name(x) for x in sorted(incident_faces[i]))
            curves = ",".join(authority.curve_names[x] for x in sorted(incident_curves[i]))
            semantic_corner_ref = int(vertex_ref[i]) if int(vertex_ref[i]) in authority.corner_names else 0
            corner = int(semantic_corner_ref != 0)
            persistent_corner = authority.corner_names.get(semantic_corner_ref, "")
            stream.write(f"{i}\t{p[0]:.17g}\t{p[1]:.17g}\t{p[2]:.17g}\t{r[0]:.17g}\t{r[1]:.17g}\t{r[2]:.17g}\t{branches}\t{curves}\t{corner}\t{semantic_corner_ref}\t{persistent_corner}\n")
    with (output / f"faces_{replay}.tsv").open("w", encoding="utf-8") as stream:
        stream.write("face\tv0\tv1\tv2\tsheet_ref\tprocedural_branch\tpersistent_sheet\n")
        for i, (f, ref) in enumerate(zip(faces, face_ref)):
            stream.write(f"{i}\t{f[0]}\t{f[1]}\t{f[2]}\t{ref}\t{authority.branch_name(int(ref))}\t{authority.sheet_names[int(ref)]}\n")
    with (output / f"features_{replay}.tsv").open("w", encoding="utf-8") as stream:
        stream.write("curve_ref\tcurve\tv0\tv1\n")
        for a, b, ref in curve_edges:
            stream.write(f"{ref}\t{authority.curve_names[ref]}\t{a}\t{b}\n")
    return off


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--authority", required=True)
    parser.add_argument("--vertices", required=True)
    parser.add_argument("--faces", required=True)
    parser.add_argument("--features")
    parser.add_argument("--output", required=True)
    parser.add_argument("--replay", required=True)
    parser.add_argument("--multiplier", type=float, required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--prism-first-height", type=float)
    parser.add_argument("--prism-foundation-target", type=float, default=0.0011)
    parser.add_argument("--prism-minimum-owned-layer-thickness", type=float)
    parser.add_argument("--prism-metric-response-factor", type=float, default=1.0)
    parser.add_argument("--two-sided-policy")
    parser.add_argument("--shape-aware", action="store_true")
    parser.add_argument("--shape-mmgs-response-reserve", type=float, default=5.5)
    parser.add_argument("--shape-support-envelope", action="store_true")
    parser.add_argument("--shape-support-subdivision-depth", type=int, default=2)
    parser.add_argument("--shape-operator-support-envelope", action="store_true")
    parser.add_argument("--shape-operator-support-subdivision-depth", type=int, default=2)
    parser.add_argument("--principal-direction-cone-support", action="store_true")
    parser.add_argument("--principal-direction-cone-subdivision-depth", type=int, default=1)
    parser.add_argument("--adaptive-sdf-carrier", action="store_true")
    parser.add_argument("--adaptive-sdf-carrier-maximum-depth", type=int, default=4)
    parser.add_argument("--interpolation-compatible-transition", action="store_true")
    args = parser.parse_args()
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    authority = make_authority(args.authority)
    vertices, faces, edges = authority.seed(args.vertices, args.faces, args.features)
    contract = output / f"v3mmg1_{args.replay}.bin"
    prism_policy = None
    if args.prism_first_height is not None:
        if not args.prism_first_height > 0.0:
            raise ValueError("prism first height must be positive")
        prism_policy = PrismMetricPolicy(
            first_height_m=args.prism_first_height,
            foundation_qualification_target=args.prism_foundation_target,
            minimum_owned_layer_thickness_m=args.prism_minimum_owned_layer_thickness,
            metric_response_factor=args.prism_metric_response_factor,
        )
    two_sided_policy = None if args.two_sided_policy is None else load_two_sided_policy(
        args.two_sided_policy
    )
    shape_policy = (ShapeMetricPolicy(mmgs_response_reserve=args.shape_mmgs_response_reserve)
                    if args.shape_aware else None)
    carrier_statistics = {"enabled": False}
    transition_statistics = {"enabled": False}
    metric_override = None
    if args.adaptive_sdf_carrier:
        if shape_policy is None:
            raise ValueError("adaptive SDF carrier requires --shape-aware")
        if any((args.shape_support_envelope, args.shape_operator_support_envelope,
                args.principal_direction_cone_support)):
            raise ValueError("adaptive carrier and continuous support envelopes are mutually exclusive")

        def carrier_metric_at(point, sheet_ref):
            vertex = type(vertices[0])(np.asarray(point, dtype=float), int(sheet_ref), 0, 0)
            metric, principal = generic_geometry_metric_from_differential(
                authority, vertex, args.multiplier)
            if prism_policy is not None:
                metric, _ = intersect_tangent_metric(
                    metric, authority.exact_normal(int(sheet_ref), point), prism_policy)
            metric, _ = apply_shape_metric(metric, principal, shape_policy)
            return metric

        vertices, faces, edges, carrier_statistics = refine_metric_carriers(
            authority, vertices, faces, edges, carrier_metric_at,
            CarrierOraclePolicy(
                normal_qualification_fraction=shape_policy.normal_qualification_fraction,
                mmgs_response_reserve=shape_policy.mmgs_response_reserve,
                maximum_depth=args.adaptive_sdf_carrier_maximum_depth,
            ), interpolation_compatible_transition=args.interpolation_compatible_transition)
        if not carrier_statistics["materialization_authorized"]:
            raise RuntimeError("adaptive SDF carrier adequacy unresolved at bounded depth")
        if args.interpolation_compatible_transition:
            if edges:
                raise ValueError("transition transport is qualified only on smooth supports")

            predictor_transition_active = any(
                int(row.get("transition_promoted_faces", 0)) > 0
                for row in carrier_statistics.get("levels", []))
            if not predictor_transition_active:
                transition_statistics = {
                    "enabled": True,
                    "role": "conservative interpolation-compatible transport; physical target unchanged",
                    "active_faces": 0,
                    "requested_vertices": 0,
                    "changed_vertices": 0,
                    "metric_edge_ratio_limit": CarrierOraclePolicy().allowed_metric_edge_ratio,
                    "weakening_allowed": False,
                    "exact_transport_skipped_after_direct_differential_no_op": True,
                }
            else:

                def contract_metric_at(point, sheet_ref):
                    vertex = type(vertices[0])(np.asarray(point, dtype=float), int(sheet_ref), 0, 0)
                    metric, principal = generic_geometry_metric(authority, vertex, args.multiplier)
                    if prism_policy is not None:
                        metric, _ = intersect_tangent_metric(
                            metric, authority.exact_normal(int(sheet_ref), point), prism_policy)
                    metric, _ = apply_shape_metric(metric, principal, shape_policy)
                    return metric

                metric_override, transition_statistics = promote_face_probe_metrics(
                    authority, vertices, faces, contract_metric_at,
                    CarrierOraclePolicy(
                        normal_qualification_fraction=shape_policy.normal_qualification_fraction,
                        mmgs_response_reserve=shape_policy.mmgs_response_reserve,
                        maximum_depth=args.adaptive_sdf_carrier_maximum_depth,
                    ))
    support_policy = (SeedSupportEnvelopePolicy(
        subdivision_depth=args.shape_support_subdivision_depth)
        if args.shape_support_envelope else None)
    operator_support_policy = (OperatorSupportPolicy(
        subdivision_depth=args.shape_operator_support_subdivision_depth)
        if args.shape_operator_support_envelope else None)
    metric_statistics = write_contract(
        contract, vertices, faces, edges, authority, args.multiplier, prism_policy,
        two_sided_policy, shape_policy, support_policy, operator_support_policy,
        (args.principal_direction_cone_subdivision_depth
         if args.principal_direction_cone_support else None),
        metric_override,
    )
    raw_mesh = output / f"mmgs_raw_{args.replay}.mesh"
    runner_command = ([sys.executable, args.runner] if args.runner.lower().endswith(".py")
                      else [args.runner])
    run = subprocess.run(runner_command + [str(contract), str(raw_mesh)], text=True,
                         capture_output=True, check=False)
    (output / f"mmgs_stdout_{args.replay}.txt").write_text(run.stdout, encoding="utf-8")
    (output / f"mmgs_stderr_{args.replay}.txt").write_text(run.stderr, encoding="utf-8")
    if run.returncode:
        raise RuntimeError(f"MMGS runner failed: {run.returncode}")
    raw, vertex_ref, out_faces, face_ref, out_edges, edge_ref, corners = parse_medit(raw_mesh)
    points, incident_faces, incident_curves, curve_edges = canonicalize_and_project(
        authority, raw, out_faces, face_ref, out_edges, edge_ref, corners)
    topo = topology_report(points, out_faces, face_ref, curve_edges, incident_curves, authority)
    off = write_outputs(output, args.replay, points, raw, vertex_ref, out_faces, face_ref, curve_edges,
                        incident_faces, incident_curves, authority)
    report = {"schema": "V3GenericLiveSDFProvider/v1", "authority": authority.name,
              "provider_contains_geometry_specific_logic": False,
              "geometry_definition_module": "procedural_authorities.py",
              "custom_topology_operations": 0, "topology_engine": "stock_MMGS_5.8.0",
              "input_vertices": len(vertices), "input_triangles": len(faces),
              "output_vertices": len(points), "output_triangles": len(out_faces),
              "persistent_sheet_count": len(set(map(int, face_ref))),
              "persistent_curve_count": len(set(ref for _, _, ref in curve_edges)),
              "unknown_sheet_assignments": sum(int(ref) not in authority.sheet_names for ref in face_ref),
              "unknown_curve_assignments": sum(ref not in authority.curve_names for _, _, ref in curve_edges),
              "persistent_corner_count": sum(int(ref) in authority.corner_names for ref in vertex_ref),
              "persistent_corner_ids": sorted({
                  authority.corner_names[int(ref)] for ref in vertex_ref
                  if int(ref) in authority.corner_names
              }),
              "unknown_vertex_references": sum(
                  int(ref) > 0 and int(ref) not in authority.sheet_names
                  and int(ref) not in authority.curve_names
                  and int(ref) not in authority.corner_names for ref in vertex_ref
              ),
              **topo, "metric_multiplier": args.multiplier,
              "prism_host_metric": {
                  "enabled": prism_policy is not None,
                  "first_height_m": None if prism_policy is None else prism_policy.first_height_m,
                  "foundation_qualification_target": None if prism_policy is None else prism_policy.foundation_qualification_target,
                  "minimum_owned_layer_thickness_m": None if prism_policy is None else prism_policy.minimum_owned_layer_thickness_m,
                  "metric_response_factor": None if prism_policy is None else prism_policy.metric_response_factor,
                  "reference_shape": None if prism_policy is None else prism_policy.reference_shape,
                  **metric_statistics,
              },
              "adaptive_sdf_metric_carrier": carrier_statistics,
              "interpolation_compatible_metric_transition": transition_statistics,
              "contract_sha256": sha256(contract), "raw_mesh_sha256": sha256(raw_mesh),
              "surface_sha256": sha256(off)}
    report["pass_before_geometry_certificate"] = bool(
        topo["topology_pass"] and report["unknown_sheet_assignments"] == 0
        and report["unknown_curve_assignments"] == 0
        and report["unknown_vertex_references"] == 0
        and set(report["persistent_corner_ids"]) == set(authority.corner_names.values())
    )
    (output / f"provider_result_{args.replay}.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["pass_before_geometry_certificate"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
