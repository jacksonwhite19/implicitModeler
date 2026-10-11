"""Certified seed-support envelopes for the generic shape-aware MMGS metric.

The CGAL triangle is only a parameter carrier.  Its support is the image of
the closed barycentric simplex under the persistent-sheet projector.  A face
envelope is attached to all three CGAL vertices; incident face envelopes are
intersected, never averaged.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from procedural_authorities import (HMAX, POSITION_LIMIT, SeedVertex,
                                    curvature_size)
from prism_host_metric import PrismMetricPolicy, intersect_tangent_metric
from shape_aware_metric import (ShapeMetricPolicy, apply_shape_metric,
                                generic_geometry_metric)


@dataclass(frozen=True)
class SeedSupportEnvelopePolicy:
    subdivision_depth: int = 2
    interpolation_guard: float = 1.0 + 128.0 * np.finfo(float).eps
    support_position_radius_m: float = POSITION_LIMIT


def _spd_majorant(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Deterministic pairwise Loewner majorant in the a-normalized basis."""
    values, vectors = np.linalg.eigh(0.5 * (a + a.T))
    root = (vectors * np.sqrt(values)) @ vectors.T
    inv_root = (vectors * (1.0 / np.sqrt(values))) @ vectors.T
    relative = inv_root @ b @ inv_root
    relative = 0.5 * (relative + relative.T)
    r_values, r_vectors = np.linalg.eigh(relative)
    capped = (r_vectors * np.maximum(r_values, 1.0)) @ r_vectors.T
    result = root @ capped @ root
    return 0.5 * (result + result.T)


def _box_corners(points: np.ndarray, expansion: float) -> np.ndarray:
    lo = np.min(points, axis=0) - expansion
    hi = np.max(points, axis=0) + expansion
    return np.asarray([[x, y, z] for x in (lo[0], hi[0])
                       for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def _subdivide(authority, sheet_ref: int, tri: np.ndarray) -> list[np.ndarray]:
    a, b, c = tri
    ab = authority.project_sheet(sheet_ref, 0.5 * (a + b))
    bc = authority.project_sheet(sheet_ref, 0.5 * (b + c))
    ca = authority.project_sheet(sheet_ref, 0.5 * (c + a))
    return [np.asarray([a, ab, ca]), np.asarray([ab, b, bc]),
            np.asarray([ca, bc, c]), np.asarray([ab, bc, ca])]


def _support_cells(authority, sheet_ref: int, tri: np.ndarray,
                   depth: int) -> list[np.ndarray]:
    cells = [np.asarray(tri, dtype=float)]
    for _ in range(depth):
        cells = [child for cell in cells
                 for child in _subdivide(authority, sheet_ref, cell)]
    return cells


def _point_metric(authority, sheet_ref: int, point: np.ndarray,
                  multiplier: float, prism_policy: PrismMetricPolicy | None,
                  shape_policy: ShapeMetricPolicy) -> tuple[np.ndarray, dict]:
    vertex = SeedVertex(np.asarray(point, dtype=float), sheet_ref, 0, 0)
    metric, principal = generic_geometry_metric(authority, vertex, multiplier)
    if prism_policy is not None:
        metric, _ = intersect_tangent_metric(
            metric, authority.exact_normal(sheet_ref, point), prism_policy)
    return apply_shape_metric(metric, principal, shape_policy)


def _support_curvature_bound(authority, sheet_ref: int,
                             cells: list[np.ndarray], expansion: float) -> float:
    bounds = []
    for cell in cells:
        value = authority.curvature_upper(
            sheet_ref, _box_corners(cell, expansion))
        if not math.isfinite(value):
            return math.inf
        bounds.append(float(value))
    return math.nextafter(max(bounds, default=0.0), math.inf)


def _prism_cap(prism_policy: PrismMetricPolicy | None,
               normal: np.ndarray) -> float:
    if prism_policy is None:
        return HMAX
    _, report = intersect_tangent_metric(np.eye(3), normal, prism_policy)
    return float(report["prism_tangent_cap_m"])


def apply_seed_support_envelopes(
    metrics: list[np.ndarray], vertices, faces, authority, multiplier: float,
    prism_policy: PrismMetricPolicy | None, shape_policy: ShapeMetricPolicy,
    policy: SeedSupportEnvelopePolicy, vertex_shape_rows: list[dict] | None = None,
) -> tuple[list[np.ndarray], dict[str, object]]:
    """Intersect certified face-support shape envelopes at shared vertices.

    The interval authority supplies a spectral upper bound for every point on
    each projected support cell.  When a principal-direction cone is not part
    of the live authority, the least-trace tensor that dominates every possible
    tangent direction is the tangent-isotropic spectral cap.  This fallback is
    local to one seed support.  It is not a global scalar size policy.
    """
    result = [np.asarray(metric, dtype=float).copy() for metric in metrics]
    face_envelopes: list[np.ndarray | None] = []
    statuses: list[str] = []
    active_faces = 0
    active_area = 0.0
    total_area = 0.0
    unresolved = 0
    support_strength = []
    sample_strength = []
    correction_ratios = []
    fallback_faces = 0

    for face in faces:
        tri = np.asarray([vertices[index].point for index in face.vertices])
        area = 0.5 * float(np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])))
        total_area += area
        cells = _support_cells(authority, face.sheet_ref, tri,
                               policy.subdivision_depth)
        # Vertex point metrics were already built by the provider.  Only the
        # projected support centroid is an additional proposal point.  The
        # interval spectral cap below, not this proposal sample, is authority.
        point_metrics = [result[index] for index in face.vertices]
        point_shape = ([] if vertex_shape_rows is None else
                       [vertex_shape_rows[index] for index in face.vertices])
        any_active = any(bool(row["active"]) for row in point_shape)
        if not any_active:
            centre = authority.project_sheet(face.sheet_ref, tri.mean(axis=0))
            centre_metric, centre_shape = _point_metric(
                authority, face.sheet_ref, centre, multiplier,
                prism_policy, shape_policy)
            point_metrics.append(centre_metric)
            point_shape.append(centre_shape)
            any_active = bool(centre_shape["active"])

        # A shape-inactive support is a certified no-op only when every
        # proposal base is tangent-isotropic.  Otherwise the interval spectral
        # cap remains necessary because activation can occur between nodes.
        if not any_active:
            anisotropy = max(float(row["anisotropy_before"])
                             for row in point_shape)
            if anisotropy <= 1.0 + 256.0 * np.finfo(float).eps:
                face_envelopes.append(None)
                statuses.append("CertifiedEnvelope")
                continue

        curvature_bound = _support_curvature_bound(
            authority, face.sheet_ref, cells,
            policy.support_position_radius_m / (2 ** policy.subdivision_depth))
        if not math.isfinite(curvature_bound):
            face_envelopes.append(None)
            statuses.append("Unresolved")
            unresolved += 1
            continue

        sample = point_metrics[0]
        for metric in point_metrics[1:]:
            sample = _spd_majorant(sample, metric)
        sample_lambda = max(float(np.linalg.eigvalsh(metric).max())
                            for metric in point_metrics)

        # apply_shape_metric never requests a tangent length shorter than the
        # shortest incoming geometry/prism length.  Hence 1/h_bound^2 is a
        # certified spectral upper bound for every shape tensor on the support.
        h_bound = min(curvature_size(curvature_bound, multiplier),
                      _prism_cap(prism_policy,
                                 authority.exact_normal(face.sheet_ref,
                                                        tri.mean(axis=0))))
        lambda_bound = math.nextafter(1.0 / (h_bound * h_bound), math.inf)
        lambda_bound *= policy.interpolation_guard
        spectral_cap = lambda_bound * np.eye(3)
        envelope = _spd_majorant(sample, spectral_cap)
        face_envelopes.append(envelope)
        statuses.append("CertifiedEnvelope")
        active_faces += 1
        active_area += area
        fallback_faces += 1
        support_strength.append(lambda_bound)
        sample_strength.append(sample_lambda)
        correction_ratios.append(math.sqrt(lambda_bound / max(sample_lambda, 1e-300)))

    if unresolved:
        return result, {
            "enabled": True, "status": "Unresolved", "unresolved_supports": unresolved,
            "certified_supports": len(faces) - unresolved,
            "mmgs_materialization_authorized": False,
        }

    # Shared vertices receive the intersection of all incident hard support
    # requirements.  Applying them in canonical face order is deterministic.
    changed_vertices = np.zeros(len(vertices), dtype=bool)
    for face, envelope in zip(faces, face_envelopes):
        if envelope is None:
            continue
        for index in face.vertices:
            before = result[index]
            after = _spd_majorant(before, envelope)
            changed_vertices[index] |= not np.allclose(
                before, after, rtol=1.0e-13, atol=1.0e-8)
            result[index] = after

    def dist(values: list[float]) -> dict[str, float | None]:
        if not values:
            return {key: None for key in
                    ("minimum", "median", "p95", "maximum")}
        a = np.asarray(values, dtype=float)
        return {"minimum": float(a.min()), "median": float(np.quantile(a, .5)),
                "p95": float(np.quantile(a, .95)), "maximum": float(a.max())}

    return result, {
        "enabled": True,
        "status": "CertifiedEnvelope",
        "support_definition": "project_sheet(sheet_ref,sum(lambda_i*p_i)), lambda in closed barycentric simplex",
        "support_boundary_split_by_persistent_authority": True,
        "subdivision_depth": policy.subdivision_depth,
        "spd_dominance": "M_support - M_shape(x) positive_semidefinite",
        "least_restrictive_objective": "minimum trace under unknown tangent-direction cone; local spectral fallback",
        "rotating_direction_policy": "ambient tensor majorant; unresolved direction cone uses least-trace isotropic spectral cap on that support",
        "interpolation_guard": policy.interpolation_guard,
        "certified_supports": len(faces),
        "unresolved_supports": 0,
        "shape_support_active_faces": active_faces,
        "shape_support_active_area_m2": active_area,
        "total_seed_area_m2": total_area,
        "shape_support_active_area_fraction": active_area / total_area if total_area else 0.0,
        "direction_ambiguous_spectral_fallback_faces": fallback_faces,
        "changed_seed_vertices": int(np.count_nonzero(changed_vertices)),
        "support_spectral_strength": dist(support_strength),
        "sample_spectral_strength": dist(sample_strength),
        "support_to_sample_length_ratio": dist(correction_ratios),
        "mmgs_materialization_authorized": True,
    }
