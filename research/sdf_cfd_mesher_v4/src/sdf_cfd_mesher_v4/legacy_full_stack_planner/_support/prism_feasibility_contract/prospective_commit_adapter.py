"""Generic fail-closed admission for one full conformal prism layer.

This module does not generate trajectories or alter a mesh.  A caller supplies
the frozen bottom surface, one prospective displacement, and the independent
SDF/topology audit results.  The candidate exists only in transient arrays.
The caller's mutation callback runs only when every required gate passes.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Callable, Mapping

import numpy as np


FOUNDATION_DETERMINANT_MINIMUM = 0.001

REQUIRED_ANALYTIC_GATES = (
    "identity_and_trajectory_authority",
    "smooth_offset_regularity",
    "outward_fluid_branch_clearance",
)

REQUIRED_GLOBAL_GATES = (
    "trajectory_and_edge_crossing_free",
    "trajectory_to_nonincident_face_clear",
    "candidate_face_intersection_free",
    "nonincident_positive_volume_overlap_free",
    "sdf_fluid_side_and_branch_ownership",
    "persistent_identity_conformal",
)


@dataclass(frozen=True)
class CandidateLayerDecision:
    commit: bool
    source_faces: int
    candidate_prisms: int
    minimum_top_orientation_cosine: float
    top_orientation_failures: int
    minimum_exact_wedge_jacobian: float
    nonpositive_exact_wedge_jacobians: int
    minimum_signed_volume: float
    nonpositive_signed_volumes: int
    minimum_foundation_determinant: float
    foundation_determinant_failures: int
    analytic_gate_failures: tuple[str, ...]
    global_gate_failures: tuple[str, ...]
    unproven_required_gates: tuple[str, ...]
    rejection_reasons: tuple[str, ...]
    mutation_callback_invoked: bool = False

    def to_dict(self) -> dict:
        return asdict(self)


def _det_symm(matrix: np.ndarray) -> np.ndarray:
    xx, xy, xz = matrix[..., 0, 0], matrix[..., 0, 1], matrix[..., 0, 2]
    yy, yz, zz = matrix[..., 1, 1], matrix[..., 1, 2], matrix[..., 2, 2]
    return np.abs(
        xx * yy * zz + 2 * xy * xz * yz
        - xx * yz * yz - yy * xz * xz - zz * xy * xy
    )


def _face_vectors(prisms: np.ndarray) -> np.ndarray:
    # Official type-6 node order.  The lower cap is oriented outward.
    patterns = ([0, 2, 1], [3, 4, 5], [0, 1, 4, 3], [1, 2, 5, 4], [2, 0, 3, 5])
    vectors = []
    for pattern in patterns:
        polygon = prisms[:, pattern, :]
        vectors.append(
            0.5 * np.sum(
                np.cross(polygon, np.roll(polygon, -1, axis=1)), axis=1
            )
        )
    return np.stack(vectors, axis=1)


def _foundation_determinant(
    prisms: np.ndarray, *, lower_cap_is_uncoupled_wall: bool
) -> np.ndarray:
    vectors = _face_vectors(prisms)
    mask = np.ones((len(prisms), 5), dtype=np.float64)
    if lower_cap_is_uncoupled_wall:
        mask[:, 0] = 0.0
    magnitudes = np.linalg.norm(vectors, axis=2)
    average = np.sum(magnitudes * mask, axis=1) / np.sum(mask, axis=1)
    normalized = vectors / average[:, None, None]
    tensor = np.einsum("nfi,nfj,nf->nij", normalized, normalized, mask)
    return _det_symm(tensor)


def _exact_wedge_jacobian_and_volume(prisms: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return the exact minimum type-6 Jacobian and signed volume.

    The determinant is affine in the triangular reference coordinates, so its
    minimum is on a vertical reference edge.  Each edge restriction is a
    quadratic in the extrusion coordinate.  Endpoints and an interior convex
    stationary point therefore give the complete minimum.
    """
    p0, p1, p2, p3, p4, p5 = [prisms[:, i, :] for i in range(6)]
    a0 = p1 - p0
    a1 = (p4 - p3) - a0
    b0 = p2 - p0
    b1 = (p5 - p3) - b0
    q0 = np.cross(a0, b0)
    q1 = np.cross(a1, b0) + np.cross(a0, b1)
    q2 = np.cross(a1, b1)
    vertical = np.stack((p3 - p0, p4 - p1, p5 - p2), axis=1)
    coefficients = np.stack(
        (
            np.einsum("nij,nj->ni", vertical, q0),
            np.einsum("nij,nj->ni", vertical, q1),
            np.einsum("nij,nj->ni", vertical, q2),
        ),
        axis=2,
    )
    values = [coefficients[:, :, 0], np.sum(coefficients, axis=2)]
    denominator = 2.0 * coefficients[:, :, 2]
    stationary = np.divide(
        -coefficients[:, :, 1],
        denominator,
        out=np.full_like(denominator, -1.0),
        where=denominator != 0.0,
    )
    interior = (
        (stationary > 0.0)
        & (stationary < 1.0)
        & (coefficients[:, :, 2] > 0.0)
    )
    stationary_value = (
        coefficients[:, :, 0]
        + coefficients[:, :, 1] * stationary
        + coefficients[:, :, 2] * stationary * stationary
    )
    values.append(np.where(interior, stationary_value, np.inf))
    minimum = np.min(np.stack(values, axis=2), axis=(1, 2))
    volume = np.einsum(
        "ni,ni->n",
        np.sum(vertical, axis=1) / 6.0,
        q0 + 0.5 * q1 + q2 / 3.0,
    )
    return minimum, volume


def evaluate_full_layer_candidate(
    bottom_points: np.ndarray,
    faces: np.ndarray,
    trajectories: np.ndarray,
    displacement: float,
    *,
    lower_cap_is_uncoupled_wall: bool,
    analytic_gates: Mapping[str, bool | None],
    global_gates: Mapping[str, bool | None],
    chunk_faces: int = 100_000,
) -> CandidateLayerDecision:
    """Evaluate a complete proposed layer without committing it."""
    bottom_points = np.asarray(bottom_points, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    trajectories = np.asarray(trajectories, dtype=np.float64)
    if bottom_points.ndim != 2 or bottom_points.shape[1] != 3:
        raise ValueError("bottom_points must have shape (N,3)")
    if trajectories.shape != bottom_points.shape:
        raise ValueError("trajectories must match bottom_points")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError("faces must have shape (M,3)")
    if not np.isfinite(displacement) or displacement <= 0.0:
        raise ValueError("displacement must be finite and positive")
    if len(faces) == 0 or len(bottom_points) == 0:
        raise ValueError("candidate layer must be nonempty")
    if np.min(faces) < 0 or np.max(faces) >= len(bottom_points):
        raise ValueError("face index outside bottom_points")

    top_points = bottom_points + float(displacement) * trajectories
    finite_authority = bool(
        np.all(np.isfinite(bottom_points))
        and np.all(np.isfinite(top_points))
        and np.all(np.isfinite(trajectories))
        and np.all(np.linalg.norm(trajectories, axis=1) > 0.0)
    )

    analytic = dict(analytic_gates)
    if analytic.get("identity_and_trajectory_authority") is True and not finite_authority:
        analytic["identity_and_trajectory_authority"] = False

    minimum_top_cosine = np.inf
    minimum_jacobian = np.inf
    minimum_volume = np.inf
    minimum_determinant = np.inf
    top_failures = 0
    jacobian_failures = 0
    volume_failures = 0
    determinant_failures = 0

    for start in range(0, len(faces), int(chunk_faces)):
        selected = faces[start : start + int(chunk_faces)]
        bottom = bottom_points[selected]
        top = top_points[selected]
        prisms = np.concatenate((bottom, top), axis=1)

        bottom_cross = np.cross(bottom[:, 1] - bottom[:, 0], bottom[:, 2] - bottom[:, 0])
        top_cross = np.cross(top[:, 1] - top[:, 0], top[:, 2] - top[:, 0])
        denominator = np.linalg.norm(bottom_cross, axis=1) * np.linalg.norm(top_cross, axis=1)
        cosine = np.divide(
            np.einsum("ni,ni->n", bottom_cross, top_cross),
            denominator,
            out=np.full(len(selected), -np.inf),
            where=denominator > 0.0,
        )
        jacobian, volume = _exact_wedge_jacobian_and_volume(prisms)
        determinant = _foundation_determinant(
            prisms, lower_cap_is_uncoupled_wall=lower_cap_is_uncoupled_wall
        )

        minimum_top_cosine = min(minimum_top_cosine, float(np.min(cosine)))
        minimum_jacobian = min(minimum_jacobian, float(np.min(jacobian)))
        minimum_volume = min(minimum_volume, float(np.min(volume)))
        minimum_determinant = min(minimum_determinant, float(np.min(determinant)))
        top_failures += int(np.sum(cosine <= 0.0))
        jacobian_failures += int(np.sum(jacobian <= 0.0))
        volume_failures += int(np.sum(volume <= 0.0))
        determinant_failures += int(
            np.sum(determinant < FOUNDATION_DETERMINANT_MINIMUM)
        )

    analytic_failures = tuple(
        name for name in REQUIRED_ANALYTIC_GATES if analytic.get(name) is False
    )
    global_failures = tuple(
        name for name in REQUIRED_GLOBAL_GATES if global_gates.get(name) is False
    )
    unproven = tuple(
        name
        for name in (*REQUIRED_ANALYTIC_GATES, *REQUIRED_GLOBAL_GATES)
        if (analytic if name in REQUIRED_ANALYTIC_GATES else global_gates).get(name) is not True
        and name not in analytic_failures
        and name not in global_failures
    )

    reasons: list[str] = []
    if analytic_failures:
        reasons.append("AnalyticAuthorityOrFeasibility")
    if top_failures:
        reasons.append("TopFaceOrientation")
    if jacobian_failures:
        reasons.append("ExactType6Jacobian")
    if volume_failures:
        reasons.append("SignedVolume")
    if determinant_failures:
        reasons.append("FoundationDeterminant")
    if global_failures:
        reasons.append("GlobalGeometryOrIdentity")
    if unproven:
        reasons.append("UnprovenRequiredGate")

    # Strictly positive complete wedge Jacobian also rules out a local
    # differential fold of any bilinear side.  Global side/face intersections
    # remain in the independent global gates above.
    commit = not reasons
    return CandidateLayerDecision(
        commit=commit,
        source_faces=len(faces),
        candidate_prisms=len(faces),
        minimum_top_orientation_cosine=minimum_top_cosine,
        top_orientation_failures=top_failures,
        minimum_exact_wedge_jacobian=minimum_jacobian,
        nonpositive_exact_wedge_jacobians=jacobian_failures,
        minimum_signed_volume=minimum_volume,
        nonpositive_signed_volumes=volume_failures,
        minimum_foundation_determinant=minimum_determinant,
        foundation_determinant_failures=determinant_failures,
        analytic_gate_failures=analytic_failures,
        global_gate_failures=global_failures,
        unproven_required_gates=unproven,
        rejection_reasons=tuple(reasons),
    )


def commit_atomically(
    decision: CandidateLayerDecision, mutation_callback: Callable[[], None]
) -> CandidateLayerDecision:
    """Invoke the mutation only for a fully admitted candidate."""
    if not decision.commit:
        return decision
    mutation_callback()
    values = decision.to_dict()
    values["mutation_callback_invoked"] = True
    return CandidateLayerDecision(**values)

