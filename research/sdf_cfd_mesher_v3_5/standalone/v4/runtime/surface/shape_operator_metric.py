"""Basis-free shape-operator support metric utilities.

The continuous object is C=S.T@S, not an ordered pair of principal vectors.
All majorants are expressed in one ambient frame.  Equal or crossing
eigenvalues therefore need no direction correspondence.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from procedural_authorities import (NORMAL_LIMIT_RAD, curvature_size,
                                    tangent_metric, unit)
from shape_aware_metric import ShapeMetricPolicy, apply_shape_metric


@dataclass(frozen=True)
class ShapeOperatorPolicy:
    subdivision_depth: int = 2
    interval_roundoff_ulps: int = 8
    interpolation_guard: float = 1.0 + 128.0 * np.finfo(float).eps


def normal_derivative_operator(gradient: np.ndarray,
                               hessian: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return n, S=P H P/|g|, and C=S.T S for a length-valued SDF.

    For a tangent input, differentiating n=g/|g| gives Dn=P H/|g|.
    The right projector makes the ambient extension vanish on n; it changes
    nothing on tangent inputs.  The sign convention disappears in C.
    """
    gradient = np.asarray(gradient, dtype=float)
    hessian = 0.5 * (np.asarray(hessian, dtype=float) +
                     np.asarray(hessian, dtype=float).T)
    magnitude = float(np.linalg.norm(gradient))
    if not magnitude > 0.0:
        raise ValueError("zero SDF gradient")
    normal = gradient / magnitude
    projector = np.eye(3) - np.outer(normal, normal)
    operator = projector @ hessian @ projector / magnitude
    operator = 0.5 * (operator + operator.T)
    square = operator.T @ operator
    return normal, operator, 0.5 * (square + square.T)


def psd_min(matrix: np.ndarray) -> float:
    return float(np.linalg.eigvalsh(0.5 * (matrix + matrix.T)).min())


def spd_majorant(a: np.ndarray, b: np.ndarray,
                 floor: float = 1.0e-18) -> np.ndarray:
    """Deterministic Loewner majorant of two symmetric PSD tensors."""
    a = 0.5 * (np.asarray(a, dtype=float) + np.asarray(a, dtype=float).T)
    b = 0.5 * (np.asarray(b, dtype=float) + np.asarray(b, dtype=float).T)
    values, vectors = np.linalg.eigh(a)
    values = np.maximum(values, floor)
    root = (vectors * np.sqrt(values)) @ vectors.T
    inv_root = (vectors * (1.0 / np.sqrt(values))) @ vectors.T
    relative = inv_root @ b @ inv_root
    relative = 0.5 * (relative + relative.T)
    r_values, r_vectors = np.linalg.eigh(relative)
    cap = (r_vectors * np.maximum(r_values, 1.0)) @ r_vectors.T
    result = root @ cap @ root
    return 0.5 * (result + result.T)


def interval_quadratic_majorant(lower: np.ndarray,
                                upper: np.ndarray) -> tuple[np.ndarray, dict]:
    """Majorize every symmetric matrix in an entrywise interval hull.

    If C=C0+Delta and |Delta_ij|<=R_ij, then
    ||Delta||_2<=||R||_infinity.  Hence Q=C0+rho*I dominates C in quadratic
    form.  This construction is basis-free and outward conservative once the
    entry intervals are outward conservative.
    """
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    centre = 0.5 * (lower + upper)
    centre = 0.5 * (centre + centre.T)
    radius = 0.5 * (upper - lower)
    radius = np.maximum(radius, radius.T)
    rho = float(np.max(np.sum(np.abs(radius), axis=1)))
    q = centre + math.nextafter(rho, math.inf) * np.eye(centre.shape[0])
    values, vectors = np.linalg.eigh(0.5 * (q + q.T))
    q = (vectors * np.maximum(values, 0.0)) @ vectors.T
    return 0.5 * (q + q.T), {
        "centre_eigenvalues": np.linalg.eigvalsh(centre).tolist(),
        "interval_operator_radius": rho,
        "majorant_eigenvalues": np.linalg.eigvalsh(q).tolist(),
        "objective": "minimum trace for the certified entrywise spectral-radius guard",
    }


def tangent_basis(normal: np.ndarray) -> np.ndarray:
    normal = unit(np.asarray(normal, dtype=float))
    reference = (np.array([0.0, 0.0, 1.0]) if abs(normal[2]) < 0.9
                 else np.array([0.0, 1.0, 0.0]))
    first = unit(np.cross(normal, reference))
    second = unit(np.cross(normal, first))
    return np.column_stack((first, second))


def operator_shape_metric(base_metric: np.ndarray, q_support: np.ndarray,
                          normal: np.ndarray,
                          policy: ShapeMetricPolicy,
                          multiplier: float = 1.0) -> tuple[np.ndarray, dict]:
    """Apply the established chord-plane rule using a support operator bound.

    Q has units 1/m^2 and bounds squared normal derivative.  Its tangent
    eigenvalues therefore provide curvature-magnitude caps without tracking
    pointwise principal-vector identities.  The existing shape rule then
    includes the qualified MMGS response reserve and output triangle
    conditioning envelope.
    """
    basis = tangent_basis(normal)
    tangent_q = basis.T @ q_support @ basis
    tangent_q = 0.5 * (tangent_q + tangent_q.T)
    values, vectors = np.linalg.eigh(tangent_q)
    values = np.maximum(values, 0.0)
    directions = tuple(unit(basis @ vectors[:, i]) for i in range(2))
    curvature_caps = tuple(math.sqrt(float(value)) for value in values)
    geometry_lengths = tuple(curvature_size(2.5 * value, multiplier)
                             for value in curvature_caps)
    support_geometry = tangent_metric(
        directions[0], geometry_lengths[0], directions[1], geometry_lengths[1],
        normal, min(geometry_lengths))
    base_metric = spd_majorant(base_metric, support_geometry)
    principal = {
        "directions": directions,
        # The established pointwise authority carries a 2.50 reserve on the
        # differential curvature.  Preserve it here; this task does not retune
        # that frozen surface policy.
        "curvatures": tuple(2.5 * value for value in curvature_caps),
        "normal": unit(normal),
    }
    metric, report = apply_shape_metric(base_metric, principal, policy)
    report.update({
        "basis_free_operator": True,
        "support_C_eigenvalues": values.tolist(),
        "support_curvature_caps": list(curvature_caps),
        "reserved_curvature_caps": list(principal["curvatures"]),
        "support_geometry_lengths_m": list(geometry_lengths),
        "normal_angle_limit_deg": math.degrees(NORMAL_LIMIT_RAD),
    })
    return metric, report
