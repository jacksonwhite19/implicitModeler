"""Generic one-pass tangent metric for planar-normal conditioning."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from procedural_authorities import NORMAL_LIMIT_RAD, curvature_size, tangent_metric


@dataclass(frozen=True)
class ShapeMetricPolicy:
    normal_qualification_fraction: float = 0.90
    mmgs_response_reserve: float = 5.5
    curvature_reserve_is_embedded: bool = True
    normal_limit_rad: float = NORMAL_LIMIT_RAD


def directional_length(metric: np.ndarray, direction: np.ndarray) -> float:
    direction = direction / float(np.linalg.norm(direction))
    return 1.0 / math.sqrt(float(direction @ metric @ direction))


def directional_cap(metric: np.ndarray, direction: np.ndarray, length: float) -> np.ndarray:
    direction = direction / float(np.linalg.norm(direction))
    target = 1.0 / max(length, 1.0e-9) ** 2
    current = float(direction @ metric @ direction)
    if current >= target:
        return metric
    return metric + (target - current) * np.outer(direction, direction)


def generic_geometry_metric(authority, vertex, multiplier: float) -> tuple[np.ndarray, dict]:
    d0, d1, k0, k1, normal = authority._normal_map_principal_data(
        vertex.sheet_ref, vertex.point)
    h0 = curvature_size(k0, multiplier)
    h1 = curvature_size(k1, multiplier)
    metric = tangent_metric(d0, h0, d1, h1, normal, min(h0, h1))
    return metric, {"directions": (d0, d1), "curvatures": (k0, k1),
                    "geometry_lengths": (h0, h1), "normal": normal}


def generic_geometry_metric_from_differential(authority, vertex,
                                              multiplier: float) -> tuple[np.ndarray, dict]:
    """Fast direct-SDF metric query for the pre-MMGS carrier oracle.

    It follows the same V3 2.50 differential reserve as the pointwise metric,
    but obtains the local normal derivative directly from grad(phi) and H(phi)
    instead of finite-differencing projected normals.  This function is not
    used to write the final MMGS metric contract.
    """
    gradient, hessian = authority.gradient_hessian(vertex.sheet_ref, vertex.point)
    magnitude = float(np.linalg.norm(gradient))
    normal = gradient / magnitude
    axis = np.array([0., 0., 1.]) if abs(normal[2]) < .9 else np.array([0., 1., 0.])
    d0 = np.cross(normal, axis); d0 /= float(np.linalg.norm(d0))
    d1 = np.cross(normal, d0); d1 /= float(np.linalg.norm(d1))
    basis = np.column_stack((d0, d1))
    projector = np.eye(3) - np.outer(normal, normal)
    operator = projector @ hessian @ projector / magnitude
    block = 0.5 * (basis.T @ operator @ basis + basis.T @ operator.T @ basis)
    values, vectors = np.linalg.eigh(block)
    directions = tuple((basis @ vectors[:, i]) / np.linalg.norm(basis @ vectors[:, i])
                       for i in range(2))
    curvatures = tuple(2.50 * abs(float(values[i])) for i in range(2))
    lengths = tuple(curvature_size(value, multiplier) for value in curvatures)
    metric = tangent_metric(directions[0], lengths[0], directions[1], lengths[1],
                            normal, min(lengths))
    return metric, {"directions": directions, "curvatures": curvatures,
                    "geometry_lengths": lengths, "normal": normal,
                    "oracle_direct_differential": True}


def apply_shape_metric(metric: np.ndarray, principal: dict,
                       policy: ShapeMetricPolicy) -> tuple[np.ndarray, dict]:
    """Intersect M with a chord-plane tangent-conditioning term.

    For a tangent edge matrix E and normal-offset vector b, the chord-plane
    slope is ||E^-T b||.  With local shape-operator norm kappa,
    ||b|| <= kappa*L^2/sqrt(2), and ||E^-T|| <= sqrt(2)/a, giving
    tan(theta) <= kappa*L^2/a.  The measured MMGS response reserve bounds the
    product of output metric-edge inflation and tangent-aspect amplification.

    The existing M_normal is accepted for an isotropic metric.  M_shape acts on
    excess directional scale: it limits the longer tangent length so the
    additional chord-plane amplification above the isotropic reference remains
    within the qualification angle.  This avoids globally shrinking already
    isotropic certified surfaces.
    """
    directions = principal["directions"]
    curvatures = principal["curvatures"]
    lengths = [directional_length(metric, direction) for direction in directions]
    short_index = int(np.argmin(lengths)); long_index = 1 - short_index
    short = lengths[short_index]; long = lengths[long_index]
    kappa = max(abs(float(value)) for value in curvatures)
    theta = policy.normal_qualification_fraction * policy.normal_limit_rad
    active = False
    requested_long = long
    predicted_before = 0.0
    predicted_after = 0.0
    if kappa > 0.0 and long > short * (1.0 + 1.0e-12):
        # Only the anisotropic excess is charged to M_shape.  The isotropic
        # reference remains governed by the already-qualified M_normal term.
        excess_ratio = long / short
        predicted_before = (kappa * policy.mmgs_response_reserve *
                            long * (excess_ratio - 1.0))
        available = math.tan(theta)
        # Solve kappa*R*h_long*(h_long/h_short - 1) <= tan(theta).
        coefficient = kappa * policy.mmgs_response_reserve / short
        requested_long = (short + math.sqrt(short * short +
                          4.0 * available / coefficient)) * 0.5
        requested_long = min(long, requested_long)
        if requested_long < long:
            metric = directional_cap(metric, directions[long_index], requested_long)
            active = True
        ratio_after = requested_long / short
        predicted_after = (kappa * policy.mmgs_response_reserve *
                           requested_long * max(0.0, ratio_after - 1.0))
    return metric, {
        "active": active,
        "short_direction": short_index,
        "long_direction": long_index,
        "short_length_m": short,
        "long_length_before_m": long,
        "long_length_after_m": requested_long,
        "anisotropy_before": long / short,
        "anisotropy_after": requested_long / short,
        "reserved_excess_chord_slope_before": predicted_before,
        "reserved_excess_chord_slope_after": predicted_after,
        "qualification_tangent": math.tan(theta),
        "mmgs_response_reserve": policy.mmgs_response_reserve,
    }
