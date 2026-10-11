"""Generic prism-host contribution for the V3 tangent SPD surface metric.

The metric is only a sizing request.  Stock MMGS owns all topology changes and
the exact prospective prism evaluator remains the acceptance authority.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from fractions import Fraction
import math
from pathlib import Path
import sys

import numpy as np

V3_ROOT = Path(__file__).resolve().parent.parent
if str(V3_ROOT) not in sys.path:
    sys.path.insert(0, str(V3_ROOT))

from prism_feasibility_contract.prospective_commit_adapter import (
    _foundation_determinant,
)


@dataclass(frozen=True)
class PrismMetricPolicy:
    first_height_m: float
    foundation_qualification_target: float
    minimum_owned_layer_thickness_m: float | None = None
    metric_response_factor: float = 1.0
    reference_shape: str = "right_isosceles_parallel_trajectory"


def _unit(v: np.ndarray) -> np.ndarray:
    value = np.asarray(v, dtype=np.float64)
    length = float(np.linalg.norm(value))
    if not length > 0.0:
        raise ValueError("zero direction")
    return value / length


def foundation_reference_determinant(
    tangent_to_height_ratio: float, *, lower_cap_is_uncoupled_wall: bool = True
) -> float:
    """Exact Foundation value for the frozen right-isosceles reference prism."""
    ratio = float(tangent_to_height_ratio)
    bottom = np.asarray(
        [[0.0, 0.0, 0.0], [ratio, 0.0, 0.0], [0.0, ratio, 0.0]],
        dtype=np.float64,
    )
    top = bottom + np.asarray([0.0, 0.0, 1.0])
    prism = np.concatenate((bottom, top), axis=0)[None, :, :]
    return float(
        _foundation_determinant(
            prism, lower_cap_is_uncoupled_wall=lower_cap_is_uncoupled_wall
        )[0]
    )


@lru_cache(maxsize=16)
def tangent_to_height_ratio(target: float, lower_cap_is_uncoupled_wall: bool = True) -> float:
    """Largest certified reference ratio whose exact determinant meets target."""
    if not 0.0 < target < 1.0:
        raise ValueError("foundation target must lie in (0,1)")
    lower, upper = 1.0, 1024.0
    if foundation_reference_determinant(
        lower, lower_cap_is_uncoupled_wall=lower_cap_is_uncoupled_wall
    ) < target:
        raise RuntimeError("reference lower bracket does not pass")
    while foundation_reference_determinant(
        upper, lower_cap_is_uncoupled_wall=lower_cap_is_uncoupled_wall
    ) >= target:
        upper *= 2.0
    for _ in range(96):
        middle = 0.5 * (lower + upper)
        if foundation_reference_determinant(
            middle, lower_cap_is_uncoupled_wall=lower_cap_is_uncoupled_wall
        ) >= target:
            lower = middle
        else:
            upper = middle
    # Return the passing side of the bracket.  The final projected mesh is
    # independently evaluated; this value is not itself an acceptance proof.
    return lower


def _tangent_basis(normal: np.ndarray) -> np.ndarray:
    n = _unit(normal)
    reference = np.asarray([0.0, 0.0, 1.0])
    if abs(float(np.dot(n, reference))) > 0.9:
        reference = np.asarray([0.0, 1.0, 0.0])
    first = _unit(np.cross(n, reference))
    second = _unit(np.cross(n, first))
    return np.column_stack((first, second))


def _exact_psd(matrix: list[list[Fraction]]) -> bool:
    """Construction predicate on binary64 inputs; final acceptance stays separate."""
    a, b, c = matrix[0]
    _, d, e = matrix[1]
    _, _, f = matrix[2]
    return (min(a, d, f) >= 0 and a*d-b*b >= 0 and a*f-c*c >= 0
            and d*f-e*e >= 0
            and a*(d*f-e*e)-b*(b*f-c*e)+c*(b*e-c*d) >= 0)


def _preserve_geometry(result: np.ndarray, required: np.ndarray) -> tuple[np.ndarray, dict]:
    """Strengthen a candidate by an exact, outward-rounded diagonal addition.

    If R-M is already PSD, preserve R bit for bit. Otherwise the exact
    Gershgorin row bound s makes (R-M)+s*I symmetric diagonally dominant with
    nonnegative diagonal, hence PSD. Rounding each R_ii+s upward adds another
    nonnegative diagonal. The result therefore majorizes both M and R.
    There is no tolerance, guessed reserve or acceptance report here.
    """
    if not np.isfinite(result).all():
        raise ValueError('NonfinitePreservedMetric')
    delta = [[Fraction(float(result[i,j]))-Fraction(float(required[i,j]))
              for j in range(3)] for i in range(3)]
    if _exact_psd(delta):
        return result, dict(geometry_preservation='ComputedGeometryMajorant/1',
                            geometry_preservation_diagonals_changed=0)
    shift = max(Fraction(0), *(sum(abs(delta[i][j]) for j in range(3) if i != j)
                               -delta[i][i] for i in range(3)))
    strengthened = result.copy()
    for i in range(3):
        target = Fraction(float(result[i,i])) + shift
        try:
            rounded = float(target)
        except OverflowError as error:
            raise ValueError('UnrepresentablePreservedMetric') from error
        if not math.isfinite(rounded):
            raise ValueError('UnrepresentablePreservedMetric')
        if Fraction(rounded) < target:
            rounded = float(np.nextafter(rounded, math.inf))
        if not math.isfinite(rounded):
            raise ValueError('UnrepresentablePreservedMetric')
        strengthened[i,i] = rounded
    return strengthened, dict(geometry_preservation='ComputedGeometryMajorant/1',
        geometry_preservation_diagonals_changed=int(np.count_nonzero(np.diag(strengthened) != np.diag(result))))


def intersect_tangent_metric(
    geometry_metric: np.ndarray,
    normal: np.ndarray,
    policy: PrismMetricPolicy,
    *, preserve_required_geometry: bool = False,
    include_reference_diagnostics: bool = True,
) -> tuple[np.ndarray, dict[str, object]]:
    """Intersect a geometry SPD tensor with the generic prism tangent cap.

    The geometry tensor is diagonalized only in the exact tangent plane.  Each
    geometry tangent size is capped independently.  Therefore a direction that
    is already fine enough is not refined when only its orthogonal direction
    consumes prism-conditioning margin.
    """
    matrix = np.asarray(geometry_metric, dtype=np.float64)
    if type(include_reference_diagnostics) is not bool:
        raise ValueError('ExplicitReferenceDiagnosticRoleRequired')
    if type(preserve_required_geometry) is not bool:
        raise ValueError('ExplicitMetricPreservationContractRequired')
    if preserve_required_geometry:
        if (matrix.shape != (3,3) or not np.isfinite(matrix).all()
                or not np.array_equal(matrix, matrix.T)):
            raise ValueError('FiniteSymmetricRequiredMetric')
        exact = [[Fraction(float(v)) for v in row] for row in matrix]
        a,b,c=exact[0];_,d,e=exact[1];_,_,f=exact[2]
        if not (a>0 and a*d-b*b>0 and a*(d*f-e*e)-b*(b*f-c*e)+c*(b*e-c*d)>0):
            raise ValueError('RequiredGeometryMetricNotSPD')
        direction = np.asarray(normal, dtype=np.float64)
        if direction.shape != (3,) or not np.isfinite(direction).all():
            raise ValueError('FiniteMetricNormalRequired')
    if matrix.shape != (3, 3) or not np.allclose(matrix, matrix.T, atol=1.0e-12):
        raise ValueError("geometry metric must be symmetric 3x3")
    n = _unit(normal)
    tangent = _tangent_basis(n)
    tangent_block = tangent.T @ matrix @ tangent
    eigenvalues, eigenvectors = np.linalg.eigh(tangent_block)
    if not np.all(eigenvalues > 0.0):
        raise ValueError("geometry tangent metric is not SPD")
    directions = tangent @ eigenvectors
    geometry_sizes = 1.0 / np.sqrt(eigenvalues)
    wall_ratio = tangent_to_height_ratio(
        policy.foundation_qualification_target, True
    )
    wall_cap = wall_ratio * float(policy.first_height_m)
    owned_ratio = tangent_to_height_ratio(
        policy.foundation_qualification_target, False
    )
    owned_cap = np.inf
    if policy.minimum_owned_layer_thickness_m is not None:
        if not policy.minimum_owned_layer_thickness_m > 0.0:
            raise ValueError("minimum owned layer thickness must be positive")
        owned_cap = owned_ratio * float(policy.minimum_owned_layer_thickness_m)
    if not 0.0 < policy.metric_response_factor <= 1.0:
        raise ValueError("metric response factor must lie in (0,1]")
    reference_cap = min(wall_cap, owned_cap)
    prism_cap = reference_cap * float(policy.metric_response_factor)
    final_sizes = np.minimum(geometry_sizes, prism_cap)

    result = np.zeros((3, 3), dtype=np.float64)
    for direction, size in zip(directions.T, final_sizes):
        result += np.outer(direction, direction) / float(size * size)
    normal_size = float(np.min(final_sizes))
    result += np.outer(n, n) / (normal_size * normal_size)
    result = 0.5 * (result + result.T)
    if not np.all(np.linalg.eigvalsh(result) > 0.0):
        raise RuntimeError("intersected prism metric is not SPD")
    preservation = {}
    if preserve_required_geometry:
        result, preservation = _preserve_geometry(result, matrix)

    changed = geometry_sizes > prism_cap * (1.0 + 32.0 * np.finfo(float).eps)
    return result, {
        "geometry_tangent_sizes_m": sorted(map(float, geometry_sizes)),
        "final_tangent_sizes_m": sorted(map(float, final_sizes)),
        "prism_tangent_cap_m": float(prism_cap),
        "wall_tangent_to_height_ratio": float(wall_ratio),
        "owned_layer_tangent_to_height_ratio": float(owned_ratio),
        "wall_tangent_cap_m": float(wall_cap),
        "owned_layer_tangent_cap_m": None if not np.isfinite(owned_cap) else float(owned_cap),
        "reference_tangent_cap_m": float(reference_cap),
        "metric_response_factor": float(policy.metric_response_factor),
        "directions_refined": int(np.sum(changed)),
        "prism_metric_active": bool(np.any(changed)),
        "wall_reference_determinant": foundation_reference_determinant(
            wall_ratio, lower_cap_is_uncoupled_wall=True
        ) if include_reference_diagnostics else None,
        "owned_layer_reference_determinant": foundation_reference_determinant(
            owned_ratio, lower_cap_is_uncoupled_wall=False
        ) if include_reference_diagnostics else None,
        **preservation,
    }
