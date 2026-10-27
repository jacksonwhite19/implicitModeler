"""Continuous basis-free shape-operator envelope on projected seed supports."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from procedural_authorities import POSITION_LIMIT
from shape_aware_metric import ShapeMetricPolicy
from shape_operator_metric import (interval_quadratic_majorant,
                                   operator_shape_metric, spd_majorant)


@dataclass(frozen=True)
class OperatorSupportPolicy:
    subdivision_depth: int = 2
    interpolation_guard: float = 1.0 + 128.0 * np.finfo(float).eps
    support_position_radius_m: float = POSITION_LIMIT


Interval = tuple[np.ndarray, np.ndarray]


def _out(lo, hi) -> Interval:
    return (np.nextafter(np.asarray(lo, dtype=float), -np.inf),
            np.nextafter(np.asarray(hi, dtype=float), np.inf))


def _const(value) -> Interval:
    value = np.asarray(value, dtype=float)
    return value, value


def _add(a: Interval, b: Interval) -> Interval:
    return _out(a[0]+b[0], a[1]+b[1])


def _neg(a: Interval) -> Interval:
    return _out(-a[1], -a[0])


def _sub(a: Interval, b: Interval) -> Interval:
    return _add(a, _neg(b))


def _mul(a: Interval, b: Interval) -> Interval:
    products = np.stack((a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1]))
    return _out(np.min(products, axis=0), np.max(products, axis=0))


def _reciprocal(a: Interval) -> Interval:
    if np.any((a[0] <= 0.) & (a[1] >= 0.)):
        raise ArithmeticError("interval division by a range containing zero")
    return _out(1./a[1], 1./a[0])


def _div(a: Interval, b: Interval) -> Interval:
    return _mul(a, _reciprocal(b))


def _sqrt(a: Interval) -> Interval:
    if np.any(a[0] < 0.):
        raise ArithmeticError("negative square-root interval")
    return _out(np.sqrt(np.maximum(a[0], 0.)), np.sqrt(np.maximum(a[1], 0.)))


def _hull(*values: Interval) -> Interval:
    return _out(np.minimum.reduce([v[0] for v in values]),
                np.maximum.reduce([v[1] for v in values]))


def _clip01(a: Interval) -> Interval:
    return _out(np.clip(a[0], 0., 1.), np.clip(a[1], 0., 1.))


def _sum(values: list[Interval]) -> Interval:
    result = _const(np.zeros_like(values[0][0]))
    for value in values:
        result = _add(result, value)
    return result


def _outer(a: list[Interval], b: list[Interval]) -> list[list[Interval]]:
    return [[_mul(x, y) for y in b] for x in a]


def _matmul(a: list[list[Interval]], b: list[list[Interval]]) -> list[list[Interval]]:
    return [[_sum([_mul(a[i][k], b[k][j]) for k in range(3)])
             for j in range(3)] for i in range(3)]


def _transpose(a: list[list[Interval]]) -> list[list[Interval]]:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _ellipsoid_all(x: list[Interval], axes: np.ndarray):
    inv2 = 1./(axes*axes); scale = float(np.min(axes))
    q2 = _sum([_mul(_mul(x[i], x[i]), _const(inv2[i])) for i in range(3)])
    q = _sqrt(q2)
    value = _mul(_const(scale), _sub(q, _const(1.)))
    gradient = [_div(_mul(_const(scale*inv2[i]), x[i]), q) for i in range(3)]
    q3 = _mul(_mul(q, q), q)
    bp = [_mul(_const(inv2[i]), x[i]) for i in range(3)]
    hessian = []
    for i in range(3):
        row = []
        for j in range(3):
            diagonal = _div(_const(inv2[i] if i == j else 0.), q)
            rank = _div(_mul(bp[i], bp[j]), q3)
            row.append(_mul(_const(scale), _sub(diagonal, rank)))
        hessian.append(row)
    return value, gradient, hessian


def _gradient_hessian_interval(authority, lo: np.ndarray, hi: np.ndarray,
                               sheet_ref: int):
    x = [_out(lo[i], hi[i]) for i in range(3)]
    fixture = authority.fixture
    if fixture in ("sphere", "already_qualified_smooth_surface", "narrow_gap"):
        centre, _ = authority._sphere(sheet_ref)
        q = [_sub(x[i], _const(centre[i])) for i in range(3)]
        radius = _sqrt(_sum([_mul(v, v) for v in q]))
        gradient = [_div(v, radius) for v in q]
        outer = _outer(gradient, gradient)
        hessian = [[_div(_sub(_const(1. if i == j else 0.), outer[i][j]), radius)
                    for j in range(3)] for i in range(3)]
        return gradient, hessian
    if fixture == "capped_cylinder":
        if sheet_ref != 1:
            gradient = [_const(0.), _const(0.),
                        _const(-1. if sheet_ref == 2 else 1.)]
            return gradient, [[_const(0.) for _ in range(3)] for _ in range(3)]
        rho = _sqrt(_add(_mul(x[0], x[0]), _mul(x[1], x[1])))
        radial = [_div(x[0], rho), _div(x[1], rho)]
        gradient = [radial[0], radial[1], _const(0.)]
        outer = _outer(gradient, gradient)
        hessian = [[_div(_sub(_const(1. if i == j else 0.), outer[i][j]), rho)
                    if i < 2 and j < 2 else _const(0.)
                    for j in range(3)] for i in range(3)]
        return gradient, hessian
    torus = authority._torus_parameters()
    if fixture == "concave_pocket" or torus is not None:
        major, _ = (.32, .16) if torus is None else torus
        rho = _sqrt(_add(_mul(x[0], x[0]), _mul(x[1], x[1])))
        q = _sub(rho, _const(major))
        distance = _sqrt(_add(_mul(q, q), _mul(x[2], x[2])))
        radial = [_div(x[0], rho), _div(x[1], rho)]
        fr = _div(q, distance); fz = _div(x[2], distance)
        d3 = _mul(_mul(distance, distance), distance)
        frr = _div(_mul(x[2], x[2]), d3)
        fzz = _div(_mul(q, q), d3)
        frz = _neg(_div(_mul(q, x[2]), d3))
        gradient = [_mul(fr, radial[0]), _mul(fr, radial[1]), fz]
        hessian = [[_const(0.) for _ in range(3)] for _ in range(3)]
        for i in range(2):
            for j in range(2):
                hessian[i][j] = _add(_mul(frr, _mul(radial[i], radial[j])),
                                      _mul(_div(fr, rho),
                                           _sub(_const(1. if i == j else 0.),
                                                _mul(radial[i], radial[j]))))
            hessian[i][2] = _mul(frz, radial[i]); hessian[2][i] = hessian[i][2]
        hessian[2][2] = fzz
        return gradient, hessian
    axes = authority._ellipsoid_axes()
    if axes is not None:
        _, gradient, hessian = _ellipsoid_all(x, axes)
        return gradient, hessian
    parameters = authority._blend_parameters()
    if parameters is not None:
        wing, body, width = parameters
        av, ag, ah = _ellipsoid_all(x, wing)
        bv, bg, bh = _ellipsoid_all(x, body)
        hraw = _add(_const(.5), _mul(_const(.5/width), _sub(bv, av)))
        h = _clip01(hraw); one_minus = _sub(_const(1.), h)
        gradient = [_add(_mul(h, ag[i]), _mul(one_minus, bg[i])) for i in range(3)]
        difference = [_sub(ag[i], bg[i]) for i in range(3)]
        blend_hessian = [[_sub(_add(_mul(h, ah[i][j]),
                                           _mul(one_minus, bh[i][j])),
                                  _mul(_const(1./(2.*width)),
                                       _mul(difference[i], difference[j])))
                          for j in range(3)] for i in range(3)]
        # A support box can straddle the C1 blend-band boundary.  Include the
        # exact branch Hessian in the interval hull when that occurs.
        hessian = blend_hessian
        if float(np.asarray(hraw[0])) <= 0.:
            hessian = [[_hull(hessian[i][j], bh[i][j]) for j in range(3)] for i in range(3)]
        if float(np.asarray(hraw[1])) >= 1.:
            hessian = [[_hull(hessian[i][j], ah[i][j]) for j in range(3)] for i in range(3)]
        return gradient, hessian
    raise ValueError(f"no interval differential authority for {fixture}")


def _operator_square_interval(gradient, hessian) -> Interval:
    magnitude = _sqrt(_sum([_mul(value, value) for value in gradient]))
    if float(np.asarray(magnitude[0])) <= 0.:
        raise ArithmeticError("gradient lower bound is nonpositive")
    normal = [_div(value, magnitude) for value in gradient]
    nn = _outer(normal, normal)
    projector = [[_sub(_const(1. if i == j else 0.), nn[i][j])
                  for j in range(3)] for i in range(3)]
    operator = _matmul(_matmul(projector, hessian), projector)
    operator = [[_div(operator[i][j], magnitude) for j in range(3)] for i in range(3)]
    square = _matmul(_transpose(operator), operator)
    lower = np.array([[float(np.asarray(square[i][j][0])) for j in range(3)] for i in range(3)])
    upper = np.array([[float(np.asarray(square[i][j][1])) for j in range(3)] for i in range(3)])
    lower = np.minimum(lower, lower.T); upper = np.maximum(upper, upper.T)
    return _out(lower, upper)


def _subdivide(authority, sheet_ref: int, triangle: np.ndarray):
    a, b, c = triangle
    ab = authority.project_sheet(sheet_ref, .5*(a+b))
    bc = authority.project_sheet(sheet_ref, .5*(b+c))
    ca = authority.project_sheet(sheet_ref, .5*(c+a))
    return [np.asarray([a, ab, ca]), np.asarray([ab, b, bc]),
            np.asarray([ca, bc, c]), np.asarray([ab, bc, ca])]


def _cells(authority, sheet_ref: int, triangle: np.ndarray, depth: int):
    cells = [triangle]
    for _ in range(depth):
        cells = [child for cell in cells for child in _subdivide(authority, sheet_ref, cell)]
    return cells


def _distribution(values):
    if not values:
        return {k: None for k in ("minimum", "median", "p95", "maximum")}
    a = np.asarray(values, dtype=float)
    return {"minimum": float(a.min()), "median": float(np.quantile(a, .5)),
            "p95": float(np.quantile(a, .95)), "maximum": float(a.max())}


def apply_shape_operator_support_envelopes(metrics, vertices, faces, authority,
                                           multiplier: float,
                                           shape_policy: ShapeMetricPolicy,
                                           policy: OperatorSupportPolicy):
    result = [np.asarray(metric, dtype=float).copy() for metric in metrics]
    envelopes = []
    unresolved = 0
    active_area = 0.; total_area = 0.; changed = np.zeros(len(vertices), dtype=bool)
    subdivision_cells = []; anisotropy = []; interval_radius = []
    for face in faces:
        tri = np.asarray([vertices[index].point for index in face.vertices])
        area = .5*float(np.linalg.norm(np.cross(tri[1]-tri[0], tri[2]-tri[0])))
        total_area += area
        cell_q = []
        try:
            cells = _cells(authority, face.sheet_ref, tri, policy.subdivision_depth)
            expansion = policy.support_position_radius_m/(2**policy.subdivision_depth)
            for cell in cells:
                lo = np.min(cell, axis=0)-expansion; hi = np.max(cell, axis=0)+expansion
                gradient, hessian = _gradient_hessian_interval(authority, lo, hi, face.sheet_ref)
                lower, upper = _operator_square_interval(gradient, hessian)
                q, detail = interval_quadratic_majorant(lower, upper)
                cell_q.append(q)
                interval_radius.append(detail["interval_operator_radius"])
            envelope = cell_q[0]
            for q in cell_q[1:]:
                envelope = spd_majorant(envelope, q)
            envelope *= policy.interpolation_guard
            envelopes.append(envelope)
            eig = np.linalg.eigvalsh(envelope)
            positive = eig[eig > max(1.e-18, eig.max()*1.e-12)]
            ratio = float(positive.max()/positive.min()) if len(positive) >= 2 else math.inf
            anisotropy.append(math.sqrt(ratio))
            active_area += area
            subdivision_cells.append(len(cells))
        except (ArithmeticError, ValueError, FloatingPointError):
            envelopes.append(None); unresolved += 1
    if unresolved:
        return result, {"enabled": True, "status": "Unresolved",
                        "unresolved_supports": unresolved,
                        "certified_supports": len(faces)-unresolved,
                        "mmgs_materialization_authorized": False}
    for face, envelope in zip(faces, envelopes):
        for index in face.vertices:
            proposal, _ = operator_shape_metric(
                result[index], envelope,
                authority.exact_normal(face.sheet_ref, vertices[index].point),
                shape_policy, multiplier)
            after = spd_majorant(result[index], proposal)
            changed[index] |= not np.allclose(result[index], after, rtol=1.e-13, atol=1.e-8)
            result[index] = after
    return result, {
        "enabled": True, "status": "Certified", "certified_supports": len(faces),
        "unresolved_supports": 0, "principal_vector_tracking": False,
        "continuous_tensor_family": "C(x)=S(x)^T S(x), S=P H P/|grad(phi)|",
        "quadratic_dominance": "v^T C(x) v <= v^T Q_support v",
        "interval_authority": "outward-rounded analytic SDF gradient/Hessian arithmetic",
        "tightness_objective": "minimum trace under the certified entrywise spectral-radius guard",
        "subdivision_depth": policy.subdivision_depth,
        "subdivision_cells": _distribution(subdivision_cells),
        "support_active_area_m2": active_area,
        "total_seed_area_m2": total_area,
        "support_active_area_fraction": active_area/total_area if total_area else 0.,
        "changed_seed_vertices": int(np.count_nonzero(changed)),
        "Q_tangent_anisotropy": _distribution(anisotropy),
        "interval_operator_radius": _distribution(interval_radius),
        "interpolation_guard": policy.interpolation_guard,
        "mmgs_materialization_authorized": True,
    }
