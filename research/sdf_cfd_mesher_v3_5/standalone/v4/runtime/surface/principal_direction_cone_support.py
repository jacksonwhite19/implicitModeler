"""Certified principal-line cone metric on projected CGAL seed supports."""

from __future__ import annotations

import math
import numpy as np

from principal_direction_cone import cone_majorant_2d, davis_kahan_line_cone
from procedural_authorities import POSITION_LIMIT
from shape_aware_metric import ShapeMetricPolicy
from shape_operator_metric import (normal_derivative_operator,
                                   operator_shape_metric, spd_majorant,
                                   tangent_basis)
from shape_operator_support_envelope import (_cells,
                                             _gradient_hessian_interval,
                                             _operator_square_interval)


def _distribution(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {"minimum": None, "p05": None, "median": None,
                "p95": None, "maximum": None}
    return {"minimum": float(a.min()), "p05": float(np.quantile(a, .05)),
            "median": float(np.quantile(a, .5)),
            "p95": float(np.quantile(a, .95)), "maximum": float(a.max())}


def _interval_transform_2d(lower, upper, basis):
    lo = np.zeros((2, 2)); hi = np.zeros((2, 2))
    for a in range(2):
        for b in range(2):
            for i in range(3):
                for j in range(3):
                    c = float(basis[i, a]*basis[j, b])
                    if c >= 0.:
                        lo[a, b] += c*lower[i, j]; hi[a, b] += c*upper[i, j]
                    else:
                        lo[a, b] += c*upper[i, j]; hi[a, b] += c*lower[i, j]
            lo[a, b] = math.nextafter(lo[a, b], -math.inf)
            hi[a, b] = math.nextafter(hi[a, b], math.inf)
    return np.minimum(lo, lo.T), np.maximum(hi, hi.T)


def _cell_cone(authority, sheet_ref, cell, expansion):
    centre = authority.project_sheet(sheet_ref, np.mean(cell, axis=0))
    gradient, hessian = authority.gradient_hessian(sheet_ref, centre)
    normal, _, exact_c = normal_derivative_operator(gradient, hessian)
    basis = tangent_basis(normal)
    exact = .5*(basis.T@exact_c@basis + basis.T@exact_c.T@basis)
    values, vectors = np.linalg.eigh(exact)
    lo3, hi3 = _operator_square_interval(*_gradient_hessian_interval(
        authority, np.min(cell, axis=0)-expansion,
        np.max(cell, axis=0)+expansion, sheet_ref))
    lo, hi = _interval_transform_2d(lo3, hi3, basis)
    midpoint = .5*(lo+hi); radius = .5*(hi-lo)
    epsilon = math.nextafter(float(np.linalg.norm(midpoint-exact, 2)) +
                             float(np.max(np.sum(np.abs(radius), axis=1))),
                             math.inf)
    gap = float(values[1]-values[0])
    _, beta = davis_kahan_line_cone(gap, epsilon)
    strong = math.nextafter(max(0., float(values[1])+epsilon), math.inf)
    weak = math.nextafter(max(0., float(values[0])+epsilon), math.inf)
    local, detail = cone_majorant_2d(strong, weak, beta)
    frame = np.column_stack((vectors[:, 1], vectors[:, 0]))
    q2 = frame@local@frame.T
    q3 = basis@q2@basis.T
    q3 = .5*(q3+q3.T)
    equal = abs(strong-weak) <= 256*np.finfo(float).eps*max(strong, 1.)
    return q3, {
        "status": "DirectionIrrelevant" if equal else "CertifiedCone",
        "half_angle_deg": math.degrees(beta),
        "area_inflation": float(detail["area_inflation"]),
        "spectral_gap": gap, "interval_perturbation": epsilon,
    }


def apply_principal_cone_support(metrics, vertices, faces, authority,
                                 shape_policy: ShapeMetricPolicy,
                                 subdivision_depth: int = 1):
    result = [np.asarray(m, dtype=float).copy() for m in metrics]
    changed = np.zeros(len(vertices), dtype=bool)
    widths=[]; inflation=[]; gaps=[]; perturbations=[]; statuses=[]
    expansion = POSITION_LIMIT/(2**subdivision_depth)
    for face in faces:
        tri=np.asarray([vertices[i].point for i in face.vertices])
        cells=_cells(authority, face.sheet_ref, tri, subdivision_depth)
        requirements=[]
        for cell in cells:
            q, detail=_cell_cone(authority, face.sheet_ref, cell, expansion)
            requirements.append(q)
            widths.append(detail["half_angle_deg"])
            inflation.append(detail["area_inflation"])
            gaps.append(detail["spectral_gap"])
            perturbations.append(detail["interval_perturbation"])
            statuses.append(detail["status"])
        for index in face.vertices:
            normal=authority.exact_normal(face.sheet_ref, vertices[index].point)
            combined=result[index]
            for q in requirements:
                proposal,_=operator_shape_metric(metrics[index], q, normal,
                                                 shape_policy, 1.0)
                combined=spd_majorant(combined, proposal)
            changed[index] |= not np.allclose(result[index], combined,
                                               rtol=1e-13, atol=1e-8)
            result[index]=combined
    return result, {
        "enabled": True, "status": "Certified", "unresolved_supports": 0,
        "projective_line_representation": "v is equivalent to -v",
        "spectral_bound": "Davis-Kahan sin-theta bound; full projective cone when gap is unseparated",
        "cone_majorant_objective": "minimum log determinant",
        "subdivision_depth": subdivision_depth,
        "subsupports": len(statuses),
        "status_counts": {s:statuses.count(s) for s in sorted(set(statuses))},
        "cone_half_angle_deg": _distribution(widths),
        "cone_area_inflation": _distribution(inflation),
        "spectral_gap": _distribution(gaps),
        "interval_perturbation": _distribution(perturbations),
        "changed_seed_vertices": int(np.count_nonzero(changed)),
        "mmgs_materialization_authorized": True,
    }
