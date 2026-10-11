"""Certified unoriented principal-line cones and least-area SPD majorants."""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import minimize_scalar


def line_angle(first: np.ndarray, second: np.ndarray) -> float:
    """Acute angle between unoriented lines; v and -v are identical."""
    first = np.asarray(first, dtype=float); second = np.asarray(second, dtype=float)
    first /= np.linalg.norm(first); second /= np.linalg.norm(second)
    return math.acos(float(np.clip(abs(np.dot(first, second)), 0., 1.)))


def davis_kahan_line_cone(gap: float, perturbation: float) -> tuple[str, float]:
    """Return a conservative line cone from a symmetric perturbation bound.

    If the reference eigenvalue gap is delta and ||E||<=eps, the separation
    from the other perturbed eigenvalue is at least delta-eps.  The sin-theta
    theorem gives sin(theta)<=eps/(delta-eps).  When delta<=2eps, the line is
    not continuously identifiable and direction must be treated as irrelevant
    or unresolved from eigenvalue-magnitude evidence.
    """
    gap = float(gap); perturbation = float(max(0., perturbation))
    if gap <= 2.*perturbation:
        # The eigenspace is not identifiable, but the complete projective line
        # space is still a certified cone. Direction is called irrelevant only
        # when the directional metric strengths themselves agree.
        return "CertifiedCone", .5*math.pi
    ratio = min(1., perturbation/(gap-perturbation))
    return "CertifiedCone", math.asin(ratio)


def cone_majorant_2d(strong: float, weak: float, half_angle: float) -> tuple[np.ndarray, dict]:
    """Minimum-area diagonal majorant for an unoriented symmetric line cone.

    The family is weak*I + (strong-weak) u(alpha)u(alpha)^T for
    |alpha|<=half_angle.  Symmetry makes a diagonal majorant optimal in the
    cone-centre frame.  For A=diag(a,b) to dominate u u^T it is necessary and
    sufficient that cos(alpha)^2/a + sin(alpha)^2/b <= 1.  The endpoint is the
    worst angle.  A deterministic bounded scalar solve minimizes log det of
    weak*I+(strong-weak)A.
    """
    strong = float(max(strong, weak)); weak = float(max(0., weak))
    beta = float(np.clip(half_angle, 0., .5*math.pi))
    delta = strong-weak
    if delta <= 256*np.finfo(float).eps*max(strong, 1.):
        q = strong*np.eye(2)
        return q, {"status": "DirectionIrrelevant", "half_angle_rad": beta,
                   "area_inflation": 1., "a": 1., "b": 1.}
    if beta <= 64*np.finfo(float).eps:
        q = np.diag([strong, weak])
        return q, {"status": "CertifiedCone", "half_angle_rad": beta,
                   "area_inflation": 1., "a": 1., "b": 0.}
    if beta >= .5*math.pi-64*np.finfo(float).eps:
        q = strong*np.eye(2)
        return q, {"status": "CertifiedCone", "half_angle_rad": beta,
                   "area_inflation": math.sqrt(strong/max(weak, 1e-300)),
                   "a": 1., "b": 1.}
    s2 = math.sin(beta)**2; c2 = math.cos(beta)**2

    def b_from_a(a: float) -> float:
        return s2/max(1e-300, 1.-c2/a)

    def objective(log_a: float) -> float:
        a = math.exp(log_a); b = b_from_a(a)
        return math.log(weak+delta*a)+math.log(max(1e-300, weak+delta*b))

    # a=1 gives the safe isotropic cap.  Very large a approaches a singular
    # trade in the centre direction; 1e8 is beyond any useful minimum here.
    solved = minimize_scalar(objective, bounds=(0., math.log(1.e8)),
                             method="bounded", options={"xatol": 1.e-14,
                                                        "maxiter": 512})
    candidates = [(1., 1.), (math.exp(float(solved.x)),
                              b_from_a(math.exp(float(solved.x))))]
    a, b = min(candidates, key=lambda pair:
               (weak+delta*pair[0])*(weak+delta*pair[1]))
    q = np.diag([weak+delta*a, weak+delta*b])
    # Outward guard keeps the analytic endpoint inequality away from equality.
    q *= 1.+256*np.finfo(float).eps
    baseline = math.sqrt(max(strong*max(weak, 1e-300), 1e-300))
    return q, {"status": "CertifiedCone", "half_angle_rad": beta,
               "area_inflation": math.sqrt(float(np.linalg.det(q)))/baseline,
               "a": a, "b": b, "optimizer_success": bool(solved.success),
               "endpoint_quadratic": c2/a+s2/b}


def rotate_tensor(tensor: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.asarray([[c, -s], [s, c]])
    return rotation @ tensor @ rotation.T


def cone_majorant_in_frame(strong: float, weak: float, half_angle: float,
                           centre_angle: float) -> tuple[np.ndarray, dict]:
    local, report = cone_majorant_2d(strong, weak, half_angle)
    return rotate_tensor(local, centre_angle), report


def cone_dominance_margin(q: np.ndarray, strong: float, weak: float,
                          half_angle: float, samples: int = 4097) -> float:
    angles = np.linspace(-half_angle, half_angle, samples)
    result = math.inf
    for angle in angles:
        family = rotate_tensor(np.diag([strong, weak]), float(angle))
        result = min(result, float(np.linalg.eigvalsh(q-family).min()))
    return result
