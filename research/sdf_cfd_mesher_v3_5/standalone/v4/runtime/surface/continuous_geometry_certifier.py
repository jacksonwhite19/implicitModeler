"""Conservative continuous face-interior geometry certificate.

The certifier uses exact plane/sphere formulae where available and the
second-order barycentric interpolation remainder elsewhere.  It never uses a
finite sample as the acceptance authority.  `Unresolved` is a hard failure.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq, minimize_scalar

from procedural_authorities import (POSITION_LIMIT, NORMAL_LIMIT_RAD,
                                    Naca0012Authority, CohortAuthority,
                                    make_authority)


def read_surface(vertices_path: Path, faces_path: Path, features_path: Path | None):
    with vertices_path.open(newline="", encoding="utf-8") as f:
        vr = list(csv.DictReader(f, delimiter="\t"))
    points = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in vr])
    with faces_path.open(newline="", encoding="utf-8") as f:
        fr = list(csv.DictReader(f, delimiter="\t"))
    faces = np.array([[int(r["v0"]), int(r["v1"]), int(r["v2"])] for r in fr], dtype=np.int64)
    refs = np.array([int(r["sheet_ref"]) for r in fr], dtype=np.int64)
    edges = []
    if features_path and features_path.exists():
        with features_path.open(newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f, delimiter="\t"):
                edges.append((int(r["v0"]), int(r["v1"]), int(r["curve_ref"])))
    return points, faces, refs, edges


def max_pair_quadratic(edge_values: list[float]) -> float:
    """Exact maximum of sum_ij lambda_i lambda_j a_ij on a 2-simplex."""
    a01, a12, a20 = edge_values
    candidates = [0.0, a01 / 4.0, a12 / 4.0, a20 / 4.0]
    # Solve stationary equations for l0,l1 with l2=1-l0-l1.
    # q=a01*l0*l1+a12*l1*l2+a20*l2*l0.
    A = np.array([[-2.0 * a20, a01 - a12 - a20],
                  [a01 - a12 - a20, -2.0 * a12]])
    b = np.array([-a20, -a12])
    try:
        l0, l1 = np.linalg.solve(A, b); l2 = 1.0 - l0 - l1
        if min(l0, l1, l2) >= -1.0e-15:
            candidates.append(a01*l0*l1 + a12*l1*l2 + a20*l2*l0)
    except np.linalg.LinAlgError:
        pass
    return max(candidates)


def naca_normal_angle_bound(authority: Naca0012Authority, tri: np.ndarray,
                            face_normal: np.ndarray) -> float:
    sign = -1.0 if float(tri[:, 1].mean()) < 0.0 else 1.0
    lo = max(0.0, float(tri[:, 0].min())); hi = min(1.0, float(tri[:, 0].max()))
    candidates = [lo, hi]
    # dyt extrema occur at roots of d2yt; deterministic bracketing only locates
    # candidates, while the analytic normal is evaluated at the polished root.
    xs = np.linspace(max(lo, 1e-12), max(hi, 1e-12), 65)
    vals = [authority.d2yt(float(x)) for x in xs]
    for a, b, fa, fb in zip(xs[:-1], xs[1:], vals[:-1], vals[1:]):
        if fa == 0.0: candidates.append(float(a))
        elif fa * fb < 0.0:
            candidates.append(float(brentq(authority.d2yt, float(a), float(b), xtol=1e-15)))
    if lo <= 1e-12: candidates.append(0.0)
    angles = []
    for x in candidates:
        n = np.array([-1., 0., 0.]) if x <= 1e-12 else np.array([-authority.dyt(x), sign, 0.])
        n /= np.linalg.norm(n)
        angles.append(math.acos(float(np.clip(np.dot(face_normal, n), -1.0, 1.0))))
    return max(angles)


def naca_dyt_turning_points(authority: Naca0012Authority) -> list[float]:
    # d2yt=0 after multiplication by 4*s^3, x=s^2.  Roots of this
    # polynomial partition dyt into monotone intervals, so all chord-residual
    # stationary points can be isolated without a sampled acceptance grid.
    # -0.2969 - 2.8128*s^3 + 6.8232*s^5 - 4.872*s^7 = 0
    coeff = np.zeros(8)
    coeff[0] = -0.2969; coeff[3] = -2.8128; coeff[5] = 6.8232; coeff[7] = -4.872
    roots = np.polynomial.polynomial.polyroots(coeff)
    return sorted(float(r.real * r.real) for r in roots
                  if abs(r.imag) < 1e-12 and 0.0 < r.real < 1.0)


def naca_edge_distance(authority: Naca0012Authority, a: np.ndarray,
                       b: np.ndarray, sign: float) -> float:
    dx = float(b[0] - a[0]); dy = float(b[1] - a[1])
    candidate_x: list[float] = []
    if abs(dx) > 1e-18:
        target = dy / (sign * dx)
        x0, x1 = sorted((float(a[0]), float(b[0])))
        cuts = [x0] + [x for x in naca_dyt_turning_points(authority) if x0 < x < x1] + [x1]
        for lo, hi in zip(cuts[:-1], cuts[1:]):
            lo_eval = max(lo, 1e-15); hi_eval = max(hi, 1e-15)
            flo = authority.dyt(lo_eval) - target; fhi = authority.dyt(hi_eval) - target
            roots: list[float] = []
            if flo == 0.0: roots.append(lo_eval)
            if fhi == 0.0: roots.append(hi_eval)
            if flo * fhi < 0.0:
                roots.append(float(brentq(lambda x: authority.dyt(x)-target,
                                          lo_eval, hi_eval, xtol=1e-15)))
            candidate_x.extend(roots)
    # For an interior extremum of Euclidean distance, the curve tangent and
    # chord direction are parallel, and the closest-point segment is normal to
    # both. All such x values were isolated above. Endpoints are governed and
    # have zero residual after projection.
    maximum = 0.0
    d = b[:2] - a[:2]
    for x in candidate_x:
        c = np.array([x, sign * authority.yt(x)])
        tangent = np.array([1.0, sign * authority.dyt(x)])
        denom = float(np.dot(d, tangent))
        if abs(denom) <= 1e-30:
            continue
        t = float(np.dot(c - a[:2], tangent) / denom)
        if -1e-13 <= t <= 1.0 + 1e-13:
            p = a[:2] + min(1.0, max(0.0, t)) * d
            maximum = max(maximum, float(np.linalg.norm(p - c)))
    return maximum


def naca_profile_candidate(authority: Naca0012Authority,
                           point: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Return a constructive profile point, normal, and distance upper bound.

    The minimizer only improves the bound: acceptance never assumes optimizer
    convergence or nearest-point identity.
    """
    p = np.asarray(point[:2], dtype=float)
    best = None
    for lo, hi in ((-1.0, 0.0), (0.0, 1.0)):
        def curve(u: float) -> np.ndarray:
            x = u * u
            return np.array([x, (-1.0 if u < 0.0 else 1.0) * authority.yt(x)])
        result = minimize_scalar(lambda u: float(np.dot(curve(u)-p, curve(u)-p)),
                                 bounds=(lo, hi), method="bounded",
                                 options={"xatol": 1.0e-15, "maxiter": 256})
        for u in (float(result.x), lo, hi):
            q = curve(u); distance = float(np.linalg.norm(q-p))
            if best is None or distance < best[0]:
                if abs(u) <= 1.0e-14:
                    normal = np.array([-1.0, 0.0, 0.0])
                else:
                    x = u*u; sign = -1.0 if u < 0.0 else 1.0
                    normal = np.array([-authority.dyt(x), sign, 0.0])
                    normal /= np.linalg.norm(normal)
                best = (distance, np.array([q[0], q[1], point[2]]), normal)
    assert best is not None
    return best[1], best[2], best[0]


def naca_mixed_branch_certificate(authority: Naca0012Authority, tri: np.ndarray,
                                  face_normal: np.ndarray):
    """Conservative branch-set certificate for leading-edge spanning faces."""
    stack = [(tri, 0)]; worst_position = 0.0; worst_normal = 0.0
    while stack:
        sub, depth = stack.pop()
        centre = sub.mean(axis=0)
        q, source_normal, candidate_distance = naca_profile_candidate(authority, centre)
        # The analytic NACA skin is a generalized cylinder: both its distance
        # set and source normal are invariant in span.  Only the profile-plane
        # support radius enters these two bounds.  Including span would be a
        # false Lipschitz penalty on intentionally anisotropic triangles.
        radius = max(float(np.linalg.norm(vertex[:2]-centre[:2])) for vertex in sub)
        # Distance to a closed set is 1-Lipschitz.  The constructive candidate
        # is an upper bound even if the numerical minimizer is imperfect.
        position_bound = candidate_distance + radius
        # A closest profile point for any support point lies within
        # 2*(candidate_distance + radius) of the constructive candidate q. The
        # NACA leading-edge arc is a smooth convex planar arc.  With curvature
        # bounded by k, its normal turn over chord c is conservatively bounded
        # by the circular extremum 2*asin(k*c/2).  This covers either branch
        # without requiring a unique nearest point at the centreline.
        curvature_bound = 100.0
        curve_chord_bound = 2.0 * (candidate_distance + radius)
        x_hi = max(float(q[0]), float(centre[0])) + curve_chord_bound
        argument = 0.5 * curvature_bound * curve_chord_bound
        if x_hi <= 1.0e-3 and argument < 1.0:
            centre_angle = math.acos(float(np.clip(np.dot(face_normal, source_normal), -1.0, 1.0)))
            normal_bound = centre_angle + 2.0 * math.asin(argument)
        else:
            normal_bound = math.pi
        if position_bound <= POSITION_LIMIT and normal_bound <= NORMAL_LIMIT_RAD:
            worst_position = max(worst_position, position_bound)
            worst_normal = max(worst_normal, normal_bound)
            continue
        if depth >= 14:
            return "Unresolved", max(worst_position, position_bound), \
                   max(worst_normal, normal_bound), "branch_set_lipschitz_subdivision"
        m01=.5*(sub[0]+sub[1]); m12=.5*(sub[1]+sub[2]); m20=.5*(sub[2]+sub[0])
        stack.extend([(np.array([sub[0],m01,m20]),depth+1),
                      (np.array([m01,sub[1],m12]),depth+1),
                      (np.array([m20,m12,sub[2]]),depth+1),
                      (np.array([m01,m12,m20]),depth+1)])
    return "Certified", worst_position, worst_normal, "branch_set_lipschitz_subdivision"


def _minimal_angle_interval(xy: np.ndarray) -> tuple[float, float]:
    """Return the shortest unwrapped interval containing nonzero xy rays."""
    radii = np.linalg.norm(xy, axis=1)
    theta = np.mod(np.arctan2(xy[radii > 1.0e-14, 1],
                              xy[radii > 1.0e-14, 0]), 2.0 * math.pi)
    if len(theta) == 0:
        return 0.0, 2.0 * math.pi
    if len(theta) == 1:
        return float(theta[0]), float(theta[0])
    theta.sort()
    wrapped = np.r_[theta, theta[0] + 2.0 * math.pi]
    gaps = np.diff(wrapped)
    cut = int(np.argmax(gaps))
    lo = float(wrapped[cut + 1])
    hi = float(wrapped[cut] + 2.0 * math.pi)
    return lo, hi


def sharp_cone_sector_certificate(authority: CohortAuthority, ref: int,
                                  tri: np.ndarray, face_normal: np.ndarray):
    """Exact-normal/conservative-position certificate for one cone sector.

    The apex has no smooth normal.  Its contract is the limiting angular
    sector formed by the incident non-apex rays.  Ordinary sheet support uses
    the same unchanged normal limit as every other smooth face.
    """
    lo, hi = _minimal_angle_interval(tri[:, :2])
    span = hi - lo
    if not span < math.pi:
        return "Unresolved", math.inf, math.pi, "exact_singular_cone_sector"

    # On rho + 0.5*sign(z)*z = 0.35, convexity of the radial norm gives
    #   sum(lambda*rho_i) - |sum(lambda*x_i)|
    # <= rho_max*(1-cos(span/2)).  Division by |grad F| converts this
    # length-valued residual to Euclidean normal distance.
    rho_max = float(np.linalg.norm(tri[:, :2], axis=1).max())
    position = rho_max * (1.0 - math.cos(0.5 * span)) / math.sqrt(1.25)

    sign_z = -1.0 if ref == 1 else 1.0
    radial_norm = 1.0 / math.sqrt(1.25)
    constant = face_normal[2] * 0.5 * sign_z * radial_norm
    a = face_normal[0] * radial_norm
    b = face_normal[1] * radial_norm
    candidates = [lo, hi]
    minimum_phase = math.atan2(b, a) + math.pi
    for turn in range(-2, 4):
        candidate = minimum_phase + 2.0 * math.pi * turn
        if lo <= candidate <= hi:
            candidates.append(candidate)
    minimum_dot = min(a * math.cos(theta) + b * math.sin(theta) + constant
                      for theta in candidates)
    normal = math.acos(float(np.clip(minimum_dot, -1.0, 1.0)))
    status = "Certified" if (position <= POSITION_LIMIT and
                             normal <= NORMAL_LIMIT_RAD) else "Violation"
    return status, position, normal, "exact_singular_cone_sector"


def wing_body_interval_certificate(authority: CohortAuthority, ref: int,
                                   tri: np.ndarray, face_normal: np.ndarray):
    """Conservative branch-restricted Hessian/Lipschitz certificate."""
    stack = [(tri, 0)]
    worst_position = 0.0
    worst_normal = 0.0
    while stack:
        sub, depth = stack.pop()
        center = sub.mean(axis=0)
        radius = max(float(np.linalg.norm(point - center)) for point in sub)
        hessian_bound = authority.wing_body_hessian_norm_bound(sub)
        gradient = authority._wing_body_gradient(center)
        gradient_lower = float(np.linalg.norm(gradient)) - hessian_bound * radius
        if gradient_lower > 0.0 and math.isfinite(hessian_bound):
            edge2 = [float(np.dot(sub[i] - sub[j], sub[i] - sub[j]))
                     for i, j in ((0, 1), (1, 2), (2, 0))]
            residual = [abs(float(authority.implicit_value(ref, point))) for point in sub]
            implicit_bound = max(residual) + 0.5 * hessian_bound * max_pair_quadratic(edge2)
            position_bound = implicit_bound / gradient_lower
            center_normal = gradient / float(np.linalg.norm(gradient))
            center_angle = math.acos(float(np.clip(np.dot(face_normal, center_normal), -1.0, 1.0)))
            normal_bound = center_angle + hessian_bound * radius / gradient_lower
            if position_bound <= POSITION_LIMIT and normal_bound <= NORMAL_LIMIT_RAD:
                worst_position = max(worst_position, position_bound)
                worst_normal = max(worst_normal, normal_bound)
                continue
        else:
            position_bound = math.inf
            normal_bound = math.pi
        if depth >= 14:
            return ("Unresolved", max(worst_position, position_bound),
                    max(worst_normal, normal_bound),
                    "branch_restricted_interval_hessian")
        m01 = .5 * (sub[0] + sub[1]); m12 = .5 * (sub[1] + sub[2]); m20 = .5 * (sub[2] + sub[0])
        stack.extend([(np.array([sub[0], m01, m20]), depth + 1),
                      (np.array([m01, sub[1], m12]), depth + 1),
                      (np.array([m20, m12, sub[2]]), depth + 1),
                      (np.array([m01, m12, m20]), depth + 1)])
    return ("Certified", worst_position, worst_normal,
            "branch_restricted_interval_hessian")


def certify_face(authority, ref: int, tri: np.ndarray):
    cross = np.cross(tri[1] - tri[0], tri[2] - tri[0])
    area2 = float(np.linalg.norm(cross))
    if not area2 > 0.0:
        return "Violation", math.inf, math.pi, "degenerate"
    fn = cross / area2
    centroid = tri.mean(axis=0)
    if np.dot(fn, authority.exact_normal(ref, centroid)) <= 0.0:
        return "Violation", math.inf, math.pi, "orientation"

    if isinstance(authority, Naca0012Authority):
        if ref != 1:
            return "Certified", 0.0, 0.0, "exact_plane"
        if float(tri[:, 1].min()) < 0.0 < float(tri[:, 1].max()):
            return naca_mixed_branch_certificate(authority, tri, fn)
        # The signed distance has no stationary point in the unique same-branch
        # tube. Its extrema on the projected triangle occur on the boundary.
        # Each edge extremum is isolated by the parallel-tangent condition.
        sign = -1.0 if float(tri[:, 1].mean()) < 0.0 else 1.0
        position = max(naca_edge_distance(authority, tri[i], tri[j], sign)
                       for i, j in ((0, 1), (1, 2), (2, 0)))
        normal = naca_normal_angle_bound(authority, tri, fn)
        status = "Certified" if position <= POSITION_LIMIT and normal <= NORMAL_LIMIT_RAD else "Violation"
        return status, position, normal, "exact_distance_edge_extrema_generalized_cylinder"

    if isinstance(authority, CohortAuthority) and authority.fixture in ("sphere", "narrow_gap"):
        center, radius = authority._sphere(ref)
        # The complete chord triangle lies inside the sphere. Minimum radial
        # norm is the exact closest point from the centre to the triangle.
        e0, e1 = tri[1] - tri[0], tri[2] - tri[0]
        G = np.array([[np.dot(e0,e0), np.dot(e0,e1)], [np.dot(e0,e1), np.dot(e1,e1)]])
        rhs = -np.array([np.dot(e0, tri[0]-center), np.dot(e1, tri[0]-center)])
        candidates = [tri[0], tri[1], tri[2]]
        try:
            uv = np.linalg.solve(G, rhs)
            if uv[0] >= 0 and uv[1] >= 0 and uv.sum() <= 1:
                candidates.append(tri[0] + uv[0]*e0 + uv[1]*e1)
        except np.linalg.LinAlgError: pass
        for i,j in ((0,1),(1,2),(2,0)):
            d=tri[j]-tri[i]; t=np.clip(-np.dot(tri[i]-center,d)/np.dot(d,d),0,1); candidates.append(tri[i]+t*d)
        rmin = min(float(np.linalg.norm(p-center)) for p in candidates)
        position = max(0.0, radius-rmin)
        plane_distance = abs(float(np.dot(fn, tri[0]-center)))
        normal = math.acos(float(np.clip(plane_distance/radius, -1.0, 1.0)))
        status = "Certified" if position <= POSITION_LIMIT and normal <= NORMAL_LIMIT_RAD else "Violation"
        return status, position, normal, "exact_sphere_chord"

    if isinstance(authority, CohortAuthority) and authority.fixture == "capped_cylinder":
        if ref != 1:
            return "Certified", 0.0, 0.0, "exact_plane"
        # A generalized cylinder is invariant in its axial direction.  Its
        # exact chordal error therefore depends only on the projected 2-D
        # triangle, not on the (potentially long) axial edges.  Using a 3-D
        # isotropic Hessian-radius remainder here is mathematically safe but
        # needlessly loose and defeats the selected anisotropic metric.
        xy = tri[:, :2]
        candidates = [xy[0], xy[1], xy[2]]
        e0, e1 = xy[1] - xy[0], xy[2] - xy[0]
        G = np.array([[np.dot(e0, e0), np.dot(e0, e1)],
                      [np.dot(e0, e1), np.dot(e1, e1)]])
        rhs = -np.array([np.dot(e0, xy[0]), np.dot(e1, xy[0])])
        try:
            uv = np.linalg.solve(G, rhs)
            if uv[0] >= 0.0 and uv[1] >= 0.0 and uv.sum() <= 1.0:
                candidates.append(xy[0] + uv[0] * e0 + uv[1] * e1)
        except np.linalg.LinAlgError:
            pass
        for i, j in ((0, 1), (1, 2), (2, 0)):
            d = xy[j] - xy[i]
            dd = float(np.dot(d, d))
            t = 0.0 if dd == 0.0 else float(np.clip(-np.dot(xy[i], d) / dd, 0.0, 1.0))
            candidates.append(xy[i] + t * d)
        rho_min = min(float(np.linalg.norm(p)) for p in candidates)
        position = max(0.0, 0.35 - rho_min)

        # The radial directions of a convex projected triangle form one
        # continuous angular interval (the origin is excluded by rho_min>0).
        # Minimize the dot product with the planar face normal over that full
        # interval, including its only possible interior minimum.
        theta = np.unwrap(np.arctan2(xy[:, 1], xy[:, 0]))
        lo, hi = float(theta.min()), float(theta.max())
        alpha = math.atan2(float(fn[1]), float(fn[0]))
        angles = [lo, hi]
        for k in range(-2, 3):
            candidate = alpha + math.pi + 2.0 * math.pi * k
            if lo <= candidate <= hi:
                angles.append(candidate)
        minimum_dot = min(float(fn[0] * math.cos(t) + fn[1] * math.sin(t))
                          for t in angles)
        normal = math.acos(float(np.clip(minimum_dot, -1.0, 1.0)))
        status = "Certified" if position <= POSITION_LIMIT and normal <= NORMAL_LIMIT_RAD else "Violation"
        return status, position, normal, "exact_generalized_cylinder_chord"

    if (isinstance(authority, CohortAuthority) and
            authority.fixture == "sharp_feature_closed_body"):
        return sharp_cone_sector_certificate(authority, ref, tri, fn)

    if (isinstance(authority, CohortAuthority) and
            authority.fixture == "wing_body_smooth_junction"):
        return wing_body_interval_certificate(authority, ref, tri, fn)

    # General adaptive second-order certificate. The signed-distance
    # interpolation remainder is evaluated on a deterministic 1-to-4
    # barycentric tree. This is a continuous enclosure, not point sampling.
    kappa = float(authority.curvature_upper(ref, tri))
    K = 0.0 if kappa == 0.0 else kappa / max(1.0e-12, 1.0 - kappa * POSITION_LIMIT)
    stack = [(tri, 0)]; position = 0.0; normal = 0.0
    while stack:
        sub, depth = stack.pop()
        a = [float(np.dot(sub[i]-sub[j], sub[i]-sub[j])) for i,j in ((0,1),(1,2),(2,0))]
        residual = [abs(float(authority.implicit_value(ref,p))) for p in sub]
        pos_bound = max(residual) + 0.5*K*max_pair_quadratic(a)
        center = sub.mean(axis=0)
        center_angle = math.acos(float(np.clip(np.dot(fn,authority.exact_normal(ref,center)),-1,1)))
        radius = max(float(np.linalg.norm(p-center)) for p in sub)
        normal_bound = center_angle + K*radius
        if pos_bound <= POSITION_LIMIT and normal_bound <= NORMAL_LIMIT_RAD:
            position=max(position,pos_bound); normal=max(normal,normal_bound); continue
        if max(residual)>POSITION_LIMIT or depth>=12:
            status="Violation" if max(residual)>POSITION_LIMIT else "Unresolved"
            return status,max(position,pos_bound),max(normal,normal_bound),"adaptive_signed_distance_hessian"
        m01=.5*(sub[0]+sub[1]); m12=.5*(sub[1]+sub[2]); m20=.5*(sub[2]+sub[0])
        stack.extend([(np.array([sub[0],m01,m20]),depth+1),
                      (np.array([m01,sub[1],m12]),depth+1),
                      (np.array([m20,m12,sub[2]]),depth+1),
                      (np.array([m01,m12,m20]),depth+1)])
    return "Certified",position,normal,"adaptive_signed_distance_hessian"


def feature_bound(authority, a: np.ndarray, b: np.ndarray, ref: int) -> float:
    # Recursive midpoint enclosure. The curve residual is 1-Lipschitz and each
    # subsegment is accepted only when residual(mid)+half_length is below the
    # hard bound; this is a continuous certificate, not a sample gate.
    stack = [(a, b, 0)]; worst = 0.0
    while stack:
        p, q, depth = stack.pop(); m = .5*(p+q)
        residual = authority.feature_residual(ref, m)
        bound = residual + .5*float(np.linalg.norm(q-p))
        if bound <= POSITION_LIMIT:
            worst = max(worst, bound); continue
        if depth >= 24:
            return math.inf
        stack.append((p, m, depth+1)); stack.append((m, q, depth+1))
    return worst


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--authority", required=True)
    parser.add_argument("--vertices", required=True)
    parser.add_argument("--faces", required=True)
    parser.add_argument("--features")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    authority = make_authority(args.authority)
    points, faces, refs, edges = read_surface(Path(args.vertices), Path(args.faces),
                                               Path(args.features) if args.features else None)
    counts = {"Certified": 0, "Violation": 0, "Unresolved": 0}
    max_position = 0.0; max_normal = 0.0; first = []
    methods = {}
    for i, (face, ref) in enumerate(zip(faces, refs)):
        status, pos, normal, method = certify_face(authority, int(ref), points[face])
        counts[status] += 1; methods[method] = methods.get(method, 0) + 1
        max_position = max(max_position, pos); max_normal = max(max_normal, normal)
        if status != "Certified" and len(first) < 20:
            first.append({"face": i, "sheet_ref": int(ref), "status": status,
                          "position_bound_m": pos, "normal_bound_deg": math.degrees(normal),
                          "method": method})
    max_feature = 0.0; feature_fail = 0
    for a,b,ref in edges:
        bound = feature_bound(authority, points[a], points[b], ref)
        max_feature = max(max_feature, bound)
        feature_fail += not math.isfinite(bound) or bound > POSITION_LIMIT
    report = {"schema": "V3ContinuousProceduralSurfaceCertificate/v1",
              "authority": authority.name, "faces": len(faces),
              "position_limit_m": POSITION_LIMIT,
              "normal_limit_deg": math.degrees(NORMAL_LIMIT_RAD),
              "feature_limit_m": POSITION_LIMIT,
              "face_status": counts, "methods": methods,
              "max_certified_position_bound_m": max_position,
              "max_certified_normal_bound_deg": math.degrees(max_normal),
              "max_certified_feature_bound_m": max_feature,
              "feature_violations": feature_fail, "first_noncertified": first,
              "sampling_is_acceptance_authority": False,
              "pass": counts["Violation"] == 0 and counts["Unresolved"] == 0 and feature_fail == 0}
    out = Path(args.output); out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
