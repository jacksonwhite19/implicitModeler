"""Private V2 vertex-only geometric envelope; no cell/material/source authority.

Each non-cap vertex has a triangle ID and two binary64 barycentric weights.
The pure verifier interprets all binary64 inputs as exact dyadic rationals.
Closest-point search and optional contraction are proposers, never predicates.
Callers must retain their existing protected-input custody and mesh admission.
"""
from fractions import Fraction as F
import math
import numpy as np

SCHEMA = 'ShapeTransitionVertexEnvelope/2'


def _tick(check):
    if check is not None:
        check()


def _array(value, dtype, shape, name):
    a = np.asarray(value)
    if a.dtype != np.dtype(dtype) or a.ndim != len(shape) or any(
            n is not None and a.shape[i] != n for i, n in enumerate(shape)):
        raise ValueError(name)
    if a.dtype.kind == 'f' and not np.isfinite(a).all():
        raise ValueError(name)
    return np.array(a, copy=True, order='C')


def _inputs(points, cap, triangles, maximum_distance, check):
    _tick(check)
    p = _array(points, '<f8', (None, 3), 'InvalidEnvelopePoints')
    c = _array(cap, '<f8', (None, 3), 'InvalidEnvelopeCap')
    t = _array(triangles, '<i8', (None, 3), 'InvalidEnvelopeTriangles')
    if not len(c) or not len(t) or len(p) < len(c):
        raise ValueError('EmptyEnvelopeGeometry')
    if p[:len(c)].tobytes() != c.tobytes():
        raise ValueError('ChangedProtectedCap')
    if np.any(t < 0) or np.any(t >= len(c)):
        raise ValueError('InvalidEnvelopeTriangleIndex')
    d = float(maximum_distance)
    if not math.isfinite(d) or d <= 0:
        raise ValueError('PositiveFiniteEnvelopeDistanceRequired')
    exact = []
    for i, row in enumerate(t):
        if i % 256 == 0:
            _tick(check)
        a, b, cc = [tuple(F(float(z)) for z in c[j]) for j in row]
        ab = tuple(y-x for x, y in zip(a, b))
        ac = tuple(y-x for x, y in zip(a, cc))
        normal = _cross(ab, ac)
        if not any(normal):
            raise ValueError('DegenerateEnvelopeTriangle')
        exact.append((a, ab, ac, normal))
    _tick(check)
    return p, c, t, F(d)**2, exact


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def _predicate(point, geometry, weights, distance2):
    u, v = (F(float(z)) for z in weights)
    if u < 0 or v < 0 or u+v > 1:
        return False
    a, ab, ac, normal = geometry
    p = tuple(F(float(z)) for z in point)
    q = tuple(a[i]+u*ab[i]+v*ac[i] for i in range(3))
    if sum(normal[i]*(p[i]-a[i]) for i in range(3)) <= 0:
        return False
    return sum((p[i]-q[i])**2 for i in range(3)) <= distance2


def verify(points, cap, triangles, maximum_distance, witnesses, check=None):
    """Pure exact predicate; reject incomplete, changed or invalid witnesses.

    This proves only the declared vertices lie in this oriented geometric
    envelope. It proves neither material ancestry nor an every-cell tube.
    """
    p, c, t, d2, exact = _inputs(points, cap, triangles, maximum_distance, check)
    if type(witnesses) is not dict or set(witnesses) != {
            'vertex_ids', 'triangle_ids', 'weights'}:
        raise ValueError('InvalidEnvelopeWitnessSchema')
    n = len(p)-len(c)
    ids = _array(witnesses['vertex_ids'], '<i8', (n,), 'InvalidEnvelopeVertexIDs')
    tids = _array(witnesses['triangle_ids'], '<i8', (n,), 'InvalidEnvelopeTriangleIDs')
    uv = _array(witnesses['weights'], '<f8', (n, 2), 'InvalidEnvelopeWeights')
    if not np.array_equal(ids, np.arange(len(c), len(p), dtype='<i8')):
        raise ValueError('IncompleteEnvelopeVertexPopulation')
    if np.any(tids < 0) or np.any(tids >= len(t)):
        raise ValueError('InvalidEnvelopeWitnessTriangle')
    for vertex, triangle, weights in zip(ids, tids, uv):
        _tick(check)
        if not _predicate(p[vertex], exact[triangle], weights, d2):
            raise ValueError('VertexOutsideDeclaredEnvelope')
    _tick(check)
    return dict(schema=SCHEMA, passed=True, vertices_checked=n,
                exact_dyadic=True, vertex_only=True,
                material_ancestry=False, every_cell_tube=False,
                scientific_mesh_acceptance=False)


def _simplex(u, v):
    """Make proposed binary64 weights an exact member of the closed simplex."""
    u = min(1., max(0., float(u)))
    v = min(1., max(0., float(v)))
    remaining = 1-F(u)
    if F(v) > remaining:
        v = float(remaining)
        if F(v) > remaining:
            v = float(np.nextafter(v, 0.))
    return np.array([u, v], dtype='<f8')


def _closest(point, triangle):
    """Floating face/edge candidates only; exact predicate decides validity."""
    origin = triangle[0]
    # Normalize before subtraction/products to avoid avoidable overflow.
    scale = max(float(np.max(np.abs(triangle))), float(np.max(np.abs(point))))
    if not scale:
        return []
    x = triangle/scale
    p = point/scale
    ab, ac, ap = x[1]-x[0], x[2]-x[0], p-x[0]
    choices = [np.array([0., 0.]), np.array([1., 0.]), np.array([0., 1.])]
    matrix = np.stack((ab, ac), axis=1)
    try:
        uv = np.linalg.lstsq(matrix, ap, rcond=None)[0]
        if np.isfinite(uv).all() and uv[0] >= 0 and uv[1] >= 0 and uv.sum() <= 1:
            choices.append(_simplex(*uv))
    except np.linalg.LinAlgError:
        pass
    for i, j, weights in ((0, 1, lambda s: (s, 0.)),
                          (0, 2, lambda s: (0., s)),
                          (1, 2, lambda s: (1.-s, s))):
        edge = x[j]-x[i]
        den = float(edge@edge)
        if den > 0 and math.isfinite(den):
            s = float(np.clip((p-x[i])@edge/den, 0., 1.))
            choices.append(_simplex(*weights(s)))
    def distance(uv):
        return float(np.linalg.norm(p-(x[0]+uv[0]*ab+uv[1]*ac)))
    return sorted(choices, key=distance)


def _contract(point, geometry, weights, d2, check):
    a, ab, ac, normal = geometry
    p = tuple(F(float(z)) for z in point)
    # Contract only an already outward proposal. Never reflect through a cap.
    if sum(normal[i]*(p[i]-a[i]) for i in range(3)) <= 0:
        return None
    u, v = map(lambda z: F(float(z)), weights)
    q = tuple(a[i]+u*ab[i]+v*ac[i] for i in range(3))
    delta = tuple(p[i]-q[i] for i in range(3))
    norm2 = sum(z*z for z in delta)
    # sqrt ratio can under/overflow; dyadic halving still provides a fallback.
    ratio = d2/norm2
    factor = math.sqrt(float(ratio)) if ratio < 1 else 1.
    if factor:
        factor = F(float(np.nextafter(factor, 0.)))
    else:
        # Ratios below binary64 range still admit a dyadic contraction factor.
        exponent = (ratio.numerator.bit_length()-ratio.denominator.bit_length())//2
        factor = F(2)**exponent
        while factor*factor > ratio:
            factor /= 2
    for iteration in range(128):
        _tick(check)
        step = factor
        try:
            out = np.array([float(q[i]+step*delta[i]) for i in range(3)], dtype='<f8')
        except OverflowError:
            out = np.full(3, np.inf)
        if np.isfinite(out).all() and _predicate(out, geometry, weights, d2):
            return out
        # First try rounding the contraction inward, then a geometric backoff.
        rounded = float(factor)
        factor = (F(float(np.nextafter(rounded, 0.)))
                  if iteration < 32 and rounded else factor/2)
        if not factor:
            break
    return None


def witness(points, cap, triangles, maximum_distance, check=None):
    """Propose and verify a private vertex envelope, optionally contracting.

    KD-tree candidates are heuristic. No nearest-triangle or search-completeness
    claim is made. Failure to find a representable witness fails closed.
    """
    from scipy.spatial import cKDTree
    p, c, t, d2, exact = _inputs(points, cap, triangles, maximum_distance, check)
    ids = np.arange(len(c), len(p), dtype='<i8')
    tids = np.empty(len(ids), dtype='<i8')
    weights = np.empty((len(ids), 2), dtype='<f8')
    # Common scaling prevents centroid overflow while retaining tree indices.
    scale = max(float(np.max(np.abs(c))), float(np.max(np.abs(p))), 1.)
    centres = (c[t]/scale).mean(axis=1)
    tree = cKDTree(centres)
    vertices = cKDTree(c/scale)
    incidence = [[] for _ in c]
    for triangle, face in enumerate(t):
        for vertex in face:
            incidence[vertex].append(triangle)
    projected = tested = 0
    for row, vertex in enumerate(ids):
        _tick(check)
        _, nearest = tree.query(p[vertex]/scale, k=min(32, len(t)))
        _, nearby = vertices.query(p[vertex]/scale, k=min(4, len(c)))
        candidates = list(dict.fromkeys([int(z) for z in np.atleast_1d(nearest)] +
            [triangle for z in np.atleast_1d(nearby) for triangle in incidence[int(z)]]))
        chosen = None
        # Prefer an unchanged point before considering any projected proposal.
        proposals = []
        for triangle in candidates:
            _tick(check)
            for uv in _closest(p[vertex], c[t[triangle]]):
                tested += 1
                if _predicate(p[vertex], exact[triangle], uv, d2):
                    chosen = (triangle, uv, p[vertex].copy())
                    break
                proposals.append((triangle, uv))
            if chosen is not None:
                break
        if chosen is None:
            for triangle, uv in proposals:
                out = _contract(p[vertex], exact[triangle], uv, d2, check)
                if out is not None:
                    chosen = (triangle, uv, out)
                    projected += 1
                    break
        if chosen is None:
            raise ValueError('NoRepresentableEnvelopeWitness')
        triangle, uv, p[vertex] = chosen
        tids[row], weights[row] = triangle, uv
    result = dict(vertex_ids=ids, triangle_ids=tids, weights=weights)
    receipt = verify(p, c, t, maximum_distance, result, check)
    return p, result, dict(receipt, projected_vertices=projected,
                           candidate_weights_tested=tested,
                           proposer_search_complete=False,
                           protected_cap_bytes_unchanged=True)
