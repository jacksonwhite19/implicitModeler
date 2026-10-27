"""Private coordinate feasibility probe. Never a mesher or admission issuer.

The reference is the inherited cap's adjacent-normal differences, not a plane,
sphere, scalar SDF approximation or a replacement source geometry. No cleanup.
"""
from collections import defaultdict
from contextlib import contextmanager
import hashlib
import json
import time
import numpy as np


def digest(a):
    a = np.ascontiguousarray(a)
    return hashlib.sha256(a.dtype.str.encode()+str(a.shape).encode()+a.tobytes()).hexdigest()


def incidence(faces):
    faces = np.asarray(faces)
    if faces.dtype != np.dtype('<i8') or faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError('TriangleConnectivityRequired')
    uses = defaultdict(list)
    for i, face in enumerate(faces):
        if len(set(map(int, face))) != 3 or min(face) < 0:
            raise ValueError('DistinctNonnegativeTriangleVerticesRequired')
        for a, b in zip(face, np.roll(face, -1)):
            uses[tuple(sorted((int(a), int(b))))].append((i, int(a), int(b)))
    keys = sorted(uses)
    for edge in keys:
        pair = uses[edge]
        if len(pair) != 2 or pair[0][1:] != pair[1][1:][::-1]:
            raise ValueError('ClosedOppositeEdgeWindingRequired', edge)
    return np.asarray(keys, dtype='<i8'), np.asarray([[r[0] for r in uses[k]] for k in keys], dtype='<i8')


def normals(points, faces):
    tri = np.asarray(points)[faces]
    raw = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    size = np.linalg.norm(raw, axis=1)
    if not np.isfinite(raw).all() or np.any(size <= 0):
        raise ValueError('FiniteNondegenerateNormalsRequired')
    return raw/size[:, None]


def angles(unit_normals, pairs):
    a, b = unit_normals[pairs[:, 0]], unit_normals[pairs[:, 1]]
    dot = np.clip(np.einsum('ij,ij->i', a, b), -1., 1.)
    return np.arctan2(np.linalg.norm(np.cross(a, b), axis=1), dot)


def residuals(unit_normals, reference_normals, pairs):
    """Dimensionless squared normal-difference error /16, in [0,1]."""
    i, j = pairs.T
    delta = (unit_normals[j]-unit_normals[i])-(reference_normals[j]-reference_normals[i])
    return np.einsum('ij,ij->i', delta, delta)/16.


def worst_edge(points, faces):
    edges, pairs = incidence(faces)
    turn = angles(normals(points, faces), pairs)
    # Edges are lexicographically ordered; argmax gives stable smallest-edge ties.
    index = int(np.argmax(turn))
    return edges, pairs, turn, index


def closure(cells, owners, outer, moving):
    """Complete affected stars, all actual face neighbours, full outer edge halo."""
    moving = tuple(sorted(map(int, moving)))
    if not moving or len(set(moving)) != len(moving):
        raise ValueError('UniqueMovableVerticesRequired')
    selected = {c['id'] for c in cells if set(c['vertices']).intersection(moving)}
    byid = {c['id']: c for c in cells}
    if len(byid) != len(cells) or not selected:
        raise ValueError('UniqueAffectedCellIdentityRequired')
    if not set(moving) <= {v for i in selected for v in byid[i]['vertices']}:
        raise ValueError('UnknownMovableVertex')
    keys = {tuple(sorted(f)) for i in selected for f in byid[i]['faces']}
    if any(k not in owners or len(owners[k]) not in (1, 2) for k in keys):
        raise ValueError('CompleteActualFaceOwnersRequired')
    actual = defaultdict(list)
    for c in cells:
        for face in c['faces']:
            k = tuple(sorted(face))
            if k in keys:
                actual[k].append((c['id'], tuple(face)))
    if any(sorted(actual[k]) != sorted(owners[k]) for k in keys):
        raise ValueError('IncompleteActualNeighbourClosure')
    needed = {i for k in keys for i, _ in owners[k]}
    if not needed <= byid.keys():
        raise ValueError('MissingActualNeighbour')
    edges, pairs = incidence(outer)
    affected_faces = np.flatnonzero(np.isin(outer, moving).any(axis=1))
    halo_edges = np.flatnonzero(np.isin(pairs, affected_faces).any(axis=1))
    halo_faces = np.unique(pairs[halo_edges])
    if not len(halo_edges):
        raise ValueError('NoAffectedOuterFront')
    return dict(moving=list(moving), selected=sorted(selected), needed=sorted(needed),
        face_keys=[list(k) for k in sorted(keys)], affected_outer_faces=affected_faces.tolist(),
        halo_edges=halo_edges.tolist(), halo_faces=halo_faces.tolist(),
        remaining_edges=np.flatnonzero(~np.isin(np.arange(len(edges)), halo_edges)).tolist())


class FixedData:
    """Exact coordinate/connectivity preservation; no tolerance-based identity."""
    def __init__(self, points, moving, connectivity):
        self.shape = points.shape
        self.fixed = np.flatnonzero(~np.isin(np.arange(len(points)), moving))
        self.points = digest(points[self.fixed])
        self.connectivity = {k: digest(v) for k, v in connectivity.items()}

    def require(self, points, connectivity):
        if points.shape != self.shape or not np.isfinite(points).all():
            raise ValueError('ChangedOrNonfiniteCoordinatePopulation')
        if digest(points[self.fixed]) != self.points:
            raise ValueError('ChangedFixedCoordinates')
        if {k: digest(v) for k, v in connectivity.items()} != self.connectivity:
            raise ValueError('ChangedConnectivity')


def comparison(before_normals, after_normals, pairs, selected_edge, halo_edges):
    before = angles(before_normals, pairs)
    repeat = angles(before_normals, pairs)
    after = angles(after_normals, pairs)
    i, j = pairs.T
    independent = np.arccos(np.clip(np.einsum('ij,ij->i', before_normals[i], before_normals[j]), -1., 1.))
    error = max(float(np.max(np.abs(before-independent))), float(np.max(np.abs(before-repeat))))
    groups = dict(affected_edge_halo=np.asarray(halo_edges, dtype=int),
        remaining_front=np.setdiff1d(np.arange(len(pairs)), halo_edges), whole_front=np.arange(len(pairs)))
    reports = {}
    qs = [.5, .9, .95, .99, 1.]
    for name, ids in groups.items():
        a = np.quantile(before[ids], qs) if len(ids) else np.array([])
        b = np.quantile(after[ids], qs) if len(ids) else np.array([])
        reports[name] = dict(count=len(ids), quantiles=qs, before_rad=a.tolist(), after_rad=b.tolist(),
                            no_worsening=bool(np.all(b <= a+error)))
    unchanged = np.array_equal(before[groups['remaining_front']], after[groups['remaining_front']])
    improved = float(before[selected_edge]-after[selected_edge]) > error
    return dict(passed=improved and unchanged and all(r['no_worsening'] for r in reports.values()),
        measured_recomputation_error_rad=error, selected_before_rad=float(before[selected_edge]),
        selected_after_rad=float(after[selected_edge]), selected_improved=improved,
        remaining_front_bit_equal=unchanged, groups=reports, hypothesis_criteria_only=True)


class Limits:
    def __init__(self, seconds, maximum, clock=time.perf_counter):
        if seconds <= 0 or maximum <= 0:
            raise ValueError('PositiveProbeLimitsRequired')
        self.clock = clock
        self.end = clock()+seconds
        self.maximum = maximum
        self.calls = 0

    def check(self):
        if self.clock() >= self.end:
            raise TimeoutError('PrivateProbeDeadline')

    def evaluation(self):
        self.check()
        self.calls += 1
        if self.calls > self.maximum:
            raise TimeoutError('PrivateProbeEvaluationLimit')


def solve(base, scale, normal_reference, halo_pairs, get_normals, constraints, limits, options, event):
    """Single SLSQP endpoint, with no restart, backoff, cleanup or admission.

    No analytic gradients are supplied. SciPy's bounded finite differences are
    used for the small coordinate domain; objective consistency is tested alone.
    """
    from scipy.optimize import minimize
    base = np.array(base, dtype='<f8', copy=True)
    if base.ndim != 2 or base.shape[1] != 3 or not np.isfinite(base).all() or not scale > 0:
        raise ValueError('InvalidProbeParameterization')
    cache = {}

    def values(z):
        limits.check()
        key = np.asarray(z, dtype='<f8').tobytes()
        if key not in cache:
            limits.evaluation()
            points = base+scale*np.asarray(z).reshape(-1, 3)
            slacks = np.asarray(constraints(points), dtype=float)
            r = residuals(get_normals(points), normal_reference, halo_pairs)
            if not np.isfinite(slacks).all() or not np.isfinite(r).all():
                raise ValueError('NonfiniteProbeEvaluation')
            cache.clear()
            cache[key] = (slacks, r)
        return cache[key]

    zero = np.zeros(base.size)
    slack, initial = values(zero)
    if np.any(slack < 0):
        raise ValueError('BaselineNumericalOrDomainFailure')
    initial_objective = float(initial.max())
    event('initial', dict(objective=initial_objective, constraints=len(slack), coordinates=base.tolist()))

    def inequality(y):
        slack, r = values(y[:-1])
        return np.r_[slack, y[-1]-r]

    fit = minimize(lambda y: float(y[-1]), np.r_[zero, initial_objective], method='SLSQP',
        bounds=[(-1., 1.)]*base.size+[(0., 1.)], constraints=[dict(type='ineq', fun=inequality)],
        options=dict(maxiter=options['maxiter'], ftol=options['ftol']))
    limits.check()
    if not np.isfinite(fit.x).all() or np.any(np.abs(fit.x[:-1]) > 1.):
        raise ValueError('InvalidSolverEndpoint')
    endpoint = base+scale*fit.x[:-1].reshape(-1, 3)
    slack, final = values(fit.x[:-1])
    record = dict(solver_success=bool(fit.success), solver_status=int(fit.status),
        solver_message=str(fit.message), iterations=int(fit.nit), evaluations=limits.calls,
        initial_objective=initial_objective, raw_objective=float(final.max()),
        raw_constraint_minimum=float(slack.min()), raw_coordinates=endpoint.tolist(),
        cleanup=False, scientific_transition_accepted=False)
    event('raw_proposal', record)
    return endpoint, record


def decide(checks, comparison_record, preservation):
    """All named retained-domain owners must pass. Missing is never success."""
    required = {'hard_numerical', 'full_front_shape', 'exact_vertex_envelope', 'endpoint_collision', 'bounds'}
    if set(checks) != required:
        raise ValueError('IncompleteProbeChecks')
    return dict(keep_hypothesis=bool(preservation and comparison_record['passed'] and all(checks.values())),
        checks=checks, preservation=bool(preservation), comparison=comparison_record,
        scientific_transition_accepted=False, generic_support=False,
        live_source_clearance=False, independent_collision_coverage=False, exact_filling=False,
        superiority_to_radial_only=False)


@contextmanager
def evidence(path, metadata):
    """Always retain partial progress, including cooperative timeout failures."""
    with path.open('x', encoding='utf-8') as stream:
        def event(stage, record):
            stream.write(json.dumps(dict(stage=stage, acceptance=False, **record), sort_keys=True)+'\n')
            stream.flush()
        event('start', metadata)
        try:
            yield event
        except BaseException as error:
            event('stopped', dict(error_type=type(error).__name__, error=str(error)))
            raise
        else:
            event('complete', dict(scientific_transition_accepted=False))
