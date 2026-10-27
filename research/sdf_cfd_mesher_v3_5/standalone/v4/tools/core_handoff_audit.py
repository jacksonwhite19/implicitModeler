"""Descriptive pre-core admission evidence; no invented geometric thresholds.

Shell qualification is necessary but does not qualify a tet-core handoff.
Distances below are diagnostics in binary64, not exact intersection proofs.
"""
import json
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash, identity
from geometry_family_authority import Source

VERSION = 'V4CoreHandoffAudit/1'
QUANTILES = (0, .5, .9, .95, .99, .995, .999, 1)
LABELS = ('min', 'median', 'p90', 'p95', 'p99', 'p99.5', 'p99.9', 'max')


def distribution(values):
    a = np.asarray(values)
    if not a.size:
        return None
    if not np.isfinite(a).all():
        raise ValueError('NonfiniteHandoffDiagnostic')
    return dict(zip(LABELS, map(float, np.quantile(a, QUANTILES))))


def surface(x, f):
    x = np.asarray(x); f = np.asarray(f)
    if f.ndim != 2 or f.shape[1] != 3 or not np.isfinite(x).all():
        raise ValueError('TriangularFiniteInterfaceRequired')
    if np.any(f < 0) or np.any(f >= len(x)):
        raise ValueError('InterfaceIndex')
    edges = np.unique(np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), 1), axis=0)
    q = x[f]
    lengths = np.linalg.norm(q - np.roll(q, -1, axis=1), axis=2)
    av = np.cross(q[:, 1] - q[:, 0], q[:, 2] - q[:, 0]) / 2
    area = np.linalg.norm(av, axis=1)
    if np.any(area == 0) or np.any(lengths == 0):
        raise ValueError('DegenerateInterfaceTriangle')
    a, b, c = lengths.T
    angles = np.degrees(np.arccos(np.clip(np.stack([(b*b+c*c-a*a)/(2*b*c),
        (c*c+a*a-b*b)/(2*c*a), (a*a+b*b-c*c)/(2*a*b)], 1), -1, 1)))
    semiperimeter = lengths.sum(1)/2
    values = dict(longest_edge=lengths.max(1), shortest_edge=lengths.min(1),
        edge_ratio=lengths.max(1)/lengths.min(1), area=area,
        minimum_angle=angles.min(1), maximum_angle=angles.max(1),
        circumradius=a*b*c/(4*area), inradius=area/semiperimeter,
        # Descriptive, named standard shape measure; not a new gate.
        radius_ratio=(a*b*c/(4*area))/(2*area/semiperimeter))
    el = np.linalg.norm(x[edges[:, 0]] - x[edges[:, 1]], axis=1)
    return dict(edges=edges, edge_length=el, face_edges=lengths, normal=av/area[:, None],
                centre=q.mean(1), **values)


def medial_depth(x, spec, branch=0):
    """Distance to the declared primitive's medial curve/point, in metres.

    Only an inward single-component analytic source is currently supported.
    This is remaining inward capacity, not two-sided thickness. Unknown and
    competing branches return no value; they are never guessed from a schedule.
    """
    if not spec.get('cavity') or len(spec['components']) != 1 or branch != 0:
        raise ValueError('CoreDepthBranchNotSupported')
    d = spec['components'][branch]
    p = np.asarray(x) - np.asarray(d.get('centre', [0, 0, 0]))
    kind = d['primitive']
    if kind == 'bent_capsule':
        angle = np.clip(np.arctan2(p[:, 1], p[:, 0]), -d['angle'], d['angle'])
        near = np.c_[d['major']*np.cos(angle), d['major']*np.sin(angle), np.zeros(len(p))]
    elif kind == 'capsule':
        near = np.zeros_like(p); near[:, 2] = np.clip(p[:, 2], -d['half_length'], d['half_length'])
    else:
        raise ValueError('CoreDepthPrimitiveNotSupported')
    values = np.linalg.norm(p-near, axis=1)
    sample = Source(spec).evaluate(x)
    if np.any(sample.boundary_normal_selection != branch):
        raise ValueError('CoreDepthBranchMismatch')
    return values, dict(method='Euclidean distance to authoritative medial curve; one-sided inward capacity',
        source_identity=Source(spec).source_identity, source_branch=branch,
        primitive=kind, implementation=VERSION)


def nearest_surface(points, tri, progress=None):
    """All-triangle distance query: no nearest-centroid approximation/screening."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    u = b-a; v = c-a
    uu = (u*u).sum(1); uv = (u*v).sum(1); vv = (v*v).sum(1)
    den = uu*vv-uv*uv
    if np.any(den <= 0):
        raise ValueError('DegenerateDistanceTriangle')
    ids = np.empty(len(points), int); distance = np.empty(len(points))
    for i, p in enumerate(points):
        w = p-a; wu = (w*u).sum(1); wv = (w*v).sum(1)
        s = (wu*vv-wv*uv)/den; t = (wv*uu-wu*uv)/den
        ds = ((w-s[:, None]*u-t[:, None]*v)**2).sum(1)
        ds[(s < 0) | (t < 0) | (s+t > 1)] = np.inf
        for start, end in ((a, b), (b, c), (c, a)):
            edge = end-start; along = np.clip(((p-start)*edge).sum(1)/(edge*edge).sum(1), 0, 1)
            ds = np.minimum(ds, ((p-start-along[:, None]*edge)**2).sum(1))
        ids[i] = np.argmin(ds); distance[i] = np.sqrt(ds[ids[i]])
        if progress is not None and i % 256 == 0:
            progress(i, len(points))
    return ids, distance


def lineage_class(source_pair, wall_edges, terminal_length, wall_length, layers, owner_types):
    if len(set(map(int, source_pair))) != 2:
        return 'transition-created'
    if tuple(sorted(map(int, source_pair))) not in wall_edges:
        return 'transition-created' if set(owner_types) - {'prism'} else 'unresolved'
    if len(set(map(int, layers))) > 1:
        return 'mixed'
    # "Stretch" here means measured net length growth, with no tolerance/gate.
    return 'inherited with stretch' if terminal_length > wall_length else 'directly inherited'


def pre_core_audit(interface, source_spec=None):
    path = Path(interface)
    x = np.load(path/'points.npy'); f = np.load(path/'connectivity.npy').reshape(-1, 3)
    m = json.loads((path/'INTERFACE.json').read_text())
    offsets = np.load(path/'offsets.npy')
    for key, arr in [('coordinates', x), ('connectivity', f.ravel()), ('offsets', offsets)]:
        if array_hash(arr) != m[key]:
            raise ValueError('CoreHandoffInterfaceBindingMismatch:'+key)
    a = surface(x, f)
    owner = [r['cell_type'] for r in m['ancestry']]
    result = dict(schema=VERSION, interface_identity=m['identity'],
        shell_qualification='Separate prerequisite; not inferred by this audit',
        core_handoff_status='CoreHandoffNotEstablished',
        numeric_handoff_gate=None, edge_distribution=distribution(a['edge_length']),
        triangle_distributions={k:distribution(a[k]) for k in ('longest_edge', 'shortest_edge', 'edge_ratio', 'area', 'minimum_angle', 'maximum_angle', 'circumradius', 'inradius', 'radius_ratio')},
        owners={k:owner.count(k) for k in sorted(set(owner))},
        depth='Not measured', depth_reason='Source branch authority not supplied',
        risk_localization=dict(method='Stable descending edge length and circumradius; descriptive ordering only',
            edge_order=np.argsort(-a['edge_length'], kind='stable').tolist(),
            face_order=np.argsort(-a['circumradius'], kind='stable').tolist(),
            face_owner=owner,
            face_layer=[r.get('layer') for r in m['ancestry']],
            source_column=[r.get('source') for r in m['ancestry']],
            last_prism_layer=[r.get('column',{}).get('last_prism_layer') for r in m['ancestry']]),
        admission_rule='Generate diagnostic core only. CoreHandoffQualified requires identity-bound passing core, interface and complete geometry evidence; shell gates alone do not grant it.')
    if source_spec is not None:
        try:
            depth, binding = medial_depth(x, source_spec)
            result.update(depth=distribution(depth), depth_binding=binding, depth_reason=None)
            result['risk_localization']['vertex_depth'] = depth.tolist()
            face_depth,_=medial_depth(a['centre'],source_spec)
            edge_depth,_=medial_depth(x[a['edges']].mean(1),source_spec)
            result['risk_localization']['face_depth'] = face_depth.tolist()
            result['risk_localization']['edge_depth'] = edge_depth.tolist()
            if np.all(face_depth>0) and np.all(edge_depth>0):
                result['edge_over_midpoint_depth'] = distribution(a['edge_length']/edge_depth)
                result['circumradius_over_centroid_depth'] = distribution(a['circumradius']/face_depth)
            else:
                result['depth_reason']='Medial zero sampled; ratios not defined'
        except ValueError as exc:
            result['depth_reason'] = str(exc)
    result['identity'] = identity(result)
    return result
