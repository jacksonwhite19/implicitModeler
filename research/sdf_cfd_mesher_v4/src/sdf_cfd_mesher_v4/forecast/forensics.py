"""Diagnostic semantics and bounded, stable failure-component operations.

No geometry changes, warning thresholds, mesh admission or optimizer exists here.
"""
from enum import Enum
from fractions import Fraction as Q
import numpy as np

class GeometryUse(str,Enum):
    FORECAST='forecast'
    CANDIDATE='candidate'
    COMMITTED='committed'

def interpret_hard_events(events,use):
    """Diagnostic disposition only. A clear diagnostic never authorizes commit."""
    use=GeometryUse(use);events=list(events)
    kind={GeometryUse.FORECAST:'ForecastHardEvent',GeometryUse.CANDIDATE:'CandidateHardFailure',GeometryUse.COMMITTED:'CommittedHardFailure'}[use]
    return dict(use=use.value,event=kind if events else 'NoReportedHardEvent',hard_events=events,
                may_commit=False if events else None,committed_geometry_invalid=bool(events) and use==GeometryUse.COMMITTED)

def interpret_certificate(record,use):
    """Preserve exact certificate evidence; change only its consumer meaning."""
    use=GeometryUse(use);kind=record['classification']
    if kind=='CertifiedForbiddenIntersection':
        event={GeometryUse.FORECAST:'PredictedFutureIntersection',GeometryUse.CANDIDATE:'CandidateHardFailure',GeometryUse.COMMITTED:'CommittedHardFailure'}[use]
    elif kind in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):event='CertifiedPairClear'
    else:event='UnresolvedContinuousGeometry'
    return dict(use=use.value,event=event,certificate=record,may_commit=False if event!='CertifiedPairClear' else None,
                committed_geometry_invalid=use==GeometryUse.COMMITTED and event=='CommittedHardFailure')

def cell_events(r):
    values=dict(volume=r['volume']<=0 or r['signed_volume']<=0,jacobian=r['jacobian']<=0,determinant=r['determinant']<.0011,
                aspect=r['aspect']>1000,singular_operator=not r['G_bound_available'])
    return [k for k,v in values.items() if v]

def face_events(r):
    values=dict(interpolation_weight=r['weight']<.055,skewness=r['skewness']>3.63636,symmetric_volume_ratio=r['symmetric_volume_ratio']<.01,
                unavailable_neighbor_metric=not all(np.isfinite(r[k]) for k in ('weight','skewness','symmetric_volume_ratio','nonorthogonality')))
    return [k for k,v in values.items() if v]

def cluster_failures(topology,cells,faces,intersection):
    """Components of the induced failed-source-column graph, no halo expansion.

    Source-edge adjacency joins failure supports. Exact intersection evidence
    additionally joins its two surface supports. Layers remain event attributes.
    These are sampled failure footprints, not global shell components.
    """
    events=[]
    for r in cells:
        for kind in cell_events(r):events.append(dict(kind=kind,entity='cell',columns=[r['column']],layers=[r['layer']],id=r['cell_id']))
    for r in faces:
        for kind in face_events(r):events.append(dict(kind=kind,entity='face',columns=sorted(set((r['owner_column'],r['neighbor_column']))),layers=[r['layer']] if r['kind']=='side' else [r['layer'],r['layer']+1],id=[r['kind'],r['layer'],r['entity']]))
    sides=[intersection['side_a'],intersection['side_b']]
    support=sorted(set(int(c) for _,e in sides for c in topology.edge_triangles.row(e)))
    events.append(dict(kind='exact_intersection',entity='pair',columns=support,layers=sorted({k for k,_ in sides}),id=sides))
    columns=set(c for r in events for c in r['columns']);adj={c:set() for c in columns}
    for c in columns:
        adj[c].update(int(j) for e in topology.triangle_edges[c] for j in topology.edge_triangles.row(e) if int(j) in columns)
    for c in support:adj[c].update(support)
    groups=[];remaining=set(columns)
    while remaining:
        root=min(remaining);remaining.remove(root);group={root};work=[root]
        while work:
            neighbors=adj[work.pop()]&remaining;remaining-=neighbors;group|=neighbors;work.extend(sorted(neighbors))
        selected=[r for r in events if set(r['columns'])&group]
        groups.append(dict(component=f'component_{len(groups):02}',columns=sorted(group),events=selected))
    return groups

def first_invalid(events):
    result={}
    for r in events:
        # A cap interface is evaluated after its upper neighbor is available.
        available=max(r['layers']);result[r['kind']]=min(result.get(r['kind'],available),available)
    return result

def dependency_window(first_layer):
    if first_layer<1:raise ValueError('NoPrismAtLayerZero')
    return dict(earliest_committed_layer_with_causal_access=0,
                latest_committed_layer_before_invalid_transition=first_layer-1,
                argument='Each uncoupled nominal x[j+1] reads x[j]. Core metrics read both layer endpoints and complete fixed local neighbors. This is dependency reachability, not a proven feasible correction or trigger.')

def exact_wedge(prism):
    """Exact-rational arithmetic replay of the preserved wedge polynomial.

    Same definitions as prospective_commit_adapter._exact_wedge_jacobian_and_volume:
    J is affine on the reference triangle and quadratic on each vertical edge.
    Check endpoint values and each interior convex quadratic stationary point.
    No new metric, geometric tolerance, or alternative prism decomposition.
    """
    p=[tuple(Q(*float(x).as_integer_ratio()) for x in v) for v in prism]
    def sub(a,b):return tuple(x-y for x,y in zip(a,b))
    def add(a,b):return tuple(x+y for x,y in zip(a,b))
    def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
    def dot(a,b):return sum(x*y for x,y in zip(a,b))
    a0=sub(p[1],p[0]);a1=sub(sub(p[4],p[3]),a0);b0=sub(p[2],p[0]);b1=sub(sub(p[5],p[3]),b0)
    q0=cross(a0,b0);q1=add(cross(a1,b0),cross(a0,b1));q2=cross(a1,b1)
    vertical=[sub(p[j+3],p[j]) for j in range(3)];candidates=[]
    for j,v in enumerate(vertical):
        c0,c1,c2=[dot(v,q) for q in (q0,q1,q2)]
        for z in (Q(0),Q(1)):
            candidates.append((c0+c1*z+c2*z*z,j,z))
        if c2>0:
            z=-c1/(2*c2)
            if 0<z<1:candidates.append((c0+c1*z+c2*z*z,j,z))
    value,corner,z=min(candidates)
    mean=tuple(sum(v[i] for v in vertical)/6 for i in range(3));integ=tuple(q0[i]+q1[i]/2+q2[i]/3 for i in range(3));volume=dot(mean,integ)
    return dict(jacobian=str(value),jacobian_sign=(value>0)-(value<0),minimum_reference_corner=corner,minimum_reference_height=str(z),signed_volume=str(volume),volume_sign=(volume>0)-(volume<0))
