"""Compose existing exact star witnesses; no joint endpoint shortcut.

The caller supplies complete stars from protected full incidence. Final witness
replay must independently enumerate all original tetrahedra. No publication.
"""
import numpy as np
from background_local_repair import sphere_link,preserve_step
from background_mixed_geometry import exact_tetrahedron


def compose_group(points,stars,targets,fixed,*,max_depth,max_attempts):
    vertices=sorted(targets);goal=np.array([targets[v] for v in vertices],dtype=np.float64)
    if (not vertices or goal.shape!=(len(vertices),3) or not np.isfinite(goal).all()
            or any(v in fixed or v not in stars for v in vertices)):
        raise ValueError('InvalidCoordinatedScope')
    if type(max_depth) is not int or type(max_attempts) is not int or min(max_depth,max_attempts)<1:
        raise ValueError('InvalidPreservationWorkBound')
    original=points[vertices].copy();witnesses=[];attempts=0
    union={c['id']:c for v in vertices for c in stars[v]}
    for v in vertices:sphere_link(stars[v],v)
    # Reject a nonpositive joint target early. Positivity here is necessary,
    # but never sufficient for preservation or acceptance.
    try:
        points[vertices]=goal
        for c in union.values():
            r=exact_tetrahedron(points,c,positive_signs_only=True)
            if r['volume_sign']<=0 or r['jacobian_sign']<=0:raise ValueError('NonpositiveJointEndpoint')
    finally:points[vertices]=original
    def advance(target,depth):
        nonlocal attempts
        entry=points[vertices].copy();tail=len(witnesses)
        try:
            for j,v in enumerate(vertices):
                if points[v].tobytes()==target[j].tobytes():continue
                attempts+=1
                if attempts>max_attempts:raise ValueError('PreservationWorkBound')
                w=preserve_step(points,stars[v],v,target[j],fixed)
                points[v]=target[j];witnesses.append(w)
            return
        except ValueError as failure:
            points[vertices]=entry;del witnesses[tail:]
            if str(failure)=='PreservationWorkBound' or depth>=max_depth:raise
        midpoint=entry/2+target/2
        if (not np.isfinite(midpoint).all() or midpoint.tobytes() in (entry.tobytes(),target.tobytes())):
            raise ValueError('NoRepresentablePreservationProgress')
        try:
            advance(midpoint,depth+1);advance(target,depth+1)
        except BaseException:
            points[vertices]=entry;del witnesses[tail:];raise
    try:advance(goal,0)
    except BaseException:
        points[vertices]=original;witnesses.clear();raise
    return dict(witnesses=witnesses,attempts=attempts,affected_cells=sorted(union))
