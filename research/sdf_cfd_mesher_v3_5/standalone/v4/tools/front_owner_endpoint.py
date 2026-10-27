"""Round proposed row endpoints against the existing actual-owner arithmetic.

No new predicate: reuse Foundation geometry and face_metrics, then require the
ordinary complete mixed quality owner. Only private new-front points may move.
"""
import numpy as np
from tet_layer_row import product,RowConflict

def close_weight(front,new,faces,centres,volumes,controls,check=None):
    from background_materialization import build
    from background_mixed_geometry import batch_geometry
    import local_termination_topology as topology
    cq=topology.q.legacy_modules()['component_quality'];fg=cq.FG
    front=np.asarray(front);new=np.asarray(new).copy();faces=np.asarray(faces)
    cells=build(product(faces,len(front))[::3],0)
    S,C=fg.face_area_centre_tri(front[faces]);apex=faces.max(axis=1)
    direction=new-front;direction/=np.linalg.norm(direction,axis=1)[:,None]
    original=new.copy();history=[]
    for attempt in range(controls['maxiter']):
        if check:check()
        points=np.concatenate((front,new));g=batch_geometry(points,cells)
        tc=np.asarray([g[c['id']]['centre'] for c in cells]);tv=np.asarray([g[c['id']]['volume'] for c in cells])
        weight,skew,ratio,_=cq.face_metrics(front[faces],S,C,np.asarray(centres),tc,np.asarray(volumes),tv)
        bad=weight<.055
        history.append(dict(iteration=attempt,failed_weight=int(bad.sum()),minimum_weight=float(weight.min())))
        if not bad.any():
            changed=np.flatnonzero(np.any(new!=original,axis=1))
            return new,dict(method='ExistingFoundationFaceWeight/RepresentableEndpoint',iterations=history,
                changed_vertices=len(changed),maximum_displacement_m=float(np.linalg.norm(new-original,axis=1).max()),
                complete_quality_and_geometry_still_required=True)
        dp=np.einsum('ni,ni->n',S,C-centres);dt=np.einsum('ni,ni->n',S,tc-C)
        if np.any(dp<=0) or np.any(dt<=0):raise RowConflict('InvalidActualOwnerWeightOrientation')
        outward=np.unique(apex[bad&(dt<dp)]);inward=np.unique(apex[bad&(dt>=dp)])
        if np.intersect1d(outward,inward).size:raise RowConflict('ConflictingActualOwnerEndpointDirections')
        new[outward]=np.nextafter(new[outward],new[outward]+direction[outward])
        new[inward]=np.nextafter(new[inward],new[inward]-direction[inward])
    raise RowConflict('ActualOwnerWeightEndpointBound',history=history)
