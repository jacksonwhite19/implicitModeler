"""Call-scoped solver arithmetic on immutable incidence and fresh coordinates.

This proposes coordinates, not acceptance. The source closure is guarded for
the whole group; per-trial coordinates have an unchanged-byte check. Existing
independent QA/admission still own every accepted mesh.
"""
from contextlib import contextmanager
from collections import defaultdict
import numpy as np
from sdf_cfd_mesher_v4.common.authority import frozen


def _tensor(areas,groups):
    result=np.empty(len(areas))
    for mask,ids in groups:
        S=areas[ids][:,mask]
        average=np.linalg.norm(S,axis=2).mean(axis=1)
        scaled=S/average[:,None,None]
        result[ids]=np.linalg.det(scaled.transpose(0,2,1)@scaled)
    return result


def _groups(masks):
    groups=defaultdict(list)
    for j,mask in enumerate(masks):groups[tuple(mask)].append(j)
    return tuple((frozen(mask,'?'),frozen(ids,'<i8')) for mask,ids in groups.items())


@contextmanager
def prepared(model,star,owners,needed,volume_scale,outer):
    """Own a complete source closure until the solver returns, before adoption.

    Unsupported eligibility and nonfinite results keep LocalRepair.constraints.
    Numerical exceptions propagate. No saved
    cache, caller-supplied proof or geometry-family dispatch is consumed.
    """
    legacy=lambda:model.constraints(star,owners,needed,volume_scale,prospective_outer=outer)
    from background_local_repair import LocalRepair
    if (type(model) is not LocalRepair or np.__version__!='1.26.4' or np.geterr()!=dict(divide='warn',over='warn',under='ignore',invalid='warn')
        or type(star) is not list or not star or len(star)>4096 or type(owners) not in (dict,defaultdict)
        or type(volume_scale) is not np.ndarray or volume_scale.dtype!=np.dtype('float64') or volume_scale.shape!=(len(star),)
        or any(type(rows) is not list for rows in owners.values())
        or any(type(c) is not dict or type(c.get('id')) is not int or type(c.get('kind')) is not str or c['kind']!='tetrahedron'
            or type(c.get('faces')) is not list or any(type(f) is not tuple for f in c['faces'])
            or tuple(map(len,c['faces']))!=(3,3,3,3) for c in star)):
        yield legacy;return
    if len({c['id'] for c in star})!=len(star) or any(model.byid.get(c['id']) is not c for c in star):
        yield legacy;return
    from prism_hotpath_v2 import _fresh_call_custody
    import local_termination_topology as t
    closure=[model.byid[i] for i in sorted(needed)]
    custody=_fresh_call_custody(model.original,closure,owners,outer)
    ids=sorted({v for c in closure for v in c['vertices']}|{v for c in closure for f in c['faces'] for v in f})
    ids=frozen(ids,'<i8');points=model.points
    if points[ids].tobytes()!=model.original[ids].tobytes():
        custody();yield legacy;return
    try:
        scale_binding=(volume_scale.dtype.str,volume_scale.shape,volume_scale.strides,volume_scale.tobytes())
        lookup={int(v):j for j,v in enumerate(ids)}
        faceids=frozen([[[lookup[v] for v in f] for f in c['faces']] for c in star],'<i8')
        interface={tuple(sorted(f)) for f in outer}
        actual=[[len(owners[tuple(sorted(f))])==2 for f in c['faces']] for c in star]
        final=[[len(owners[tuple(sorted(f))])==2 or tuple(sorted(f)) in interface for f in c['faces']] for c in star]
        if any(not any(mask) for mask in actual):
            yield legacy;return
        actual_groups=_groups(actual);final_groups=_groups(final)
        cellids=tuple(c['id'] for c in star);count=len(cellids);moving=set(cellids);positions={i:j for j,i in enumerate(cellids)}
        fixed={};fixed_bindings={}
        for i in needed-moving:
            g=t.geometry(points,model.byid[i]);cached=model.geometry[i]
            if np.asarray(g['centre']).tobytes()!=np.asarray(cached['centre']).tobytes() or float(g['volume']).hex()!=float(cached['volume']).hex():
                raise ValueError('ChangedLocalConstraintFixedGeometry')
            fixed[i]=dict(centre=frozen(g['centre'],'<f8'),volume=float(g['volume']))
            fixed_bindings[i]=(np.asarray(cached['centre']).tobytes(),float(cached['volume']).hex())
        paired=[r for _,r in sorted(owners.items()) if len(r)==2]
        pairids=frozen([[lookup[v] for v in r[0][1]] for r in paired],'<i8')
        pairs=tuple((r[0][0],r[1][0]) for r in paired)
        scales=frozen(volume_scale,'<f8');fg=t.q.legacy_modules()['component_quality'].FG
        cq=t.q.legacy_modules()['component_quality'];errors=np.geterr();shape=points.shape;strides=points.strides;alive=[True]
        if points[ids].tobytes()!=model.original[ids].tobytes():raise ValueError('ChangedLocalConstraintCoordinates')

        def evaluate():
            if not alive[0]:raise ValueError('ClosedLocalConstraintPlan')
            if model.points is not points or points.shape!=shape or points.strides!=strides or points.dtype!=np.dtype('float64'):
                raise ValueError('ChangedLocalConstraintCoordinates')
            if np.geterr()!=errors:raise ValueError('ChangedLocalConstraintArithmetic')
            raw=points[ids].tobytes();local=np.frombuffer(raw,dtype='<f8').reshape(-1,3)
            try:
                if not np.isfinite(local).all():return legacy()
                sf=np.empty((count,4,3));cf=np.empty_like(sf)
                for role in range(4):sf[:,role],cf[:,role]=fg.face_area_centre_tri(local[faceids[:,role]])
                est=cf.mean(axis=1);pyr=np.einsum('nfi,nfi->nf',sf,cf-est[:,None,:])
                total=sum(pyr[:,j] for j in range(4));volumes=total/3
                centres=np.sum(pyr[:,:,None]*(.75*cf+.25*est[:,None,:]),axis=1)/total[:,None]
                B=abs(sf).sum(axis=1)
                aspects=[max(max(b)/min(b),sum(b)/(6*abs(v)**(2/3))) if v else float('inf') for b,v in zip(B,volumes)]
                determinants=_tensor(sf,actual_groups)
                vals=[]
                for j in range(count):
                    vals.extend((volumes[j]/scales[j],determinants[j]/.0011-1,1000/aspects[j]-1))
                if pairs:
                    vs=local[pairids];S,C=fg.face_area_centre_tri(vs)
                    def geometry(i):
                        return dict(centre=centres[positions[i]],volume=volumes[positions[i]]) if i in positions else fixed[i]
                    a=[geometry(i) for i,_ in pairs];b=[geometry(i) for _,i in pairs]
                    weight,skew,ratio,_=cq.face_metrics(vs,S,C,np.array([g['centre'] for g in a]),
                        np.array([g['centre'] for g in b]),np.array([g['volume'] for g in a]),np.array([g['volume'] for g in b]))
                    vals.extend(weight/.055-1);vals.extend(ratio/.01-1);vals.extend(3.63636-skew)
                values=np.asarray(vals)
                # Preserve the original finite fallback order. Unusual trials
                # use the existing owner, including its scalar/error behavior.
                if not np.isfinite(values).all():return legacy()
                values[1:3*count:3]=_tensor(sf,final_groups)/.0011-1
                if not np.isfinite(values).all():return legacy()
                return values
            finally:
                if points[ids].tobytes()!=raw:raise ValueError('ChangedSameCallFreshGeometryInputs')
                if model.points is not points or points.shape!=shape or points.strides!=strides:raise ValueError('ChangedLocalConstraintCoordinates')
                if np.geterr()!=errors:raise ValueError('ChangedLocalConstraintArithmetic')
        yield evaluate
    finally:
        if 'alive' in locals():alive[0]=False
        custody()
        if 'scale_binding' in locals() and (volume_scale.dtype.str,volume_scale.shape,volume_scale.strides,volume_scale.tobytes())!=scale_binding:
            raise ValueError('ChangedLocalConstraintScale')
        if 'fixed_bindings' in locals():
            for i,binding in fixed_bindings.items():
                current=model.geometry[i]
                if (np.asarray(current['centre']).tobytes(),float(current['volume']).hex())!=binding:
                    raise ValueError('ChangedLocalConstraintFixedGeometry')
