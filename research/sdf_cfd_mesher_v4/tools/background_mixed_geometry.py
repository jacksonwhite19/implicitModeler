"""Fresh mixed-cell Foundation arithmetic and existing exact dyadic primitives.

No receipt input. Used only by the explicit joined-background execution option.
"""
from fractions import Fraction
import numpy as np
from sdf_cfd_mesher_v4.forecast import exact_dyadic as dyadic


def exact_tetrahedron(points,cell,*,fallback=None,positive_signs_only=False):
    if type(positive_signs_only) is not bool:raise ValueError('InvalidExactPositiveSignOutput')
    import local_termination_topology as t
    vertices=set(cell['vertices']);faces=cell.get('faces')
    if not faces:return (fallback or t.exact_cell)(points,cell)
    face=faces[0]
    # Preserve the general/malformed source owner's behavior and scope.
    if (cell['kind']!='tetrahedron' or len(vertices)!=4 or len(face)!=3 or
            len(set(face))!=3 or not set(face)<=vertices or
            any(len(f)!=3 or len(set(f))!=3 or not set(f)<=vertices for f in cell['faces'])):
        return (fallback or t.exact_cell)(points,cell)
    apex=next(v for v in vertices if v not in face)
    ids=[*reversed(face),apex]
    # Keep the original point-conversion order, including competing infinity/
    # NaN errors. Reorder only the successfully converted exact coordinates.
    stored={v:[float(x).as_integer_ratio() for x in points[v]] for v in vertices}
    ratios=[ratio for v in ids for ratio in stored[v]]
    denominator=max(d for _,d in ratios)
    values=[n*(denominator//d) for n,d in ratios]
    a,b,d,p=[values[i:i+3] for i in range(0,12,3)]
    numerator=dyadic._dot(dyadic._cross(dyadic._sub(b,a),dyadic._sub(d,a)),dyadic._sub(p,a))
    sign=(numerator>0)-(numerator<0)
    if (positive_signs_only and sign>0 and type(cell) is dict
            and type(cell['vertices']) in (tuple,list) and len(cell['vertices'])==4
            and type(faces) in (tuple,list) and len(faces)==4
            and all(type(f) in (tuple,list) and all(type(v) is int for v in f) for f in faces)
            and len({tuple(sorted(f)) for f in faces})==4):
        return dict(jacobian_sign=sign,volume_sign=sign,method='AffineTetrahedronExactTripleProduct')
    scale=denominator**3
    return dict(jacobian=str(Fraction(numerator,scale)),jacobian_sign=sign,
                signed_volume=str(Fraction(numerator,6*scale)),volume_sign=sign,
                method='AffineTetrahedronExactTripleProduct')


def batch_geometry(points,cells,*,_determinants=None,_owners=None,_selected=None):
    """Existing prism batch expressions parameterized by source face count.

    Supports only homogeneous triangle/quad face-role shapes. Nonstandard
    cells use their existing scalar owner. No padding, face aliases or reordering.
    """
    import local_termination_topology as t
    if not cells:return {}
    shapes={tuple(map(len,c['faces'])) for c in cells}
    if (len(shapes)!=1 or any(n not in (3,4) for n in next(iter(shapes))) or
            any(c['kind'] not in ('prism','tetrahedron') for c in cells)):
        return {c['id']:t.geometry(points,c) for c in cells}
    count=len(cells[0]['faces']);fg=t.q.legacy_modules()['component_quality'].FG
    sf=np.empty((len(cells),count,3));cf=np.empty_like(sf)
    for role in range(count):
        ids=np.asarray([c['faces'][role] for c in cells],dtype=np.int64)
        fn=fg.face_area_centre_tri if ids.shape[1]==3 else fg.face_area_centre_quad
        sf[:,role],cf[:,role]=fn(points[ids])
    est=cf.mean(axis=1);pyr=np.einsum('nfi,nfi->nf',sf,cf-est[:,None,:])
    total=sum(pyr[:,j] for j in range(count));volumes=total/3
    centres=np.sum(pyr[:,:,None]*(.75*cf+.25*est[:,None,:]),axis=1)/total[:,None]
    bounds=abs(sf).sum(axis=1);result={}
    if _determinants is not None:
        # Fresh bounded outputs, not a receipt or caller-supplied tensor cache.
        from background_cell_tensor import determinants
        values=determinants(sf,cells,_owners,_selected)
        if values is not None:_determinants.update(values)
    for j,c in enumerate(cells):
        B=bounds[j];V=volumes[j]
        aspect=max(max(B)/min(B),sum(B)/(6*abs(V)**(2/3))) if V else float('inf')
        result[c['id']]=dict(centre=centres[j],volume=float(V),areas=sf[j],face_centres=cf[j],aspect=float(aspect))
    return result
