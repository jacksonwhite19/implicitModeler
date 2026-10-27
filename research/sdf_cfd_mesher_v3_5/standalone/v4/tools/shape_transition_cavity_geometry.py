"""Existing cavity point-membership contract through V4 exact cell owners.

Source cavities must already have valid closed oriented incidence. A retained
interior source vertex is admitted by its complete source star. Every other
point must lie strictly inside one original tetrahedron; a new shared-face
point is deliberately not sufficient. No source-fluid or mesh acceptance.
"""
from fractions import Fraction
import numpy as np


def _volume(q):
    from background_materialization import build
    import local_termination_topology as t
    cell=build(np.array([[0,1,2,3]],dtype='<i8'),0)[0]
    return Fraction(t.exact_cell(q,cell)['signed_volume'])


def strict_inside_tet(p,q):
    volume=_volume(q)
    if not volume:return False
    parts=[]
    for i in range(4):
        replaced=q.copy();replaced[i]=p;parts.append(_volume(replaced))
    return all(v*volume>0 for v in parts) and sum(parts)==volume


def interior_point_valid(p,x,t):
    """Preserve byte-exact source coincidence and strict dyadic membership."""
    same=np.flatnonzero(np.all(np.asarray(x).view('u8')==np.asarray(p).view('u8'),axis=1))
    if len(same):
        from background_materialization import build
        from background_topology_repair import boundary
        _,faces,_=boundary(build(t,0));exterior=np.unique(faces)
        if np.intersect1d(same,exterior).size:return False
        return bool(np.intersect1d(same,np.unique(t)).size)
    return any(strict_inside_tet(p,x[row]) for row in t)
