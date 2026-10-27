"""Mechanical dispatch to retained proofs and exact noncoincident 2D CAD."""
from rank_geometry import Patch
from retained_canonical_adapter import canonical_pair,canonical_object
import retained_rank_dispatch as lower
import rank3_certificate as rank3
from coincident_rank3_certificate import coincident
from noncoincident_rank3_certificate import cert as exact_noncoincident
from functools import lru_cache

def certify(a,b):
 a,b=canonical_pair(a,b)
 return cached_certificate(a.points,a.ids,a.declared_dimension,b.points,b.ids,b.declared_dimension)

@lru_cache(maxsize=8192)
def cached_certificate(ap,ai,ad,bp,bi,bd):
 a=Patch(ap,ai,ad);b=Patch(bp,bi,bd)
 if a.rank!=3 or b.rank!=3 or a.declared_dimension!=2 or b.declared_dimension!=2:return canonical_object(lower.certify(a,b))
 lo,hi=a.bounds;lb,hb=b.bounds
 if any(hi[i]<lb[i] or hb[i]<lo[i] for i in range(3)):
  return {'classification':'CertifiedSeparated','method':'FastAABB','ranks':[3,3]}
 r=coincident(a,b)
 if r is not None:return canonical_object(r)
 c=rank3.contact(a,b)
 for call in (rank3.supporting_planes,rank3.radial,rank3.implicit):
  r=call(a,b,c)
  if r:return canonical_object({**r,'ranks':[3,3]})
 r=rank3.implicit(b,a,c)
 if r:return canonical_object({**r,'ranks':[3,3]})
 return canonical_object(exact_noncoincident(a,b))
