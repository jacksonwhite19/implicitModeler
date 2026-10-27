"""Exact rank-aware dispatcher. The frozen rank-3 implementation is retained."""
from rank_geometry import *
import rank3_certificate as fullrank
def certify(a,b):
 if (b.rank,b.points,tuple(map(str,b.ids)),b.declared_dimension)<(a.rank,a.points,tuple(map(str,a.ids)),a.declared_dimension):a,b=b,a
 ranks=[a.rank,b.rank]
 regularity=[]
 for p in (a,b):
  ok,why=p.valid();regularity.append({'rank':p.rank,'declared_dimension':p.declared_dimension,'admission':why})
  if not ok:return {'classification':'InvalidSidePatch','method':'ExactRankRegularityFailure','ranks':ranks,'regularity':regularity,'hard_geometric_failure':True}
 c=owned_set(a,b)
 lo,hi=a.bounds;lb,hb=b.bounds
 if any(hi[i]<lb[i] or hb[i]<lo[i] for i in range(3)):
  return {'classification':'CertifiedSeparated','method':'FastAABB','ranks':ranks}
 if ranks==[3,3]:
  return {**fullrank.certify(a,b),'ranks':ranks}
 if a.rank==0:
  point=a.points[0];out=[point] if membership(point,b) else []
  r=geometric_result(out,c,'Rank0Exact',{'case':'ExactPointImageMembership','point':point,'other_rank':b.rank},False)
 elif a.rank==1:
  x,y=a.interval
  if b.rank==1:
   out=line_line(x,y,*b.interval);proof={'case':'ExactLineIntervalIntersection','interval_a':a.interval,'interval_b':b.interval}
  elif b.rank==2:
   out=segment_planar(x,y,b);proof={'case':'ExactLinePlaneAndConvexImageClipping','line_interval':a.interval,'plane':b.planar['normal'],'offset':b.planar['offset']}
  else:
   from rank_mixed_certificate import finite_line_quadric
   out,proof=finite_line_quadric(a.interval,b)
  r=geometric_result(out,c,'Rank1Exact',proof,convex=b.rank!=3 or proof['case']=='ExactRulingInterval')
 elif b.rank==2:
  out,proof=plane_plane(a,b);r=geometric_result(out,c,'Rank2Exact',proof)
 else:
  from rank_mixed_certificate import mixed_planar
  r=mixed_planar(b,a,c)
 return {**r,'ranks':ranks,'regularity':regularity}
