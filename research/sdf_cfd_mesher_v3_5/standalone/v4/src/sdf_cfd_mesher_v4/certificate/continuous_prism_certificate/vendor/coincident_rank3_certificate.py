"""Exact bounded images on one full-rank bilinear surface.

No tolerance, sampling, nonlinear solve or subdivision is used here.
"""
from fractions import Fraction as Q
from rank_geometry import Patch,owned_set,owned,encode,eq,membership
from rank3_certificate import sub,dot,clean,padd,pmul,PARAM_CORNERS

def chart(a,b):
 """Coordinates of B in A's exact [A,D,E] chart, and coincidence polynomial."""
 C,*_=a.terms
 Cb,A,D,E=b.terms
 vectors=(sub(Cb,C),A,D,E);keys=((0,0),(1,0),(0,1),(1,1))
 X,Y,Z=[clean({k:dot(row,w) for k,w in zip(keys,vectors)}) for row in a.inverse]
 return (X,Y,Z),padd(Z,pmul(X,Y),-Q(1))

def poly_value(p,s,t):return sum((c*s**i*t**j for (i,j),c in p.items()),Q(0))
def poly_record(p):return [[i,j,str(p[i,j])] for i,j in sorted(p)]

def coincident(a,b):
 """Return None unless BOTH ranks are 3 and exact coincidence is established."""
 if a.rank!=3 or b.rank!=3:return None
 polys,F=chart(a,b)
 if F:return None
 X,Y,Z=polys
 # Z=XY has no s^2 or t^2 terms. Integral-domain factorization proves
 # X_s=0 or Y_s=0, and X_t=0 or Y_t=0. Nonzero full rank forces one
 # coordinate to depend affinely only on s, the other only on t.
 dxs={j:c for (i,j),c in X.items() if i==1};dys={j:c for (i,j),c in Y.items() if i==1}
 dxt={i:c for (i,j),c in X.items() if j==1};dyt={i:c for (i,j),c in Y.items() if j==1}
 axis=None
 if not dxt and not dys:axis='direct'
 elif not dxs and not dyt:axis='swapped'
 else:
  return {'classification':'InvalidSidePatch','method':'CoincidentChartRegularityFailure','reason':'ExactCoincidenceStructureContradiction','hard_geometric_failure':True}
 x0=X.get((0,0),Q(0));y0=Y.get((0,0),Q(0))
 kx=(1,0) if axis=='direct' else (0,1);ky=(0,1) if axis=='direct' else (1,0)
 ax=X.get(kx,Q(0));ay=Y.get(ky,Q(0));jac=ax*ay*(1 if axis=='direct' else -1)
 if not jac:
  return {'classification':'InvalidSidePatch','method':'CoincidentChartRegularityFailure','reason':'CollapsedChart','hard_geometric_failure':True}
 xb=sorted((x0,x0+ax));yb=sorted((y0,y0+ay))
 lo=(max(Q(0),xb[0]),max(Q(0),yb[0]));hi=(min(Q(1),xb[1]),min(Q(1),yb[1]))
 c=owned_set(a,b)
 proof={'coincidence_polynomial':[],'A_inverse':encode(a.inverse),'A_origin':encode(a.terms[0]),'B_chart':[poly_record(p) for p in polys],
  'chart_structure':axis,'chart_jacobian':str(jac),'corner_jacobians':[str(jac)]*4,'injectivity':'Two nonzero separate affine coordinate functions; inverse is unique',
  'image_theorem':'s^2 coefficient gives X_s*Y_s=0 in Q[t]; t^2 gives X_t*Y_t=0 in Q[s]. Full rank excludes a constant chart coordinate. Only separable affine direct/swapped maps remain.',
  'A_rectangle':[['0','1'],['0','1']],'B_rectangle':encode((xb,yb)),'intersection_bounds':encode((lo,hi)),
  'boundary_arrangement':'Four exact axis-aligned segments per rectangle; interval clipping includes boundary contacts and containment','sampled_evidence':False}
 result={'method':'CoincidentRank3ExactImage','ranks':[3,3],'proof':proof,'authorized_contact':encode(c),'chart_regular':True}
 if any(lo[i]>hi[i] for i in range(2)):
  return {'classification':'CertifiedSeparated',**result,'exclusion':'Exact empty coordinate-interval intersection'}
 dim=sum(lo[i]<hi[i] for i in range(2))
 xy=sorted(set((x,y) for x in (lo[0],hi[0]) for y in (lo[1],hi[1])))
 def image(p):return a.at(*p)
 vertices=[image(p) for p in xy]
 center=tuple((lo[i]+hi[i])/2 for i in range(2))
 witnesses=[center]+xy
 forbidden=next((p for p in witnesses if not owned(image(p),c)),None)
 result.update(intersection_dimension=dim,intersection_chart_vertices=encode(xy),intersection_image_vertices=encode(vertices))
 if forbidden is None:
  # The intersection is a point or a ruling segment. A positive area
  # rectangle cannot have all four distinct corners on one allowed segment.
  if dim==2:raise ArithmeticError('PositiveAreaCannotBeOwnedByOneEdge')
  return {'classification':'CertifiedAllowedIncidentContact',**result,'legal_contact_only':True,
   'ownership_proof':'All exact interval endpoints lie in one authorized point or segment; the intersection is a point or surface ruling segment'}
 x,y=forbidden;s=(x-x0)/ax;t=(y-y0)/ay
 if axis=='swapped':s,t=t,s
 point=image(forbidden)
 assert 0<=s<=1 and 0<=t<=1 and point==b.at(s,t)
 assert membership(point,a) and membership(point,b) and not owned(point,c)
 kind='InteriorInterior' if dim==2 and all(0<q<1 for q in (*forbidden,s,t)) else ('BoundaryInteriorOrBoundaryContact' if dim<=1 else 'PositiveAreaOverlap')
 return {'classification':'CertifiedForbiddenIntersection',**result,'point':encode(point),'point_enclosure':[[str(z),str(z)] for z in point],
  'parameters':encode((*forbidden,s,t)),'parameter_enclosure':[[str(z),str(z)] for z in (*forbidden,s,t)],'residual_exact':'0',
  'contact_type':kind,'witness_authority':'Exact rectangle image intersection and unique inverse chart parameters'}
