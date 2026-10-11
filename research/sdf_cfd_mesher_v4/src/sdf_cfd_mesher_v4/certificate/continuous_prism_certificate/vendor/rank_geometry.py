"""Exact image geometry and topology ownership; no floating predicates."""
from fractions import Fraction as Q
from functools import cached_property,cmp_to_key
import itertools
from rank3_certificate import Patch as BasePatch,add,sub,mul,dot,cross,nonzero,PARAM_CORNERS,EDGE_CORNERS
from incident_contact_authority import contact
def sign(x):
 if isinstance(x,(int,Q)):return (x>0)-(x<0)
 import sympy as s
 x=s.cancel(x)
 if x==0:return 0
 y=s.sign(x)
 if y in (s.Integer(-1),s.Integer(0),s.Integer(1)):return int(y)
 y=s.simplify(x)
 if y.is_positive is True:return 1
 if y.is_negative is True:return -1
 if y.is_zero is True:return 0
 raise ArithmeticError('ExactAlgebraicSignUndecided:'+str(x))
def eq(a,b):return all(sign(x-y)==0 for x,y in zip(a,b))
def encode(x):
 if isinstance(x,dict):return {str(k):encode(v) for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [encode(v) for v in x]
 if isinstance(x,(int,bool)) or x is None:return x
 return str(x)
class Patch(BasePatch):
 def __init__(self,points,ids,declared_dimension=2):
  super().__init__(points,ids);self.declared_dimension=declared_dimension
 @classmethod
 def binary(cls,points,ids,declared_dimension=2):
  return cls(tuple(tuple(Q(*float(v).as_integer_ratio()) for v in p) for p in points),tuple(ids),declared_dimension)
 @cached_property
 def rank(self):
  _,A,D,E=self.terms
  if dot(A,cross(D,E)):return 3
  if any(nonzero(cross(a,b)) for a,b in itertools.combinations((A,D,E),2)):return 2
  return int(any(nonzero(x) for x in (A,D,E)))
 @cached_property
 def planar(self):
  C,*vec=self.terms
  e=next(x for x in vec if nonzero(x));f=next(x for x in vec if nonzero(cross(e,x)))
  n=cross(e,f);axes=next((i,j) for i,j in itertools.combinations(range(3),2) if e[i]*f[j]-e[j]*f[i])
  A,D,E=vec
  jac=[(A[axes[0]]+v*E[axes[0]])*(D[axes[1]]+u*E[axes[1]])-(A[axes[1]]+v*E[axes[1]])*(D[axes[0]]+u*E[axes[0]]) for u,v in PARAM_CORNERS]
  regular=all(x>0 for x in jac) or all(x<0 for x in jac)
  poly=[self.points[i] for i in (0,1,3,2)]
  normals=cross(sub(poly[1],poly[0]),sub(poly[2],poly[1]))
  hs=[(cross(normals,sub(poly[(i+1)%4],poly[i])),poly[i]) for i in range(4)]
  return {'regular':regular,'normal':n,'offset':dot(n,C),'axes':axes,'jacobians':jac,'polygon':poly,'halfspaces':hs,'basis_vectors':(e,f)}
 @cached_property
 def interval(self):
  axis=next(j for j in range(3) if len(set(p[j] for p in self.points))>1)
  lo=min(self.points,key=lambda p:p[axis]);hi=max(self.points,key=lambda p:p[axis])
  return lo,hi
 def valid(self):
  if self.declared_dimension==2:
   if self.rank<2:return False,'CollapsedTwoDimensionalSide'
   if self.rank==2 and not self.planar['regular']:return False,'NonRegularPlanarBilinearSide'
   return True,'Rank3Injective' if self.rank==3 else 'StrictCornerJacobiansConvexJordanImage'
  if self.declared_dimension==1:return self.rank==1,'DeclaredLineImage'
  if self.declared_dimension==0:return self.rank==0,'DeclaredPointImage'
  return False,'InvalidDeclaredDimension'
def owned_set(a,b):
 c=contact(a,b);points=[]
 for label in sorted(c['vertices'],key=str):
  p=a.points[a.ids.index(label)]
  if not any(eq(p,q) for q in points):points.append(p)
 segments=[]
 for edge in c['edges']:
  labels=list(edge)
  if len(labels)!=2:continue
  x,y=[a.points[a.ids.index(z)] for z in labels]
  if x!=y and not any({x,y}==set(e) for e in segments):segments.append((x,y))
 if len(segments)>1:raise ValueError('MoreThanOneAllowedSharedEdge')
 if not segments and len(points)>1:raise ValueError('MoreThanOneAllowedSharedEndpoint')
 if segments and any(not onsegment(p,*segments[0]) for p in points):raise ValueError('EndpointOutsideAllowedEdge')
 return {'points':points,'segments':segments}
def onsegment(p,a,b):
 d=sub(b,a);w=sub(p,a)
 if not nonzero(d):return eq(p,a)
 return all(sign(x)==0 for x in cross(w,d)) and sign(dot(w,d))>=0 and sign(dot(w,d)-dot(d,d))<=0
def owned(p,c):return any(eq(p,x) for x in c['points']) or any(onsegment(p,*e) for e in c['segments'])
def dedup(points):
 out=[]
 for p in points:
  if not any(eq(p,q) for q in out):out.append(p)
 return out
def membership(p,side):
 if side.rank==0:return eq(p,side.points[0])
 if side.rank==1:return onsegment(p,*side.interval)
 if side.rank==2:
  v=side.planar
  return sign(dot(v['normal'],p)-v['offset'])==0 and all(sign(dot(n,sub(p,a)))>=0 for n,a in v['halfspaces'])
 C,*_=side.terms
 X,Y,Z=[dot(row,sub(p,C)) for row in side.inverse]
 return sign(Z-X*Y)==0 and all(sign(t)>=0 and sign(1-t)>=0 for t in (X,Y))
def geometric_result(points,c,method,proof,convex=True):
 """A finite point set, or vertices of the proved exact convex intersection."""
 points=dedup(points)
 record={'method':method,'proof':encode(proof),'authorized_contact':encode(c)}
 if not points:return {'classification':'CertifiedSeparated',**record}
 bad=next((p for p in points if not owned(p,c)),None)
 if bad is None and convex:
  mid=tuple(sum(p[j] for p in points)/len(points) for j in range(3))
  if not owned(mid,c):bad=mid
 if bad is not None:
  return {'classification':'CertifiedForbiddenIntersection',**record,'point':encode(bad),'point_enclosure':[[str(v),str(v)] for v in bad],'witness_authority':'Exact image intersection membership','intersection_set_vertices':encode(points),'residual_exact':'0'}
 return {'classification':'CertifiedAllowedIncidentContact',**record,'intersection_set_vertices':encode(points),'legal_contact_only':True}
def clip_polygon(poly,halfspaces):
 for n,a in halfspaces:
  if not poly:break
  out=[];prev=poly[-1];fp=dot(n,sub(prev,a));ip=sign(fp)>=0
  for cur in poly:
   fc=dot(n,sub(cur,a));ic=sign(fc)>=0
   if ic!=ip:
    t=fp/(fp-fc);out.append(add(prev,mul(t,sub(cur,prev))))
   if ic:out.append(cur)
   prev,fp,ip=cur,fc,ic
  poly=dedup(out)
 return poly
def clip_line(origin,direction,halfspaces,lo=None,hi=None):
 for n,a in halfspaces:
  f=dot(n,sub(origin,a));g=dot(n,direction);sg=sign(g)
  if sg==0:
   if sign(f)<0:return None
   continue
  root=-f/g
  if sg>0:
   if lo is None or sign(root-lo)>0:lo=root
  else:
   if hi is None or sign(root-hi)<0:hi=root
  if lo is not None and hi is not None and sign(lo-hi)>0:return None
 return lo,hi
def segment_planar(a,b,p):
 plane=p.planar;d=sub(b,a);f=dot(plane['normal'],a)-plane['offset'];g=dot(plane['normal'],d)
 if sign(g)==0:
  if sign(f)!=0:return []
  r=clip_line(a,d,plane['halfspaces'],Q(0),Q(1))
  return [] if r is None else [add(a,mul(t,d)) for t in r]
 t=-f/g
 if sign(t)<0 or sign(1-t)<0:return []
 x=add(a,mul(t,d))
 return [x] if membership(x,p) else []
def plane_plane(a,b):
 pa,pb=a.planar,b.planar;n,m=pa['normal'],pb['normal'];r=cross(n,m)
 if not nonzero(r):
  if dot(n,sub(b.points[0],a.points[0]))!=0:return [],{'case':'ExactParallelDistinctPlanes','normal_a':n,'normal_b':m}
  out=clip_polygon(pa['polygon'],pb['halfspaces'])
  return out,{'case':'ExactCoplanarConvexImageClipping','planar_images':'Strict same-sign corner Jacobians; simple convex boundary; bilinear image equals closed convex quadrilateral','axes_a':pa['axes'],'axes_b':pb['axes'],'jacobians_a':pa['jacobians'],'jacobians_b':pb['jacobians']}
 r2=dot(r,r);origin=mul(1/r2,add(mul(pa['offset'],cross(m,r)),mul(pb['offset'],cross(r,n))))
 interval=clip_line(origin,r,pa['halfspaces']+pb['halfspaces'])
 out=[] if interval is None else [add(origin,mul(t,r)) for t in interval]
 return out,{'case':'ExactPlaneIntersectionLineClipping','line_origin':origin,'line_direction':r,'intersection_interval':interval}
def line_line(a,b,c,d):
 e=sub(b,a);f=sub(d,c);n=cross(e,f)
 if nonzero(n):
  if dot(sub(c,a),n)!=0:return []
  t=dot(cross(sub(c,a),f),n)/dot(n,n);s=dot(cross(sub(c,a),e),n)/dot(n,n)
  return [add(a,mul(t,e))] if 0<=t<=1 and 0<=s<=1 else []
 if nonzero(cross(sub(c,a),e)):return []
 i=next(i for i in range(3) if e[i]);u=(c[i]-a[i])/e[i];v=(d[i]-a[i])/e[i];lo=max(Q(0),min(u,v));hi=min(Q(1),max(u,v))
 return [] if lo>hi else [add(a,mul(t,e)) for t in (lo,hi)]
