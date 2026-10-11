"""Generic exact-rational bilinear certificate; unresolved is always failure."""
from fractions import Fraction as Q
from dataclasses import dataclass
from functools import cached_property
import itertools,time,math
import numpy as np
from incident_contact_authority import contact,contains,legal_edge,EDGE_CORNERS,PARAM_CORNERS
ZERO=Q(0);ONE=Q(1)
def add(a,b):return tuple(x+y for x,y in zip(a,b))
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def mul(k,a):return tuple(k*x for x in a)
def dot(a,b):return sum((x*y for x,y in zip(a,b)),ZERO)
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def nonzero(a):return any(x!=0 for x in a)
def clean(P):return {k:v for k,v in P.items() if v}
def padd(P,R,scale=ONE):
 out=P.copy()
 for k,v in R.items():out[k]=out.get(k,ZERO)+scale*v
 return clean(out)
def pmul(P,R):
 out={}
 for (i,j),a in P.items():
  for (k,l),b in R.items():out[i+k,j+l]=out.get((i+k,j+l),ZERO)+a*b
 return clean(out)
def bernstein(P):
 if not P:return {},(0,0)
 m=max(i for i,j in P);n=max(j for i,j in P);out={}
 for k in range(m+1):
  for l in range(n+1):out[k,l]=sum((a*Q(math.comb(k,i),math.comb(m,i))*Q(math.comb(l,j),math.comb(n,j)) for (i,j),a in P.items() if i<=k and j<=l),ZERO)
 return out,(m,n)
def divide_boundary(P,axis,value):
 # Exact polynomial division by the legal boundary factor x-value.
 result={};remainders=[];other=1-axis
 groups=sorted(set(k[other] for k in P))
 for g in groups:
  co={k[axis]:v for k,v in P.items() if k[other]==g};n=max(co);carry=co.get(n,ZERO)
  for i in range(n-1,-1,-1):
   key=(i,g) if axis==0 else (g,i)
   if carry:result[key]=carry
   carry=co.get(i,ZERO)+value*carry
  remainders.append(carry)
 return clean(result) if all(x==0 for x in remainders) else None
@dataclass
class Patch:
 points:tuple
 ids:tuple
 @classmethod
 def binary(cls,points,ids):return cls(tuple(tuple(Q(*float(v).as_integer_ratio()) for v in p) for p in points),tuple(ids))
 @cached_property
 def terms(self):
  C=self.points[0];A=sub(self.points[1],C);D=sub(self.points[2],C);E=sub(sub(self.points[3],self.points[2]),A);return C,A,D,E
 @cached_property
 def bounds(self):return tuple(min(p[j] for p in self.points) for j in range(3)),tuple(max(p[j] for p in self.points) for j in range(3))
 @cached_property
 def inverse(self):
  C,A,D,E=self.terms;det=dot(A,cross(D,E))
  if det==0:return None
  return tuple(mul(ONE/det,v) for v in (cross(D,E),cross(E,A),cross(A,D)))
 @cached_property
 def injective(self):
  if self.inverse is not None:return True
  C,A,D,E=self.terms
  normals=[cross(add(A,mul(v,E)),add(D,mul(u,E))) for u,v in PARAM_CORNERS]
  guide=tuple(sum(n[j] for n in normals) for j in range(3))
  # Strictly positive projected corner Jacobians give a convex, nonfolding
  # bilinear planar map. This is an exact admission proof, not a quality limit.
  return all(dot(n,guide)>0 for n in normals)
 def at(self,u,v):
  C,A,D,E=self.terms;return add(add(C,mul(u,A)),add(mul(v,D),mul(u*v,E)))
 def subcontrol(self,box):
  a,b,c,d=box;return tuple(self.at(u,v) for u,v in [(a,c),(b,c),(a,d),(b,d)])
def result(status,method,**evidence):return {'classification':status,'method':method,**evidence}
def accepted(c,method,**evidence):return result('CertifiedAllowedIncidentContact' if c['vertices'] or c['edges'] else 'CertifiedSeparated',method,**evidence)
def zero_locus_owned(B,degree,c,p):
 if not B or all(x==0 for x in B.values()):return False
 vals=list(B.values())
 if not (min(vals)>=0 or max(vals)<=0):return False
 m,n=degree
 # Open interior cannot be zero when at least one coefficient is strictly signed.
 for axis,value in [(0,0),(0,1),(1,0),(1,1)]:
  selected=[v for (i,j),v in B.items() if (i if axis==0 else j)==value*(m if axis==0 else n)]
  if all(v==0 for v in selected) and not legal_edge(c,p,axis,value):return False
 for U,V in PARAM_CORNERS:
  if B[U*m,V*n]==0 and not contains(c,p,U,V):return False
 return True
def supporting_planes(a,b,c):
 candidates=[(ONE,ZERO,ZERO),(ZERO,ONE,ZERO),(ZERO,ZERO,ONE)]
 candidates.append(sub(tuple(sum(p[j] for p in a.points)/4 for j in range(3)),tuple(sum(p[j] for p in b.points)/4 for j in range(3))))
 accumulated_a=[ZERO]*4;accumulated_b=[ZERO]*4
 for p in [a,b]:
  C,A,D,E=p.terms
  normals=[cross(A,D),cross(A,add(D,E)),cross(add(A,E),D),cross(add(A,E),add(D,E))]
  candidates+=normals
  for i,j in EDGE_CORNERS:
   edge=sub(p.points[j],p.points[i]);candidates.extend(cross(edge,n) for n in normals)
 for n in candidates:
  if not nonzero(n):continue
  va=[dot(n,p) for p in a.points];vb=[dot(n,p) for p in b.points]
  if max(va)<min(vb) or max(vb)<min(va):return accepted(c,'FastSupportingPlane')
  if max(va)==min(vb):offset=max(va);aa=[offset-v for v in va];bb=[v-offset for v in vb]
  elif max(vb)==min(va):offset=max(vb);aa=[v-offset for v in va];bb=[offset-v for v in vb]
  else:continue
  accumulated_a=[x+y for x,y in zip(accumulated_a,aa)];accumulated_b=[x+y for x,y in zip(accumulated_b,bb)]
  for p,values in [(a,accumulated_a),(b,accumulated_b)]:
   B={k:v for k,v in zip(PARAM_CORNERS,values)}
   other=b if p is a else a
   if other.injective and zero_locus_owned(B,(1,1),c,p):return accepted(c,'FastOwnedSupportingPlane')
 return None
def reedge(p,edge):
 i,j=next((i,j) for i,j in EDGE_CORNERS if frozenset((p.ids[i],p.ids[j]))==edge)
 # Put legal edge at u=0. Choose orientation by topology-independent endpoint geometry.
 if p.points[j]<p.points[i]:i,j=j,i
 def adjacent(k,exclude):return next(x for x,y in EDGE_CORNERS if y==k and x!=exclude) if not any(x==k and y!=exclude for x,y in EDGE_CORNERS) else next(y for x,y in EDGE_CORNERS if x==k and y!=exclude)
 oi=adjacent(i,j);oj=adjacent(j,i)
 return Patch(tuple(p.points[k] for k in [i,oi,j,oj]),tuple(p.ids[k] for k in [i,oi,j,oj]))
def radial(a,b,c):
 if len(c['edges'])!=1:return None
 if not (a.injective and b.injective):return None
 edge=next(iter(c['edges']));pa,pb=reedge(a,edge),reedge(b,edge);C,A,D,E=pa.terms;_,B,Db,F=pb.terms
 assert D==Db
 triples=[dot(D,cross(add(A,mul(i,E)),add(B,mul(j,F)))) for i in [ZERO,ONE] for j in [ZERO,ONE]]
 if min(triples)>0 or max(triples)<0:return accepted(c,'FastExactRadialTriple')
 d2=dot(D,D);vals=[d2*dot(add(A,mul(i,E)),add(B,mul(j,F)))-dot(D,add(A,mul(i,E)))*dot(D,add(B,mul(j,F))) for i in [ZERO,ONE] for j in [ZERO,ONE]]
 if max(vals)<0:return accepted(c,'FastExactOppositeRadial')
 return None
def implicit(a,b,c):
 inv=a.inverse
 if inv is None:return None
 C,A,D,E=a.terms;C2,B,F,G=b.terms;vectors=[sub(C2,C),B,F,G];keys=[(0,0),(1,0),(0,1),(1,1)]
 X,Y,Z=[clean({k:dot(row,w) for k,w in zip(keys,vectors)}) for row in inv];P=padd(Z,pmul(X,Y),-ONE)
 if not P:return None
 original=P.copy();factors=[]
 for axis,value in [(0,ZERO),(0,ONE),(1,ZERO),(1,ONE)]:
  if legal_edge(c,b,axis,value):
   divided=divide_boundary(P,axis,value)
   if divided is not None and divided:P=divided;factors.append([axis,int(value)])
 Bc,degree=bernstein(P);vals=list(Bc.values())
 if min(vals)>0 or max(vals)<0:
  return accepted(c,'ExactImplicitBernstein',legal_boundary_factors=factors,degree=list(degree),bernstein=[str(Bc[k]) for k in sorted(Bc)],implicit_basis='Actual [A,D,E] inverse; Q=Z-X*Y')
 Bo,deg=bernstein(original)
 if zero_locus_owned(Bo,deg,c,b):return accepted(c,'ExactImplicitBernsteinOwnedZeros',degree=list(deg),bernstein=[str(Bo[k]) for k in sorted(Bo)])
 return None
def witness(a,b,c,parameters):
 U,V,S,T=parameters;P=a.at(U,V);Qp=b.at(S,T)
 if P!=Qp or (contains(c,a,U,V) and contains(c,b,S,T)):return None
 _,A,D,E=a.terms;_,B,F,G=b.terms;n1=cross(add(A,mul(V,E)),add(D,mul(U,E)));n2=cross(add(B,mul(T,G)),add(F,mul(S,G)))
 typ='Tangential' if not nonzero(cross(n1,n2)) else 'Transverse'
 boundary=any(x in [ZERO,ONE] for x in parameters);typ+=('BoundaryContact' if boundary else 'InteriorInterior')
 return result('CertifiedForbiddenIntersection','ExactRootWitness',parameters=[str(x) for x in parameters],parameter_enclosure=[[str(x),str(x)] for x in parameters],point=[str(x) for x in P],point_enclosure=[[str(x),str(x)] for x in P],residual_exact='0',contact_type=typ)
def dyadic_witness(a,b,c):
 grid=[ZERO,Q(1,2),ONE];points={}
 for U,V in itertools.product(grid,repeat=2):points.setdefault(a.at(U,V),[]).append((U,V))
 for S,T in itertools.product(grid,repeat=2):
  for U,V in points.get(b.at(S,T),[]):
   r=witness(a,b,c,(U,V,S,T))
   if r:return r
 return None
def fallback(a,b,c,max_boxes=50000,max_depth=28,seconds=45):
 """Exact tensor-Bernstein convex hulls on the full four-parameter domain.

 Positive separation bounds exclude boxes. Exact rational zeros reject; finite
 sampling never certifies absence. Reaching any declared bound returns unresolved.
 """
 started=time.perf_counter();stack=[((ZERO,ONE,ZERO,ONE,ZERO,ONE,ZERO,ONE),0)];count=0;depth_seen=0
 while stack:
  box,depth=stack.pop();count+=1;depth_seen=max(depth_seen,depth)
  pa=a.subcontrol(box[:4]);pb=b.subcontrol(box[4:]);la=[min(p[j] for p in pa) for j in range(3)];ha=[max(p[j] for p in pa) for j in range(3)];lb=[min(p[j] for p in pb) for j in range(3)];hb=[max(p[j] for p in pb) for j in range(3)]
  if any(ha[j]<lb[j] or hb[j]<la[j] for j in range(3)):continue
  mids=tuple((box[2*j]+box[2*j+1])/2 for j in range(4));w=witness(a,b,c,mids)
  if w:return {**w,'fallback_boxes':count,'fallback_depth':depth_seen}
  if count>=max_boxes or depth>=max_depth or time.perf_counter()-started>=seconds:return result('Unresolved','ExactSubdivisionLimit',box=[str(x) for x in box],fallback_boxes=count,fallback_depth=depth_seen,remaining_boxes=len(stack),reason='Legal-boundary accumulation or non-excluded contact; no pass inferred')
  # Physical derivative length decides subdivision only; all proof bounds remain exact.
  scales=[]
  for p in [a,b]:
   C,A,D,E=p.terms;scales.extend([max(float(dot(A,A)),float(dot(add(A,E),add(A,E)))),max(float(dot(D,D)),float(dot(add(D,E),add(D,E))))])
  dim=max(range(4),key=lambda j:float(box[2*j+1]-box[2*j])*math.sqrt(scales[j]));mid=mids[dim];left=list(box);right=list(box);left[2*dim+1]=mid;right[2*dim]=mid;stack.extend([(tuple(right),depth+1),(tuple(left),depth+1)])
 return accepted(c,'ExactSubdivision',fallback_boxes=count,fallback_depth=depth_seen)
def certify(a,b):
 c=contact(a,b);lo,hi=a.bounds;lb,hb=b.bounds
 if any(hi[j]<lb[j] or hb[j]<lo[j] for j in range(3)):return accepted(c,'FastAABB')
 # Canonical geometry orientation makes pair order independent without ID routing.
 if (b.points,tuple(map(str,b.ids)))<(a.points,tuple(map(str,a.ids))):a,b=b,a
 for call in [supporting_planes,radial,implicit]:
  r=call(a,b,c)
  if r:return r
 r=implicit(b,a,c)
 if r:return r
 r=dyadic_witness(a,b,c)
 if r:return r
 return fallback(a,b,c)
