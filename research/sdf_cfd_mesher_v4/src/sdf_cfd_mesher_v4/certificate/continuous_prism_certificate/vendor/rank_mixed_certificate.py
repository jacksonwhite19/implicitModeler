"""Exact univariate sign-cell certificates for a quadric against a plane/line."""
from fractions import Fraction as Q
from functools import cmp_to_key
from rank_geometry import *
import sympy as sp
T=sp.Symbol('t',real=True)
def sx(x):return sp.Rational(x.numerator,x.denominator) if isinstance(x,Q) else sp.sympify(x)
def cancel(x):return sp.cancel(x)
def roots(poly):
 P=sp.Poly(poly,T)
 if P.degree()<=0:return []
 if P.degree()>2:raise ArithmeticError('UnexpectedDegreeAboveTwo')
 return list(P.real_roots())
def sorted_exact(values):
 vals=sorted(values,key=cmp_to_key(lambda a,b:sign(a-b)));out=[]
 for x in vals:
  if not out or sign(x-out[-1])!=0:out.append(x)
 return out
def at(expr,t):return cancel(expr.subs(T,t)) if hasattr(expr,'subs') else expr
def numerator(expr):return sp.Poly(sp.fraction(cancel(expr))[0],T)
def authority_polys(point,c):
 out=[]
 for q in c['points']:out.extend(point[i]-sx(q[i]) for i in range(3))
 for a,b in c['segments']:
  w=sub(point,tuple(sx(x) for x in a));e=tuple(sx(x) for x in sub(b,a))
  out.extend(cross(w,e));out.extend((dot(w,e),dot(w,e)-dot(e,e)))
 return out
def rational_between(a,b):
 power=0
 while True:
  scale=2**power;r=(sp.floor(a*scale)+1)/scale
  if sign(r-a)>0 and sign(b-r)>0:return r
  power+=8
def finite_line_quadric(line,full):
 a,b=line;d=sub(b,a);C,*_=full.terms
 XYZ=[sum(sx(row[j])*(sx(a[j]-C[j])+T*sx(d[j])) for j in range(3)) for row in full.inverse]
 X,Y,Z=XYZ;P=sp.Poly(sp.expand(Z-X*Y),T);out=[]
 if P.is_zero:
  hs=[(tuple(row),C) for row in []]
  lo,hi=Q(0),Q(1)
  for expr in (X,1-X,Y,1-Y):
   pp=sp.Poly(expr,T);aa=pp.nth(0);bb=pp.nth(1)
   if sign(bb)==0:
    if sign(aa)<0:return [],{'case':'RulingOutsideParameterBox'}
   elif sign(bb)>0:
    rr=-aa/bb
    if sign(rr-lo)>0:lo=rr
   else:
    rr=-aa/bb
    if sign(rr-hi)<0:hi=rr
  if sign(lo-hi)<=0:out=[add(a,mul(t,d)) for t in (lo,hi)]
  return out,{'case':'ExactRulingInterval','interval':(lo,hi),'rank3_parameter_polynomials':XYZ}
 for t in roots(P):
  if sign(t)<0 or sign(1-t)<0:continue
  if all(sign(at(v,t))>=0 and sign(1-at(v,t))>=0 for v in (X,Y)):out.append(add(a,mul(t,d)))
 return out,{'case':'ExactLineQuadricRootIsolation','polynomial':P.as_expr(),'real_roots':roots(P),'rank3_parameter_polynomials':XYZ}
def mixed_planar(full,plane,c):
 n=plane.planar['normal'];h=plane.planar['offset'];C,A,D,E=full.terms
 f0=dot(n,C)-h;f1=dot(n,A);f2=dot(n,D);f3=dot(n,E)
 numerator_v=sx(f0)+T*sx(f1);den=sx(f2)+T*sx(f3)
 branch_records=[];vertical_points=[]
 if f3!=0:
  u=-f2/f3
  if 0<=u<=1 and f0+f1*u==0:vertical_points=segment_planar(full.at(u,Q(0)),full.at(u,Q(1)),plane);branch_records.append({'case':'DenominatorZeroVerticalBranch','u':u,'image':vertical_points})
 elif f2==0:
  if f1==0:
   if f0==0:raise ArithmeticError('Rank3ImageCannotLieInOnePlane')
  else:
   u=-f0/f1
   if 0<=u<=1:vertical_points=segment_planar(full.at(u,Q(0)),full.at(u,Q(1)),plane);branch_records.append({'case':'EntirePlaneEquationVertical','u':u,'image':vertical_points})
 if f2==0 and f3==0:
  return geometric_result(vertical_points,c,'Rank2ExactMixedPlane',{'branches':branch_records,'plane_coefficients':[f0,f1,f2,f3]})
 vr=cancel(-numerator_v/den)
 point=tuple(cancel(sx(C[j])+T*sx(A[j])+vr*sx(D[j])+T*vr*sx(E[j])) for j in range(3))
 inequalities=[T,1-T,vr,1-vr]
 inequalities.extend(cancel(sum(sx(normal[j])*(point[j]-sx(anchor[j])) for j in range(3))) for normal,anchor in plane.planar['halfspaces'])
 polys=[numerator(x).as_expr() for x in inequalities]+[sp.expand(den)]
 polys.extend(numerator(x).as_expr() for x in authority_polys(point,c))
 allroots=[sp.Integer(0),sp.Integer(1)]
 for p in polys:
  allroots.extend(r for r in roots(p) if sign(r)>=0 and sign(1-r)>=0)
 breaks=sorted_exact(allroots);samples=[]
 for i,r in enumerate(breaks):
  samples.append(('root',r))
  if i+1<len(breaks):samples.append(('open_sign_cell',rational_between(r,breaks[i+1])))
 decisions=[];intersection_points=vertical_points.copy()
 for kind,u in samples:
  if sign(at(den,u))==0:
   decisions.append({'cell':kind,'parameter':str(u),'decision':'VerticalBranchSeparatelyHandled'});continue
  ss=[sign(at(g,u)) for g in inequalities]
  if min(ss)<0:
   decisions.append({'cell':kind,'parameter':str(u),'decision':'Excluded','inequality_signs':ss});continue
  p=tuple(at(x,u) for x in point)
  if not membership(p,full) or not membership(p,plane):raise ArithmeticError('ExactMixedMembershipContradiction')
  islegal=owned(p,c);decisions.append({'cell':kind,'parameter':str(u),'decision':'Allowed' if islegal else 'Forbidden','inequality_signs':ss})
  intersection_points.append(p)
  if not islegal:
   proof={'case':'ExactBilinearPlaneRationalBranch','plane_coefficients':[f0,f1,f2,f3],'parameter_full':[u,at(vr,u)],'critical_polynomials':polys,'critical_roots':breaks,'cell_decisions':decisions,'vertical_branches':branch_records}
   return geometric_result([p],c,'Rank2ExactMixedPlane',proof,False)
 proof={'case':'CompleteExactUnivariateSignPartition','plane_coefficients':[f0,f1,f2,f3],'v_of_u':vr,'critical_polynomials':polys,'critical_roots':breaks,'cell_decisions':decisions,'vertical_branches':branch_records,'completeness':'All rational inequality and ownership predicates have constant sign on every open cell; roots and denominator-zero branches checked separately'}
 return geometric_result(intersection_points,c,'Rank2ExactMixedPlane',proof,False)
