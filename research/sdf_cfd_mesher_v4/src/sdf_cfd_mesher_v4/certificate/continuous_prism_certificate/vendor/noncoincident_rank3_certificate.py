"""Exact 2D sign-invariant decomposition for bounded noncoincident quadrics."""
from fractions import Fraction as Q
import sympy as sp
from exact_algebraic import *
from rank_geometry import owned_set,encode
from rank3_certificate import dot,sub,cross
from coincident_rank3_certificate import chart
from itertools import combinations

def poly_from_dict(d):return sp.Poly.from_dict(d,(S,T),domain=QQ)
def pexpr(expr):return sp.Poly(sp.expand(expr),S,T,domain=QQ)

def factor_basis(polys):
 factors={}
 for p in polys:
  if p.is_zero or p.total_degree()==0:continue
  for q,_ in sp.factor_list(p)[1]:
   q=primitive(q.as_expr(),(S,T));factors[str(q.as_expr())]=q
 return [factors[k] for k in sorted(factors)]

def project(basis):
 candidates=[];traces=[];vertical=[];positive=[]
 def put(expr,reason):
  p=sp.Poly(expr,S,domain=QQ)
  if not p.is_zero and p.degree()>0:candidates.append(p);traces.append({'reason':reason,'polynomial':polynomial_record(p)})
 for i,p in enumerate(basis):
  pt=sp.Poly(p.as_expr(),T)
  if pt.degree()==0:
   put(pt.as_expr(),['content_or_vertical_factor',i]);vertical.append(i);continue
  positive.append(i)
  for c in pt.all_coeffs():put(c,['coefficient',i])
  if pt.degree()>1:
   put(sp.discriminant(pt.as_expr(),T),['discriminant',i])
   put(sp.resultant(pt.as_expr(),sp.diff(pt.as_expr(),T),T),['derivative_resultant',i])
 for i,j in combinations(positive,2):
  r=sp.resultant(basis[i].as_expr(),basis[j].as_expr(),T)
  if r==0:raise ArithmeticError('NonCoprimeProjectionBasis')
  put(r,['pair_resultant',i,j])
 factors={}
 for p in candidates:
  for q,_ in sp.factor_list(p)[1]:
   q=primitive(q.as_expr(),(S,));factors[str(q.as_expr())]=q
 out=[factors[k] for k in sorted(factors)]
 roots=ordered([sp.Integer(0),sp.Integer(1)]+[r for p in out for r in unit_roots(p)])
 return out,roots,traces

def cells(points):
 for i,p in enumerate(points):
  yield 'section',p,[p,p]
  if i+1<len(points):yield 'sector',between(p,points[i+1]),[p,points[i+1]]

def cert(a,b):
 (xd,yd,zd),fd=chart(a,b)
 if not fd:raise ArithmeticError('CoincidentPairMisrouted')
 X,Y,Z,F=[poly_from_dict(p) for p in (xd,yd,zd,fd)]
 bounds_polys=[pexpr(S),pexpr(1-S),pexpr(T),pexpr(1-T),X,pexpr(1-X.as_expr()),Y,pexpr(1-Y.as_expr())]
 C,B,D,E=b.terms
 physical=[pexpr(rational(C[j])+S*rational(B[j])+T*rational(D[j])+S*T*rational(E[j])) for j in range(3)]
 c=owned_set(a,b);point_pred=[];edge_pred=[];topology=[]
 for q in c['points']:
  ps=[pexpr(physical[j].as_expr()-rational(q[j])) for j in range(3)]
  point_pred.append(ps);topology.extend(ps)
 for q,r in c['segments']:
  direction=tuple(rational(v) for v in sub(r,q));w=tuple(physical[j].as_expr()-rational(q[j]) for j in range(3))
  eqs=[pexpr(x) for x in cross(w,direction)]
  lower=pexpr(dot(w,direction));upper=pexpr(dot(direction,direction)-lower.as_expr())
  edge_pred.append((eqs,lower,upper));topology.extend(eqs+[lower,upper])
 allpolys=[F]+bounds_polys+topology;basis=factor_basis(allpolys)
 projection,xroots,projection_trace=project(basis)
 records=[];allowed_found=False;nullifications=[]
 proof={'method':'FullCoefficientDiscriminantResultantProjection','F':polynomial_record(F),'B_chart':[polynomial_record(p) for p in (X,Y,Z)],
  'inequalities':[polynomial_record(p) for p in bounds_polys],'topology_predicates':[polynomial_record(p) for p in topology],
  'factor_basis':[polynomial_record(p) for p in basis],'projection_polynomials':[polynomial_record(p) for p in projection],
  'projection_trace':projection_trace,'critical_s_roots':[root_record(r) for r in xroots],
  'completeness':'Primitive irreducible factor basis; all coefficients, discriminants and pair resultants project degree changes, root mergers and crossings. Full sections and sectors over every critical fiber and open strip are sign-invariant. Nullified fibers are lifted using every remaining predicate.','sample_authority':'Exact CAD cell representatives, not a finite geometric witness grid'}
 def legal(ar):
  if any(all(ar.sign(p)==0 for p in ps) for ps in point_pred):return True
  return any(all(ar.sign(p)==0 for p in eqs) and ar.sign(lo)>=0 and ar.sign(hi)>=0 for eqs,lo,hi in edge_pred)
 for xkind,x,xb in cells(xroots):
  yr=[sp.Integer(0),sp.Integer(1)];zero=[]
  for i,p in enumerate(basis):
   pt=specialize(p,x)
   if pt.is_zero:zero.append(i)
   else:yr.extend(unit_roots(pt))
  yr=ordered(yr)
  if zero:nullifications.append({'s':root_record(x),'factor_indices':zero})
  fiber={'s_kind':xkind,'s_sample':root_record(x),'s_bounds':[root_record(z) for z in xb],'t_roots':[root_record(z) for z in yr],'cells':[],'nullified_factors':zero}
  records.append(fiber)
  for ykind,y,yb in cells(yr):
   ar=CellArithmetic(x,y);fs=ar.sign(F)
   rec={'t_kind':ykind,'t_sample':root_record(y),'t_bounds':[root_record(z) for z in yb],'F_sign':fs}
   fiber['cells'].append(rec)
   if fs:
    rec['decision']='OffIntersection';continue
   signs=[ar.sign(p) for p in bounds_polys];rec['bound_signs']=signs
   if min(signs)<0:rec['decision']='OutsideBoundedImage';continue
   if legal(ar):allowed_found=True;rec['decision']='AllowedContact';continue
   rec['decision']='ForbiddenContact'
   point=[ar.expression(p) for p in physical];u,v=ar.expression(X),ar.expression(Y)
   # Analytic normal-cross polynomials determine transverse/tangent contact.
   _,Aa,Da,Ea=a.terms
   na=cross(tuple(rational(Aa[j])+Y.as_expr()*rational(Ea[j]) for j in range(3)),tuple(rational(Da[j])+X.as_expr()*rational(Ea[j]) for j in range(3)))
   nb=cross(tuple(rational(B[j])+T*rational(E[j]) for j in range(3)),tuple(rational(D[j])+S*rational(E[j]) for j in range(3)))
   transverse=any(ar.sign(pexpr(z))!=0 for z in cross(na,nb))
   abound=any(signs[k]==0 for k in (4,5,6,7));bbound=any(signs[k]==0 for k in (0,1,2,3))
   contact_type='BoundaryBoundary' if abound and bbound else ('BoundaryInterior' if abound or bbound else ('TransverseInteriorInterior' if transverse else 'TangentialInteriorInterior'))
   proof.update(fibers=records,nullifications=nullifications,complete_traversal=False,existence_proven=True)
   return {'classification':'CertifiedForbiddenIntersection','method':'NoncoincidentRank3ExactCAD','ranks':[3,3],'proof':proof,'parameters':[str(u),str(v),str(x),str(y)],'B_parameter_roots':[root_record(x),root_record(y)],'point':[str(q) for q in point],'residual_exact':'0','witness_authority':'F=0 and all parameter-image inequalities evaluated exactly in the selected real algebraic field','contact_type':contact_type,'authorized_contact':encode(c),'fallback_depth':0,'generic_subdivision_used':False}
 proof.update(fibers=records,nullifications=nullifications,complete_traversal=True,existence_proven=allowed_found)
 return {'classification':'CertifiedAllowedIncidentContact' if allowed_found else 'CertifiedSeparated','method':'NoncoincidentRank3ExactCAD','ranks':[3,3],'proof':proof,'legal_contact_only':allowed_found,'authorized_contact':encode(c),'fallback_depth':0,'generic_subdivision_used':False}
