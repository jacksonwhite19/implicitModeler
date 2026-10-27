"""Exact algebraic roots, rational intervals, signs and deterministic records."""
import sympy as sp
from functools import lru_cache,cmp_to_key
S,T=sp.symbols('s t',real=True)
QQ=sp.QQ

def rational(x):
 return sp.Rational(x.numerator,x.denominator) if hasattr(x,'numerator') else sp.Rational(x)

def primitive(poly,gens):
 p=sp.Poly(poly,*gens,domain=QQ)
 if p.is_zero:return p
 _,p=p.clear_denoms();_,p=p.primitive()
 if p.LC()<0:p=-p
 return p

def polynomial_record(p):
 return [[list(m),str(c)] for m,c in p.terms()]

_intervals={}
def bounds(x):
 x=sp.sympify(x)
 if x.is_Rational:return x,x
 if not isinstance(x,sp.CRootOf):raise ArithmeticError('ExpectedExactRealRoot:'+str(x))
 key=str(x)
 if key not in _intervals:_intervals[key]=x._get_interval()
 q=_intervals[key]
 return rational(q.a),rational(q.b)

def refine(x):
 if x.is_Rational:return
 key=str(x);bounds(x);_intervals[key]=_intervals[key].refine()

def compare(a,b):
 a=sp.sympify(a);b=sp.sympify(b)
 if a==b:return 0
 if a.is_Rational and b.is_Rational:return 1 if a>b else -1
 # real_roots produces roots of irreducible QQ factors, so distinct
 # canonical CRootOf objects denote distinct algebraic numbers.
 while True:
  la,ha=bounds(a);lb,hb=bounds(b)
  if ha<lb:return -1
  if hb<la:return 1
  if not a.is_Rational:refine(a)
  if not b.is_Rational:refine(b)

def ordered(values):
 out=[]
 for x in sorted(set(values),key=cmp_to_key(compare)):
  if not out or compare(out[-1],x):out.append(x)
 return out

def between(a,b):
 a=sp.sympify(a);b=sp.sympify(b)
 assert compare(a,b)<0
 while True:
  _,hi=bounds(a);lo,_=bounds(b)
  if hi<lo:return (hi+lo)/2
  if not a.is_Rational:refine(a)
  if not b.is_Rational:refine(b)

def unit_roots(p):
 if p.is_zero or p.degree()<=0:return []
 q=p.sqf_part()
 # Only QQ root isolation is permitted. Algebraic-domain real_roots uses
 # numerical conjugate filtering internally and is deliberately not called.
 if q.domain==QQ:
  candidates=q.real_roots(radicals=False)
 else:
  candidates=q.lift().sqf_part().real_roots(radicals=False)
  alpha=q.domain.ext.root
  candidates=[r for r in candidates if polynomial_zero_at(q,alpha,r)]
 return ordered([r for r in candidates if compare(r,0)>=0 and compare(r,1)<=0])

@lru_cache(maxsize=20000)
def root_record(x):
 x=sp.sympify(x)
 if x.is_Rational:return {'rational':str(x),'interval':[str(x),str(x)]}
 p=sp.Poly(x.poly.as_expr(),x.poly.gen,domain=QQ)
 intervals=p.intervals(eps=sp.Rational(1,2**80))
 a,b=intervals[x.index][0]
 return {'polynomial_coefficients':[str(c) for c in p.all_coeffs()],'root_index':x.index,'interval':[str(a),str(b)],'isolation':'Exact QQ Sturm/continued-fraction real-root isolation'}

def iadd(a,b):return a[0]+b[0],a[1]+b[1]
def imul(a,b):
 v=[a[i]*b[j] for i in (0,1) for j in (0,1)]
 return min(v),max(v)
def ipow(a,k):
 z=(sp.Integer(1),sp.Integer(1))
 for _ in range(k):z=imul(z,a)
 return z
def polynomial_interval(p,x,y):
 bx,by=bounds(x),bounds(y);z=(sp.Integer(0),sp.Integer(0))
 for (i,j),c in p.terms():z=iadd(z,imul((c,c),imul(ipow(bx,i),ipow(by,j))))
 return z

class CellArithmetic:
 def __init__(self,x,y):
  self.x=sp.sympify(x);self.y=sp.sympify(y);self.K=None
 def is_zero(self,p):
  return polynomial_zero_at(specialize(p,self.x),self.x,self.y)
 def sign(self,p):
  if p.is_zero:return 0
  # A strict rational interval already proves a nonzero sign.
  lo,hi=polynomial_interval(p,self.x,self.y)
  if lo>0:return 1
  if hi<0:return -1
  if self.is_zero(p):return 0
  while True:
   refine(self.x);refine(self.y)
   lo,hi=polynomial_interval(p,self.x,self.y)
   if lo>0:return 1
   if hi<0:return -1
 def expression(self,p):return sp.expand(p.as_expr().subs({S:self.x,T:self.y}))

@lru_cache(maxsize=10000)
def exact_field(x):
 x=sp.sympify(x)
 if x.is_Rational:return QQ,QQ.convert(x)
 # Supply the exact minimal polynomial and its selected real root explicitly.
 # K.unit is the generator, so no PSLQ or cross-field embedding is requested.
 K=QQ.algebraic_field((x.poly.as_expr(),x))
 return K,K.unit

def field_sign(value,alpha):
 if not value:return 0
 if alpha.is_Rational:return 1 if value>0 else -1
 coefficients=[rational(c) for c in value.to_list()]
 while True:
  z=(sp.Integer(0),sp.Integer(0));b=bounds(alpha)
  for c in coefficients:z=iadd(imul(z,b),(c,c))
  if z[0]>0:return 1
  if z[1]<0:return -1
  refine(alpha)

def eval_field(poly,q):
 K=poly.domain;v=K.zero;x=K.convert(q)
 for c in poly.rep.to_list():v=v*x+c
 return v

def sturm_variations(chain,q,alpha):
 signs=[field_sign(eval_field(p,q),alpha) for p in chain]
 signs=[z for z in signs if z]
 return sum(a!=b for a,b in zip(signs,signs[1:]))

def polynomial_zero_at(p,alpha,beta):
 alpha=sp.sympify(alpha);beta=sp.sympify(beta)
 if p.is_zero:return True
 if beta.is_Rational:return not eval_field(p,beta)
 # Every root of this GCD is a root of beta's irreducible QQ polynomial.
 # Beta's strict rational isolating interval contains only the selected root.
 m=sp.Poly(beta.poly.as_expr().subs(beta.poly.gen,T),T,domain=p.domain)
 g=p.gcd(m)
 if g.degree()<=0:return False
 lo,hi=bounds(beta)
 chain=g.sturm()
 count=sturm_variations(chain,lo,alpha)-sturm_variations(chain,hi,alpha)
 if count not in (0,1):raise ArithmeticError('NonisolatingBetaInterval')
 return count==1

def specialize(p,x):
 x=sp.sympify(x);K,kx=exact_field(x)
 coeff={}
 for (i,j),c in p.terms():coeff[j]=coeff.get(j,K.zero)+K.convert(c)*kx**i
 return sp.Poly.from_dict({(j,):v for j,v in coeff.items()},T,domain=K)
