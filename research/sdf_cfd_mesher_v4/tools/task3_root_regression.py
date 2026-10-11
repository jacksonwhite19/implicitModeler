"""Exact root-form fixtures and unchanged-certificate regression evidence."""
from pathlib import Path
import sys,json,time,inspect,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
import sympy as sp
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel,repaired_kernel
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.scaled_root_adapter import normalize
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.repaired_runtime import CertificateSession
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import canonical_bytes
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
OUT=ROOT/'results/task3_certificate_repair'

def write(name,v):(OUT/name).write_text(json.dumps(plain(v),indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf-8')
def saved():return json.loads((ROOT/'results/task3/CERTIFICATE_FAILURE_REPRODUCER.json').read_text())
def blocker():return sp.sympify(saved()['result']['message'].split(':',1)[1],locals={'CRootOf':sp.CRootOf})
def pair_from_saved():return [kernel.Patch.binary(s['points'],tuple(tuple(i) for i in s['labels'])) for s in saved()['surfaces']]

def qualify():
    old=kernel.module('exact_algebraic');new=repaired_kernel.module('exact_algebraic');x=sp.Symbol('x')
    r=sp.CRootOf(x**3-x-1,0);real3=[sp.CRootOf(x**3-3*x+1,i) for i in range(3)]
    values=[r,2*r,sp.Rational(3,5)*r,-2*r,-sp.Rational(3,5)*r]+[-2*v for v in real3]+[blocker()]
    rows=[]
    for v in values:
        q=normalize(v);c,a=v.as_coeff_Mul() if not isinstance(v,sp.CRootOf) else (sp.S.One,v)
        # Independent substitution identity of primitive polynomials.
        transformed=sp.Poly(a.poly.as_expr().subs(a.poly.gen,x/c),x,domain=sp.QQ).monic()
        assert transformed==sp.Poly(q.poly.as_expr().subs(q.poly.gen,x),x,domain=sp.QQ).monic()
        # Exact scaled source interval overlaps the selected q interval after
        # refinement until it excludes every other root of q.
        intervals=sp.Poly(q.poly).intervals();index=q.index
        for _ in range(10000):
            lo,hi=old.bounds(a);left,right=sorted((c*lo,c*hi))
            hits=[j for j,((l,h),_) in enumerate(intervals) if left<=h and l<=right]
            if hits==[index]:break
            old.refine(a)
        else:raise AssertionError('ExactRootIndexVerificationFailed')
        assert new.compare(v,q)==0 and new.compare(q,v)==0
        assert new.compare(v,v+0)==0
        assert new.compare(v,0)==(1 if c>0 else -1)*old.compare(a,0)
        assert new.compare(-v,0)==-new.compare(v,0)
        assert new.compare(v,sp.Rational(new.bounds(v)[0])-1)==1
        assert new.compare(v,sp.Rational(new.bounds(v)[1])+1)==-1
        p=sp.Poly(q.poly.as_expr().subs(q.poly.gen,new.T),new.T,domain=sp.QQ)
        assert new.polynomial_zero_at(p,sp.S.Zero,v)
        K,k=new.exact_field(v);assert not sum(K.convert(co)*k**(p.degree()-j) for j,co in enumerate(p.all_coeffs()))
        cell=new.CellArithmetic(v,sp.S.Zero)
        pp=sp.Poly(p.as_expr().subs(new.T,new.S),new.S,new.T,domain=sp.QQ)
        assert cell.sign(pp)==0
        assert len(new.ordered([q,v]))==1
        rows.append(dict(expression=str(v),expression_type=type(v).__name__,scale=str(c),source_polynomial=str(a.poly.as_expr()),source_root_index=a.index,
                         normalized=str(q),root_record=new.root_record(v),sign=new.compare(v,0),exact_polynomial_identity=True,exact_index_isolation=True,field_zero=True))
    # The bare-root scientific record is byte identical.
    assert canonical_bytes(old.root_record(r))==canonical_bytes(new.root_record(r))
    fixtures=[]
    def patch(points,start=0,ids=None,dimension=2):return kernel.Patch.binary(points,tuple(range(start,start+4)) if ids is None else ids,dimension)
    a=patch([(0,0,0),(1,0,0),(0,1,0),(1,1,0)])
    pairs=[('legal_edge',a,patch([(1,0,0),(2,0,0),(1,1,0),(2,1,0)],ids=(1,4,3,5))),
           ('legal_vertex',a,patch([(1,1,0),(2,1,0),(1,2,0),(2,2,0)],ids=(3,4,5,6))),
           ('planar_overlap',a,patch([(.1,.1,0),(1.1,.1,0),(.1,1.1,0),(1.1,1.1,0)],4)),
           ('separated',a,patch([(0,0,1),(1,0,1),(0,1,1),(1,1,1)],4))]
    b=patch([(0,0,0),(1,0,0),(0,1,0),(1,1,1)])
    pairs.extend([('coincident_rank3',b,patch(b.points,4)),('noncoincident_rank3',b,patch([(0,0,-1/32),(1,0,-1/32),(0,1,3/32),(1,1,35/32)],4))])
    point=patch([(0,0,0)]*4,dimension=0)
    line=patch([(0,0,0),(1,0,0),(0,0,0),(1,0,0)],4,dimension=1)
    pairs.extend([('rank0_owned_point',point,point),('rank0_rank1',point,line),('rank1_rank2',line,a),('rank1_rank3',line,b),('rank2_rank3',a,patch(b.points,8))])
    for name,a,b in pairs:
        oldr=kernel.certificate.certify(a,b);newr=CertificateSession().pair(a,b)
        assert canonical_bytes(oldr)==canonical_bytes(newr),(name,oldr,newr)
        fixtures.append(dict(name=name,result=newr,byte_identical=True))
    try:old.bounds(blocker())
    except ArithmeticError as err:old_error=str(err)
    else:raise AssertionError('FrozenBlockerDidNotReproduce')
    pair=CertificateSession().pair(*pair_from_saved())
    return dict(root_fixtures=rows,unchanged_pair_fixtures=fixtures,old_exception=old_error,repaired_pair=pair,
                source_assumptions='QQ irreducible polynomial; real indexed root; nonzero rational scale; negative scale reverses only real-root order',
                caller='noncoincident_rank3_certificate.project -> exact_algebraic.unit_roots -> compare(root,0) -> bounds',
                expected_operation='Exact root inclusion and ordering on the closed parameter interval [0,1]',
                sympy_version=sp.__version__,raw_constructor_source=inspect.getsource(sp.CRootOf._new),new_kernel_identity=repaired_kernel.VERSION)

if __name__=='__main__':
    t=time.perf_counter();r=qualify();write('ROOT_REGRESSION.json',r)
    write('ROOT_REGRESSION_TIMING.json',dict(seconds=time.perf_counter()-t))
    print(json.dumps(dict(passed=True,root_fixtures=len(r['root_fixtures']),unchanged_pairs=len(r['unchanged_pair_fixtures']),repaired_pair=r['repaired_pair']['classification'],hash=scientific_hash(r),seconds=time.perf_counter()-t)))
