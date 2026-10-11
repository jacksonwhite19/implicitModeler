"""Exact representation adapter for a rational multiple of a real CRootOf.

For a nonzero rational c and an irreducible QQ polynomial p, c*alpha is
the corresponding real root of p(t/c). Scaling preserves irreducibility.
Negative scaling reverses the real-root order. No numerical embedding,
root approximation, predicate, or intersection theorem is used here.
The raw SymPy constructor prevents its public constructor from factoring
the rational scale back out. Its exact isolation and field operations are
the same operations used by the preserved certificate.
"""
from functools import lru_cache,cmp_to_key
import sympy as sp
from sympy.polys.rootisolation import dup_isolate_real_roots_sqf

@lru_cache(maxsize=20000)
def initial_intervals(poly):
    # The same exact routine used by CRootOf._get_reals_sqf. Keep its initial
    # immutable intervals separate from SymPy's process-wide refinement cache.
    return tuple(dup_isolate_real_roots_sqf(poly.rep.to_list(),poly.rep.dom,blackbox=True))

@lru_cache(maxsize=20000)
def normalize(value):
    value=sp.sympify(value)
    if value.is_Rational:
        return value
    if isinstance(value,sp.CRootOf):
        scale,root=sp.S.One,value
    else:
        scale,root=value.as_coeff_Mul()
    if not scale.is_Rational or scale==0 or not isinstance(root,sp.CRootOf) or root.is_real is not True:
        raise ArithmeticError('ExpectedRationalMultipleOfExactRealRoot:'+str(value))
    p=sp.Poly(root.poly.as_expr(),root.poly.gen,domain=sp.QQ)
    # Public CRootOf supplies an irreducible factor. Fail closed for raw
    # externally constructed roots that do not obey that existing contract.
    if not p.is_irreducible:
        raise ArithmeticError('ExpectedIrreducibleExactRootPolynomial')
    t=p.gen
    q=sp.Poly(p.as_expr().subs(t,t/scale),t,domain=sp.QQ)
    _,q=q.clear_denoms();_,q=q.primitive()
    if q.LC()<0:q=-q
    index=root.index if scale>0 else int(p.count_roots(-sp.oo,sp.oo))-1-root.index
    # _new only stores the polynomial and index; _get_interval subsequently
    # performs exact QQ isolation. It does not choose a root numerically.
    return sp.CRootOf._new(q,index)

def install(module):
    """Adapt entry values in a private copy of the frozen arithmetic module."""
    positions={'bounds':(0,),'refine':(0,),'compare':(0,1),'between':(0,1),
               'root_record':(0,),'exact_field':(0,),'specialize':(1,),
               'polynomial_zero_at':(1,2)}
    for name,indices in positions.items():
        original=getattr(module,name)
        def wrapped(*args,_fn=original,_indices=indices,**kwargs):
            values=list(args)
            for i in _indices:values[i]=normalize(values[i])
            return _fn(*values,**kwargs)
        wrapped.__name__=name
        setattr(module,name,wrapped)
    def ordered(values):
        # Comparisons refine private exact intervals. A set's randomized
        # traversal must not select the comparison/refinement history.
        # The final order is still the unchanged exact algebraic comparison.
        initial=sorted({normalize(v) for v in values},key=str)
        out=[]
        for value in sorted(initial,key=cmp_to_key(module.compare)):
            if not out or module.compare(out[-1],value):out.append(value)
        return out
    module.ordered=ordered
    def bounds(value):
        value=normalize(value)
        if value.is_Rational:return value,value
        key=str(value)
        if key not in module._intervals:
            module._intervals[key]=initial_intervals(value.poly)[value.index]
        q=module._intervals[key]
        return module.rational(q.a),module.rational(q.b)
    module.bounds=bounds
    original_cell=module.CellArithmetic
    class NormalizedCellArithmetic(original_cell):
        def __init__(self,x,y):super().__init__(normalize(x),normalize(y))
    module.CellArithmetic=NormalizedCellArithmetic
