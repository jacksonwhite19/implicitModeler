"""Exact representation bridge for rationally preprocessed SymPy roots.

The sealed vendor predicates remain byte-identical. This bridge normalizes
QQ real_roots outputs before the vendor's exact ordering and field routines.
No floating-point approximation or acceptance rule is introduced.
"""
import sympy as sp
from functools import cmp_to_key
from .scaled_root_adapter import initial_intervals


def install(arithmetic):
    def canonical_real_root(value):
        value=sp.sympify(value)
        if value.is_Rational or isinstance(value,sp.CRootOf):return value
        shift,term=value.as_coeff_Add();scale,root=term.as_coeff_Mul()
        if not (shift.is_Rational and scale.is_Rational and scale!=0 and isinstance(root,sp.CRootOf) and root.is_real):
            raise ArithmeticError('ExpectedRationalAffineRealRoot:'+str(value))
        x=root.poly.gen
        p=arithmetic.primitive(root.poly.as_expr().subs(x,(x-shift)/scale),(x,))
        index=root.index if scale>0 else int(p.count_roots(-sp.oo,sp.oo))-1-root.index
        # Avoid SymPy repeating the rational preprocessing. The polynomial is
        # irreducible over QQ and the affine map determines the real-root index.
        return sp.CRootOf._new(sp.PurePoly(p.as_expr(),x),index)

    def unit_roots(p):
        if p.is_zero or p.degree()<=0:return []
        q=p.sqf_part()
        if q.domain==arithmetic.QQ:
            candidates=[canonical_real_root(r) for r in q.real_roots(radicals=False)]
        else:
            candidates=[canonical_real_root(r) for r in q.lift().sqf_part().real_roots(radicals=False)]
            alpha=q.domain.ext.root
            candidates=[r for r in candidates if arithmetic.polynomial_zero_at(q,alpha,r)]
        return arithmetic.ordered([r for r in candidates if arithmetic.compare(r,0)>=0 and arithmetic.compare(r,1)<=0])

    arithmetic.canonical_real_root=canonical_real_root
    arithmetic.legacy_unit_roots=arithmetic.unit_roots
    arithmetic.unit_roots=unit_roots

    # Reuse the qualified private-interval ownership rule. SymPy's global
    # CRootOf interval may have been refined by another pair before restart.
    # Exact sector samples must not depend on that unrelated cache history.
    def bounds(value):
        value=canonical_real_root(value)
        if value.is_Rational:return value,value
        key=str(value)
        if key not in arithmetic._intervals:
            arithmetic._intervals[key]=initial_intervals(value.poly)[value.index]
        q=arithmetic._intervals[key]
        return arithmetic.rational(q.a),arithmetic.rational(q.b)
    def ordered(values):
        initial=sorted({canonical_real_root(v) for v in values},key=str)
        out=[]
        for value in sorted(initial,key=cmp_to_key(arithmetic.compare)):
            if not out or arithmetic.compare(out[-1],value):out.append(value)
        return out
    arithmetic.bounds=bounds
    arithmetic.ordered=ordered
