"""Explicit qualification progression convention; no geometry generation.

The growth reference is an advisor-adopted engineering policy, not a hard
quality gate or production default. Physics is supplied, never inferred here.
"""
import math
from decimal import Decimal, localcontext
from fractions import Fraction
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import identity, file_hash
from production_prism_sizing import read_request


def positive(x):
    if isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or x<=0:
        raise ValueError('PositiveFiniteNumberRequired')
    return float(x)


def explicit(h1,r,n):
    h1=positive(h1);r=positive(r)
    if type(n) is not int or n<2 or r<1:raise ValueError('OutsideIncreasingQualificationFamily')
    increments=[h1*r**k for k in range(n)]
    total=0.
    for x in increments:
        if not math.isfinite(x) or x<=0:raise ValueError('InvalidIncrement')
        nxt=total+x
        if not math.isfinite(nxt) or nxt<=total:raise ValueError('InvalidSequentialTotal')
        total=nxt
    assert increments[0].hex()==h1.hex()
    return increments,total


def solve(h1,target,n):
    """Stable sum-form root, then minimize sequential binary64 residual.

80-digit Decimal bisection avoids cancellation near r=1. Search terminates
at 256 bisections. Evaluate the nearest float and its immediate neighbours;
choose smallest exact residual, then distance to root, then smaller float.
No increment or physical input is rescaled.
"""
    h1=positive(h1);target=positive(target)
    if type(n) is not int or n<2 or n*Fraction(h1)>Fraction(target):
        raise ValueError('NoIncreasingFamilySolution')
    with localcontext() as ctx:
        ctx.prec=80
        h,t=Decimal.from_float(h1),Decimal.from_float(target)
        def total(r):
            term=h;s=term
            for _ in range(1,n):term*=r;s+=term
            return s
        lo=Decimal(1);hi=Decimal(2)
        if n*Fraction(h1)==Fraction(target):root=lo
        else:
            while total(hi)<t:hi*=2
            for _ in range(256):
                mid=(lo+hi)/2
                if total(mid)<t:lo=mid
                else:hi=mid
            root=(lo+hi)/2
        rf=float(root)
        candidates=sorted(set([rf,math.nextafter(rf,-math.inf),math.nextafter(rf,math.inf)]))
        scored=[]
        for r in candidates:
            if r<1:continue
            inc,s=explicit(h1,r,n)
            scored.append((abs(Fraction(s)-Fraction(target)),abs(Decimal.from_float(r)-root),r,inc,s))
        _,_,r,inc,s=min(scored)
    return dict(N=n,growth=r,growth_hex=r.hex(),increments=inc,increments_hex=[x.hex() for x in inc],
                final_increment=inc[-1],final_first_ratio=inc[-1]/h1,nominal_total=s,
                nominal_total_hex=s.hex(),physical_total_intent=target,
                difference=s-target,difference_hex=(s-target).hex())


def family(h1,target):
    h1=positive(h1);target=positive(target)
    maximum=math.floor(Fraction(target)/Fraction(h1))
    return [solve(h1,target,n) for n in range(2,maximum+1)]


def first_crossing(rows,reference):
    reference=positive(reference)
    return next((r['N'] for r in rows if r['growth']<=reference),None)


def adopt(rows,reference):
    """Reference must be explicitly supplied by the qualification declaration."""
    n=first_crossing(rows,reference)
    if n is None:raise ValueError('NoScheduleMeetsAdoptedReference')
    return next(r for r in rows if r['N']==n)


def verify_bound_request(path,surface,expected_identity):
    result=read_request(path,surface)
    if identity(result['declaration'])!=expected_identity:raise ValueError('SealedRequestMutation')
    return result
