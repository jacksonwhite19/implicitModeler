"""Memoize pure exact polynomial preparation, not sign/refinement decisions.

Each worker owns its private kernel namespace. The QQ root candidates depend
only on the exact polynomial and its domain. Every algebraic-domain filter,
unit-interval comparison, ordering and private interval refinement still runs.
Neither CellArithmetic nor the CAD traversal is changed.
"""
from contextlib import contextmanager
from functools import lru_cache
from collections import Counter
from .incremental import kernel

@contextmanager
def memoized_exact_preparation(capacity=4096):
    if capacity<1:raise ValueError('PositivePolynomialCacheBoundRequired')
    m=kernel.module('exact_algebraic');n=kernel.module('noncoincident_rank3_certificate')
    original=m.unit_roots;original_alias=n.unit_roots;stats=Counter()
    @lru_cache(maxsize=capacity)
    def candidates(p):
        q=p.sqf_part()
        if q.domain==m.QQ:roots=q.real_roots(radicals=False)
        else:roots=q.lift().sqf_part().real_roots(radicals=False)
        return q,tuple(roots)
    def unit_roots(p):
        stats['requests']+=1
        if p.is_zero or p.degree()<=0:return []
        q,roots=candidates(p)
        if q.domain!=m.QQ:
            alpha=q.domain.ext.root
            roots=[r for r in roots if m.polynomial_zero_at(q,alpha,r)]
        return m.ordered([r for r in roots if m.compare(r,0)>=0 and m.compare(r,1)<=0])
    m.unit_roots=n.unit_roots=unit_roots
    try:yield stats
    finally:
        stats.update({k:v for k,v in candidates.cache_info()._asdict().items() if k!='maxsize'})
        m.unit_roots=original;n.unit_roots=original_alias
