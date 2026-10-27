"""Memoize textual interval keys without changing exact root arithmetic.

This context is process-local and must not overlap another thread using the
same kernel namespace. Each worker owns its namespace. Keys and interval
refinement operations are identical to the frozen scaled-root adapter.
"""
from contextlib import contextmanager
from functools import lru_cache
from .incremental import kernel

@contextmanager
def memoized_root_keys():
    m=kernel.module('exact_algebraic');adapter=kernel.adapter
    previous_bounds,previous_refine=m.bounds,m.refine
    text=lru_cache(maxsize=20000)(str)
    def bounds(value):
        value=adapter.normalize(value)
        if value.is_Rational:return value,value
        key=text(value)
        if key not in m._intervals:m._intervals[key]=adapter.initial_intervals(value.poly)[value.index]
        q=m._intervals[key]
        return m.rational(q.a),m.rational(q.b)
    def refine(value):
        value=adapter.normalize(value)
        if value.is_Rational:return
        key=text(value);bounds(value);m._intervals[key]=m._intervals[key].refine()
    m.bounds,m.refine=bounds,refine
    try:yield text
    finally:m.bounds,m.refine=previous_bounds,previous_refine
