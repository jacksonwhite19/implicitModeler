"""Bounded process-local reuse of pure exact polynomial operations.

Keys include the immutable Poly (coefficients, generators and exact domain),
all positional arguments, and sorted keyword arguments. No sign decision,
CAD traversal or mutable private isolating interval is cached. Returned lists
are copied so a caller cannot change the cached sequence. Caches are ephemeral:
a restarted process recomputes them under its current SymPy implementation.
"""
from contextlib import contextmanager
from functools import lru_cache
import sympy as sp

VERSION = 'V4ExactPolynomialOperations1'
_ORIGINALS = {name: getattr(sp.Poly, name) for name in ('real_roots', 'gcd', 'sturm')}

def _make(name):
    original = _ORIGINALS[name]
    @lru_cache(maxsize=4096)
    def cached(args, kwargs):
        value = original(*args, **dict(kwargs))
        return tuple(value) if isinstance(value, list) else value
    def call(*args, **kwargs):
        value = cached(args, tuple(sorted(kwargs.items())))
        return list(value) if name in ('real_roots', 'sturm') else value
    return cached, call

_OPERATIONS = {name: _make(name) for name in _ORIGINALS}

def cache_info():
    return {name: cache.cache_info()._asdict() for name, (cache, _) in _OPERATIONS.items()}

def clear_cache():
    for cache, _ in _OPERATIONS.values():
        cache.cache_clear()

@contextmanager
def memoized_cad_algebra():
    """Use only in a certificate worker's single-threaded exact section."""
    previous = {name: getattr(sp.Poly, name) for name in _OPERATIONS}
    try:
        for name, (_, call) in _OPERATIONS.items():
            setattr(sp.Poly, name, call)
        yield
    finally:
        for name, original in previous.items():
            setattr(sp.Poly, name, original)
