"""Bounded pure-key cache; coordinates and ownership are never cached here."""
from functools import lru_cache
from contextlib import ExitStack, contextmanager
import sys
from unittest.mock import patch
from sdf_cfd_mesher_v4.advancing_front import mixed_front

ORIGINAL = mixed_front.face_key
ORIGINAL_CYCLE = mixed_front.cycle


def cycle(vertices):
    value = tuple(vertices)
    if len(value) < 3 or len(set(value)) != len(value):
        raise ValueError('DegenerateFaceConnectivity')
    # Unique scalar IDs imply a unique least first vertex. All other rotations
    # are lexicographically larger regardless of their remaining entries.
    if not all(type(v) in (str, int) for v in value):
        return ORIGINAL_CYCLE(value)
    first = value.index(min(value))
    return value[first:] + value[:first]


@lru_cache(maxsize=524288)
def cached(value):
    return mixed_front.identity(min(cycle(value), cycle(tuple(reversed(value)))))


def face_key(vertices):
    return cached(tuple(vertices))


@contextmanager
def installed():
    cached.cache_clear()
    with ExitStack() as stack:
        for module in list(sys.modules.values()):
            if module is None or module is sys.modules[__name__]:
                continue
            if getattr(module, 'face_key', None) is ORIGINAL:
                stack.enter_context(patch.object(module, 'face_key', face_key))
            if getattr(module, 'cycle', None) is ORIGINAL_CYCLE:
                stack.enter_context(patch.object(module, 'cycle', cycle))
        try:
            yield
        finally:
            cached.cache_clear()
