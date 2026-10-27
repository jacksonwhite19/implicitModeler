"""Opt-in CPU reuse for the existing shared-schedule codec.

The pinned codec, schema, physical writer and exact expanded digest are unchanged.
Only repeated JSON work is shared. No authentication, token or digest is skipped.
"""
from contextlib import contextmanager
from copy import deepcopy
import threading
from unittest.mock import patch

import current_front_dependencies as codec
from sdf_cfd_mesher_v4.common.authority import identity
from incremental_transaction import canonical

_row = codec._row
_validated = codec._validated
_lock = threading.RLock()
_thread = None


def metadata_row(key, row):
    # Retain the original shape, types, keys and JSON validation of metadata.
    if not isinstance(row, dict) or set(row) != {'current_layer', 'region', 'remaining_schedule'}:
        return _row(key, row)
    if not isinstance(row['remaining_schedule'], list):
        return _row(key, row)
    _row(key, dict(row, remaining_schedule=[]))


def plain_json(value):
    """Only builtin JSON trees use the compact encoder fast path.

    Custom containers can override iteration/encoding. Preserve the original
    pretty encoder for them, including non-string object keys and tuples.
    """
    kind=type(value)
    if kind is str or kind is int or kind is float or kind is bool or kind is type(None):return True
    if kind is list:return all(plain_json(v) for v in value)
    if kind is dict:return all(type(k) is str and plain_json(v) for k,v in value.items())
    return False


def encode_rows(values):
    """Cache by exact JSON text, never mutable list identity or Python equality.

    Every supplied schedule is serialized once. Equal canonical JSON text implies
    equal input to the existing compact identity encoder; signed zero, bool/int,
    strings and nonfinite rejection retain their original JSON distinctions.
    """
    from collections.abc import Mapping
    rows = {}; schedules = {}; seen = set(); encoded = {}
    iterator = values.items() if isinstance(values, Mapping) else values
    for key, row in iterator:
        metadata_row(key, row)
        if key in seen:
            raise ValueError('DuplicateDependencySource')
        seen.add(key)
        schedule = row['remaining_schedule']
        # This text is an operation-local equality key, not the physical or
        # expanded scientific encoding. Compact and pretty JSON preserve the
        # same distinctions for builtin trees. Original digest/writer stays.
        text = canonical(schedule) if plain_json(schedule) else codec._json(schedule)
        digest = encoded.get(text)
        if digest is None:
            digest = identity(schedule)
            encoded[text] = digest
        if digest in schedules and schedules[digest] != schedule:
            raise ValueError('ConflictingSharedSchedule')
        if digest not in schedules:
            schedules[digest] = deepcopy(schedule)
        rows[key] = dict(current_layer=row['current_layer'], region=row['region'], schedule=digest)
    expanded, size = codec._digest_rows(rows, schedules)
    return rows, schedules, expanded, size


def validated(body, shared):
    """Validate each shared schedule once, while retaining the original validator.

    Parsed built-in JSON containers only. Foreign/custom containers use the
    original implementation. All envelope, root, usage and expanded-byte checks
    stay in that implementation. Other threads still use its original row check.
    """
    if _thread is not None and _thread != threading.get_ident():
        return _validated(body, shared)
    if (type(body) is not dict or type(shared) is not dict or
            type(shared.get('schedules')) is not dict or type(body.get('rows')) is not dict):
        return _validated(body, shared)
    schedules = shared['schedules']
    if any(type(v) is not list for v in schedules.values()):
        return _validated(body, shared)
    for value in schedules.values():
        codec._json(value)
    allowed = {id(v): v for v in schedules.values()}
    thread = threading.get_ident()
    def checked(key, row):
        if (threading.get_ident() != thread or type(row) is not dict or
                id(row.get('remaining_schedule')) not in allowed or
                allowed[id(row['remaining_schedule'])] is not row['remaining_schedule']):
            return _row(key, row)
        return metadata_row(key, row)
    with _lock, patch.object(codec, '_row', checked):
        return _validated(body, shared)


@contextmanager
def installed():
    """Bounded installer, restored on failure/exit; old reader bytes stay pinned."""
    global _thread
    if not _lock.acquire(blocking=False):
        raise ValueError('ConcurrentSharedScheduleHotpath')
    if _thread is not None:
        _lock.release()
        raise ValueError('NestedSharedScheduleHotpath')
    _thread = threading.get_ident()
    try:
        with patch.object(codec, 'encode_rows', encode_rows), patch.object(codec, '_validated', validated):
            yield
    finally:
        _thread = None
        _lock.release()
