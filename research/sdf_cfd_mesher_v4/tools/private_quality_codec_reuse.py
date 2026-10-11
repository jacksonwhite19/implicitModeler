"""Capsule accepted-byte verification inside the existing private quality writer.

No writer, format or receipt is replaced. Reuse the original input canonical
stream while checking every emitted blob and every database readback exactly.
"""
from contextlib import contextmanager
import threading
import json
from itertools import chain
from unittest.mock import patch
import zlib

import single_quality_component as quality
import critical_quality_codec as codec
from incremental_transaction import canonical

_lock = threading.Lock()
_base_hashes = quality.hashes


def same_key(key, expected):
    """Identical builtin strings necessarily have identical JSON encodings.

    Do not use subclass equality: custom __eq__ can hide a changed key. All
    other cases retain canonical comparison and errors. Unequal strings can
    encode identically (a Unicode scalar and its explicit surrogate pair).
    """
    if type(key) is str and type(expected) is str and key == expected:
        return True
    return canonical(key) == canonical(expected)


@contextmanager
def verified_private_codec(rows, *, workers=1, process=False, reuse_hash_stream=True):
    """Original list-backed private writer only; never trust a cold file witness."""
    if type(reuse_hash_stream) is not bool:raise ValueError('InvalidPrivateQualityHashReuse')
    if type(workers) is not int or workers not in (1,2,4):raise ValueError('InvalidQualityCodecWorkers')
    if type(process) is not bool:raise ValueError('InvalidQualityCodecProcess')
    if type(rows) is not list:
        yield
        return
    if not _lock.acquire(blocking=False):
        raise ValueError('ConcurrentPrivateQualityCodecReuse')
    thread = threading.get_ident()
    old_hashes = quality.hashes; old_verify = quality._verify; old_encode = codec.encoded
    accepted = []; expected_blobs = []; blobs = {}; order = []; logical = []; database = []
    old_connect = quality.sqlite3.connect
    def connect(*a,**k):
        db = old_connect(*a,**k)
        if threading.get_ident()==thread and not database and logical:
            # The original writer opens its private connection immediately after
            # the accepted input hash; retain the connection object, not a name.
            database.append(db)
        return db
    def hashes(values):
        if threading.get_ident() != thread or logical:
            return old_hashes(values)
        original=[]
        iterator=iter(values)
        for i,row in enumerate(iterator):
            original.append(row)
            if i>=len(rows) or row is not rows[i]:
                return old_hashes(chain(original,iterator))
        if len(original)!=len(rows):
            return old_hashes(original)
        if reuse_hash_stream and old_hashes is _base_hashes:
            from private_quality_hash_stream import hashes as private_hashes
            result=private_hashes(original,accepted,workers,process,codec_witness=expected_blobs)
            logical.append(result)
            return result
        snapshots=[];byte_rows=[]
        for row in original:
            data = canonical(row)
            snapshot=json.loads(data)
            if canonical(snapshot)!=data:
                raise ValueError('NoncanonicalPrivateQualityInput')
            # Both legacy logical hashes see this same detached JSON snapshot.
            # No supplied-row alias can mutate it between compact/pretty hashes.
            snapshots.append(snapshot); byte_rows.append(data)
        result = old_hashes(snapshots)
        accepted.extend(byte_rows)
        logical.append(result)
        return result
    def encoded(values, workers=1, process=False):
        offset = len(order)
        own_values = (values is rows or (type(values) is list and bool(values) and
            len(values)+offset<=len(rows) and all(v is rows[offset+i] for i,v in enumerate(values))))
        if threading.get_ident() != thread or not own_values:
            yield from old_encode(values, workers, process)
            return
        if len(logical) != 1 or len(accepted) != len(rows):
            raise ValueError('MissingPrivateQualityInputWitness')
        if expected_blobs and len(expected_blobs)!=len(rows):
            raise ValueError('IncompletePrivateQualityCodecWitness')
        count = 0
        for local, (inputs, payload) in enumerate(old_encode(values, workers, process)):
            i = offset+local
            if i >= len(accepted):
                raise ValueError('ExtraPrivateQualityCodecRow')
            key = values[local]['key']
            if expected_blobs and (same_key(key,expected_blobs[i][0])
                                   and (inputs,payload)==expected_blobs[i][1]):
                # Exact independently derived bytes, not decoded equality.
                # The original codec still executes on supplied rows. Mutation,
                # pickle changes and noncanonical encodings cannot pass this.
                pass
            else:
                # Preserve original failure distinctions and the unoptimized
                # fallback when a custom hash stream cannot supply a witness.
                raw_inputs = zlib.decompress(inputs)
                raw_payload = b'null' if payload is None else zlib.decompress(payload)
                reconstructed = b'{"inputs":'+raw_inputs+b',"key":'+canonical(key)+b',"payload":'+raw_payload+b'}'
                if reconstructed != accepted[i]:
                    raise ValueError('ChangedPrivateQualityInput')
                if expected_blobs or inputs != zlib.compress(raw_inputs) or payload is not None and payload != zlib.compress(raw_payload):
                    raise ValueError('NoncanonicalPrivateQualityCodec')
            if key in blobs:
                raise ValueError('DuplicatePrivateQualityKey')
            blobs[key] = (inputs, payload); order.append(key); count += 1
            yield inputs, payload
        if count != len(values):
            raise ValueError('IncompletePrivateQualityCodec')
    def verify(db, expected, witness=None):
        if threading.get_ident() != thread or not database or db is not database[0]:
            return old_verify(db, expected, witness)
        if len(logical) != 1 or expected != logical[0] or len(order) != expected['count'] or witness is None:
            raise ValueError('UnboundPrivateQualityReadback')
        validated = dict(order=order, blobs=blobs)
        quality._verify_encoded(db, expected, validated)
        witness.update(validated)
    try:
        with patch.object(quality,'hashes',hashes), patch.object(codec,'encoded',encoded), patch.object(quality,'_verify',verify), \
                patch.object(quality.sqlite3,'connect',connect):
            yield
    finally:
        _lock.release()
