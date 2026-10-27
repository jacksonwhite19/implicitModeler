"""Production execution: current geometry, qualified BVH, exact pair results.

This module imports no fixture, replay, historical scope, or verifier code.
Complete final-shell certification remains mandatory after local updates.
"""
from collections import Counter, OrderedDict
import hashlib
import json
import struct
import time
from pathlib import Path
import numpy as np
from . import kernel
from .guard import affected_guard
from .inputs import geometry

class CertificationFailure(RuntimeError):
    def __init__(self, record):
        self.record = record
        super().__init__(str(record))

def canonical_bytes(record):
    return (json.dumps(record, sort_keys=True, separators=(',', ':')) + '\n').encode()

class CertificateSession:
    """Optional bounded exact-authority memoization; no ID-only cache.

    Capacity zero is the uncached runtime baseline. All keys include source
    identity, ordered exact coordinates, original topology labels and rank
    declarations. Changed coordinates therefore cannot reuse stale entries.
    """
    def __init__(self, cache_capacity=0):
        self.capacity = int(cache_capacity)
        if self.capacity < 0:
            raise ValueError('NegativeCacheCapacity')
        self.cache = OrderedDict()
        self.hits = self.misses = 0

    def pair(self, a, b):
        authorities = tuple(sorted(((a.points, a.ids, a.declared_dimension),
                                    (b.points, b.ids, b.declared_dimension)), key=str))
        key = (kernel.VERSION, authorities)
        if self.capacity and key in self.cache:
            self.hits += 1
            value = self.cache.pop(key); self.cache[key] = value
            return json.loads(value)
        self.misses += 1
        result = kernel.certificate.certify(a, b)
        if self.capacity:
            # Immutable stored bytes prevent caller mutation of future results.
            self.cache[key] = canonical_bytes(result)
            if len(self.cache) > self.capacity:
                self.cache.popitem(last=False)
        return result

def side_arrays(positions, triangles, layer, vertices=None):
    x, f = geometry(positions, triangles, vertices)
    vertices = np.arange(x.shape[1]) if vertices is None else np.asarray(vertices)
    if x.ndim != 3 or x.shape[2] != 3 or not np.isfinite(x).all():
        raise ValueError('InvalidFixedCoordinates')
    if f.ndim != 2 or f.shape[1] != 3 or (len(f) and (f.min() < 0 or f.max() >= x.shape[1])):
        raise ValueError('InvalidTriangleConnectivity')
    if len(vertices) != x.shape[1] or len(set(map(int, vertices))) != len(vertices):
        raise ValueError('InvalidLineageIdentity')
    if not 1 <= layer < x.shape[0]:
        raise ValueError('InvalidLayer')
    edges = sorted(set(tuple(sorted((int(row[i]), int(row[(i+1)%3])))) for row in f for i in range(3)))
    quads = np.array([x[layer-1:layer+1, list(edge)].reshape(4, 3) for edge in edges])
    ids = [((layer-1, int(vertices[a])), (layer-1, int(vertices[b])),
            (layer, int(vertices[a])), (layer, int(vertices[b]))) for a, b in edges]
    return quads, ids, edges

def certify_layer(positions, triangles, layer, *, vertices=None, state='current', session=None, sink=None, pair_observer=None):
    started = time.perf_counter(); times = Counter(); counts = Counter(); methods = Counter()
    session = CertificateSession() if session is None else session
    t = time.perf_counter(); quads, labels, edges = side_arrays(positions, triangles, layer, vertices)
    patches = [kernel.Patch.binary(q, ids) for q, ids in zip(quads, labels)]
    times['side_extraction_and_exact_coordinates'] = time.perf_counter() - t
    # Certify side validity even when a side has no overlapping AABB partner.
    for patch in patches:
        valid, reason = patch.valid()
        if not valid:
            raise CertificationFailure({'classification': 'InvalidSidePatch', 'reason': reason})
    t = time.perf_counter(); lo, hi = kernel.enumerator.boxes(quads); times['AABB'] = time.perf_counter() - t
    t = time.perf_counter(); pairs, stats = kernel.enumerator.bvh_pairs(lo, hi)
    pairs = sorted(pairs); times['BVH_build_and_enumeration'] = time.perf_counter() - t
    if pair_observer is not None:
        pair_observer(state, layer, np.asarray(pairs, dtype=np.int32))
    vertices = np.arange(np.asarray(positions).shape[1]) if vertices is None else np.asarray(vertices)
    stream_hash = hashlib.sha256()
    for ia, ib in pairs:
        a, b = patches[ia], patches[ib]
        incident = bool(set(a.ids) & set(b.ids))
        t = time.perf_counter(); result = session.pair(a, b)
        times['certificate'] += time.perf_counter() - t
        methods[result['method']] += 1; counts[result['classification']] += 1
        counts['incident' if incident else 'nonincident'] += 1
        record = {'state': state, 'layer': layer, 'pair': [ia, ib], 'incident': incident,
                  'source_edges': [[int(vertices[v]) for v in edges[ia]], [int(vertices[v]) for v in edges[ib]]], **result}
        encoded = canonical_bytes(record); stream_hash.update(encoded)
        if sink is not None:
            t = time.perf_counter(); sink(encoded); times['serialization_output'] += time.perf_counter() - t
        if result['classification'] not in ('CertifiedSeparated', 'CertifiedAllowedIncidentContact'):
            raise CertificationFailure(record)
    return {'pass': True, 'state': state, 'layer': layer, 'sides': len(patches), 'pairs': len(pairs),
            'counts': dict(counts), 'methods': dict(methods), 'times': dict(times),
            'BVH': stats, 'records_sha256': stream_hash.hexdigest(), 'seconds': time.perf_counter() - started}

def certify_shell(positions, triangles, *, vertices=None, state='current', session=None, sink=None, pair_observer=None):
    geometry(positions, triangles, vertices)
    session = CertificateSession() if session is None else session
    results = [certify_layer(positions, triangles, k, vertices=vertices, state=state, session=session,
                            sink=sink, pair_observer=pair_observer) for k in range(1, len(positions))]
    return {'pass': True, 'mode': 'production', 'coverage': 'complete_supplied_shell', 'layers': results,
            'pairs': sum(r['pairs'] for r in results), 'cache_hits': session.hits, 'cache_misses': session.misses,
            'complete_final_shell_required_after_local_updates': True}

def certify_local(positions, triangles, changed_columns, *, vertices=None, reference=None, session=None, sink=None):
    guard = affected_guard(positions, triangles, changed_columns, reference)
    f = np.asarray(triangles)[guard['columns']]
    # Keep complete global coordinate and ancestry arrays. Only columns are
    # selected; shared vertices cannot acquire separate local coordinates.
    result = certify_shell(positions, f, vertices=vertices, state='local', session=session, sink=sink)
    result.update(coverage='complete_changed_column_guard', guard={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in guard.items()})
    return result

def read_fixed_layer(path):
    with open(path, 'rb') as stream:
        if stream.read(8) != b'V3SIDE1\0':
            raise ValueError('OnlyFixedV3SIDE1IsQualified; V3SIDH1 remains the legacy interval-height path')
        n, m = struct.unpack('<QQ', stream.read(16))
        x = np.frombuffer(stream.read(2*n*3*8), dtype='<f8').reshape(2, n, 3)
        f = np.frombuffer(stream.read(m*3*8), dtype='<u8').reshape(m, 3)
        if stream.read(1):
            raise ValueError('TrailingInput')
    return x, f

def certify_fixed_layer(path, *, session=None, sink=None):
    x, f = read_fixed_layer(path)
    return certify_layer(x, f, 1, session=session, sink=sink)
