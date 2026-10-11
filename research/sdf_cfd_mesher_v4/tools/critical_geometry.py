"""Exact packed witnesses and authenticated source-key reuse; no new theorem."""
from fractions import Fraction
import numpy as np
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key


def exact_witnesses(points, cells):
    """Same exact rational mean, sharing each binary64 conversion once."""
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('InvalidWitnessCoordinates')
    ratios = [[float(x).as_integer_ratio() for x in row] for row in points]
    denominator = max(d for row in ratios for _, d in row)
    packed = np.array([[n * (denominator // d) for n, d in row] for row in ratios], dtype=object)
    del ratios
    if hasattr(cells, 'connectivity_chunks'):
        chunks = cells.connectivity_chunks
    elif hasattr(cells, 'packed'):
        chunks = [np.asarray(a[:, 3:]) for a in cells.packed.cell_segments]
        if cells.added:
            chunks.append(np.asarray([c['vertices'] for c in cells.added], dtype=np.int64))
    else:
        chunks = [np.asarray([sorted(set(c['vertices'])) for c in cells], dtype=np.int64)]
    result = []
    for ids in chunks:
        if ids.ndim != 2 or not ids.shape[1] or (ids.size and (ids.min() < 0 or ids.max() >= len(points))):
            raise ValueError('InvalidWitnessConnectivity')
        if np.any(np.diff(np.sort(ids, axis=1), axis=1) == 0):
            raise ValueError('DuplicateWitnessVertex')
        # Bounded chunks avoid a full (cells,vertices,3) object array.
        for start in range(0, len(ids), 16384):
            sums = packed[ids[start:start+16384]].sum(axis=1)
            result.extend(tuple(Fraction(int(v), denominator * ids.shape[1]) for v in row) for row in sums)
    return result


def degree_process(points, faces, connectivity, workers):
    """Isolated exact witness preparation plus the qualified degree workers."""
    from types import SimpleNamespace
    from prism_degree import degree, PERFORMANCE
    import time
    import psutil
    start = time.perf_counter()
    cpu = time.process_time()
    witnesses = exact_witnesses(points, SimpleNamespace(connectivity_chunks=connectivity))
    prepared = time.perf_counter()
    result = degree(points, faces, witnesses, workers=workers)
    return result, dict(witness_seconds=prepared-start, degree_seconds=time.perf_counter()-prepared,
                        total_seconds=time.perf_counter()-start, coordinator_cpu=time.process_time()-cpu,
                        coordinator_peak_rss=psutil.Process().memory_info().peak_wset, isolated_process=True, workers=workers,
                        worker_details=dict(PERFORMANCE))


def merge(newer, parent):
    newer = iter(newer)
    item = next(newer, None)
    for old in parent:
        while item is not None and item[0] < old[0]:
            yield item
            item = next(newer, None)
        if item is not None and item[0] == old[0]:
            yield item
            item = next(newer, None)
        else:
            yield old
    while item is not None:
        yield item
        item = next(newer, None)


def source_items(index):
    """Ordered numeric face, owner winding, and previously authenticated hash."""
    from persistent_numeric_incidence import PackedClosure
    if isinstance(index, PackedClosure):
        yield from merge(source_items(index.delta), source_items(index.parent))
        return
    index.live()
    lookup = index.lookup
    rows = np.asarray(lookup.rows)
    if len(rows) != len(index) or not np.array_equal(np.sort(rows), np.arange(len(rows))):
        raise ValueError('InvalidSourceLookupPermutation')
    hashes = np.empty(len(rows), dtype='S64')
    hashes[rows] = lookup.hashes
    # Only the first owner winding is required by the complete proof query.
    for numeric, winding, digest in zip(np.asarray(index.keys), np.asarray(index.winding)[:, 0], hashes):
        yield tuple(int(v) for v in numeric if v >= 0), tuple(int(v) for v in winding if v >= 0), bytes(digest).decode('ascii')


def proof_geometry(owners, n):
    newer = ((key, rows[0][1], face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in rows[0][1])))
             for key, rows in sorted(owners.delta.items()))
    faces, keys = [], []
    for _, winding, digest in merge(newer, source_items(owners.parent)):
        faces.append(winding)
        keys.append(digest)
    return faces, keys


def ndarray_views(index):
    """Zero-copy read-only views retain the original guarded mmap storage."""
    from persistent_numeric_incidence import PackedClosure
    if isinstance(index, PackedClosure):
        ndarray_views(index.parent)
        ndarray_views(index.delta)
        return index
    for name in ('keys', 'owners', 'winding', 'cells', 'search'):
        setattr(index, name, np.asarray(getattr(index, name)))
    if hasattr(index, 'lookup'):
        for name in ('hashes', 'rows'):
            setattr(index.lookup, name, np.asarray(getattr(index.lookup, name)))
    return index
