"""Strict admission to the exact binary64 coordinate/topology domain."""
import numpy as np

def geometry(positions, triangles, vertices=None):
    x = np.asarray(positions)
    f = np.asarray(triangles)
    if x.dtype != np.dtype('float64') or x.ndim != 3 or x.shape[-1] != 3:
        raise ValueError('CoordinatesMustBeBinary64LayerVertexXYZ')
    if x.shape[0] < 2 or x.shape[1] < 3 or not np.isfinite(x).all():
        raise ValueError('EmptyOrNonfiniteLayerGeometry')
    if f.ndim != 2 or f.shape[1] != 3 or len(f) == 0 or f.dtype.kind not in 'iu':
        raise ValueError('TriangleIndicesMustBeNonemptyIntegers')
    if f.min() < 0 or f.max() >= x.shape[1]:
        raise ValueError('TriangleIndexOutsideCoordinates')
    if np.any((f[:,0] == f[:,1]) | (f[:,0] == f[:,2]) | (f[:,1] == f[:,2])):
        raise ValueError('RepeatedTriangleVertex')
    if vertices is not None:
        ids = np.asarray(vertices)
        if ids.ndim != 1 or ids.dtype.kind not in 'iu' or len(ids) != x.shape[1] or len(np.unique(ids)) != len(ids):
            raise ValueError('LineageIdentitiesMustBeUniqueIntegers')
    return x, f

def column_indices(values, count):
    a = np.asarray(values)
    if a.ndim != 1 or (a.size and a.dtype.kind not in 'iu'):
        raise ValueError('ChangedColumnsMustBeIntegers')
    if a.size and (a.min() < 0 or a.max() >= count):
        raise ValueError('ChangedColumnOutsideInput')
    return np.asarray(sorted(set(map(int, a))), dtype=np.int64)
