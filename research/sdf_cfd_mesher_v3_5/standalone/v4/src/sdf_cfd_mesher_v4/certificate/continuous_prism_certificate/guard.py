"""Complete swept-column AABB guard for supplied changed geometry."""
import numpy as np
from .inputs import geometry, column_indices

def affected_guard(positions, triangles, changed_columns, reference=None):
    x, f = geometry(positions, triangles)
    targets = column_indices(changed_columns, len(f))
    if len(targets) and (targets.min() < 0 or targets.max() >= len(f)):
        raise ValueError('ChangedColumnOutsideInput')
    changed_vertices = np.unique(f[targets]) if len(targets) else np.empty(0, dtype=np.int64)
    if reference is not None:
        old = np.asarray(reference)
        if old.shape != x.shape or old.dtype != x.dtype or not np.isfinite(old).all():
            raise ValueError('ReferenceShapeMismatch')
        changed_vertices = np.union1d(changed_vertices, np.flatnonzero(np.any(x != old, axis=(0, 2))))
    # Every incident column is affected when any shared lineage moves.
    affected = np.flatnonzero(np.any(np.isin(f, changed_vertices), axis=1))
    lo = np.empty((len(f), 3)); hi = np.empty_like(lo)
    for start in range(0, len(f), 1500):
        q = x[:, f[start:start+1500]]
        lo[start:start+1500] = q.min(axis=(0, 2))
        hi[start:start+1500] = q.max(axis=(0, 2))
        if reference is not None:
            q = old[:, f[start:start+1500]]
            lo[start:start+1500] = np.minimum(lo[start:start+1500], q.min(axis=(0, 2)))
            hi[start:start+1500] = np.maximum(hi[start:start+1500], q.max(axis=(0, 2)))
    selected = np.zeros(len(f), dtype=bool)
    # The frozen guard's outward enlargement is retained. It is broad-phase
    # conservatism, never a contact tolerance or an intersection proof.
    padding = 128 * np.finfo(float).eps
    for column in affected:
        selected |= np.all(hi >= lo[column] - padding, axis=1) & np.all(lo <= hi[column] + padding, axis=1)
    guards = np.flatnonzero(selected)
    return {'columns': guards, 'affected_columns': affected, 'changed_vertices': changed_vertices,
            'complete_input_column_count': len(f), 'reference_union': reference is not None,
            'rule': 'Shared-lineage incidence closure plus full-input swept-column AABB overlap'}
