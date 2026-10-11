"""Inclusive projected-box lookup; preserves dense candidate order exactly."""
import numpy as np
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex

class ProjectedBoundaryIndex:
    def __init__(self, lo, hi):
        self.lo, self.hi = np.asarray(lo), np.asarray(hi)
        if not np.isfinite(self.lo).all() or not np.isfinite(self.hi).all():
            self.index = None
            return
        self.index = AABBIndex(np.column_stack((lo, np.zeros(len(lo)))),
                               np.column_stack((hi, np.zeros(len(hi)))))

    def containing(self, point):
        index = self.index
        if index is None:
            return np.flatnonzero(np.all(point >= self.lo, axis=1) & np.all(point <= self.hi, axis=1)).tolist()
        point = np.array([point[0], point[1], 0.0])
        stack = [] if index.root is None else [index.root]
        found = []
        while stack:
            lower, upper, left, right = index.nodes[stack.pop()]
            if np.any(point < lower) or np.any(point > upper):
                continue
            if right is None:
                mask = np.all(point >= index.lo[left], axis=1) & np.all(point <= index.hi[left], axis=1)
                found.extend(left[mask].tolist())
            else:
                stack.extend((right, left))
        return sorted(found)
