"""Exact committed-coordinate protection for an already bound fixed topology.

One complete native guard establishes cell membership/connectivity. Subsequent
trials use the same templates and layer map, and may only change coordinates.
This is not a remesh guard. Final production publication still uses its full
committed-state checks.
"""
import numpy as np


class FixedTrialGuard:
    def __init__(self, before, candidate, points, source_count):
        from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import preserves_committed_cells
        self.valid = preserves_committed_cells(candidate,before)
        self.shape = points.shape
        ids = sorted({v for c in before['cells'].values() for v in c['vertices']})
        self.indices = []
        expected = []
        for key in ids:
            tag,layer,source,vertex = key.split(':')
            if tag!='layer' or source!='source':raise ValueError('UnboundNativeVertexKey')
            index=int(layer)*source_count+int(vertex)
            if not np.array_equal(points[index],candidate['vertices'][key]):
                raise ValueError('TrialCoordinateIndexMismatch')
            self.indices.append(index);expected.append(before['vertices'][key])
        self.indices=np.asarray(self.indices,dtype=np.int64)
        self.expected=np.asarray(expected,dtype=float)

    def preserves(self,points):
        return bool(self.valid and points.shape==self.shape and
                    np.array_equal(points[self.indices],self.expected))
