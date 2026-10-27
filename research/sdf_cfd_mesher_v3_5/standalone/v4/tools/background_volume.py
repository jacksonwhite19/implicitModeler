"""Existing exact-volume extension and FULL global degree for a live append.

The caller authenticates the accepted shell/cache, unchanged prefix/incidence,
positive fresh exact cells, complete mixed certificate and its independent
coverage before calling. This helper cannot create or read a LiveAcceptance.
"""
import numpy as np
from prism_hotpath_v2 import exact_centroid,exact_boundary


def audit(points,accepted,old_ids,additions,new_exact,outer,wall):
    old_ids=set(old_ids);ids=[c['id'] for c in additions]
    if (set(accepted.witness)!=old_ids or accepted.total!=accepted.boundary_total or
            len(ids)!=len(set(ids)) or set(ids)&old_ids):
        raise ValueError('ChangedAcceptedVolumeCoverage')
    if (set(new_exact)!=set(ids) or any(r['jacobian_sign']<=0 or r['volume_sign']<=0
                                     for r in new_exact.values())):
        raise ValueError('IncompletePositiveNewExactCells')
    if not np.isfinite(points).all():raise ValueError('InvalidVolumeCoordinates')
    result=accepted.update(points,additions,(),{},new_exact,outer,wall,
                           sample=exact_centroid,contribution=exact_boundary)
    # Every old AND new interior witness is checked against the new full boundary.
    degree=result.degree(points,outer,wall)
    return dict(interior_degree=degree,witness_ids=sorted(result.witness),
        cell_volume=str(result.total),boundary_volume=str(result.boundary_total),
        exact_volume_balance=result.total==result.boundary_total,positive_cells=True,
        conditional_nonoverlap=degree['passed'],
        prerequisites='Must also require complete unique-face exact certificate and oriented manifold incidence. Degree samples alone are not a volume certificate.',
        proof='With nonintersecting cell boundaries, interior overlap without a boundary crossing requires containment or coincident domains. Each standard-cell interior witness has global oriented boundary degree one, excluding containment; opposite internal face incidence and positive orientation exclude coincident domains. Complete face matching closes the oriented cell complex. Exact total cell/boundary volume balance additionally checks filling accounting.')
