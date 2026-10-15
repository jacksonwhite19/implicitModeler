"""Private complete-star contraction proposer; existing owners admit the edit.

No coordinates move. No quality threshold, geometry authority or publisher is
defined here. Protected attachment cells are derived from current cap incidence.
"""
from itertools import combinations
import numpy as np
from background_materialization import build
from background_topology_repair import key, boundary, orient
from coupled_model import CoupledPatch


def attachment_cells(source):
    protected=set()
    for face in source['cap_ids'][source['cap_faces']]:
        uses=source['owners'][key(face)]
        fresh=[i for i,_ in uses if i in source['editable_ids']]
        if len(uses)!=2 or len(fresh)!=1:
            raise ValueError('InvalidCurrentCapAttachment')
        protected.update(fresh)
    return protected


def link(cells, simplex):
    """Full simplicial link, including the empty simplex."""
    simplex=set(simplex);out=set()
    for cell in cells:
        vertices=set(cell['vertices'])
        if not simplex<=vertices:continue
        rest=sorted(vertices-simplex)
        for n in range(len(rest)+1):out.update(combinations(rest,n))
    return out


def propose(source, removed, retained, *, protected_ids, first_id, check):
    check();removed=int(removed);retained=int(retained)
    prefix=source['old_point_count'];byid=source['byid']
    if removed==retained:raise ValueError('IdenticalContractionEndpoints')
    if min(removed,retained)<prefix:raise ValueError('ContractionRequiresFreshEndpoints')
    if removed in source['physical_outer_vertices']:raise ValueError('RemovedPhysicalOuterVertex')
    ids=set(source['stars'].get(removed,()))
    if not ids or not ids<=source['editable_ids']:raise ValueError('InvalidRemovedVertexStar')
    if ids&set(protected_ids):raise ValueError('ProtectedAttachmentInRemovedStar')
    old=[byid[i] for i in sorted(ids)]
    if any(c['kind']!='tetrahedron' or removed not in c['vertices'] for c in old):
        raise ValueError('InvalidRemovedVertexStar')
    chain,faces,_=boundary(old)
    if any(removed in f for f in faces):raise ValueError('IncompleteInteriorRemovedStar')
    if retained not in {v for c in old for v in c['vertices']}:raise ValueError('NotAnExistingContractionEdge')
    adjacent=[byid[i] for i in source['stars'].get(retained,())]
    if any(c['kind']!='tetrahedron' or retained not in c['vertices'] for c in adjacent):
        raise ValueError('InvalidRetainedVertexStar')
    if link(old,{removed}) & link(adjacent,{retained}) != link(old,{removed,retained}):
        raise ValueError('ContractionLinkCondition')
    rows=[[retained if v==removed else v for v in c['vertices']]
          for c in old if retained not in c['vertices']]
    if not rows:raise ValueError('EmptyContractionReplacement')
    if len(set(map(key,rows)))!=len(rows):raise ValueError('DuplicateContractionReplacement')
    if first_id<=max(byid):raise ValueError('AliasedContractionCellNamespace')
    replacement=build(np.asarray(orient(source['points'],rows),dtype='<i8'),int(first_id))
    if boundary(replacement)[0]!=chain:raise ValueError('ChangedContractionBoundary')
    patch=CoupledPatch(source,ids,ids,replacement,source['points'],source,set(),check=check)
    # For a closed interior-star contraction every surviving vertex lies on the
    # unchanged star boundary. This path must never become a smoothing policy.
    if patch.moving:raise ValueError('UnexpectedContractionMovingSites')
    return patch,dict(operation='interior_edge_contraction',removed_vertex=removed,retained_vertex=retained,
        retired_cell_ids=sorted(ids),new_cell_ids=sorted(patch.active),old_cells=len(old),new_cells=len(replacement),
        coordinates_unchanged=True,attachment_cells_unchanged=True,link_condition_passed=True,
        exact_oriented_boundary_unchanged=True,quality=patch.initial_quality,
        scientific_mesh_acceptance=False)
