"""Caller-selected graph regions and exact dependency invalidation."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class LocalRegion:
    vertex_ids: tuple[int,...]
    edge_ids: tuple[int,...]
    triangle_ids: tuple[int,...]
    oversized: bool

def expand_vertices(topology,vertices,rings=0):
    if rings<0:raise ValueError('NegativeRings')
    n=len(topology.vertex_neighbors.offsets)-1;v=set(map(int,vertices))
    if any(i<0 or i>=n for i in v):raise ValueError('InvalidVertexID')
    for _ in range(rings):v.update(j for i in tuple(v) for j in topology.vertex_neighbors.row(i))
    return np.asarray(sorted(v),dtype=np.int64)
def dependencies(topology,vertices,rings=0):
    v=expand_vertices(topology,vertices,rings)
    e=np.unique(np.concatenate([topology.vertex_edges.row(i) for i in v])) if len(v) else np.empty(0,dtype=np.int64)
    t=np.unique(np.concatenate([topology.vertex_triangles.row(i) for i in v])) if len(v) else np.empty(0,dtype=np.int64)
    return v,e,t
def extract_regions(topology,mask,*,entity='vertex',rings=0,max_vertices):
    if max_vertices<1:raise ValueError('InvalidRegionBound')
    mask=np.asarray(mask)
    if mask.dtype!=np.bool_ or mask.ndim!=1:raise ValueError('BooleanMaskRequired')
    population={'vertex':len(topology.vertex_neighbors.offsets)-1,'edge':len(topology.edges),'triangle':len(topology.triangles)}
    if entity not in population or len(mask)!=population[entity]:raise ValueError('MaskPopulationMismatch')
    ids=np.flatnonzero(mask)
    vertices=ids if entity=='vertex' else np.unique((topology.edges if entity=='edge' else topology.triangles)[ids])
    remaining=set(map(int,expand_vertices(topology,vertices,rings)));regions=[]
    while remaining:
        seed=min(remaining);remaining.remove(seed);stack=[seed];group=[]
        while stack:
            v=stack.pop();group.append(v)
            linked=[int(x) for x in topology.vertex_neighbors.row(v) if x in remaining]
            remaining.difference_update(linked);stack.extend(reversed(linked))
        v,e,t=dependencies(topology,sorted(group))
        regions.append(LocalRegion(tuple(map(int,v)),tuple(map(int,e)),tuple(map(int,t)),len(v)>max_vertices))
    return tuple(regions)
