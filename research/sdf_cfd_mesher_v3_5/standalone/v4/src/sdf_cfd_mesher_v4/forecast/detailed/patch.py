"""Stable core columns with a one-face-neighbor metric halo; no recursive owner closure."""
from dataclasses import dataclass
import numpy as np
from ...common.authority import frozen,array_hash,identity

class LocalityFailure(RuntimeError):pass
class IncompleteStencil(RuntimeError):pass

@dataclass(frozen=True)
class Patch:
    core_columns: np.ndarray
    columns: np.ndarray
    vertices: np.ndarray
    core_vertices: np.ndarray
    side_edges: np.ndarray
    topology_identity: str

    def record(self):
        return {name:getattr(self,name).tolist() for name in ('core_columns','columns','vertices','core_vertices','side_edges')}|dict(topology_identity=self.topology_identity)

def column_patch(topology,anchor,rings=0,*,max_columns=512):
    if rings<0 or not 0<=anchor<len(topology.triangles):raise ValueError('InvalidPatchRequest')
    core={int(anchor)}
    for _ in range(rings):
        core.update(int(c) for e in np.unique(topology.triangle_edges[sorted(core)]) for c in topology.edge_triangles.row(e))
        if len(core)>max_columns:raise LocalityFailure('CoreExceedsCallerBound')
    core=np.asarray(sorted(core),dtype=np.int64);sides=np.unique(topology.triangle_edges[core]);columns=set(map(int,core))
    for edge in sides:
        owners=topology.edge_triangles.row(edge)
        if len(owners)!=2:raise IncompleteStencil('SideRequiresTwoActualColumns')
        columns.update(map(int,owners))
    if len(columns)>max_columns or len(columns)==len(topology.triangles):raise LocalityFailure('HaloExceedsCallerBoundOrWholeShell')
    columns=np.asarray(sorted(columns),dtype=np.int64)
    return Patch(*(frozen(a,'<i8') for a in (core,columns,np.unique(topology.triangles[columns]),np.unique(topology.triangles[core]),sides)),topology.sha256)

def require_closure(patch,topology):
    if patch.topology_identity!=topology.sha256:raise IncompleteStencil('TopologyIdentityMismatch')
    expected=np.unique(topology.triangle_edges[patch.core_columns])
    if not np.array_equal(expected,patch.side_edges):raise IncompleteStencil('MissingCoreSide')
    columns=set(patch.columns)
    for e in expected:
        owners=topology.edge_triangles.row(e)
        if len(owners)!=2 or not set(owners).issubset(columns):raise IncompleteStencil('MissingSideNeighbor')
    if not set(np.unique(topology.triangles[patch.columns])).issubset(set(patch.vertices)):raise IncompleteStencil('MissingVertex')

def prisms(front,forecast,topology,patch,*,domain=None):
    require_closure(patch,topology)
    if domain is not None:domain.validate(front,forecast,topology)
    elif front.committed_layer!=0 or not np.array_equal(forecast.layers,np.arange(1,21)):raise ValueError('Task2RequiresFrozenTwentyLayerForecast')
    f=topology.triangles[patch.columns];local=np.searchsorted(patch.vertices,f)
    x=np.concatenate((front.coordinates[None,patch.vertices],forecast.vertex['coordinates'][:,patch.vertices]))
    # Layer-major, then stable source triangle order. Only local prisms exist.
    p=np.concatenate((x[:-1,local],x[1:,local]),axis=2)
    return x,p

def patch_key(front,forecast,topology,patch):
    return identity(dict(version='V4LocalDetailed/1',geometry=front.geometry_authority,surface=front.surface_authority,
                         topology=topology.sha256,patch=patch.record(),wall=array_hash(front.coordinates[patch.vertices]),
                         future=array_hash(forecast.vertex['coordinates'][:,patch.vertices])))
