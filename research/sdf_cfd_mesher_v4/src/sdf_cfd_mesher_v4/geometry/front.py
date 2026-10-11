"""Read-only committed front and deterministic topology; no layer commit engine."""
from dataclasses import dataclass
import json
import numpy as np
from ..common.authority import frozen,array_hash,identity,reference,REPO,ROOT

@dataclass(frozen=True)
class CSR:
    offsets: np.ndarray
    values: np.ndarray
    def row(self,i):return self.values[self.offsets[i]:self.offsets[i+1]]

def csr(rows,values,n):
    order=np.lexsort((values,rows));r=np.asarray(rows)[order];v=np.asarray(values)[order]
    return CSR(frozen(np.r_[0,np.cumsum(np.bincount(r,minlength=n))],'<i8'),frozen(v,'<i8'))

@dataclass(frozen=True)
class Topology:
    triangles: np.ndarray
    edges: np.ndarray
    triangle_edges: np.ndarray
    vertex_neighbors: CSR
    vertex_edges: CSR
    vertex_triangles: CSR
    edge_triangles: CSR
    sha256: str

def build_topology(triangles,nvertices):
    f=frozen(triangles,'<i8')
    if f.ndim!=2 or f.shape[1]!=3 or np.any(f<0) or np.any(f>=nvertices):raise ValueError('InvalidTopologyReference')
    if np.any(np.diff(np.sort(f,axis=1),axis=1)==0):raise ValueError('RepeatedTriangleVertex')
    pairs=np.sort(f[:,[[0,1],[1,2],[2,0]]].reshape(-1,2),axis=1)
    edges,inverse=np.unique(pairs,axis=0,return_inverse=True)
    edgeids=np.repeat(np.arange(len(edges)),2);triids=np.repeat(np.arange(len(f)),3)
    vn=csr(edges.ravel(),edges[:,::-1].ravel(),nvertices)
    ve=csr(edges.ravel(),edgeids,nvertices);vt=csr(f.ravel(),triids,nvertices)
    et=csr(inverse,triids,len(edges))
    return Topology(f,frozen(edges,'<i8'),frozen(inverse.reshape(-1,3),'<i8'),vn,ve,vt,et,array_hash(f))

@dataclass(frozen=True)
class CommittedFrontState:
    committed_layer: int
    coordinates: np.ndarray
    vertex_ids: tuple[str,...]
    triangles: np.ndarray
    source_ancestry: tuple[tuple[int,...],...]
    target_distances: np.ndarray
    geometry_authority: str
    surface_authority: str
    coordinate_hash: str
    topology_hash: str

    @classmethod
    def create(cls,layer,coordinates,ids,triangles,ancestry,targets,geometry,surface):
        x=frozen(coordinates,'<f8');f=frozen(triangles,'<i8');d=frozen(targets,'<f8');ids=tuple(ids)
        if x.ndim!=2 or x.shape[1]!=3 or d.shape!=(len(x),):raise ValueError('FrontShapeMismatch')
        if layer<0 or len(ids)!=len(x) or len(set(ids))!=len(ids) or len(ancestry)!=len(x):raise ValueError('InvalidFrontIdentity')
        if not np.isfinite(x).all() or not np.isfinite(d).all():raise ValueError('NonfiniteFront')
        if f.ndim!=2 or f.shape[1]!=3 or np.any(f<0) or np.any(f>=len(x)):raise ValueError('InvalidTopologyReference')
        return cls(layer,x,ids,f,tuple(tuple(a) for a in ancestry),d,geometry,surface,array_hash(x),array_hash(f))

def load_sf2_wall():
    p,surface=reference('SF2 surface');_,geometry=reference('common geometry')
    with np.load(p) as z:x=z['wall'];f=z['faces'];refs=z['refs']
    # Persistent V3 IDs and sheet ancestry, referenced in place.
    binding=json.loads((ROOT/'authorities/TASK1_SOURCE_BINDING.json').read_text())
    r=next(r for r in binding['files'] if r['path'].endswith('/input_state/state.json'))
    from ..common.authority import file_hash
    q=REPO/r['path'];assert file_hash(q)==r['sha256']
    state=json.loads(q.read_text());ids=state['vertex_order']
    ancestry=[set() for _ in x]
    for face,ref in zip(f,refs):
        for v in face:ancestry[v].add(int(ref))
    if x.shape!=(53153,3) or f.shape!=(106302,3):raise ValueError('SF2PopulationMismatch')
    front=CommittedFrontState.create(0,x,ids,f,[sorted(a) for a in ancestry],np.zeros(len(x)),geometry,surface)
    return front
