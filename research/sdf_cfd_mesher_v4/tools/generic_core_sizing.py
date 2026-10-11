"""Backend-independent terminal-prism sizing. All lengths are metres.

No production growth or cap defaults. Geometry and index arrays are immutable;
query state is local. Optional constraints must be positive, finite and continuous
if continuity of the composed field is required. Only the base transition has
the unconditional g-Lipschitz guarantee.
"""
from dataclasses import dataclass
from typing import Protocol
import heapq
import math
import hashlib
import json
from pathlib import Path
import numpy as np


class SizingConstraint(Protocol):
    def evaluate(self, position) -> float: ...


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name)
    return value


def equal_volume_seed(volume):
    volume = np.asarray(volume, dtype=float)
    if not np.all(np.isfinite(volume)) or np.any(volume <= 0):
        raise ValueError('PositiveTerminalVolumeRequired')
    # Factor outside cbrt avoids intermediate overflow.
    return np.cbrt(volume) * np.cbrt(6 * math.sqrt(2))


def triangle_distances(point, triangles):
    """Euclidean distance to closed triangles, including edges and vertices.

    Identical row-local arithmetic is used by brute force and BVH leaves.
    Degenerate triangles are rejected by the field constructor.
    """
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac = b-a, c-a
    n = np.cross(ab, ac)
    n2 = np.einsum('ij,ij->i', n, n)
    ap = point-a
    plane = np.einsum('ij,ij->i', ap, n)
    projected = point - (plane/n2)[:, None]*n
    inside = np.ones(len(a), dtype=bool)
    best = np.full(len(a), np.inf)
    for u, v in ((a,b), (b,c), (c,a)):
        edge = v-u
        side = np.einsum('ij,ij->i', np.cross(edge, projected-u), n)
        inside &= side >= 0
        t = np.clip(np.einsum('ij,ij->i', point-u, edge) /
                    np.einsum('ij,ij->i', edge, edge), 0, 1)
        delta = point-(u+t[:,None]*edge)
        best = np.minimum(best, np.einsum('ij,ij->i', delta, delta))
    best = np.where(inside, plane*plane/n2, best)
    return np.sqrt(np.maximum(best, 0))


@dataclass(frozen=True)
class Node:
    lo: np.ndarray
    hi: np.ndarray
    seed: float
    left: int
    right: int
    ids: np.ndarray


class CoreSizingField:
    def __init__(self, triangles, seeds, *, g, h_max, constraints=(), leaf_size=16):
        self.g = positive(g, 'PositiveExplicitGrowthRequired')
        self.h_max = positive(h_max, 'PositiveExplicitMaximumRequired')
        t = np.array(triangles, dtype=float, copy=True)
        s = np.array(seeds, dtype=float, copy=True)
        if (t.ndim != 3 or t.shape[1:] != (3,3) or not len(t) or
                s.shape != (len(t),) or not np.all(np.isfinite(t)) or
                not np.all(np.isfinite(s)) or np.any(s <= 0)):
            raise ValueError('InvalidSeedGeometry')
        n = np.cross(t[:,1]-t[:,0], t[:,2]-t[:,0])
        if np.any(np.einsum('ij,ij->i',n,n) <= 0) or not np.all(np.isfinite(n)):
            raise ValueError('DegenerateSeedTriangle')
        if not isinstance(leaf_size,int) or leaf_size < 1:
            raise ValueError('LeafSize')
        t.flags.writeable = False; s.flags.writeable = False
        self.triangles, self.seeds = t,s
        self.constraints = tuple(constraints)
        nodes = []
        centers = t.mean(axis=1)
        def build(ids):
            slot = len(nodes); nodes.append(None)
            lo, hi = t[ids].min(axis=(0,1)), t[ids].max(axis=(0,1))
            lo.flags.writeable = False; hi.flags.writeable = False
            left = right = -1
            if len(ids) > leaf_size:
                axis = int(np.argmax(hi-lo))
                order = np.lexsort((ids, centers[ids,axis]))
                ordered = ids[order]; mid = len(ids)//2
                left, right = build(ordered[:mid]), build(ordered[mid:])
                kept = np.empty(0, dtype=np.int64)
            else:
                kept = np.sort(ids)
            kept.flags.writeable = False
            nodes[slot] = Node(lo,hi,float(s[ids].min()),left,right,kept)
            return slot
        build(np.arange(len(t),dtype=np.int64))
        self.nodes = tuple(nodes)
        self.array_bytes = t.nbytes+s.nbytes+sum(n.lo.nbytes+n.hi.nbytes+n.ids.nbytes for n in nodes)

    @staticmethod
    def point(position):
        p = np.asarray(position,dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError('FinitePositionRequired')
        return p

    def values(self, p, ids):
        d = triangle_distances(p,self.triangles[ids])
        with np.errstate(over='ignore'):
            return np.minimum(self.h_max,self.seeds[ids]+self.g*d)

    def lower_bound(self, p, node):
        # Outward rounding and a scale-dependent roundoff allowance avoid
        # pruning boundary ties in binary64 distance evaluation.
        delta = np.maximum(np.maximum(node.lo-p,p-node.hi),0)
        distance = math.hypot(*delta)
        scale = max(1.,float(np.max(np.abs(p))),float(np.max(np.abs(node.lo))),
                    float(np.max(np.abs(node.hi))))
        distance = max(0.,distance-128*np.finfo(float).eps*scale)
        value = node.seed+self.g*distance
        return np.nextafter(value,-math.inf)

    def evaluate_with_work(self, position):
        p = self.point(position)
        best = self.h_max; visited = tested = 0
        queue = [(self.lower_bound(p,self.nodes[0]),0)]
        while queue:
            bound, i = heapq.heappop(queue)
            if bound > best:
                continue
            node = self.nodes[i]; visited += 1
            if node.left < 0:
                best = min(best,float(self.values(p,node.ids).min()))
                tested += len(node.ids)
            else:
                for child in (node.left,node.right):
                    b = self.lower_bound(p,self.nodes[child])
                    if b <= best:
                        heapq.heappush(queue,(b,child))
        for constraint in self.constraints:
            best = min(best,positive(constraint.evaluate(p),'InvalidRefinementConstraint'))
        return best, dict(nodes=visited,triangles=tested)

    def evaluate(self, position):
        return self.evaluate_with_work(position)[0]

    def brute_force(self, position):
        p = self.point(position)
        result = float(self.values(p,np.arange(len(self.seeds))).min())
        for constraint in self.constraints:
            result = min(result,positive(constraint.evaluate(p),'InvalidRefinementConstraint'))
        return result


class GmshSizingAdapter:
    """The callback replaces lc. Gmsh remains a consumer of sizing policy."""
    def __init__(self, field):
        self.field = field

    def __call__(self, dim, tag, x, y, z, lc):
        return self.field.evaluate((x,y,z))

    def install(self, gmsh):
        for name,value in {'Mesh.MeshSizeFromPoints':0,
                           'Mesh.MeshSizeFromCurvature':0,
                           'Mesh.MeshSizeExtendFromBoundary':0,
                           'Mesh.MeshSizeFactor':1,
                           'Mesh.MeshSizeMin':0,
                           'Mesh.MeshSizeMax':self.field.h_max}.items():
            gmsh.option.setNumber(name,value)
            if gmsh.option.getNumber(name) != value:
                raise RuntimeError('GmshSizingOption:'+name)
        gmsh.model.mesh.setSizeCallback(self)


def add_analytic_box(gmsh, bounds):
    """Six exact planes; no subdivision count or mesh size is supplied."""
    b = np.asarray(bounds,float)
    if b.shape != (3,2) or not np.all(np.isfinite(b)) or np.any(b[:,0]>=b[:,1]):
        raise ValueError('BoxBounds')
    return gmsh.model.occ.addBox(*b[:,0],*(b[:,1]-b[:,0]))


def open_architecture(path, expected_identity):
    """Verify compact sizing evidence against a caller-held authority anchor.

    This authenticates a seed definition, NOT production g/h_max. No field is
    instantiated and no meshing starts here. Root paths are authenticated data.
    """
    path=Path(path)
    record=json.loads(path.read_text())
    payload={k:v for k,v in record.items() if k!='identity'}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    if record.get('identity')!=expected_identity or digest!=expected_identity:
        raise ValueError('SizingAuthorityIdentity')
    if record['production_parameters']!={'g':None,'h_max':None}:
        raise ValueError('UnexpectedProductionParameterBinding')
    for name,expected in record['files'].items():
        p=(path.parent/name).resolve()
        actual=hashlib.sha256(p.read_bytes()).hexdigest()
        if actual!=expected:
            raise ValueError('SizingComponentCorruption:'+name)
    return record
