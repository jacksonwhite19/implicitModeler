"""Opt-in exact implementation of the sealed sizing field; no policy changes."""
import math
import numpy as np
from generic_core_sizing import CoreSizingField


def cross3(a, b):
    # Same three multiply/subtract expressions as numpy.cross, no contraction.
    out = np.empty_like(a)
    out[:, 0] = a[:, 1]*b[:, 2] - a[:, 2]*b[:, 1]
    out[:, 1] = a[:, 2]*b[:, 0] - a[:, 0]*b[:, 2]
    out[:, 2] = a[:, 0]*b[:, 1] - a[:, 1]*b[:, 0]
    return out


class ExactCachedSizingField(CoreSizingField):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        a, b, c = self.triangles[:, 0], self.triangles[:, 1], self.triangles[:, 2]
        self.normals = np.cross(b-a, c-a)
        self.normal2 = np.einsum('ij,ij->i', self.normals, self.normals)
        self.edges = tuple(v-u for u, v in ((a,b),(b,c),(c,a)))
        self.edge2 = tuple(np.einsum('ij,ij->i', e, e) for e in self.edges)
        for array in (self.normals, self.normal2, *self.edges, *self.edge2):
            array.flags.writeable = False
            self.array_bytes += array.nbytes
        self.bounds = {id(n): (tuple(map(float,n.lo)), tuple(map(float,n.hi)),
            max(1., *map(abs,n.lo), *map(abs,n.hi))) for n in self.nodes}

    def lower_bound(self, p, node):
        lo, hi, scale = self.bounds[id(node)]
        x,y,z = map(float,p)
        distance = math.hypot(max(lo[0]-x,x-hi[0],0.),
                              max(lo[1]-y,y-hi[1],0.),
                              max(lo[2]-z,z-hi[2],0.))
        scale = max(scale,abs(x),abs(y),abs(z))
        distance = max(0.,distance-128*np.finfo(float).eps*scale)
        return math.nextafter(node.seed+self.g*distance,-math.inf)

    def values(self, p, ids):
        a = self.triangles[ids,0]
        n = self.normals[ids]; n2 = self.normal2[ids]
        plane = np.einsum('ij,ij->i',p-a,n)
        projected = p-(plane/n2)[:,None]*n
        inside = np.ones(len(ids),dtype=bool)
        best = np.full(len(ids),np.inf)
        for i in range(3):
            u = self.triangles[ids,i]; edge = self.edges[i][ids]
            side = np.einsum('ij,ij->i',cross3(edge,projected-u),n)
            inside &= side >= 0
            t = np.clip(np.einsum('ij,ij->i',p-u,edge)/self.edge2[i][ids],0,1)
            delta = p-(u+t[:,None]*edge)
            best = np.minimum(best,np.einsum('ij,ij->i',delta,delta))
        best = np.where(inside,plane*plane/n2,best)
        d = np.sqrt(np.maximum(best,0))
        with np.errstate(over='ignore'):
            return np.minimum(self.h_max,self.seeds[ids]+self.g*d)
