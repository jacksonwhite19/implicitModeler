"""Immutable accepted-front input for the existing staged prism selectors.

This module encodes coordinates and their authority. It selects no increments.
The scalar meaning is explicit; compatible feature vectors are never normalized.
"""
from pathlib import Path
import hashlib
import json
import numpy as np


def array_hash(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


class AcceptedFrontState:
    schema = 'V3.AcceptedFrontState/1'

    def __init__(self, wall, first_front, faces, vertex_order, source_normals,
                 incidents, later_vectors, first_height, scalar_quantity,
                 *, height_tolerance=1e-12, ownership='C_prospective_final_fluid'):
        self.wall = np.array(wall, dtype='<f8', copy=True)
        self.first = np.array(first_front, dtype='<f8', copy=True)
        self.faces = np.array(faces, dtype='<i8', copy=True)
        self.normals = np.array(source_normals, dtype='<f8', copy=True)
        self.vectors = np.array(later_vectors, dtype='<f8', copy=True)
        self.vertex_order = tuple(vertex_order)
        self.incidents = tuple(tuple(sorted(s)) for s in incidents)
        self.first_height = float(first_height)
        self.height_tolerance = float(height_tolerance)
        self.scalar_quantity = scalar_quantity
        self.ownership = ownership
        n = len(self.wall)
        if self.wall.shape != (n, 3) or self.first.shape != self.wall.shape:
            raise ValueError('AcceptedFrontShapeMismatch')
        if len(self.vertex_order) != n or len(set(self.vertex_order)) != n:
            raise ValueError('AmbiguousPersistentVertexOrder')
        if self.faces.ndim != 2 or self.faces.shape[1] != 3 or self.faces.min() < 0 or self.faces.max() >= n:
            raise ValueError('InvalidWallTopology')
        if self.vectors.ndim != 3 or self.vectors.shape[1:] != (n, 3):
            raise ValueError('InvalidLaterVectorField')
        if self.normals.ndim != 2 or self.normals.shape[1] != 3 or len(self.incidents) != n:
            raise ValueError('InvalidIncidentSheetAuthority')
        if not all(np.isfinite(a).all() for a in [self.wall,self.first,self.normals,self.vectors]):
            raise ValueError('NonfiniteAcceptedState')
        if not np.allclose(np.linalg.norm(self.normals,axis=1),1,atol=1e-13,rtol=0):
            raise ValueError('NonunitSourceSheetNormal')
        if not all(s and min(s)>=0 and max(s)<len(self.normals) for s in self.incidents):
            raise ValueError('IncompleteIncidentSheetAuthority')
        self.pairs=np.array([(v,s) for v,ss in enumerate(self.incidents) for s in ss],dtype='<i8')
        heights=np.einsum('ij,ij->i',self.normals[self.pairs[:,1]],(self.first-self.wall)[self.pairs[:,0]])
        if self.first_height<=0 or not 0<self.height_tolerance<=1e-12 or np.max(abs(heights-self.first_height))>self.height_tolerance:
            raise ValueError('AcceptedFirstHeightAuthorityMismatch')
        if scalar_quantity not in ['path_length_m','metres_times_declared_dimensionless_vector']:
            raise ValueError('AmbiguousScalarIncrementMeaning')
        if scalar_quantity=='path_length_m' and not np.allclose(np.linalg.norm(self.vectors,axis=2),1,atol=1e-13,rtol=0):
            raise ValueError('PathLengthRequiresUnitLaterVectors')
        if ownership not in ['B_final_isolated_shell','C_prospective_final_fluid']:
            raise ValueError('AmbiguousPlannerOwnership')
        for a in [self.wall,self.first,self.faces,self.normals,self.vectors,self.pairs]:a.flags.writeable=False
        self.first_hash=array_hash(self.first)
        self._cache=None

    def positions(self, increments, wall=None, faces=None):
        h=np.asarray(increments,dtype='<f8')
        if h.shape!=(len(self.wall),len(self.vectors)+1) or not np.isfinite(h).all() or np.any(h<=0):
            raise ValueError('InvalidScalarSchedule')
        if not np.all(h[:,0]==self.first_height):raise ValueError('ImmutableFirstIncrementChanged')
        if wall is not None and not np.array_equal(wall,self.wall):raise ValueError('WallAuthorityMismatch')
        if faces is not None and not np.array_equal(faces,self.faces):raise ValueError('WallTopologyMismatch')
        if array_hash(self.first)!=self.first_hash:raise ValueError('AcceptedFrontMutated')
        if self._cache is not None and np.array_equal(h,self._cache[0]):return self._cache[1]
        x=np.empty((h.shape[1]+1,len(self.wall),3),dtype='<f8');x[0]=self.wall;x[1]=self.first
        for k in range(1,h.shape[1]):x[k+1]=x[k]+h[:,k,None]*self.vectors[k-1]
        x.flags.writeable=False;self._cache=(h.copy(),x)
        return x

    def determinant(self, prisms, layer, predicate_module):
        if self.ownership=='C_prospective_final_fluid':
            return predicate_module._foundation_determinant(prisms,lower_cap_is_uncoupled_wall=(layer==0))
        vectors=predicate_module._face_vectors(prisms);mask=np.ones(5);mask[0]=layer>0;mask[1]=layer<len(self.vectors)
        avg=np.sum(np.linalg.norm(vectors,axis=2)*mask,axis=1)/sum(mask)
        scaled=vectors/avg[:,None,None]
        return predicate_module._det_symm(np.einsum('nfi,nfj,f->nij',scaled,scaled,mask))

    def save(self, directory, increments):
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
        x=self.positions(increments);arrays=dict(wall=self.wall,first_front=self.first,faces=self.faces,source_normals=self.normals,later_vectors=self.vectors,increments=np.asarray(increments,dtype='<f8'),positions=x,delta=np.subtract(x[1:],x[:-1],dtype=np.float64))
        for k,a in arrays.items():np.save(directory/(k+'.npy'),a,allow_pickle=False)
        m=dict(schema=self.schema,vertex_order=self.vertex_order,incidents=self.incidents,first_height=self.first_height,height_tolerance=self.height_tolerance,scalar_quantity=self.scalar_quantity,ownership=self.ownership,first_hash=self.first_hash,arrays={k:array_hash(a) for k,a in arrays.items()},arithmetic='X1 immutable; Xk=fl64(Xprev+scalar*declared_vector); delta=fl64(Xk-Xprev). No normalization.')
        (directory/'state.json').write_text(json.dumps(m,sort_keys=True,indent=2)+'\n')

    @classmethod
    def load(cls,directory):
        directory=Path(directory);m=json.loads((directory/'state.json').read_text())
        if m.get('schema')!=cls.schema:raise ValueError('UnsupportedAcceptedStateSchema')
        a={k:np.load(directory/(k+'.npy'),allow_pickle=False) for k in m['arrays']}
        if any(array_hash(v)!=m['arrays'][k] for k,v in a.items()):raise ValueError('AcceptedStateHashMismatch')
        s=cls(a['wall'],a['first_front'],a['faces'],m['vertex_order'],a['source_normals'],m['incidents'],a['later_vectors'],m['first_height'],m['scalar_quantity'],height_tolerance=m['height_tolerance'],ownership=m['ownership'])
        x=s.positions(a['increments'])
        if s.first_hash!=m['first_hash'] or not np.array_equal(x,a['positions']) or not np.array_equal(np.subtract(x[1:],x[:-1]),a['delta']):raise ValueError('AcceptedStateReconstructionMismatch')
        return s,a['increments']
