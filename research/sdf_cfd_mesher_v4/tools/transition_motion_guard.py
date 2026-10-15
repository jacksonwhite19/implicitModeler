"""Call-local changed-face screening with the existing exact collision owners.

The construction owner exclusively controls coordinates and topology. This is
an endpoint proposal screen, not a saved proof or continuous motion certificate.
Final independent transition admission remains mandatory.
"""
import time
import numpy as np
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
from production_collision_localization import classify


class MotionGuard:
    def __init__(self, points, faces, fixed_count, check=None):
        self.points=points
        self.faces=tuple(tuple(map(int,f)) for f in faces)
        self.fixed_count=fixed_count
        self.check=check or (lambda:None)
        if (type(points) is not np.ndarray or points.dtype!=np.dtype('<f8')
                or points.ndim!=2 or points.shape[1:]!=(3,) or not np.isfinite(points).all()
                or type(fixed_count) is not int or not 0<=fixed_count<=len(points)):
            raise ValueError('InvalidMotionGuardInputs')
        if any(len(f) not in (3,4) or len(set(f))!=len(f) or min(f)<0 or max(f)>=len(points) for f in self.faces):
            raise ValueError('InvalidMotionGuardFaces')
        if len({tuple(sorted(f)) for f in self.faces})!=len(self.faces):
            raise ValueError('DuplicateMotionGuardFace')
        # Preserve perimeter order. Native bilinear labels use (0,1,3,2).
        self.labels=np.asarray([f+(f[2],) if len(f)==3 else (f[0],f[1],f[3],f[2]) for f in self.faces],dtype=np.uint64).reshape(-1,4)
        geometry=points[self.labels]
        self.index=AABBIndex(geometry.min(axis=1),geometry.max(axis=1))
        self.postings={}
        for i,f in enumerate(self.faces):
            for v in f:
                if v>=fixed_count:self.postings.setdefault(v,[]).append(i)
        self.parent=np.full(len(self.index.nodes),-1,dtype=np.intp)
        self.leaf=np.empty(len(self.faces),dtype=np.intp)
        for i,(_,_,left,right) in enumerate(self.index.nodes):
            if right is None:self.leaf[left]=i
            else:self.parent[left]=i;self.parent[right]=i
        self.check()

    def affected(self, vertices):
        if any(not isinstance(v,(int,np.integer)) or isinstance(v,(bool,np.bool_)) for v in vertices):
            raise ValueError('InvalidMotionGuardSites')
        ids=tuple(map(int,vertices))
        if len(set(ids))!=len(ids) or any(v<self.fixed_count or v>=len(self.points) for v in ids):
            raise ValueError('InvalidMotionGuardSites')
        return sorted({i for v in ids for i in self.postings.get(v,())})

    def update(self, vertices):
        """Refit the existing inclusive tree after an exclusively owned edit."""
        active=self.affected(vertices)
        if not active:return active
        idx=self.index;geometry=self.points[self.labels[active]]
        if not np.isfinite(geometry).all():raise ValueError('NonfiniteMotionCoordinates')
        idx.lo[active]=geometry.min(axis=1);idx.hi[active]=geometry.max(axis=1)
        dirty=set(map(int,self.leaf[active]));pending=list(dirty)
        while pending:
            p=int(self.parent[pending.pop()])
            if p>=0 and p not in dirty:dirty.add(p);pending.append(p)
        # Existing median tree allocates every parent before either child.
        for i in sorted(dirty,reverse=True):
            _,_,left,right=idx.nodes[i]
            if right is None:lo=idx.lo[left].min(axis=0);hi=idx.hi[left].max(axis=0)
            else:lo=np.minimum(idx.nodes[left][0],idx.nodes[right][0]);hi=np.maximum(idx.nodes[left][1],idx.nodes[right][1])
            idx.nodes[i]=(lo,hi,left,right);idx._node_lo[i]=lo;idx._node_hi[i]=hi
        return active

    def screen(self, vertices, proposed):
        """Test all changed/changed and changed/current endpoint face pairs.

        Always restore coordinates and tree, including exceptions. A passing
        report may be consumed only synchronously by this construction owner.
        """
        started=time.perf_counter();self.affected(vertices);ids=np.asarray(vertices,dtype=np.intp)
        proposed=np.asarray(proposed,dtype='<f8')
        if proposed.shape!=(len(ids),3) or not np.isfinite(proposed).all():raise ValueError('InvalidMotionProposal')
        original=self.points[ids].copy()
        try:
            self.check();self.points[ids]=proposed;active=self.update(ids)
            pairs=set();active_set=set(active)
            for offset in range(0,len(active),256):
                self.check();block=active[offset:offset+256]
                for i, matches in zip(block,self.index.overlaps_batch(block)):
                    for j in matches:
                        if i!=j and not (j in active_set and j<i):pairs.add((min(i,j),max(i,j)))
            ordered=np.asarray(sorted(pairs),dtype=np.uint64).reshape(-1,2)
            if not len(ordered):return dict(passed=True,required_pairs=0,failures=[],seconds=time.perf_counter()-started)
            population=np.unique(ordered);local=np.searchsorted(population,ordered).astype(np.uint64)
            result=classify(self.points,self.labels[population],local)
            self.check()
            if np.any(result['codes']==11):raise ValueError('MotionPredicateInfrastructureFailure',result['exact_rows'])
            failed=np.flatnonzero((result['codes']==0)|(result['codes']>=10))
            failures=[dict(pair=ordered[i].tolist(),faces=[list(self.faces[j]) for j in ordered[i]],code=int(result['codes'][i])) for i in failed]
            return dict(passed=not len(failed),required_pairs=len(ordered),changed_faces=len(active),failures=failures,
                timings=result['timings'],seconds=time.perf_counter()-started,endpoint_only=True,scientific_mesh_acceptance=False)
        finally:
            self.points[ids]=original;self.update(ids)
