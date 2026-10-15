"""Staged generic vertex-star search and exact relative-edit preservation.

No publication authority. Filling preservation is conditional on the original
backend mesh being an embedded conforming filling. Quality uses V4 owners.
"""
from collections import defaultdict
from fractions import Fraction
import hashlib
import json
import time
import numpy as np


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def array_digest(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def position(value):
    return [float(v).hex() for v in value]


def sphere_link(cells, vertex):
    """Closed oriented triangulated sphere, including every vertex link."""
    faces=[]
    for c in cells:
        if (c['kind']!='tetrahedron' or len(c['vertices'])!=4 or
                len(set(c['vertices']))!=4 or vertex not in c['vertices']):
            raise ValueError('InvalidInteriorStarCell')
        from background_materialization import PATTERNS
        expected=[tuple(c['vertices'][i] for i in pat) for pat in PATTERNS]
        if len(c['faces'])!=4 or any(
                not any(tuple(f)==e[k:]+e[:k] for k in range(3))
                for f,e in zip(c['faces'],expected)):
            raise ValueError('ChangedTetFaceConvention')
        opposite=[f for f in c['faces'] if vertex not in f]
        if len(opposite)!=1 or len(opposite[0])!=3:
            raise ValueError('MissingStarLinkFace')
        faces.append(tuple(opposite[0]))
    if not faces or len({tuple(sorted(f)) for f in faces})!=len(faces):
        raise ValueError('DuplicateOrEmptyStarLink')
    edges=defaultdict(list); vertex_links=defaultdict(list)
    for j,f in enumerate(faces):
        if len(set(f))!=3:raise ValueError('DegenerateStarLink')
        for k,a in enumerate(f):
            b=f[(k+1)%3]; edges[tuple(sorted((a,b)))].append((j,a,b))
            vertex_links[a].append((f[k-1],f[(k+1)%3]))
    adjacency=defaultdict(set)
    for rows in edges.values():
        if len(rows)!=2 or rows[0][1:]!=rows[1][1:][::-1]:
            raise ValueError('OpenOrUnorientedStarLink')
        a,b=rows[0][0],rows[1][0];adjacency[a].add(b);adjacency[b].add(a)
    def connected(graph):
        seen=set();todo=[next(iter(graph))]
        while todo:
            v=todo.pop()
            if v not in seen:seen.add(v);todo.extend(graph[v]-seen)
        return len(seen)==len(graph)
    if not connected(adjacency):raise ValueError('DisconnectedStarLink')
    for pairs in vertex_links.values():
        graph=defaultdict(set);uses=defaultdict(int)
        for a,b in pairs:graph[a].add(b);graph[b].add(a);uses[a]+=1;uses[b]+=1
        if any(len(graph[a])!=2 or uses[a]!=2 for a in graph) or not connected(graph):
            raise ValueError('NoncyclicStarLinkVertex')
    if len(vertex_links)-len(edges)+len(faces)!=2:
        raise ValueError('NonSphereStarLink')
    return faces


def preserve_step(points, cells, vertex, after, fixed):
    """Exact before/after cones on an identical boundary; never reorient."""
    from background_mixed_geometry import exact_tetrahedron
    if vertex in fixed:raise ValueError('FixedRepairVertex')
    if np.shape(after)!=(3,) or not np.isfinite(after).all():
        raise ValueError('InvalidRepairPosition')
    faces=sphere_link(cells,vertex);old=points[vertex].copy()
    boundary=np.asarray(sorted(set(v for f in faces for v in f)),dtype=np.int64)
    boundary_bits=points[boundary].tobytes()
    def volumes():
        result=[]
        for c in cells:
            r=exact_tetrahedron(points,c)
            if r['jacobian_sign']<=0 or r['volume_sign']<=0:
                raise ValueError('NonpositiveRepairCone')
            result.append(Fraction(r['signed_volume']))
        return result
    before=volumes()
    try:
        points[vertex]=after
        new=volumes()
        if points[boundary].tobytes()!=boundary_bits:raise ValueError('ChangedStarBoundary')
        if sum(before)!=sum(new):raise ValueError('ChangedExactStarVolume')
    finally:points[vertex]=old
    return dict(vertex=int(vertex),cells=[c['id'] for c in cells],before=position(old),
                after=position(after),oriented_link_sha256=digest(faces),
                exact_star_volume=str(sum(before)),method='SingleVertexFixedOrientedSphereKernel/1')


def star_index(cells, wanted):
    """Enumerate ALL core cells, not just reported bad cells."""
    stars={v:[] for v in wanted}
    for c in cells:
        if c['kind']!='tetrahedron':continue
        for v in c['vertices']:
            if v in stars:stars[v].append(c)
    return stars


def verify_edits(original, candidate, cells, fixed, edits):
    """Replay only relative changes; this is not original-mesh qualification."""
    if original.shape!=candidate.shape or not np.isfinite(candidate).all():
        raise ValueError('InvalidRepairedArray')
    work=original.copy();stars=star_index(cells,{r['vertex'] for r in edits})
    for r in edits:
        v=r['vertex']
        if position(work[v])!=r['before']:raise ValueError('BrokenRepairSequence')
        after=np.asarray([float.fromhex(z) for z in r['after']])
        actual=preserve_step(work,stars[v],v,after,fixed)
        if actual!=r:raise ValueError('ChangedRepairWitness')
        work[v]=after
    if work.tobytes()!=candidate.tobytes():raise ValueError('UnwitnessedRepairChange')
    return dict(passed=True,edits=len(edits),original_points_sha256=array_digest(original),
                repaired_points_sha256=array_digest(candidate),sequence_sha256=digest(edits),
                premise='Original pinned backend core is an embedded conforming filling',
                scope='Relative fixed-connectivity interior-star edits only; no original collision/filling proof or publication authority')


class LocalRepair:
    """Existing three-coordinate SLSQP method; stock V4 metric operations."""
    def __init__(self,points,cells,owners,core_ids,fixed,failures):
        self.original=points;self.points=points.copy();self.cells=cells;self.owners=owners
        self.core_ids=set(core_ids);self.fixed=set(fixed);self.byid={c['id']:c for c in cells}
        if len(self.byid)!=len(cells) or not self.core_ids<=set(self.byid):
            raise ValueError('ChangedRepairCellPopulation')
        bad=set()
        for r in failures:
            if r['kind'] in ('cell_quality','exact_cell'):bad.add(r['id'])
            elif r['kind']=='face_quality':bad.update(r['owners'])
            else:raise ValueError('UnsupportedRepairFailure')
        self.wanted=sorted({v for i in bad if i in self.core_ids
                            for v in self.byid[i]['vertices']} - self.fixed)
        self.stars=star_index(cells,set(self.wanted));self.geometry={};self.edits=[];self.trials=[]

    def scope(self,star):
        ids={c['id'] for c in star};owners={tuple(sorted(f)):self.owners[tuple(sorted(f))]
                                          for c in star for f in c['faces']}
        needed={i for rows in owners.values() for i,_ in rows}
        if not ids<=self.core_ids or any(len(r) not in (1,2) for r in owners.values()):
            raise ValueError('InvalidRepairNeighbourClosure')
        for i in needed-ids:
            if i not in self.geometry:
                import local_termination_topology as t
                self.geometry[i]=t.geometry(self.points,self.byid[i])
        return owners,needed

    def constraints(self,star,owners,needed,volume_scale,*,prospective_outer=None):
        if prospective_outer is not None:
            from prism_hotpath_v2 import _fresh_call_custody
            # Include the actual one-hop neighbours used by the face metrics.
            custody=_fresh_call_custody(self.points,[self.byid[i] for i in sorted(needed)],owners,prospective_outer)
        from background_mixed_geometry import batch_geometry
        import local_termination_topology as t
        cq=t.q.legacy_modules()['component_quality'];det={}
        geom={i:self.geometry[i] for i in needed if i in self.geometry}
        fresh=batch_geometry(self.points,star,_determinants=det,_owners=owners,
                             _selected=tuple(True for _ in star))
        geom.update(fresh);vals=[]
        for j,c in enumerate(star):
            g=geom[c['id']]
            if c['id'] not in det:
                mask=np.array([len(owners[tuple(sorted(f))])==2 for f in c['faces']])
                S=g['areas'][mask];avg=np.linalg.norm(S,axis=1).mean()
                det[c['id']]=float(np.linalg.det((S/avg).T@(S/avg)))
            vals.extend((g['volume']/volume_scale[j],det[c['id']]/.0011-1,1000/g['aspect']-1))
        rows=[r for _,r in sorted(owners.items()) if len(r)==2]
        if rows:
            vs=self.points[np.asarray([r[0][1] for r in rows],dtype=np.int64)]
            S,C=cq.FG.face_area_centre_tri(vs);a=[geom[r[0][0]] for r in rows];b=[geom[r[1][0]] for r in rows]
            w,sk,ratio,_=cq.face_metrics(vs,S,C,np.array([g['centre'] for g in a]),
                                       np.array([g['centre'] for g in b]),np.array([g['volume'] for g in a]),
                                       np.array([g['volume'] for g in b]))
            vals.extend(w/.055-1);vals.extend(ratio/.01-1);vals.extend(3.63636-sk)
        value=np.asarray(vals)
        value=value if np.isfinite(value).all() else np.full_like(value,-1e30)
        if prospective_outer is not None:
            from prism_hotpath_v2 import _completed_incidence_geometry
            # Preserve the original caller's ordering: ordinary constraints
            # and their finite-value fallback first, then prospective D slots.
            # Only private areas freshly calculated for this trial are used.
            final=_completed_incidence_geometry(star,prospective_outer,owners,fresh)
            for j,r in enumerate(final['cells']):value[3*j+1]=r['determinant']/.0011-1
            custody()
        return value

    def run(self,*,seconds,maxiter,ftol):
        from scipy.optimize import minimize
        import local_termination_topology as t
        started=time.perf_counter();deadline=started+seconds
        class BudgetExpired(Exception):pass
        for v in self.wanted:
            if time.perf_counter()>=deadline:break
            tick=time.perf_counter();star=self.stars[v];sphere_link(star,v)
            owners,needed=self.scope(star);base=self.points[v].copy()
            neighbours=sorted({z for c in star for z in c['vertices']}-{v})
            scale=float(np.linalg.norm(self.points[neighbours]-base,axis=1).min())
            if not scale>0:raise ValueError('ZeroRepairScale')
            volume_scale=np.array([abs(t.geometry(self.points,c)['volume']) for c in star])
            if np.any(volume_scale<=0):raise ValueError('NonpositiveRepairInput')
            cache={}
            def values(z):
                if time.perf_counter()>=deadline:raise BudgetExpired()
                key=tuple(z)
                if key not in cache:
                    try:
                        self.points[v]=base+scale*z
                        cache[key]=self.constraints(star,owners,needed,volume_scale)
                    finally:self.points[v]=base
                return cache[key]
            try:before=values(np.zeros(3))
            except BudgetExpired:break
            if np.all(before>=0):continue
            try:
                fit=minimize(lambda z:float(z@z),np.zeros(3),method='SLSQP',bounds=[(-1,1)]*3,
                             constraints=[dict(type='ineq',fun=values)],options=dict(maxiter=maxiter,ftol=ftol))
                after=values(fit.x);target=base+scale*fit.x
            except BudgetExpired:break
            adopted=bool(np.all(after[before>=0]>=0) and ((after<0).sum()<(before<0).sum()
                         or ((after<0).sum()==(before<0).sum() and after.min()>before.min())))
            rejection=None
            if adopted:
                try:witness=preserve_step(self.points,star,v,target,self.fixed)
                except ValueError as e:adopted=False;rejection=str(e)
            if adopted:
                self.points[v]=target;self.edits.append(witness)
                # Invalidate every changed geometry; one-hop fixed neighbours
                # remain valid. Later stars can encounter these cells anew.
                for c in star:self.geometry.pop(c['id'],None)
            self.trials.append(dict(vertex=v,cells=len(star),adopted=adopted,rejection=rejection,
                failures_before=int((before<0).sum()),failures_after=int((after<0).sum()),
                minimum_before=float(before.min()),minimum_after=float(after.min()),
                solver_success=bool(fit.success),evaluations=len(cache),seconds=time.perf_counter()-tick))
            if len(self.trials)%50==0:
                print(json.dumps(dict(stage='local_star_repair',attempts=len(self.trials),
                    adopted=len(self.edits),seconds=time.perf_counter()-started)),flush=True)
        certificate=verify_edits(self.original,self.points,self.cells,self.fixed,self.edits)
        return dict(seconds=time.perf_counter()-started,eligible_vertices=len(self.wanted),
                    attempts=len(self.trials),edits=len(self.edits),relative_preservation=certificate,
                    trials=self.trials,witnesses=self.edits)
