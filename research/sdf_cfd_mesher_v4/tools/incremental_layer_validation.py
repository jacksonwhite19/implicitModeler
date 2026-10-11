"""Dependency-local reuse of the existing mixed-cell predicates.

No thresholds or geometric predicates are changed. A cache is private to a
verified committed state. Updates return a new cache, leaving rollback intact.
"""
from fractions import Fraction
import time
import numpy as np
import local_termination_topology as t
import cavity_volume_audit as volume


def key(face):
    return tuple(sorted(face))


class Topology:
    def __init__(self, cells, owners, boundary_keys=None):
        self.cells = cells if isinstance(cells,dict) else {c['id']: c for c in cells}
        self.owners = owners
        self.boundary_keys=({k for k,v in owners.items() if len(v)==1} if boundary_keys is None else boundary_keys)

    def update(self, replacements=(), retired=()):
        cells = self.cells.copy()
        owners = self.owners.copy()
        touched = set()
        replacements = list(replacements)
        removed = set(retired) | {c['id'] for c in replacements if c['id'] in cells}
        for ident in sorted(removed):
            old = cells.pop(ident)
            for face in old['faces']:
                k = key(face); touched.add(k)
                owners[k] = [r for r in owners[k] if r[0] != ident]
                if not owners[k]: del owners[k]
        for c in replacements:
            if c['id'] in cells: raise ValueError('DuplicateCell')
            cells[c['id']] = c
            for face in c['faces']:
                k = key(face); touched.add(k)
                rows = list(owners.get(k, ())) + [(c['id'], face)]
                if len(rows) > 2: raise ValueError('NonmanifoldCellFace')
                if len(rows) == 2 and not t.reverse_cycle(rows[0][1], rows[1][1]):
                    raise ValueError('InternalFaceOrientationMismatch')
                owners[k] = rows
        boundary=self.boundary_keys.copy()
        for k in touched:
            if len(owners.get(k,()))==1:boundary.add(k)
            else:boundary.discard(k)
        return Topology(cells, owners,boundary), touched

    def boundary(self, outer, wall):
        intended = {key(f) for f in outer} | {key(f) for f in wall}
        actual = self.boundary_keys
        if actual != intended: raise ValueError('BoundaryOwnershipMismatch')
        # Only the exposed front is traversed, never historical interior faces.
        links = t.vertex_links(outer)
        edges = {}
        for face in outer:
            for a,b in zip(face, face[1:]+face[:1]):
                edges.setdefault(tuple(sorted((a,b))), []).append((a,b))
        if not links['passed'] or any(len(v)!=2 or v[0]!=v[1][::-1] for v in edges.values()):
            raise ValueError('NonmanifoldOuterInterface')
        return links


class Quality:
    def __init__(self, cells, faces, exact, prospective):
        self.cells=cells; self.faces=faces; self.exact=exact; self.prospective=prospective

    @classmethod
    def receipts(cls, state, numeric_faces):
        from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
        n=len(state['wall']['vertices'])
        def source(v): return f'layer:{v//n}:source:{v%n}'
        mapped={face_key(tuple(source(v) for v in f)): key(f) for f in numeric_faces}
        cells={}; faces={}; exact={}; prospective={}
        for name, row in state['quality'].items():
            if name.startswith('cell:'):
                i=int(name[5:]); cells[i]=row['value']; exact[i]=row['exact']
            elif name.startswith('face:'): faces[mapped[name[5:]]]=row['value']
            elif name.startswith('final_incidence:'): prospective[int(name.split(':')[1])]=row['value']
        if len(cells)!=len(state['cells']) or len(prospective)!=len(cells):
            raise ValueError('MissingCommittedQualityAuthority')
        return cls(cells,faces,exact,prospective)

    def update(self, points, topology, changed_cells, touched_faces, outer, *, retain_geometry=False):
        """Changed cells are selected by the transaction, not by layer number."""
        tick=time.perf_counter(); changed=set(changed_cells)
        # Same validator, restricted owners. No historical face metric loop.
        local={k:topology.owners[k] for i in changed for f in topology.cells[i]['faces']
               if (k:=key(f)) in topology.owners}
        selected=[topology.cells[i] for i in sorted(changed)]
        from prism_hotpath_v2 import batch_quality
        geom={}
        q=batch_quality(points,selected + [topology.cells[i] for i in sorted(
            {i for rows in local.values() for i,_ in rows}-changed)], local,1,
            active_cells=changed,_geometry_out=geom)
        if not q['passed']: raise ValueError('IncrementalHardQualityFailure:'+str(q['failures'][:3]))
        cells=self.cells.copy(); faces=self.faces.copy(); exact=self.exact.copy(); prospective=self.prospective.copy()
        for i in self.cells.keys()-topology.cells.keys():
            cells.pop(i);exact.pop(i);prospective.pop(i)
        for f in touched_faces:
            if len(topology.owners.get(f,()))!=2:faces.pop(f,None)
        for row in q['cells']: cells[row['id']]=row
        for row in q['exact']: exact[row['id']]=row
        for row in q['faces']: faces[key(row['vertices'])]={k:v for k,v in row.items() if k!='vertices'}
        role_cells={i for f in touched_faces for i,_ in topology.owners.get(f,())}-changed
        for i in sorted(role_cells):
            c=topology.cells[i]; g=geom.get(i)
            # Retirement-only neighbours need not occur in the new-cell closure.
            if g is None:g=t.geometry(points,c);geom[i]=g
            mask=np.array([len(topology.owners[key(f)])==2 for f in c['faces']])
            S=g['areas'][mask]; avg=np.linalg.norm(S,axis=1).mean()
            determinant=float(np.linalg.det((S/avg).T@(S/avg)))
            if not determinant>=.0011: raise ValueError('ChangedRoleDeterminantFailure')
            cells[i]=dict(cells[i],determinant=determinant)
        # New/changed geometry needs prospective incidence. Unchanged geometry
        # may reuse only when the effective selected face mask is unchanged.
        interface={key(f) for f in outer}; prospective_recomputed=0
        for i in sorted(changed|role_cells):
            c=topology.cells[i]
            mask=tuple(len(topology.owners[key(f)])==2 or key(f) in interface for f in c['faces'])
            oldmask=getattr(self,'masks',{}).get(i)
            if i not in changed and oldmask==mask: continue
            g=geom.get(i)
            if g is None:g=t.geometry(points,c)
            S=g['areas'][np.array(mask)]; avg=np.linalg.norm(S,axis=1).mean()
            det=float(np.linalg.det((S/avg).T@(S/avg)))
            if not det>=.0011:raise ValueError('ProspectiveDeterminantFailure')
            prospective[i]=dict(id=i,column=c['column'],cell_type=c['kind'],determinant=det)
            prospective_recomputed+=1
        result=Quality(cells,faces,exact,prospective)
        if retain_geometry:result.geometry={i:geom[i] for i in changed|role_cells}
        result.masks=getattr(self,'masks',{}).copy()
        for i in self.cells.keys()-topology.cells.keys():result.masks.pop(i,None)
        for i in changed|role_cells:
            result.masks[i]=tuple(len(topology.owners[key(f)])==2 or key(f) in interface for f in topology.cells[i]['faces'])
        counts=dict(new_changed_cells=len(changed),historical_role_determinants=len(role_cells),
            exact_jacobians=len(q['exact']),face_metrics=len(q['faces']),prospective_determinants=prospective_recomputed,
            historical_exact_jacobians=0,seconds=time.perf_counter()-tick)
        return result,counts

    def bind_masks(self, topology, outer):
        interface={key(f) for f in outer}
        self.masks={i:tuple(len(topology.owners[key(f)])==2 or key(f) in interface for f in c['faces'])
                    for i,c in topology.cells.items()}


class ExactVolume:
    def __init__(self, points, cells, outer, wall, exact):
        self.total=sum((Fraction(r['signed_volume']) for r in exact.values()),Fraction())
        self.witness={c['id']:self.sample(points,c) for c in cells}
        self.boundary={tuple(f):self.contribution(points,f) for f in [tuple(reversed(f)) for f in wall]+list(outer)}
        self.boundary_total=sum(self.boundary.values(),Fraction())
        self.coordinates={v:tuple(points[v]) for f in self.boundary for v in f}

    @staticmethod
    def sample(points,c):
        ids=sorted(set(c['vertices']))
        return tuple(sum(Fraction(float(points[v,j])) for v in ids)/len(ids) for j in range(3))

    @staticmethod
    def contribution(points,f):
        p=[tuple(Fraction(float(v)) for v in points[i]) for i in f]
        return t.dot(p[0],t.cross(p[1],p[2]))/6

    def update(self, points, replacements, retired, old_exact, new_exact, outer, wall,*,sample=None,contribution=None):
        sample=self.sample if sample is None else sample
        contribution=self.contribution if contribution is None else contribution
        result=object.__new__(ExactVolume); result.total=self.total; result.witness=self.witness.copy()
        for i in set(retired)|{c['id'] for c in replacements if c['id'] in old_exact}:
            result.total-=Fraction(old_exact[i]['signed_volume']);result.witness.pop(i)
        for c in replacements:
            result.total+=Fraction(new_exact[c['id']]['signed_volume']);result.witness[c['id']]=sample(points,c)
        wanted={tuple(f) for f in [tuple(reversed(f)) for f in wall]+list(outer)}
        result.boundary=self.boundary.copy();result.boundary_total=self.boundary_total
        for f in self.boundary.keys()-wanted:result.boundary_total-=result.boundary.pop(f)
        for f in wanted-self.boundary.keys():
            result.boundary[f]=contribution(points,f);result.boundary_total+=result.boundary[f]
        for f in wanted & self.boundary.keys():
            if any(self.coordinates[v]!=tuple(points[v]) for v in f):
                result.boundary_total-=result.boundary[f]
                result.boundary[f]=contribution(points,f);result.boundary_total+=result.boundary[f]
        result.coordinates={v:tuple(points[v]) for f in wanted for v in f}
        if result.total!=result.boundary_total:raise ValueError('ExactVolumeBalanceFailure')
        return result

    def degree(self,points,outer,wall):
        # Deliberately GLOBAL. No old witness degree is assumed unchanged.
        result=volume.degree(points,[tuple(reversed(f)) for f in wall]+list(outer),
                             [self.witness[i] for i in sorted(self.witness)])
        if not result['passed']:raise ValueError('GlobalDegreeFailure')
        return result
