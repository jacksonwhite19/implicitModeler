"""Elementary current-core transactions; V4 owns all numeric acceptance.

Combinatorics reuse the reviewed V3/V4 cavity studies. Working IDs are unique
live core IDs. No full tetrahedron-uniqueness scan or full owner-map copy occurs
per trial. Only actual affected face owners and one-hop geometry are materialized.
"""
from collections import defaultdict,Counter
from itertools import combinations
import numpy as np
from background_materialization import build
from prism_hotpath_v2 import reverse_cycle,batch_quality
import local_termination_topology as t

def key(row):return tuple(sorted(map(int,row)))
def parity(row):return -1 if sum(row[i]>row[j] for i in range(len(row)) for j in range(i+1,len(row)))%2 else 1
def boundary(cells):
    uses=defaultdict(list)
    for c in cells:
        for f in c['faces']:uses[key(f)].append((c['id'],tuple(f)))
    if any(len(rows)>2 or len(rows)==2 and not reverse_cycle(rows[0][1],rows[1][1]) for rows in uses.values()):raise ValueError('InvalidOrientedCavityIncidence')
    faces=[rows[0][1] for rows in uses.values() if len(rows)==1]
    edges=Counter(key((f[i],f[(i+1)%3])) for f in faces for i in range(3))
    if not faces or any(n!=2 for n in edges.values()):raise ValueError('NonmanifoldCavityBoundary')
    return {key(f):parity(f) for f in faces},faces,[rows[0][1] for rows in uses.values()]
def orient(points,rows):
    out=[]
    for row in rows:
        row=list(map(int,row));cell=build(np.array([row],dtype=np.int64),0)[0];exact=t.exact_cell(points,cell)
        if exact['volume_sign']==0:raise ValueError('DegenerateReplacement')
        if exact['volume_sign']<0:row[0],row[1]=row[1],row[0]
        cell=build(np.array([row],dtype=np.int64),0)[0];exact=t.exact_cell(points,cell)
        if min(exact['volume_sign'],exact['jacobian_sign'])<=0:raise ValueError('InvalidReplacementOrientation')
        out.append(row)
    return out
def cavity_geometry(points,cells):
    from native_mixed_face_certificate import certify
    from rolling_fast_state import verify_fresh_coverage
    from cavity_volume_audit import audit
    chain,outer,faces=boundary(cells);ids=np.array(sorted({v for c in cells for v in c['vertices']}),dtype=np.int64)
    lookup={int(v):i for i,v in enumerate(ids)};local=points[ids]
    compact=build(np.array([[lookup[v] for v in c['vertices']] for c in cells],dtype=np.int64),0)
    unique=[tuple(lookup[v] for v in f) for f in faces];outside=[tuple(lookup[v] for v in f) for f in outer]
    cert=certify(local,unique,range(len(unique)),workers=4);verify_fresh_coverage(local,unique,range(len(unique)),cert)
    volume=audit(local,compact,outside,[])
    if not (cert['passed'] and volume['positive_cells'] and volume['exact_volume_balance'] and volume['interior_degree']['passed']):raise ValueError('CoreCavityRequiredAdmissionFailed')
    return dict(passed=True,boundary_chain=[dict(face=list(f),sign=s) for f,s in sorted(chain.items())],
        collision=dict(passed=True,required_pairs=cert['required_pairs'],proof_hash=cert['proof_hash'],independent_coverage_passed=True),
        filling={k:v for k,v in volume.items() if k!='exact_cells'},scope='Closed edit cavity only, relative to the pinned backend embedding premise.')
class CoreTopology:
    def __init__(self,points,cells,owners,core_ids,far_faces):
        self.points=points;self.byid={c['id']:c for c in cells};self.owners=owners;self.core_ids=set(core_ids)
        self.far={key(f) for f in far_faces}
        # Neighbour references can extend beyond a materialized local closure.
        # An external live ID must never alias a newly created cell.
        self.next_id=max(max(self.byid),max(i for rows in owners.values() for i,_ in rows))+1
        if len(self.byid)!=len(cells) or not self.core_ids<=self.byid.keys() or any(self.byid[i]['kind']!='tetrahedron' for i in self.core_ids):raise ValueError('InvalidCurrentCorePopulation')
    def edge_star(self,seed,edge,*,_expanded=False):
        pending=[seed];seen=set();edge=set(edge)
        while pending:
            i=pending.pop()
            if i in seen:continue
            seen.add(i)
            if not _expanded and len(seen)>4:return None
            for f in self.byid[i]['faces']:
                if edge<=set(f):
                    rows=self.owners[key(f)]
                    if len(rows)!=2 or any(z not in self.core_ids for z,_ in rows):return None
                    pending.extend(z for z,_ in rows if z not in seen)
        return sorted(seen)
    def inventory(self,seed):
        if seed not in self.core_ids:return []
        cell=self.byid[seed];out=[]
        for face in sorted(cell['faces'],key=key):
            rows=self.owners[key(face)];ids=sorted(i for i,_ in rows)
            if len(ids)==2 and all(i in self.core_ids for i in ids):
                u,v=[next(iter(set(self.byid[i]['vertices'])-set(face))) for i in ids];a,b,c=key(face)
                out.append(dict(operation='2to3',remove=ids,insert=[(u,v,a,b),(u,v,b,c),(u,v,c,a)]))
        for edge in combinations(sorted(cell['vertices']),2):
            ids=self.edge_star(seed,edge)
            if not ids or len(ids) not in (3,4):continue
            ring=sorted({v for i in ids for v in self.byid[i]['vertices']}-set(edge));u,v=edge
            if len(ids)==3 and len(ring)==3:
                a,b,c=ring;out.append(dict(operation='3to2',remove=ids,insert=[(a,b,c,u),(a,c,b,v)]))
            elif len(ids)==4 and len(ring)==4:
                pairs={key(set(self.byid[i]['vertices'])-set(edge)) for i in ids}
                for a,c in combinations(ring,2):
                    if key((a,c)) in pairs:continue
                    b,d=sorted(set(ring)-{a,c});out.append(dict(operation='4to4',remove=ids,insert=[(a,c,u,b),(a,c,b,v),(a,c,v,d),(a,c,d,u)]))
        unique={(row['operation'],tuple(row['remove']),tuple(sorted(map(key,row['insert'])))):row for row in out}
        return [unique[k] for k in sorted(unique)]
    def expanded_inventory(self,seed,check):
        """Complete larger edge stars and their protected one-hop cavities.

        Qhull is a proposer only. Exact oriented boundary/actual-owner quality
        and the existing independent cavity predicates still decide adoption.
        No point insertion, boundary motion, jitter or geometry authority.
        """
        from scipy.spatial import Delaunay,QhullError
        if seed not in self.core_ids:return
        cell=self.byid[seed];patches={tuple(sorted({seed}|{i for f in cell['faces'] for i,_ in self.owners[key(f)] if i in self.core_ids}))}
        def triangulations(poly):
            check()
            if len(poly)<3:
                yield [];return
            for j in range(1,len(poly)-1):
                for left in triangulations(poly[:j+1]):
                    for right in triangulations(poly[j:]):
                        check();yield left+right+[(poly[0],poly[j],poly[-1])]
        for edge in combinations(sorted(cell['vertices']),2):
            check();ids=self.edge_star(seed,edge,_expanded=True)
            if not ids:continue
            patches.add(tuple(sorted(set(ids)|{i for z in ids for f in self.byid[z]['faces'] for i,_ in self.owners[key(f)] if i in self.core_ids})))
            if len(ids)<=4:continue
            pairs={key(set(self.byid[i]['vertices'])-set(edge)) for i in ids};adj=defaultdict(set)
            for a,b in pairs:adj[a].add(b);adj[b].add(a)
            if len(adj)!=len(ids) or any(len(row)!=2 for row in adj.values()):continue
            first=min(adj);poly=[first,min(adj[first])]
            while len(poly)<len(adj):
                nextrow=adj[poly[-1]]-{poly[-2]}
                if len(nextrow)!=1:break
                v=next(iter(nextrow))
                if v in poly:break
                poly.append(v)
            if len(poly)!=len(adj) or first not in adj[poly[-1]]:continue
            u,v=edge
            for triangles in triangulations(poly):
                check();rows=[row for a,b,c in triangles for row in ((u,a,b,c),(v,a,c,b))]
                yield dict(operation='closed_edge_retriangulation',remove=ids,insert=rows,edge=list(edge),ring_vertices=poly)
        for ids in sorted(patches,key=lambda row:(len(row),row)):
            check();vertices=np.asarray(sorted({v for i in ids for v in self.byid[i]['vertices']}),dtype='<i8')
            try:mesh=Delaunay(self.points[vertices])
            except QhullError as exc:
                yield dict(operation='local_delaunay',generation_rejection='LocalDelaunayRepresentation',detail=str(exc),remove=list(ids));continue
            check();rows=vertices[mesh.simplices].tolist()
            yield dict(operation='local_delaunay',remove=list(ids),insert=rows,
                scope='Seed/full-edge-star plus complete one-face-neighbour patch; unchanged exact cavity boundary required.')
    def stamp(self,proposal):
        faces={key(f) for i in proposal['remove'] for f in self.byid[i]['faces']}
        faces.update(key(f) for row in proposal['insert'] for f in combinations(row,3))
        return (proposal['operation'],tuple(proposal['remove']),tuple(sorted(map(key,proposal['insert']))),
            tuple((f,tuple(sorted(i for i,_ in self.owners.get(f,[])))) for f in sorted(faces)))
    def view(self,proposal):
        remove=set(proposal['remove'])
        if not remove<=self.core_ids:raise ValueError('InvalidCoreRemovalScope')
        old=[self.byid[i] for i in sorted(remove)];rows=orient(self.points,proposal['insert'])
        if {v for row in rows for v in row}!={v for c in old for v in c['vertices']}:raise ValueError('ChangedVertexSet')
        if len(set(map(key,rows)))!=len(rows):raise ValueError('DuplicateReplacement')
        new=build(np.asarray(rows,dtype=np.int64),self.next_id)
        if boundary(old)[0]!=boundary(new)[0]:raise ValueError('ChangedOrientedCavityBoundary')
        keys={key(f) for c in old+new for f in c['faces']};owners={k:[(i,f) for i,f in self.owners.get(k,[]) if i not in remove] for k in keys}
        for c in new:
            for f in c['faces']:owners[key(f)].append((c['id'],f))
        for k,uses in owners.items():
            if not uses:continue
            if len(uses)==1:
                if k not in self.far or len(self.owners.get(k,[]))!=1:raise ValueError('UnexpectedNewBoundaryFace')
            elif len(uses)!=2 or not reverse_cycle(uses[0][1],uses[1][1]):raise ValueError('ReplacementMustRetainActualOwners')
        lookup={c['id']:c for c in new};needed={i for uses in owners.values() for i,_ in uses}
        cells=[lookup[i] if i in lookup else self.byid[i] for i in sorted(needed)]
        return dict(removed=old,added=new,owners=owners,cells=cells,touched=keys)
    def quality(self,view):
        q=batch_quality(self.points,view['cells'],view['owners'],0,active_cells={c['id'] for c in view['added']},mixed_cells=True,face_batch_size=4096,compact=True)
        far=[f for c in view['added'] for f in c['faces'] if key(f) in self.far]
        if far:
            from capsule_core_control_audit import boundary_skewness
            centres={c['id']:t.geometry(self.points,c)['centre'] for c in view['added']}
            values=boundary_skewness(self.points[np.asarray(far)],np.asarray([centres[view['owners'][key(f)][0][0]] for f in far]))
            if not np.isfinite(values).all() or np.any(values>3.63636):return dict(q,passed=False,boundary_quality_failed=True)
        return q
    def commit(self,view):
        for c in view['removed']:del self.byid[c['id']];self.core_ids.remove(c['id'])
        for c in view['added']:self.byid[c['id']]=c;self.core_ids.add(c['id'])
        for k,uses in view['owners'].items():
            if uses:self.owners[k]=uses
            else:self.owners.pop(k,None)
        self.next_id+=len(view['added'])
