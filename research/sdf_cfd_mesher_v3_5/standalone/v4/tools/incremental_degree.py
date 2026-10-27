"""Exact closed-chain support and direction-compatible boundary degree reuse.

Only insertion-only prism chains are optimized. The reference ray predicate is
unchanged. A witness outside every added-cell box has zero delta degree because
the verified boundary delta is the sum of closed cell boundaries. A separate
outward-rounded interval test excludes new ray/edge degeneracy before retaining
the old first-direction receipt. All other witnesses use the original audit.
"""
import time
from collections import Counter
from fractions import Fraction as F
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import cavity_volume_audit as global_audit
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash

SCHEMA='V4ClosedChainDegreeReceipt/1'

def cycle(f):
    f=tuple(map(int,f));return min(f[i:]+f[:i] for i in range(len(f)))

def chain(faces):
    out=Counter()
    for f in faces:
        a=cycle(f);b=cycle(tuple(reversed(f)));out[min(a,b)]+=1 if a<b else -1
    return Counter({k:v for k,v in out.items() if v})

def boundary_delta(old,new,points,additions):
    old_list=[cycle(f) for f in old];new_list=[cycle(f) for f in new]
    old=set(old_list);new=set(new_list)
    if len(old)!=len(old_list) or len(new)!=len(new_list):raise ValueError('DuplicateBoundaryFace')
    if any(not {v for f in c['faces'] for v in f}<=set(c['vertices']) for c in additions):raise ValueError('CellSupportIncomplete')
    retired=sorted(old-new);added=sorted(new-old)
    delta=chain(added+[tuple(reversed(f)) for f in retired])
    cells=chain([f for c in additions for f in c['faces']])
    if delta!=cells:raise ValueError('BoundaryDeltaNotAddedCellChain')
    vertices=sorted({v for f in retired+added for v in f})
    return dict(schema='V4OrientedBoundaryDelta/1',retired=retired,added=added,
        unchanged=len(old&new),chain_equal=True,
        geometry=identity([[v,points[v].tolist()] for v in vertices]),
        topology=identity(dict(retired=retired,added=added)),
        ancestry=[dict(cell=c['id'],column=c['column'],layer=c['layer']) for c in sorted(additions,key=lambda c:c['id'])])

class PointIndex:
    def __init__(self,points):
        self.points=np.asarray(points,float);self.tree=cKDTree(self.points)
        self.pairs=0;self.max_candidates=0

    def query(self,lo,hi):
        # Rounded point coordinates preserve containment in outward-rounded
        # boxes. Radius covers every box corner under the infinity norm.
        center=lo/2+hi/2
        radius=np.nextafter(max(np.max(center-lo),np.max(hi-center)),np.inf)
        ids=np.asarray(self.tree.query_ball_point(center,radius,p=np.inf),dtype=np.int64)
        if len(ids):ids=ids[np.all(self.points[ids]>=lo,axis=1)&np.all(self.points[ids]<=hi,axis=1)]
        self.pairs+=len(ids);self.max_candidates=max(self.max_candidates,len(ids))
        return ids

    @property
    def array_bytes(self):return self.points.nbytes+self.tree.indices.nbytes

def interval(values):
    a=np.asarray(values,float);return np.nextafter(a,-np.inf),np.nextafter(a,np.inf)

def sub(a,b):return np.nextafter(a[0]-b[1],-np.inf),np.nextafter(a[1]-b[0],np.inf)

def mul(a,b):
    v=np.array([a[0]*b[0],a[0]*b[1],a[1]*b[0],a[1]*b[1]])
    return np.nextafter(v.min(axis=0),-np.inf),np.nextafter(v.max(axis=0),np.inf)

def possible_collinear(a,b,p):
    u=sub(b,a);w=sub(p,a)
    det=sub(mul((u[0][0],u[1][0]),(w[0][:,1],w[1][:,1])),
            mul((u[0][1],u[1][1]),(w[0][:,0],w[1][:,0])))
    return (det[0]<=0)&(det[1]>=0) | ~np.isfinite(det[0]) | ~np.isfinite(det[1])

def witness_digest(w):return identity([[str(x.numerator),str(x.denominator)] for x in w])

class DegreeState:
    def __init__(self,points,boundary,witness,result=None):
        self.points=np.array(points,copy=True);self.boundary=[tuple(f) for f in boundary];self.witness=dict(witness)
        ids=sorted(witness)
        if result is None:result=global_audit.degree(points,boundary,[witness[i] for i in ids])
        if not result['passed'] or len(result['values'])!=len(ids):raise ValueError('InvalidBootstrapDegree')
        self.rows=dict(zip(ids,result['values']))
        self.implementation=identity(dict(schema=SCHEMA,reference=file_hash(Path(global_audit.__file__)),implementation=file_hash(Path(__file__))))
        self.root=identity(dict(schema=SCHEMA,implementation=self.implementation,
            boundary=identity(dict(faces=sorted(cycle(f) for f in boundary),points=array_hash(points))),
            receipts=[[i,witness_digest(witness[i]),self.rows[i]] for i in ids]))

    def update(self,points,boundary,witness,additions):
        start=time.perf_counter();times={};old_ids=sorted(self.witness);new_ids=sorted(witness.keys()-self.witness.keys())
        # Conservative fallback: changed/retired witnesses, changed committed
        # coordinates, unsupported cell types or unproved chain equality.
        reason=None
        if not self.witness.keys()<=witness.keys() or any(witness[i]!=self.witness[i] for i in self.witness if i in witness):reason='ChangedOrRetiredWitness'
        elif len(points)<len(self.points) or not np.array_equal(points[:len(self.points)],self.points):reason='ChangedCommittedGeometry'
        elif any(c['kind']!='prism' for c in additions):reason='UnsupportedCellType'
        tick=time.perf_counter()
        try:delta=boundary_delta(self.boundary,boundary,points,additions) if reason is None else None
        except ValueError:reason='UnprovedBoundaryChain';delta=None
        times['boundary_delta']=time.perf_counter()-tick
        if reason:
            stats={};result=global_audit.degree(points,boundary,[witness[i] for i in sorted(witness)],stats=stats)
            candidate=DegreeState(points,boundary,witness,result)
            return candidate,dict(fallback=reason,times=times,seconds=time.perf_counter()-start,counts=stats,passed=result['passed'])
        tick=time.perf_counter()
        W=np.asarray([[float(x) for x in self.witness[i]] for i in old_ids]);index=PointIndex(W)
        selected=set()
        for c in additions:
            xyz=points[c['vertices']]
            selected.update(map(int,index.query(np.nextafter(xyz.min(axis=0),-np.inf),np.nextafter(xyz.max(axis=0),np.inf))))
        support_count=len(selected);support_pairs=index.pairs
        times['affected_witness_query']=time.perf_counter()-tick;tick=time.perf_counter()
        # Only k=1 receipts can retain the same first-valid direction without
        # revisiting previously rejected directions. All k>1 use global fallback.
        selected.update(j for j,i in enumerate(old_ids) if self.rows[i]['ray']!=[1,1,2])
        UV=np.asarray([[float(w[1]-w[0]),float(w[2]-2*w[0])] for w in (self.witness[i] for i in old_ids)])
        ray_index=PointIndex(UV);point_interval=interval(UV)
        vertices={v for f in delta['added'] for v in f}
        projected={v:(F(float(points[v,1]))-F(float(points[v,0])),F(float(points[v,2]))-2*F(float(points[v,0]))) for v in vertices}
        degenerate_pairs=0
        for f in delta['added']:
            a,b,c=[projected[v] for v in f]
            determinant=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            if determinant==0:
                lo=np.nextafter(np.array([float(min(q[j] for q in (a,b,c))) for j in range(2)]),-np.inf)
                hi=np.nextafter(np.array([float(max(q[j] for q in (a,b,c))) for j in range(2)]),np.inf)
                hit=ray_index.query(lo,hi);selected.update(map(int,hit));degenerate_pairs+=len(hit)
                continue
            for u,v in ((a,b),(b,c),(c,a)):
                lo=np.nextafter(np.array([float(min(u[j],v[j])) for j in range(2)]),-np.inf)
                hi=np.nextafter(np.array([float(max(u[j],v[j])) for j in range(2)]),np.inf)
                hit=ray_index.query(lo,hi)
                if len(hit):
                    possible=possible_collinear(interval(u),interval(v),(point_interval[0][hit],point_interval[1][hit]))
                    selected.update(map(int,hit[possible]))
        times['direction_screening']=time.perf_counter()-tick;tick=time.perf_counter()
        affected=[old_ids[j] for j in sorted(selected)];stats={}
        recomputed=global_audit.degree(points,boundary,[witness[i] for i in affected],stats=stats)
        if not recomputed['passed']:raise ValueError('IncrementalHistoricalDegreeFailure')
        times['affected_exact_degree']=time.perf_counter()-tick;tick=time.perf_counter()
        new_stats={};new=global_audit.degree(points,boundary,[witness[i] for i in new_ids],stats=new_stats)
        if not new['passed']:raise ValueError('IncrementalNewDegreeFailure')
        times['new_witness_full_degree']=time.perf_counter()-tick;tick=time.perf_counter()
        candidate=object.__new__(DegreeState);candidate.points=np.array(points,copy=True);candidate.boundary=[tuple(f) for f in boundary]
        candidate.witness=dict(witness);candidate.rows=self.rows.copy();candidate.rows.update(zip(affected,recomputed['values']));candidate.rows.update(zip(new_ids,new['values']))
        candidate.implementation=self.implementation
        selection_hash=identity(affected)
        candidate.root=identity(dict(schema=SCHEMA,implementation=self.implementation,prior=self.root,delta=identity(delta),
            affected=selection_hash,receipts=[[i,witness_digest(witness[i]),candidate.rows[i]] for i in affected+new_ids],
            reused_count=len(old_ids)-len(affected),reuse='ClosedCellChainSupportZeroAndFirstRayNondegenerate'))
        times['receipt_update']=time.perf_counter()-tick
        metrics=dict(passed=True,fallback=None,delta=delta,delta_identity=identity(delta),root=candidate.root,
            affected_identity=selection_hash,times=times,seconds=time.perf_counter()-start,
            counts=dict(historical=len(old_ids),new=len(new_ids),support_selected=support_count,selected=len(affected),
                reused=len(old_ids)-len(affected),cell_box_witness_pairs=support_pairs,
                projected_edge_witness_pairs=ray_index.pairs,projected_degenerate_face_pairs=degenerate_pairs,
                exact_changed_face_contribution_evaluations=0,historical_full_boundary=stats,new_full_boundary=new_stats),
            memory=dict(index_array_bytes=index.array_bytes+ray_index.array_bytes,interval_array_bytes=sum(x.nbytes for x in point_interval),
                largest_query_ids_bytes=max(index.max_candidates,ray_index.max_candidates)*8,
                receipt_repr_bytes=len(str(candidate.rows).encode()),native_tree_bytes='Not measured'))
        return candidate,metrics

    def result(self):return dict(values=[self.rows[i] for i in sorted(self.rows)],unresolved=[],passed=all(r['degree']==1 for r in self.rows.values()))
