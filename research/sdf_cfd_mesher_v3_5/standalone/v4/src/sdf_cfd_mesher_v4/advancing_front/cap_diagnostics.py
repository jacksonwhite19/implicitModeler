"""Study-only exact control diagnostics. No geometry admission or new gates.

At reference height zero the preserved wedge polynomial is
J_j(0) = ((w1-w0) cross (w2-w0)) dot (x_j-w_j).
Thus moving other top vertices cannot change this particular corner value.
These are exact binary64-as-rational identities, not a surrogate quality metric.
"""
from fractions import Fraction as Q
import numpy as np
from .local_relaxation import tangent_basis
from .local_relaxation import RelaxationPatch
from ..common.authority import frozen
from ..forecast.detailed.patch import Patch,require_closure,LocalityFailure
from ..runtime.regions import dependencies
from ..runtime.frozen_imports import legacy_modules
from ..forecast.detailed.metrics import cell_metrics

def active_patch(top,vertices,max_vertices=512,max_columns=4096):
    """Same frozen core/halo rules with an explicit dependency-closed active set."""
    active=np.unique(np.asarray(vertices,dtype=int))
    if not len(active):raise ValueError('NoActiveVertices')
    if active[0]<0 or active[-1]>=len(top.vertex_neighbors.offsets)-1:raise ValueError('InvalidActiveVertex')
    if len(active)>max_vertices:raise LocalityFailure('ActiveVertexBound')
    _,_,affected=dependencies(top,active);core=set(map(int,affected))
    for e in np.unique(top.triangle_edges[affected]):core.update(map(int,top.edge_triangles.row(e)))
    core=np.array(sorted(core));sides=np.unique(top.triangle_edges[core]);cols=set(map(int,core))
    for e in sides:cols.update(map(int,top.edge_triangles.row(e)))
    if len(cols)>max_columns or len(cols)==len(top.triangles):raise LocalityFailure('VerificationClosureBound')
    cols=np.array(sorted(cols));cv=np.unique(top.triangles[core]);pv=np.unique(top.triangles[cols])
    p=Patch(*(frozen(a,'<i8') for a in (core,cols,pv,cv,sides)),top.sha256);require_closure(p,top)
    return RelaxationPatch(frozen(active,'<i8'),np.setdiff1d(cv,active),np.setdiff1d(pv,cv),affected,np.setdiff1d(core,affected),p)

def rational_vector(v):return tuple(Q(float(x)) for x in v)
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])

def exact_rank(rows):
    """Rational rank; no numerical rank tolerance or geometry-health cutoff."""
    pivots={}
    for row in rows:
        r=[Q(v) for v in row]
        for j,p in sorted(pivots.items()):
            if r[j]:
                scale=r[j];r=[a-scale*b for a,b in zip(r,p)]
        nz=next((j for j,a in enumerate(r) if a),None)
        if nz is not None:
            scale=r[nz];pivots[nz]=[a/scale for a in r]
    return len(pivots)

def tangent_rank(normals):
    b=tangent_basis(normals)
    ranks=[exact_rank([[Q(float(v)) for v in row] for row in block]) for block in b]
    return dict(vertices=len(b),dofs=2*len(b),independent_rank=sum(ranks),block_ranks=ranks)

def bottom_corner(wall,top,corner,basis=None):
    w=[rational_vector(v) for v in wall];x=[rational_vector(v) for v in top]
    area=cross(sub(w[1],w[0]),sub(w[2],w[0]));j=dot(area,sub(x[corner],w[corner]))
    out=dict(jacobian=str(j),sign=(j>0)-(j<0),wall_area_vector=[str(v) for v in area])
    if basis is not None:
        coeff=[dot(area,rational_vector(basis[:,a])) for a in range(2)]
        out.update(tangent_derivative=[str(v) for v in coeff],tangent_derivative_float=[float(v) for v in coeff],tangent_controls_corner=any(coeff))
    return out

def locked_corners(front,topology,coordinates,patch,normals):
    active=set(map(int,patch.active));affected=set(map(int,patch.affected_columns));rows=[]
    for col in patch.detailed.core_columns:
        ids=topology.triangles[col];w=front.coordinates[ids];x=coordinates[ids];basis=tangent_basis(normals[ids])
        for j,v in enumerate(ids):
            e=bottom_corner(w,x,j,basis[j])
            if e['sign']<=0:
                rows.append(dict(column=int(col),corner=j,vertex=int(v),active=int(v) in active,
                    affected_column=int(col) in affected,exact=e,
                    active_space_derivative=e['tangent_derivative'] if int(v) in active else ['0','0'],
                    invariant_under_all_active_moves=int(v) not in active,
                    wall=w[j].tolist(),layer1=x[j].tolist(),normal=normals[int(v)].tolist()))
    return rows

def direction_coverage(directions):
    a=np.asarray(directions,float)
    rank=exact_rank([[Q(float(x)) for x in row] for row in a.T])
    opposite=[];duplicate=[]
    for i in range(len(a)):
        for j in range(i):
            if np.array_equal(a[i],a[j]):duplicate.append([j,i])
            if np.array_equal(a[i],-a[j]) and np.any(a[i]):opposite.append([j,i])
    return dict(rank=rank,directions=len(a),ambient_dofs=a.shape[1],opposite_pairs=opposite,duplicate_pairs=duplicate,
                nonzero_columns=[int(i) for i in np.flatnonzero(np.any(a,axis=0))])

def immediate_gates_pass(numeric_gate,exact_gate,certificate):
    """Immediate gates only; not private-step admission or future comparison.

    Never creates, accepts, advances or writes a front.
    """
    complete=certificate.get('complete',certificate.get('classification')=='LocalCoreSidePairsCertified')
    return bool(numeric_gate.get('passed') and exact_gate.get('passed') and complete
                and not certificate.get('forbidden') and not certificate.get('unresolved') and not certificate.get('invalid_surfaces'))

def current_layer_metrics(front,top,patch,proposed):
    """The same preserved primitives restricted to the actual candidate layer.

    This rejects numeric failures before optional future work. It does not
    authorize a candidate without exact signs and continuous certification.
    `proposed` supports local coordinate overlays; no global copy is needed.
    """
    require_closure(patch,top)
    f=top.triangles[patch.columns]
    p=np.concatenate((front.coordinates[f],proposed[f]),axis=1)
    g=cell_metrics(p,np.ones(len(p),dtype=int));core=np.searchsorted(patch.columns,patch.core_columns)
    cq=legacy_modules()['component_quality'];fg=cq.FG;ed=top.edges[patch.side_edges]
    vs=np.stack((front.coordinates[ed[:,0]],front.coordinates[ed[:,1]],proposed[ed[:,1]],proposed[ed[:,0]]),axis=1)
    own=np.array([top.edge_triangles.row(e) for e in patch.side_edges]);ix=np.searchsorted(patch.columns,own)
    S,C=fg.face_area_centre_quad(vs)
    values=cq.face_metrics(vs,S,C,g['centre'][ix[:,0]],g['centre'][ix[:,1]],g['volume'][ix[:,0]],g['volume'][ix[:,1]])
    cells={k:v[core][None] for k,v in g.items()};cells['column']=patch.core_columns[None];cells['layer']=np.ones((1,len(core)),int)
    cells['cell_id']=patch.core_columns[None]
    return dict(cells=cells,face_ids=[('side',1,int(e),int(a),int(b)) for e,(a,b) in zip(patch.side_edges,own)],
        faces=dict(zip(('weight','skewness','symmetric_volume_ratio','nonorthogonality'),values)))
