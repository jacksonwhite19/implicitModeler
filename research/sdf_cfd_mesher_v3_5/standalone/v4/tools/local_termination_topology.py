"""Study-only mixed-cell termination templates; no production trigger."""
from fractions import Fraction as F
from collections import defaultdict, Counter
import residual_layer5_study as r
np=r.np;q=r.q
ROLES=((0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5))

def reduced_face(ids):
    out=[]
    for v in ids:
        if not out or v!=out[-1]:out.append(int(v))
    if len(out)>1 and out[-1]==out[0]:out.pop()
    if len(set(out))<3:return None
    if len(set(out))!=len(out):raise ValueError('NonconsecutiveFaceAlias')
    return tuple(out)

def cell(ids,column,layer):
    if not 0<=column<1000000 or layer<1:raise ValueError('StudyCellIdRange')
    ids=list(map(int,ids));unique=set(ids)
    if len(unique)==3:return None
    kind={4:'tetrahedron',5:'pyramid',6:'prism'}[len(unique)]
    faces=[reduced_face([ids[i] for i in face]) for face in ROLES]
    faces=[f for f in faces if f is not None]
    if len({tuple(sorted(f)) for f in faces})!=len(faces):raise ValueError('DuplicateCellFace')
    return dict(id=layer*1000000+column,column=column,layer=layer,kind=kind,vertices=ids,faces=faces)

def construct(wall,triangles,prefix,high,stopped):
    """Integer node aliasing selects real tets/pyramids, never collapsed prisms."""
    n=len(wall);k=len(prefix);stopped=set(map(int,stopped))
    if k<1:raise ValueError('CannotTerminateFirstLayer')
    if not stopped<=set(range(n)):raise ValueError('UnknownStoppedVertex')
    points=np.concatenate([wall]+prefix+[high]);upper=np.arange(n)+n*(k+1)
    for v in stopped:upper[v]=k*n+v
    cells=[]
    for layer in range(1,k+1):
        for col,tri in enumerate(triangles):cells.append(cell(np.r_[tri+(layer-1)*n,tri+layer*n],col,layer))
    for col,tri in enumerate(triangles):
        z=cell(np.r_[tri+k*n,upper[tri]],col,k+1)
        if z:cells.append(z)
    outer=[tuple(map(int,upper[tri])) for tri in triangles]
    last_prism=[k+1 if all(int(v) not in stopped for v in tri) else k for tri in triangles]
    return points,cells,outer,dict(stopped_vertices=sorted(stopped),last_prism_layer=last_prism,
        terminated_columns=[i for i,tri in enumerate(triangles) if all(int(v) in stopped for v in tri)],
        transition_columns=[i for i,tri in enumerate(triangles) if 0<sum(int(v) in stopped for v in tri)<3],
        continuing_columns=[i for i,x in enumerate(last_prism) if x==k+1])

def reverse_cycle(a,b):
    return any(tuple(a)==tuple(np.roll(tuple(reversed(b)),i)) for i in range(len(a)))

def vertex_links(outer):
    links=defaultdict(list)
    for face in outer:
        for j,v in enumerate(face):links[v].append((face[j-1],face[(j+1)%len(face)]))
    bad=[]
    for vertex,edges in sorted(links.items()):
        adj=defaultdict(set)
        for a,b in edges:adj[a].add(b);adj[b].add(a)
        seen=set();todo=[next(iter(adj))]
        while todo:
            v=todo.pop()
            if v in seen:continue
            seen.add(v);todo.extend(adj[v]-seen)
        if len(seen)!=len(adj) or any(len(v)!=2 for v in adj.values()):bad.append(vertex)
    return dict(passed=not bad,vertices=len(links),bad_vertices=bad)

def ownership(cells,outer,wall_faces):
    owners=defaultdict(list)
    for c in cells:
        for f in c['faces']:owners[tuple(sorted(f))].append((c['id'],f))
    return _ownership_report(owners,outer,wall_faces)

def _ownership_from_core_incidence(prism_owners,additions,inverse,unique,mapping,last,outer,wall_faces):
    """Consume this call's boundary-audit traversal in the same ownership owner."""
    owners=defaultdict(list,{k:list(rows) for k,rows in prism_owners.items()})
    keys=[tuple(row) for row in np.sort(mapping[unique],axis=1).tolist()]
    # Preserve cell/face row order and physical winding. Canonical keys are
    # identity only; the audit's ordered faces remain the physical faces.
    if len(inverse)!=4*len(additions):raise ValueError('ChangedCoreIncidencePopulation')
    for j,c in enumerate(additions):
        if c['id']!=last+j or len(c['faces'])!=4:raise ValueError('ChangedCoreIncidenceCell')
        for role,face in enumerate(c['faces']):
            k=keys[int(inverse[4*j+role])]
            # Bind the fresh materialization to its audit group without sorting
            # every face again. Reuse its physical tuple, not another face copy.
            if (len(face)!=3 or face[0]==face[1] or face[0]==face[2] or face[1]==face[2] or
                    face[0] not in k or face[1] not in k or face[2] not in k):
                raise ValueError('ChangedCoreIncidenceFace')
            owners[k].append((c['id'],face))
    return _ownership_report(owners,outer,wall_faces)

def _ownership_report(owners,outer,wall_faces):
    intended={tuple(sorted(f)) for f in outer}|{tuple(sorted(f)) for f in wall_faces}
    failures=[]
    for key,rows in sorted(owners.items()):
        if key in intended:
            if len(rows)!=1:failures.append(('BoundaryOwnership',key,len(rows)))
        elif len(rows)!=2:failures.append(('InternalOwnership',key,len(rows)))
        elif not reverse_cycle(rows[0][1],rows[1][1]):failures.append(('InternalOrientation',key))
    for key in intended-set(owners):failures.append(('MissingBoundary',key))
    edges=defaultdict(list)
    for i,face in enumerate(outer):
        for a,b in zip(face,face[1:]+face[:1]):edges[tuple(sorted((a,b)))].append((a,b,i))
    bad_edges=[(key,val) for key,val in edges.items() if len(val)!=2 or val[0][:2]!=val[1][:2][::-1]]
    return owners,dict(passed=not failures and not bad_edges,failures=failures,outer_bad_edges=bad_edges,
        internal_faces=sum(len(v)==2 for v in owners.values()),boundary_faces=sum(len(v)==1 for v in owners.values()),
        outer_faces=len(outer),outer_edges=len(edges),outer_duplicate_faces=len(outer)-len(set(tuple(sorted(f)) for f in outer)))

def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def exact_cell(points,c):
    if c['kind']=='prism':return q.exact_wedge(points[c['vertices']])
    p={v:tuple(F(float(x)) for x in points[v]) for v in set(c['vertices'])}
    if c['kind']=='tetrahedron':
        face=c['faces'][0];apex=next(v for v in p if v not in face)
        a,b,d=[p[v] for v in reversed(face)];J=dot(cross(sub(b,a),sub(d,a)),sub(p[apex],a))
        return dict(jacobian=str(J),jacobian_sign=(J>0)-(J<0),signed_volume=str(J/6),volume_sign=(J>0)-(J<0),method='AffineTetrahedronExactTripleProduct')
    face=next(f for f in c['faces'] if len(f)==4);A=p[next(v for v in p if v not in face)]
    a,b,d,e=[p[v] for v in reversed(face)];B=sub(b,a);C=sub(e,a);D=sub(sub(d,b),C)
    values=[]
    for u,v in ((0,0),(1,0),(1,1),(0,1)):
        Q=tuple(a[i]+u*B[i]+v*C[i]+u*v*D[i] for i in range(3))
        values.append(dot(cross(tuple(B[i]+v*D[i] for i in range(3)),tuple(C[i]+u*D[i] for i in range(3))),sub(A,Q)))
    J=min(values);V=sum(values)/12
    return dict(jacobian=str(J),jacobian_sign=(J>0)-(J<0),signed_volume=str(V),volume_sign=(V>0)-(V<0),
        corner_coefficients=list(map(str,values)),method='BilinearBasePyramidJacobianDividedBy(1-w)^2; bilinear coefficient extrema at four corners')

def geometry(points,c):
    fg=q.legacy_modules()['component_quality'].FG;sf=[];cf=[]
    for face in c['faces']:
        x=points[list(face)][None];S,C=fg.face_area_centre_tri(x) if len(face)==3 else fg.face_area_centre_quad(x)
        sf.append(S[0]);cf.append(C[0])
    sf=np.array(sf);cf=np.array(cf);est=cf.mean(0);pyr=np.einsum('ij,ij->i',sf,cf-est)
    V=sum(pyr)/3;C=np.sum(pyr[:,None]*(.75*cf+.25*est),axis=0)/sum(pyr)
    B=abs(sf).sum(0);aspect=max(max(B)/min(B),sum(B)/(6*abs(V)**(2/3))) if V else float('inf')
    return dict(centre=C,volume=float(V),areas=sf,face_centres=cf,aspect=float(aspect))

def quality(points,cells,owners,base_layer,*,active_cells=None):
    cq=q.legacy_modules()['component_quality'];byid={c['id']:c for c in cells};selected={c['id'] for c in cells if c['layer']>=base_layer and (active_cells is None or c['id'] in active_cells)};needed=set(selected)
    for rows in owners.values():
        if any(i in selected for i,f in rows):needed.update(i for i,f in rows)
    geom={i:geometry(points,byid[i]) for i in sorted(needed)};bad=[];records=[];exact=[]
    for c in cells:
        if c['id'] not in selected:continue
        i=c['id'];g=geom[i];valid=exact_cell(points,c);exact.append(dict(id=i,**valid))
        if valid['jacobian_sign']<=0 or valid['volume_sign']<=0:bad.append(dict(kind='exact_cell',id=i,column=c['column'],detail=valid))
        # OpenFOAM internal/coupled-face area tensor; exposed outer faces excluded.
        mask=np.array([len(owners[tuple(sorted(f))])==2 for f in c['faces']]);S=g['areas'][mask]
        avg=np.linalg.norm(S,axis=1).mean();tensor=(S/avg).T@(S/avg);det=float(np.linalg.det(tensor))
        rec=dict(id=i,column=c['column'],cell_type=c['kind'],volume=g['volume'],determinant=det,aspect=g['aspect']);records.append(rec)
        if not g['volume']>0 or not det>=.0011 or not g['aspect']<=1000:bad.append(dict(kind='cell_quality',**rec))
    faces=[]
    for key,rows in sorted(owners.items()):
        if len(rows)!=2 or not any(i in selected for i,f in rows):continue
        (a,f),(b,_)=rows;vs=points[list(f)][None];S,C=cq.FG.face_area_centre_tri(vs) if len(f)==3 else cq.FG.face_area_centre_quad(vs)
        vals=cq.face_metrics(vs,S,C,geom[a]['centre'][None],geom[b]['centre'][None],np.array([geom[a]['volume']]),np.array([geom[b]['volume']]))
        wt,sk,vr,no=[float(v[0]) for v in vals];rec=dict(vertices=f,owners=[a,b],weight=wt,skewness=sk,symmetric_volume_ratio=vr);faces.append(rec)
        if not wt>=.055 or not sk<=3.63636 or not vr>=.01:bad.append(dict(kind='face_quality',**rec))
    return dict(passed=not bad,failures=bad,cells=records,faces=faces,exact=exact)

# These bindings belong to the defining module, before any caller installation.
# Cold dispatch must preserve a substituted quality or geometry owner.
_STANDARD_GEOMETRY = geometry
_STANDARD_QUALITY = quality
