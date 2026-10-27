"""Study-only preservation guard before terminal-surface cavity remeshing.

A transverse intersection strictly inside two regular frozen prism tops
implies overlapping open cell interiors. Surface triangulation cannot repair
that overlap without changing at least one frozen cell's geometric domain.
"""
from fractions import Fraction as F
from collections import defaultdict
import termination_geometry_certificate as cert
t=cert.t; np=t.np

def barycentric(p, tri):
    a,b,c=tri; u=t.sub(b,a);v=t.sub(c,a);w=t.sub(p,a)
    uu=t.dot(u,u);uv=t.dot(u,v);vv=t.dot(v,v);den=uu*vv-uv*uv
    if den==0:raise ValueError('DegenerateFrozenTop')
    s=(t.dot(w,u)*vv-t.dot(w,v)*uv)/den
    z=(t.dot(w,v)*uu-t.dot(w,u)*uv)/den
    return (1-s-z,s,z)

def overlap_witness(points, a, b, lower_a, lower_b):
    hits=cert.triangle_intersection_points(points,a,b)
    if len(hits)<2:return dict(proven=False,reason='NoTransverseInteriorSegment')
    p=tuple(sum(x[j] for x in hits)/len(hits) for j in range(3))
    tops=[cert.rational(points[list(f)]) for f in (a,b)]
    lows=[cert.rational(points[list(f)]) for f in (lower_a,lower_b)]
    weights=[barycentric(p,x) for x in tops]
    normals=[t.cross(t.sub(x[1],x[0]),t.sub(x[2],x[0])) for x in tops]
    A,B=normals;aa=t.dot(A,A);ab=t.dot(A,B);bb=t.dot(B,B);det=aa*bb-ab*ab
    if det<=0 or not all(v>0 for row in weights for v in row):
        return dict(proven=False,reason='NotStrictTransverseCapInterior')
    # Solve A.d=B.d=-1 exactly. This points into both regular prism images.
    d=tuple(-((bb-ab)*A[j]+(aa-ab)*B[j])/det for j in range(3))
    jac=[]
    for upper,lower,lam,n in zip(tops,lows,weights,normals):
        velocity=tuple(sum(lam[i]*(upper[i][j]-lower[i][j]) for i in range(3)) for j in range(3))
        jac.append(t.dot(n,velocity))
    regular=all(j>0 for j in jac)
    return dict(proven=regular,reason='FrozenPrismOpenInteriorsOverlap' if regular else 'NonregularFrozenTop',
        intersection_points=[[str(v) for v in x] for x in hits],point=list(map(str,p)),
        barycentric=[[str(v) for v in row] for row in weights],gram_determinant=str(det),
        top_jacobians=list(map(str,jac)),common_inward_direction=list(map(str,d)),
        inward_dot_products=[str(t.dot(n,d)) for n in normals],
        proof='Strict interior barycentric coordinates and positive boundary Jacobians give local diffeomorphic prism neighborhoods. A common strictly inward direction enters both open cell interiors for sufficiently small positive displacement. Their intersection is open and has positive volume. No numerical tolerance or chosen displacement is used.')

def components(pairs):
    adj=defaultdict(set)
    for a,b in pairs:adj[a].add(b);adj[b].add(a)
    result=[]
    while adj:
        todo=[min(adj)];seen=set()
        while todo:
            v=todo.pop()
            if v in seen:continue
            seen.add(v);todo.extend(adj[v]-seen)
        result.append(sorted(seen))
        for v in seen:del adj[v]
    return result

def cavity_boundary(faces, selected):
    incidence=defaultdict(list)
    for i in sorted(selected):
        f=faces[i]
        for a,b in zip(f,f[1:]+f[:1]):incidence[tuple(sorted((a,b)))].append((a,b,i))
    boundary=[rows[0] for key,rows in sorted(incidence.items()) if len(rows)==1]
    outgoing=defaultdict(list)
    for a,b,i in boundary:outgoing[a].append(b)
    loops=[];remaining={(a,b) for a,b,i in boundary};simple=all(len(v)==1 for v in outgoing.values())
    if simple:
        while remaining:
            start=min(remaining)[0];loop=[start];cur=start
            while True:
                nxt=outgoing[cur][0]
                if (cur,nxt) not in remaining:raise ValueError('BrokenCavityBoundary')
                remaining.remove((cur,nxt));cur=nxt
                if cur==start:break
                loop.append(cur)
            loops.append(loop)
    return dict(boundary_edges=boundary,loops=loops,simple_disjoint_loops=simple,
        nonmanifold_edges=[list(k) for k,v in incidence.items() if len(v)>2])

def remesh_preserving_cells(faces, witnesses):
    """Fail closed before any triangulator when frozen cell domains overlap."""
    if any(w['proven'] for w in witnesses):
        return dict(status='RejectedFrozenVolumeOverlap',changed=False,new_vertices=[],faces=faces,
                    regenerated_cells=[],reason='Retriangulating or hiding the boundary does not remove overlap of preserved cell interiors.')
    return dict(status='NoRepairCertified',changed=False,new_vertices=[],faces=faces,regenerated_cells=[],
                reason='Absence of this obstruction alone does not qualify an arbitrary cavity.')
