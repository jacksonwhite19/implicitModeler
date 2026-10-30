"""Exact oriented boundary degree at every cell-interior witness.

Combined with a complete pairwise face certificate, positive cell maps and
two-owner oriented incidence, degree one excludes containment/duplicate volume.
No face-crossing result is inferred from degree samples alone.
"""
from fractions import Fraction as F
from projected_boundary_index import ProjectedBoundaryIndex
import local_cavity_remesh as s
np=s.np;t=s.t

def degree(points,faces,samples,*,stats=None):
    P=s.cert.rational(points);samples=list(samples);pending=list(range(len(samples)));out=[None]*len(samples)
    for k in range(1,9):
        dy=F(k);dz=F(k*k+1)
        projected=[(p[1]-dy*p[0],p[2]-dz*p[0]) for p in P]
        tri=[[projected[v] for v in f] for f in faces]
        lo=np.nextafter(np.array([[float(min(p[j] for p in row)) for j in range(2)] for row in tri]),-np.inf)
        hi=np.nextafter(np.array([[float(max(p[j] for p in row)) for j in range(2)] for row in tri]),np.inf)
        candidate_index=ProjectedBoundaryIndex(lo,hi)
        retry=[]
        for index in pending:
            p=samples[index];uv=(p[1]-dy*p[0],p[2]-dz*p[0]);v=np.array(list(map(float,uv)))
            ids=candidate_index.containing(v);count=0;ambiguous=False
            if stats is not None:stats['candidate_pairs']=stats.get('candidate_pairs',0)+len(ids)
            for i in ids:
                if stats is not None:stats['exact_contribution_evaluations']=stats.get('exact_contribution_evaluations',0)+1
                a,b,c=tri[i];B=(b[0]-a[0],b[1]-a[1]);C=(c[0]-a[0],c[1]-a[1]);W=(uv[0]-a[0],uv[1]-a[1])
                det=B[0]*C[1]-B[1]*C[0]
                if det==0:
                    # Conservatively retry projected degenerate triangles.
                    ambiguous=True;break
                u=(W[0]*C[1]-W[1]*C[0])/det;w=(B[0]*W[1]-B[1]*W[0])/det
                if min(u,w,1-u-w)<0:continue
                xyz=[P[v] for v in faces[i]];x=xyz[0][0]+u*(xyz[1][0]-xyz[0][0])+w*(xyz[2][0]-xyz[0][0])
                if x<p[0]:continue
                if x==p[0] or min(u,w,1-u-w)==0:ambiguous=True;break
                count+=1 if det>0 else -1
            if ambiguous:retry.append(index)
            else:out[index]=dict(degree=count,ray=[1,int(dy),int(dz)])
        pending=retry
        if not pending:break
    return dict(values=out,unresolved=pending,passed=not pending and all(x['degree']==1 for x in out))

def audit(points,cells,outer,wall,*,_degree=None):
    P=s.cert.rational(points);samples=[];valid=[]
    for cell in cells:
        ids=sorted(set(cell['vertices']));samples.append(tuple(sum(P[v][j] for v in ids)/len(ids) for j in range(3)))
        z=t.exact_cell(points,cell);valid.append(dict(id=cell['id'],**z))
    boundary=[tuple(reversed(f)) for f in wall]+outer
    # Default remains dynamic for the original reader and existing prism scope.
    d=(degree if _degree is None else _degree)(points,boundary,samples)
    cell_volume=sum(F(x['signed_volume']) for x in valid)
    boundary_volume=sum(t.dot(P[f[0]],t.cross(P[f[1]],P[f[2]]))/6 for f in boundary)
    return dict(interior_degree=d,exact_cells=valid,cell_volume=str(cell_volume),boundary_volume=str(boundary_volume),
        exact_volume_balance=cell_volume==boundary_volume,positive_cells=all(x['volume_sign']>0 and x['jacobian_sign']>0 for x in valid),
        conditional_nonoverlap=d['passed'],
        prerequisites='Must also require complete unique-face exact certificate and oriented manifold incidence. Degree samples alone are not a volume certificate.',
        proof='With nonintersecting cell boundaries, interior overlap without a boundary crossing requires containment or coincident domains. Each standard-cell interior witness has global oriented boundary degree one, excluding containment; opposite internal face incidence and positive orientation exclude coincident domains. Complete face matching closes the oriented cell complex. Exact total cell/boundary volume balance additionally checks filling accounting.')

if __name__=='__main__':
    import time
    st=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=s.r.reproduce();z=s.load(s.OUT/'SELECTED.json')
    p,c,f=s.construct(case.x,case.f,prefix+[high],z['vertex_depth']);result=audit(p,c,f,[tuple(map(int,x)) for x in case.f]);s.save('VOLUME_AUDIT.json',result)
    s.save('VOLUME_RUNTIME.json',dict(seconds=time.perf_counter()-st,peak_rss=s.q.psutil.Process().memory_info().peak_wset))
    print('volume',result['interior_degree']['passed'],result['exact_volume_balance'],result['positive_cells'],flush=True)
