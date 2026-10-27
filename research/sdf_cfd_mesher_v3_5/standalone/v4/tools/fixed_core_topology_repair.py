"""Local bistellar/Delaunay proposals with unchanged cavity boundary and guards.

Candidates are search states, never qualified meshes. Exact cavity volume equality
is checked for each adopted topology change; final certificate remains mandatory.
"""
import sys,time,itertools
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial import Delaunay,QhullError
from fractions import Fraction as F
from fixed_core_study import *

def exact_volume(q):
    a=[[F(float(z)) for z in p] for p in q];b=[[a[j][i]-a[0][i] for i in range(3)] for j in (1,2,3)]
    return (b[0][0]*(b[1][1]*b[2][2]-b[1][2]*b[2][1])-b[0][1]*(b[1][0]*b[2][2]-b[1][2]*b[2][0])+b[0][2]*(b[1][0]*b[2][1]-b[1][1]*b[2][0]))/6
def boundary(t):
    rows=t[:,PATS].reshape(-1,3);un,ix,n=np.unique(np.sort(rows,1),axis=0,return_index=True,return_counts=True)
    if np.any(n>2):raise ValueError('NonmanifoldCavity')
    return rows[ix[n==1]]
def orient(x,t):
    t=np.asarray(t,dtype='i8').copy();negative=volume(x[t])<0;t[negative]=t[negative][:,[0,2,1,3]];return t
def invariant(x,old,new):
    try:a=boundary(old);b=boundary(new)
    except ValueError:return False
    if {tuple(sorted(f)) for f in a}!={tuple(sorted(f)) for f in b}:return False
    # Matching outward orientation, not merely unordered face membership.
    def oriented(fs):return {min(tuple(np.roll(f,k)) for k in range(3)) for f in fs}
    if oriented(a)!=oriented(b):return False
    nv=[exact_volume(q) for q in x[new]]
    return all(z>0 for z in nv) and sum(nv)==sum(exact_volume(q) for q in x[old])
def patch_interface(x,t,region,iface,ci,m):
    bf=boundary(t[sorted(region)]);bkeys={tuple(sorted(z)) for z in bf};external={}
    for face,own in zip(m['faces'],m['owners']):
        a,b=map(int,own)
        if (a in region)!=(b in region):external[tuple(sorted(face))]=(m['centres'][b if a in region else a],m['volume'][b if a in region else a])
    for j,face in enumerate(ci[iface.f]):
        if int(m['interface_owner'][j]) in region:external[tuple(sorted(face))]=(iface.c[j],iface.v[j])
    if set(external)!=bkeys:raise ValueError('IncompleteCavityGuard')
    vertices=np.unique(bf);p=Interface.__new__(Interface);p.x=x[vertices];p.f=np.searchsorted(vertices,bf);p.core_boundary_sign=1
    p.c=np.array([external[tuple(sorted(z))][0] for z in bf]);p.v=np.array([external[tuple(sorted(z))][1] for z in bf]);p.area,p.fc=FG.face_area_centre_tri(p.x[p.f]);return p,vertices
def better(old,new):
    # No lost passing guard. Counts control search; never alter acceptance gates.
    keys=old['failures']
    return all(new['failures'][k]==0 for k in keys if old['failures'][k]==0) and sum(new['failures'].values())<sum(old['failures'].values())
def repair(src,interface,dest,method):
    import psutil
    start=time.perf_counter();dest.mkdir(parents=True,exist_ok=False);x,t,ci,iface=load_core(src,interface);x0=x.copy();t0=t.copy();q,m=evaluate(x,t,ci,iface)
    records=[];accepted=0;last=start;counts=dict(proposed=0,boundary_or_exact_invariant_rejected=0,quality_compared=0)
    # Rebuild after accepted changes; stop at a complete non-improving sweep or
    # the explicit elapsed-time bound. This never changes scientific gates.
    for sweep in range(32):
        prior=accepted;bad=set(map(int,m['bad']));proposals=[]
        if method=='fixed_flips':bad={i for i in bad if np.all(np.isin(t[i],ci))}
        if method in ('flips','fixed_flips'):
            for face,(a,b) in zip(m['faces'],m['owners']):
                if int(a) not in bad and int(b) not in bad:continue
                u=int(np.setdiff1d(t[a],face)[0]);v=int(np.setdiff1d(t[b],face)[0]);ff=list(map(int,face))
                proposals.append((set(map(int,(a,b))),np.array([[u,v,ff[k],ff[(k+1)%3]] for k in range(3)]),None,'2-3'))
            stars={}
            for i,tet in enumerate(t):
                for edge in itertools.combinations(sorted(map(int,tet)),2):stars.setdefault(edge,[]).append(i)
            for (a,b),ids in stars.items():
                if len(ids) not in (3,4) or not bad.intersection(ids):continue
                ring=sorted(set(t[ids].ravel())-{a,b})
                if len(ids)==3 and len(ring)==3:proposals.append((set(ids),np.array([[a,*ring],[b,*ring]]),None,'3-2'))
                if len(ids)==4 and len(ring)==4:
                    graph={v:set() for v in ring}
                    for tet in t[ids]:
                        u,v=sorted(set(tet)-{a,b});graph[u].add(v);graph[v].add(u)
                    if any(len(z)!=2 for z in graph.values()):continue
                    cycle=[ring[0],min(graph[ring[0]])]
                    while len(cycle)<4:cycle.append(next(v for v in sorted(graph[cycle[-1]]) if v!=cycle[-2]))
                    for j in (0,1):
                        u,v=cycle[j],cycle[(j+2)%4];outer=[a,cycle[(j+1)%4],b,cycle[(j+3)%4]]
                        proposals.append((set(ids),np.array([[u,v,outer[k],outer[(k+1)%4]] for k in range(4)]),None,'4-4'))
        else:
            adjacency={i:set() for i in range(len(t))}
            for a,b in m['owners']:adjacency[int(a)].add(int(b));adjacency[int(b)].add(int(a))
            for i in sorted(bad):
                region={i}|adjacency[i]
                if method=='insert':region|=set().union(*(adjacency[j] for j in list(region)))
                points=np.unique(t[sorted(region)]);newpoint=None
                if method=='insert':
                    # Strict barycentre of the largest-volume tet in this
                    # measured cavity. It lies inside committed core geometry.
                    j=max(region,key=lambda z:m['volume'][z]);newpoint=x[t[j]].mean(0)
                    pts=np.vstack((x[points],newpoint));ids=np.r_[points,len(x)]
                else:pts=x[points];ids=points
                try:new=ids[Delaunay(pts).simplices]
                except QhullError:continue
                proposals.append((region,new,newpoint,'DelaunayInsertion' if newpoint is not None else 'DelaunayCavity'))
        used=set()
        for region,new,point,kind in proposals:
            if time.perf_counter()-start>105:break
            if used.intersection(region):continue
            ids=sorted(region);candidate_x=np.vstack((x,point)) if point is not None else x
            new=orient(candidate_x,new)
            counts['proposed']+=1
            if not invariant(candidate_x,t[ids],new):counts['boundary_or_exact_invariant_rejected']+=1;continue
            counts['quality_compared']+=1
            patch,pci=patch_interface(x,t,region,iface,ci,m);before,_=evaluate(x,t[ids],pci,patch);after,_=evaluate(candidate_x,new,pci,patch)
            ok=better(before,after);records.append(dict(sweep=sweep,kind=kind,cells=ids,new_cells=len(new),accepted=ok,before=before['failures'],after=after['failures']))
            if ok:
                # Publish one local search change, then rebuild indices. Never
                # apply stale cell ids after connectivity changes.
                t=np.vstack((np.delete(t,ids,axis=0),new));x=candidate_x;accepted+=1
                q,m=evaluate(x,t,ci,iface);break
        if time.perf_counter()-last>10:
            print(json.dumps(dict(stage=method,elapsed=time.perf_counter()-start,accepted=accepted,bad_cells=q['bad_cells'])),flush=True);last=time.perf_counter()
        if accepted==prior or time.perf_counter()-start>105:break
    q,m=evaluate(x,t,ci,iface);np.save(dest/'coordinates.npy',x);np.save(dest/'tets.npy',t)
    tags=np.arange(1,len(x)+1,dtype='i8')
    for key,z in dict(node_tags=tags,interface_tags=tags[ci],interface_owner=m['interface_owner'],farfield_tags=np.empty(0,dtype='i8'),tet_tags=np.arange(1,len(t)+1),connectivity_tags=tags[t]).items():np.save(dest/(key+'.npy'),z)
    np.savez_compressed(dest/'METRICS.npz',**m);save(dest/'QUALITY.json',q)
    info=psutil.Process().memory_info();result=dict(method=method,seconds=time.perf_counter()-start,counts=counts,accepted_changes=accepted,trials=records,quality=q,inserted_vertices=len(x)-len(x0),moved_vertices=int(np.any(x[:len(x0)]!=x0,axis=1).sum()),connectivity_equal=np.array_equal(t,t0),peak_rss_bytes=getattr(info,'peak_wset',info.rss),retained_rss_bytes=info.rss)
    save(dest/'TRIAL.json',result);print(json.dumps({k:z for k,z in result.items() if k!='trials'}),flush=True)
if __name__=='__main__':repair(*map(Path,sys.argv[1:4]),sys.argv[4])
