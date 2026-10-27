"""Fixed-interface tet diagnostics. Uses the existing frozen metric definitions."""
import sys, json, time, hashlib
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from types import SimpleNamespace
from scipy.spatial import cKDTree
from repair_bulk_core import load,V3,topology,cell_metrics,face_metrics,distance_surface
from audit_mixed_bulk_probe import centres

PATS=np.array(((0,2,1),(0,1,3),(1,2,3),(2,0,3)))
FG=load('fixed_core_foundation',V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
def volume(q):return np.einsum('...i,...i->...',q[...,1,:]-q[...,0,:],np.cross(q[...,2,:]-q[...,0,:],q[...,3,:]-q[...,0,:]))/6.0
def determinant(q):
    av=np.stack([.5*np.cross(q[:,b]-q[:,a],q[:,c]-q[:,a]) for a,b,c in PATS],axis=1)
    av=av/np.linalg.norm(av,axis=2).mean(1)[:,None,None]
    return np.linalg.det(np.einsum('nfi,nfj->nij',av,av))
METRIC=SimpleNamespace(volume_np=volume,determinant_np=determinant,PATS=PATS)
def save(path,obj):
    Path(path).write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=False,default=lambda x:x.tolist() if hasattr(x,'tolist') else x.item())+'\n')
def identity(x,t):
    h=hashlib.sha256(b'V4FixedInterfaceCoreArrays/1\0');h.update(np.asarray(x,dtype='<f8').tobytes());h.update(np.asarray(t,dtype='<i8').tobytes());return h.hexdigest()
def load_core(folder,interface):
    x=np.load(folder/'coordinates.npy');t=np.load(folder/'tets.npy');tags=np.load(folder/'node_tags.npy');it=np.load(folder/'interface_tags.npy');order=np.argsort(tags)
    ci=order[np.searchsorted(tags[order],it)]
    return x,t,ci,Interface(interface)
class Interface:
    def __init__(self,path):
        self.x=np.load(path/'points.npy');self.f=np.load(path/'connectivity.npy').reshape(-1,3)
        self.c=np.empty((len(self.f),3));self.v=np.empty(len(self.f));self.kind=np.empty(len(self.f),dtype='U12')
        for p in sorted(path.glob('owner_geometry_*.npy')):
            arity=p.stem.rsplit('_',1)[1];ids=np.load(path/f'owner_faces_{arity}.npy')
            self.c[ids],self.v[ids]=centres(np.load(p),FG);self.kind[ids]={'4':'tetrahedron','5':'pyramid','6':'prism'}[arity]
        self.area,self.fc=FG.face_area_centre_tri(self.x[self.f])
    def mapping(self,x,t,ci):
        if not np.array_equal(x[ci].view('u8'),self.x.view('u8')):raise ValueError('FrozenInterfaceMoved')
        rows=t[:,PATS].reshape(-1,3);keys=np.sort(rows,1)
        un,first,count=np.unique(keys,axis=0,return_index=True,return_counts=True)
        if np.any(count>2):raise ValueError('NonManifoldCore')
        boundary={tuple(k):int(first[j]//4) for j,k in enumerate(un) if count[j]==1}
        wanted=[tuple(sorted(f)) for f in ci[self.f]]
        if len(set(wanted))!=len(wanted) or set(boundary)!=set(wanted):raise ValueError('FrozenInterfaceConnectivityChanged')
        def oriented(fs):return {min(tuple(np.roll(f,k)) for k in range(3)) for f in fs}
        expected=ci[self.f]
        if getattr(self,'core_boundary_sign',-1)<0:expected=expected[:,::-1]
        actual=rows[first[count==1]]
        if oriented(actual)!=oriented(expected):raise ValueError('FrozenInterfaceOrientationChanged')
        return np.array([boundary[k] for k in wanted])
def evaluate(x,t,ci,interface):
    own=interface.mapping(x,t,ci);faces,owners=topology(t)
    v,d,a,c=cell_metrics(x,t,METRIC);w,r,s,n=face_metrics(x,faces,owners,v,c,FG)
    # Preserve the existing core adapter's binary64 interface-centroid path.
    tc,tv=centres(x[t],FG)
    da=abs(np.einsum('ij,ij->i',interface.area,interface.fc-interface.c));db=abs(np.einsum('ij,ij->i',interface.area,tc[own]-interface.fc))
    iw=np.minimum(da,db)/(da+db);ir=np.minimum(interface.v,tv[own])/np.maximum(interface.v,tv[own]);isk=FG.internal_face_skewness(x[ci[interface.f]],interface.fc,interface.area,interface.c,tc[own])
    fail=dict(volume=np.flatnonzero(v<=0),determinant=np.flatnonzero(d<.0011),aspect=np.flatnonzero(a>1000),weight=np.flatnonzero(w<.055),ratio=np.flatnonzero(r<.01),skew=np.flatnonzero(s>3.63636),interface_weight=np.flatnonzero(iw<.055),interface_ratio=np.flatnonzero(ir<.01),interface_skew=np.flatnonzero(isk>3.63636))
    bad=np.unique(np.r_[fail['volume'],fail['determinant'],fail['aspect'],owners[np.unique(np.r_[fail['weight'],fail['ratio'],fail['skew']])].ravel(),own[np.unique(np.r_[fail['interface_weight'],fail['interface_ratio'],fail['interface_skew']])]])
    if not all(np.isfinite(z).all() for z in (v,d,a,w,r,s,iw,ir,isk)):raise ValueError('NonfiniteQuality')
    summary=dict(tets=len(t),vertices=len(x),interior_vertices=len(x)-len(ci),bad_cells=len(bad),failures={k:len(z) for k,z in fail.items()},minimum=dict(volume=float(v.min()),determinant=float(d.min()),weight=float(w.min()),ratio=float(r.min()),interface_weight=float(iw.min()),interface_ratio=float(ir.min())),maximum=dict(aspect=float(a.max()),skew=float(s.max()),nonorthogonality=float(n.max()),interface_skew=float(isk.max())),interface_exact=True,identity=identity(x,t),passed=not len(bad))
    return summary,dict(volume=v,determinant=d,aspect=a,centres=c,weight=w,ratio=r,skew=s,nonorthogonality=n,interface_weight=iw,interface_ratio=ir,interface_skew=isk,faces=faces,owners=owners,interface_owner=own,bad=bad,**{'failed_'+k:z for k,z in fail.items()})
def components(bad,owners):
    graph={int(i):set() for i in bad}
    for a,b in owners:
        if int(a) in graph and int(b) in graph:graph[int(a)].add(int(b));graph[int(b)].add(int(a))
    remaining=set(graph);result=[]
    while remaining:
        todo=[min(remaining)];seen=set()
        while todo:
            i=todo.pop()
            if i not in seen:seen.add(i);todo.extend(graph[i]-seen)
        remaining-=seen;result.append(sorted(seen))
    return sorted(result,key=lambda z:(-len(z),z[0]))
def quantiles(z):return dict(zip(('min','q25','median','q75','max'),map(float,np.quantile(z,[0,.25,.5,.75,1])))) if len(z) else None
def local_sizes(iface):
    """Boundary-area and altitude sizing, independent of mesher runtime."""
    q=iface.x[iface.f];edges=np.linalg.norm(q-np.roll(q,1,axis=1),axis=2)
    area=np.linalg.norm(iface.area,axis=1);h=np.minimum(np.sqrt(4*area/np.sqrt(3)),2*area/edges.max(1))
    vertex=np.full(len(iface.x),np.inf);np.minimum.at(vertex,iface.f.ravel(),np.repeat(h,3))
    return vertex,h
def forensics(folder,interface,dest):
    start=time.perf_counter();dest.mkdir(parents=True,exist_ok=True);x,t,ci,iface=load_core(folder,interface);q,m=evaluate(x,t,ci,iface)
    np.savez_compressed(dest/'METRICS.npz',**m);save(dest/'QUALITY.json',q)
    own=m['interface_owner'];bad=m['bad'];fixed=set(map(int,ci));records=[]
    for ids in components(bad,m['owners']):
        ix=np.flatnonzero(np.isin(own,ids));vs=np.unique(t[ids]);adj=np.intersect1d(ids,own);kinds={str(k):int(np.sum(iface.kind[ix]==k)) for k in np.unique(iface.kind)}
        records.append(dict(cells=ids,count=len(ids),interface_faces=ix,interface_face_count=len(ix),fixed_vertices=int(np.isin(vs,ci).sum()),movable_vertices=int((~np.isin(vs,ci)).sum()),interface_adjacent_cells=len(adj),noninterface_adjacent_cells=len(ids)-len(adj),shell_types=kinds,bounds=[m['centres'][ids].min(0),m['centres'][ids].max(0)],location='mixed' if len(adj) and len(adj)<len(ids) else 'shell-interface adjacent' if len(adj) else 'non-interface adjacent',transition_adjacent=bool(kinds.get('pyramid',0)+kinds.get('tetrahedron',0))))
    save(dest/'COMPONENTS.json',records)
    corr={}
    for k in np.unique(iface.kind):
        ix=np.flatnonzero(iface.kind==k);corr[str(k)]=dict(faces=len(ix),bad_adjacent_cells=len(np.intersect1d(own[ix],bad)),weight_failures=int((m['interface_weight'][ix]<.055).sum()),ratio_failures=int((m['interface_ratio'][ix]<.01).sum()),skew_failures=int((m['interface_skew'][ix]>3.63636).sum()),determinant_failed_owners=int((m['determinant'][own[ix]]<.0011).sum()))
    interior=np.setdiff1d(np.arange(len(x)),ci);edges=np.array([(a,b) for a in range(4) for b in range(a+1,4)])
    ie=np.linalg.norm(iface.x[iface.f]-iface.x[np.roll(iface.f,1,axis=1)],axis=2)
    te=np.linalg.norm(x[t[:,edges[:,0]]]-x[t[:,edges[:,1]]],axis=2)
    nearest=cKDTree(x[interior]).query(iface.fc)[0]
    sizing=dict(interface_edges=quantiles(ie.ravel()),tet_edges=quantiles(te.ravel()),bad_tet_edges=quantiles(te[bad].ravel()),face_to_nearest_interior_vertex=quantiles(nearest),interface_triangle_area=quantiles(np.linalg.norm(iface.area,axis=1)),bad_cells_all_vertices_fixed=int(np.all(np.isin(t[bad],ci),axis=1).sum()),bad_interface_faces_all_owner_vertices_fixed=int(np.all(np.isin(t[own[np.unique(np.r_[m['failed_interface_weight'],m['failed_interface_ratio'],m['failed_interface_skew']])]],ci),axis=1).sum()))
    reps=np.unique(np.r_[np.argmin(m['determinant']),own[np.argmin(m['interface_weight'])],[z['cells'][0] for z in records[:8]]])
    examples=[]
    for i in reps:
        fs=np.flatnonzero(np.any(m['owners']==i,axis=1));ifs=np.flatnonzero(own==i);vs=t[i];area=np.linalg.norm(FG.face_area_centre_tri(x[vs[PATS]])[0],axis=1)
        examples.append(dict(cell=int(i),vertices=vs,centre=m['centres'][i],volume=m['volume'][i],determinant=m['determinant'][i],aspect=m['aspect'][i],weight_min=float(m['weight'][fs].min()) if len(fs) else None,ratio_min=float(m['ratio'][fs].min()) if len(fs) else None,skew_max=float(m['skew'][fs].max()) if len(fs) else None,nonorthogonality_max=float(m['nonorthogonality'][fs].max()) if len(fs) else None,edges=te[i],face_areas=area,interior_vertex_surface_distances=[distance_surface(x[v],iface.x[iface.f]) for v in vs if v not in fixed],interface_faces=ifs,interface_owner_types=iface.kind[ifs]))
    result=dict(quality=q,component_count=len(records),largest_component=records[0]['count'],interface_adjacent_bad_cells=len(np.intersect1d(bad,own)),noninterface_adjacent_bad_cells=len(np.setdiff1d(bad,own)),face_type_correlation=corr,sizing=sizing,representatives=examples,curvature='Not measured',medial_axis_classification='Not measured',seconds=time.perf_counter()-start)
    save(dest/'FORENSICS.json',result);print(json.dumps({k:z for k,z in result.items() if k!='representatives'},default=lambda z:z.tolist()),flush=True)
    return result
if __name__=='__main__':forensics(*map(Path,sys.argv[1:4]))
