"""Bounded fixed-interface relocation using existing sparse local feasibility.

All full-core arrays are read-only inputs. Only measured failing vertex stars
enter the nonlinear model; publication requires independent frozen gates.
"""
from pathlib import Path
import sys
sys.dont_write_bytecode=True
import importlib.util,json,time,os
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash

ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'research/sdf_cfd_mesher_v4/results/fixed_interface_bulk_core_repair'
SRC=OUT/'source/sf2_core/third_standard'
IFACE=OUT/'source/sf2_interface'
V3=ROOT/'research/sdf_cfd_mesher_v3'


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m


def write(name,value):
    (OUT/name).write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')


def topology(tets):
    pats=np.array(((0,2,1),(0,1,3),(1,2,3),(2,0,3)))
    faces=tets[:,pats].reshape(-1,3);keys=np.sort(faces,axis=1)
    order=np.lexsort((keys[:,2],keys[:,1],keys[:,0]));sorted_keys=keys[order]
    starts=np.flatnonzero(np.r_[True,np.any(sorted_keys[1:]!=sorted_keys[:-1],axis=1)])
    counts=np.diff(np.r_[starts,len(order)])
    if np.any(counts>2):raise ValueError('NonmanifoldCore')
    first=order[starts[counts==2]];second=order[starts[counts==2]+1]
    return faces[first],np.column_stack((first//4,second//4))


def cell_metrics(x,tets,f):
    n=len(tets);vol=np.empty(n);det=np.empty(n);aspect=np.empty(n);centres=np.empty((n,3))
    for start in range(0,n,50000):
        sl=slice(start,min(n,start+50000));q=x[tets[sl]]
        vol[sl]=f.volume_np(q);det[sl]=f.determinant_np(q);centres[sl]=q.mean(axis=1)
        av=np.stack([.5*np.cross(q[:,b]-q[:,a],q[:,c]-q[:,a]) for a,b,c in f.PATS],axis=1)
        ss=np.abs(av).sum(axis=1)
        aspect[sl]=np.maximum(ss.max(axis=1)/(ss.min(axis=1)+1e-300),ss.sum(axis=1)/(6*np.maximum(vol[sl],1e-300)**(2/3)))
    return vol,det,aspect,centres


def face_metrics(x,faces,owners,vol,centres,fg):
    n=len(faces);weight=np.empty(n);ratio=np.empty(n);skew=np.empty(n);angle=np.empty(n)
    for start in range(0,n,50000):
        sl=slice(start,min(n,start+50000));q=x[faces[sl]];oa,ob=owners[sl].T
        area,fc=fg.face_area_centre_tri(q);a=centres[oa];b=centres[ob]
        da=np.abs(np.einsum('ij,ij->i',area,fc-a));db=np.abs(np.einsum('ij,ij->i',area,b-fc))
        weight[sl]=np.minimum(da,db)/(da+db);ratio[sl]=np.minimum(vol[oa],vol[ob])/np.maximum(vol[oa],vol[ob])
        skew[sl]=fg.internal_face_skewness(q,fc,area,a,b)
        angle[sl]=np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',area,b-a)/(np.linalg.norm(area,axis=1)*np.linalg.norm(b-a,axis=1)),-1,1)))
    return weight,ratio,skew,angle


def distance_surface(point,tri):
    a,b,c=tri[:,0],tri[:,1],tri[:,2];u=b-a;v=c-a;w=point-a
    uu=np.einsum('ij,ij->i',u,u);uv=np.einsum('ij,ij->i',u,v);vv=np.einsum('ij,ij->i',v,v)
    wu=np.einsum('ij,ij->i',w,u);wv=np.einsum('ij,ij->i',w,v);den=uu*vv-uv*uv
    s=(wu*vv-wv*uv)/den;t=(wv*uu-wu*uv)/den
    d=np.linalg.norm(w-s[:,None]*u-t[:,None]*v,axis=1);d[(s<0)|(t<0)|(s+t>1)]=np.inf
    for a,b in ((a,b),(b,c),(c,a)):
        edge=b-a;alpha=np.clip(np.einsum('ij,ij->i',point-a,edge)/np.einsum('ij,ij->i',edge,edge),0,1)
        d=np.minimum(d,np.linalg.norm(point-a-alpha[:,None]*edge,axis=1))
    return float(d.min())


def local_guard(core,inv,candidate,f,fg,boundaries):
    ids=np.union1d(inv['affected'],inv['guard_owners'])
    v,d,a,c=cell_metrics(candidate,core['tets'][ids],f)
    own=np.searchsorted(ids,inv['guard_owners'])
    w,r,s,n=face_metrics(candidate,inv['guard_faces'],own,v,c,fg)
    weights=[w];ratios=[r];skews=[s]
    for face_vertices,owner,bc,bv in boundaries:
        selected=np.flatnonzero(np.isin(owner,inv['affected']))
        if not len(selected):continue
        j=np.searchsorted(ids,owner[selected]);q=candidate[face_vertices[selected]]
        area,fc=fg.face_area_centre_tri(q)
        da=np.abs(np.einsum('ij,ij->i',area,fc-c[j]));db=np.abs(np.einsum('ij,ij->i',area,bc[selected]-fc))
        weights.append(np.minimum(da,db)/(da+db));ratios.append(np.minimum(v[j],bv[selected])/np.maximum(v[j],bv[selected]))
        skews.append(fg.internal_face_skewness(q,fc,area,c[j],bc[selected]))
    w=np.concatenate(weights);r=np.concatenate(ratios);s=np.concatenate(skews)
    if not all(np.isfinite(values).all() for values in (v,d,a,c,w,r,s)):
        raise ValueError('NonfiniteRepairQuality')
    failures=dict(volume=int(np.count_nonzero(v<=0)),determinant=int(np.count_nonzero(d<.0011)),weight=int(np.count_nonzero(w<.055)),ratio=int(np.count_nonzero(r<.01)),skew=int(np.count_nonzero(s>3.63636)),aspect=int(np.count_nonzero(a>1000)))
    return dict(pass_gate=not any(failures.values()),failures=failures,minimum=dict(volume=float(v.min()),determinant=float(d.min()),weight=float(w.min()),ratio=float(r.min())),maximum=dict(aspect=float(a.max()),skew=float(s.max())))


def main(label='A'):
    started=time.perf_counter()
    f=load('bulk_reference_feasibility',V3/'fresh_supported_refinement_family_qualification_2026-09-09/fallback_scope_20260910/source/solve_coordinate_feasibility.py')
    f.SKEW_MAX=3.63636
    fg=load('bulk_foundation',V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    core={k:np.load(SRC/(k+'.npy'),mmap_mode='r') for k in ['coordinates','tets','node_tags','interface_tags','farfield_tags','interface_owner','farfield_owner']}
    x=core['coordinates'];tets=core['tets'];order=np.argsort(core['node_tags']);tags=core['node_tags'][order]
    ci=order[np.searchsorted(tags,core['interface_tags'])];fi=order[np.searchsorted(tags,core['farfield_tags'])]
    fixed=np.union1d(ci,fi);core['interface_indices']=ci;core['farfield_indices']=fi
    faces,owners=topology(tets)
    vol,det,aspect,centres=cell_metrics(x,tets,f);weight,ratio,skew,angle=face_metrics(x,faces,owners,vol,centres,fg)
    bad_faces=np.flatnonzero(weight<.055);bad_det=np.flatnonzero(det<.0011)
    bad=np.union1d(bad_det,np.unique(owners[bad_faces]));bad_set=set(map(int,bad))
    graph={int(i):set() for i in bad}
    for a,b in owners[np.all(np.isin(owners,bad),axis=1)]:graph[int(a)].add(int(b));graph[int(b)].add(int(a))
    components=[];remaining=set(graph)
    while remaining:
        pending=[min(remaining)];component=set()
        while pending:
            i=pending.pop()
            if i not in component:component.add(i);pending.extend(graph[i]-component)
        remaining-=component;components.append(sorted(component))
    cap=np.load(IFACE/'connectivity.npy').reshape(-1,3)
    sx=np.load(IFACE/'points.npy');shell_tri=sx[cap]
    support=np.load(V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/support/SUPPORT.npz')
    support_tri=support['positions'][0][support['faces']]
    records=[]
    for i in bad:
        incident=np.flatnonzero(np.any(owners==i,axis=1));q=x[tets[i]]
        edges=[float(np.linalg.norm(q[a]-q[b])) for a in range(4) for b in range(a+1,4)]
        records.append(dict(cell=int(i),centroid=centres[i],vertices=tets[i],incident_faces=incident,volume=vol[i],determinant=det[i],minimum_weight=float(weight[incident].min()),minimum_ratio=float(ratio[incident].min()),maximum_skew=float(skew[incident].max()),aspect=aspect[i],maximum_nonorthogonality=float(angle[incident].max()),edge_lengths=edges,
            shell_distance=distance_surface(centres[i],shell_tri),support_distance=distance_surface(centres[i],support_tri),shell_adjacent=bool(i in core['interface_owner']),support_adjacent=bool(i in core['farfield_owner'])))
    baseline=dict(determinant=len(bad_det),weight_faces=len(bad_faces),weight_cells=len(np.unique(owners[bad_faces])),both_cells=len(np.intersect1d(bad_det,owners[bad_faces])),volume=int(np.count_nonzero(vol<=0)),skew=int(np.count_nonzero(skew>3.63636)),ratio=int(np.count_nonzero(ratio<.01)),aspect=int(np.count_nonzero(aspect>1000)),cells=records,components=components)
    if label=='A':write('FAILURES.json',baseline)
    # Existing local feasibility equations consume native interface owner geometry.
    auth=OUT/'local_authority';auth.mkdir(exist_ok=True)
    pc,pv=fg.prism_cell_centres(np.load(IFACE/'owner_geometry_6.npy'))
    support_centres,support_volumes=fg.prism_cell_centres(np.concatenate([support['positions'][0,support['faces']],support['positions'][1,support['faces']]],axis=1))
    boundaries=[(ci[cap],core['interface_owner'],pc,pv),(fi[support['faces']],core['farfield_owner'],support_centres,support_volumes)]
    area,fc=fg.face_area_centre_tri(shell_tri);normal=area/np.linalg.norm(area,axis=1)[:,None]
    for name,a in dict(outer_faces=cap,prism_centres=pc,cap_normals=normal,d_prism=np.einsum('ij,ij->i',fc-pc,normal)).items():np.save(auth/(name+'.npy'),a)
    f.AUTH=auth
    movable=set()
    for i in bad_det:movable.update(int(v) for v in tets[i] if v not in fixed)
    for i in bad_faces:
        shared=set(faces[i]);movable.update(int(v) for v in tets[owners[i]].ravel() if v not in shared and v not in fixed)
    movable=np.asarray(sorted(movable),dtype=np.int64)
    affected=np.flatnonzero(np.any(np.isin(tets,movable),axis=1))
    guard=np.any(np.isin(owners,affected),axis=1)
    scales=[]
    laplace_positions=[]
    for v in movable:
        star=np.flatnonzero(np.any(tets==v,axis=1));neighbors=np.setdiff1d(np.unique(tets[star]),[v]);scales.append(float(np.linalg.norm(x[neighbors]-x[v],axis=1).min()));laplace_positions.append(x[neighbors].mean(axis=0))
    inv=dict(movable=movable,affected=affected,guard_faces=faces[guard],guard_owners=owners[guard],guard_interface=np.flatnonzero(np.isin(core['interface_owner'],affected)),bad_interface=np.empty(0,dtype=np.int64),local_scale=np.asarray(scales))
    if np.intersect1d(movable,fixed).size:raise ValueError('FrozenVertexMarkedMovable')
    if label=='A':np.savez(OUT/'LOCAL_INVENTORY.npz',**inv)
    trials=[]
    for method in ['laplacian','constrained_movement','feasibility']:
        tic=time.perf_counter()
        if method=='laplacian':
            candidate=x.copy();candidate[movable]=np.asarray(laplace_positions);trial=dict(method=method)
        else:
            candidate,trial=f.solve_once(core,inv,1.0,'baseline','movement' if method=='constrained_movement' else 'feasibility')
            trial['method']=method
        guard_result=local_guard(core,inv,candidate,f,fg,boundaries)
        trial.update(seconds=time.perf_counter()-tic,guard=guard_result,fixed_exact=np.array_equal(candidate[fixed].view('u8'),x[fixed].view('u8')),moved=int(np.count_nonzero(np.any(candidate!=x,axis=1))),maximum_displacement=float(np.linalg.norm(candidate-x,axis=1).max()))
        trials.append(trial);write('TRIALS_'+label+'.json',trials)
        if guard_result['pass_gate'] and trial['fixed_exact']:
            final=OUT/('repaired_'+label);final.mkdir(exist_ok=True)
            np.save(final/'coordinates.npy',candidate)
            for name in ['tets','node_tags','interface_tags','farfield_tags','interface_owner','farfield_owner','connectivity_tags','tet_tags']:
                source=SRC/(name+'.npy');target=final/source.name
                if not target.exists():os.link(source,target)
            import psutil
            write('REPAIR_'+label+'.json',dict(passed_local=True,movable_vertices=len(movable),affected_cells=len(affected),guard_faces=int(guard.sum()),fixed_vertices=len(fixed),coordinates_hash=array_hash(candidate),connectivity_hash=array_hash(tets),seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset))
            print('LOCAL REPAIR PASSED',label,method,flush=True);return
        del candidate
    raise RuntimeError('LocalRelocationDidNotQualify; connectivity repair required')


if __name__=='__main__':main(sys.argv[1] if len(sys.argv)>1 else 'A')
