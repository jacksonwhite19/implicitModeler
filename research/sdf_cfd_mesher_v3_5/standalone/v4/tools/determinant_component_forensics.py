"""Dependency-local face-area tensor and finite-difference diagnostics."""
import itertools,time,json
import numpy as np
import psutil
from constrained_cavity import *
from fixed_core_study import determinant,distance_surface
from determinant_component_audit import OUT,SOURCE,INTERFACE,PRIOR

def ordered_components(x,t,m):
    return sorted(components(m['bad'],m['owners']),key=lambda c:(float(m['determinant'][c].min()),min(scientific_key(x[t[i]]) for i in c)))

def coordinate_support(x,t,ci,iface,m,seeds):
    """Close complete coordinate dependencies by face-connected growth.

    Only cells incident to a seed's movable vertex are admitted. This frees the
    selected vertices without growing unrelated isotropic face rings.
    """
    moving=np.setdiff1d(np.unique(t[seeds]),ci)
    required=set(np.flatnonzero(np.any(np.isin(t,moving),axis=1)).tolist())
    region=set(map(int,seeds));rounds=0
    while region!=required:
        adjacent=set(m['owners'][np.any(np.isin(m['owners'],list(region)),axis=1)].ravel().tolist())
        new=adjacent&required-region
        if not new:raise ValueError('DisconnectedCoordinateSupport')
        region|=new;rounds+=1
    data,row=cavity(x,t,ci,iface,m,sorted(region),0)
    inside=np.setdiff1d(data['global_vertices'],data['global_vertices'][data['boundary_vertices']])
    if not set(moving)<=set(inside):raise ValueError('CoordinateSupportNotClosed')
    row.update(initial_seeds=seeds,required_movable_vertices=moving,face_growth_rounds=rounds)
    return data,row

def sensitivity(q,movable):
    edges=np.array([np.linalg.norm(q[a]-q[b]) for a,b in itertools.combinations(range(4),2)])
    step=float(np.median(edges)*np.cbrt(np.finfo(float).eps));base=float(determinant(q[None])[0]);rows=[]
    for v in movable:
        for axis in range(3):
            p=q.copy();p[v,axis]+=step;plus=float(determinant(p[None])[0]);p=q.copy();p[v,axis]-=step;minus=float(determinant(p[None])[0])
            rows.append(dict(local_vertex=int(v),axis=axis,step=step,minus=minus,plus=plus,central_derivative=(plus-minus)/(2*step)))
    return dict(base=base,rows=rows)

def run():
    tick=time.perf_counter();out=OUT/'forensics';out.mkdir(exist_ok=False)
    x,t,ci,iface=load_core(SOURCE,INTERFACE);q,m=evaluate(x,t,ci,iface);ordered=ordered_components(x,t,m)
    # Select by scientific evidence, then verify against the preserved probe record.
    a=ordered[0];b=sorted([c for c in ordered if c!=a],key=lambda c:(-len(c),c[0]))[0]
    previous=json.loads((PRIOR/'remaining_probe/SELECTION.json').read_text())['selected']
    assert [a,b]==previous,'PreservedProbeMismatch'
    records=[]
    for label,seeds in zip(('A','B'),(a,b)):
        rows=[]
        for depth in (1,2,3,4,5):
            d,r=cavity(x,t,ci,iface,m,seeds,depth);np.savez_compressed(out/f'{label}_r{depth}.npz',**d);rows.append(r)
        d=dict(np.load(out/f'{label}_r3.npz'));bf=x[d['global_vertices'][d['f']]];cells=[]
        for i in seeds:
            v=t[i];z=x[v];av=np.array([.5*np.cross(z[b]-z[a],z[c]-z[a]) for a,b,c in PATS]);areas=np.linalg.norm(av,axis=1);normal=av/areas.mean();gram=normal.T@normal
            mov=np.flatnonzero(~np.isin(v,ci));ints=np.setdiff1d(d['global_vertices'],d['global_vertices'][d['boundary_vertices']])
            fs=np.flatnonzero(np.any(m['owners']==i,axis=1));shell=np.flatnonzero(m['interface_owner']==i)
            cells.append(dict(cell=int(i),scientific_key=scientific_key(z),vertices=v,coordinates=z,edges=[np.linalg.norm(z[a]-z[b]) for a,b in itertools.combinations(range(4),2)],face_areas=areas,face_area_vectors=av,normalized_face_gram=gram,eigenvalues=np.linalg.eigvalsh(gram),volume=m['volume'][i],determinant=m['determinant'][i],aspect=m['aspect'][i],interface_faces=shell,shell_owner_types=iface.kind[shell],neighbours=np.setdiff1d(m['owners'][fs],i),movable_global_vertices=v[mov],ring3_interior_vertices=ints,centre_boundary_distance=distance_surface(z.mean(0),bf),movable_boundary_distances={str(int(v[j])):distance_surface(z[j],bf) for j in mov},sensitivity=sensitivity(z,mov)))
        records.append(dict(label=label,seeds=seeds,cavities=rows,cells=cells))
    descriptions=[]
    for ids in ordered:
        vs=np.unique(t[ids]);edges=np.concatenate([np.linalg.norm(x[t[i,a]]-x[t[i,b]])[None] for i in ids for a,b in itertools.combinations(range(4),2)])
        descriptions.append(dict(cells=ids,size=len(ids),minimum_determinant=float(m['determinant'][ids].min()),interface_adjacent=int(np.isin(ids,m['interface_owner']).sum()),movable_vertices=int((~np.isin(vs,ci)).sum()),edge_anisotropy=float(edges.max()/edges.min()),key=min(scientific_key(x[t[i]]) for i in ids)))
    save(out/'PROBES.json',records);save(out/'COMPONENTS.json',descriptions);save(out/'BASELINE.json',q)
    save(out/'PERFORMANCE.json',dict(seconds=time.perf_counter()-tick,peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss))
    print(json.dumps(dict(identity=q['identity'],failures=q['failures'],components=len(ordered),probes=[dict(label=r['label'],cells=r['seeds'],geometry=[{k:c[k] for k in ('cell','determinant','eigenvalues','edges','movable_global_vertices','centre_boundary_distance')} for c in r['cells']],cavities=r['cavities']) for r in records]),default=lambda a:a.tolist()),flush=True)

if __name__=='__main__':run()
