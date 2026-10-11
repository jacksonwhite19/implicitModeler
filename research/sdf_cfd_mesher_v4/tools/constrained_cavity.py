"""Exact-boundary local cavity contracts and deterministic Steiner proposals.

Research only. Proposals cannot publish unless all invariant, quality, and
continuous-geometry checks pass. No case, count, or cell-id special cases.
"""
import sys, time, hashlib, itertools
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.optimize import linprog
from fixed_core_study import load_core,evaluate,save,components,PATS,volume,identity
from fixed_core_topology_repair import boundary,orient,exact_volume,patch_interface

def scientific_key(q):
    return hashlib.sha256(b'V4TetCoordinateSet/1\0'+b''.join(sorted(np.asarray(p,dtype='<f8').tobytes() for p in q))).hexdigest()

def oriented(fs):return {min(tuple(np.roll(f,k)) for k in range(3)) for f in fs}

def invariants(x,old,new):
    """Detailed diagnostics, including duplicates and internal orientation."""
    out=dict(duplicate_tets=len(new)-len(np.unique(np.sort(new,axis=1),axis=0)))
    rows=new[:,PATS].reshape(-1,3)
    keys,inv,n=np.unique(np.sort(rows,1),axis=0,return_inverse=True,return_counts=True)
    out['nonmanifold_faces']=int((n>2).sum())
    oldb=boundary(old);nb=rows[np.array([np.flatnonzero(inv==j)[0] for j in np.flatnonzero(n==1)],dtype=int)]
    a={tuple(sorted(f)) for f in oldb};b={tuple(sorted(f)) for f in nb}
    out.update(missing_boundary=sorted(a-b),extra_boundary=sorted(b-a),boundary_orientation_equal=oriented(oldb)==oriented(nb))
    wrong=[]
    for j in np.flatnonzero(n==2):
        f,g=rows[inv==j]
        if oriented([f])!=oriented([g[::-1]]):wrong.append(keys[j].tolist())
    out['same_orientation_internal_faces']=wrong
    ev=[exact_volume(q) for q in x[new]];v0=sum(exact_volume(q) for q in x[old])
    out.update(nonpositive_exact_tets=sum(v<=0 for v in ev),old_exact_volume=str(v0),new_exact_volume=str(sum(ev)),exact_volume_equal=sum(ev)==v0)
    out['passed']=not(out['duplicate_tets'] or out['nonmanifold_faces'] or out['missing_boundary'] or out['extra_boundary'] or wrong or out['nonpositive_exact_tets']) and out['boundary_orientation_equal'] and out['exact_volume_equal']
    return out

def strict_inside_tet(p,q):
    v=exact_volume(q)
    if not v:return False
    parts=[]
    for i in range(4):r=q.copy();r[i]=p;parts.append(exact_volume(r))
    return all(z*v>0 for z in parts) and sum(parts)==v

def strict_inside(p,x,t):return any(strict_inside_tet(p,x[z]) for z in t)

def interior_point_valid(p,x,t):
    """Admit a point in a previously validated cavity.

    A source interior vertex belongs to several closed tets, not the strict
    interior of any one tet. Its complete source star and absence from the
    exact cavity boundary establish its inherited interior status. New points
    still require strict exact inclusion in an original tet.
    """
    same=np.flatnonzero(np.all(np.asarray(x).view('u8')==np.asarray(p).view('u8'),axis=1))
    if len(same):
        exterior=np.unique(boundary(t))
        if np.intersect1d(same,exterior).size:return False
        return bool(np.intersect1d(same,np.unique(t)).size)
    return strict_inside(p,x,t)

def kernel_point(x,faces):
    """Chebyshev centre of the visibility kernel; exact signs decide validity.

    LP gives a candidate only. It never proves a geometric predicate.
    """
    q=x[faces];origin=q.reshape(-1,3).mean(0);scale=np.max(np.ptp(q.reshape(-1,3),axis=0));r=(q-origin)/scale
    n=np.cross(r[:,1]-r[:,0],r[:,2]-r[:,0]);n/=np.linalg.norm(n,axis=1)[:,None]
    b=np.einsum('ij,ij->i',n,r[:,0]);sol=linprog([0,0,0,-1],A_ub=np.c_[n,np.ones(len(n))],b_ub=b,bounds=[(None,None)]*4,method='highs')
    if not sol.success:return None,dict(solver=sol.message,strict_kernel=False)
    p=origin+scale*sol.x[:3]
    valid=all(exact_volume(np.vstack((p,z)))>0 for z in q)
    return p,dict(solver=sol.message,radius=float(sol.x[3]*scale),strict_kernel=valid)

def cavity(x,t,ci,iface,m,seed,depth):
    region=set(map(int,np.atleast_1d(seed)))
    for _ in range(depth):region.update(map(int,m['owners'][np.any(np.isin(m['owners'],list(region)),axis=1)].ravel()))
    ids=sorted(region);bf=boundary(t[ids]);vs=np.unique(t[ids]);bvs=np.unique(bf);interior=np.setdiff1d(vs,bvs)
    patch,pci=patch_interface(x,t,region,iface,ci,m)
    shell=np.flatnonzero(np.isin(m['interface_owner'],ids))
    # A compact input preserves exact binary64 coordinates and oriented facets.
    xx=x[vs];tt=np.searchsorted(vs,t[ids]);ff=np.searchsorted(vs,bf)
    return dict(x=xx,t=tt,f=ff,global_vertices=vs,global_cells=np.array(ids),boundary_vertices=np.searchsorted(vs,bvs),shell_faces=shell,
                shell_frozen=np.flatnonzero(np.isin(vs,ci)),guard_centres=patch.c,guard_volumes=patch.v,guard_order=np.searchsorted(vs,pci)),dict(seed=int(seed) if np.ndim(seed)==0 else list(map(int,seed)),depth=depth,cells=len(ids),vertices=len(vs),boundary_facets=len(bf),boundary_vertices=len(bvs),shell_facets=len(shell),shell_frozen_vertices=int(np.isin(vs,ci).sum()),non_shell_boundary_vertices=int((~np.isin(bvs,ci)).sum()),movable_interior_vertices=len(interior),exact_volume=str(sum(exact_volume(q) for q in x[t[ids]])),boundary_sha256=hashlib.sha256(np.asarray(bf,dtype='<i8').tobytes()).hexdigest())

def patch_from_input(data):
    from fixed_core_study import Interface,FG
    p=Interface.__new__(Interface);ids=data['guard_order'];p.x=data['x'][ids];p.f=np.searchsorted(ids,data['f']);p.c=data['guard_centres'];p.v=data['guard_volumes'];p.core_boundary_sign=1
    p.area,p.fc=FG.face_area_centre_tri(p.x[p.f]);return p,ids

def quality(data,x,t):
    p,ci=patch_from_input(data)
    return evaluate(x,t,ci,p)

def extract(src,interface,dest):
    tick=time.perf_counter();dest.mkdir(parents=True,exist_ok=True);x,t,ci,iface=load_core(src,interface);q,m=evaluate(x,t,ci,iface)
    frozen=m['bad'][np.all(np.isin(t[m['bad']],ci),axis=1)];comps=components(m['bad'],m['owners']);records=[];cavs=[]
    for seed in frozen:
        seed=int(seed);fs=np.flatnonzero(np.any(m['owners']==seed,axis=1));ifs=np.flatnonzero(m['interface_owner']==seed)
        record=dict(cell=seed,key=scientific_key(x[t[seed]]),vertices=t[seed],coordinates=x[t[seed]],shell_faces=ifs,shell_owner_types=iface.kind[ifs],neighbours=np.setdiff1d(m['owners'][fs],seed),determinant=m['determinant'][seed],volume=m['volume'][seed],aspect=m['aspect'][seed],weight=m['weight'][fs],ratio=m['ratio'][fs],skew=m['skew'][fs],interface_weight=m['interface_weight'][ifs],interface_ratio=m['interface_ratio'][ifs],bad_component=next(z for z in comps if seed in z))
        records.append(record)
        for depth in range(4):
            data,row=cavity(x,t,ci,iface,m,seed,depth);name=f'star_{seed}_ring_{depth}';np.savez_compressed(dest/(name+'.npz'),**data);row['file']=name+'.npz';cavs.append(row)
    graph=[]
    for a,b in itertools.combinations(map(int,frozen),2):graph.append(dict(a=a,b=b,shared_vertices=np.intersect1d(t[a],t[b]),shared_faces=len(np.intersect1d(t[a],t[b]))==3))
    overlaps=[]
    for a,b in itertools.combinations(cavs,2):
        if a['seed']==b['seed'] or a['depth']!=b['depth']:continue
        aa=np.load(dest/a['file']);bb=np.load(dest/b['file']);common=np.intersect1d(aa['global_cells'],bb['global_cells']);bv=np.intersect1d(aa['global_vertices'][aa['boundary_vertices']],bb['global_vertices'][bb['boundary_vertices']])
        overlaps.append(dict(a=a['seed'],b=b['seed'],depth=a['depth'],shared_cells=common,shared_boundary_vertices=bv))
    save(dest/'FROZEN_STARS.json',records);save(dest/'CAVITIES.json',cavs);save(dest/'CONNECTIVITY.json',dict(stars=graph,cavities=overlaps));save(dest/'BASELINE.json',q)
    save(dest/'EXTRACTION.json',dict(seconds=time.perf_counter()-tick,frozen_cells=frozen,identity=identity(x,t)));print(dict(frozen=frozen.tolist(),quality=q,cavities=cavs),flush=True)

if __name__=='__main__':extract(*map(Path,sys.argv[1:4]))
