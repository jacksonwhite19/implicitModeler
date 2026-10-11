"""Adapters for preserved geometry-only primitives. No manufactured field is evaluated.

Fan face geometry, area tensor and separation are the preserved geometry.py
expressions. Foundation cell centres, exact wedge extrema, determinant masks,
internal face measures and constrained gain call unchanged frozen functions.
"""
import numpy as np
from ...runtime.frozen_imports import legacy_modules,operator_module
from .patch import prisms,require_closure

ROLES=([0,2,1],[3,4,5],[0,1,4,3],[1,2,5,4],[2,0,3,5])

def cell_metrics(p,layers,*,required_gain_only=False,conditioning=True,internal_caps=False):
    m=legacy_modules();cq=m['component_quality'];fg=cq.FG;pm=cq.PM
    sf=[];cf=[]
    # These statements preserve geometry.py's polygon fan expression/order.
    for ids in ROLES:
        vs=p[:,ids];mid=vs.mean(1);nxt=np.roll(vs,-1,axis=1)
        st=.5*np.cross(vs-mid[:,None],nxt-mid[:,None]);ct=(vs+nxt+mid[:,None])/3
        S=st.sum(1);normal=S/np.linalg.norm(S,axis=1)[:,None]
        an=np.einsum('nfi,ni->nf',st,normal);C=np.einsum('nf,nfi->ni',an,ct)/an.sum(1)[:,None]
        sign=np.where(np.einsum('ni,ni->n',S,C-p.mean(1))>=0,1.,-1.)
        sf.append(S*sign[:,None]);cf.append(C)
    sf=np.stack(sf,1);cf=np.stack(cf,1);op=operator_module()
    wall=np.zeros((len(p),5),bool);wall[:,0]=layers==1
    # Sigma is defined by the preserved weighted face matrix even when the
    # production normal-equation solve is singular. Never use a pseudoinverse.
    area=np.linalg.norm(sf,axis=-1);N=sf/area[...,None];W=area/area.sum(-1)[:,None]
    diagnostics={}
    # G_bound is the only consumed production gain. The specialized path uses
    # the identical frozen expression and preserves the singular-cell fallback.
    def bound(sf,wall):
        m=op.matrices(sf,wall)
        return np.linalg.svd(m['B']@m['Pb'],compute_uv=False)[:,0] if required_gain_only else op.gains(m)['G_bound']
    if conditioning:
        sw=np.linalg.svd(N*np.sqrt(W)[...,None],compute_uv=False)
        sigma=sw[:,-1];gain=np.full(len(p),np.nan);gain_available=np.ones(len(p),bool)
        try:gain[:]=bound(sf,wall)
        except np.linalg.LinAlgError:
            for i in range(len(p)):
                try:gain[i]=bound(sf[i:i+1],wall[i:i+1])[0]
                except np.linalg.LinAlgError:gain_available[i]=False
        diagnostics=dict(sigma=sigma,G_bound=gain,G_bound_available=gain_available,G_weighted=1/sigma,condition=sw[:,0]/sigma)
    C,V=fg.prism_cell_centres(p);jac,vol=pm._exact_wedge_jacobian_and_volume(p)
    determinant=np.empty(len(p))
    for layer in np.unique(layers):
        selected=layers==layer
        determinant[selected]=(pm._foundation_determinant(p[selected],lower_cap_is_uncoupled_wall=int(layer)==1)
                               if internal_caps else cq.determinant(p[selected],int(layer)))
    d=p[:,3:].mean(1)-p[:,:3].mean(1);d/=np.linalg.norm(d,axis=1)[:,None]
    pn=N[:,2:]-np.einsum('nfi,ni->nf',N[:,2:],d)[:,:,None]*d[:,None]
    pn/=np.linalg.norm(pn,axis=2)[:,:,None]
    sep=np.minimum.reduce([np.degrees(np.arccos(np.clip(abs(np.einsum('ni,ni->n',pn[:,i],pn[:,j])),-1,1))) for i,j in [(0,1),(1,2),(0,2)]])
    B=np.abs(sf).sum(1);aspect=np.maximum(B.max(1)/B.min(1),B.sum(1)/(6*np.maximum(V,1e-300)**(2/3)))
    return dict(centre=C,volume=V,signed_volume=vol,jacobian=jac,determinant=determinant,aspect=aspect,
                **diagnostics,side_normal_separation_degrees=sep)

def measure(front,forecast,topology,patch,*,domain=None):
    require_closure(patch,topology);x,p=prisms(front,forecast,topology,patch,domain=domain)
    nc=len(patch.columns);absolute=np.asarray(forecast.layers);nh=len(absolute);layers=np.repeat(absolute,nc)
    g=cell_metrics(p.reshape(-1,6,3),layers,internal_caps=domain is not None)
    g={k:v.reshape((nh,nc)+v.shape[1:]) for k,v in g.items()}
    core=np.searchsorted(patch.columns,patch.core_columns);fg=legacy_modules()['component_quality'].FG
    cq=legacy_modules()['component_quality'];faces=[];metrics={k:[] for k in ('weight','skewness','symmetric_volume_ratio','nonorthogonality')}
    previous=None
    if domain is not None and front.committed_layer>0:
        local=np.searchsorted(patch.vertices,topology.triangles[patch.columns])
        low=domain.prefix[-2,patch.vertices];high=domain.prefix[-1,patch.vertices]
        previous=cell_metrics(np.concatenate((low[local],high[local]),axis=1),np.full(nc,front.committed_layer),internal_caps=True)
    for k in range(nh):
        ed=topology.edges[patch.side_edges];local=np.searchsorted(patch.vertices,ed)
        vs=np.stack((x[k,local[:,0]],x[k,local[:,1]],x[k+1,local[:,1]],x[k+1,local[:,0]]),axis=1)
        own=np.array([topology.edge_triangles.row(e) for e in patch.side_edges]);ix=np.searchsorted(patch.columns,own)
        S,Cf=fg.face_area_centre_quad(vs)
        val=cq.face_metrics(vs,S,Cf,g['centre'][k,ix[:,0]],g['centre'][k,ix[:,1]],g['volume'][k,ix[:,0]],g['volume'][k,ix[:,1]])
        faces.extend(('side',int(absolute[k]),int(e),int(a),int(b)) for e,(a,b) in zip(patch.side_edges,own))
        for key,v in zip(metrics,val):metrics[key].extend(v)
        if k>0 or previous is not None:
            f=np.searchsorted(patch.vertices,topology.triangles[patch.core_columns]);vs=x[k,f];S,Cf=fg.face_area_centre_tri(vs)
            val=cq.face_metrics(vs,S,Cf,(g['centre'][k-1,core] if k>0 else previous['centre'][core]),g['centre'][k,core],(g['volume'][k-1,core] if k>0 else previous['volume'][core]),g['volume'][k,core])
            faces.extend(('cap',int(absolute[k])-1,int(c),int(c),int(c)) for c in patch.core_columns)
            for key,v in zip(metrics,val):metrics[key].extend(v)
    cells={k:v[:,core] for k,v in g.items()}
    cells['column']=np.broadcast_to(patch.core_columns,(nh,len(core))).copy()
    cells['layer']=np.broadcast_to(absolute[:,None],(nh,len(core))).copy()
    cells['cell_id']=(cells['layer']-1)*len(topology.triangles)+cells['column']
    # Boundary wall faces do not have a neighbor. The provisional top has no core
    # mesh: its neighbor-dependent metrics are explicitly unavailable.
    return dict(cells=cells,face_ids=faces,faces={k:np.asarray(v) for k,v in metrics.items()},
                unavailable=dict(outer_cap_neighbor_metrics=len(core),reason='No exterior core mesh exists in the provisional forecast'),
                counts=dict(core_vertices=len(patch.core_vertices),halo_vertices=len(patch.vertices)-len(patch.core_vertices),columns=nc,
                            core_columns=len(core),prisms=nh*nc,**(dict(committed_context_prisms=nc if previous is not None else 0) if domain is not None else {}),measured_prisms=nh*len(core),faces=len(faces),reconstruction_rows=5*nh*len(core),sensitivity_dofs=0))
