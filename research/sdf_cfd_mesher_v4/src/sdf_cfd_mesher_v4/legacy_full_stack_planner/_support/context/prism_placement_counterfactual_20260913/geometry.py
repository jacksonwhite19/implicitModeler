from common import *
import numpy as np,importlib.util
V3=H.parent
sys.path.insert(0,str(V3/'foundation_aware_prism_shell_qualification'))
import foundation_geometry as FG
s=importlib.util.spec_from_file_location('placement_predicates',V3/'prism_feasibility_contract/prospective_commit_adapter.py');PM=importlib.util.module_from_spec(s);sys.modules[s.name]=PM;s.loader.exec_module(PM)
ROLES=([0,2,1],[3,4,5],[0,1,4,3],[1,2,5,4],[2,0,3,5])

def cell_geometry(p,direction=None):
 sf=[];cf=[];flux=[];warp=[];field=read(G/'FIELD_AUTHORITY.json');a=np.array(field['a_m_per_s']);Hm=np.array(field['H_per_s'])
 for ids in ROLES:
  vs=p[:,ids];mid=vs.mean(1);nxt=np.roll(vs,-1,axis=1);st=.5*np.cross(vs-mid[:,None],nxt-mid[:,None]);ct=(vs+nxt+mid[:,None])/3;S=st.sum(1);normal=S/np.linalg.norm(S,axis=1)[:,None];an=np.einsum('nfi,ni->nf',st,normal);C=np.einsum('nf,nfi->ni',an,ct)/an.sum(1)[:,None];q=np.einsum('nfi,nfi->n',st,a+ct@Hm.T)
  sign=np.where(np.einsum('ni,ni->n',S,C-p.mean(1))>=0,1.,-1.);sf.append(S*sign[:,None]);cf.append(C);flux.append(q*sign);warp.append(abs(np.einsum('nfi,ni->nf',vs-C[:,None],normal)).max(1))
 Sf=np.stack(sf,1);Cf=np.stack(cf,1);q=np.stack(flux,1);areas=np.linalg.norm(Sf,axis=2);N=Sf/areas[:,:,None];W=areas/areas.sum(1)[:,None];cest=Cf.mean(1);pyr3=np.einsum('nfi,nfi->nf',Sf,Cf-cest[:,None]);V=pyr3.sum(1)/3;C=np.einsum('nf,nfi->ni',pyr3,.75*Cf+.25*cest[:,None])/(3*V[:,None]);L,S,Vt=np.linalg.svd(N*np.sqrt(W)[:,:,None],full_matrices=False);Su=np.linalg.svd(N/np.sqrt(5),compute_uv=False)
 M=np.einsum('nf,nfi,nfj->nij',areas,N,N);U=np.linalg.solve(M,np.einsum('nfi,nf->ni',N,q)[...,None])[...,0];exact=a+C@Hm.T;error=U-exact
 d=np.array(direction) if direction is not None else p[:,3:].mean(1)-p[:,:3].mean(1);d/=np.linalg.norm(d,axis=1)[:,None];pn=N[:,2:]-np.einsum('nfi,ni->nf',N[:,2:],d)[:,:,None]*d[:,None];pn/=np.linalg.norm(pn,axis=2)[:,:,None];ss=np.linalg.svd(pn/np.sqrt(3),compute_uv=False);sep=np.minimum.reduce([np.degrees(np.arccos(np.clip(abs(np.einsum('ni,ni->n',pn[:,i],pn[:,j])),-1,1))) for i,j in [(0,1),(1,2),(0,2)]])
 jac,vol=PM._exact_wedge_jacobian_and_volume(p);ori=np.einsum('ni,ni->n',np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),np.cross(p[:,4]-p[:,3],p[:,5]-p[:,3]));B=np.abs(Sf).sum(1);aspect=np.maximum(B.max(1)/B.min(1),B.sum(1)/(6*np.maximum(V,1e-300)**(2/3)))
 return dict(Sf=Sf,Cf=Cf,N=N,W=W,areas=areas,C=C,V=V,sigma=S[:,-1],equal_sigma=Su[:,-1],condition=S[:,0]/S[:,-1],degradation=S[:,-1]/Su[:,-1],separation=sep,projected_condition=ss[:,0]/ss[:,1],error=error,error_norm=np.linalg.norm(error,axis=1),relative_error=np.linalg.norm(error,axis=1)/np.linalg.norm(exact,axis=1),weak_error=np.einsum('ni,ni->n',error,Vt[:,-1]),U=U,exact=exact,warp=np.stack(warp,1).max(1),jacobian=jac,signed_volume=vol,orientation=ori,aspect=aspect,closure=q.sum(1),closure_scale=np.sum(areas*(np.linalg.norm(a)+np.linalg.norm(Hm,2)*np.linalg.norm(Cf,axis=2)),axis=1))

def quality(p,g,cells,roles,native,layers):
 # Every face of every affected prism is guarded, including fixed core neighbors.
 own,nei,C0,V0=native;ids,ix=np.unique(roles.ravel(),return_index=True);a=g['areas'].ravel()[ix];Sf=g['Sf'].reshape(-1,3)[ix];Cf=g['Cf'].reshape(-1,3)[ix];cell=cells.repeat(5)[ix];Sf*=np.where(own[ids]==cell,1.,-1.)[:,None]
 order=np.argsort(cells);keys=cells[order]
 def values(which,base,key):
  result=np.array(base[which]);loc=np.searchsorted(keys,which);hit=(loc<len(keys));hit[hit]&=keys[loc[hit]]==which[hit];result[hit]=g[key][order[loc[hit]]];return result
 internal=ids<len(nei);fi=ids[internal];fc=Cf[internal];sf=Sf[internal];oo=values(own[fi],C0,'C');nn=values(nei[fi],C0,'C');vo=values(own[fi],V0,'V');vn=values(nei[fi],V0,'V');da=abs(np.einsum('ni,ni->n',sf,fc-oo));db=abs(np.einsum('ni,ni->n',sf,nn-fc));weight=np.minimum(da,db)/np.maximum(da+db,1e-300);ratio=np.minimum(vo,vn)/np.maximum(vo,vn);nonortho=np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n',sf,nn-oo)/(np.linalg.norm(sf,axis=1)*np.linalg.norm(nn-oo,axis=1)),-1,1)))
 # Ordered geometry for each selected face; triangles padded with repeated last point only for skew extent.
 fvs=np.stack([p[:,ids+[ids[-1]]*(4-len(ids))] for ids in ROLES],1).reshape(-1,4,3)[ix][internal];skew=FG.internal_face_skewness(fvs,fc,sf,oo,nn)
 det=np.empty(len(p));row=np.asarray(layers);det[row==1]=PM._foundation_determinant(p[row==1],lower_cap_is_uncoupled_wall=True);det[row!=1]=PM._foundation_determinant(p[row!=1],lower_cap_is_uncoupled_wall=False)
 fails=dict(determinant=int((det<.0011).sum()),weight=int((weight<.055).sum()),skew=int((skew>4/1.1).sum()),volume_ratio=int((ratio<.01).sum()),aspect=int((g['aspect']>1000).sum()),jacobian=int((g['jacobian']<=0).sum()),volume=int(((g['V']<=0)|(g['signed_volume']<=0)).sum()),orientation=int((g['orientation']<=0).sum()),finite=int(not all(np.isfinite(z).all() for z in [p,det,weight,skew,ratio])))
 cw=np.ones(len(p));cs=np.zeros(len(p));cn=np.zeros(len(p));cr=np.ones(len(p));lookup={int(c):i for i,c in enumerate(cells)}
 for j,face in enumerate(fi):
  for c in (int(own[face]),int(nei[face])):
   if c in lookup:
    k=lookup[c];cw[k]=min(cw[k],weight[j]);cs[k]=max(cs[k],skew[j]);cn[k]=max(cn[k],nonortho[j]);cr[k]=min(cr[k],ratio[j])
 g.update(determinant=det,weight=cw,skewness=cs,nonorthogonality=cn,volume_ratio=cr)
 return dict(pass_gate=not any(fails.values()),failures=fails,minimum_determinant=float(det.min()),minimum_weight=float(weight.min()),maximum_skewness=float(skew.max()),minimum_volume_ratio=float(ratio.min()),maximum_aspect=float(g['aspect'].max()),maximum_nonorthogonality=float(nonortho.max()),minimum_jacobian=float(g['jacobian'].min()),minimum_volume=float(g['V'].min()),core_neighbor_included=True)
