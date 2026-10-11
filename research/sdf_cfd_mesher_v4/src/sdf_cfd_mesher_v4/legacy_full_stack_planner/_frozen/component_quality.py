"""Geometry-only primitive constraints and complete local numeric admission."""
from accepted_inputs import *
def face_metrics(vs,S,C,Co,Cn,Vo,Vn):
 da=abs(np.einsum('ni,ni->n',S,C-Co));db=abs(np.einsum('ni,ni->n',S,Cn-C))
 w=np.minimum(da,db)/np.maximum(da+db,1e-300)
 sk=FG.internal_face_skewness(vs,C,S,Co,Cn)
 vr=np.minimum(Vo,Vn)/np.maximum(Vo,Vn)
 d=Cn-Co;cos=abs(np.einsum('ni,ni->n',S,d))/(np.linalg.norm(S,axis=1)*np.linalg.norm(d,axis=1))
 return w,sk,vr,np.degrees(np.arccos(np.clip(cos,-1,1)))
def determinant(p,k):
 d=PM._foundation_determinant(p,lower_cap_is_uncoupled_wall=k==1)
 if k==20:
  sf=PM._face_vectors(p);mask=np.array([1.,0.,1.,1.,1.]);a=np.sum(np.linalg.norm(sf,axis=2)*mask,axis=1)/4
  s=sf/a[:,None,None];d=np.minimum(d,PM._det_symm(np.einsum('nfi,nfj,f->nij',s,s,mask)))
 return d
def global_failure_rows(state,schedule):
 """Read all nominal geometry; return only exact failing constraint supports."""
 x=state.positions(schedule);f=state.faces;e,ef=FG.unique_manifold_edges(f);rows=[];previous=None
 for k in range(1,21):
  lo,hi=x[k-1:k+1];p=np.concatenate((lo[f],hi[f]),axis=1);C,V=FG.prism_cell_centres(p)
  dt=determinant(p,k);j,vol=PM._exact_wedge_jacobian_and_volume(p)
  ori=np.einsum('ni,ni->n',np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),np.cross(p[:,4]-p[:,3],p[:,5]-p[:,3]))
  bad=np.flatnonzero((dt<.0011)|(j<=0)|(vol<=0)|(V<=0)|(ori<=0))
  rows.extend(dict(kind='cell',layer=k,columns=[int(i)],vertices=f[i].tolist()) for i in bad)
  vs=np.stack((lo[e[:,0]],lo[e[:,1]],hi[e[:,1]],hi[e[:,0]]),axis=1);S,Cf=FG.face_area_centre_quad(vs)
  w,sk,vr,no=face_metrics(vs,S,Cf,C[ef[:,0]],C[ef[:,1]],V[ef[:,0]],V[ef[:,1]])
  bad=np.flatnonzero((w<.055)|(sk>4/1.1)|(vr<.01))
  rows.extend(dict(kind='side',layer=k,columns=ef[i].tolist(),vertices=np.unique(f[ef[i]]).tolist()) for i in bad)
  if previous is not None:
   S,Cf=FG.face_area_centre_tri(lo[f]);w,sk,vr,no=face_metrics(lo[f],S,Cf,previous[0],C,previous[1],V)
   bad=np.flatnonzero((w<.055)|(sk>4/1.1)|(vr<.01))
   rows.extend(dict(kind='horizontal',layer=k,columns=[int(i)],vertices=f[i].tolist()) for i in bad)
  previous=C,V
 return rows
def context(faces,owned):
 e,ef=FG.unique_manifold_edges(faces);affected=np.flatnonzero(np.any(np.isin(faces,owned),axis=1))
 side=np.flatnonzero(np.any(np.isin(ef,affected),axis=1));cols=np.union1d(affected,ef[side].ravel());vv=np.unique(faces[cols])
 return dict(owned=np.asarray(owned),columns=cols,vertices=vv,faces=np.searchsorted(vv,faces[cols]),edges=np.searchsorted(vv,e[side]),owners=np.searchsorted(cols,ef[side]),affected=affected)
def local_positions(state,schedule,vertices):
 x=np.empty((21,len(vertices),3));x[0]=state.wall[vertices];x[1]=state.first[vertices]
 for k in range(1,20):x[k+1]=x[k]+schedule[:,k,None]*state.vectors[k-1,vertices]
 return x
def evaluate_positions(x,c):
 f=c['faces']
 p=np.concatenate((x[:-1,f],x[1:,f]),axis=2).transpose(1,0,2,3).reshape(-1,6,3)
 g=geom.cell_geometry(p);nc=len(f);C=g['C'].reshape(nc,20,3);V=g['V'].reshape(nc,20)
 det=np.empty((nc,20));weights=[];skews=[];ratios=[];nonos=[]
 for k in range(1,21):
  det[:,k-1]=determinant(p.reshape(nc,20,6,3)[:,k-1],k)
  lo,hi=x[k-1:k+1];ed=c['edges'];own=c['owners'];vs=np.stack((lo[ed[:,0]],lo[ed[:,1]],hi[ed[:,1]],hi[ed[:,0]]),axis=1);S,Cf=FG.face_area_centre_quad(vs)
  mm=face_metrics(vs,S,Cf,C[own[:,0],k-1],C[own[:,1],k-1],V[own[:,0],k-1],V[own[:,1],k-1])
  for dest,val in zip((weights,skews,ratios,nonos),mm):dest.append(val)
  if k>1:
   vs=lo[f];S,Cf=FG.face_area_centre_tri(vs);mm=face_metrics(vs,S,Cf,C[:,k-2],C[:,k-1],V[:,k-2],V[:,k-1])
   for dest,val in zip((weights,skews,ratios,nonos),mm):dest.append(val)
 wt,sk,vr,no=[np.concatenate(a) for a in (weights,skews,ratios,nonos)]
 failures=dict(determinant=int((det<.0011).sum()),weight=int((wt<.055).sum()),skew=int((sk>4/1.1).sum()),symmetric_volume=int((vr<.01).sum()),aspect=int((g['aspect']>1000).sum()),jacobian=int((g['jacobian']<=0).sum()),volume=int(((g['V']<=0)|(g['signed_volume']<=0)).sum()),orientation=int((g['orientation']<=0).sum()),finite=int(not all(np.isfinite(v).all() for v in (x,det,wt,sk,vr,g['sigma']))))
 q=dict(pass_gate=not any(failures.values()),failures=failures,determinant_min=float(det.min()),weight_min=float(wt.min()),skew_max=float(sk.max()),symmetric_volume_min=float(vr.min()),aspect_max=float(g['aspect'].max()),nonorthogonality_max=float(no.max()),jacobian_min=float(g['jacobian'].min()),volume_min=float(g['V'].min()))
 g.update(determinant=det.ravel(),face_weight=wt,face_skew=sk,face_volume_ratio=vr,face_nonorthogonality=no)
 return g,q
