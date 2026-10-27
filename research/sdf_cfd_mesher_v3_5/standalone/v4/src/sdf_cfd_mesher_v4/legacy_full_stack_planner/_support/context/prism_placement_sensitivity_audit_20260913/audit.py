from common import *
import numpy as np
from scipy.optimize import linprog, minimize
from scipy.sparse import csr_matrix
PREV=G/'prism_placement_counterfactual_20260913'
sys.path.append(str(PREV))
from geometry import cell_geometry,quality,ROLES
import placement

METRICS=['sigma','equal_sigma','separation','projected_condition','determinant','weight','skewness','volume_ratio','nonorthogonality','jacobian','V','error_norm','aspect']
def ah(x):return hashlib.sha256(np.ascontiguousarray(x).tobytes()).hexdigest()
def prs(x,f):return np.concatenate([x[:-1,f].transpose(1,0,2,3),x[1:,f].transpose(1,0,2,3)],axis=2).reshape(-1,6,3)
def ev(x,f,cells,roles,native,di):
 p=prs(x,f);g=cell_geometry(p,di);q=quality(p,g,cells,roles,native,np.tile(np.arange(1,21),len(f)));return g,q
def local_sigma_gradient(p,g):
 A=g['N']*np.sqrt(g['W'])[:,:,None];u,s,vt=np.linalg.svd(A,full_matrices=False)
 grads=np.zeros((len(p),6,3))
 for j in range(6):
  for k in range(3):
   ds=[]
   for r,ids in enumerate(ROLES):
    vs=p[:,ids];dv=np.zeros_like(vs)
    if j in ids:dv[:,ids.index(j),k]=1
    ss=.5*np.cross(vs,np.roll(vs,-1,axis=1)).sum(1)
    sign=np.sign(np.einsum('ni,ni->n',ss,g['Sf'][:,r]))
    ds.append(.5*(np.cross(dv,np.roll(vs,-1,axis=1))+np.cross(vs,np.roll(dv,-1,axis=1))).sum(1)*sign[:,None])
   ds=np.stack(ds,1);da=np.einsum('nfi,nfi->nf',g['N'],ds)
   dn=(ds-g['N']*da[:,:,None])/g['areas'][:,:,None]
   dw=(da-g['W']*da.sum(1)[:,None])/g['areas'].sum(1)[:,None]
   dA=np.sqrt(g['W'])[:,:,None]*dn+g['N']*dw[:,:,None]/(2*np.sqrt(g['W'])[:,:,None])
   grads[:,j,k]=np.einsum('nf,nfi,ni->n',u[:,:,-1],dA,vt[:,-1])
 return grads,s,(s[:,1]-s[:,2])/s[:,0]

def feasibility(J,active,protect,bounds):
 d=J.shape[1]
 if d==0:return dict(success=True,positive=False,t=0.,direction=[])
 rn=np.linalg.norm(J,axis=1);a=active[rn[active]>1e-12]
 if len(a)!=len(active):return dict(success=True,positive=False,t=0.,reason='Zero active gradient',direction=[0.]*d)
 jj=J/np.maximum(rn[:,None],1e-300)
 protect=np.array([i for i in protect if rn[i]>1e-12 and i not in set(a)],int)
 mat=np.concatenate([np.c_[-jj[a],np.ones(len(a))],np.c_[-jj[protect],np.zeros(len(protect))]])
 result=linprog(np.r_[np.zeros(d),-1.],A_ub=csr_matrix(mat),b_ub=np.zeros(len(mat)),bounds=bounds+[(0.,1.)],method='highs',options={'time_limit':3.})
 if not result.success:return dict(success=False,status=int(result.status),message=result.message,positive=False,direction=[0.]*d)
 rawx=result.x.copy();direction=result.x[:-1].copy();norm=np.linalg.norm(direction);direction/=max(norm,1e-300)
 residual=float(np.max(mat@result.x)) if len(mat) else 0
 c=np.r_[np.zeros(d),-1.];dual=result.ineqlin.marginals;low=result.lower.marginals;up=result.upper.marginals
 station=c-mat.T@dual-low-up
 dualobj=sum(b[0]*l+b[1]*u for b,l,u in zip(bounds+[(0,1)],low,up))
 return dict(success=True,t=float(result.x[-1]),positive=bool(result.x[-1]>1e-8),direction=direction.tolist(),constraint_residual=residual,active_min_rate=float(np.min(J[a]@direction)),protected_min_rate=float(np.min(J[protect]@direction)) if len(protect) else None,dual_stationarity=float(np.max(abs(station))),duality_gap=float(abs(result.fun-dualobj)),dual_active_rows=a.tolist(),dual_protected_rows=protect.tolist(),dual_inequality_marginals=dual.tolist(),dual_lower_marginals=low.tolist(),dual_upper_marginals=up.tolist(),raw_solution=rawx.tolist())

def best_direction(J,active,bounds):
 d=J.shape[1];a=J[active]
 if len(a)==1:
  z=a[0].copy()
  for j,(lo,hi) in enumerate(bounds):
   if lo==0:z[j]=max(z[j],0)
   if hi==0:z[j]=min(z[j],0)
  z/=max(np.linalg.norm(z),1e-300)
  return z,dict(method='exact unit-Euclidean projection of single active gradient onto authority orthant',success=True,rate=float((a@z)[0]))
 rn=np.linalg.norm(a,axis=1);aa=a/max(rn.max(),1e-300)
 res=minimize(lambda z:.5*np.dot(z,z),np.zeros(d),jac=lambda z:z,bounds=[(None if lo<0 else 0,None if hi>0 else 0) for lo,hi in bounds],constraints=[dict(type='ineq',fun=lambda z:aa@z-1,jac=lambda z:aa)],method='SLSQP',options={'maxiter':100,'ftol':1e-10})
 z=res.x/max(np.linalg.norm(res.x),1e-300)
 return z,dict(method='minimum-norm QP for multi-active maximin',success=bool(res.success),rate=float(np.min(a@z)),message=res.message)

def main(level):
 _,out,gg,_,_,wing=paths(level)
 native=(raw(gg/'owner.i64',dtype='<i8'),raw(gg/'neighbour.i64',dtype='<i8'),raw(out/'handoff/C.f64',3),raw(out/'handoff/V.f64'))
 state=read(wing/'planner/selected_state/state.json');normal=np.load(wing/'planner/selected_state/source_normals.npy')
 results=[]
 for folder in sorted((PREV/(level+'_validated')).glob('patch_*')):
  if not (folder/'GEOMETRY.npz').exists():continue
  tick=time.monotonic();z={k:v for k,v in np.load(folder/'GEOMETRY.npz').items()};x=z['before'];f=z['faces'];cols=z['columns'];cells=z['cell_ids'];target=z['targets']
  old=read(folder/'RESULT.json');bounds0=read(folder/'BOUNDS.json')['bounds'];roles=np.load(G/'prism_column_geometry_forensics_20260912'/level/'FACE_ROLES.npy',mmap_mode='r')[cols].reshape(-1,5)
  di=np.repeat(x[-1,f].mean(1)-x[0,f].mean(1),20,axis=0);p=prs(x,f);g,q=ev(x,f,cells,roles,native,di)
  inc={int(v):normal[state['incidents'][int(z['vertex_ids'][v])]] for v in z['movable']}
  replay,bcheck=placement.proposal(x,f,z['movable'],inc)
  assert np.array_equal(replay,z['direction']), 'Prior direction mismatch'
  localgrad,spec,gap=local_sigma_gradient(p,g);basis=[];dofs=[];box=[];envelope=[];candidate=[];radii=[];rankcounts={}
  for b in bcheck:
   v=b['vertex'];rank=b['rank'];rankcounts[rank]=rankcounts.get(rank,0)+1
   if rank==3:continue
   bb=np.asarray(b['basis']);radii.append(b['radius'])
   assert np.max(abs(inc[v]@bb))<1e-10
   for k in range(2,20):
    for j in range(bb.shape[1]):
     dofs.append((k,v,j));basis.append(bb[:,j]);lo,hi=b['lower'][j],b['upper'][j]
     tol=8192*np.finfo(float).eps*b['radius']
     box.append((0. if lo>=-tol else -1.,0. if hi<=tol else 1.))
     envelope.append((float(lo),float(hi),float(b['radius'])))
     candidate.append(float(z['direction'][k,v]@bb[:,j]))
  basis=np.array(basis);candidate=np.array(candidate);nd=len(dofs);J=np.zeros((len(p),nd));index={}
  for j,(k,v,b) in enumerate(dofs):
   for ci,tri in enumerate(f):
    loc=np.flatnonzero(tri==v)
    if len(loc):
     m=int(loc[0]);J[ci*20+k-1,j]=localgrad[ci*20+k-1,m+3]@basis[j];J[ci*20+k,j]=localgrad[ci*20+k,m]@basis[j]
  ti=np.flatnonzero(np.repeat(np.isin(cols,target),20));ni=np.setdiff1d(np.arange(len(p)),ti)
  floor=max(1e-12,8192*np.finfo(float).eps*float(g['sigma'].max()))
  active=ti[g['sigma'][ti]<=g['sigma'][ti].min()+floor]
  weakest=ni[g['sigma'][ni]<=g['sigma'][ti].min()+floor]
  best,bs=best_direction(J,active,box);common=feasibility(J,active,weakest,box);protected=feasibility(J,active,np.arange(len(p)),box)
  candnorm=float(np.linalg.norm(candidate));cu=candidate/candnorm if candnorm>1e-12 else np.zeros_like(candidate)
  angle=lambda a,b:None if np.linalg.norm(a)*np.linalg.norm(b)<1e-14 else float(np.degrees(np.arccos(np.clip(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1))))
  if nd==0:cat='MovementAuthorityRankBlocked'
  elif not bs['success'] or np.any(gap[active]<=np.sqrt(np.finfo(float).eps)):cat='NonsmoothOrHigherOrderRequired'
  elif protected['positive']:cat='CommonImprovementDirectionExists'
  elif common['positive'] or bs['rate']>1e-12:cat='TargetOnlyImprovementExists'
  else:cat='NoFirstOrderImprovement'
  minstep=float(np.linalg.norm(np.diff(x,axis=0),axis=2).min());length=min(minstep,min(radii)) if radii else minstep
  def displacement(coeff):
   delta=np.zeros_like(x)
   for j,(k,v,b) in enumerate(dofs):delta[k,v]+=basis[j]*coeff[j]
   return delta
  def symmetric_bound(coeff):
   slack=[]
   for c,(lo,hi,rad) in zip(coeff,envelope):
    if abs(c)>1e-14:slack.append(min(-lo,hi,rad)/abs(c))
   return min([length]+[.25*v for v in slack])
  directions={'old_unit':cu,'best_unit':best}
  if protected.get('positive'):directions['neighbor_protected_unit']=np.array(protected['direction'])
  # A reproducible geometry-basis direction, no cell or probe routing.
  rand=np.cos(np.arange(nd)+1.);rand=np.array([r if lo<0<hi else 0 for r,(lo,hi) in zip(rand,box)]);rand/=max(np.linalg.norm(rand),1e-300);directions['basis_control']=rand
  fd={};matrix={}
  for name,d in directions.items():
   delta=displacement(d);h0=symmetric_bound(d)
   if np.linalg.norm(d)<1e-12 or h0<=1e-14:
    fd[name]=dict(centered_admissible=False,reason='zero direction or one-sided envelope');continue
   reductions=0
   while True:
    hs=h0*np.array([1e-3,5e-4,2.5e-4]);der=[];checks=[]
    for h in hs:
     gp,qp=ev(x+h*delta,f,cells,roles,native,di);gm,qm=ev(x-h*delta,f,cells,roles,native,di)
     der.append(np.stack([(gp[m]-gm[m])/(2*h) for m in METRICS],1));checks.append(qp['pass_gate'] and qm['pass_gate'])
    if all(checks):break
    reductions+=1
    assert reductions<=8,'No centered hard-admissible validation radius'
    h0*=.25
   der=np.stack(der);exact=J@d;er=np.max(abs(der[:,: ,0]-exact),axis=1);scale=np.maximum(abs(exact),1e-8/length)
   normerr=np.max(abs(der[:,: ,0]-exact)/scale,axis=1);converge=np.max(abs(der[2,:,0]-der[1,:,0])/scale)
   allowed=np.maximum(1e-4*abs(exact),1e-8/length)
   ratio=np.max(abs(der[:,:,0]-exact)/allowed,axis=1);conv=float(np.max(abs(der[2,:,0]-der[1,:,0])/allowed))
   fd[name]=dict(centered_admissible=True,step_m=hs.tolist(),hard_checks_pass=all(checks),sigma_absolute_derivative_errors=er.tolist(),sigma_scaled_errors=normerr.tolist(),final_estimate_change=float(converge),declared_tolerance_ratios=ratio.tolist(),convergence_tolerance_ratio=conv,pass_gate=bool(ratio[-1]<=1 and conv<=1),target_active_rate=exact[active].tolist(),target_max_error_rate=float(der[-1,ti[np.argmax(g['error_norm'][ti])],METRICS.index('error_norm')]),neighbor_max_error_rate=float(der[-1,ni[np.argmax(g['error_norm'][ni])],METRICS.index('error_norm')]) if len(ni) else None)
   fd[name]['hard_margin_step_reductions']=reductions
   matrix[name]=der
  # Dense audit of the old direction only. No fitted coordinates are saved.
  dense=[];alphas=np.unique(np.r_[np.linspace(0,1,65),placement.ALPHAS])
  for alpha in alphas:
   ag,aq=ev(x+alpha*z['direction'],f,cells,roles,native,di)
   dense.append(dict(alpha=float(alpha),minimum_sigma=float(ag['sigma'][ti].min()),active_cell=int(cells[ti[np.argmin(ag['sigma'][ti])]]),minimum_neighbor_sigma=float(ag['sigma'][ni].min()),maximum_target_error=float(ag['error_norm'][ti].max()),maximum_neighbor_error=float(ag['error_norm'][ni].max()),hard_pass=aq['pass_gate'],minimum_determinant=aq['minimum_determinant'],minimum_weight=aq['minimum_weight'],maximum_skewness=aq['maximum_skewness'],minimum_volume_ratio=aq['minimum_volume_ratio']))
  oldobj=max([old['baseline_min_sigma']]+[r['objective'] for r in read(folder/'TRIALS.json')['repeated'][0]['trials'] if r['numeric_pass']])
  ci=np.arange(len(cols))*20+np.argmin(g['sigma'].reshape(-1,20),axis=1);CJ=J[ci];norm=np.linalg.norm(CJ,axis=1);cn=CJ/np.maximum(norm[:,None],1e-300);cos=cn@cn.T
  pair=np.triu(np.ones_like(cos,bool),1)&(norm[:,None]>1e-12)&(norm[None,:]>1e-12);angles=np.degrees(np.arccos(np.clip(cos[pair],-1,1)))
  margins={'determinant':(g['determinant']-.0011)/.0011,'weight':(g['weight']-.055)/.055,'skewness':(4/1.1-g['skewness'])/(4/1.1),'volume_ratio':(g['volume_ratio']-.01)/.01,'aspect':(1000-g['aspect'])/1000,'jacobian':g['jacobian']/np.maximum(abs(g['jacobian']),1e-300),'volume':g['V']/np.maximum(abs(g['V']),1e-300)}
  summary=dict(level=level,patch=folder.name,targets=target.tolist(),columns=cols.tolist(),prior_status=old['status'],raw_all_layer_DOF=int(x.size),raw_movable_DOF=3*21*len(z['movable']),fixed_layers_remove_DOF=9*len(z['movable']),equality_constrained_DOF=nd,per_layer_admissible_DOF=[0,0]+[nd//18]*18+[0],rank_counts=rankcounts,original_direction_norm_m=candnorm,candidate_basis_fraction=(1/nd if candnorm>1e-12 and nd else 0),basis_reproduction_error=float(np.max(abs(displacement(candidate)-z['direction']))),candidate_best_angle_degrees=angle(candidate,best),candidate_active_sigma_rates=(J[active]@cu).tolist(),active_cells=cells[active].tolist(),active_layers=(active%20+1).tolist(),active_count=len(active),minimum_sigma=float(g['sigma'][ti].min()),minimum_singular_gap=float(gap.min()),degenerate_cells=int((gap<=np.sqrt(np.finfo(float).eps)).sum()),best=bs,common=common,neighbor_protected=protected,classification=cat,FD=fd,dense_discrete_miss=bool(max(r['minimum_sigma'] for r in dense if r['hard_pass'])>oldobj+floor),dense_best_gain_over_discrete=float(max(r['minimum_sigma'] for r in dense if r['hard_pass'])-oldobj),dense_active_set_count=len(set(r['active_cell'] for r in dense)),angle_statistics=stats(angles),competing_pairs=int((angles>105).sum()),strong_conflict_pairs=int((angles>=165).sum()),near_active_hard={k:int((v<=1e-3).sum()) for k,v in margins.items()},minimum_normalized_hard_margin={k:float(v.min()) for k,v in margins.items()},one_sided_basis_count=sum(lo==0 or hi==0 for lo,hi in box),local_contact_scope='NonDifferentiableLocalCheck; exact preserved certificates at origin, no global collision certificate for diagnostic perturbations',seconds=time.monotonic()-tick)
  dest=D/'qualified'/level/folder.name;dest.mkdir(parents=True,exist_ok=True)
  np.savez_compressed(dest/'SENSITIVITY.npz',cell_ids=cells,columns=cols,target_indices=ti,neighbor_indices=ni,active=active,dofs=np.array(dofs),basis=basis,bounds=np.array(box),envelope=np.array(envelope),J_sigma=J,column_min_indices=ci,conflict=cn,gap=gap,singular_values=spec,candidate_coefficients=candidate,best_coefficients=best,base=np.stack([g[m] for m in METRICS],1),**{'FD_'+k:v for k,v in matrix.items()})
  write(dest/'RESULT.json',summary);write(dest/'DENSE_ALPHA.json',dense);results.append(summary)
  write(D/'qualified'/(level+'_RESULTS.json'),results);print(level,folder.name,nd,cat,candnorm,summary['candidate_best_angle_degrees'],summary['seconds'],flush=True)
if __name__=='__main__':main(sys.argv[1])
