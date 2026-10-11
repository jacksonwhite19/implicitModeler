"""Generic component-owned look-ahead planning with atomic finite admission.

No production writer is provided. Quality constraints always precede ranking.
Compression is evaluated inside every finite direction trial.
"""
from component_compression import *
from scipy.optimize import linprog,minimize
from scipy.sparse import csr_matrix
import ast
TAU=1e-12
MAX_PLANNER_ITERATIONS=12
def declared_guards(before,after,columns,targets):
 b=np.asarray(before).reshape(-1,20);a=np.asarray(after).reshape(-1,20);mask=np.isin(columns,targets)
 target_change=a[mask].min(1)-b[mask].min(1)
 floor=np.minimum(b[~mask],b.min());neighbor_margin=a[~mask]-floor
 return dict(pass_gate=bool(np.all(target_change>=-TAU) and np.all(neighbor_margin>=-TAU)),target_changes=target_change.tolist(),target_regressions=int((target_change<-TAU).sum()),neighbor_weak_tail_violations=int((neighbor_margin<-TAU).sum()))
def atomic_admit(original,candidate,hard_pass,guard_pass):
 return candidate.copy() if hard_pass and guard_pass else original.copy()
def direction_space(state,nominal,owner):
 faces=state.faces;basis=[];dofs=[];bounds=[];radii=[];groups=[];total=float(nominal[0].sum())
 for v in owner['direction_vertices']:
  n=state.normals[list(state.incidents[v])];_,sv,vt=np.linalg.svd(n,full_matrices=True);rank=int((sv>1e-11*sv[0]).sum());bb=vt[rank:].T
  if not bb.shape[1]:continue
  inc=np.flatnonzero(np.any(faces==v,axis=1));ring=np.unique(faces[inc]);ee=state.wall[ring]-state.wall[v];length=np.linalg.norm(ee,axis=1);radius=min(.25,.5*float(length[length>0].min())/total);projected=ee@bb;ids=[]
  for k in range(bb.shape[1]):
   ids.append(len(dofs));dofs.append(v);basis.append(bb[:,k]);radii.append(radius);bounds.append((max(-radius,.5*float(projected[:,k].min())/total),min(radius,.5*float(projected[:,k].max())/total)))
  groups.append(ids)
 return dict(basis=np.array(basis),dofs=np.array(dofs,int),bounds=np.array(bounds),radii=np.array(radii),groups=groups)
def planned_state(state,z,space):
 delta=np.zeros_like(state.wall)
 for v,b,q in zip(space['dofs'],space['basis'],z):delta[v]+=b*q
 first=state.first.copy();changed=np.flatnonzero(np.any(delta!=0,axis=1));first[changed]+=state.first_height*delta[changed];vectors=state.vectors.copy();d=state.vectors[0,changed]+delta[changed];d/=np.linalg.norm(d,axis=1)[:,None];vectors[:,changed]=d[None]
 return State(state.wall,first,state.faces,state.vertex_order,state.normals,state.incidents,vectors,state.first_height,'path_length_m',ownership=state.ownership)
def sigma_gradient(p,g):
 path=G/'prism_placement_sensitivity_audit_20260913/audit.py';node=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='local_sigma_gradient');ns=dict(np=np,ROLES=geom.ROLES);exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),ns);return ns['local_sigma_gradient'](p,g)
def direction_jacobian(state,z,space,c,h,x,g):
 f=c['faces'];p=np.concatenate((x[:-1,f],x[1:,f]),axis=2).transpose(1,0,2,3).reshape(-1,6,3);loc,s,gap=sigma_gradient(p,g)
 if gap.min()<=np.sqrt(np.finfo(float).eps):raise RuntimeError('RepeatedSingularValueRequiresUnqualifiedSubspacePolicy')
 delta=np.zeros_like(state.wall)
 for v,b,q in zip(space['dofs'],space['basis'],z):delta[v]+=q*b
 J=np.zeros((len(p),len(z)));where={int(v):i for i,v in enumerate(c['vertices'])}
 for j,(v,b)in enumerate(zip(space['dofs'],space['basis'])):
  vi=where[int(v)];raw=state.vectors[0,v]+delta[v];dn=raw/np.linalg.norm(raw);dd=(b-dn*(dn@b))/np.linalg.norm(raw);dx=np.zeros((21,3));dx[1]=state.first_height*b
  for k in range(1,20):dx[k+1]=dx[k]+h[vi,k]*dd
  for ci,row in enumerate(f):
   hit=np.flatnonzero(row==vi)
   if len(hit):
    a=int(hit[0]);idx=np.arange(ci*20,(ci+1)*20);J[idx,j]=np.einsum('ni,ni->n',loc[idx,a],dx[:-1])+np.einsum('ni,ni->n',loc[idx,a+3],dx[1:])
 return J,float(gap.min())
def lexicographic_step(z,space,columns,targets,baseline,current,J,rho):
 n=len(z);is_target=np.repeat(np.isin(columns,targets),20);ti=np.flatnonzero(is_target);cur=float(current[ti].min());bcol=baseline.reshape(-1,20).min(1);base_min=float(baseline.min());rows=[];rhs=[]
 for i in range(len(current)):
  if is_target[i]:
   rows.append(np.r_[-J[i],1.,0.]);rhs.append(current[i]-cur)
   # Every layer must exceed its column's frozen minimum, not just current active cell.
   # The arithmetic allowance belongs to finite admission, not the modeled loss budget.
   rows.append(np.r_[-J[i],0.,0.]);rhs.append(current[i]-bcol[i//20])
  else:
   rows.append(np.r_[-J[i],0.,0.]);rhs.append(current[i]-min(baseline[i],base_min))
  rows.append(np.r_[-J[i],0.,-1.]);rhs.append(current[i]-baseline[i])
 bounds=[]
 for j in range(n):
  rank=next(len(g) for g in space['groups']if j in g);rr=rho*space['radii'][j]/np.sqrt(rank);bounds.append((max(-rr,space['bounds'][j,0]-z[j]),min(rr,space['bounds'][j,1]-z[j])))
 bounds.extend([(0,None),(0,None)]);A=np.array(rows);b=np.array(rhs);opts=dict(primal_feasibility_tolerance=1e-9,dual_feasibility_tolerance=1e-9,time_limit=3.)
 primary=linprog(np.r_[np.zeros(n),-1.,0.],A_ub=csr_matrix(A*1e5),b_ub=b*1e5,bounds=bounds,method='highs-ds',options=opts)
 if not primary.success:return None,dict(reason='PrimaryLinearizedSearchFailure',message=primary.message)
 gain=float(primary.x[n]);bounds[n]=(max(0.,gain-TAU),None)
 secondary=linprog(np.r_[np.zeros(n+1),1.],A_ub=csr_matrix(A*1e5),b_ub=b*1e5,bounds=bounds,method='highs-ds',options=opts)
 if not secondary.success:return primary.x[:n],dict(primary_gain=gain,reason='SecondaryLinearizedSearchFailurePrimaryRetained',message=secondary.message,phase_candidates=[dict(phase='primary',step=primary.x[:n].tolist())])
 bounds[-1]=(0.,float(secondary.x[-1])+TAU);R=np.diag(1/np.maximum(space['radii'],1e-15)**2);R/=max(float(np.linalg.norm(R,2)),1.)
 fun=lambda y:float((z+y[:n])@R@(z+y[:n]));jac=lambda y:np.r_[2*R@(z+y[:n]),0.,0.]
 tertiary=minimize(fun,secondary.x,jac=jac,method='SLSQP',bounds=bounds,constraints=[dict(type='ineq',fun=lambda y:b-A@y,jac=lambda y:-A)],options=dict(maxiter=80,ftol=1e-12))
 y=tertiary.x if tertiary.success and np.min(b-A@tertiary.x)>=-1e-9 else secondary.x
 phases=[];seen=set()
 for label,point in [('displacement',y),('neighbor',secondary.x),('primary',primary.x)]:
  key=np.ascontiguousarray(point[:n]).tobytes()
  if key not in seen:phases.append(dict(phase=label,step=point[:n].tolist()));seen.add(key)
 return y[:n],dict(primary_gain=gain,worst_remaining_loss=float(y[-1]),displacement_phase_success=bool(tertiary.success),phase_candidates=phases)
def plan_component(state,nominal,accepted_schedule,owner,baseline_sigma,compression_evaluator,continuous_admission):
 """Injected admission interfaces forbid hidden global compression fallback."""
 c=context(state.faces,owner['owned_vertices']);space=direction_space(state,nominal,owner);z=np.zeros(len(space['dofs']));history=[];rho=.5
 if not len(z):return dict(reason='DirectionInfeasible',qualified=False,history=[],direction=z)
 h,x,g,r=compression_evaluator(state,nominal,owner)
 if not r['pass_gate']:return dict(reason=r['reason'],qualified=False,history=[],direction=z)
 initial=g['sigma'].copy()
 for iteration in range(MAX_PLANNER_ITERATIONS):
  J,gap=direction_jacobian(state,z,space,c,h,x,g);step,lr=lexicographic_step(z,space,c['columns'],owner['target_columns'],baseline_sigma,g['sigma'],J,rho)
  event=dict(iteration=iteration,linear=lr,singular_gap=gap,rho=rho)
  if step is None:history.append(event);break
  trials=[];ok=False
  for proposal in lr.get('phase_candidates',[dict(phase='preferred',step=step.tolist())]):
   trial_z=z+np.asarray(proposal['step']);trial_state=planned_state(state,trial_z,space);hh,xx,gg,rr=compression_evaluator(trial_state,nominal,owner);guard=declared_guards(baseline_sigma,gg['sigma'],c['columns'],owner['target_columns']);ti=np.flatnonzero(np.repeat(np.isin(c['columns'],owner['target_columns']),20));gain=float(gg['sigma'][ti].min()-g['sigma'][ti].min())
   ok=rr['pass_gate']and guard['pass_gate']and gain>TAU
   if ok:ok=continuous_admission(trial_state,hh,owner)
   trials.append(dict(phase=proposal['phase'],compression=rr,guards=guard,finite_gain=gain,accepted=bool(ok)))
   if ok:break
  event.update(compression=rr,guards=guard,finite_gain=gain,accepted=bool(ok),phase_trials=trials,selected_phase=trials[-1]['phase']);history.append(event)
  print('Planner iteration',iteration,'accepted',bool(ok),'finite_gain',gain,'rho',rho,flush=True)
  if ok:z,h,x,g=trial_z,hh,xx,gg
  else:rho*=.5
 return dict(direction=z,schedule=h,positions=x,geometry=g,history=history,space=space,qualified=False,reason='FiniteSearchCompleteRequiresStageTargets',initial_sigma=initial)
