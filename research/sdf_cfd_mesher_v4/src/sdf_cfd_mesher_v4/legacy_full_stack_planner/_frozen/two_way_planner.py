"""Baseline-ordered lower-tail planning with explicit protected floors.

Physical generation, compression and continuous admission are injected frozen
authorities. This module adds no column-ID-specific behavior.
"""
from component_planner import *
from diagnostic_convergence import IterationObserver
from linear_solve import solve_lp
from trust_region import stage_assessment,next_radius

class TailObserver(IterationObserver):
 def record(self,*args):
  stop=super().record(*args);event=args[1]
  change=float(event.get('accepted_lexicographic_change_max',0.));row=self.records[-1]
  row['accepted_lexicographic_change_max']=change
  if change>row['convergence_thresholds']['sigma']:
   self.stable=0;row['stable_iterations']=0
   if stop=='DeterministicConvergenceCriteriaSatisfied':stop=None
  write(self.root/'ITERATIONS.json',self.records)
  return stop

def tail_definition(baseline,columns,targets):
 b=np.asarray(baseline).reshape(-1,20);mask=np.isin(columns,targets)
 order=sorted(np.flatnonzero(mask),key=lambda i:(float(b[i].min()),int(columns[i])))
 ceiling=float(b[mask].min(1).max())
 neighbor=np.minimum(b.min(1),ceiling)
 return mask,order,ceiling,neighbor

def tail_guards(baseline,before,after,columns,targets):
 mask,order,ceiling,neighbor=tail_definition(baseline,columns,targets)
 a=np.asarray(after).reshape(-1,20);old=np.asarray(before).reshape(-1,20)
 original=np.asarray(baseline).reshape(-1,20).min(1)
 changes=a[mask].min(1)-original[mask];margin=a[~mask].min(1)-neighbor[~mask]
 return dict(pass_gate=bool(np.all(changes>=-TAU)and np.all(margin>=-TAU)),target_regressions=int((changes<-TAU).sum()),target_changes=changes.tolist(),neighbor_weak_tail_violations=int((margin<-TAU).sum()),neighbor_floor=ceiling,minimum_neighbor_margin=float(margin.min()),target_order=[int(columns[i])for i in order])

def lex_vector(sigma,columns,targets,baseline):
 mask,order,_,_=tail_definition(baseline,columns,targets);a=np.asarray(sigma).reshape(-1,20).min(1)
 return np.r_[a[mask].min(),a[order]]

def positive_lex_change(old,new):
 for delta in new-old:
  if delta>TAU:return True
  if delta< -TAU:return False
 return False

def incumbent_objective(sigma,z,space,columns,targets,baseline):
 guard=tail_guards(baseline,baseline,sigma,columns,targets)
 worst_loss=max(0.,float(np.max(np.asarray(baseline)-np.asarray(sigma))))
 displacement=float(np.sum((np.asarray(z)/space['radii'])**2))
 return np.r_[int(guard['pass_gate']),lex_vector(sigma,columns,targets,baseline),int(guard['neighbor_weak_tail_violations']==0),-worst_loss,-displacement]

def lower_tail_step(z,space,columns,targets,baseline,current,J,rho,extra_inequalities=None):
 n=len(z);mask,order,ceiling,neighbor=tail_definition(baseline,columns,targets);cm=baseline.reshape(-1,20).min(1)
 floor=np.repeat(np.where(mask,cm,neighbor),20);bounds=[]
 for j in range(n):
  rank=next(len(g)for g in space['groups']if j in g);rr=rho*space['radii'][j]/np.sqrt(rank)
  bounds.append((max(-rr,space['bounds'][j,0]-z[j]),min(rr,space['bounds'][j,1]-z[j])))
 bounds.extend([(0,None),(0,None)]);opts=dict(primal_feasibility_tolerance=1e-9,dual_feasibility_tolerance=1e-9,time_limit=3.)
 stages=[];last=None;prefixes=[]
 def problem(focus=None,loss_limit=None):
  rows=[];rhs=[]
  for i in range(len(current)):
   rows.append(np.r_[-J[i],0.,0.]);rhs.append(current[i]-floor[i])
   rows.append(np.r_[-J[i],0.,-1.]);rhs.append(current[i]-baseline[i])
  if extra_inequalities is not None:
   extra_A,extra_b=extra_inequalities
   for row,value in zip(extra_A,extra_b):rows.append(np.r_[row,0.,0.]);rhs.append(value)
  reference=None
  if focus is not None:
   reference=float(current[focus].min())
   for i in focus:
    rows.append(np.r_[-J[i],1.,0.]);rhs.append(current[i]-reference)
  bb=list(bounds)
  bb[n]=(0.,0.)if focus is None else(None,None)
  if loss_limit is not None:bb[-1]=(0.,loss_limit)
  return np.asarray(rows),np.asarray(rhs),bb,reference
 objectives=[('target_set',np.flatnonzero(np.repeat(mask,20)))]
 objectives.extend(('target_column_'+str(int(columns[i])),np.arange(i*20,(i+1)*20))for i in order)
 for label,focus in objectives:
  A,b,bb,reference=problem(focus);r=solve_lp(np.r_[np.zeros(n),-1.,0.],A_ub=csr_matrix(A*1e5),b_ub=b*1e5,bounds=bb,method='highs-ds',options=opts)
  rec=dict(level=label,success=bool(r.success),message=r.message,recovery=r.execution_recovery);stages.append(rec)
  if not r.success:return None,dict(reason='LowerTailLinearizedSearchFailure',stages=stages)
  achieved=reference+float(r.x[n]);rec.update(gain=float(r.x[n]),floor=achieved)
  floor[focus]=np.maximum(floor[focus],achieved-TAU);last=r.x.copy();prefixes.append((label,last.copy()))
 complete=last.copy();A,b,bb,_=problem();secondary=solve_lp(np.r_[np.zeros(n+1),1.],A_ub=csr_matrix(A*1e5),b_ub=b*1e5,bounds=bb,method='highs-ds',options=opts)
 if secondary.success:last=secondary.x.copy()
 stages.append(dict(level='neighbor_loss',success=bool(secondary.success),message=secondary.message,recovery=secondary.execution_recovery))
 last[n]=0.;loss=float(last[-1]);A,b,bb,_=problem(loss_limit=loss+TAU)
 R=np.diag(1/np.maximum(space['radii'],1e-15)**2);R/=max(float(np.linalg.norm(R,2)),1.)
 fun=lambda y:float((z+y[:n])@R@(z+y[:n]));jac=lambda y:np.r_[2*R@(z+y[:n]),0.,0.]
 tertiary=minimize(fun,last,jac=jac,method='SLSQP',bounds=bb,constraints=[dict(type='ineq',fun=lambda y:b-A@y,jac=lambda y:-A)],options=dict(maxiter=80,ftol=1e-12))
 preferred=tertiary.x if tertiary.success and np.min(b-A@tertiary.x)>=-1e-9 else last
 phases=[];seen=set()
 for label,y in [('displacement',preferred),('neighbor',last),('complete_target_lexicographic',complete)]+[('retained_prefix_'+label,y)for label,y in reversed(prefixes[:-1])]:
  key=np.ascontiguousarray(y[:n]).tobytes()
  if key not in seen:phases.append(dict(phase=label,step=y[:n].tolist()));seen.add(key)
 return preferred[:n],dict(primary_gain=stages[0]['gain'],stages=stages,modeled_target_floors=floor.reshape(-1,20).min(1)[mask].tolist(),target_order=[int(columns[i])for i in order],migration_floor=ceiling,worst_remaining_loss=loss,displacement_phase_success=bool(tertiary.success),phase_candidates=phases)

def plan_lower_tail(state,nominal,accepted_schedule,owner,baseline_sigma,evaluate,admit,*,root,max_iterations=100,success=None,additional_admission=None,admission_linearization=None,iteration_checkpoint=None,restoration_search=None):
 c=context(state.faces,owner['owned_vertices']);space=direction_space(state,nominal,owner);z=np.zeros(len(space['dofs']));rho=.5;history=[]
 if not len(z):return dict(reason='DirectionInfeasible',qualified=False,history=[],direction=z)
 h,x,g,r=evaluate(state,nominal,owner)
 if not r['pass_gate']:return dict(reason=r['reason'],qualified=False,history=[],direction=z)
 initial=g['sigma'].copy();quality=r['quality'];observer=TailObserver(state,owner,baseline_sigma,space,root,success)
 observer.initial_positive_scales={key:g[key].copy()for key in ('jacobian','V','signed_volume','orientation')}
 mask,order,ceiling,neighbor=tail_definition(baseline_sigma,c['columns'],owner['target_columns'])
 target_floor=baseline_sigma.reshape(-1,20).min(1)
 reason='LowerTailIterationCeiling'
 for iteration in range(max_iterations):
  old_vector=lex_vector(g['sigma'],c['columns'],owner['target_columns'],baseline_sigma);before=g['sigma'].copy();old_objective=incumbent_objective(g['sigma'],z,space,c['columns'],owner['target_columns'],baseline_sigma)
  J,gap=direction_jacobian(state,z,space,c,h,x,g);extra=None;admission_model=None
  restoring=restoration_search is not None and tail_guards(baseline_sigma,baseline_sigma,g['sigma'],c['columns'],owner['target_columns'])['target_regressions']>0
  proposal_rho=min(1.,2*rho)if restoring else rho
  if restoring:step,lr=restoration_search.propose(state,z,space,c,nominal,owner,baseline_sigma,g,h,evaluate,proposal_rho)
  else:
   if admission_linearization is not None:extra,admission_model=admission_linearization(state,z,space,c,h,x,g)
   step,lr=lower_tail_step(z,space,c['columns'],owner['target_columns'],baseline_sigma,g['sigma'],J,rho,extra)
  event=dict(iteration=iteration,linear=lr,singular_gap=gap,rho=proposal_rho,additional_admission_model=admission_model,original_floor_restoration=restoring)
  if step is None:
   history.append(event);observer.record(iteration,event,z,float(old_vector[0]),h,x,g,quality);reason=lr.get('reason','RestorationModelIndeterminate')if restoring else'LowerTailLinearizedSearchFailure';break
  trials=[];ok=False
  for proposal in lr['phase_candidates']:
   candidate=z+np.asarray(proposal['step']);st=planned_state(state,candidate,space);hh,xx,gg,rr=evaluate(st,nominal,owner)
   guard=tail_guards(baseline_sigma,before,gg['sigma'],c['columns'],owner['target_columns'])
   vector=lex_vector(gg['sigma'],c['columns'],owner['target_columns'],baseline_sigma);new_objective=incumbent_objective(gg['sigma'],candidate,space,c['columns'],owner['target_columns'],baseline_sigma);positive=positive_lex_change(old_objective,new_objective)
   extra_guard=additional_admission(gg,rr['quality'])if additional_admission is not None else dict(pass_gate=True)
   ok=bool(rr['pass_gate']and guard['pass_gate']and positive and extra_guard['pass_gate'])
   rejection=None
   if not rr['pass_gate']:rejection=rr.get('reason','PhysicalOrQuality')
   elif not guard['pass_gate']:rejection='ProtectedScientificFloor'
   elif not extra_guard['pass_gate']:rejection='FunctionalOrHardQualityAdmission'
   elif not positive:rejection='LexicographicNonImprovement'
   if ok:
    ok=admit(st,hh,owner)
    if not ok:rejection='ContinuousOrPhysicalGuard'
   assess=stage_assessment(g['sigma'],g['sigma']+J@np.asarray(proposal['step']),gg['sigma'],baseline_sigma,z,candidate,space['radii'],c['columns'],owner['target_columns'],space['groups'],np.asarray(proposal['step']),proposal_rho,TAU)
   if restoring:assess.update(ratio=None,meaningful_prediction=False,assessment_note='Restoration model differs from fixed-schedule Jacobian; no expansion from unavailable stage prediction')
   trials.append(dict(phase=proposal['phase'],compression=rr,guards=guard,additional_admission=extra_guard,finite_gain=float(vector[0]-old_vector[0]),lexicographic_change=(vector-old_vector).tolist(),incumbent_objective_before=old_objective.tolist(),candidate_objective=new_objective.tolist(),lexicographic_improvement=positive,accepted=bool(ok),first_rejection_reason=rejection,trust_assessment=assess))
   if ok:break
  event.update(compression=rr,guards=guard,finite_gain=float(vector[0]-old_vector[0]),accepted=bool(ok),phase_trials=trials,selected_phase=trials[-1]['phase'],accepted_lexicographic_change_max=float(np.max(abs(vector-old_vector)))if ok else 0.);history.append(event)
  print('Lower-tail iteration',iteration,'accepted',ok,'lex_change',(vector-old_vector).tolist(),'rho',rho,flush=True)
  updated_radius,action,trust_reason=next_radius(rho,ok,assess)
  event['trust_update']=dict(before=rho,after=updated_radius,action=action,reason=trust_reason,selected_assessment=assess)
  rho=updated_radius
  if ok:z,h,x,g,quality=candidate,hh,xx,gg,rr['quality']
  # Only scientific baseline floors are committed while the component searches.
  # Within-LP modeled floors remain temporary; accepted gains are incumbent data.
  target_floor=baseline_sigma.reshape(-1,20).min(1)
  observer.floor=np.where(mask[:,None],target_floor[:,None],neighbor[:,None])*np.ones((len(c['columns']),20))
  stop=observer.record(iteration,event,z,float(old_vector[0]),h,x,g,quality)
  if stop:reason='QualificationTargetSetReached'if stop=='DiagnosticMaterialTargetReached'else stop;break
  if iteration_checkpoint is not None:
   checkpoint=iteration_checkpoint(iteration,event,observer.records)
   if checkpoint:reason=checkpoint;break
 committed=reason=='QualificationTargetSetReached'
 write(Path(root)/'FLOOR_COMMITMENT.json',dict(rule='Commit achieved target floors only when the complete component qualification stage closes; no transient iteration gain is a hard floor',protected_baseline_floors=target_floor[mask].tolist(),migration_floor=ceiling,search_committed_new_floors=False,final_stage_committed=committed,final_stage_floors=g['sigma'].reshape(-1,20).min(1)[mask].tolist()if committed else None))
 write(Path(root)/'STOPPING_RESULT.json',dict(reason=reason,iterations=len(history),max_iterations=max_iterations,objective_family_changed=False,protected_floor_semantics='FrozenScientificAuthority',incumbent_policy_changed=True,migration_guard_changed=False,physical_contract_changed=False))
 return dict(direction=z,schedule=h,positions=x,geometry=g,history=history,space=space,qualified=False,reason=reason,initial_sigma=initial)
