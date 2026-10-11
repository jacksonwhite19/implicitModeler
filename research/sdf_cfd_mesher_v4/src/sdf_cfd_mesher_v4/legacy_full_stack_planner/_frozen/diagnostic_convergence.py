"""Unchanged finite search with explicit, deterministic stopping diagnostics.

The linear objectives, trust-region formula, candidates and finite admission
are imported from the sealed planner. This module changes only the horizon
and records the state after each complete iteration.
"""
from component_planner import *

def clean(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {str(k):clean(x)for k,x in v.items()}
 if isinstance(v,list):return [clean(x)for x in v]
 return v

class IterationObserver:
 def __init__(self,state,owner,baseline,space,root,success=None):
  self.state=state;self.owner=owner;self.c=context(state.faces,owner['owned_vertices']);self.columns=self.c['columns'];self.baseline=np.asarray(baseline).reshape(-1,20);self.space=space;self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True);self.success=success;self.started=time.perf_counter();self.records=[];self.previous_active=None;self.stable=0;self.certificate_count=0
  self.mask=np.isin(self.columns,owner['target_columns']);self.floor=np.where(self.mask[:,None],self.baseline.min(1)[:,None],np.minimum(self.baseline,self.baseline.min()));self.initial_positive_scales=None
 def record(self,iteration,event,z,previous_min,h,x,g,current_quality):
  sig=g['sigma'].reshape(-1,20);err=g['error_norm'].reshape(-1,20);minimum=float(sig[self.mask].min());eps=np.finfo(float).eps;atol=max(TAU,64*eps*max(1.,abs(minimum)));stol=float(np.sqrt(eps))
  rows=[]
  for i in np.flatnonzero(self.mask):
   k=int(sig[i].argmin());rows.append(dict(column=int(self.columns[i]),baseline_sigma=float(self.baseline[i].min()),sigma=float(sig[i,k]),gain=float(sig[i,k]-self.baseline[i].min()),active_layers=(np.flatnonzero(sig[i]<=sig[i,k]+atol)+1).tolist(),equal_sigma=float(g['equal_sigma'].reshape(-1,20)[i,k]),condition=float(g['condition'].reshape(-1,20)[i,k]),side_separation=float(g['separation'].reshape(-1,20)[i,k]),smooth_error_max=float(err[i].max())))
  active_target=[]
  for i,k in np.argwhere((sig<=minimum+atol)&self.mask[:,None]):active_target.append([int(self.columns[i]),int(k+1)])
  margin=sig-self.floor
  active_floor=[[int(self.columns[i]),int(k+1),'target'if self.mask[i]else'neighbor']for i,k in np.argwhere(margin/self.floor<=stol)]
  hard_arrays={'determinant':g['determinant']/.0011-1,'weight':g['face_weight']/.055-1,'skew':1-g['face_skew']/(4/1.1),'symmetric_volume':g['face_volume_ratio']/.01-1,'aspect':1-g['aspect']/1000}
  if self.initial_positive_scales is None:raise RuntimeError('MissingFrozenPositiveScales')
  hard_arrays.update({name:g[key]/self.initial_positive_scales[key] for name,key in [('jacobian','jacobian'),('volume','V'),('signed_volume','signed_volume'),('orientation','orientation')]})
  hard_active={name:np.flatnonzero(values<=stol).tolist()for name,values in hard_arrays.items()}
  hard_margins={name:float(values.min())for name,values in hard_arrays.items()};hard_margins.update(jacobian_min=float(g['jacobian'].min()),volume_min=float(g['V'].min()))
  bounds=self.space['bounds'];rad=self.space['radii'];lower=(z-bounds[:,0])/rad;upper=(bounds[:,1]-z)/rad
  active=dict(target_prisms=active_target,target_column_layers=[[r['column'],r['active_layers']]for r in rows],floors=active_floor,hard_rows=hard_active,direction_lower=np.flatnonzero(lower<=stol).tolist(),direction_upper=np.flatnonzero(upper<=stol).tolist())
  attempted={p['phase']for p in event.get('phase_trials',[])}
  phases=[p for p in event.get('linear',{}).get('phase_candidates',[])if p['phase']in attempted];norms=[float(np.max(np.abs(np.asarray(p['step']))/rad))for p in phases]
  maxnorm=max(norms)if norms else None;selected=event.get('selected_phase');chosen=next((np.asarray(p['step'])for p in phases if p['phase']==selected),None)
  gain=minimum-previous_min;small=abs(gain)<=atol and maxnorm is not None and maxnorm<=stol and active==self.previous_active
  self.stable=self.stable+1 if small else 0;self.previous_active=active
  certpath=self.root/'CERTIFICATION.json';certs=read(certpath)if certpath.exists()else[];newcert=certs[self.certificate_count:];self.certificate_count=len(certs)
  record=dict(iteration=iteration,accepted=bool(event.get('accepted',False)),target_minimum=minimum,target_set_improvement=gain,targets=rows,component_columns=[dict(column=int(c),sigma=float(a.min()),smooth_error_max=float(e.max()))for c,a,e in zip(self.columns,sig,err)],step_norm=float(np.linalg.norm(chosen))if chosen is not None else None,maximum_proposed_normalized_step=maxnorm,trust_region=event['rho'],linear_objective=event.get('linear',{}).get('primary_gain'),finite_candidate_gain=event.get('finite_gain'),active_constraints=active,guard_margins=dict(target_min=float(margin[self.mask].min()),neighbor_min=float(margin[~self.mask].min())),hard_quality_margins=hard_margins,hard_quality=current_quality,certificate_results=[dict(pass_gate=v.get('pass_gate'),reason=v.get('reason'),checks=v.get('checks',[]))for v in newcert],phase_trials=event.get('phase_trials',[]),convergence_thresholds=dict(sigma=atol,normalized_step=stol,consecutive_required=5),stable_iterations=self.stable,elapsed_seconds=time.perf_counter()-self.started)
  self.records.append(clean(record));write(self.root/'ITERATIONS.json',self.records)
  np.savez_compressed(self.root/('accepted_state_%03d.npz'%iteration),direction=z,schedule=h,positions=x,sigma=sig,error=err)
  if self.success is not None and self.success(rows):return 'DiagnosticMaterialTargetReached'
  if self.stable>=5:return 'DeterministicConvergenceCriteriaSatisfied'
  return None

def plan_diagnostic(state,nominal,accepted_schedule,owner,baseline_sigma,compression_evaluator,continuous_admission,*,root,max_iterations=100,success=None):
 c=context(state.faces,owner['owned_vertices']);space=direction_space(state,nominal,owner);z=np.zeros(len(space['dofs']));history=[];rho=.5
 if not len(z):return dict(reason='DirectionInfeasible',qualified=False,history=[],direction=z)
 h,x,g,r=compression_evaluator(state,nominal,owner)
 if not r['pass_gate']:return dict(reason=r['reason'],qualified=False,history=[],direction=z)
 initial=g['sigma'].copy();quality=r['quality'];observer=IterationObserver(state,owner,baseline_sigma,space,root,success);observer.initial_positive_scales={key:g[key].copy() for key in ('jacobian','V','signed_volume','orientation')};assert all(np.all(v>0) for v in observer.initial_positive_scales.values());reason='DiagnosticIterationCeiling'
 ti=np.flatnonzero(np.repeat(np.isin(c['columns'],owner['target_columns']),20))
 for iteration in range(max_iterations):
  previous_min=float(g['sigma'][ti].min());J,gap=direction_jacobian(state,z,space,c,h,x,g);step,lr=lexicographic_step(z,space,c['columns'],owner['target_columns'],baseline_sigma,g['sigma'],J,rho)
  event=dict(iteration=iteration,linear=lr,singular_gap=gap,rho=rho)
  if step is None:
   history.append(event);observer.record(iteration,event,z,previous_min,h,x,g,quality);reason='LinearizedSearchFailure';break
  trials=[];ok=False
  for proposal in lr.get('phase_candidates',[dict(phase='preferred',step=step.tolist())]):
   trial_z=z+np.asarray(proposal['step']);trial_state=planned_state(state,trial_z,space);hh,xx,gg,rr=compression_evaluator(trial_state,nominal,owner);guard=declared_guards(baseline_sigma,gg['sigma'],c['columns'],owner['target_columns']);gain=float(gg['sigma'][ti].min()-g['sigma'][ti].min())
   ok=rr['pass_gate']and guard['pass_gate']and gain>TAU
   if ok:ok=continuous_admission(trial_state,hh,owner)
   trials.append(dict(phase=proposal['phase'],compression=rr,guards=guard,finite_gain=gain,accepted=bool(ok)))
   if ok:break
  event.update(compression=rr,guards=guard,finite_gain=gain,accepted=bool(ok),phase_trials=trials,selected_phase=trials[-1]['phase']);history.append(event)
  print('Planner iteration',iteration,'accepted',bool(ok),'finite_gain',gain,'rho',rho,flush=True)
  if ok:z,h,x,g,quality=trial_z,hh,xx,gg,rr['quality']
  else:rho*=.5
  stop=observer.record(iteration,event,z,previous_min,h,x,g,quality)
  if stop:reason=stop;break
 write(Path(root)/'STOPPING_RESULT.json',dict(reason=reason,iterations=len(history),max_iterations=max_iterations,convergence_established=reason=='DeterministicConvergenceCriteriaSatisfied',objective_changed=False,trust_region_mathematics_changed=False,candidate_ranking_changed=False))
 return dict(direction=z,schedule=h,positions=x,geometry=g,history=history,space=space,qualified=False,reason=reason,initial_sigma=initial)
