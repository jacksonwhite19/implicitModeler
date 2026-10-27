"""Generic shared-front proposals; ordinary V4 owners still admit every cell.

A geometric plane kernel is a capability test, not shape dispatch or source
authority. No archived acceptance, accepted-prefix edit, or changed gate.
"""
import time
import numpy as np
from tet_layer_row import RowConflict,propose_row,signed_six
from sdf_cfd_mesher_v4.common.authority import identity
from transition_construction_evidence import emit,array_record

def kernel(front,faces,controls,check=None):
 """Return a strictly interior cap-plane kernel proposal if this LP finds one."""
 from scipy.optimize import linprog
 if check:check()
 p=np.asarray(front,dtype='<f8');f=np.asarray(faces,dtype='<i8')
 centre=p.mean(axis=0);scale=float(np.max(np.ptp(p,axis=0)))
 if not np.isfinite(scale) or scale<=0:raise RowConflict('CapKernelScaleConflict')
 x=(p-centre)/scale;tri=x[f];S=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);norm=np.linalg.norm(S,axis=1)
 if np.any(norm<=0):raise RowConflict('CapKernelDegenerateFace')
 N=S/norm[:,None];b=np.einsum('fi,fi->f',N,tri[:,0])
 def valid(c):
  margin=np.einsum('fi,fi->f',S,tri[:,0]-(c-centre)/scale)
  return np.isfinite(c).all() and np.all(margin>0)
 base=dict(controls=controls,normalization_centre_m=centre,scale_m=scale,normal=N,plane_bound_normalized=b,
  mean_margin_m=b*scale,faces=f,front=p,formula='N dot x + radius <= b in (front-mean)/scale coordinates; radius>=0')
 if valid(centre):
  emit(check,'cap_plane_kernel',**base,outcome='success',method='mean',point_m=centre)
  return centre,dict(method='ActualCapPlaneKernel/mean',minimum_plane_margin=float(np.min(b))*scale)
 if check:check()
 fit=linprog([0.,0.,0.,-1.],A_ub=np.c_[N,np.ones(len(N))],b_ub=b,
  bounds=list(zip(x.min(axis=0),x.max(axis=0)))+[(0.,None)],method='highs',options=dict(maxiter=controls['maxiter'],time_limit=check.remaining_seconds() if check and hasattr(check,'remaining_seconds') else controls['seconds']))
 if check:check()
 fit_record={k:getattr(fit,k,None) for k in ('success','status','message','nit','x','fun','slack','con','ineqlin','lower','upper')}
 if not fit.success or not np.isfinite(fit.x).all() or fit.x[3]<=0:
  ref=emit(check,'cap_plane_kernel',**base,outcome='failure',reason='CapKernelProposalConflict',solver=fit_record)
  raise RowConflict('CapKernelProposalConflict',status=int(fit.status),message=str(fit.message),iterations=int(fit.nit),diagnostic_identity=ref)
 c=centre+scale*fit.x[:3]
 if not valid(c):
  ref=emit(check,'cap_plane_kernel',**base,outcome='failure',reason='CapKernelEndpointRepresentabilityConflict',solver=fit_record,point_m=c)
  raise RowConflict('CapKernelEndpointRepresentabilityConflict',diagnostic_identity=ref)
 emit(check,'cap_plane_kernel',**base,outcome='success',method='ChebyshevLP',solver=fit_record,point_m=c)
 return c,dict(method='ActualCapPlaneKernel/ChebyshevLP',minimum_plane_margin=float(np.min(b-N@fit.x[:3]))*scale)

def radial_row(front,faces,centres,volumes,controls,original,level,bounds,centre,check=None,*,metric=None,objective='metric'):
 """A kernel radial product has positive algebraic factors for all three roles.

Each ray can have a different positive increment. Floating placement remains
only a proposal, checked by existing exact signs/quality/source owners.
"""
 p=np.asarray(front);f=np.asarray(faces);tri=p[f];S=.5*np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);area=np.linalg.norm(S,axis=1);N=S/area[:,None];C=tri.mean(axis=1)
 if check:check()
 rays=p-centre;length=np.linalg.norm(rays,axis=1)
 if np.any(length<=0):raise RowConflict('RadialFrontAtKernel')
 direction=rays/length[:,None];projection=np.einsum('fvi,fi->fv',direction[f],N)
 if np.any(projection<=0):raise RowConflict('RadialKernelDirectionConflict')
 dp=np.einsum('fi,fi->f',N,C-centres)
 if np.any(dp<=0) or np.any(volumes<=0):raise RowConflict('RadialActualOwnerConflict')
 lower=np.maximum(4*.055/(1-.055)*dp,.03*volumes/area);upper=np.minimum(4*(1-.055)/.055*dp,300*volumes/area)
 apex=f.max(axis=1);component=np.einsum('fi,fi->f',direction[apex],N);offset=np.einsum('fi,fi->f',N,p[apex]-C)
 lo=np.zeros(len(p));hi=np.full(len(p),np.inf)
 np.maximum.at(lo,apex,(lower-offset)/component);np.minimum.at(hi,apex,(upper-offset)/component)
 radius=(level+1)/controls['rows']*controls['maximum_distance_m'];delta=p-original;b=np.einsum('vi,vi->v',delta,direction);c=np.einsum('vi,vi->v',delta,delta)-radius*radius
 if np.any(c>0):raise RowConflict('RadialPreviousDistanceConflict')
 maximum=-b+np.sqrt(np.maximum(0,b*b-c));hi=np.minimum(hi,maximum)
 if bounds is not None:
  bounds=np.asarray(bounds)
  for axis in range(3):
   d=direction[:,axis];active=d!=0
   hi[active]=np.minimum(hi[active],(np.where(d[active]>0,bounds[axis,1],bounds[axis,0])-p[active,axis])/d[active])
 hi=np.nextafter(hi,0.);lo=np.maximum(lo,0.)
 if np.any(lo>hi) or np.any(hi<=0):raise RowConflict('RadialOwnerDistanceConflict',vertices=np.flatnonzero((lo>hi)|(hi<=0)).tolist())
 wanted=np.zeros(len(p));count=np.zeros(len(p));np.add.at(wanted,f.ravel(),(4*dp[:,None]/projection).ravel());np.add.at(count,f.ravel(),1)
 depth=np.minimum(hi,np.maximum(np.nextafter(lo,np.inf),wanted/count))
 from tet_layer_radial_depth import choose_floor_blend,improve_depths
 if metric is None:
  depth,depth_record=choose_floor_blend(p,f,direction,depth,lo,hi,controls,check)
 else:
  from front_row_objective import propose
  depth,depth_record=propose(p,f,direction,depth,lo,hi,metric,controls,check,objective=objective)
 depth,sweep_record=improve_depths(p,f,direction,depth,lo,hi,controls,check)
 depth_record['complete_top_vertex_sweeps']=sweep_record
 new=p+depth[:,None]*direction
 # Preserve closed owner inequalities at the realized binary64 endpoint.
 # A lower-bound depth rounded in world coordinates can fall one ULP below
 # its face height. Move the proposal inward/outward; never relax the bound.
 for attempt in range(controls['maxiter']):
  if check:check()
  heights=np.einsum('fi,fi->f',N,new[apex]-C)
  low=heights<lower;high=heights>upper
  if not low.any() and not high.any():break
  step=np.max(np.divide(np.abs(np.spacing(new)),np.abs(direction),out=np.zeros_like(new),where=direction!=0),axis=1)
  increase=np.zeros(len(p));decrease=np.zeros(len(p))
  np.maximum.at(increase,apex[low],(lower[low]-heights[low])/component[low]);np.maximum.at(decrease,apex[high],(heights[high]-upper[high])/component[high])
  lowv=np.unique(apex[low]);highv=np.unique(apex[high])
  depth[lowv]+=np.maximum(increase[lowv],step[lowv]);depth[highv]-=np.maximum(decrease[highv],step[highv])
  if np.any(depth>hi) or np.any(depth<lo):raise RowConflict('RadialClosedOwnerRepresentabilityConflict')
  new=p+depth[:,None]*direction
 for attempt in range(controls['maxiter']):
  if check:check()
  outside=np.linalg.norm(new-original,axis=1)>radius
  if bounds is not None:outside|=np.any((new<=bounds[:,0])|(new>=bounds[:,1]),axis=1)
  if not outside.any():break
  new[outside]=np.nextafter(new[outside],p[outside])
 signs=signed_six(p,new,f);heights=np.einsum('fi,fi->f',N,new[apex]-C)
 if np.any(signs<=0) or np.any(heights<lower) or np.any(heights>upper) or np.any(np.linalg.norm(new-original,axis=1)>radius):raise RowConflict('RadialEndpointConflict',nonpositive=int(np.sum(signs<=0)),below_owner=int(np.sum(heights<lower)),above_owner=int(np.sum(heights>upper)),outside_distance=int(np.sum(np.linalg.norm(new-original,axis=1)>radius)))
 if bounds is not None and (np.any(new<=bounds[:,0]) or np.any(new>=bounds[:,1])):raise RowConflict('RadialEndpointDomainConflict')
 return dict(front=new,direction=direction,displacement=new-p,diagnostics=dict(method='ActualCapKernel/RadialSharedRows',row_index=level,depth_proposal=depth_record,floating_nonpositive_by_role=np.sum(signs<=0,axis=0).tolist(),actual_owner_intervals_used=len(f),minimum_signed_six=float(np.min(signs)),proposal_only=True))

def complete_rows(front,faces,centres,volumes,controls,bounds=None,check=None,*,metric=None,row_finish=None,objective='metric',_diagnostic_context=None,_proposal_policy=None):
 """Construct from actual preceding rows; no extrapolated original directions."""
 from background_materialization import build
 from background_mixed_geometry import batch_geometry
 from tet_layer_row import product
 if _proposal_policy is not None:
  from coupled_transition_row import validate_policy
  validate_policy(_proposal_policy)
  if metric is not None:raise ValueError('CoupledDisplacementMetricPolicyUnsupported')
 outer_check=check;deadline=time.perf_counter()+controls['seconds']
 def check():
  if outer_check:outer_check()
  if time.perf_counter()>=deadline:raise RowConflict('CompleteRowPlanningTimeLimit')
 check.remaining_seconds=lambda:min(max(0.,deadline-time.perf_counter()),outer_check.remaining_seconds() if outer_check and hasattr(outer_check,'remaining_seconds') else controls['seconds'])
 for name in ('repair_progress','repair_diagnostics','retain_partial_row','transition_diagnostic_context','coupled_vertex_trace'):
  if outer_check is not None and hasattr(outer_check,name):setattr(check,name,getattr(outer_check,name))
 if _diagnostic_context is not None:check.transition_diagnostic_context=_diagnostic_context
 emit(check,'transition_initialization_operands',front=array_record(front),faces=array_record(faces),
  actual_owner_centres=array_record(centres),actual_owner_volumes=array_record(volumes),controls=controls,bounds=bounds)
 original=np.asarray(front).copy();current=original.copy();points=[original];records=[];kernel_point=None;kernel_record=None
 if metric is not None:
  metric=np.asarray(metric)
  if metric.shape!=((controls['rows']+1)*len(original),3,3):raise RowConflict('InvalidCompleteRowMetric')
 try:kernel_point,kernel_record=kernel(current,faces,controls,check)
 except RowConflict as error:
  if error.reason.endswith('TimeLimit'):raise
  kernel_record=dict(reason=error.reason,details=error.details)
 for level in range(controls['rows']):
  if check:check()
  if level and row_finish is not None:
   # Coordinate completion changes the actual front. Rebuild this geometric
   # capability instead of using the original cap's now stale kernel.
   try:kernel_point,kernel_record=kernel(current,faces,controls,check)
   except RowConflict as error:
    if _proposal_policy is None or error.reason.endswith('TimeLimit'):raise
    kernel_point=None;kernel_record=dict(reason=error.reason,details=error.details)
  try:
   if kernel_point is None:raise RowConflict('NoCapKernelProposal',current_front_kernel=kernel_record)
   row_metric=None if metric is None else metric[level*len(original):(level+2)*len(original)]
   result=radial_row(current,faces,centres,volumes,controls,original,level,bounds,kernel_point,check,metric=row_metric,objective=objective)
  except RowConflict as radial_conflict:
   if radial_conflict.reason.endswith('TimeLimit'):raise
   # The candidate must expose a failed metric-construction domain. Do not
   # silently call an unshaped fallback and claim the requested method ran.
   if metric is not None:raise
   emit(check,'radial_proposal_refusal',row_index=level,reason=radial_conflict.reason,details=radial_conflict.details,kernel=kernel_record)
   try:result=propose_row(current,faces,centres,volumes,controls,original=original,row_index=level,bounds=bounds,check=check)
   except RowConflict as fallback_conflict:
    ref=emit(check,'transition_initialization_refusal',row_index=level,kernel=kernel_record,
     radial=dict(reason=radial_conflict.reason,details=radial_conflict.details),fallback=dict(reason=fallback_conflict.reason,details=fallback_conflict.details))
    fallback_conflict.details.update(current_front_kernel=kernel_record,radial_fallback=dict(reason=radial_conflict.reason,details=radial_conflict.details),diagnostic_refusal_identity=ref)
    if _proposal_policy is None or fallback_conflict.reason!='SharedActualOwnerIntervalConflict':raise
    from coupled_transition_row import propose as coupled_propose
    result=coupled_propose(current,faces,centres,volumes,controls,original=original,row_index=level,bounds=bounds,check=check,policy=_proposal_policy)
    result['diagnostics']['legacy_refusal']=dict(reason=fallback_conflict.reason,details=fallback_conflict.details)
   result['diagnostics']['radial_fallback']=dict(reason=radial_conflict.reason,details=radial_conflict.details)
  next_front=result['front']
  result['diagnostics']['current_front_kernel']=kernel_record
  if row_finish is not None:
   from front_owner_endpoint import close_weight
   next_front,endpoint=close_weight(current,next_front,faces,centres,volumes,controls,check)
   result['diagnostics']['actual_weight_endpoint']=endpoint
  if result['diagnostics'].get('method') in ('V35CoupledSharedDisplacement/1','V35CoupledSharedDisplacement/2'):
   from coupled_transition_row import validate_endpoint
   result['diagnostics']['post_rounding_endpoint']=validate_endpoint(current,next_front,faces,centres,volumes,controls,original,level,bounds,check)
  points.append(next_front);records.append(result['diagnostics'])
  if row_finish is not None:
   completed,quality_record=row_finish(np.stack(points),records,check)
   points=[p.copy() for p in completed];current=points[-2];next_front=points[-1]
   records[-1]['row_quality_completion']=quality_record
  # Actual outer owner is staircase role2, with the same winding/materialization
  # and Foundation geometry as the ordinary mixed acceptance owner.
  localpoints=np.concatenate((current,next_front));cells=build(product(faces,len(current)),0);outer=[cells[j] for j in range(2,len(cells),3)];geometry=batch_geometry(localpoints,outer)
  centres=np.asarray([geometry[c['id']]['centre'] for c in outer]);volumes=np.asarray([geometry[c['id']]['volume'] for c in outer]);current=next_front
 check()
 return np.stack(points),dict(kernel=kernel_record,rows=records,proposal_only=True)

def co_design(authority,wall,first,faces,refs,increments,positions,vectors,background_policy,*,_hint_out=None,_observer=None):
 """Future-only prism proposals; caller rechecks ordinary full prism owners."""
 from accepted_trajectory_contract import _planned_quality
 if _hint_out is not None and (type(_hint_out) is not dict or _hint_out):raise ValueError('EmptyPrivateTransitionProposalOutputRequired')
 from tet_layer_controls import variable_shape
 if variable_shape(background_policy['tet_transition']):
  from tet_layer_controls import seed_controls
  seed_policy=dict(background_policy,tet_transition=seed_controls(background_policy['tet_transition']))
  pos,vec,report=co_design(authority,wall,first,faces,refs,increments,positions,vectors,seed_policy,_observer=_observer)
  return pos,vec,dict(report,shape_transition_scope='Prism/positive-seed forecast only. Shape remeshing and all admission occur later against actual accepted cap.',shape_volume_hint_reused=False)
 controls=background_policy['tet_transition'];bounds=np.asarray(background_policy['bounds_m']);start=time.perf_counter();deadline=start+controls['seconds']
 from contextlib import nullcontext
 with (_observer.upstream() if _observer is not None else nullcontext()):
  def check():
   authority.verify()
   if time.perf_counter()>=deadline:raise RowConflict('CoupledTransitionPlanningTimeLimit')
  check.remaining_seconds=lambda:max(0.,deadline-time.perf_counter())
  check();original=np.asarray(positions).copy();original_vectors=np.asarray(vectors).copy();attempts=[]
  def trial(pos):
   check()
   with (_observer.phase('forecast.prism_quality') if _observer is not None else nullcontext()):
    points,cells,outer,owners,checked=_planned_quality(wall,faces,pos)
   if _observer is not None:_observer.quality(checked['quality']['failures'],'forecast.prism_quality')
   if not checked['quality']['passed']:raise RowConflict('CoupledPrismQualityConflict',records=len(checked['quality']['failures']))
   with (_observer.phase('forecast.terminal_geometry') if _observer is not None else nullcontext()):
    from background_mixed_geometry import batch_geometry
    population={c['id']:c for c in cells};terminal=[owners[tuple(sorted(f))][0][0] for f in outer];g=batch_geometry(points,[population[i] for i in dict.fromkeys(terminal)])
    # Reproduce the actual native interface's lexical persistent source order,
    # oriented cycle and hashed face order. Numeric-order diagonals can differ.
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import cycle,face_key
    n=len(wall);level=len(pos)-1;names=[f'layer:{level}:source:{i}' for i in range(n)];lookup={name:i for i,name in enumerate(names)};order=sorted(range(n),key=lambda i:names[i]);inverse={v:i for i,v in enumerate(order)}
    ordered=[]
    for face,owner in zip(outer,terminal):
     native=cycle(tuple(names[int(v)-level*n] for v in face));ordered.append((face_key(native),[inverse[lookup[v]] for v in native],owner))
    ordered.sort(key=lambda row:row[0]);cap=pos[-1][order];compact=np.asarray([row[1] for row in ordered],dtype='<i8');terminal=[row[2] for row in ordered]
    centres=np.asarray([g[i]['centre'] for i in terminal]);volumes=np.asarray([g[i]['volume'] for i in terminal])
   from tet_layer_adapter import initialize,place
   with (_observer.phase('forecast.initialize') if _observer is not None else nullcontext()):
    state=initialize(points,cells,owners,np.asarray(order,dtype='<i8')+level*n,compact,terminal,controls,bounds,prism_centres=centres,prism_volumes=volumes,check=check)
   check()
   if _observer is not None:
    check.repair_progress=_observer.placement()
    check.repair_observer=_observer
   with (_observer.phase('forecast.placement') if _observer is not None else nullcontext()):
    placement=place(state,check=check)
   result=placement['quality'];check()
   if not result['passed']:raise RowConflict('CoupledTransitionQualityConflict',records=len(result['failures']),summary=result['summary'],first_failures=result['failures'][:3])
   fronts=state['points'][state['compact_ids']].reshape(controls['rows']+1,len(cap),3).copy()
   hint=dict(cap=cap.copy(),faces=compact.copy(),fronts=fronts,direction=state['direction'].copy(),controls_identity=identity(controls),construction=dict(state['construction'],forecast_placement=dict(initial_records=placement['initial_records'],groups=len(placement['groups']),sweeps=placement['sweeps'],seconds=placement['seconds'])))
   return dict(state['construction'],numeric_quality=result['summary'],complete_exact_quality_passed=True,source_collision_filling_admission_required=True),hint
  try:
   summary,hint=trial(original);check()
   if _hint_out is not None:_hint_out.update(hint)
   return original,original_vectors,dict(method='CompleteSharedRowForecast/unchangedPrisms',transition=summary,seconds=time.perf_counter()-start,proposal_only=True)
  except RowConflict as error:
   if error.reason.endswith('TimeLimit'):raise
   attempts.append(dict(method='unchangedPrisms',reason=error.reason,details=error.details))
  check()
  if len(increments)<=1:raise RowConflict('ImmutableFirstFrontTransitionConflict',attempts=attempts)
  # Geometric kernel capability of actual fixed first front, with no shape-name
  # or SDF formula selection. All later directions are new proposals only.
  with (_observer.phase('forecast.fallback') if _observer is not None else nullcontext()):
   centre,capability=kernel(first,faces,controls,check);pos=original.copy();vec=original_vectors.copy()
   for level in range(2,len(pos)):
    direction=pos[level-1]-centre;direction/=np.linalg.norm(direction,axis=1)[:,None];direction[direction==0]=0.;vec[level-2]=direction;pos[level]=pos[level-1]+increments[level-1,:,None]*direction
  summary,hint=trial(pos);check()
  if _hint_out is not None:_hint_out.update(hint)
  return pos,vec,dict(method='CompleteSharedRowForecast/kernelRadialFuturePrisms',kernel=capability,attempts=attempts,transition=summary,first_unchanged=True,path_lengths_unchanged=True,seconds=time.perf_counter()-start,proposal_only=True)
