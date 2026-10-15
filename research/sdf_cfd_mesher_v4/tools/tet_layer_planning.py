"""Generic shared-front proposals; ordinary V4 owners still admit every cell.

A geometric plane kernel is a capability test, not shape dispatch or source
authority. No archived acceptance, accepted-prefix edit, or changed gate.
"""
import time
import numpy as np
from tet_layer_row import RowConflict,propose_row,signed_six
from sdf_cfd_mesher_v4.common.authority import identity

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
 if valid(centre):return centre,dict(method='ActualCapPlaneKernel/mean',minimum_plane_margin=float(np.min(b))*scale)
 if check:check()
 fit=linprog([0.,0.,0.,-1.],A_ub=np.c_[N,np.ones(len(N))],b_ub=b,
  bounds=list(zip(x.min(axis=0),x.max(axis=0)))+[(0.,None)],method='highs',options=dict(maxiter=controls['maxiter'],time_limit=check.remaining_seconds() if check and hasattr(check,'remaining_seconds') else controls['seconds']))
 if check:check()
 if not fit.success or not np.isfinite(fit.x).all() or fit.x[3]<=0:raise RowConflict('CapKernelProposalConflict',status=int(fit.status))
 c=centre+scale*fit.x[:3]
 if not valid(c):raise RowConflict('CapKernelEndpointRepresentabilityConflict')
 return c,dict(method='ActualCapPlaneKernel/ChebyshevLP',minimum_plane_margin=float(np.min(b-N@fit.x[:3]))*scale)

def radial_row(front,faces,centres,volumes,controls,original,level,bounds,centre,check=None):
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
 depth,depth_record=choose_floor_blend(p,f,direction,depth,lo,hi,controls,check)
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

def complete_rows(front,faces,centres,volumes,controls,bounds=None,check=None):
 """Construct from actual preceding rows; no extrapolated original directions."""
 from background_materialization import build
 from background_mixed_geometry import batch_geometry
 from tet_layer_row import product
 outer_check=check;deadline=time.perf_counter()+controls['seconds']
 def check():
  if outer_check:outer_check()
  if time.perf_counter()>=deadline:raise RowConflict('CompleteRowPlanningTimeLimit')
 check.remaining_seconds=lambda:min(max(0.,deadline-time.perf_counter()),outer_check.remaining_seconds() if outer_check and hasattr(outer_check,'remaining_seconds') else controls['seconds'])
 original=np.asarray(front).copy();current=original.copy();points=[original];records=[];kernel_point=None;kernel_record=None
 try:kernel_point,kernel_record=kernel(current,faces,controls,check)
 except RowConflict as error:
  if error.reason.endswith('TimeLimit'):raise
  kernel_record=dict(reason=error.reason,details=error.details)
 for level in range(controls['rows']):
  if check:check()
  try:
   if kernel_point is None:raise RowConflict('NoCapKernelProposal')
   result=radial_row(current,faces,centres,volumes,controls,original,level,bounds,kernel_point,check)
  except RowConflict as radial_conflict:
   if radial_conflict.reason.endswith('TimeLimit'):raise
   result=propose_row(current,faces,centres,volumes,controls,original=original,row_index=level,bounds=bounds,check=check)
   result['diagnostics']['radial_fallback']=dict(reason=radial_conflict.reason,details=radial_conflict.details)
  next_front=result['front'];points.append(next_front);records.append(result['diagnostics'])
  # Actual outer owner is staircase role2, with the same winding/materialization
  # and Foundation geometry as the ordinary mixed acceptance owner.
  localpoints=np.concatenate((current,next_front));cells=build(product(faces,len(current)),0);outer=[cells[j] for j in range(2,len(cells),3)];geometry=batch_geometry(localpoints,outer)
  centres=np.asarray([geometry[c['id']]['centre'] for c in outer]);volumes=np.asarray([geometry[c['id']]['volume'] for c in outer]);current=next_front
 check()
 return np.stack(points),dict(kernel=kernel_record,rows=records,proposal_only=True)

def co_design(authority,wall,first,faces,refs,increments,positions,vectors,background_policy,*,_hint_out=None):
 """Future-only prism proposals; caller rechecks ordinary full prism owners."""
 from accepted_trajectory_contract import _planned_quality
 if _hint_out is not None and (type(_hint_out) is not dict or _hint_out):raise ValueError('EmptyPrivateTransitionProposalOutputRequired')
 from tet_layer_controls import variable_shape
 if variable_shape(background_policy['tet_transition']):
  from tet_layer_controls import seed_controls
  seed_policy=dict(background_policy,tet_transition=seed_controls(background_policy['tet_transition']))
  pos,vec,report=co_design(authority,wall,first,faces,refs,increments,positions,vectors,seed_policy)
  return pos,vec,dict(report,shape_transition_scope='Prism/positive-seed forecast only. Shape remeshing and all admission occur later against actual accepted cap.',shape_volume_hint_reused=False)
 controls=background_policy['tet_transition'];bounds=np.asarray(background_policy['bounds_m']);start=time.perf_counter();deadline=start+controls['seconds']
 def check():
  authority.verify()
  if time.perf_counter()>=deadline:raise RowConflict('CoupledTransitionPlanningTimeLimit')
 check.remaining_seconds=lambda:max(0.,deadline-time.perf_counter())
 check();original=np.asarray(positions).copy();original_vectors=np.asarray(vectors).copy();attempts=[]
 def trial(pos):
  check()
  points,cells,outer,owners,checked=_planned_quality(wall,faces,pos)
  if not checked['quality']['passed']:raise RowConflict('CoupledPrismQualityConflict',records=len(checked['quality']['failures']))
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
  state=initialize(points,cells,owners,np.asarray(order,dtype='<i8')+level*n,compact,terminal,controls,bounds,prism_centres=centres,prism_volumes=volumes,check=check)
  check();placement=place(state,check=check);result=placement['quality'];check()
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
 centre,capability=kernel(first,faces,controls,check);pos=original.copy();vec=original_vectors.copy()
 for level in range(2,len(pos)):
  direction=pos[level-1]-centre;direction/=np.linalg.norm(direction,axis=1)[:,None];direction[direction==0]=0.;vec[level-2]=direction;pos[level]=pos[level-1]+increments[level-1,:,None]*direction
 summary,hint=trial(pos);check()
 if _hint_out is not None:_hint_out.update(hint)
 return pos,vec,dict(method='CompleteSharedRowForecast/kernelRadialFuturePrisms',kernel=capability,attempts=attempts,transition=summary,first_unchanged=True,path_lengths_unchanged=True,seconds=time.perf_counter()-start,proposal_only=True)
