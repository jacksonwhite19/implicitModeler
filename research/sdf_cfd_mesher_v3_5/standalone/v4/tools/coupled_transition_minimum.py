"""Policy 2: physical minimum movement within original cumulative cube/domain.

Every LP remains a floating proposal; no global feasibility/admission claim.
"""
import time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity
from tet_layer_row import RowConflict,_descending,signed_six
from transition_construction_evidence import emit
from coupled_displacement_minimum import solve,box,TAU

def configuration():
    body=dict(schema='V35CoupledSharedDisplacement/2',seed='AllIncidentNormalAndActualOwnerSlabs',
        objective='PhysicalL1DisplacementFromCurrentFront/seed-and-greedy-completion',
        coordinate_domain='Original cumulative-radius inscribed cube intersect strict background domain; no owner-scale box',
        variable_scaling='mean(4*actual_owner_dp); never a physical bound',constraint_normalization='cumulative row radius',
        primal_dual_tolerance=TAU,physical_residual_budget='tau*row_radius_m',
        construction_reserve='max(64*max(abs(spacing(current,physical_box_lower,physical_box_upper))),16*tau*row_radius_m)',
        reserve_scope='Incident outward, normalized affine completed-tet planes, inward-reserved owner slabs; original actual endpoint gates remain binding',
        completion='ExistingDescendingAffineStaircase/one-pass/physical-L1',candidates_per_row=1,
        maximum_seed_lp_per_vertex=1,maximum_completion_lp_per_vertex=1,
        iterations='unchanged controls.maxiter',deadline='shared unchanged controls.seconds',
        numerical_quality='Ordinary actual-neighbour complete-prefix veto; no relaxed gates',
        legacy='Preserve successful radial/fallback bytes; enter only after SharedActualOwnerIntervalConflict')
    return dict(body,identity=identity(body))

def validate_policy(policy):
    if type(policy) is not dict or policy!=configuration():raise ValueError('InvalidCoupledMinimumPolicyIdentity')

def seed_vertex(current,original,radius,bounds,normal,lower,upper,offset,inc,cap,scale,maxiter,tick):
    lo,hi,reserve=box(current,original,radius,bounds)
    M=-normal[inc];b=np.full(len(inc),-reserve)
    if cap:
        M=np.vstack((M,-normal[cap],normal[cap]));b=np.r_[b,-(lower[cap]-offset[cap])-reserve,upper[cap]-offset[cap]-reserve]
    return solve(current,lo,hi,radius,scale,M,b,maxiter,tick,reserve=reserve)

class MinimumCompletion:
    def __init__(self,front,original,radius,bounds,dp,incidence,normal,lower,upper,controls,tick,trace):
        self.front=front;self.original=original;self.radius=radius;self.bounds=bounds;self.dp=dp;self.incidence=incidence
        self.normal=normal;self.lower=lower;self.upper=upper;self.controls=controls;self.tick=tick;self.trace=trace
    def valid(self,v,new,value,gradient,ns,heights,caprows):
        _,_,reserve=box(self.front[v],self.original[v],self.radius,self.bounds);budget=TAU*self.radius;size=np.linalg.norm(gradient,axis=1)
        valid=not np.any(size==0)
        if valid and (np.any(value/size<reserve-budget) or np.any(ns@(new-self.front[v])<reserve-budget)):valid=False
        if valid and caprows and (np.any(heights<self.lower[caprows]+reserve-budget) or np.any(heights>self.upper[caprows]-reserve+budget)):valid=False
        if self.trace is not None:self.trace('descending_minimum_seed_decision',dict(vertex=v,current_m=self.front[v],coordinates_m=new,
            signed_six=value,affine_gradient=gradient,gradient_norm=size,incident_normals=ns,incident_outward_m=ns@(new-self.front[v]),
            apex_faces=caprows,apex_heights_m=heights,lower_height_m=self.lower[caprows],upper_height_m=self.upper[caprows],
            reserve_m=reserve,physical_residual_budget_m=budget,keep_seed=bool(valid)))
        return valid
    def __call__(self,v,goal,radius,rows,rhs,indices,cells,incident,caprows,partial):
        lo,hi,reserve=box(self.front[v],self.original[v],radius,self.bounds)
        M=rows[:,:3]/radius;b=rhs-M@(self.front[v]-goal)-reserve
        scale=float(np.mean(4*self.dp[self.incidence[v]]))
        new,record=solve(self.front[v],lo,hi,radius,scale,M,b,self.controls['maxiter'],self.tick,reserve=reserve)
        record.update(vertex=v,completed_tet_indices=indices,completed_tets=cells,incident_faces=incident,apex_faces=caprows,
            original_affine_rows=rows,original_affine_rhs=rhs,goal_m=goal,partial_front=partial.copy() if new is None else None)
        if self.trace is not None:self.trace('descending_minimum_lp',record)
        if new is None:raise RowConflict('CoupledMinimumCompletionRefusal',vertex=v,status=record['solver_status'],
            scope='Greedy fixed-neighbour numerical-reserve/cumulative-cube search; not global infeasibility')
        return new,record

def propose(front,faces,owner_centres,owner_volumes,controls,*,original=None,row_index=0,bounds=None,check=None,policy=None):
    validate_policy(policy)
    for key in ('maximum_distance_m','seconds'):
        if type(controls.get(key)) not in (int,float) or not np.isfinite(controls[key]) or controls[key]<=0:raise RowConflict('InvalidCoupledControl',key=key)
    for key in ('rows','maxiter'):
        if type(controls.get(key)) is not int or controls[key]<=0:raise RowConflict('InvalidCoupledControl',key=key)
    if type(row_index) is not int or not 0<=row_index<controls['rows']:raise RowConflict('InvalidCoupledRowIndex')
    raw_faces=np.asarray(faces)
    if raw_faces.dtype.kind not in 'iu':raise RowConflict('InvalidCoupledFaceIndexType')
    p=np.asarray(front,dtype='<f8');f=np.asarray(faces,dtype='<i8');owners=np.asarray(owner_centres,dtype='<f8');volume=np.asarray(owner_volumes,dtype='<f8')
    n=len(p);original=p if original is None else np.asarray(original,dtype='<f8')
    if p.shape!=(n,3) or n==0 or original.shape!=p.shape or f.ndim!=2 or f.shape[1:]!=(3,) or len(f)==0 or owners.shape!=(len(f),3) or volume.shape!=(len(f),):raise RowConflict('InvalidCoupledGeometryShape')
    if not all(np.isfinite(x).all() for x in (p,original,owners,volume)) or np.any(volume<=0) or f.min()<0 or f.max()>=n:raise RowConflict('InvalidCoupledGeometryValues')
    if any(len(set(row))!=3 for row in f) or len({tuple(sorted(row)) for row in f})!=len(f):raise RowConflict('InvalidCoupledFaceConnectivity')
    if bounds is not None:
        bounds=np.asarray(bounds,dtype='<f8')
        if bounds.shape!=(3,2) or not np.isfinite(bounds).all() or np.any(bounds[:,0]>=bounds[:,1]):raise RowConflict('InvalidCoupledBounds')
        if np.any(p<=bounds[:,0]) or np.any(p>=bounds[:,1]):raise RowConflict('CoupledFrontOutsideDomain')
    start=time.perf_counter();deadline=start+controls['seconds']
    def tick():
        if check is not None:check()
        if time.perf_counter()>=deadline:raise RowConflict('CoupledRowTimeLimit')
    tick.remaining_seconds=lambda:min(max(0.,deadline-time.perf_counter()),check.remaining_seconds() if check is not None and hasattr(check,'remaining_seconds') else controls['seconds'])
    tick();tri=p[f];s=.5*np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);area=np.linalg.norm(s,axis=1)
    if np.any(area<=0):raise RowConflict('CoupledDegenerateCap')
    normal=s/area[:,None];centre=tri.mean(axis=1);dp=np.einsum('fi,fi->f',normal,centre-owners)
    if np.any(dp<=0):raise RowConflict('CoupledInvalidActualOwner')
    lower=np.maximum(4*.055/(1-.055)*dp,.03*volume/area);upper=np.minimum(4*(1-.055)/.055*dp,300*volume/area)
    if np.any(lower>upper):raise RowConflict('CoupledEmptyFaceSlab')
    apex=f.max(axis=1);offset=np.einsum('fi,fi->f',normal,p[apex]-centre)
    incidence=[[] for _ in range(n)];caps=[[] for _ in range(n)]
    for j,face in enumerate(f):
        caps[apex[j]].append(j)
        for v in face:incidence[v].append(j)
    if any(not row for row in incidence):raise RowConflict('CoupledUnusedVertex')
    radius=(row_index+1)/controls['rows']*controls['maximum_distance_m'];half=radius/np.sqrt(3)
    seed=p.copy();records=[];trace=getattr(check,'coupled_vertex_trace',None)
    emit(check,'coupled_row_inputs',policy=policy,row_index=row_index,front=p,faces=f,actual_owner_centres=owners,actual_owner_volumes=volume,
        normal=normal,area=area,cap_centre=centre,dp=dp,lower=lower,upper=upper,apex=apex,plane_offset=offset,original=original,radius_m=radius,bounds=bounds)
    for v in range(n):
        tick();inc=incidence[v];cap=caps[v];scale=float(np.mean(4*dp[inc]))
        placed,record=seed_vertex(p[v],original[v],radius,bounds,normal,lower,upper,offset,inc,cap,scale,controls['maxiter'],tick)
        record.update(vertex=v,incident_faces=inc,apex_faces=cap)
        if trace is not None:trace('seed_minimum_lp',record)
        if placed is None:
            ref=emit(check,'coupled_seed_refusal',policy=policy,row_index=row_index,vertex=v,model=record,partial_seed=seed,
                domain='Original cumulative cube/domain with declared numerical reserve; no owner-scale box; not whole-row infeasibility')
            raise RowConflict('CoupledMinimumSeedRefusal',vertex=v,status=record['solver_status'],diagnostic_identity=ref)
        seed[v]=placed;records.append(dict(vertex=v,L1_m=record['L1_m'],L2_m=record['L2_m'],reserve_m=record['requested_reserve_m'],iterations=record['iterations']))
    emit(check,'coupled_seed_endpoint',policy=policy,row_index=row_index,coordinates=seed,displacement=seed-p,records=records)
    try:
        new,completion=_descending(p,f,seed,original,radius,bounds,normal,centre,lower,upper,controls['maxiter'],tick,_trace=trace,_minimum_solver=MinimumCompletion(p,original,radius,bounds,dp,incidence,normal,lower,upper,controls,tick,trace))
    except RowConflict as error:
        ref=emit(check,'coupled_completion_refusal',policy=policy,row_index=row_index,reason=error.reason,details=error.details,seed=seed)
        error.details.update(proposal_policy=policy['identity'],diagnostic_identity=ref,scope='Greedy completed-neighbour/cube search refusal; not global infeasibility')
        raise
    displacement=new-p;depth=np.linalg.norm(displacement,axis=1);signs=signed_six(p,new,f);heights=np.einsum('fi,fi->f',normal,new[apex]-centre)
    outward=np.einsum('fvi,fi->fv',displacement[f],normal);distance=np.linalg.norm(new-original,axis=1)
    bad=not np.isfinite(new).all() or np.any(depth<=0) or np.any(signs<=0) or np.any(outward<=0) or np.any(heights<lower) or np.any(heights>upper) or np.any(distance>radius)
    if bounds is not None:bad=bad or np.any(new<=bounds[:,0]) or np.any(new>=bounds[:,1])
    endpoint=dict(policy=policy,row_index=row_index,coordinates=new,displacement=displacement,depth=depth,
        direction=np.divide(displacement,depth[:,None],out=np.zeros_like(displacement),where=depth[:,None]>0),signed_six=signs,
        incident_outward=outward,apex_heights=heights,lower_margin=heights-lower,upper_margin=upper-heights,distance_m=distance,radius_m=radius,
        completion=completion,passed_construction=not bad,ordinary_quality_source_collision_coverage_filling_still_required=True)
    ref=emit(check,'coupled_row_endpoint',**endpoint);tick()
    if bad:raise RowConflict('CoupledEndpointRefusal',diagnostic_identity=ref)
    return dict(front=new,direction=endpoint['direction'],displacement=displacement,diagnostics=dict(method=policy['schema'],proposal_policy=policy,
        proposal_only=True,row_index=row_index,vertices=n,tetrahedra=3*len(f),seed_lp_calls=n,completion_lp_calls=sum(x['used_lp'] for x in completion),
        minimum_signed_six=float(signs.min()),minimum_lower_margin_m=float((heights-lower).min()),minimum_upper_margin_m=float((upper-heights).min()),
        maximum_distance_m=float(distance.max()),seconds=time.perf_counter()-start,diagnostic_identity=ref,
        ordinary_quality_source_collision_coverage_filling_still_required=True))
