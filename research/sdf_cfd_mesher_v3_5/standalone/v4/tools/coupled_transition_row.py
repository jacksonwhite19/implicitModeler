"""Bounded generic vector seed plus original affine staircase completion.

Explicit opt-in proposal policy. No row, quality or live authority is implied
by an LP solution. Greedy/cube/iteration failures are restricted search failures.
"""
import time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity
from tet_layer_row import RowConflict,_descending,signed_six
from transition_construction_evidence import emit

def configuration():
    body=dict(schema='V35CoupledSharedDisplacement/1',seed='AllIncidentNormalAndActualOwnerSlabs',
        objective='MaximizeNormalizedMinimumIncidentNormalDisplacement',
        seed_box='current_front +/- mean(4*actual_owner_dp) intersect original cumulative-radius inscribed cube and domain',
        completion='ExistingDescendingAffineStaircase/one-pass',candidates_per_row=1,
        maximum_seed_lp_per_vertex=1,maximum_completion_lp_per_vertex=1,
        iterations='unchanged controls.maxiter',deadline='shared unchanged controls.seconds',
        numerical_quality='Ordinary actual-neighbour complete-prefix veto; no relaxed gates',
        legacy='Preserve successful radial/fallback bytes; enter only after SharedActualOwnerIntervalConflict')
    return dict(body,identity=identity(body))

def validate_policy(policy):
    if type(policy) is dict and policy.get('schema')=='V35CoupledSharedDisplacement/2':
        from coupled_transition_minimum import validate_policy as validate_minimum
        return validate_minimum(policy)
    if type(policy) is not dict or policy!=configuration():raise ValueError('InvalidCoupledDisplacementPolicyIdentity')

def validate_endpoint(front,new,faces,owners,volume,controls,original,row_index,bounds=None,check=None):
    """Read-only recheck after ordinary ULP endpoint rounding; no repair."""
    p=np.asarray(front);new=np.asarray(new);f=np.asarray(faces);tri=p[f]
    s=.5*np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);area=np.linalg.norm(s,axis=1);normal=s/area[:,None];centre=tri.mean(axis=1)
    dp=np.einsum('fi,fi->f',normal,centre-owners);lower=np.maximum(4*.055/(1-.055)*dp,.03*volume/area);upper=np.minimum(4*(1-.055)/.055*dp,300*volume/area)
    displacement=new-p;heights=np.einsum('fi,fi->f',normal,new[f.max(axis=1)]-centre)
    signs=signed_six(p,new,f);outward=np.einsum('fvi,fi->fv',displacement[f],normal)
    radius=(row_index+1)/controls['rows']*controls['maximum_distance_m'];distance=np.linalg.norm(new-original,axis=1)
    passed=bool(np.isfinite(new).all() and np.all(signs>0) and np.all(outward>0) and np.all(heights>=lower) and np.all(heights<=upper) and np.all(distance<=radius))
    if bounds is not None:passed=passed and bool(np.all(new>bounds[:,0]) and np.all(new<bounds[:,1]))
    record=dict(passed=passed,row_index=row_index,coordinates=new,displacement=displacement,signed_six=signs,incident_outward=outward,
        lower_margin=heights-lower,upper_margin=upper-heights,distance_m=distance,radius_m=radius,read_only=True)
    ref=emit(check,'coupled_post_rounding_endpoint',**record)
    if not passed:raise RowConflict('CoupledPostRoundingEndpointRefusal',diagnostic_identity=ref)
    return dict(passed=True,minimum_signed_six=float(signs.min()),minimum_incident_outward_m=float(outward.min()),
        minimum_lower_margin_m=float((heights-lower).min()),minimum_upper_margin_m=float((upper-heights).min()),maximum_distance_m=float(distance.max()))

def propose(front,faces,owner_centres,owner_volumes,controls,*,original=None,row_index=0,bounds=None,check=None,policy=None):
    if type(policy) is dict and policy.get('schema')=='V35CoupledSharedDisplacement/2':
        from coupled_transition_minimum import propose as minimum_propose
        return minimum_propose(front,faces,owner_centres,owner_volumes,controls,original=original,row_index=row_index,bounds=bounds,check=check,policy=policy)
    from scipy.optimize import linprog
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
        tick();inc=incidence[v];cap=caps[v];scale=float(np.mean(4*dp[inc]));lo=np.maximum(p[v]-scale,original[v]-half);hi=np.minimum(p[v]+scale,original[v]+half)
        if bounds is not None:lo=np.maximum(lo,np.nextafter(bounds[:,0],np.inf));hi=np.minimum(hi,np.nextafter(bounds[:,1],-np.inf))
        if np.any(lo>hi):raise RowConflict('CoupledSeedBoxConflict',vertex=v,scope='Restricted deterministic box; not global infeasibility')
        lo=(lo-p[v])/scale;hi=(hi-p[v])/scale
        A=np.c_[-normal[inc],np.ones(len(inc))];rhs=np.zeros(len(inc))
        if cap:
            A=np.vstack((A,np.c_[-normal[cap],np.zeros(len(cap))],np.c_[normal[cap],np.zeros(len(cap))]))
            rhs=np.r_[rhs,-(lower[cap]-offset[cap])/scale,(upper[cap]-offset[cap])/scale]
        fit=linprog([0.,0.,0.,-1.],A_ub=A,b_ub=rhs,bounds=list(zip(lo,hi))+[(None,None)],method='highs',
            options=dict(maxiter=controls['maxiter'],time_limit=tick.remaining_seconds()))
        record=dict(vertex=v,incident_faces=inc,apex_faces=cap,origin_m=p[v],scale_m=scale,A_ub=A,b_ub=rhs,lower=lo,upper=hi,
            status=int(fit.status),message=str(fit.message),nit=int(fit.nit),solution=fit.x)
        if trace is not None:trace('seed_lp',record)
        tick()
        if not fit.success or not np.isfinite(fit.x).all() or fit.x[3]<=0:
            ref=emit(check,'coupled_seed_refusal',policy=policy,row_index=row_index,vertex=v,model=record,partial_seed=seed,
                domain='Local normal/slab LP within deterministic seed box; not whole-row infeasibility')
            raise RowConflict('CoupledSeedSearchRefusal',vertex=v,status=int(fit.status),diagnostic_identity=ref)
        seed[v]=p[v]+scale*fit.x[:3];records.append(dict(vertex=v,margin_normalized=float(fit.x[3]),iterations=int(fit.nit)))
    emit(check,'coupled_seed_endpoint',policy=policy,row_index=row_index,coordinates=seed,displacement=seed-p,records=records)
    try:
        new,completion=_descending(p,f,seed,original,radius,bounds,normal,centre,lower,upper,controls['maxiter'],tick,_trace=trace)
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
