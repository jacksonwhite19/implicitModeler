"""Private bounded radial depth proposer; unchanged existing owners admit.

Optimizing this floating score establishes no exact/source/neighbour acceptance.
Caller supplies a verified cap-kernel radial placement and its current actual
owner/distance/domain depth intervals. No source or fixture dispatch is used.
"""
import numpy as np
from tet_layer_row import product,RowConflict


def tensor_values(front,faces,new):
    """Final all-four-internal-face tensor, using Foundation face areas."""
    import local_termination_topology as t
    fg=t.q.legacy_modules()['component_quality'].FG
    front=np.asarray(front);n=len(front)
    indices=product(faces,n);source=indices%n
    # Gather only the requested star. Preserve the exact coordinates and
    # Foundation tensor arithmetic without copying both complete fronts.
    tet=np.where((indices<n)[...,None],front[source],new[source])
    patterns=((0,2,1),(0,1,3),(1,2,3),(2,0,3))
    areas=np.stack([fg.face_area_centre_tri(tet[:,p,:])[0] for p in patterns],axis=1)
    mean=np.linalg.norm(areas,axis=2).mean(axis=1)
    if np.any(mean<=0) or not np.isfinite(mean).all():raise RowConflict('RadialDepthTensorDomainConflict')
    scaled=areas/mean[:,None,None];value=np.linalg.det(scaled.transpose(0,2,1)@scaled)
    if not np.isfinite(value).all():raise RowConflict('RadialDepthTensorRepresentabilityConflict')
    return value


def score(front,faces,displacement,scale):
    value=tensor_values(front,faces,front+scale*displacement)
    return float(value.min()),int(np.sum(value<.0011))


def choose_scale(front,faces,direction,depth,lo,hi,controls,check=None):
    """Return one bounded shared multiplier; caller validates final endpoint.

Depth lo/hi are supplied by the current existing owner and cumulative spatial
domain, not guessed here. A radial kernel keeps all three roles positive for
each strictly positive increment; caller retains responsibility for that domain.
"""
    from scipy.optimize import minimize_scalar
    depth=np.asarray(depth);lo=np.asarray(lo);hi=np.asarray(hi);direction=np.asarray(direction)
    if depth.ndim!=1 or lo.shape!=depth.shape or hi.shape!=depth.shape or direction.shape!=(len(depth),3) or np.any(depth<=0) or not np.isfinite(depth).all() or not np.isfinite(direction).all():raise RowConflict('InvalidRadialDepthDomain')
    lower=float(np.max(lo/depth));upper=float(np.min(hi/depth))
    lower=float(np.nextafter(max(lower,0.),np.inf));upper=float(np.nextafter(upper,0.))
    if not np.isfinite([lower,upper]).all() or lower>upper or upper<=0:raise RowConflict('RadialDepthSharedScaleConflict',minimum_scale=lower,maximum_scale=upper)
    displacement=depth[:,None]*direction;records=[];cache={}
    def objective(scale):
        if check is not None:check()
        if scale not in cache:
            minimum,failures=score(front,faces,displacement,scale);cache[scale]=(minimum,failures);records.append(dict(scale=float(scale),minimum_determinant=minimum,determinant_failures=failures))
        return -cache[scale][0]
    candidates=[lower,upper,float(np.clip(1.,lower,upper))]
    for alpha in candidates:objective(alpha)
    if lower<upper:
        fit=minimize_scalar(objective,bounds=(lower,upper),method='bounded',options=dict(maxiter=controls['maxiter'],xatol=controls['ftol']))
        if np.isfinite(fit.x):candidates.append(float(fit.x));objective(float(fit.x))
    selected=max(candidates,key=lambda alpha:cache[alpha][0]);minimum,failures=cache[selected]
    return selected,dict(method='ActualFoundationTensor/BoundedSharedRadialDepth',proposal_only=True,minimum_scale=lower,maximum_scale=upper,selected_scale=selected,minimum_determinant=minimum,determinant_failures=failures,evaluations=records,complete_exact_quality_source_collision_filling_acceptance_required=True)


def choose_floor_blend(front,faces,direction,depth,lo,hi,controls,check=None):
    """Vary each depth above its own actual-owner floor, within supplied caps.

A single beta governs interpolation, but vertices whose owner floors are high
do not impose the same reduction bound on every unrelated vertex. This remains
a radial-kernel proposal; actual endpoint representability/admission is external.
"""
    from scipy.optimize import minimize_scalar
    depth=np.asarray(depth);lo=np.asarray(lo);hi=np.asarray(hi);direction=np.asarray(direction)
    if depth.ndim!=1 or lo.shape!=depth.shape or hi.shape!=depth.shape or direction.shape!=(len(depth),3) or np.any(depth<=0) or not np.isfinite(depth).all() or not np.isfinite(direction).all() or not np.isfinite(lo).all() or not np.isfinite(hi).all() or np.any(lo<0) or np.any(lo>hi) or np.any(depth<lo) or np.any(depth>hi):raise RowConflict('InvalidRadialFloorBlendDomain')
    floor=np.nextafter(lo,np.inf)
    if np.any(floor>hi):raise RowConflict('RadialFloorRepresentabilityConflict')
    target=np.maximum(depth,floor);difference=target-floor;cache={};records=[]
    def objective(beta):
        if check is not None:check()
        if beta not in cache:
            proposed=floor+beta*difference;minimum,failures=score(front,faces,proposed[:,None]*direction,1.)
            cache[beta]=(minimum,failures);records.append(dict(beta=float(beta),minimum_determinant=minimum,determinant_failures=failures))
        return -cache[beta][0]
    lower=float(np.nextafter(0.,1.));candidates=[lower,1.]
    for beta in candidates:objective(beta)
    fit=minimize_scalar(objective,bounds=(lower,1.),method='bounded',options=dict(maxiter=controls['maxiter'],xatol=controls['ftol']))
    if np.isfinite(fit.x):candidates.append(float(fit.x));objective(float(fit.x))
    selected=max(candidates,key=lambda beta:cache[beta][0]);minimum,failures=cache[selected];proposed=floor+selected*difference
    return proposed,dict(method='ActualFoundationTensor/BoundedPerVertexFloorBlend',proposal_only=True,selected_beta=selected,minimum_determinant=minimum,determinant_failures=failures,evaluations=records,complete_exact_quality_source_collision_filling_acceptance_required=True)


def _one_sweep(front,faces,direction,depth,lo,hi,controls,check=None):
    """One bounded scalar sweep over complete affected top-vertex tet stars.

Only the radial increment is movable. No old lower-front vertex changes. Every
tested star includes all tetrahedra incident to the movable outer vertex. A
proposal is adopted only without new floating determinant failures. Full current
neighbour and source/exact acceptance remain external and mandatory.
"""
    from scipy.optimize import minimize_scalar
    front=np.asarray(front);faces=np.asarray(faces);direction=np.asarray(direction);depth=np.asarray(depth).copy();lo=np.asarray(lo);hi=np.asarray(hi);n=len(front)
    if depth.shape!=(n,) or lo.shape!=depth.shape or hi.shape!=depth.shape or direction.shape!=front.shape or not np.isfinite(depth).all() or not np.isfinite(lo).all() or not np.isfinite(hi).all() or np.any(lo<0) or np.any(lo>hi) or np.any(depth<lo) or np.any(depth>hi):raise RowConflict('InvalidRadialSweepDomain')
    new=front+depth[:,None]*direction;tets=product(faces,n);current=tensor_values(front,faces,new);initial=int(np.sum(current<.0011));initial_minimum=float(current.min())
    failures=np.flatnonzero(current<.0011);bad_tets=tets[failures];top=bad_tets>=n;vertex_min=np.full(n,np.inf)
    np.minimum.at(vertex_min,bad_tets[top]-n,np.broadcast_to(current[failures,None],bad_tets.shape)[top])
    vertices=sorted(map(int,np.flatnonzero(np.isfinite(vertex_min))),key=lambda v:float(vertex_min[v]))
    incidence=[[] for _ in range(n)]
    for j,face in enumerate(faces):
        for v in face:incidence[v].append(j)
    records=[];evaluations=0
    for v in vertices:
        if check is not None:check()
        localfaces=np.asarray(incidence[v]);localtets=tets[np.ravel(3*localfaces[:,None]+np.arange(3))];mask=np.any(localtets==v+n,axis=1);localids=np.ravel(3*localfaces[:,None]+np.arange(3))[mask]
        before=current[localids].copy();bad=before<.0011
        if not bad.any():continue
        baseline=float(depth[v]);lower=float(np.nextafter(lo[v],np.inf));upper=float(np.nextafter(hi[v],0.))
        if lower>upper:records.append(dict(vertex=v,adopted=False,reason='OpenRadialIntervalEmpty'));continue
        cache={}
        def objective(value):
            nonlocal evaluations
            if check is not None:check()
            if value not in cache:
                new[v]=front[v]+value*direction[v];candidate=tensor_values(front,faces[localfaces],new)[mask];cache[value]=candidate;evaluations+=1
            return -float(cache[value].min())
        candidates=[baseline,lower,upper]
        for value in candidates:objective(value)
        if lower<upper:
            fit=minimize_scalar(objective,bounds=(lower,upper),method='bounded',options=dict(maxiter=controls['maxiter'],xatol=controls['ftol']))
            if np.isfinite(fit.x):objective(float(fit.x))
        allowed=[value for value,values in cache.items() if np.all(values[~bad]>=.0011)]
        selected=max(allowed,key=lambda value:(-int(np.sum(cache[value]<.0011)),float(cache[value].min())))
        after=cache[selected];adopted=bool(np.sum(after<.0011)<np.sum(bad) or after.min()>before.min())
        if adopted:depth[v]=selected;current[localids]=after
        new[v]=front[v]+depth[v]*direction[v]
        records.append(dict(vertex=v,adopted=adopted,affected_tets=len(localids),before_failures=int(np.sum(bad)),after_failures=int(np.sum(after<.0011)),before_minimum=float(before.min()),after_minimum=float(after.min()),selected_depth=float(depth[v]),lower_depth=lower,upper_depth=upper,evaluations=len(cache)))
    final=tensor_values(front,faces,new)
    return depth,dict(method='ActualFoundationTensor/CompleteAffectedRadialDepthSweep',proposal_only=True,initial_determinant_failures=initial,initial_minimum_determinant=initial_minimum,determinant_failures=int(np.sum(final<.0011)),minimum_determinant=float(final.min()),vertices_tested=len(records),adopted=sum(r.get('adopted',False) for r in records),objective_evaluations=evaluations,records=records,complete_neighbour_exact_quality_source_collision_filling_acceptance_required=True)


def improve_depths(front,faces,direction,depth,lo,hi,controls,check=None):
    """Repeat complete sweeps while failure count decreases, under hard bounds."""
    import time
    started=time.perf_counter();deadline=started+controls['seconds'];working=np.asarray(depth).copy();passes=[];evaluations=0;vertices=0;adopted=0
    def tick():
        if check is not None:check()
        if time.perf_counter()>=deadline:raise RowConflict('RadialDepthSweepTimeLimit')
    for iteration in range(controls['maxiter']):
        tick();working,record=_one_sweep(front,faces,direction,working,lo,hi,controls,tick);record['pass_index']=iteration;passes.append(record)
        evaluations+=record['objective_evaluations'];vertices+=record['vertices_tested'];adopted+=record['adopted']
        if record['determinant_failures']==0 or record['determinant_failures']>=record['initial_determinant_failures']:break
    tick();first=passes[0];last=passes[-1]
    return working,dict(method='ActualFoundationTensor/BoundedCompleteAffectedRadialDepthSweeps',proposal_only=True,initial_determinant_failures=first['initial_determinant_failures'],initial_minimum_determinant=first['initial_minimum_determinant'],determinant_failures=last['determinant_failures'],minimum_determinant=last['minimum_determinant'],vertices_tested=vertices,adopted=adopted,objective_evaluations=evaluations,passes=passes,seconds=time.perf_counter()-started,complete_neighbour_exact_quality_source_collision_filling_acceptance_required=True)
