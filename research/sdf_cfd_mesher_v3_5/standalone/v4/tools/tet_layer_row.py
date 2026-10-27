"""Private generic swept-row proposer. Existing V4 owners must admit its output.

No saved authority, mesh publication, gate changes, or shape dispatch. Floating
polynomial/LP arithmetic forecasts placement; it is not an exact sign proof.
"""
import time
import numpy as np
from transition_construction_evidence import emit,intervals


class RowConflict(ValueError):
    """Conflict of this bounded proposal domain; never global infeasibility."""
    def __init__(self, reason, **details):
        self.reason=reason; self.details=details
        super().__init__(reason, details)


def triple(a,b,c):
    return np.einsum('...i,...i->...',a,np.cross(b,c))


def product(faces,n):
    f=np.asarray(faces,dtype=np.int64); ordered=np.sort(f,axis=1);a,b,c=ordered.T
    out=np.empty((3*len(f),4),dtype=np.int64)
    out[0::3]=np.column_stack((a,b,c,c+n))
    out[1::3]=np.column_stack((a,b,b+n,c+n))
    out[2::3]=np.column_stack((a,a+n,b+n,c+n))
    inv=(f[:,0]>f[:,1]).astype(int)+(f[:,0]>f[:,2])+(f[:,1]>f[:,2])
    flip=np.repeat(inv%2==1,3);flip[1::3]=~flip[1::3]
    out[flip,:2]=out[flip,:2][:,::-1]
    return out


def signed_six(front,new,faces):
    pts=np.concatenate((front,new));tet=pts[product(faces,len(front))]
    return triple(tet[:,1]-tet[:,0],tet[:,2]-tet[:,0],tet[:,3]-tet[:,0]).reshape(-1,3)


def coefficients(front,faces,w):
    f=np.asarray(faces);ordered=np.sort(f,axis=1); a,b,c=ordered.T
    e1=front[b]-front[a];e2=front[c]-front[a];wa,wb,wc=w[a],w[b],w[c]
    sign=np.where(((f[:,0]>f[:,1]).astype(int)+(f[:,0]>f[:,2])+(f[:,1]>f[:,2]))%2,-1.,1.)
    z=np.zeros(len(f));out=np.stack((np.stack((triple(e1,e2,wc),z,z),axis=1),
        np.stack((triple(e1,e2,wb),-triple(e1,wb,wc),z),axis=1),
        np.stack((triple(wa,e1,e2),triple(wa,wb,e2)+triple(wa,e1,wc),triple(wa,wb,wc)),axis=1)),axis=1)
    return out*sign[:,None,None]


def first_positive_root(coeff):
    """First strictly positive root of c0+c1*a+c2*a*a, vectorized."""
    c=np.asarray(coeff).reshape(-1,3);a,b,d=c.T
    result=np.full(len(c),np.inf)
    linear=(d==0)&(b<0);result[linear]=-a[linear]/b[linear]
    disc=b*b-4*d*a;quad=(d!=0)&(disc>=0)
    indices=np.flatnonzero(quad)
    if len(indices):
        sq=np.sqrt(disc[indices]);q=-.5*(b[indices]+np.copysign(sq,b[indices]))
        with np.errstate(divide='ignore',invalid='ignore'):
            r1=q/d[indices];r2=a[indices]/q
        r1=np.where(r1>0,r1,np.inf);r2=np.where(r2>0,r2,np.inf)
        result[indices]=np.minimum(r1,r2)
    return result.reshape(np.asarray(coeff).shape[:-1])


def _directions(faces,normals,area,n,maxiter,tick):
    direction=np.zeros((n,3));np.add.at(direction,faces.ravel(),np.repeat(normals*area[:,None],3,axis=0))
    norm=np.linalg.norm(direction,axis=1)
    if np.any(norm==0):raise RowConflict('UnresolvedSharedDirection')
    direction/=norm[:,None]
    dots=np.einsum('fvi,fi->fv',direction[faces],normals)
    bad=np.unique(faces[dots<=0]);changed=[]
    if len(bad):
        from scipy.optimize import linprog
        incidence=[[] for _ in range(n)]
        for j,face in enumerate(faces):
            for v in face:incidence[v].append(j)
        for v in bad:
            tick();ns=normals[incidence[v]]
            fit=linprog([0.,0.,0.,-1.],A_ub=np.c_[-ns,np.ones(len(ns))],b_ub=np.zeros(len(ns)),
                bounds=[(-1.,1.)]*3+[(None,None)],method='highs',options={'maxiter':maxiter,'time_limit':tick.remaining_seconds()})
            tick()
            if not fit.success or not np.isfinite(fit.x).all() or fit.x[3]<=0:
                raise RowConflict('IncidentDirectionConeConflict',vertex=int(v),status=int(fit.status))
            d=fit.x[:3];d/=np.linalg.norm(d)
            if np.any(ns@d<=0):raise RowConflict('DirectionRepresentabilityConflict',vertex=int(v))
            direction[v]=d;changed.append(int(v))
    if np.any(np.einsum('fvi,fi->fv',direction[faces],normals)<=0):raise RowConflict('IncompleteIncidentDirectionCone')
    return direction,changed


def _line_domain(front,original,w,radius,bounds):
    delta=front-original;a=np.einsum('vi,vi->v',w,w);b=2*np.einsum('vi,vi->v',delta,w);c=np.einsum('vi,vi->v',delta,delta)-radius*radius
    if np.any(c>0):raise RowConflict('PreviousFrontOutsideCumulativeDistance')
    upper=(-b+np.sqrt(np.maximum(0,b*b-4*a*c)))/(2*a)
    if bounds is not None:
        for axis in range(3):
            d=w[:,axis];nz=d!=0
            upper[nz]=np.minimum(upper[nz],(np.where(d[nz]>0,bounds[axis,1],bounds[axis,0])-front[nz,axis])/d[nz])
    return float(np.min(upper))


def affine_gradients(p,location):
    """Original affine volume gradient, shared without changing role arithmetic."""
    gradient=np.zeros((len(p),3))
    for k in range(4):
        mask=location==k
        if not mask.any():continue
        pp=p[mask];e1=pp[:,1]-pp[:,0];e2=pp[:,2]-pp[:,0];e3=pp[:,3]-pp[:,0]
        g=[-np.cross(e2,e3)-np.cross(e3,e1)-np.cross(e1,e2),np.cross(e2,e3),np.cross(e3,e1),np.cross(e1,e2)][k]
        gradient[mask]=g
    return gradient


def _descending(front,faces,goal,original,radius,bounds,normal,centre,lower,upper,maxiter,tick,*,_trace=None,_minimum_solver=None):
    """Affine shared-vertex constructor. Greedy conflicts are typed, not global."""
    from scipy.optimize import linprog
    n=len(front);pts=np.concatenate((front,goal));new=pts[n:];tet=product(faces,n);pivot=np.min(np.where(tet>=n,tet-n,n),axis=1)
    groups=[[] for _ in range(n)]
    for j,v in enumerate(pivot):groups[v].append(j)
    apex=np.max(faces,axis=1);capgroups=[[] for _ in range(n)]
    for j,v in enumerate(apex):capgroups[v].append(j)
    incidence=[[] for _ in range(n)]
    for j,face in enumerate(faces):
        for v in face:incidence[v].append(j)
    records=[]
    for v in range(n-1,-1,-1):
        tick();indices=groups[v];cells=tet[indices];p=pts[cells];location=np.argmax(cells==v+n,axis=1)
        value=triple(p[:,1]-p[:,0],p[:,2]-p[:,0],p[:,3]-p[:,0])
        caprows=capgroups[v];ns=normal[incidence[v]]
        heights=np.asarray([normal[j]@(new[v]-centre[j]) for j in caprows])
        good=np.all(value>0) and np.all(ns@(new[v]-front[v])>0) and np.linalg.norm(new[v]-original[v])<=radius
        if caprows:good=good and np.all(heights>=lower[caprows]) and np.all(heights<=upper[caprows])
        if bounds is not None:good=good and np.all(new[v]>bounds[:,0]) and np.all(new[v]<bounds[:,1])
        if good and _minimum_solver is not None:
            good=_minimum_solver.valid(v,new[v],value,affine_gradients(p,location),ns,heights,caprows)
        if good:
            records.append({'vertex':v,'completed_tets':len(indices),'used_lp':False});continue
        # Affine volume gradient for each newly completed oriented tet.
        gradient=affine_gradients(p,location)
        size=np.linalg.norm(gradient,axis=1)
        if np.any(size==0):raise RowConflict('DescendingDegeneratePlane',vertex=v)
        # Scale variables by available explicit radius, constraints by plane norm.
        g=gradient/size[:,None];constant=(value-gradient@new[v])/size
        rows=np.c_[-g*radius,np.ones(len(g))];rhs=g@goal[v]+constant
        # Shared outward movement on every incident current-front plane.
        # This is a restricted proposer domain, not a universal feasibility test.
        rows=np.vstack((rows,np.c_[-ns*radius,np.ones(len(ns))]));rhs=np.r_[rhs,ns@(goal[v]-front[v])]
        for j in capgroups[v]:
            h=float(normal[j]@(goal[v]-centre[j]));r=normal[j]*radius
            rows=np.vstack((rows,np.r_[-r,0.],np.r_[r,0.]));rhs=np.r_[rhs,h-lower[j],upper[j]-h]
        # Conservative cube lies within requested sphere; it is a proposer domain.
        half=radius/np.sqrt(3);lo=(original[v]-half-goal[v])/radius;hi=(original[v]+half-goal[v])/radius
        if bounds is not None:lo=np.maximum(lo,(bounds[:,0]-goal[v])/radius);hi=np.minimum(hi,(bounds[:,1]-goal[v])/radius)
        if np.any(lo>hi):raise RowConflict('DescendingBoundedDomainConflict',vertex=v)
        if _minimum_solver is not None:
            placed,record=_minimum_solver(v,goal[v],radius,rows,rhs,indices,cells,incidence[v],caprows,new)
            new[v]=placed;records.append(dict(vertex=v,completed_tets=len(indices),used_lp=True,objective='PhysicalL1FromCurrentFront',solver=record));continue
        fit=linprog([0.,0.,0.,-1.],A_ub=rows,b_ub=rhs,bounds=list(zip(lo,hi))+[(None,None)],method='highs',options={'maxiter':maxiter,'time_limit':tick.remaining_seconds()})
        if _trace is not None:_trace('descending_lp',dict(vertex=v,completed_tet_indices=indices,completed_tets=cells,incident_faces=incidence[v],apex_faces=caprows,
            variable_origin=goal[v],variable_scale_m=radius,A_ub=rows,b_ub=rhs,lower=lo,upper=hi,
            status=int(fit.status),message=str(fit.message),nit=int(fit.nit),solution=fit.x,partial_front=new.copy() if not fit.success else None))
        tick()
        if not fit.success or not np.isfinite(fit.x).all() or fit.x[3]<=0:
            raise RowConflict('DescendingAffinePlacementConflict',vertex=v,status=int(fit.status),domain='Greedy cube/previous shared assignments; not global infeasibility')
        new[v]=goal[v]+radius*fit.x[:3];records.append({'vertex':v,'completed_tets':len(indices),'used_lp':True})
    signs=signed_six(front,new,faces)
    if np.any(signs<=0):raise RowConflict('DescendingEndpointNonpositive',count=int(np.sum(signs<=0)))
    return new,records


def propose_row(front,faces,owner_centres,owner_volumes,controls,*,original=None,row_index=0,bounds=None,check=None,fallback=True):
    for name in ('maximum_distance_m','seconds'):
        if type(controls.get(name)) not in (int,float) or not np.isfinite(controls[name]) or controls[name]<=0:raise RowConflict('InvalidRowControl',name=name)
    for name in ('rows','maxiter'):
        if type(controls.get(name)) is not int or controls[name]<=0:raise RowConflict('InvalidRowControl',name=name)
    started=time.perf_counter();deadline=started+controls['seconds']
    def tick():
        if check is not None:check()
        if time.perf_counter()>=deadline:raise RowConflict('RowPlanningTimeLimit')
    tick.remaining_seconds=lambda:min(max(0.,deadline-time.perf_counter()),check.remaining_seconds() if check is not None and hasattr(check,'remaining_seconds') else controls['seconds'])
    tick();front=np.asarray(front,dtype=float);faces=np.asarray(faces,dtype=np.int64);owners=np.asarray(owner_centres,dtype=float);volume=np.asarray(owner_volumes,dtype=float)
    n=len(front);original=front if original is None else np.asarray(original,dtype=float)
    if front.shape!=(n,3) or original.shape!=front.shape or faces.ndim!=2 or faces.shape[1:]!=(3,) or owners.shape!=(len(faces),3) or volume.shape!=(len(faces),):raise RowConflict('InvalidRowGeometry')
    if not np.isfinite(front).all() or not np.isfinite(original).all() or not np.isfinite(owners).all() or not np.isfinite(volume).all() or np.any(volume<=0) or faces.min()<0 or faces.max()>=n:raise RowConflict('InvalidRowValues')
    if type(row_index) is not int or not 0<=row_index<controls['rows']:raise RowConflict('InvalidRowIndex')
    if bounds is not None:
        bounds=np.asarray(bounds,dtype=float)
        if bounds.shape!=(3,2) or not np.isfinite(bounds).all() or np.any(bounds[:,0]>=bounds[:,1]):raise RowConflict('InvalidRowBounds')
        if np.any(front<=bounds[:,0]) or np.any(front>=bounds[:,1]):raise RowConflict('CurrentFrontOutsideDomain')
    triangle=front[faces];s=.5*np.cross(triangle[:,1]-triangle[:,0],triangle[:,2]-triangle[:,0]);area=np.linalg.norm(s,axis=1)
    if np.any(area<=0):raise RowConflict('DegenerateCurrentFront')
    normal=s/area[:,None];centre=triangle.mean(axis=1);dp=np.einsum('fi,fi->f',normal,centre-owners)
    if np.any(dp<=0):raise RowConflict('InvalidCurrentOwnerGeometry')
    lower=np.maximum(4*.055/(1-.055)*dp,.03*volume/area);upper=np.minimum(4*(1-.055)/.055*dp,300*volume/area)
    if np.any(lower>upper):raise RowConflict('EmptyActualOwnerInterval')
    direction,changed=_directions(faces,normal,area,n,controls['maxiter'],tick);dots=np.einsum('fvi,fi->fv',direction[faces],normal)
    apex=faces.max(axis=1);projection=np.einsum('fi,fi->f',direction[apex],normal)
    wanted=np.zeros(n);count=np.zeros(n);np.add.at(wanted,faces.ravel(),(4*dp[:,None]/dots).ravel());np.add.at(count,faces.ravel(),1)
    if np.any(count==0):raise RowConflict('UnusedFrontVertex')
    depth=wanted/count;lo=np.zeros(n);hi=np.full(n,np.inf)
    np.maximum.at(lo,apex,lower/projection);np.minimum.at(hi,apex,upper/projection)
    ref=intervals(check,front=front,faces=faces,owners=owners,volume=volume,area=area,normal=normal,centre=centre,dp=dp,
        lower=lower,upper=upper,direction=direction,dots=dots,apex=apex,projection=projection,lo=lo,hi=hi,changed=changed,row_index=row_index,controls=controls)
    if np.any(lo>hi):raise RowConflict('SharedActualOwnerIntervalConflict',vertices=np.flatnonzero(lo>hi).tolist(),diagnostic_identity=ref)
    depth=np.minimum(hi,np.maximum(lo,depth));w=depth[:,None]*direction;poly=coefficients(front,faces,w)
    if not np.isfinite(poly).all():raise RowConflict('UnrepresentableRowPolynomial')
    if np.any(poly[:,:,0]<=0):raise RowConflict('IncompletePositiveDerivative')
    roots=first_positive_root(poly);nofold=float(np.min(roots));radius=(row_index+1)/controls['rows']*controls['maximum_distance_m']
    alpha_min=float(np.max(lower/(projection*depth[apex])));alpha_max=min(float(np.min(upper/(projection*depth[apex]))),_line_domain(front,original,w,radius,bounds),nofold)
    alpha_max=float(np.nextafter(alpha_max,0)) if np.isfinite(alpha_max) else alpha_max
    alpha=min(1.,alpha_max);reason=None;records=[]
    if not alpha>=alpha_min:reason='OwnerIntervalVsPositiveDomainConflict'
    else:
        for attempt in range(controls['maxiter']):
            tick();new=front+alpha*w;signs=signed_six(front,new,faces)
            heights=np.einsum('fi,fi->f',normal,new[apex]-centre)
            good=np.all(signs>0) and np.all(heights>=lower) and np.all(heights<=upper) and np.all(np.linalg.norm(new-original,axis=1)<=radius)
            if bounds is not None:good=good and np.all(new>bounds[:,0]) and np.all(new<bounds[:,1])
            if good:break
            alpha=float(np.nextafter(alpha,0))
            if alpha<alpha_min:break
        else:good=False
        if not good:reason='RowEndpointRepresentabilityConflict'
    if reason is not None:
        if not fallback:raise RowConflict(reason,minimum_step=alpha_min,maximum_step=alpha_max,first_fold_step=nofold)
        new,records=_descending(front,faces,front+alpha_min*w,original,radius,bounds,normal,centre,lower,upper,controls['maxiter'],tick)
        signs=signed_six(front,new,faces);heights=np.einsum('fi,fi->f',normal,new[apex]-centre)
        if np.any(heights<lower) or np.any(heights>upper) or np.any(np.linalg.norm(new-original,axis=1)>radius):raise RowConflict('DescendingOwnerOrDistanceEndpointFailure')
    if bounds is not None and (np.any(new<=bounds[:,0]) or np.any(new>=bounds[:,1])):raise RowConflict('RowEndpointDomainConflict')
    tick()
    emit(check,'actual_owner_row_success',row_index=row_index,front=new,direction=direction,minimum_signed_six=float(np.min(signs)),proposal_only_result=True)
    return {'front':new,'direction':direction,'displacement':new-front,'diagnostics':{'schema':'PrivateGenericRowProposal/1','proposal_only':True,'row_index':row_index,'minimum_step':alpha_min,'maximum_step':alpha_max,'first_fold_step':nofold if np.isfinite(nofold) else None,'selected_step':alpha if reason is None else None,'direction_cones_changed':changed,'fallback_reason':reason,'fallback_vertices':records,'floating_nonpositive_by_role':np.sum(signs<=0,axis=0).tolist(),'minimum_signed_six':float(np.min(signs)),'seconds':time.perf_counter()-started,'actual_owner_intervals_used':len(faces),'exact_positive_quality_source_collision_filling_acceptance_required':True}}
