"""Policy-2 physical L1 seed/completion LP, not admission or infeasibility proof."""
import numpy as np
from scipy.optimize import linprog
from tet_layer_row import RowConflict

TAU=1e-10

def box(current,original,radius,bounds):
    half=radius/np.sqrt(3);lo=np.asarray(original)-half;hi=np.asarray(original)+half
    if bounds is not None:
        lo=np.maximum(lo,np.nextafter(bounds[:,0],np.inf));hi=np.minimum(hi,np.nextafter(bounds[:,1],-np.inf))
    if not np.isfinite([lo,hi]).all() or np.any(lo>hi):raise RowConflict('CoupledMinimumBoxRefusal')
    spacing=float(np.max(np.abs(np.spacing(np.r_[current,lo,hi]))))
    reserve=max(64*spacing,16*TAU*radius)
    return lo,hi,reserve

def solve(current,lo,hi,radius,scale,M,b,maxiter,tick,*,reserve):
    """M u <= b in metres, z=u/scale, epigraph a>=abs(u)/radius.

    ALL physical plane rows are normalized by radius, never variable scale.
    tau therefore maps to tau*radius metres regardless of parameterization.
    """
    current=np.asarray(current);lo=np.asarray(lo);hi=np.asarray(hi);M=np.asarray(M);b=np.asarray(b)
    if (not np.isfinite(scale) or scale<=0 or not np.isfinite(radius) or radius<=0 or M.ndim!=2 or M.shape[1]!=3 or b.shape!=(len(M),)
        or not np.isfinite([current,lo,hi]).all() or not np.isfinite(M).all() or not np.isfinite(b).all()):raise RowConflict('CoupledMinimumInvalidModel')
    alpha=scale/radius;zlo=(lo-current)/scale;zhi=(hi-current)/scale
    if not np.isfinite(alpha) or alpha<=0 or not np.isfinite([zlo,zhi]).all() or np.any(zlo>zhi):raise RowConflict('CoupledMinimumUnrepresentableModel')
    A=np.vstack((np.c_[M*alpha,np.zeros((len(M),3))],np.c_[np.eye(3)*alpha,-np.eye(3)],np.c_[-np.eye(3)*alpha,-np.eye(3)]))
    rhs=np.r_[b/radius,np.zeros(6)]
    tick();fit=linprog([0.,0.,0.,1.,1.,1.],A_ub=A,b_ub=rhs,bounds=list(zip(zlo,zhi))+[(0.,None)]*3,
        method='highs',options=dict(maxiter=maxiter,time_limit=tick.remaining_seconds(),primal_feasibility_tolerance=TAU,dual_feasibility_tolerance=TAU))
    record=dict(current_m=current,coordinate_lower_m=lo,coordinate_upper_m=hi,radius_m=radius,variable_scale_m=scale,constraint_scale_m=radius,
        physical_M=M,physical_b_m=b,A_ub=A,b_ub=rhs,variable_lower=zlo,variable_upper=zhi,solver_status=int(fit.status),solver_message=str(fit.message),
        iterations=int(fit.nit),solution=fit.x,objective='sum(abs(displacement))/radius',primal_dual_tolerance=TAU,
        physical_plane_residual_budget_m=TAU*radius,requested_reserve_m=reserve)
    tick()
    if not fit.success or fit.x is None or not np.isfinite(fit.x).all():return None,record
    u=scale*fit.x[:3];new=current+u;actual=new-current;res=b-M@actual
    record.update(coordinates_m=new,displacement_m=actual,physical_constraint_residual_m=res,L1_m=float(np.abs(actual).sum()),L2_m=float(np.linalg.norm(actual)),
        coordinate_lower_margin_m=new-lo,coordinate_upper_margin_m=hi-new,epigraph_residual=rhs-A@fit.x)
    # Solver tolerances are NOT mesh tolerances. Reserve residuals may consume
    # tau*radius, but actual geometry must still pass all original strict gates.
    if np.any(new<lo) or np.any(new>hi) or np.any(res < -TAU*radius):
        record['numerical_endpoint_refused']=True;return None,record
    return new,record
