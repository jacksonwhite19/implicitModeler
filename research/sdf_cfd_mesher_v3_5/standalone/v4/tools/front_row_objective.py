"""Private shared-row depth proposal. Existing V4 owners alone admit a mesh.

Optimize metric strain with a positive-volume barrier across the complete new
front. Edge matching alone can prefer a collapsed cell; the barrier prevents
that objective pathology. Bounds and this objective do not prove acceptance.
"""
from itertools import combinations
import time
import numpy as np
from tet_layer_row import product, RowConflict


class RadialObjective:
    def __init__(self, front, faces, direction, metric, *, objective='metric'):
        if objective not in ('metric','physical_shape'):raise ValueError('UnknownPrivateRowObjective')
        self.objective=objective
        self.front = np.asarray(front, dtype=float)
        self.direction = np.asarray(direction, dtype=float)
        n = len(self.front)
        metric = np.asarray(metric, dtype=float)
        if (self.front.shape != (n, 3) or self.direction.shape != (n, 3)
                or metric.shape != (2*n, 3, 3) or not np.isfinite(self.front).all()
                or not np.isfinite(self.direction).all() or not np.isfinite(metric).all()
                or not np.allclose(metric, metric.transpose(0, 2, 1), rtol=0, atol=0)
                or np.any(np.linalg.eigvalsh(metric) <= 0)):
            raise RowConflict('InvalidRowMetricObjective')
        t = product(faces, n)
        self.tets = t
        self.tensor = metric[t].mean(axis=1)
        if objective=='physical_shape':self.tensor=np.broadcast_to(np.eye(3),self.tensor.shape)
        self.log_metric_volume = .5*np.linalg.slogdet(self.tensor)[1]
        top_metric = metric[n:]
        self.scale = 1/np.sqrt(np.einsum('ni,nij,nj->n', self.direction, top_metric, self.direction))
        if not np.isfinite(self.scale).all() or np.any(self.scale <= 0):
            raise RowConflict('InvalidRowMetricDepthScale')

    def evaluate(self, normalized_depth):
        n = len(self.front)
        # Build local differences before adding displacements, so a small depth
        # does not disappear in world-coordinate subtraction inside the solve.
        ids=self.tets%n; top=self.tets>=n
        displacement=(normalized_depth*self.scale)[:,None]*self.direction
        p=self.front[ids]-self.front[ids[:,0],None,:]
        p=p+np.where(top[...,None],displacement[ids],0.)
        e1,e2,e3=(p[:,i]-p[:,0] for i in (1,2,3))
        cof=np.stack((np.cross(e2,e3),np.cross(e3,e1),np.cross(e1,e2)),axis=1)
        det=np.einsum('ni,ni->n',e1,cof[:,0])
        if not np.isfinite(det).all() or np.any(det<=0):
            raise RowConflict('RowMetricVolumeRepresentabilityConflict')
        centered=p-p.mean(axis=1)[:,None,:]
        mp=np.einsum('nij,nvj->nvi',self.tensor,centered)
        # For a unit-edge regular reference tetrahedron, sum(e^T M e) equals
        # 2||F||_F^2 and det(F)=det(J)*sqrt(det(M))/sqrt(1/2).
        # E = 2||F||^2 - 4 log(det F) has its minimum at singular values 1.
        # This is a dimensionless shape AND size target, not a mesh gate.
        strain=4*np.einsum('nvi,nvi->n',centered,mp)
        gd=cof/det[:,None,None]
        gradlog=np.concatenate((-gd.sum(axis=1)[:,None,:],gd),axis=1)
        if self.objective=='metric':
            energy=np.sum(strain-4*(np.log(det)+self.log_metric_volume-.5*np.log(.5)))
            gradient=8*mp-4*gradlog
        else:
            # Separate diagnostic: best physical shape in the same ray domain,
            # independent of requested metric size/shape. Never a schema-3
            # replacement or an assertion that size controls were satisfied.
            denominator=12*np.power(det*.5,2/3)
            per_cell=strain/denominator
            energy=np.sum(per_cell)
            gradient=8*mp/denominator[:,None,None]-(2/3)*per_cell[:,None,None]*gradlog
        result = np.zeros(n)
        local=ids[top]
        contribution=np.einsum('ni,ni->n',gradient[top],self.direction[local])*self.scale[local]
        result += np.bincount(local,weights=contribution,minlength=n)
        if not np.isfinite(energy) or not np.isfinite(result).all():
            raise RowConflict('RowMetricObjectiveRepresentabilityConflict')
        return float(energy), result


def propose(front, faces, direction, depth, lower, upper, metric, controls, check=None, *, objective='metric'):
    from scipy.optimize import minimize
    started = time.perf_counter()
    if check: check()
    mode=objective
    objective = RadialObjective(front, faces, direction, metric,objective=mode)
    scale = objective.scale
    lower = np.nextafter(np.asarray(lower, dtype=float), np.inf)
    upper = np.nextafter(np.asarray(upper, dtype=float), 0.)
    # A numerical representability floor, not a quality tolerance: enough to
    # change a coordinate of the 3D point. A tiny direction component must not
    # force every coordinate to change. Final endpoint checks remain mandatory.
    ulp=np.divide(np.abs(np.spacing(objective.front)),np.abs(objective.direction),
        out=np.full_like(objective.front,np.inf),where=objective.direction!=0).min(axis=1)
    lower=np.maximum(lower,np.nextafter(ulp,np.inf))
    if (lower.shape != scale.shape or upper.shape != scale.shape
            or not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or np.any(lower <= 0) or np.any(lower >= upper)):
        raise RowConflict('EmptyRowMetricDepthDomain')
    initial = np.log(np.clip(np.asarray(depth, dtype=float), lower, upper)/scale)
    lo, hi = np.log(lower/scale), np.log(upper/scale)
    calls = 0; rejected = 0
    def fun(x):
        nonlocal calls, rejected
        if check: check()
        calls += 1
        try:
            normalized=np.exp(x);value,gradient=objective.evaluate(normalized)
            return value,gradient*normalized
        except RowConflict as error:
            if error.reason!='RowMetricVolumeRepresentabilityConflict': raise
            # An undefined positive-volume barrier rejects a private optimizer
            # trial. This is not an accepted geometry or a swallowed gate error.
            rejected += 1
            return np.inf, np.zeros_like(x)
    before = objective.evaluate(np.exp(initial))[0]
    fit = minimize(fun, initial, jac=True, method='L-BFGS-B', bounds=list(zip(lo, hi)),
                   options=dict(maxiter=controls['maxiter'], ftol=controls['ftol']))
    if check: check()
    # A solver success flag has no authority. Retain only a finite, bounded,
    # improved objective endpoint; ordinary realized geometry checks follow.
    final = initial
    if np.isfinite(fit.x).all() and np.all(fit.x >= lo) and np.all(fit.x <= hi):
        if fun(fit.x)[0] < before: final = fit.x
    proposed = np.clip(np.exp(final)*scale, lower, upper)
    after = fun(np.log(proposed/scale))[0]
    return proposed, dict(method='SharedRadialDepth/MetricStrainVolumeBarrier/1' if mode=='metric' else 'PrivatePhysicalMeanRatioDiagnostic/1',
        objective=mode,requested_metric_objective_used=mode=='metric',parameterization='LogPositiveDepth',
        proposal_only=True, vertices=len(depth), tetrahedra=len(objective.tets),
        before_energy=before, after_energy=after, energy_scope='Metric optimizer only; before determinant cleanup and endpoint correction',
        objective_calls=calls,invalid_private_trials=rejected,
        optimizer_success=bool(fit.success), optimizer_status=int(fit.status),
        optimizer_message=str(fit.message), iterations=int(fit.nit),
        seconds=time.perf_counter()-started,
        full_quality_source_collision_coverage_filling_required=True)
