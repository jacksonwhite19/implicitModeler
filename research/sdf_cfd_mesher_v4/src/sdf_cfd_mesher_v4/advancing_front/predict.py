"""Uncoupled nominal prediction only. Does not commit or generate prisms."""
from dataclasses import dataclass
import time
import numpy as np

@dataclass
class Prediction:
    coordinates: np.ndarray
    initial_phi: np.ndarray
    target: np.ndarray
    final_phi: np.ndarray
    residual: np.ndarray
    iterations: np.ndarray
    gradient_norm: np.ndarray
    normals: np.ndarray
    displacement: np.ndarray
    error_bound: np.ndarray
    curvature: np.ndarray
    projection_seconds: float
    boundary_selections: np.ndarray

def predict_next_front(points,target,sdf,*,iteration_cap=64):
    """Newton correction using raw gradients, with source-defined termination.

    DISTANCE_TOLERANCE*(1+section_position.length()) is returned by the frozen
    native query. Residual <= that source error bound ends numerical projection;
    it is not a physical quality gate. Exact binary64 stagnation outside that
    bound or the caller's operational iteration cap is a projection failure.
    """
    started=time.perf_counter();x0=np.asarray(points,dtype=float);x=x0.copy()
    target=np.broadcast_to(np.asarray(target,dtype=float),(len(x),)).copy()
    initial=sdf.evaluate(x);phi=initial.phi.copy();grad=initial.gradient.copy()
    normal=initial.normal.copy();norm=initial.gradient_norm.copy();error=initial.source_error_bound.copy()
    curvature=initial.section_curvature.copy();selection=initial.boundary_normal_selection.copy()
    counts=np.zeros(len(x),dtype=np.int64)
    active=np.flatnonzero(abs(phi-target)>error)
    while len(active):
        if np.any(counts[active]>=iteration_cap):raise RuntimeError('ProjectionIterationCap')
        proposed=x[active]-(phi[active]-target[active])[:,None]*grad[active]/norm[active,None]**2
        if np.any(np.all(proposed==x[active],axis=1)):raise RuntimeError('ProjectionBinary64Stagnation')
        x[active]=proposed;counts[active]+=1;q=sdf.evaluate(proposed)
        phi[active]=q.phi;grad[active]=q.gradient;normal[active]=q.normal;norm[active]=q.gradient_norm;error[active]=q.source_error_bound;curvature[active]=q.section_curvature;selection[active]=q.boundary_normal_selection
        active=active[abs(phi[active]-target[active])>error[active]]
    if not np.isfinite(x).all():raise RuntimeError('NonfiniteCoordinate')
    return Prediction(x,initial.phi,target,phi,phi-target,counts,norm,normal,x-x0,error,curvature,time.perf_counter()-started,selection)
