"""Bounded feasibility solve for interior points of one compact cavity only."""
import sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from constrained_cavity import *
import psutil

def run(input_file,candidate,out,mode='feasible'):
    out.mkdir(exist_ok=False);start=time.perf_counter();d=dict(np.load(input_file));a=np.load(candidate);x,t=a['x'],a['t'];fixed=np.unique(d['f']);moving=np.setdiff1d(np.unique(t),fixed)
    if len(moving)>12:raise ValueError('LocalPointSolveBound')
    base=x[moving].copy();scale=np.max(np.ptp(d['x'],axis=0));q0,m0=quality(d,x,t);cache={}
    def metrics(z):
        if time.perf_counter()-start>60:raise TimeoutError('LocalCavityPointBudget')
        key=tuple(z)
        if key not in cache:
            xx=x.copy();xx[moving]=base+scale*np.asarray(z).reshape(-1,3)
            try:
                q,m=quality(d,xx,t)
                val=np.r_[m['volume']/np.maximum(abs(m0['volume']),1e-300),m['determinant']/.0011-1,1000/m['aspect']-1,m['weight']/.055-1,m['ratio']/.01-1,3.63636-m['skew'],m['interface_weight']/.055-1,m['interface_ratio']/.01-1,3.63636-m['interface_skew']]
                if not np.isfinite(val).all():val=np.full_like(val,-1e20)
            except (ValueError,FloatingPointError):val=np.full(len(next(iter(cache.values()))),-1e20)
            cache[key]=val
        return cache[key]
    zero=np.zeros(3*len(moving));before=metrics(zero);tick=time.perf_counter()
    if mode=='margin':
        # Seek an interior feasible point instead of accepting a numerically
        # marginal optimizer success. Publication still uses the exact gates.
        fit=minimize(lambda z:-float(z[-1]),np.r_[zero,min(0,float(before.min()))],method='SLSQP',bounds=[(-1,1)]*len(zero)+[(None,1)],constraints=[dict(type='ineq',fun=lambda z:metrics(z[:-1])-z[-1])],options=dict(maxiter=150,ftol=1e-12));z=fit.x[:-1]
    else:
        fit=minimize(lambda z:float(z@z),zero,method='SLSQP',bounds=[(-1,1)]*len(zero),constraints=[dict(type='ineq',fun=metrics)],options=dict(maxiter=150,ftol=1e-12));z=fit.x
    xx=x.copy();xx[moving]=base+scale*z.reshape(-1,3);iv=invariants(xx,d['t'],t);q,m=quality(d,xx,t)
    result=dict(mode=mode,before=q0,after=q,invariants=iv,moving_vertices=moving,variables=len(fit.x),solver_success=bool(fit.success),message=fit.message,iterations=fit.nit,evaluations=len(cache),seconds=time.perf_counter()-start,peak_rss_bytes=psutil.Process().memory_info().peak_wset)
    np.savez_compressed(out/'CANDIDATE.npz',x=xx,t=t);save(out/'RESULT.json',result);print(result,flush=True)

if __name__=='__main__':run(*map(Path,sys.argv[1:4]),sys.argv[4] if len(sys.argv)>4 else 'feasible')
