"""Bounded local coordinate feasibility; unchanged hard predicates publish."""
import sys,time,json
from pathlib import Path
import numpy as np
import psutil
from scipy.optimize import minimize
from constrained_cavity import quality,invariants,interior_point_valid,save

def guard_residuals(m):
    # Monotone scaling is an optimization aid only, not a metric/gate change.
    def ratio(a,b):return np.log(np.maximum(np.asarray(a),1e-300)/b)
    return np.r_[m['volume']/np.maximum(abs(m['volume']),1e-300),ratio(m['determinant'],.0011),ratio(1000,m['aspect']),ratio(m['weight'],.055),ratio(m['ratio'],.01),3.63636-m['skew'],ratio(m['interface_weight'],.055),ratio(m['interface_ratio'],.01),3.63636-m['interface_skew']]

def run(inp,candidate,out,mode='margin',limit=90):
    out.mkdir(exist_ok=False);tick=time.perf_counter();d=dict(np.load(inp));a=np.load(candidate);x,t=a['x'].copy(),a['t'];fixed=np.unique(d['f']);moving=np.setdiff1d(np.unique(t),fixed)
    if len(moving)>64 or len(t)>512:raise ValueError('LocalSolveScopeBound')
    q0,m0=quality(d,x,t);scale=float(np.median(np.linalg.norm(d['x'][d['f']]-d['x'][np.roll(d['f'],1,axis=1)],axis=2)));base=x[moving].copy();cache=[None,None];evaluations=0;iterations=0;best=None
    def event(stage,**kw):
        row=dict(stage=stage,seconds=time.perf_counter()-tick,rss=psutil.Process().memory_info().rss,**kw)
        with (out/'EVENTS.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
    def metrics(z):
        nonlocal evaluations,best
        if time.perf_counter()-tick>limit:raise TimeoutError('LocalSolveTimeBound')
        if cache[0] is not None and np.array_equal(cache[0],z):return cache[1]
        xx=x.copy();xx[moving]=base+scale*np.asarray(z).reshape(-1,3)
        q,m=quality(d,xx,t);val=guard_residuals(m)
        # The signed volume constraint must have a useful derivative.
        val[:len(t)]=m['volume']/np.maximum(abs(m0['volume']),1e-300)
        if not np.isfinite(val).all():val=np.full_like(val,-1e20)
        evaluations+=1;cache[:]=[z.copy(),val]
        score=float(val.min())
        if best is None or score>best[0]:best=(score,z.copy(),q)
        return val
    zero=np.zeros(3*len(moving));before=metrics(zero)
    def callback(z):
        nonlocal iterations
        iterations+=1
        if iterations%5==0:event('iteration',iteration=iterations,evaluations=evaluations,best_scaled_constraint=best[0],best_failures=best[2]['failures'])
    event('start',moving_points=len(moving),tets=len(t),before=q0)
    status='completed';fit=None
    try:
        if not len(moving):raise ValueError('NoInteriorDegreesOfFreedom')
        if mode=='margin':
            fit=minimize(lambda z:-z[-1],np.r_[zero,min(0,float(before.min()))],method='SLSQP',bounds=[(-1,1)]*len(zero)+[(None,1)],constraints=[dict(type='ineq',fun=lambda z:metrics(z[:-1])-z[-1])],callback=callback,options=dict(maxiter=150,ftol=1e-12));z=fit.x[:-1]
        else:
            fit=minimize(lambda z:float(z@z),zero,method='SLSQP',bounds=[(-1,1)]*len(zero),constraints=[dict(type='ineq',fun=metrics)],callback=callback,options=dict(maxiter=150,ftol=1e-12));z=fit.x
    except (TimeoutError,ValueError) as e:status=str(e);z=best[1]
    # Do not lose a scientifically better candidate to the final solver iterate.
    if status!='completed':z=best[1]
    else:
        try:
            if float(metrics(z).min())<best[0]:z=best[1]
        except TimeoutError:status='LocalSolveTimeBound';z=best[1]
    xx=x.copy();xx[moving]=base+scale*z.reshape(-1,3);q,m=quality(d,xx,t);iv=invariants(xx,d['t'],t)
    inside={str(int(i)):interior_point_valid(xx[i],d['x'],d['t']) for i in moving}
    r=dict(status=status,mode=mode,moving_vertices=moving,variables=len(z),before=q0,after=q,invariants=iv,strict_inside=inside,accepted_proposal=q['passed'] and iv['passed'] and all(inside.values()),seconds=time.perf_counter()-tick,iterations=iterations,evaluations=evaluations,solver_success=bool(fit.success) if fit else False,message=str(fit.message) if fit else status,peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss)
    np.savez_compressed(out/'CANDIDATE.npz',x=xx,t=t);save(out/'RESULT.json',r);event('end',accepted_proposal=r['accepted_proposal'],failures=q['failures'],minimum=q['minimum']);return r

if __name__=='__main__':run(*map(Path,sys.argv[1:4]),sys.argv[4] if len(sys.argv)>4 else 'margin')
