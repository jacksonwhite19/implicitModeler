"""Bounded three-coordinate vertex-star feasibility; never a whole-core solve."""
import sys,time,json
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.optimize import minimize
from fixed_core_study import *
import psutil

def repair(src,interface,dest):
    start=time.perf_counter();dest.mkdir(parents=True,exist_ok=False)
    x,t,ci,iface=load_core(src,interface);original=x.copy();summary,m=evaluate(x,t,ci,iface)
    fixed=set(map(int,ci));v,d,a,c=cell_metrics(x,t,METRIC);tc,tv=centres(x[t],FG);faces,owners=m['faces'],m['owners'];io=m['interface_owner']
    candidates=np.setdiff1d(np.unique(t[m['bad']]),ci);trials=[];last=start
    for vertex in candidates:
        if time.perf_counter()-start>105:break
        cells=np.flatnonzero(np.any(t==vertex,axis=1));gf=np.flatnonzero(np.any(np.isin(owners,cells),axis=1));gi=np.flatnonzero(np.isin(io,cells))
        neighbors=np.setdiff1d(np.unique(t[cells]),[vertex]);scale=float(np.linalg.norm(x[neighbors]-x[vertex],axis=1).min());base=x[vertex].copy();cache={}
        def values(z):
            key=tuple(z)
            if key in cache:return cache[key]
            x[vertex]=base+scale*z
            vv,dd,aa,cc=cell_metrics(x,t[cells],METRIC);v[cells]=vv;c[cells]=cc;tc[cells],tv[cells]=centres(x[t[cells]],FG)
            ww,rr,ss,_=face_metrics(x,faces[gf],owners[gf],v,c,FG)
            if len(gi):
                area=iface.area[gi];fc=iface.fc[gi];oc=io[gi]
                da=abs(np.einsum('ij,ij->i',area,fc-iface.c[gi]));db=abs(np.einsum('ij,ij->i',area,tc[oc]-fc))
                iw=np.minimum(da,db)/(da+db);ir=np.minimum(iface.v[gi],tv[oc])/np.maximum(iface.v[gi],tv[oc]);sk=FG.internal_face_skewness(x[ci[iface.f[gi]]],fc,area,iface.c[gi],tc[oc])
            else:iw=ir=sk=np.empty(0)
            val=np.r_[vv/np.maximum(abs(m['volume'][cells]),1e-300),dd/.0011-1,1000/aa-1,ww/.055-1,rr/.01-1,3.63636-ss,iw/.055-1,ir/.01-1,3.63636-sk]
            x[vertex]=base
            if not np.isfinite(val).all():val=np.full_like(val,-1e30)
            cache[key]=val;return val
        before=values(np.zeros(3));tick=time.perf_counter()
        if np.all(before>=0):continue
        fit=minimize(lambda z:float(z@z),np.zeros(3),method='SLSQP',bounds=[(-1,1)]*3,constraints=[dict(type='ineq',fun=values)],options=dict(maxiter=30,ftol=1e-10))
        after=values(fit.x)
        # Search-state improvement only; full publication still requires all gates.
        accepted=bool(np.all(after[before>=0]>=0) and ((after<0).sum()<(before<0).sum() or ((after<0).sum()==(before<0).sum() and after.min()>before.min())))
        x[vertex]=base+scale*fit.x if accepted else base
        v[cells],d[cells],a[cells],c[cells]=cell_metrics(x,t[cells],METRIC);tc[cells],tv[cells]=centres(x[t[cells]],FG)
        trials.append(dict(vertex=int(vertex),cells=len(cells),constraint_failures_before=int((before<0).sum()),constraint_failures_after=int((after<0).sum()),minimum_before=float(before.min()),minimum_after=float(after.min()),accepted=accepted,seconds=time.perf_counter()-tick,evaluations=len(cache),solver_success=bool(fit.success)))
        del cache
        if time.perf_counter()-last>10:
            print(json.dumps(dict(stage='local_vertex_stars',attempts=len(trials),accepted=sum(z['accepted'] for z in trials),elapsed=time.perf_counter()-start,rss=psutil.Process().memory_info().rss)),flush=True);last=time.perf_counter();save(dest/'PROGRESS.json',trials)
    q,mm=evaluate(x,t,ci,iface)
    for p in src.glob('*.npy'):
        if p.name not in ('coordinates.npy',):np.save(dest/p.name,np.load(p))
    np.save(dest/'coordinates.npy',x);np.savez_compressed(dest/'METRICS.npz',**mm);save(dest/'QUALITY.json',q)
    info=psutil.Process().memory_info();result=dict(quality=q,seconds=time.perf_counter()-start,attempts=trials,moved_vertices=int(np.any(x!=original,axis=1).sum()),inserted_vertices=0,connectivity_exact=True,peak_rss_bytes=getattr(info,'peak_wset',info.rss),retained_rss_bytes=info.rss)
    save(dest/'TRIAL.json',result);print(json.dumps({k:z for k,z in result.items() if k!='attempts'}),flush=True)
if __name__=='__main__':repair(*map(Path,sys.argv[1:4]))
