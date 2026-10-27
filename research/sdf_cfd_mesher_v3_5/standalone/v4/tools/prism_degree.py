"""Exact integer evaluation of the qualified rational degree test.

Binary64 boundary coordinates share a power-of-two denominator. Each witness
has its own exact common denominator. Cross multiplication replaces Fraction
normalization; inclusive projected bounds, ray order, and ambiguity stay exact.
"""
from fractions import Fraction
from math import lcm
import time
import numpy as np
import psutil
from optimized_projected_index import ProjectedBoundaryIndex

PERFORMANCE={}


def serial(points,faces,samples,*,stats=None):
    ratios=[[float(v).as_integer_ratio() for v in row] for row in points]
    denominator=max(d for row in ratios for _,d in row)
    coordinates=[tuple(n*(denominator//d) for n,d in row) for row in ratios]
    del ratios
    witnesses=[]
    for row in samples:
        row=tuple(Fraction(v) for v in row)
        q=lcm(*(v.denominator for v in row))
        witnesses.append((tuple(v.numerator*(q//v.denominator) for v in row),q))
    pending=list(range(len(witnesses)));out=[None]*len(witnesses)
    for k in range(1,9):
        dy=k;dz=k*k+1
        projected=[(p[1]-dy*p[0],p[2]-dz*p[0]) for p in coordinates]
        tri=[[projected[v] for v in f] for f in faces]
        lo=np.nextafter(np.array([[float(Fraction(min(p[j] for p in row),denominator)) for j in range(2)] for row in tri]),-np.inf)
        hi=np.nextafter(np.array([[float(Fraction(max(p[j] for p in row),denominator)) for j in range(2)] for row in tri]),np.inf)
        candidate_index=ProjectedBoundaryIndex(lo,hi)
        descriptors=[]
        for face,(a,b,c) in zip(faces,tri):
            B=(b[0]-a[0],b[1]-a[1]);C=(c[0]-a[0],c[1]-a[1])
            det=B[0]*C[1]-B[1]*C[0]
            x0,x1,x2=(coordinates[v][0] for v in face)
            descriptors.append((a,B,C,det,x0,x1-x0,x2-x0))
        retry=[]
        for index in pending:
            p,q=witnesses[index];uv=(p[1]-dy*p[0],p[2]-dz*p[0])
            v=np.array([float(Fraction(z,q)) for z in uv])
            ids=candidate_index.containing(v);count=0;ambiguous=False
            if stats is not None:stats['candidate_pairs']=stats.get('candidate_pairs',0)+len(ids)
            for i in ids:
                if stats is not None:stats['exact_contribution_evaluations']=stats.get('exact_contribution_evaluations',0)+1
                a,B,C,det,x0,dx1,dx2=descriptors[i]
                if det==0:ambiguous=True;break
                W=(uv[0]*denominator-a[0]*q,uv[1]*denominator-a[1]*q)
                u=W[0]*C[1]-W[1]*C[0];w=B[0]*W[1]-B[1]*W[0];d=q*det
                if d<0:u=-u;w=-w;d=-d
                remaining=d-u-w
                if min(u,w,remaining)<0:continue
                relation=(x0*d+u*dx1+w*dx2)*q-p[0]*denominator*d
                if relation<0:continue
                if relation==0 or min(u,w,remaining)==0:ambiguous=True;break
                count+=1 if det>0 else -1
            if ambiguous:retry.append(index)
            else:out[index]=dict(degree=count,ray=[1,dy,dz])
        pending=retry
        if not pending:break
    return dict(values=out,unresolved=pending,passed=not pending and all(x['degree']==1 for x in out))


def _part(arguments):
    stats={};wall=time.perf_counter();cpu=time.process_time()
    result=serial(*arguments,stats=stats)
    return result,stats,dict(wall=time.perf_counter()-wall,cpu=time.process_time()-cpu,
                           rss=psutil.Process().memory_info().rss)


def degree(points,faces,samples,*,stats=None,workers=4):
    from concurrent.futures import ProcessPoolExecutor
    from multiprocessing import get_context
    if type(workers) is not int or not 1<=workers<=24:raise ValueError('InvalidDegreeWorkers')
    samples=list(samples)
    if not samples or workers==1:return serial(points,faces,samples,stats=stats)
    workers=min(workers,len(samples));size=(len(samples)+workers-1)//workers
    from optimized_mixed_certificate import Native
    Native()
    batches=[samples[i:i+size] for i in range(0,len(samples),size)]
    values=[];unresolved=[];offset=0;measurements=[];started=time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers,mp_context=get_context('spawn')) as pool:
        for result,counts,measurement in pool.map(_part,((points,faces,b) for b in batches)):
            measurements.append(measurement);values.extend(result['values'])
            unresolved.extend(offset+i for i in result['unresolved']);offset+=len(result['values'])
            if stats is not None:
                for key,value in counts.items():stats[key]=stats.get(key,0)+value
    PERFORMANCE.clear();PERFORMANCE.update(workers=workers,seconds=time.perf_counter()-started,
        worker_measurements=measurements,worker_cpu=sum(m['cpu'] for m in measurements),
        sum_retained_worker_rss=sum(m['rss'] for m in measurements))
    # Keep the normal measurement interface used by the previous qualified path.
    from optimized_global_degree import PERFORMANCE as previous
    previous.clear();previous.update(PERFORMANCE)
    return dict(values=values,unresolved=unresolved,passed=not unresolved and all(x['degree']==1 for x in values))
