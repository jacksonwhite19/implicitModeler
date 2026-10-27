"""Parallel execution of the unchanged exact global degree obligation.

Every interior witness is still checked against the complete boundary. Results
are reduced in original witness order, including unresolved indices and rays.
"""
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import time
import psutil
from cavity_volume_audit import degree as serial_degree

PERFORMANCE={}


def _part(arguments):
    import cavity_volume_audit
    from optimized_projected_index import ProjectedBoundaryIndex
    cavity_volume_audit.ProjectedBoundaryIndex=ProjectedBoundaryIndex
    points,faces,samples=arguments
    stats={};wall=time.perf_counter();cpu=time.process_time()
    result=serial_degree(points,faces,samples,stats=stats)
    return result,stats,dict(wall=time.perf_counter()-wall,cpu=time.process_time()-cpu,
        rss=psutil.Process().memory_info().rss)


def degree(points,faces,samples,*,stats=None,workers=8):
    if type(workers) is not int or not 1<=workers<=24:raise ValueError('InvalidDegreeWorkers')
    samples=list(samples)
    if not samples or workers==1:return serial_degree(points,faces,samples,stats=stats)
    workers=min(workers,len(samples));size=(len(samples)+workers-1)//workers
    # Compile once before spawning readers of the immutable native library.
    from optimized_mixed_certificate import Native
    Native()
    batches=[samples[i:i+size] for i in range(0,len(samples),size)]
    values=[];unresolved=[];offset=0;measurements=[];started=time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers,mp_context=get_context('spawn')) as pool:
        for result,counts,measurement in pool.map(_part,((points,faces,b) for b in batches)):
            measurements.append(measurement)
            values.extend(result['values'])
            unresolved.extend(offset+i for i in result['unresolved'])
            offset+=len(result['values'])
            if stats is not None:
                for k,v in counts.items():stats[k]=stats.get(k,0)+v
    PERFORMANCE.clear();PERFORMANCE.update(workers=workers,seconds=time.perf_counter()-started,
        worker_measurements=measurements,worker_cpu=sum(m['cpu'] for m in measurements),
        sum_retained_worker_rss=sum(m['rss'] for m in measurements))
    return dict(values=values,unresolved=unresolved,passed=not unresolved and all(x['degree']==1 for x in values))
