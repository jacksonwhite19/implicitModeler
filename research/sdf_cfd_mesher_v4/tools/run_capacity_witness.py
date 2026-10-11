"""Bounded fixture-authority audit. Stop before prohibited topology changes."""
import sys,json,time,hashlib,os
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import psutil
from capacity_fixtures import cohort,triangle,AnalyticSDF
from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.forecast.capacity import CapacityForecast,identity
from sdf_cfd_mesher_v4.forecast.capacity_witness import ownership_preflight,blocked_prefix
from sdf_cfd_mesher_v4.geometry.front import build_topology
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer,CELL_LIMITS
from sdf_cfd_mesher_v4.forecast.detailed.metrics import cell_metrics
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_signs
OUT=ROOT/'results/layer_capacity_witness'
def save(n,x):(OUT/n).write_text(json.dumps(x,indent=2,sort_keys=True,allow_nan=False)+'\n')
def run():
    started=time.perf_counter();s=load_schedule();cache=CapacityForecast();rows=[]
    for f in cohort(s.targets):
        low=triangle(f);high=triangle(f,float(s.targets[0]));top=build_topology(np.array([[0,1,2]]),3)
        projection='ProjectedByQualifiedSDFPredictor'
        try:
            with np.errstate(all='ignore'):high=predict_next_front(low,float(s.targets[0]),AnalyticSDF(f)).coordinates
        except (RuntimeError,ValueError) as exc:projection='Failed:'+str(exc)
        ownership=ownership_preflight(top);assert not ownership['complete']
        cap=cache.analyze(f['id'],s,0,source=f,geometry_identity=identity(f),topology_identity=top.sha256)
        row=blocked_prefix(f['id'],s,cap,ownership);row['projection']=projection
        try:evaluate_layer(top,low,high,1)
        except ValueError as e:
            assert str(e)=='IncompleteSurfaceOwnership';row['production_reproducer']=str(e)
        else:raise AssertionError('IncompleteOwnershipWasAccepted')
        # Partial cell checks are NOT a layer receipt, and cannot be committed.
        cells=cell_metrics(np.concatenate((low,high))[None],np.array([1]),conditioning=False)
        values={k:float(cells[k][0]) for k in CELL_LIMITS}
        failures=[dict(entity=0,metric=k,value=values[k]) for k,fn in CELL_LIMITS.items() if bool(fn(cells[k])[0])]
        row['partial_cell_diagnostics']=dict(values=values,failures=failures,exact=exact_signs(np.concatenate((low,high))),
              scope='Uncommitted first-layer diagnostic; analytic fallback if projection failed; incomplete face validation')
        row['unavailable_checks']=['complete interpolation weights','complete skewness','complete symmetric volume ratios','full ownership validation','complete current-vs-prefix certificate']
        rows.append(row)
    return dict(fixtures=rows,cache=dict(hits=cache.hits,misses=cache.misses),exact_collision_calls=0,
                general_sdf_study='Not started: controlled-front ownership blocker precedes generalization',
                steering='Not run: changing coordinates cannot supply absent neighboring columns'),dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset)
if __name__=='__main__':
    a,t=run();b,u=run();assert identity(a)==identity(b)
    save('FIXTURE_PREFLIGHT.json',a);save('TIMINGS.json',dict(first=t,replay=u));save('DETERMINISM.json',dict(equal=True,scientific_hash=identity(a)))
    print(json.dumps(dict(timing=t,partial_cell_failures={r['region_id']:r['partial_cell_diagnostics']['failures'] for r in a['fixtures']}),indent=2))
