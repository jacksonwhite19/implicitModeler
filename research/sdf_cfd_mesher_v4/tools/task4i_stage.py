"""Observational timers around existing kernels; return values are untouched."""
from task4i_common import *
import functools,runpy,psutil,traceback

def run():
    tool=sys.argv[1];args=sys.argv[2:];rows=[]
    from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
    from sdf_cfd_mesher_v4.forecast.detailed.exact_cache import LOCAL_EXACT
    from sdf_cfd_mesher_v4.forecast.detailed import rolling_spots
    from sdf_cfd_mesher_v4.advancing_front import staged_evaluation
    from sdf_cfd_mesher_v4.advancing_front import rolling_metrics
    if tool=='task4_certify_candidate.py':
        evaluate=rolling_metrics.evaluate_layer
        @functools.wraps(evaluate)
        def record_metrics(*a,**kw):
            result=evaluate(*a,**kw);layer=a[3]
            hashes={family:{k:array_hash(v) for k,v in result[family].items() if isinstance(v,np.ndarray)}
                for family in ('cells','sides','caps') if family in result and isinstance(result[family],dict)}
            save(f'fresh/LAYER_{layer:02}_METRIC_ARRAY_HASHES.json',hashes)
            return result
        rolling_metrics.evaluate_layer=record_metrics
    def timer(owner,name,label,detail=False):
        original=getattr(owner,name)
        @functools.wraps(original)
        def observed(*a,**kw):
            tick=time.perf_counter();result=original(*a,**kw)
            row=dict(kernel=label,seconds=time.perf_counter()-tick)
            if detail:
                row['counts']=result['counts'];row['cache']={k:v for k,v in a[0].last_stats.items() if k!='changed_cell_ids'}
            if label=='future_spots':
                planner=kw.get('planner')
                if planner is not None:row['proof_stats']=dict(planner.proofs.stats);row['proof_times']=dict(planner.proofs.times)
            if label.startswith('screen'):row['rejected']=result['rejected']
            rows.append(row);return result
        setattr(owner,name,observed)
    timer(DetailedCache,'get','detailed_future',True)
    timer(rolling_spots,'compare_spots','future_spots')
    for name in ('immediate_cell_screen','immediate_numeric_screen'):timer(staged_evaluation,name,'screen:'+name)
    timer(staged_evaluation,'exact_current','local_exact')
    sys.argv=[str(ROOT/'tools'/tool),*args]
    start=time.perf_counter()
    try:runpy.run_path(str(ROOT/'tools'/tool),run_name='__main__')
    finally:
        name=os.environ['TASK4I_STAGE_NAME']
        save('instrument/'+name+'.json',dict(tool=tool,rows=rows,seconds=time.perf_counter()-start,
            exact_local=dict(hits=LOCAL_EXACT.hits,misses=LOCAL_EXACT.misses,materializations=3*LOCAL_EXACT.misses),
            peak_rss=psutil.Process().memory_info().peak_wset))
if __name__=='__main__':run()
