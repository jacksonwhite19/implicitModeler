"""Incremental qualification in a preserved real local scope, before integration."""
from task4g_common import *
from task4f_instrument import same_arrays
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.forecast.detailed.dependency_metrics import LocalMetricEngine
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
import psutil

def run():
    require_time();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates))
    _,coords,_=prod.load_candidate(BASE/'LAYER3_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    o=next(o for o in read(BASE/'LAYER3_CORRECTIONS.json') if o['patch']==4)
    p=prod.bounded_patch(top,np.array(o['scope']['active'])).detailed;engine=LocalMetricEngine();rows=[]
    original=coords.copy();v=int(p.core_vertices[0])
    for case in ('baseline','one_vertex','rollback','exact_hit'):
        if case=='one_vertex':coords[3:,v,0]=np.nextafter(coords[3:,v,0],np.inf)
        if case=='rollback':coords=original.copy()
        f=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
        t=time.perf_counter();old=measure(front,f,top,p);oldtime=time.perf_counter()-t
        t=time.perf_counter();new=engine.measure(front,f,top,p);newtime=time.perf_counter()-t
        assert same_arrays(old,new),case
        stats=engine.last_stats.copy();stats.pop('changed_cell_ids')
        rows.append(dict(case=case,old_seconds=oldtime,new_seconds=newtime,**stats,hash=scientific_hash(new),bitwise_equal=True))
        print('QUALIFIED',case,oldtime,newtime,flush=True)
    save('BOUNDED_EQUIVALENCE.json',dict(rows=rows,scope=p.record(),peak_rss=psutil.Process().memory_info().peak_wset))
if __name__=='__main__':run()
