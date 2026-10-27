"""One shared negative-cap proposal on exact critical-corner support.

The three negative-cap diagnostics found the identical 392 required vertices.
Do not repeat exact certification of nearly identical seed-dependent fields:
first test a common field that leaves noncritical seed vertices unchanged.
"""
from task3_repair_study import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import active_patch,current_layer_metrics
from task3c_io import read as read_evidence,write as write_evidence
OUT=ROOT/'results/task3c'

def run(replay=False):
    start=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,*_=ctx
    sens=read_evidence(OUT/'SENSITIVITY.json')['sensitivities'];neg=[s for s in sens if s['component']!='component_00']
    ids=np.asarray(neg[0]['required_vertices']);assert all(s['required_vertices']==ids.tolist() for s in neg)
    audit=json.loads((OUT/'AUDIT.json').read_text());union=sorted({v for a in audit if a['component']!='component_00' for v in a['closure_plan']['active']})
    p=active_patch(top,union);field=np.zeros((len(ids),3));field[:,1]=-1
    trial,projection=project_tangent_move(coords[0,ids],field,np.full(len(ids),schedule.targets[0]/2),sdf,schedule.targets[0])
    gate=numeric_gates(current_layer_metrics(front,top,p.detailed,LayerView(coords[0],ids,trial)),candidate_only=True)
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=fc.forecast(ids,trial);dc=DetailedCache()
    current=dc.get(front,future,top,p.detailed);base=dc.get(front,fc.baseline,top,p.detailed)
    assert scientific_hash(gate)==scientific_hash(numeric_gates(current,candidate_only=True))
    allcollateral=no_new_earlier_failure(failure_times(base),failure_times(current));targets=[]
    for spec in components():
        if spec['name'] in ('component_01','component_02','component_04'):
            b=target_status(base,spec['columns'],spec['metric']);t=target_status(current,spec['columns'],spec['metric'])
            targets.append(dict(component=spec['name'],baseline=b,candidate=t,improves=hazard_improves(b,t)))
    ex=exact_candidate_jacobians(front,top,future,p.detailed);pair=pair_evidence(front,top,future)
    result=dict(active_ids=ids,verification_active_union=union,patch=p.record(),projection=projection,coordinate_hash=array_hash(trial),
        baseline=base,candidate=current,baseline_tails=sorted_tails(base),candidate_tails=sorted_tails(current),
        candidate_gate=gate,exact_jacobians=ex,targets=targets,new_earlier_failures=allcollateral,
        first_future_invalid=min(failure_times(current).values(),default=None),known_pair=pair,
        future_coordinates=future.local['coordinates'],future_normals=future.local['normal'],
        eligible=gate['passed'] and ex['passed'] and not allcollateral and all(t['improves'] for t in targets))
    h=scientific_hash(result)
    if replay:assert scientific_hash(read_evidence(OUT/'COMMON_NEGATIVE_CAP.json'))==h
    else:write_evidence(OUT/'COMMON_NEGATIVE_CAP.json',result)
    (OUT/('COMMON_CAP_REPLAY.json' if replay else 'COMMON_CAP_HASH.json')).write_text(json.dumps(dict(scientific_hash=h,eligible=result['eligible']),indent=2)+'\n')
    (OUT/('COMMON_CAP_REPLAY_TIMINGS.json' if replay else 'COMMON_CAP_TIMINGS.json')).write_text(json.dumps(dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print('common cap',gate['violations'],result['eligible'],pair['classification'],flush=True)
if __name__=='__main__':run('--replay' in sys.argv)
