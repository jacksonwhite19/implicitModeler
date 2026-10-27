"""Hold completed support fixed; compare the five historical search fields.

No old trial is repeated: all proposals have the new closed active support.
Future calculations are unnecessary for a current numeric hard-gate rejection.
"""
from task3_repair_study import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import active_patch,current_layer_metrics
from task3c_io import read as read_evidence
OUT=ROOT/'results/task3c'

def run(replay=False):
    started=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,*_=ctx;rows=[];timings={}
    audit=json.loads((OUT/'AUDIT.json').read_text())
    for a in audit:
        start=time.perf_counter();p=active_patch(top,a['closure_plan']['active']);incumbent=coords[0,p.active]
        fields,radii=search_fields(front,top,p,coords[0],schedule.targets[-1]);events=[]
        prior=read_evidence(OUT/f"{a['component']}_closed_support.json")
        ev=next(e for e in prior['history'] if e.get('potential'));projected=np.array(ev['projection']['projected'])
        cg=numeric_gates(current_layer_metrics(front,top,p.detailed,LayerView(coords[0],p.active,projected)),candidate_only=True)
        assert scientific_hash(cg)==scientific_hash(ev['candidate_gate']),'CandidateOnlyAdapterDrift'
        for round_id in range(6):
            radius=2.**(-round_id)
            for field,values in fields:
                trial,projection=project_tangent_move(incumbent,values,radius*radii,sdf,schedule.targets[0])
                gate=numeric_gates(current_layer_metrics(front,top,p.detailed,LayerView(coords[0],p.active,trial)),candidate_only=True)
                e=dict(field=field,radius=radius,projection=projection,coordinate_hash=array_hash(trial),candidate_gate=gate,
                    displacement=float(np.linalg.norm(trial-incumbent,axis=1).max()),future_evaluated=False)
                if gate['passed']:
                    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=fc.forecast(p.active,trial)
                    dc=DetailedCache();r=dc.get(front,future,top,p.detailed);baseline=dc.get(front,fc.baseline,top,p.detailed)
                    spec=next(s for s in components() if s['name']==a['component'])
                    target=target_status(r,spec['columns'],spec['metric']);bt=target_status(baseline,spec['columns'],spec['metric'])
                    e.update(future_evaluated=True,target=target,improves=hazard_improves(bt,target),
                        new_earlier_failures=no_new_earlier_failure(failure_times(baseline),failure_times(r)))
                events.append(e)
        rows.append(dict(component=a['component'],candidate_only_adapter_exact=True,events=events));timings[a['component']]=dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset)
        print(a['component'],'current passes',sum(e['candidate_gate']['passed'] for e in events),flush=True)
    h=scientific_hash(rows)
    if replay:assert scientific_hash(json.loads((OUT/'SEARCH_ABLATION.json').read_text()))==h
    else:(OUT/'SEARCH_ABLATION.json').write_text(json.dumps(plain(rows),sort_keys=True,separators=(',',':'))+'\n')
    (OUT/('SEARCH_ABLATION_REPLAY.json' if replay else 'SEARCH_ABLATION_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('SEARCH_ABLATION_REPLAY_TIMINGS.json' if replay else 'SEARCH_ABLATION_TIMINGS.json')).write_text(json.dumps(timings|dict(total_seconds=time.perf_counter()-started),indent=2,sort_keys=True)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
