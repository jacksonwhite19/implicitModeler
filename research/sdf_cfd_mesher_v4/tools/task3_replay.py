"""Replay only the already executed prefix, then stop at the same blocker."""
from task3_study import *
from task2_describe import decode
from task3_certificate_reproducer import reproduce as failure

if __name__=='__main__':
    started=time.perf_counter();context=load();first=[];second=[];timings={}
    for spec in components()[:2]:
        old=decode(json.loads((OUT/(spec['name']+'.json')).read_text()));r,p=run_component(spec,context)
        assert scientific_hash(r)==scientific_hash(old),(spec['name'],'ReplayDrift')
        first.append(old);second.append(r);timings[spec['name']]=p
    repro=decode(json.loads((OUT/'CERTIFICATE_FAILURE_REPRODUCER.json').read_text()))
    again=failure(context,repro['field_index'],repro['radius']);assert scientific_hash(repro)==scientific_hash(again)
    # Exact signs for the already tested outer-ring wall-normal proposal.
    front,top,sdf,schedule,coords,normals,*_=context;patch=build_patch(top,components()[0]['columns'],active_rings=1)
    fields,radii=search_fields(front,top,patch,coords[0],schedule.targets[-1]);trial,projection=project_tangent_move(coords[0,patch.active],fields[0][1],radii,sdf,schedule.targets[0])
    cache=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=cache.forecast(patch.active,trial)
    exact=exact_candidate_jacobians(front,top,future,patch.detailed);again_exact=exact_candidate_jacobians(front,top,future,patch.detailed)
    assert exact==again_exact;write('REJECTED_PROPOSAL_EXACT_SIGNS.json',exact)
    h1=scientific_hash(dict(components=first,certificate_stop=repro,exact_rejected_proposal=exact))
    h2=scientific_hash(dict(components=second,certificate_stop=again,exact_rejected_proposal=again_exact));assert h1==h2
    write('DETERMINISM.json',dict(run1=h1,run2=h2,passed=True,scope='Complete executed component histories, baseline/final arrays, failed candidate reproducer, and exact rejected-proposal sign evidence. Unexecuted components are not claimed.'))
    write('REPLAY_TIMINGS.json',dict(components=timings,seconds=time.perf_counter()-started,peak_rss=max(psutil.Process().memory_info().peak_wset,max(v['peak_rss'] for v in timings.values()))))
    print(json.dumps(dict(passed=True,run1=h1,run2=h2,exact_sign_counts=dict(collections.Counter(r['jacobian_sign'] for r in exact['records'])))))
