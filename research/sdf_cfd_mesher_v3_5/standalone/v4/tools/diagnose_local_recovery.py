"""Independent two-layer schedule diagnostic on the accepted compressed front."""
import json,time,sys
import general_local_correction as g
q=g.q;np=g.np
tick=time.perf_counter();case=g.c.Case(q.definitions(q.load_schedule().targets)[-1]);folder=g.OUT/case.name
high=np.load(folder/'candidate.npy');result=json.loads((folder/'result.json').read_text());targets=np.array(result['local_targets'])
assert q.array_hash(high)==result['coordinate_hash']
committed=np.broadcast_to(case.s.targets[:3,None],(3,len(high)))
plan=g.geometric_plan(case.s,committed,targets,2)
nexttargets=plan['targets'][1];ancestry=g.b.owner(case.x,case.spec)
nominal,_=g.b.sphere_project(high,float(case.s.targets[4]),case.sdf,case.spec,ancestry)
try:
    future=g.project_field(high,nexttargets,case.sdf,case.spec,ancestry,case.active,nominal)
    m=q.immediate_metrics(case.top,case.low,high,4,case.previous)
    gate,_,ct=g.w.checked(case.x,case.f,case.top,case.verify,case.prefix+[high],future,m['cells'])
    preview=dict(gate=gate,coordinate_hash=q.array_hash(future),committed=False)
except (RuntimeError,ValueError) as exc:preview=dict(error=str(exc),committed=False);ct=0.
record=g.w.clean(dict(plan=plan,preview=preview,full_depth_qualified=False,termination_candidate=False))
if '--replay' in sys.argv:
    old=json.loads((g.OUT/'RECOVERY_DIAGNOSTIC.json').read_text());equal=q.identity(old)==q.identity(record)
    g.save('RECOVERY_DETERMINISM.json',dict(equal=equal,first=q.identity(old),replay=q.identity(record)));assert equal
else:g.save('RECOVERY_DIAGNOSTIC.json',record)
g.save('RECOVERY_'+('REPLAY_' if '--replay' in sys.argv else '')+'TIMING.json',dict(seconds=time.perf_counter()-tick,certificate_seconds=ct,peak_rss=q.psutil.Process().memory_info().peak_wset))
print('Recovery join',preview.get('gate',{}).get('passed'),preview.get('error'),flush=True)
