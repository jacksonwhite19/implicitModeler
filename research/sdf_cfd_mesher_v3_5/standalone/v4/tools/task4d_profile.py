"""Preserved search census and one bounded instrumented production replay."""
from task4d_common import *
import cProfile,pstats

def census():
    rows=[]
    for layer in range(1,21):
        search=read(BASE/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'))
        timings=read(BASE/('CORRECTION_TIMINGS.json' if layer==1 else f'LAYER{layer}_PREPARE_TIMINGS.json'))
        for outcome in search['outcomes']:
            p=outcome.get('scope',outcome.get('patch'))
            if not isinstance(p,dict):continue
            index=outcome.get('region',outcome.get('patch'));key='region' if layer==1 else 'patch'
            history=[r for r in search['history'] if r[key]==index]
            ts=[r for r in timings['stages'] if r[key]==index]
            accepted=[r for r in history if r.get('accepted')]
            rows.append(dict(layer=layer,region=index,active_vertices=len(p['active']),scope=p,
                tangent_dofs=2*len(p['active']),history=[dict(mode=h.get('mode'),factor=h.get('factor'),accepted=h.get('accepted',False),reason=h.get('reason'),violations=h.get('gate',{}).get('violations'),scientific_hash=scientific_hash(h)) for h in history],accepted_steps=len(accepted),rejected_steps=len(history)-len(accepted),
                trials=len(history),search_seconds=sum(r['seconds'] for r in ts),
                forecast_seconds=sum(r.get('forecast',{}).get('seconds',0) for r in ts) if layer>1 else None,
                detailed_seconds=sum(r.get('detailed_seconds',0) for r in ts) if layer>1 else None,
                projection_seconds=sum(r.get('proposal_projection_seconds',0) for r in ts) if layer>1 else None,
                sensitivity_seconds=0,sensitivity_dofs=0,trust_region_optimizer=False,
                spot_seconds=None,serialization_seconds=None,metric_seconds=None,
                uninstrumented_note='Null is unavailable in frozen timing schema; detailed_seconds includes metric evaluation. No sensitivity or trust-region optimizer exists in production.' ))
    save('ACTIVE_SEARCH_CENSUS.json',dict(regions=rows,count=len(rows)))
    print('CENSUS',len(rows),sum(r['search_seconds'] for r in rows),flush=True)

def profile(layer):
    import task4c_runner_replay as replay
    replay.OUT=OUT/'cold_profile';replay.OUT.mkdir(exist_ok=True)
    replay.save=lambda name,value:save('COLD_'+name,value)
    prof=cProfile.Profile();prof.enable();replay.run(layer);prof.disable()
    stats=pstats.Stats(prof);rows=[]
    for (path,line,name),(cc,nc,tt,ct,callers) in stats.stats.items():
        rows.append(dict(path=path.replace(str(REPO),'REPO'),line=line,function=name,primitive_calls=cc,calls=nc,self_seconds=tt,cumulative_seconds=ct))
    save(f'PROFILE_{layer:02}.json',dict(instrumented=True,rows=sorted(rows,key=lambda r:-r['cumulative_seconds'])))
    print('PROFILE COMPLETE',layer,flush=True)

if __name__=='__main__':
    census()
    if len(sys.argv)>1:profile(int(sys.argv[1]))
