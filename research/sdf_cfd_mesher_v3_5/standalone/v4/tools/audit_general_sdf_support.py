"""Independent closure isolation and rejected alternative audit."""
import time,json
import qualify_general_sdf_witness as w
q=w.q;np=w.np

def run():
    s=q.load_schedule();isolated=[];alternatives=[];start=time.perf_counter()
    for old in q.definitions(s.targets):
        if old['kind'] not in ('wedge','frustum'):continue
        r=json.loads((w.OUT/old['name']/'result.json').read_text());x,f,c,spec=w.extended_front(old);sdf=w.ExtendedSDF(spec);top,p,b=q.foundation(x,f,c,sdf,spec);low=x;previous=None;rows=[]
        for k in range(1,r['last_valid_layer']+1):
            if k<=5:
                high=q.predict_next_front(low,s.targets[k-1],sdf,iteration_cap=256).coordinates;sealed,_=q.load_checkpoint(q.OUT/old['name'],k,top.sha256);high[p.vertices]=sealed[p.vertices]
            else:high=np.load(w.OUT/old['name']/f'front_{k:02}.npy')
            assert q.array_hash(high)==r['history'][k-1]['coordinate_hash']
            m=q.immediate_metrics(top,low,high,k,previous);events=m['events'];core_bad=[e for e in events if e['entity'] in (p.side_edges if e['kind']=='side' else p.core_columns)]
            assert not core_bad
            rows.append(dict(layer=k,core_hard_failures=0,non_core_diagnostic_failures=len(events),non_core_cell_columns=sorted(set(e['entity'] for e in events if e['kind']=='cell')),certificate_passed=r['history'][k-1]['certificate']['passed'],required_pairs=r['history'][k-1]['certificate']['required_pairs']))
            low=high;previous=m['cells']
        isolated.append(dict(name=old['name'],layers=rows,scope='Complete ordinary core/metric/certificate support. Non-core failures do not authorize exclusion of any required relation.'))
        tick=time.perf_counter();x,f,c,spec=w.exterior_front(old);sdf=w.ExteriorSDF(spec);top,p,b=q.foundation(x,f,c,sdf,spec);ox,of,oc=q.build(old)
        canonical=lambda a:sorted(tuple(sorted(tuple(map(float,v)) for v in tri)) for tri in a)
        assert canonical(x[f[c]])==canonical(ox[of[oc]])
        prefix=[];previous=None;history=[]
        for k,target in enumerate(s.targets):
            try:
                high=q.predict_next_front(prefix[-1] if prefix else x,target,sdf,iteration_cap=256).coordinates;r,m,ct=w.checked(x,f,top,p,prefix,high,previous);history.append(dict(layer=k+1,passed=r['passed'],failures=r.get('failures',[]),exact_failures=r.get('exact_failures',[])))
                if not r['passed']:break
                prefix.append(high);previous=m['cells']
            except (RuntimeError,ValueError) as exc:history.append(dict(layer=k+1,passed=False,projection_failure=str(exc)));break
        alternatives.append(dict(name=old['name'],definition=spec,foundation=b,core_triangle_coordinate_identity=True,last_valid_core_smoke=len(prefix),history=history,selected=False,reason='Did not remove all projection/interaction regularity blockers; no continuation authority assigned',seconds=time.perf_counter()-tick))
    w.save('CLOSURE_ISOLATION.json',isolated);w.save('REJECTED_CLOSURE_ALTERNATIVE.json',alternatives);w.save('SUPPORT_AUDIT_TIMING.json',dict(seconds=time.perf_counter()-start,peak_rss=q.psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run()
