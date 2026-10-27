"""Bounded study driver; no production call site or old artifact writes."""
import argparse,json,time,hashlib
import general_local_correction as g
q=g.q;np=g.np;c=g.c;w=g.w;b=g.b
from verify_coupled_tangent_study import conditioning

def general():
    tick=time.perf_counter();case=c.Case(q.definitions(q.load_schedule().targets)[-1]);setup=time.perf_counter()-tick
    support=g.support_check(case);assert support['passed'];prefix=[];previous=None;prefix_gates=[]
    for high in case.prefix:
        gate,m,_=w.checked(case.x,case.f,case.top,case.verify,prefix,high,previous)
        assert gate['passed'];prefix_gates.append(gate);prefix.append(high);previous=m['cells']
    baseline,_,_=w.checked(case.x,case.f,case.top,case.verify,case.prefix,case.high,case.previous)
    accepted,history,timing,weights=g.search_compression(case)
    assert accepted is not None
    high,targets,gate,metrics,factor=accepted
    assert np.array_equal(high[weights==0],q.predict_next_front(case.low,case.target,case.sdf,iteration_cap=256).coordinates[weights==0])
    committed=np.broadcast_to(case.s.targets[:case.k,None],(case.k,len(case.x))).copy()
    unrecovered=g.local_schedule(case.s,committed,targets)
    recovery=g.local_schedule(case.s,committed,targets,recover=True)
    plan=g.plan_future(case.s,committed,targets,2,recover=True)
    # One future join is diagnostic only. Never promote bookkeeping to recovery.
    pt=time.perf_counter()
    try:
        nexttargets=plan['targets'][1];nextnom,_=b.sphere_project(high,float(case.s.targets[case.k+1]),case.sdf,case.spec,b.owner(case.x,case.spec))
        nextfront=g.project_field(high,nexttargets,case.sdf,case.spec,b.owner(case.x,case.spec),case.active,nextnom)
        future,_,_=w.checked(case.x,case.f,case.top,case.verify,case.prefix+[high],nextfront,metrics['cells'])
        future.update(committed=False,coordinate_hash=q.array_hash(nextfront))
    except (RuntimeError,ValueError) as exc:future=dict(passed=False,committed=False,error=str(exc))
    planning_seconds=time.perf_counter()-pt
    surfaces,labels=q.surface_arrays(case.x,case.top.edges,case.prefix+[high]);pair,_=q.prove(surfaces,labels,np.array([(7757<<32)|9024],np.uint64),q.Filter())
    sample=case.sdf.evaluate(high)
    old=c.OUT/'general_asymmetric_passage/result.json';old4=c.OUT/'general_asymmetric_passage__opposed_z/result.json'
    result=dict(name=case.name,classification='CompressionResolved',mechanism='compression only',support=case.support,support_check=support,
                validated_fixed_prefix=prefix_gates,baseline=baseline,tangent_reference={str(p.relative_to(c.ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (old,old4)},
                history=history,compression_factor=factor,accepted_gate=gate,coordinate_hash=q.array_hash(high),target_hash=q.array_hash(targets),
                weights=weights,local_targets=targets,schedule_without_recovery=unrecovered,schedule_recovery_proposal=recovery,plan=plan,future_join=future,
                changed_committed_layers=[4],first_layer_unchanged=True,wall_unchanged=True,production_changed=False,termination_candidate=False,
                collision_pair=q.plain(pair),tails=case.tails(high),conditioning_before=conditioning(case,case.high),conditioning_after=conditioning(case,high),
                target_residual_max=float(np.max(abs(sample.phi-targets))),target_admission_passed=bool(np.all(abs(sample.phi-targets)<=sample.source_error_bound)),
                maximum_coordinate_change=float(np.linalg.norm(high-case.high,axis=1).max()),maximum_tangent_correction=0.,
                practical_compression_note='Substantial compression; no universal minimum height is introduced. Future total-depth feasibility is unproved.')
    return w.clean(result),dict(seconds=time.perf_counter()-tick,setup_seconds=setup,planning_seconds=planning_seconds,**timing),high

def channel(spec,expanded=False):
    tick=time.perf_counter();case=c.Case(spec);old_support=g.support_check(case);oldlast=case.k
    # New independent study history. Never repair a sealed committed coordinate.
    prefix=[];previous=None;hist=[];targets_history=[];timing=dict(projection_seconds=0.,verification_seconds=0.);last_high=None
    if expanded:
        prior=json.loads((g.OUT/case.name/'result.json').read_text());r=prior['history'][-1]['nominal_gate'];ids=set(case.active.tolist())
        for e in r.get('failures',[]):ids.update(map(int,case.f[e['columns']].ravel()))
        for e in r.get('exact_failures',[]):ids.update(map(int,case.f[e['column']].ravel()))
        for v in list(ids):ids.update(map(int,case.top.vertex_neighbors.row(v)))
        case.active=np.array(sorted(ids),int);case.patch=q.bounded_patch(case.top,case.active)
        case.core=np.union1d(case.core,case.patch.detailed.core_columns)
        _,case.verify,_=q.foundation(case.x,case.f,case.core,case.sdf,case.spec)
        case.support['controlling_vertices']=sorted(ids)
    if not old_support['passed'] and not expanded:
        # Include first-layer controlling entities and their full metric closure.
        probe=q.predict_next_front(case.x,float(case.s.targets[0]),case.sdf,iteration_cap=256).coordinates
        r,_,_=w.checked(case.x,case.f,case.top,case.verify,[],probe,None)
        ids=set(case.active.tolist())
        for e in r.get('failures',[]):ids.update(map(int,case.f[e['columns']].ravel()))
        for e in r.get('exact_failures',[]):ids.update(map(int,case.f[e['column']].ravel()))
        case.active=np.array(sorted(ids),int);case.patch=q.bounded_patch(case.top,case.active)
        case.core=np.union1d(case.core,case.patch.detailed.core_columns)
        _,case.verify,_=q.foundation(case.x,case.f,case.core,case.sdf,case.spec)
        case.support['controlling_vertices']=case.active.tolist()
    wall_support=g.support_check(case,prefix=[]);assert wall_support['passed']
    for k in range(oldlast+1):
        low=prefix[-1] if prefix else case.x;row=dict(layer=k+1,support=g.support_check(case,prefix))
        if not row['support']['passed']:row.update(committed=False,error='InvalidFixedSupport');hist.append(row);break
        pt=time.perf_counter()
        try:high=q.predict_next_front(low,float(case.s.targets[k]),case.sdf,iteration_cap=256).coordinates
        except (RuntimeError,ValueError) as exc:row.update(committed=False,error=str(exc));hist.append(row);break
        timing['projection_seconds']+=time.perf_counter()-pt
        vt=time.perf_counter();r,m,_=w.checked(case.x,case.f,case.top,case.verify,prefix,high,previous);timing['verification_seconds']+=time.perf_counter()-vt
        row['nominal_gate']=r;mechanism='NominalAdvanceValid';targets=np.full(len(high),case.s.targets[k])
        if not r['passed']:
            candidate,trials=g.tangent_search(case,prefix,previous,high);row['tangent_trials']=trials
            if candidate is not None:high,r,m=candidate;mechanism='TangentCorrectionResolved'
            elif k:
                candidate,trials,tt,weights=g.search_compression(case,prefix=prefix,previous=previous,nominal=high,previous_targets=targets_history[-1])
                row['compression_trials']=trials
                for key in timing:timing[key]+=tt[key]
                if candidate is not None:high,targets,r,m,factor=candidate;mechanism='CompressionResolved';row.update(factor=factor,weights=weights)
        row.update(gate=r,committed=bool(r['passed']),mechanism=mechanism if r['passed'] else 'Unresolved',coordinate_hash=q.array_hash(high),targets=targets)
        hist.append(row);print(case.name,'layer',k+1,'passed',r['passed'],mechanism,flush=True)
        if not r['passed']:break
        prefix.append(high.copy());targets_history.append(targets.copy());previous=m['cells'];last_high=high
    passed=len(prefix)==oldlast+1
    classification=('CompressionResolved' if any(r.get('mechanism')=='CompressionResolved' for r in hist) else 'TangentCorrectionResolved') if passed else 'SupportBlocked'
    result=dict(name=case.name+('__expanded' if expanded else ''),classification=classification,old_support=old_support,wall_support=wall_support,
                scope='Independent expanded-support rebuild from unchanged wall and fixed h1; sealed prefixes untouched',
                active_vertices=case.active,verification_columns=case.core,metric_columns=case.verify.columns,side_edges=case.verify.side_edges,
                history=hist,last_valid_layer=len(prefix),requested_test_layer=oldlast+1,original_last_valid_layer=oldlast,
                original_unadmitted_layer_resolved=passed,wall_unchanged=True,first_layer_height_unchanged=True,
                termination_candidate=False,infeasibility_proved=False,capacity_authority=False,
                limitation=None if passed else 'A bounded failed search is not incapacity. Growth redistribution cannot repair fixed support or a rejected immediate layer; no future plan is committed.')
    return w.clean(result),dict(seconds=time.perf_counter()-tick,**timing),last_high

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--case',default='general');ap.add_argument('--replay',action='store_true');ap.add_argument('--expanded',action='store_true');a=ap.parse_args()
    if a.case=='general':r,t,high=general()
    else:r,t,high=channel(next(s for s in q.definitions(q.load_schedule().targets) if s['name']==a.case),a.expanded)
    name=r['name'];key=q.identity(r);t['peak_rss']=q.psutil.Process().memory_info().peak_wset
    if a.replay:
        old=json.loads((g.OUT/name/'result.json').read_text());g.save(name+'/determinism.json',dict(equal=key==q.identity(old),first=q.identity(old),replay=key));assert key==q.identity(old)
        g.save(name+'/replay_timing.json',t)
    else:
        g.save(name+'/result.json',r);g.save(name+'/timing.json',t)
        if high is not None:np.save(g.OUT/name/'candidate.npy',high)
        if a.case=='general':
            np.save(g.OUT/name/'targets.npy',np.array(r['local_targets']))
            g.save(name+'/checkpoint_04.json',dict(study_only=True,layer=4,coordinate_hash=r['coordinate_hash'],target_hash=r['target_hash'],topology_hash=r['support']['topology_hash'],gate=r['accepted_gate'],production_commit=False))
    print(name,r['classification'],t,flush=True)
if __name__=='__main__':main()
