"""Replay compact policy inputs and invoke private qualified cavity mechanics."""
import time,json,sys
import termination_policy_cohort as c
p=c.policy;g=c.g;s=c.s;q=c.q;OUT=c.OUT

def run():
    reuse='--reuse-qualified-invocations' in sys.argv
    if reuse:c.BINDINGS.update(s.load(OUT/'SOURCE_BINDINGS.json'))
    oldrows={r['name']:r for r in s.load(OUT/'POLICY_TRACES.json')} if reuse else {}
    oldinvocations=s.load(OUT/'REMESH_INVOCATIONS.json') if reuse else {}
    cases=c.positive_cases()+c.negative_cases();rows=[];timings=[];invocations={};pure_total=0.
    for case in cases:
        name=case['name'];e=case['evidence'];st=time.perf_counter();engine=p.LocalTerminationPolicy.start(e);elapsed=time.perf_counter()-st;pure_total+=elapsed
        eligible=engine.decision['next_action']=='RollbackRemesh';initial=engine.decision.copy()
        checkpoint=OUT/'checkpoints'/f'{name}.json';checkpoint.parent.mkdir(exist_ok=True)
        engine.save(checkpoint);restarted=p.LocalTerminationPolicy.restart(checkpoint,e['context']);assert restarted.decision==engine.decision
        if case['positive']:
            assert eligible;backend=c.QualifiedBackend(case,fresh=name=='clean_curved_passage')
            if reuse:
                assert e==oldrows[name]['input']
                backend.details=oldinvocations[name];backend.runtime=0.;backend.fresh=False
                restarted.invoke(e['context'],lambda request:oldrows[name]['invocation']['receipt'],checkpoint)
            else:
                restarted.invoke(e['context'],backend,checkpoint)
            assert restarted.decision['state']=='Resolved'
            restored=p.LocalTerminationPolicy.restart(checkpoint,e['context']);assert restored.decision==restarted.decision
            # A completed checkpoint must not re-invoke geometry.
            restored.invoke(e['context'],lambda _: (_ for _ in ()).throw(AssertionError('duplicate invocation')))
            assert restored.decision==restarted.decision
            invocations[name]=backend.details
            timings.append(dict(case=name,policy_seconds=elapsed,backend_seconds=backend.runtime,fresh_full_certificate=backend.fresh,receipt_reuse=reuse))
        else:
            assert not eligible and engine.decision['disposition']==case['expected']
            timings.append(dict(case=name,policy_seconds=elapsed,backend_seconds=0.))
        rows.append(dict(name=name,positive=case['positive'],input=e,initial_decision=initial,final_decision=restarted.decision,
                         invocation=restarted.invocation,first_warning=case.get('first_warning'),first_hard_failure=case.get('first_hard_failure'),
                         first_eligible_layer=e['context']['layer'] if eligible else None,
                         invalid_next_layer_committed=case.get('invalid_next_layer_committed'),scope=case.get('scope')))
    g.save(OUT/'POLICY_TRACES.json',rows);g.save(OUT/'REMESH_INVOCATIONS.json',invocations)
    # Independent policy replay. Remesh source identity has just been verified by
    # live discovery/reconstruction; replay uses the immutable backend receipts.
    replay=[];start=time.perf_counter()
    for row in rows:
        engine=p.LocalTerminationPolicy.start(row['input']);assert engine.decision==row['initial_decision']
        if row['positive']:
            engine.invoke(row['input']['context'],lambda request,r=row:r['invocation']['receipt'])
        assert engine.decision==row['final_decision']
        replay.append(dict(name=row['name'],equal=True,decision_hash=q.identity(engine.decision)))
    g.save(OUT/'DETERMINISM.json',dict(equal=True,cases=replay,science_hash=q.identity(rows),runtime_excluded=True))
    g.save(OUT/'RUNTIME.json',dict(policy_total_seconds=pure_total,policy_max_seconds=max(r['policy_seconds'] for r in timings),regions=len(rows),stages=timings,receipt_replay_seconds=time.perf_counter()-start,peak_rss=q.psutil.Process().memory_info().peak_wset))
    g.save(OUT/'SOURCE_BINDINGS.json',c.BINDINGS)
    g.save(OUT/'COHORT_RESULT.json',dict(positive_cases=sum(r['positive'] for r in rows),negative_cases=sum(not r['positive'] for r in rows),false_positives=sum(not r['positive'] and r['initial_decision']['next_action']=='RollbackRemesh' for r in rows),false_negatives=sum(r['positive'] and r['initial_decision']['next_action']!='RollbackRemesh' for r in rows),all_positive_resolved=all(r['final_decision']['state']=='Resolved' for r in rows if r['positive'])))
    print('Policy cohort complete',len(rows),flush=True)

if __name__=='__main__':run()
