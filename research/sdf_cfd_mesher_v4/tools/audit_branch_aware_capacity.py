"""Extra bounded diagnostics; never confer candidate admission."""
import json,time
import qualify_branch_aware_capacity as d
q=d.q;np=d.np;w=d.w;b=d.b

def run():
    tick=time.perf_counter();s=q.load_schedule();rows=[]
    for old in q.definitions(s.targets):
        if old['kind'] not in ('spheres','wedge','frustum') or old.get('cavity'):continue
        x,f,top,p,base,spec,sdf,prefix,previous=d.prepare(old);k=len(prefix);target=float(s.targets[k]);data=json.loads((d.OUT/(old['name']+'.json')).read_text());high=np.load(d.OUT/(old['name']+'_candidate.npy'))
        z=sdf.evaluate(high);assert np.all(abs(z.phi-target)<=z.source_error_bound) and np.all(z.gradient_norm>0)
        start=b.owner(prefix[-1],spec);end=b.owner(high,spec);changed=np.flatnonzero(start!=end)
        fields=dict(name=old['name'],target_admitted=True,finite=True,valid_gradient=True,changed_nearest_owners=changed.tolist(),changed_core_owners=np.intersect1d(changed,p.core_vertices).tolist(),core_vertices=p.core_vertices.tolist(),core_columns=p.core_columns.tolist(),support_vertices=p.vertices.tolist(),projection_max_residual=float(max(abs(z.phi-target))),source_branch_available=True,first_unadmitted=k+1,capacity_authority=False)
        trials=data['steering'];evaluated=[v for v in trials if 'gate' in v]
        fields['steering_counts']=dict(attempts=len(trials),evaluated=len(evaluated),projection_or_evaluation_errors=sum(v['status']=='ProjectionOrEvaluationFailure' for v in trials),duplicates=sum(v['status']=='DuplicateCandidate' for v in trials),accepted=sum(v.get('passed',False) for v in trials),minimum_exact_failures_on_evaluated_trial=min((len(v['gate'].get('exact_failures',[])) for v in evaluated),default=None))
        fields['tangent_summary']=dict(corners=len(data['tangent_evidence']),nonpositive_corners=sum(v['evidence']['sign']<=0 for v in data['tangent_evidence']),nonpositive_with_nonzero_exact_tangent_derivative=sum(v['evidence']['sign']<=0 and v['evidence']['tangent_controls_corner'] for v in data['tangent_evidence']),obstruction_proved=False,scope='Exact first derivative for bottom corners only; nonzero derivative is not full nonlinear mesh feasibility, zero derivative is not global impossibility.')
        if old['kind']=='spheres':
            fields['ancestral_branch_verified']=all(b.check_source(point,target,spec,int(source),sdf) for point,source in zip(high,data['source_ancestry']));assert fields['ancestral_branch_verified']
            traces=data['ownership_transitions'];fields['ordinary_iterations']=[len(v['history'])-1 for v in traces];fields['ordinary_owner_switches']=[sum(a['owner']!=bb['owner'] for a,bb in zip(v['history'],v['history'][1:])) for v in traces]
            # Bounded diagnostic certification even though exact Jacobians reject.
            # Never an acceptance certificate: one decisive pair is sufficient
            # only to reject this candidate, not all possible continuations.
            ct=time.perf_counter()
            try:
                surfaces,labels=q.surface_arrays(x,top.edges,prefix+[high]);native=q.Filter();forest=native.forest(surfaces,len(top.edges));pairs=forest.pairs(k+1);forest.close()
                a=(pairs>>np.uint64(32)).astype(int);bb=(pairs&np.uint64(0xffffffff)).astype(int);E=len(top.edges);owners=b.owner(x,spec)[top.edges[:,0]]
                keep=(a//E==k)&(bb//E==k)&(owners[a%E]!=owners[bb%E]);pairs=pairs[keep];a=a[keep];bb=bb[keep]
                dist=np.sum((surfaces[a].mean(1)-surfaces[bb].mean(1))**2,axis=1);ordered=pairs[np.lexsort((pairs,dist))];checks=[]
                # Only one nearest pair, canonical ID tie break, fixed before exact evaluation.
                for packed in ordered[:1]:
                    result,perf=q.prove(surfaces,labels,np.array([packed],np.uint64),native)
                    ids=[int(packed>>np.uint64(32)),int(packed&np.uint64(0xffffffff))]
                    checks.append(dict(pair=ids,surfaces=surfaces[ids].tolist(),labels=labels[ids].tolist(),result=q.plain(result)))
                fields['diagnostic_certificate']=dict(candidate_pairs=len(pairs),checks=checks,complete=False,admission_authority=False,scope='Nearest cross-component current-side AABB survivor; no full proof coverage on rejected geometry')
            except (RuntimeError,ValueError,ArithmeticError) as exc:
                fields['diagnostic_certificate']=dict(error=str(exc),admission_authority=False,complete=False)
            fields['certificate_seconds']=time.perf_counter()-ct
        rows.append(fields)
    scientific=[{k:v for k,v in r.items() if k!='certificate_seconds'} for r in rows]
    # Certificate performance contains runtime; omit it from deterministic identity.
    for r in scientific:
        if 'diagnostic_certificate' in r:r['diagnostic_certificate'].pop('performance',None)
    return scientific,dict(seconds=time.perf_counter()-tick,certificate_seconds=sum(r.get('certificate_seconds',0) for r in rows),peak_rss=q.psutil.Process().memory_info().peak_wset)

if __name__=='__main__':
    a,t=run();c,u=run();assert q.identity(a)==q.identity(c)
    d.save('BRANCH_AUDIT.json',a);d.save('AUDIT_TIMINGS.json',dict(first=t,replay=u));d.save('AUDIT_DETERMINISM.json',dict(equal=True,scientific_hash=q.identity(a)))
