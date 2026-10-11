"""Bounded coordinate pattern search for individual controlling heights."""
import compressed_layer_recovery as r
q=r.q;np=r.np
def probe(case,prefix,history,previous,targets,active,passes=3):
    high,base,metric,t=r.evaluate(case,prefix,previous,targets,active);rows=[];times=[t];current=targets.copy()
    def rank(z):
        gate=z.get('gate',{});return (len(gate.get('exact_failures',[])),len(gate.get('failures',[]))) if 'gate'in z else (10**9,10**9)
    for iteration in range(passes):
        vertices=set()
        for e in base.get('gate',{}).get('exact_failures',[]):vertices.update(map(int,case.f[e['column']]))
        for e in base.get('gate',{}).get('failures',[]):vertices.update(map(int,case.f[e['columns']].ravel()))
        vertices=sorted(vertices&set(active));improved=False
        for v in vertices:
            for scale in (2.,.5,4.,.25,1.5,.75):
                trial=current.copy();trial[v]=history[-1][v]+(current[v]-history[-1][v])*scale
                hh,z,mm,tt=r.evaluate(case,prefix,previous,trial,active);times.append(tt)
                rows.append(dict(iteration=iteration,vertex=v,scale=scale,result=z,target_hash=q.array_hash(trial)))
                if z['passed']:return (hh,trial,z,mm),rows,times
                if rank(z)<rank(base):current=trial;base=z;improved=True;break
            if improved:break
        if not improved:break
    return None,rows,times
if __name__=='__main__':
    import sys
    c,p,h,m,s=r.seed();e=r.load(r.OUT/'result.json')['expansions'][-1];a=np.array(e['active']);weights=r.g.harmonic_weights(c.top,e['centres'],a)
    if '--assigned-secant' in sys.argv:
        rows=[];normals=c.sdf.evaluate(p[-1][a]).normal
        for factor in (1.,.75,.5,.25,.125,.0625,.03125):
            ratio=c.s.increments[4]/c.s.increments[3];multiplier=ratio*(1-(1-factor)*weights)
            target=h[-1]+(h[-1]-h[-2])*multiplier;target[weights==0]=c.s.targets[4]
            move=(p[-1]-p[-2])[a]*multiplier[a,None];high,z,mm,tt=r.evaluate(c,p,m,target,a,move)
            tangent=move-normals*np.sum(move*normals,axis=1)[:,None]
            if 'projection' in z:
                z['projection']['maximum_predictor_displacement']=float(np.linalg.norm(move,axis=1).max())
                z['projection']['max_tangent_displacement']=float(np.linalg.norm(tangent,axis=1).max())
            rows.append(dict(factor=factor,result=z))
        rows=r.w.clean(rows)
        if '--replay' in sys.argv:
            old=r.load(r.OUT/'ASSIGNED_SECANT.json');equal=q.identity(rows)==q.identity(old)
            r.save('ASSIGNED_SECANT_DETERMINISM.json',dict(equal=equal,first=q.identity(old),replay=q.identity(rows)));assert equal
        else:r.save('ASSIGNED_SECANT.json',rows)
        raise SystemExit(0)
    plan=r.plan(c.s,h,'receding',5,weights);targets=h[-1]+plan['height']*(1-(1-.125)*weights);targets[weights==0]=c.s.targets[4]
    accepted,rows,t=probe(c,p,h,m,targets,a);record=r.w.clean(dict(accepted=accepted is not None,trials=rows))
    if '--replay' in sys.argv:
        old=r.load(r.OUT/'HEIGHT_PROBE.json');equal=q.identity(old)==q.identity(record);r.save('HEIGHT_DETERMINISM.json',dict(equal=equal,first=q.identity(old),replay=q.identity(record)));assert equal
        r.save('HEIGHT_REPLAY_TIMING.json',t)
    else:r.save('HEIGHT_PROBE.json',record);r.save('HEIGHT_PROBE_TIMING.json',t)
    if accepted:
        high,targets,z,m=accepted;np.save(r.OUT/'height_candidate.npy',high);np.save(r.OUT/'height_targets.npy',targets)
    print('HEIGHT',accepted is not None,flush=True)
