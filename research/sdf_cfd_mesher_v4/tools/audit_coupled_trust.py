"""Replay saved LP inputs to verify actual trust-boundary usage and decisions."""
import json
import coupled_tangent_study as c
np=c.np;q=c.q
rows=[]
for path in sorted(c.OUT.glob('*/result.json')):
    if path.parent.name.startswith('exploratory'):continue
    r=json.loads(path.read_text())
    for h in r['history']:
        z=np.load(path.parent/f"model_{h['iteration']:02}.npz");u=z['u'];f=z['values'];J=z['J'];n=len(u);radius=h['trust_before'];limit=radius/np.sqrt(2)
        bounds=[(max(-1/np.sqrt(2)-v,-limit),min(1/np.sqrt(2)-v,limit)) for v in u]+[(None,None)]
        slack=np.array([0. if k[0]=='branch' else 1. for k in r['constraint_labels']])
        lp=c.linprog(np.r_[np.zeros(n),-1.],A_ub=np.c_[-J,slack],b_ub=f,bounds=bounds,method='highs-ds');assert int(lp.status)==h['LP_status']
        if not lp.success:continue
        du=lp.x[:-1];boundary=c.trust_boundary(du,radius)
        if 'score_after' in h:
            gain=float(min(f+J@du)-h['score_before']);ratio=(h['score_after']-h['score_before'])/gain if gain>0 else None
            new,action,why=c.adjust_trust(radius,h['search_step_accepted'],prediction_ratio=ratio,boundary=boundary)
            assert new==h['trust_after'] and action==h['trust_action']
        rows.append(dict(case=r['name'],iteration=h['iteration'],boundary_active=boundary,maximum_bound_usage=float(max(abs(du))/limit),unchanged_recorded_decision=True))
c.save('TRUST_BOUNDARY_AUDIT.json',dict(rows=rows,all_decisions_unchanged=True,scope='Actual-step boundary test replaces an unconditional boundary flag in the new study; all saved decisions remain identical. Existing scientific candidate replay remains valid.'))
