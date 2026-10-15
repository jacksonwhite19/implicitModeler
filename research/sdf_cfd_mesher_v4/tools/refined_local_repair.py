"""Generic strict-feasibility proposals using existing V4 metric owners.

Minimum slack is a proposal objective, never an acceptance threshold. Raw stock
quality and exact single-vertex preservation remain decisive. No publication.
"""
from collections import Counter
import time
import json
import numpy as np
from background_local_repair import LocalRepair, sphere_link, preserve_step, verify_edits


def record_key(row):
    return (row['kind'],tuple(sorted(row['vertices']))) if row['kind']=='face_quality' else (row['kind'],row['id'])


def raw_failures(q):
    """Individual stock comparisons, not solver convergence or tolerances."""
    bad=set()
    for r in q['cells']:
        for name,failed in (('volume',not r['volume']>0),('determinant',not r['determinant']>=.0011),('aspect',not r['aspect']<=1000)):
            if failed:bad.add(('cell',r['id'],name))
    for r in q['exact']:
        for name in ('volume_sign','jacobian_sign'):
            if r[name]<=0:bad.add(('exact',r['id'],name))
    for r in q['faces']:
        k=tuple(sorted(r['vertices']))
        for name,failed in (('weight',not r['weight']>=.055),('skewness',not r['skewness']<=3.63636),('ratio',not r['symmetric_volume_ratio']>=.01)):
            if failed:bad.add(('face',k,name))
    return bad


class RefinedRepair(LocalRepair):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.reports={record_key(r):r for r in args[5]}
        self.sweeps=[];self.pending=None

    def stock(self,star,owners,needed):
        from prism_hotpath_v2 import batch_quality
        local=[self.byid[i] for i in sorted(needed)]
        return batch_quality(self.points,local,owners,0,active_cells={c['id'] for c in star},
                             mixed_cells=True,face_batch_size=4096)

    def update_reports(self,star,owners,q):
        ids={c['id'] for c in star};keys={k for k,r in owners.items() if len(r)==2}
        remove=[k for k in self.reports if (k[0]=='face_quality' and k[1] in keys) or
                (k[0]!='face_quality' and k[1] in ids)]
        for k in remove:del self.reports[k]
        self.reports.update((record_key(r),r) for r in q['failures'])

    def refresh_stars(self):
        from background_local_repair import star_index
        bad=set()
        for r in self.reports.values():
            bad.update(r['owners'] if r['kind']=='face_quality' else [r['id']])
        self.wanted=sorted({v for i in bad if i in self.core_ids for v in self.byid[i]['vertices']}-self.fixed)
        self.stars=star_index(self.cells,set(self.wanted))

    def sweep(self,*,seconds,maxiter,ftol):
        from scipy.optimize import minimize
        import local_termination_topology as t
        start=time.perf_counter();deadline=start+seconds;complete=True;attempts=0;accepted=0
        class BudgetExpired(Exception):pass
        todo=self.wanted if self.pending is None else self.pending
        self.pending=None
        for cursor,v in enumerate(todo):
            if time.perf_counter()>=deadline:complete=False;self.pending=todo[cursor:];break
            star=self.stars[v];sphere_link(star,v);owners,needed=self.scope(star)
            base=self.points[v].copy();neighbours=sorted({z for c in star for z in c['vertices']}-{v})
            scale=float(np.linalg.norm(self.points[neighbours]-base,axis=1).min())
            volume_scale=np.array([abs(t.geometry(self.points,c)['volume']) for c in star])
            if scale<=0 or np.any(volume_scale<=0):raise ValueError('InvalidRepairInputScale')
            cache={}
            def values(z):
                if time.perf_counter()>=deadline:raise BudgetExpired()
                key=tuple(z)
                if key not in cache:
                    try:
                        self.points[v]=base+scale*z
                        cache[key]=self.constraints(star,owners,needed,volume_scale)
                    finally:self.points[v]=base
                return cache[key]
            try:before=values(np.zeros(3))
            except BudgetExpired:complete=False;self.pending=todo[cursor:];break
            if np.all(before>=0):continue
            tick=time.perf_counter();q_before=self.stock(star,owners,needed);bad_before=raw_failures(q_before)
            if not bad_before:continue
            passing=before>=0
            # Start feasible for the auxiliary problem. Increase its minimum
            # slack while retaining every currently passing modeled guard.
            def constraints(y):
                c=values(y[:3]);return np.r_[c-y[3],c[passing]]
            try:
                fit=minimize(lambda y:-float(y[3]),np.r_[np.zeros(3),before.min()],method='SLSQP',
                             bounds=[(-1,1)]*3+[(None,None)],constraints=[dict(type='ineq',fun=constraints)],
                             options=dict(maxiter=maxiter,ftol=ftol))
                after=values(fit.x[:3]);target=base+scale*fit.x[:3]
            except BudgetExpired:complete=False;self.pending=todo[cursor:];break
            adopted=False;reason=None;q_after=None;witness=None
            if np.isfinite(target).all() and np.all(after[0:3*len(star):3]>0):
                try:
                    witness=preserve_step(self.points,star,v,target,self.fixed)
                    try:
                        self.points[v]=target;q_after=self.stock(star,owners,needed)
                    finally:self.points[v]=base
                    bad_after=raw_failures(q_after)
                    adopted=bool(bad_after<=bad_before and (len(bad_after)<len(bad_before) or after.min()>before.min()))
                    if not adopted:reason='NoStrictGuardedImprovement'
                except ValueError as e:reason=str(e)
            else:reason='InvalidProposal'
            if adopted:
                self.points[v]=target;self.edits.append(witness);self.update_reports(star,owners,q_after)
                for c in star:self.geometry.pop(c['id'],None)
                accepted+=1
            attempts+=1
            self.trials.append(dict(vertex=v,adopted=adopted,rejection=reason,solver_success=bool(fit.success),
                minimum_before=float(before.min()),minimum_after=float(after.min()),
                raw_before=len(bad_before),raw_after=(None if q_after is None else len(raw_failures(q_after))),
                evaluations=len(cache),seconds=time.perf_counter()-tick))
            if attempts%50==0:
                print(json.dumps(dict(stage='strict_slack',attempts=attempts,accepted=accepted,
                    remaining=dict(Counter(r['kind'] for r in self.reports.values())),seconds=time.perf_counter()-start)),flush=True)
        result=dict(complete=complete,attempts=attempts,accepted=accepted,
                    remaining=dict(Counter(r['kind'] for r in self.reports.values())),seconds=time.perf_counter()-start)
        self.sweeps.append(result);return result

    def run(self,*,seconds,maxiter,ftol):
        start=time.perf_counter()
        while self.reports:
            if self.pending is None:before=len(self.reports);self.refresh_stars()
            result=self.sweep(seconds=seconds,maxiter=maxiter,ftol=ftol)
            # A partial batch is continuation of this same in-memory search.
            # A complete non-improving pass needs a different local model.
            if result['complete'] and len(self.reports)>=before:break
        preservation=verify_edits(self.original,self.points,self.cells,self.fixed,self.edits)
        return dict(seconds=time.perf_counter()-start,attempts=len(self.trials),edits=len(self.edits),
                remaining=dict(Counter(r['kind'] for r in self.reports.values())),sweeps=self.sweeps,
                    relative_preservation=preservation,trials=self.trials,witnesses=self.edits)
