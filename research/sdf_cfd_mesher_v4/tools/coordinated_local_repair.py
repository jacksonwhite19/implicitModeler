"""Existing local coordinate feasibility extended to connected free supports.

Proposal mathematics uses current stock constraints. Adoption uses raw stock
union QC and sequential exact star-witness composition, never solver flags.
"""
from collections import Counter
import time,json
import numpy as np
from refined_local_repair import RefinedRepair,raw_failures
from background_local_repair import star_index,verify_edits
from coordinated_preservation import compose_group
from contextlib import contextmanager
from contextvars import ContextVar

_work_budget=ContextVar('coordinated_repair_work_budget',default=None)
@contextmanager
def work_budget(check):
    """Call-scoped operational stop; default preserves existing consumers."""
    token=_work_budget.set(check)
    try:yield
    finally:_work_budget.reset(token)


class LastConstraintValue:
    """At most one probe, with the original exact tuple-key semantics."""
    def __init__(self,compute):
        self.compute=compute;self.key=None;self.value=None;self.evaluations=0;self.hits=0

    def __call__(self,z):
        check=_work_budget.get()
        if check is not None:check()
        key=tuple(z)
        if key!=self.key:
            value=self.compute(z)
            self.key=key;self.value=value;self.evaluations+=1
        else:self.hits+=1
        return self.value


class CoordinatedRepair(RefinedRepair):
    def groups(self):
        # A defect's actual owners define its proposal support. Do not join
        # overlapping supports transitively into a global dense solve. Complete
        # incident stars and neighbour QC still guard every local proposal.
        groups=set()
        for r in self.reports.values():
            ids=r['owners'] if r['kind']=='face_quality' else [r['id']]
            free={v for i in set(ids)&self.core_ids for v in self.byid[i]['vertices']}-self.fixed
            if free:groups.add(tuple(sorted(free)))
        return [list(g) for g in sorted(groups,key=lambda g:(len(g),g))]

    def group_trial(self,vertices,stars,*,maxiter,ftol,max_depth,max_attempts,_geometric_start=False):
        from scipy.optimize import minimize
        import local_termination_topology as t
        start=time.perf_counter();union={c['id']:c for v in vertices for c in stars[v]}
        star=[union[i] for i in sorted(union)];owners,needed=self.scope(star)
        base=self.points[vertices].copy();volume_scale=np.array([abs(t.geometry(self.points,c)['volume']) for c in star])
        scales=[]
        for v in vertices:
            neighbours=sorted({z for c in stars[v] for z in c['vertices']}-{v})
            # A short edge can itself be part of the defect. Normalize the
            # proposal by the incident-neighbour median, not that short edge.
            # Fixed boundaries, exact preservation and stock gates constrain
            # the actual movement; this changes no acceptance threshold.
            scales.append(np.median(np.linalg.norm(self.points[neighbours]-self.points[v],axis=1)))
        scales=np.asarray(scales)
        if np.any(scales<=0) or np.any(volume_scale<=0):raise ValueError('InvalidCoordinatedScale')
        def compute(z):
            try:
                self.points[vertices]=base+scales[:,None]*z.reshape(-1,3)
                return self.constraints(star,owners,needed,volume_scale)
            finally:self.points[vertices]=base
        values=LastConstraintValue(compute)
        before=values(np.zeros(3*len(vertices)));q_before=self.stock(star,owners,needed);bad_before=raw_failures(q_before)
        if not bad_before:return dict(vertices=vertices,adopted=False,reason='AlreadyPassing',seconds=time.perf_counter()-start)
        print(json.dumps(dict(stage='group_start',vertices=len(vertices),cells=len(star),
            decision_coordinates=3*len(vertices),constraints=len(before),raw_failures=len(bad_before),
            retained_constraint_vectors=1)),flush=True)
        # Reuse the historical Laplacian proposal first. Sequential neighbour
        # means provide a construction seed; they are never accepted without
        # the same exact preservation and complete raw union guards.
        try:
            for v in vertices:
                neighbours=sorted({z for c in stars[v] for z in c['vertices']}-{v})
                self.points[v]=self.points[neighbours].mean(axis=0)
            seed=self.points[vertices].copy()
        finally:self.points[vertices]=base
        proposals=[];geometric=None;geometric_record=None
        if _geometric_start:
            from background_geometric_starts import placement_seed
            geometric,geometric_record=placement_seed(self.points,vertices,stars,owners,self.byid,self.fixed,scales,_work_budget.get(),maxiter=maxiter)
            if geometric is not None:proposals.append(('geometry_altitude_volume',geometric))
        proposals.append(('laplacian',seed))
        for proposer,goal in proposals:
            adopted=False;composed=None
            after=values(((goal-base)/scales[:,None]).ravel())
            try:
                composed=compose_group(self.points,stars,dict(zip(vertices,goal)),self.fixed,
                    max_depth=max_depth,max_attempts=max_attempts)
                q_after=self.stock(star,owners,needed);bad_after=raw_failures(q_after)
                adopted=bool(bad_after<=bad_before and (len(bad_after)<len(bad_before) or after.min()>before.min()))
            except ValueError:pass
            finally:
                if not adopted:self.points[vertices]=base
                for c in star:self.geometry.pop(c['id'],None)
            if adopted:
                self.edits.extend(composed['witnesses']);self.update_reports(star,owners,q_after)
                return dict(vertices=vertices,cells=len(star),adopted=True,reason=None,proposer=proposer,
                    raw_before=len(bad_before),raw_after=len(bad_after),evaluations=values.evaluations,
                    retained_constraint_vectors=1,constraint_vector_bytes=values.value.nbytes,
                    preservation_steps=len(composed['witnesses']),seconds=time.perf_counter()-start,
                    **({'geometric_start':geometric_record} if _geometric_start else {}))
        passing=before>=0
        def constraints(y):
            c=values(y[:-1]);return np.r_[c-y[-1],c[passing]]
        # Laplacian is a separate guarded proposal. After it is rejected, its
        # clipped coordinates may violate c[passing]. Start the auxiliary
        # max-min problem at base instead: both inequality blocks are feasible
        # by construction, including constraints exactly on a gate.
        initial=np.r_[np.zeros(3*len(vertices)),before.min()]
        geometric_fraction=None
        if geometric is not None:
            delta=((geometric-base)/scales[:,None]).ravel()
            # Reuse bounded dyadic subdivision to retain all base-passing
            # modeled guards. No acceptance tolerance or gate is introduced.
            for depth in range(max_depth+1):
                fraction=2.**(-depth);probe=fraction*delta;c=values(probe)
                if np.isfinite(c).all() and np.all(c[passing]>=0) and c.min()>before.min():
                    initial=np.r_[probe,c.min()];geometric_fraction=fraction;break
        if np.any(constraints(initial)<0):raise ValueError('InfeasibleBaseProposal')
        fit=minimize(lambda y:-float(y[-1]),initial,method='SLSQP',
                     bounds=[(-1,1)]*(3*len(vertices))+[(None,None)],constraints=[dict(type='ineq',fun=constraints)],
                     options=dict(maxiter=maxiter,ftol=ftol))
        after=values(fit.x[:-1]);goal=base+scales[:,None]*fit.x[:-1].reshape(-1,3)
        adopted=False;reason=None;composed=None;q_after=None
        try:
            composed=compose_group(self.points,stars,dict(zip(vertices,goal)),self.fixed,
                                   max_depth=max_depth,max_attempts=max_attempts)
            q_after=self.stock(star,owners,needed);bad_after=raw_failures(q_after)
            adopted=bool(bad_after<=bad_before and (len(bad_after)<len(bad_before) or after.min()>before.min()))
            if not adopted:reason='NoStrictUnionImprovement'
        except ValueError as e:reason=str(e)
        finally:
            if not adopted:self.points[vertices]=base
            # No trial may leave a cached geometry from another group state.
            for c in star:self.geometry.pop(c['id'],None)
        if adopted:
            self.edits.extend(composed['witnesses']);self.update_reports(star,owners,q_after)
        return dict(vertices=vertices,cells=len(star),adopted=adopted,reason=reason,
            solver_success=bool(fit.success),minimum_before=float(before.min()),minimum_after=float(after.min()),
            raw_before=len(bad_before),raw_after=(None if q_after is None else len(raw_failures(q_after))),
            evaluations=values.evaluations,retained_constraint_vectors=1,constraint_vector_bytes=values.value.nbytes,
            preservation_steps=(0 if composed is None else len(composed['witnesses'])),
            seconds=time.perf_counter()-start,
            **({'geometric_start':geometric_record,'geometric_initial_fraction':geometric_fraction} if _geometric_start else {}))

    def coupled_sweep(self,*,maxiter,ftol,max_depth,steps_per_vertex,enrich=False):
        groups=self.groups();wanted=set(v for g in groups for v in g)
        stars=star_index(self.cells,wanted)
        if enrich:
            groups=[sorted(set(g)|{v for u in g for c in stars[u] for v in c['vertices'] if v not in self.fixed}) for g in groups]
            stars=star_index(self.cells,{v for g in groups for v in g})
        records=[]
        for g in groups:
            record=self.group_trial(g,stars,maxiter=maxiter,ftol=ftol,max_depth=max_depth,max_attempts=steps_per_vertex*len(g))
            records.append(record)
            if len(records)%10==0:
                print(json.dumps(dict(stage='coordinated',groups=len(records),accepted=sum(r['adopted'] for r in records),
                    remaining=dict(Counter(r['kind'] for r in self.reports.values())))),flush=True)
        return records

    def run(self,*,maxiter,ftol,max_depth,steps_per_vertex):
        start=time.perf_counter();records=[];sweeps=[];enrich=False
        while self.reports:
            before=len(self.reports)
            current=self.coupled_sweep(maxiter=maxiter,ftol=ftol,max_depth=max_depth,
                                       steps_per_vertex=steps_per_vertex,enrich=enrich)
            records.extend(current);sweeps.append(dict(enriched=enrich,groups=len(current),
                adopted=sum(r['adopted'] for r in current),remaining=dict(Counter(r['kind'] for r in self.reports.values()))))
            if len(self.reports)>=before:
                if enrich:break
                enrich=True
        preservation=verify_edits(self.original,self.points,self.cells,self.fixed,self.edits)
        return dict(seconds=time.perf_counter()-start,edits=len(self.edits),groups=len(records),
                    sweeps=sweeps,remaining=dict(Counter(r['kind'] for r in self.reports.values())),
                    relative_preservation=preservation,trials=records,witnesses=self.edits)
