"""Finite scalar-schedule proposals for the existing V4 preserved-cap contract.

No geometric authority, trajectory or topology changes. Cap screening is only
necessary; a numerical proposal still needs source, collision and V4 admission.
"""
from dataclasses import dataclass,asdict
from pathlib import Path
import json,time
import numpy as np
from collision_policy import digest,state_identity


@dataclass(frozen=True)
class Policy:
    max_audits:int=6
    max_seconds:float=900.
    def __post_init__(self):
        if type(self.max_audits) is not int or self.max_audits<=0 or not np.isfinite(self.max_seconds) or self.max_seconds<=0:
            raise ValueError('InvalidCapPlanningBudget')


def contract():
    from finite_research.direct_fill_contract import TARGETS
    from finite_support.transition_front_targets import triangle_ratio
    return TARGETS['outer_triangles'],triangle_ratio


def cap_report(state,h):
    from finite_support.transition_front_targets import population_report
    from finite_research.direct_fill_contract import classify
    target,metric=contract()
    with np.errstate(invalid='ignore',divide='ignore',over='ignore'):
        scores=metric(state.positions(h)[-1],state.faces)
    valid=np.isfinite(scores)&(scores>0)
    bad=~valid|(scores<target['minimum'])
    finite=np.isfinite(scores).all()
    return dict(passed=bool(len(scores)>0 and not bad.any()),minimum=float(scores.min()) if finite and len(scores) else None,below_minimum=int(bad.sum()),
        failed_faces=np.flatnonzero(bad).tolist(),quantiles=np.quantile(scores,[0,.01,.05,.5,.95,1]).tolist() if finite and len(scores) else None,
        population=classify(population_report(scores,target),target) if valid.all() and len(scores) else None,
        invalid_count=int((~valid).sum()),
        count=len(scores),required_minimum=target['minimum'],percentage_targets_blocking=False)


def bounds(planner,h):
    # Existing skew.apply_field permits row2 down to H1; base layer2 permits
    # growth to nominal row3. Existing independent-late family (start>=3)
    # permits .05*nominal through the incoming schedule. No arbitrary floor.
    low=np.minimum(h,.05*planner.request.nominal[None,:])
    high=h.copy();low[:,0]=high[:,0]=planner.request.first_height
    low[:,1]=np.minimum(h[:,1],planner.request.first_height)
    high[:,1]=np.maximum(h[:,1],planner.request.nominal[2])
    return low,high


def family(planner,state,h,failed_faces):
    """New finite hybrid policy using V3-derived scalar bounds and linear tapers.

    Rows 2..N may change; X1 is immutable. Later increments only compress.
    Five supports, 39 scales and three row2 modes are new orchestration; the
    constant global field uses the same scalar schedule coordinates.
    """
    seeds=np.unique(state.faces[failed_faces])
    _,neighbors=planner.base.build_incidence(state.faces,len(state.wall))
    dist=np.full(len(state.wall),np.inf);dist[seeds]=0.;front=set(map(int,seeds))
    for ring in range(1,17):
        front={int(v) for u in front for v in neighbors[u] if dist[v]==np.inf}
        if not front:break
        dist[list(front)]=ring
    low,high=bounds(planner,h)
    for radius in (2,4,8,16,None):
        support=np.ones(len(h)) if radius is None else np.maximum(0.,1.-dist/radius)
        for scale in np.linspace(.05,1.,39):
            factor=1.-(1.-scale)*support
            for mode in ('retain','lower','upper'):
                q=h.copy();q[:,2:]=np.maximum(low[:,2:],h[:,2:]*factor[:,None])
                target=h[:,1] if mode=='retain' else (low[:,1] if mode=='lower' else high[:,1])
                q[:,1]=h[:,1]+support*(target-h[:,1])
                q=np.minimum(high,np.maximum(low,q))
                yield q,dict(radius_rings=radius,scale=float(scale),row2_mode=mode)


def select(planner,state,h,folder,*,policy=Policy()):
    """Return an independently audited numerical proposal or the unchanged input.

    All finite cap candidates are screened. At most max_audits of the largest
    area-weighted retained schedules get full unchanged B/C/wedge/weight/skew
    audits. A budget refusal does not mean the entire admissible family fails.
    """
    from planner import jsonable
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    h=np.asarray(h,dtype='<f8').copy();started=time.monotonic();binding=state_identity(state)
    low,high=bounds(planner,h)
    baseline=cap_report(state,h)
    report=dict(schema='V35CapPlanning/1',policy=asdict(policy),baseline=baseline,input_schedule=digest(h),
        state_identity=binding,allowed_rows=list(range(2,h.shape[1]+1)),first_front_fixed=True,
        lower_bound_sources=['skew.apply_field: row2 H1 floor','base._select_independent_late_candidate: rows3+ .05 nominal'],
        upper_bound_sources=['base.select_layer2_component: row2 nominal row3','rows3+ incoming schedule'],
        candidate_order='descending wall-area-weighted retained scalar thickness, stable enumeration tie break',
        candidate_count=0,cap_compatible_count=0,history=[],numerical_proposal_passed=False,
        source_global_proof='pending',mesh_accepted=False)
    np.savez_compressed(folder/'INPUT.npz',schedule=h,lower=low,upper=high)
    def save():
        if state_identity(state)!=binding:raise ValueError('CapPlanningStateChanged')
        report['seconds']=time.monotonic()-started
        (folder/'REPORT.json').write_text(json.dumps(report,indent=2,default=jsonable,allow_nan=False)+'\n')
    if baseline['passed']:
        audit=planner.audit(state,h)
        passed=bool(audit['numerical_feasible'] and time.monotonic()-started<=policy.max_seconds)
        report.update(outcome='CompatibleCapNoop' if passed else 'CompatibleCapNumericalOrBudgetRefusal',
            numerical_proposal_passed=passed,baseline_numerical=audit,final=baseline,final_schedule=digest(h))
        save();return h,report
    tri=state.wall[state.faces];area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/6
    weights=np.zeros(len(h))
    for k in range(3):np.add.at(weights,state.faces[:,k],area)
    candidates=[];seen={digest(h)};screen=[]
    for q,description in family(planner,state,h,baseline['failed_faces']):
        if time.monotonic()-started>=policy.max_seconds:
            report['screen_truncated']=True;break
        key=digest(q)
        if key in seen:continue
        seen.add(key)
        if not np.isfinite(q).all() or np.any(q<low) or np.any(q>high) or not np.array_equal(q[:,0],h[:,0]):
            raise ValueError('CapPlanningBoundsViolation')
        cap=cap_report(state,q)
        row=dict(index=len(screen),schedule=key,**description,cap=cap)
        # Failure face identities are fully available for each selected operand;
        # the screen stores counts/distributions to bound diagnostic size.
        row['cap']={k:v for k,v in cap.items() if k!='failed_faces'}
        screen.append(row)
        if cap['passed']:candidates.append((float(np.dot(weights,q.sum(1))),row['index'],q))
    report.update(candidate_count=len(screen),cap_compatible_count=len(candidates))
    (folder/'SCREEN.json').write_text(json.dumps(screen,indent=2,allow_nan=False)+'\n')
    result=h;report['outcome']='CapFamilyExhausted' if not candidates else 'CapNumericalBudgetExhausted'
    if report.get('screen_truncated'):report['outcome']='CapScreenBudgetExhausted'
    for retained,index,q in sorted(candidates,key=lambda r:(-r[0],r[1])):
        if len(report['history'])>=policy.max_audits or time.monotonic()-started>=policy.max_seconds:break
        row=dict(screen_index=index,schedule=digest(q),weighted_retained=retained)
        report['history'].append(row);save()
        np.savez_compressed(folder/('CANDIDATE_%02d.npz'%len(report['history'])),schedule=q)
        tick=time.monotonic();audit=planner.audit(state,q)
        row.update(audit=audit,seconds=time.monotonic()-tick)
        if time.monotonic()-started>policy.max_seconds:
            row['outcome']='CapPlanningTimeBudget';save();break
        if audit['numerical_feasible']:
            row['outcome']='NumericalCapProposal';result=q
            report.update(outcome='NumericalCapProposal',numerical_proposal_passed=True)
            save();break
        row['outcome']='OtherNumericalGateRefusal';save()
    if (len(report['history'])==len(candidates) and candidates and not report['numerical_proposal_passed']
        and not report.get('screen_truncated') and time.monotonic()-started<=policy.max_seconds):
        report['outcome']='CapFiniteFamilyNumericalRefusal'
    report.update(final=cap_report(state,result),final_schedule=digest(result),
        fully_audited=len(report['history']),remaining_cap_candidates=len(candidates)-len(report['history']))
    np.savez_compressed(folder/'RESULT.npz',schedule=result)
    save();return result,report
