"""Bounded front correction kernels and explicit local schedules.

Reuses V4 projection, dependency closure, gates and certificate adapters.
Native callers inject complete validation; historical callers retain the original validator.
"""
import sys,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import coupled_tangent_study as c
q=c.q;np=c.np;w=c.w;b=c.b
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import spsolve
from scipy.optimize import brentq
OUT=ROOT/'results/general_local_front_correction'
def save(name,value):q.write(OUT/name,w.clean(value))

def harmonic_weights(top,centres,active):
    """Graph-harmonic taper: fixed value 1 on failure vertices, 0 outside.

    This is ordinary Dirichlet Laplacian smoothing, not a mesh-validity rule.
    """
    centres=set(map(int,centres));active=np.array(sorted(set(map(int,active))),int)
    unknown=[v for v in active if v not in centres];index={v:i for i,v in enumerate(unknown)}
    data=[];rr=[];cc=[];rhs=np.zeros(len(unknown));weights=np.zeros(len(top.vertex_neighbors.offsets)-1);weights[list(centres)]=1.
    for v,i in index.items():
        neighbors=top.vertex_neighbors.row(v);rr.append(i);cc.append(i);data.append(float(len(neighbors)))
        for n in neighbors:
            if n in index:rr.append(i);cc.append(index[n]);data.append(-1.)
            elif n in centres:rhs[i]+=1.
    if unknown:weights[unknown]=spsolve(csr_matrix((data,(rr,cc)),shape=(len(unknown),len(unknown))),rhs)
    if not np.isfinite(weights).all():raise ValueError('InvalidHarmonicSupport')
    return weights

def target_field(previous,nominal,weights,factor,layer):
    if layer==1:
        if factor!=1.:raise ValueError('FirstLayerAuthorityProtected')
        return np.broadcast_to(nominal,np.shape(previous)).copy()
    if not 0.<factor<=1.:raise ValueError('NonpositiveOrExpandedCompressionFactor')
    return previous+(nominal-previous)*(1-(1-factor)*weights)

def project_field(low,targets,sdf,spec,ancestry,active,nominal):
    """Vary target only on declared support; source branch remains explicit."""
    high=nominal.copy();active=np.asarray(active,int)
    if spec['kind']=='spheres' and not spec.get('cavity'):
        for value in np.unique(targets[active]):
            ids=active[targets[active]==value];high[ids],_=b.sphere_project(low[ids],float(value),sdf,spec,ancestry[ids])
    else:high[active]=q.predict_next_front(low[active],targets[active],sdf,iteration_cap=256).coordinates
    z=sdf.evaluate(high)
    if not np.all(abs(z.phi-targets)<=z.source_error_bound):raise ValueError('AdjustedTargetAdmissionFailure')
    return high

def local_schedule(schedule,committed_targets,adjusted_next,*,recover=False):
    """Explicit vertex schedules; geometric recovery is a proposal, not proof.

    Recover by scaling future nominal increments with one common multiplier.
    This preserves their relative growth and reports the new joining ratio.
    No recovery is committed until immediate geometry is validated.
    """
    k=len(committed_targets);nv=len(adjusted_next)
    if k<1:raise ValueError('FirstLayerAuthorityProtected')
    if k>=len(schedule.increments):raise ValueError('LayerScheduleComplete')
    h=np.broadcast_to(schedule.increments,(nv,20)).copy();targets=np.asarray(committed_targets).T
    if not np.array_equal(targets[:,0],np.full(nv,schedule.targets[0])):raise ValueError('FirstLayerAuthorityProtected')
    h[:,:k]=np.diff(np.c_[np.zeros(nv),targets],axis=1)
    # Preserve stored binary64 increments when both bounding targets are nominal.
    # Subtracting cumulative floats otherwise invents a layer-2 adjustment.
    bounds=np.c_[np.ones(nv,dtype=bool),targets==schedule.targets[:k]]
    unchanged=bounds[:,:k]&bounds[:,1:k+1]
    h[:,:k]=np.where(unchanged,schedule.increments[:k],h[:,:k])
    nominal_next=(targets[:,-1]==schedule.targets[k-1])&(adjusted_next==schedule.targets[k])
    h[:,k]=np.where(nominal_next,schedule.increments[k],adjusted_next-targets[:,-1])
    if np.any(h[:,k]<=0):raise ValueError('NonpositiveAdjustedLayer')
    loss=schedule.targets[k]-adjusted_next;recovery=np.zeros(nv)
    if recover and k+1<20:
        future=float(np.sum(schedule.increments[k+1:]));h[:,k+1:] *= (1+loss/future)[:,None];recovery=loss.copy()
    cumulative=np.cumsum(h,axis=1)
    assert np.array_equal(h[:,0],np.full(nv,schedule.increments[0]))
    return dict(heights=h,cumulative_offsets=cumulative,nominal_cumulative=schedule.targets,nominal_next_height=float(schedule.increments[k]),adjusted_next_height=h[:,k],requested_total=float(schedule.targets[-1]),planned_total=cumulative[:,-1],proposed_recovery=recovery,unrecovered=float(schedule.targets[-1])-cumulative[:,-1],growth=h[:,1:]/h[:,:-1],changed_layers=np.flatnonzero(np.any(h!=schedule.increments,axis=0))+1,recovery_qualified=False)

def geometric_recovery(height,count,demand):
    """Standard geometric progression fit using installed SciPy Brent solver.

    Diagnostic alternative to proportional redistribution. No growth gate.
    """
    if height<=0 or count<1 or demand<=0:raise ValueError('InvalidRecoveryDemand')
    def total(r):return height*sum(r**j for j in range(1,count+1))-demand
    hi=1.
    for _ in range(64):
        if total(hi)>=0:break
        hi*=2
    else:raise ValueError('RecoveryBracketUnavailable')
    r=brentq(total,0.,hi);return dict(ratio=r,heights=[height*r**j for j in range(1,count+1)],scope='Unvalidated future proposal; not capacity authority')

def support_check(case,prefix=None,verify=None):
    from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import rational_vector,cross,sub
    prefix=case.prefix if prefix is None else prefix;verify=case.verify if verify is None else verify
    low=prefix[-1] if prefix else case.x;bad=[]
    for col in verify.columns:
        vv=[rational_vector(v) for v in low[case.f[col]]];area=cross(sub(vv[1],vv[0]),sub(vv[2],vv[0]))
        if not any(area):bad.append(int(col))
    owners=all(len(case.top.edge_triangles.row(e))==2 for e in verify.side_edges)
    return dict(passed=bool(np.isfinite(low[verify.vertices]).all() and not bad and owners),zero_fixed_triangles=bad,complete_ownership=owners,topology_hash=case.top.sha256,scope='All metric-dependency columns, including fixed halo')

def search_compression(case,*,prefix=None,previous=None,weights=None,nominal=None,previous_targets=None,iterations=10,validator=None,record_gate=None):
    """Dyadic bracket then scalar bisection, retaining only fully verified points.

    No feasibility monotonicity is assumed outside the sampled bracket. The
    result is least compression found in this bounded scalar family, not a
    global minimum or a universal minimum acceptable height.
    """
    prefix=case.prefix if prefix is None else prefix;k=len(prefix);low=prefix[-1] if prefix else case.x
    if k==0:raise ValueError('FirstLayerAuthorityProtected')
    if not support_check(case,prefix)['passed']:raise ValueError('InvalidFixedSupport')
    previous=case.previous if previous is None else previous
    active=case.active;weights=harmonic_weights(case.top,case.support['controlling_vertices'],active) if weights is None else weights
    target=float(case.s.targets[k]);prev=np.full(len(low),float(case.s.targets[k-1])) if previous_targets is None else previous_targets
    if nominal is None:nominal=q.predict_next_front(low,target,case.sdf,iteration_cap=256).coordinates
    ancestry=(case.ancestry if hasattr(case,'ancestry') else b.owner(case.x,case.spec));records=[];timing=dict(projection_seconds=0.,verification_seconds=0.,certificate_seconds=0.);good=None;last_bad=1.
    def evaluate(factor):
        nonlocal good
        row=dict(factor=float(factor),layer=k+1)
        progress=getattr(case,'search_progress',None)
        if progress:progress('trial_start',stage='CompressionCorrection',factor=float(factor))
        try:
            targets=target_field(prev,target,weights,float(factor),k+1);tick=time.perf_counter();high=project_field(low,targets,case.sdf,case.spec,ancestry,active,nominal);timing['projection_seconds']+=time.perf_counter()-tick
            case.evaluation_targets=targets
            tick=time.perf_counter();gate,metrics,ct=(validator or w.checked)(case.x,case.f,case.top,case.verify,prefix,high,previous);timing['verification_seconds']+=time.perf_counter()-tick;timing['certificate_seconds']+=ct
            row.update(gate=record_gate(gate) if record_gate else gate,passed=gate['passed'],coordinate_hash=q.array_hash(high),target_hash=q.array_hash(targets),minimum_height=float(min(targets-prev)),maximum_compression=float(max(1-(targets-prev)/(target-prev))))
            if gate['passed']:good=(high,targets,gate,metrics,float(factor))
        except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:row.update(passed=False,error=str(exc))
        records.append(row)
        if progress:progress('trial_end',stage='CompressionCorrection',factor=float(factor),passed=row['passed'])
        return row['passed']
    for j in range(1,iterations+1):
        factor=2.**-j
        if evaluate(factor):
            lo=factor;hi=last_bad
            for _ in range(12):
                mid=(lo+hi)/2
                if evaluate(mid):lo=mid
                else:hi=mid
            break
        last_bad=factor
    return good,records,timing,weights

def plan_future(schedule,committed_targets,adjusted_next,horizon,*,recover=False):
    """Bounded uncommitted schedule view. Applying a plan never commits it."""
    k=len(committed_targets)
    if not 1<=horizon<=20-k:raise ValueError('InvalidPlanningHorizon')
    state=local_schedule(schedule,committed_targets,adjusted_next,recover=recover)
    return dict(first_layer=k+1,layers=list(range(k+1,k+1+horizon)),
                targets=state['cumulative_offsets'][:,k:k+horizon].T,
                heights=state['heights'][:,k:k+horizon].T,
                reason='Current correction plus the next recovery join',
                committed=False,recovery_qualified=False)

def geometric_plan(schedule,committed_targets,adjusted_next,horizon):
    """Smooth geometric recovery proposal; exact acceptance remains external."""
    state=local_schedule(schedule,committed_targets,adjusted_next)
    k=len(committed_targets);heights=state['heights'].copy();ratios=np.ones(len(adjusted_next))
    if not 1<=horizon<=20-k:raise ValueError('InvalidPlanningHorizon')
    for v in np.flatnonzero(adjusted_next!=schedule.targets[k]):
        if k+1<20:
            fit=geometric_recovery(float(heights[v,k]),19-k,float(schedule.targets[-1]-adjusted_next[v]))
            heights[v,k+1:]=fit['heights'];ratios[v]=fit['ratio']
    offsets=np.cumsum(heights,axis=1)
    return dict(layers=list(range(k+1,k+1+horizon)),targets=offsets[:,k:k+horizon].T,
                heights=heights,cumulative_offsets=offsets,ratios=ratios,
                planned_total=offsets[:,-1],unrecovered=schedule.targets[-1]-offsets[:,-1],
                committed=False,recovery_qualified=False,reason='Two-layer diagnostic of smooth recovery join; later geometry remains untested')

def tangent_search(case,prefix,previous,high,validator=None,record_gate=None):
    """Existing deterministic Laplacian/spring fields, fixed SDF targets."""
    from types import SimpleNamespace
    low=prefix[-1] if prefix else case.x;k=len(prefix);history=[]
    modes,_=w.search_fields(SimpleNamespace(coordinates=case.x),case.top,case.patch,low,float(case.s.targets[-1]))
    for mode,direction in modes:
        for scale in (.5,1.,.25,.125,.0625,.03125,2.,4.):
            progress=getattr(case,'search_progress',None)
            if progress:progress('trial_start',stage='TangentialCorrection',mode=mode,scale=scale)
            try:
                trial=high.copy();trial[case.active]=w.tangent_trial(high[case.active],direction,scale*case.s.increments[k],case.sdf,float(case.s.targets[k]))
                gate,metric,_=(validator or w.checked)(case.x,case.f,case.top,case.verify,prefix,trial,previous)
                history.append(dict(mode=mode,scale=scale,gate=record_gate(gate) if record_gate else gate,coordinate_hash=q.array_hash(trial)))
                if progress:progress('trial_end',stage='TangentialCorrection',mode=mode,scale=scale,passed=gate['passed'])
                if gate['passed']:return (trial,gate,metric),history
            except (RuntimeError,ValueError) as exc:history.append(dict(mode=mode,scale=scale,error=str(exc)))
    return None,history
