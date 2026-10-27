"""Private receding-horizon study. No production imports or authority updates."""
import json,time,hashlib,sys
from pathlib import Path
import general_local_correction as g
q=g.q;np=g.np;c=g.c;b=g.b;w=g.w
OUT=g.ROOT/'results/compressed_layer_recovery'
def save(name,value):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(w.clean(value),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n',newline='\n')
def load(p):return json.loads(p.read_text())

def seed():
    case=c.Case(q.definitions(q.load_schedule().targets)[-1]);folder=g.OUT/case.name;r=load(folder/'result.json')
    high=np.load(folder/'candidate.npy');targets=np.load(folder/'targets.npy')
    assert q.array_hash(high)==r['coordinate_hash'] and q.array_hash(targets)==r['target_hash']
    nominal=q.predict_next_front(case.low,case.target,case.sdf,iteration_cap=256).coordinates
    replay=g.project_field(case.low,targets,case.sdf,case.spec,b.owner(case.x,case.spec),case.active,nominal)
    assert np.array_equal(high,replay)
    gate,metric,ct=w.checked(case.x,case.f,case.top,case.verify,case.prefix,high,case.previous)
    assert gate['passed'] and q.identity(gate)==q.identity(r['accepted_gate'])
    history=[np.full(len(high),v) for v in case.s.targets[:3]]+[targets]
    return case,case.prefix+[high],history,metric['cells'],dict(coordinate_hash=q.array_hash(high),target_hash=q.array_hash(targets),gate=gate,branch_replay=True,certificate_seconds=ct)

def plan(schedule,history,strategy,horizon,weights):
    k=len(history);actual=history[-1];last=history[-1]-history[-2];nom=float(schedule.increments[k]);debt=schedule.targets[k-1]-actual
    if strategy=='immediate':height=np.full(len(actual),nom)
    elif strategy=='uniform':height=nom+debt/horizon
    elif strategy in ('smooth','neighbor','receding'):
        height=last+(nom-last)/horizon
        if strategy=='neighbor':
            centre=float(np.min(height[weights==1]));height=nom+(centre-nom)*weights
    else:raise ValueError('UnknownRecoveryStrategy')
    targets=actual+height;targets[weights==0]=schedule.targets[k]
    provisional=[targets.copy()];increments=[targets-actual];cursor=targets.copy();last_height=targets-actual
    for j in range(1,min(horizon,len(schedule.increments)-k)):
        following=float(schedule.increments[k+j])
        if strategy=='immediate':step=np.full(len(actual),following)
        elif strategy=='uniform':step=following+debt/horizon
        else:
            step=last_height+(following-last_height)/(horizon-j)
            if strategy=='neighbor':step=following+(float(min(step[weights==1]))-following)*weights
        next_target=cursor+step;next_target[weights==0]=schedule.targets[k+j]
        increments.append(next_target-cursor);provisional.append(next_target);last_height=next_target-cursor;cursor=next_target
    return dict(strategy=strategy,horizon=horizon,layer=k+1,height=targets-actual,targets=targets,
                provisional_targets=np.array(provisional),provisional_increments=np.array(increments),
                remaining_debt=debt,committed=False,scope='Only next entry eligible for validation; reforecast after acceptance')

def project(case,low,targets,active,tangent=None):
    tick=time.perf_counter();points=low.copy();displacement=np.zeros_like(low)
    if tangent is not None:displacement[active]=tangent;points+=displacement
    # Analytic source-branch projection, not Newton with a larger cap.
    high=points.copy();ancestry=b.owner(case.x,case.spec);records=[]
    for target in np.unique(targets):
        ids=np.flatnonzero(targets==target);high[ids],rr=b.sphere_project(points[ids],float(target),case.sdf,case.spec,ancestry[ids]);records.extend(rr)
    sample=case.sdf.evaluate(high)
    if not np.all(abs(sample.phi-targets)<=sample.source_error_bound):raise ValueError('LocalTargetAdmissionFailure')
    return high,dict(target_hash=q.array_hash(targets),residual_max=float(max(abs(sample.phi-targets))),
                     ownership_before=b.owner(low,case.spec),ownership_after=b.owner(high,case.spec),
                     source_ancestry=ancestry,analytic_projection=True,newton_iterations=0,
                     max_tangent_displacement=float(np.linalg.norm(displacement,axis=1).max())),time.perf_counter()-tick

def evaluate(case,prefix,previous,targets,active,tangent=None):
    timing=dict(projection=0.,hard_gates=0.,certificate=0.);tick=time.perf_counter()
    try:high,projection,pt=project(case,prefix[-1],targets,active,tangent);timing['projection']=pt
    except (ValueError,RuntimeError) as exc:return None,dict(passed=False,projection_failure=str(exc)),None,timing
    vt=time.perf_counter();gate,m,ct=w.checked(case.x,case.f,case.top,case.verify,prefix,high,previous)
    timing.update(hard_gates=time.perf_counter()-vt-ct,certificate=ct)
    return high,dict(passed=gate['passed'],gate=gate,projection=projection,coordinate_hash=q.array_hash(high)),m,timing

def accounting(schedule,history,initial_loss):
    k=len(history);actual=history[-1];height=actual-history[-2];debt=schedule.targets[k-1]-actual
    percentage=[float(100*(loss-d)/loss) if loss!=0 else None for loss,d in zip(initial_loss,debt)]
    return dict(layer=k,nominal_cumulative=float(schedule.targets[k-1]),actual_cumulative=actual,
                nominal_increment=float(schedule.increments[k-1]),actual_increment=height,unrecovered=debt,
                recovered=initial_loss-debt,recovery_percentage=percentage,
                growth=height/(history[-2]-history[-3]),minimum_height=float(min(height)),maximum_unrecovered=float(max(debt)),
                increased_debt=bool(np.any(debt>initial_loss)),scope='Per-vertex offsets; triangle columns retain all three vertex values')

def condition(case,low,high,layer):
    from verify_coupled_tangent_study import conditioning
    old=(case.low,case.k);case.low=low;case.k=layer-1
    try:return conditioning(case,high)
    finally:case.low,case.k=old

def neighbors(case,targets,heights):
    e=case.top.edges[case.verify.side_edges]
    return dict(maximum_target_difference=float(max(abs(targets[e[:,0]]-targets[e[:,1]]))),
                maximum_increment_difference=float(max(abs(heights[e[:,0]]-heights[e[:,1]]))),hard_smoothness_threshold=False)

def tangent_trials(case,prefix,previous,targets,active):
    from types import SimpleNamespace
    patch=q.bounded_patch(case.top,active)
    modes,_=w.search_fields(SimpleNamespace(coordinates=case.x),case.top,patch,prefix[-1],float(case.s.targets[-1]))
    basis=w.tangent_basis(case.sdf.evaluate(prefix[-1][active]).normal);trials=[];times=[]
    for mode,field in modes:
        tangent=np.einsum('nij,nj->ni',basis,np.einsum('nji,nj->ni',basis,field))
        norm=np.linalg.norm(tangent,axis=1);tangent=np.divide(tangent,norm[:,None],out=np.zeros_like(tangent),where=norm[:,None]!=0)
        for amplitude in (.125,.25,.5,1.,-.125,-.25,-.5,-1.):
            move=tangent*float(case.s.increments[len(prefix)])*amplitude
            high,r,m,t=evaluate(case,prefix,previous,targets,active,move);times.append(t)
            trials.append(dict(mode=mode,amplitude=amplitude,result=r))
            if r['passed']:return (high,r,m),trials,times
    return None,trials,times

def secant_trials(case,prefix,history,previous,active,weights):
    """Continue the preceding valid displacement, then measure/project targets.

    This predictor uses only allowed tangent and normal motion. Actual SDF
    offsets are explicit and must increase. It is not mesh admission.
    """
    k=len(prefix);low=prefix[-1];step=low-prefix[-2];ratio=float(case.s.increments[k]/case.s.increments[k-1]);trials=[];times=[]
    normals=case.sdf.evaluate(low[active]).normal
    for factor in (1.,.75,.5,.25,.125,.0625,.03125,.015625):
        move=step[active]*(ratio*(1-(1-factor)*weights[active]))[:,None]
        sample=case.sdf.evaluate(low[active]+move);targets=np.full(len(low),case.s.targets[k]);targets[active]=sample.phi
        if np.any(targets<=history[-1]):
            trials.append(dict(factor=factor,rejected='NonpositiveMeasuredHeight',vertices=np.flatnonzero(targets<=history[-1])));continue
        high,r,m,t=evaluate(case,prefix,previous,targets,active,move);times.append(t)
        tangent=move-normals*np.sum(move*normals,axis=1)[:,None]
        if 'projection' in r:r['projection']['max_tangent_displacement']=float(np.linalg.norm(tangent,axis=1).max())
        trials.append(dict(factor=factor,result=r,targets=targets,predictor='Previous-valid-displacement secant; exact acceptance independent'))
        if r['passed']:return (high,targets,r,m,factor),trials,times
    return None,trials,times

def run(depth=8,replay=False):
    tick=time.perf_counter();case,prefix,history,previous,sr=seed();sr.pop('certificate_seconds');initial_loss=case.s.targets[3]-history[-1]
    active=case.active.copy();centres=set(case.support['controlling_vertices']);weights=g.harmonic_weights(case.top,centres,active)
    comparison=[];times=[];plan_seconds=0.;tangent_seconds=0.;layer_times=[]
    for strategy in ('immediate','uniform','smooth','neighbor'):
        for horizon in ((1,) if strategy=='immediate' else (2,3,4,5)):
            pt=time.perf_counter();p=plan(case.s,history,strategy,horizon,weights);plan_seconds+=time.perf_counter()-pt
            _,r,_,t=evaluate(case,prefix,previous,p['targets'],active)
            comparison.append(dict(strategy=strategy,horizon=horizon,plan=p,result=r));times.append(t)
            print(strategy,horizon,r['passed'],len(r.get('gate',{}).get('exact_failures',[])),flush=True)
    layers=[];expansions=[]
    while len(prefix)<depth:
        k=len(prefix);attempts=[];accepted=None;layer_tick=time.perf_counter()
        # Finite scalar continuation search; no minimum physical-height gate.
        for expansion in range(2):
            for horizon in (2,3,4,5):
                pt=time.perf_counter();p=plan(case.s,history,'receding',horizon,weights);plan_seconds+=time.perf_counter()-pt
                for factor in (1.,.5,.25,.125,.0625,.03125,.015625,.0078125,.00390625):
                    targets=history[-1]+p['height']*(1-(1-factor)*weights);targets[weights==0]=case.s.targets[k]
                    high,r,m,t=evaluate(case,prefix,previous,targets,active);times.append(t)
                    attempts.append(dict(horizon=horizon,factor=factor,expansion=expansion,result=r,target_hash=q.array_hash(targets)))
                    if r['passed']:accepted=(high,targets,r,m,horizon,factor);break
                if accepted:break
            if accepted:break
            # Add all controlling upper vertices and one neighbor ring. Full
            # scientific scope already covers all fixture cells and relations.
            ids=set(centres)
            for a in attempts:
                for e in a['result'].get('gate',{}).get('exact_failures',[]):ids.update(map(int,case.f[e['column']]))
                for e in a['result'].get('gate',{}).get('failures',[]):ids.update(map(int,case.f[e['columns']].ravel()))
            enlarged=set(ids)
            for v in ids:enlarged.update(map(int,case.top.vertex_neighbors.row(v)))
            if enlarged<=set(active):break
            if len(enlarged)>2048:break
            active=np.array(sorted(enlarged));centres=ids;weights=g.harmonic_weights(case.top,centres,active)
            expansions.append(dict(layer=k+1,active=active,centres=sorted(centres),reason='Controlling rejected-candidate cells and metric owners plus one ring'))
        tangent_history=[]
        if accepted is None:
            for horizon,factor in ((5,.125),(5,.25),(2,.125)):
                p=plan(case.s,history,'receding',horizon,weights)
                targets=history[-1]+p['height']*(1-(1-factor)*weights);targets[weights==0]=case.s.targets[k]
                st=time.perf_counter();candidate,trials,tt=tangent_trials(case,prefix,previous,targets,active);tangent_seconds+=time.perf_counter()-st;times+=tt
                tangent_history.append(dict(horizon=horizon,factor=factor,trials=trials))
                if candidate:
                    high,r,m=candidate;accepted=(high,targets,r,m,horizon,factor);break
        secant_history=[]
        if accepted is None:
            layer_times.append(dict(layer=k+1,seconds=time.perf_counter()-layer_tick,committed=False))
            candidate,secant_history,tt=secant_trials(case,prefix,history,previous,active,weights);times+=tt
            if candidate:
                high,targets,r,m,factor=candidate;accepted=(high,targets,r,m,2,factor)
        if accepted is None:
            layers.append(dict(layer=k+1,committed=False,attempts=attempts,tangent_trials=tangent_history,secant_trials=secant_history));print('STOP',k+1,flush=True);break
        high,targets,r,m,horizon,factor=accepted
        diag=condition(case,prefix[-1],high,k+1);prefix.append(high);history.append(targets);previous=m['cells']
        record=dict(layer=k+1,committed=True,attempts=attempts,tangent_trials=tangent_history,secant_trials=secant_history,accepted=r,horizon=horizon,factor=factor,
                    accounting=accounting(case.s,history,initial_loss),conditioning=diag,
                    neighbors=neighbors(case,targets,history[-1]-history[-2]),active=active)
        layers.append(record);print('COMMIT',k+1,'factor',factor,'height',record['accounting']['minimum_height'],flush=True)
        layer_times.append(dict(layer=k+1,seconds=time.perf_counter()-layer_tick,committed=True))
        if not replay:
            save(f'checkpoint_{k+1:02}.json',record);np.save(OUT/f'front_{k+1:02}.npy',high);np.save(OUT/f'targets_{k+1:02}.npy',targets)
    result=w.clean(dict(seed=sr,comparison=comparison,layers=layers,expansions=expansions,last_valid=len(prefix),requested_study_depth=depth,
                        production_modified=False,termination_candidate=False,infeasibility_proved=False))
    if replay:
        old=load(OUT/'result.json');equal=q.identity(old)==q.identity(result);save('DETERMINISM.json',dict(equal=equal,first=q.identity(old),replay=q.identity(result)));assert equal
    else:save('result.json',result)
    save(('REPLAY_' if replay else '')+'TIMING.json',dict(seconds=time.perf_counter()-tick,recovery_plan_seconds=plan_seconds,tangent_inclusive_seconds=tangent_seconds,layer_times=layer_times,stages=times,peak_rss=q.psutil.Process().memory_info().peak_wset))
    return result
if __name__=='__main__':run(replay='--replay' in sys.argv)
