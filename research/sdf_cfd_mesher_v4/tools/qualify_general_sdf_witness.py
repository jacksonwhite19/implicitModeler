"""Bounded closure continuation and authoritative composite-SDF diagnostics."""
import json,time,math,ast,sys
from pathlib import Path
import numpy as np
from types import SimpleNamespace
from fractions import Fraction
import qualify_capacity_valid_prefix as q
from channel_closure_study import extended_front,ExtendedSDF,exterior_front,ExteriorSDF
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import tangent_basis,search_fields
OUT=q.ROOT/'results/general_sdf_capacity_witness'


def clean(v):
    if isinstance(v,np.ndarray):return clean(v.tolist())
    if isinstance(v,np.generic):return clean(v.item())
    if isinstance(v,float) and not math.isfinite(v):return repr(v)
    if isinstance(v,dict):return {str(k):clean(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [clean(x) for x in v]
    return v

def save(name,value):q.write(OUT/name,clean(value))

def checked(x,f,top,p,prefix,high,previous):
    try:return q.validate(x,f,top,p,prefix,high,previous)
    except (RuntimeError,ValueError) as exc:
        payload=exc.args[0] if exc.args else None
        if not isinstance(payload,tuple) or payload[0]!='InvalidSidePatch':raise
        side=int(payload[1]);edge=side%len(top.edges);k=side//len(top.edges)
        low=x if k==0 else (prefix+[high])[k-1];up=(prefix+[high])[k];a,b=top.edges[edge]
        surface=np.array([low[a],low[b],up[a],up[b]])
        return dict(layer=len(prefix)+1,passed=False,failures=[dict(kind='side',metric='RequiredInteractionSideRegularity',layer=k+1,entity=edge,columns=top.edge_triangles.row(edge).tolist(),value=payload[2])],exact_failures=[],invalid_required_surface=dict(packed_side=side,edge=edge,layer=k+1,coordinates=surface.tolist(),reason=payload[2],source_vertices=top.edges[edge].tolist()),coordinate_hash=q.array_hash(high)),None,0.

def tangent_trial(points,direction,distance,sdf,target):
    sample=sdf.evaluate(points);basis=tangent_basis(sample.normal)
    field=np.einsum('nij,nj->ni',basis,np.einsum('nji,nj->ni',basis,direction));length=np.linalg.norm(field,axis=1);field=np.divide(field,length[:,None],out=np.zeros_like(field),where=length[:,None]!=0)
    return q.predict_next_front(points+distance*field,target,sdf,iteration_cap=256).coordinates

def steering(x,f,top,p,prefix,high,previous,sdf,spec,rec,schedule):
    tick=time.perf_counter();low=prefix[-1];k=len(prefix);events=list(rec.get('failures',[]))
    events.extend(dict(kind='cell',metric='jacobian',layer=k+1,entity=b['column'],columns=[b['column']],value=b['jacobian']) for b in rec.get('exact_failures',[]))
    if not events:return [],None,None,dict(seconds=time.perf_counter()-tick,certificate_seconds=0.)
    try:
        _,field,_=q.controlling_support(SimpleNamespace(coordinates=low),top,high,events)
        active=np.unique(np.concatenate([f[e['columns']].ravel() for e in events]));patch=q.bounded_patch(top,active)
        core=np.union1d(p.core_columns,patch.detailed.core_columns);_,verify,_=q.foundation(x,f,core,sdf,spec)
        modes=q.correction_fields(SimpleNamespace(coordinates=low),top,patch,field)
        more,_=search_fields(SimpleNamespace(coordinates=x),top,patch,low,float(schedule.targets[-1]));modes+=more
    except (RuntimeError,ValueError) as exc:return [dict(status='QualifiedSupportUnavailable',error=str(exc))],None,None,dict(seconds=time.perf_counter()-tick,certificate_seconds=0.)
    history=[];seen=set();ct=0.
    for mode,direction in modes:
        for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
            row=dict(mode=mode,factor=factor,active_vertices=active.tolist(),verification_columns=core.tolist())
            try:
                trial=high.copy();trial[active]=tangent_trial(high[active],direction,factor*schedule.increments[k],sdf,float(schedule.targets[k]));key=q.array_hash(trial)
                if key in seen:row.update(status='DuplicateCandidate',coordinate_hash=key);history.append(row);continue
                seen.add(key);r,m,c=checked(x,f,top,verify,prefix,trial,previous);ct+=c;row.update(status='Evaluated',coordinate_hash=key,passed=r['passed'],gate=r)
                history.append(row)
                if r['passed']:return history,(trial,r),m,dict(seconds=time.perf_counter()-tick,certificate_seconds=ct)
            except (RuntimeError,ValueError,np.linalg.LinAlgError) as exc:row.update(status='ProjectionOrEvaluationFailure',error=str(exc));history.append(row)
    return history,None,None,dict(seconds=time.perf_counter()-tick,certificate_seconds=ct)

def rings(top,core_vertices,changed_vertices):
    seen=set(map(int,core_vertices));front=set(seen);distance=0
    while front:
        if front.intersection(map(int,changed_vertices)):return distance
        following=set(int(v) for u in front for v in top.vertex_neighbors.row(u))-seen;seen|=following;front=following;distance+=1
    return None

def channels(persist=True):
    schedule=q.load_schedule();records=[];times=[]
    for old in q.definitions(schedule.targets):
        if old['kind'] not in ('wedge','frustum'):continue
        tick=time.perf_counter();wall,of,oc=q.build(old);osdf=q.FixtureSDF(old)
        x,f,core,spec=extended_front(old);sdf=ExtendedSDF(spec);top,p,base=q.foundation(x,f,core,sdf,spec)
        assert np.array_equal(f,of) and np.array_equal(x[p.vertices],wall[p.vertices])
        changed=np.flatnonzero(np.any(x!=wall,axis=1));authority=dict(definition=spec,foundation=base,core_vertices=p.core_vertices.tolist(),core_triangles=core.tolist(),metric_halo_vertices=p.vertices.tolist(),changed_closure_vertices=changed.tolist(),minimum_topological_rings=rings(top,p.core_vertices,changed),core_and_metric_halo_wall_identity=True,topology_unchanged=True)
        setup_seconds=time.perf_counter()-tick;closure_tick=time.perf_counter()
        # Bound the independent closure projection study. It confers no commit.
        diagnostic=x;closure=[]
        for k,target in enumerate(schedule.targets):
            try:
                pred=q.predict_next_front(diagnostic,target,sdf,iteration_cap=256);closure.append(dict(layer=k+1,projection_passed=True,max_iterations=int(pred.iterations.max()),coordinate_hash=q.array_hash(pred.coordinates)));diagnostic=pred.coordinates
            except (RuntimeError,ValueError) as exc:closure.append(dict(layer=k+1,projection_passed=False,error=str(exc)));break
        closure_seconds=time.perf_counter()-closure_tick;rolling_tick=time.perf_counter()
        prefix=[];previous=None;history=[];polls=[];ct=0.;st=0.;migration=[]
        for k,target in enumerate(schedule.targets):
            try:pred=q.predict_next_front(prefix[-1] if prefix else x,target,sdf,iteration_cap=256);high=pred.coordinates
            except (RuntimeError,ValueError) as exc:history.append(dict(layer=k+1,passed=False,committed=False,projection_failure=str(exc)));break
            if k<5:
                sealed,receipt=q.load_checkpoint(q.OUT/old['name'],k+1,top.sha256)
                high[p.vertices]=sealed[p.vertices]
                assert np.all(abs(sdf.evaluate(high).phi-target)<=sdf.evaluate(high).source_error_bound)
                migration.append(dict(layer=k+1,source_checkpoint_hash=q.identity(receipt),core_coordinate_hash=q.array_hash(sealed[p.core_vertices]),halo_coordinate_hash=q.array_hash(sealed[p.vertices]),identity_preserved=True,closure_recertified=True))
            rec,m,c=checked(x,f,top,p,prefix,high,previous);ct+=c
            if not rec['passed'] and k>=5:
                ps,accepted,am,tt=steering(x,f,top,p,prefix,high,previous,sdf,spec,rec,schedule);polls.append(dict(layer=k+1,trials=ps));st+=tt['seconds'];ct+=tt['certificate_seconds']
                if accepted is not None:high,rec=accepted;m=am
            rec['committed']=bool(rec['passed']);rec['projection_iterations_max']=int(pred.iterations.max());history.append(rec)
            if not rec['passed']:break
            prefix.append(high);previous=m['cells']
            if persist and k>=5:
                folder=OUT/old['name'];folder.mkdir(exist_ok=True);np.save(folder/f'front_{k+1:02}.npy',high);save(old['name']+f'/checkpoint_{k+1:02}.json',dict(layer=k+1,coordinate_hash=q.array_hash(high),topology_hash=top.sha256,gate=rec))
        # Diagnose whether the first failed geometric inputs changed with closure.
        terminal=history[-1];entities=[]
        for e in terminal.get('failures',[]):entities.extend(e['columns'])
        entities.extend(b['column'] for b in terminal.get('exact_failures',[]));vertices=np.unique(f[np.unique(entities)]) if entities else np.array([],int)
        local_independent=bool(len(vertices) and np.array_equal(x[vertices],wall[vertices]))
        if len(vertices):
            samples=np.r_[prefix[-1][vertices],high[vertices]];a=sdf.evaluate(samples);b=osdf.evaluate(samples)
            local_sdf_equal=bool(np.all(abs(a.phi-b.phi)<=a.source_error_bound+b.source_error_bound) and np.all(np.linalg.norm(a.normal-b.normal,axis=1)<=64*np.finfo(float).eps))
        else:local_sdf_equal=False
        classification='SteerableThroughRequestedLayers' if len(prefix)==20 else ('CoreQualityLimitBeforeCapacity' if local_independent and local_sdf_equal else 'UnresolvedCoreContinuation')
        row=dict(name=old['name'],authority=authority,closure_projection_study=closure,sealed_prefix_migration=migration,history=history,steering=polls,last_valid_layer=len(prefix),first_unadmitted_layer=len(prefix)+1 if len(prefix)<20 else None,classification=classification,failed_input_wall_unchanged=local_independent,failed_input_SDF_normal_unchanged=local_sdf_equal,failure_vertices=vertices.tolist(),capacity_authority=False,remaining=q.schedule_demand(schedule,len(prefix)))
        records.append(row);times.append(dict(name=old['name'],seconds=time.perf_counter()-tick,setup_seconds=setup_seconds,closure_projection_seconds=closure_seconds,rolling_seconds=time.perf_counter()-rolling_tick,certificate_seconds=ct,steering_seconds=st));print(old['name'],len(prefix),classification,flush=True)
        if persist:save(old['name']+'/result.json',row)
    return records,times


def trace(point,sdf,target,centres,radii,cap=256):
    p=point.copy();history=[]
    for k in range(cap+1):
        z=sdf.evaluate(p[None]);dist=np.linalg.norm(p[None]-centres,axis=1)-radii;owner=int(np.argmin(dist));res=float(z.phi[0]-target)
        history.append(dict(iteration=k,coordinates=p.tolist(),phi=float(z.phi[0]),residual=res,source_error_bound=float(z.source_error_bound[0]),owner=owner,normal=z.normal[0].tolist(),gradient_norm=float(z.gradient_norm[0])))
        if abs(res)<=z.source_error_bound[0]:return history,'Converged'
        if k==cap:return history,'ProjectionIterationCap'
        p=p-res*z.gradient[0]/z.gradient_norm[0]**2
    raise AssertionError

def general(persist=True):
    tick=time.perf_counter();schedule=q.load_schedule();spec=q.definitions(schedule.targets)[-1];x,f,core=q.build(spec);sdf=q.FixtureSDF(spec);top,p,base=q.foundation(x,f,core,sdf,spec)
    prefix=[q.load_checkpoint(q.OUT/spec['name'],k,top.sha256)[0] for k in (1,2,3)];low=prefix[-1];target=float(schedule.targets[3]);centres=np.array(spec['centres']);radii=np.array(spec['radii'])
    previous=q.immediate_metrics(top,prefix[1],prefix[2],3,q.immediate_metrics(top,prefix[0],prefix[1],2,q.immediate_metrics(top,x,prefix[0],1)['cells'])['cells'])['cells']
    failed=[];traces=[]
    for v in range(len(low)):
        try:q.predict_next_front(low[v:v+1],target,sdf)
        except (RuntimeError,ValueError):failed.append(v)
    for v in failed:
        history,status=trace(low[v],sdf,target,centres,radii)
        traces.append(dict(vertex=v,source_component=int(np.argmin(np.linalg.norm(x[v]-centres,axis=1)-radii)),neighbors=top.vertex_neighbors.row(v).tolist(),triangles=top.vertex_triangles.row(v).tolist(),current_layer=3,target=target,history=history,status=status))
    high=q.predict_next_front(low,target,sdf,iteration_cap=256).coordinates;nominal,m,ct=checked(x,f,top,p,prefix,high,previous)
    diagnosis_seconds=time.perf_counter()-tick
    trials,accepted,am,tt=steering(x,f,top,p,prefix,high,previous,sdf,spec,nominal,schedule);ct+=tt['certificate_seconds']
    print('general steering complete',len(trials),accepted is not None,flush=True)
    if persist:save('GENERAL_STEERING.json',dict(trials=trials,nominal=nominal))
    witness_tick=time.perf_counter()
    # Ray brackets demonstrate existence, not absence, and remain diagnostic.
    brackets=[]
    for v in failed:
        sample=sdf.evaluate(low[v:v+1]);basis=tangent_basis(sample.normal)[0]
        directions=[('gradient',sample.normal[0])]+[(f'tangent_{j}_{sign:+g}',sign*basis[:,j]) for j in range(2) for sign in (1.,-1.)]
        rays=[]
        for name,d in directions:
            grid=np.linspace(0.,float(schedule.targets[-1]-schedule.targets[2]),65);values=sdf.evaluate(low[v]+grid[:,None]*d).phi-target
            crossings=np.flatnonzero(values[:-1]*values[1:]<=0);records=[]
            for j in crossings:
                a=float(grid[j]);b=float(grid[j+1]);fa=float(values[j]);fb=float(values[j+1]);initial_points=[low[v]+a*d,low[v]+b*d]
                exact_margins=[[str(sum((Fraction(float(xx))-Fraction(float(cc)))**2 for xx,cc in zip(point,c))-(Fraction(float(r))+Fraction(target))**2) for c,r in zip(centres,radii)] for point in initial_points]
                for _ in range(64):
                    mid=(a+b)/2
                    if mid==a or mid==b:break
                    fm=float(sdf.evaluate((low[v]+mid*d)[None]).phi[0]-target)
                    if fa*fm<=0:b=mid;fb=fm
                    else:a=mid;fa=fm
                records.append(dict(t=[a,b],residual=[fa,fb],points=[(low[v]+a*d).tolist(),(low[v]+b*d).tolist()],initial_points=[z.tolist() for z in initial_points],exact_squared_endpoint_margins=exact_margins,scope='Exact endpoint signs and continuity prove existence; refined numeric bracket is diagnostic, not exact root representation'))
            rays.append(dict(direction=name,brackets=records,sample_range=[0,float(grid[-1])]))
        brackets.append(dict(vertex=v,rays=rays,converged_newton_point=high[v].tolist(),residual=float(sdf.evaluate(high[v:v+1]).phi[0]-target)))
    pairs=[]
    for i in range(3):
        for j in range(i):
            d2=sum((Fraction(float(a))-Fraction(float(b)))**2 for a,b in zip(centres[i],centres[j]));history=[]
            for k,t in enumerate(schedule.targets):
                demand=Fraction(float(radii[i]))+Fraction(float(radii[j]))+2*Fraction(float(t));overlap=demand*demand-d2
                history.append(dict(layer=k+1,squared_overlap_margin=str(overlap),overlap=overlap>=0))
            pairs.append(dict(components=[j,i],centre_distance_squared=str(d2),first_offset_ball_contact=next(z['layer'] for z in history if z['overlap']),history=history,scope='Exact binary64 parameter inequality. Offset-ball overlap proves opposing-branch convergence, not impossibility of every discrete steering candidate.'))
    witness_seconds=time.perf_counter()-witness_tick
    certificate_tick=time.perf_counter()
    # Exact certificate on diagnostic independent source-radial continuation.
    owners=np.argmin(np.linalg.norm(x[:,None]-centres,axis=2)-radii,axis=1);radial=centres[owners]+(x-centres[owners])*(1+target/radii[owners])[:,None]
    # Bound diagnostic exact work to nearest cross-component current sides.
    # This is not completeness certification and has no acceptance authority.
    surfaces,labels=q.surface_arrays(x,top.edges,prefix+[radial]);native=q.Filter();forest=native.forest(surfaces,len(top.edges));population=forest.pairs(4);forest.close()
    ia=(population>>np.uint64(32)).astype(int);ib=(population&np.uint64(0xffffffff)).astype(int);edge_owner=owners[top.edges[:,0]]
    mask=(edge_owner[ia%len(top.edges)]!=edge_owner[ib%len(top.edges)])&(ia//len(top.edges)==3)&(ib//len(top.edges)==3)
    selected=population[mask];aa=ia[mask];bb=ib[mask];distance=np.sum((surfaces[aa].mean(1)-surfaces[bb].mean(1))**2,axis=1);order=np.lexsort((selected,distance));checks=[]
    for packed in selected[order[:8]]:
        a=int(packed>>np.uint64(32));b=int(packed&np.uint64(0xffffffff))
        try:
            result,perf=q.prove(surfaces,labels,np.array([packed],dtype=np.uint64),native)
            result=q.plain(result);checks.append(dict(pair=[a,b],result=result,surfaces=surfaces[[a,b]].tolist(),labels=labels[[a,b]].tolist()))
            if any(z['result']['classification']=='CertifiedForbiddenIntersection' for z in result['fallback']):break
        except (RuntimeError,ValueError,ArithmeticError) as exc:
            checks.append(dict(pair=[a,b],error=str(exc),classification='UnresolvedDiagnosticExactResult',surfaces=surfaces[[a,b]].tolist(),labels=labels[[a,b]].tolist()));break
    collision=dict(candidate_population=len(selected),checks=checks,complete=False,scope='At most eight nearest cross-component AABB survivors; diagnostic radial continuation only')
    ct+=time.perf_counter()-certificate_tick
    row=dict(definition=spec,foundation=base,failed_vertices=failed,traces=traces,nominal_validation=nominal,ray_brackets=brackets,opposing_branch_pairs=pairs,diagnostic_radial_certificate=collision,radial_candidate_is_not_SDF_admitted=True,steering=trials,steering_success=accepted is not None,
        projection_classification='DiscreteProjectionDefect',mechanism='Nearest-sphere owner switching; 64-step cap before convergence. Unit gradients remain nondegenerate; target solutions exist.',last_valid_layer=4 if accepted else 3,first_unadmitted_layer=None if accepted else 4,warning_layer=0,warning_lead=4,
        witness_type='DirectOpposingFrontConvergenceWitness',witness_scope='Direct branch convergence only; not an unavoidable capacity boundary',classification='SteerableDiscreteHazard' if accepted else 'GeneralSDFWarningOnly',capacity_authority=False)
    if persist:save('GENERAL_SDF.json',row);np.save(OUT/'general_uncommitted_nominal_layer4.npy',high)
    return row,dict(seconds=time.perf_counter()-tick,projection_diagnosis_seconds=diagnosis_seconds,SDF_witness_seconds=witness_seconds,steering_seconds=tt['seconds'],certificate_seconds=ct)

if __name__=='__main__':
    start=time.perf_counter();a,at=channels();b,bt=general();save('RESULTS.json',dict(channels=a,general=b));save('TIMINGS.json',dict(seconds=time.perf_counter()-start,channels=at,general=bt,peak_rss=q.psutil.Process().memory_info().peak_wset))
    c,ct=channels(False);d,dt=general(False);equal=q.identity(clean(dict(channels=a,general=b)))==q.identity(clean(dict(channels=c,general=d)));assert equal
    save('DETERMINISM.json',dict(equal=equal,scientific_hash=q.identity(clean(dict(channels=a,general=b)))));save('REPLAY_TIMINGS.json',dict(channels=ct,general=dt,peak_rss=q.psutil.Process().memory_info().peak_wset))
