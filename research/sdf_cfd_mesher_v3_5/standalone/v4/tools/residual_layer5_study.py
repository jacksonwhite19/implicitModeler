"""Read-only baseline reproduction and bounded residual-cell repair study."""
import json, time, subprocess
from pathlib import Path
import compressed_layer_recovery as r
from scipy.optimize import minimize
from types import SimpleNamespace
np=r.np; q=r.q; w=r.w; b=r.b
OUT=r.g.ROOT/'results/residual_layer5_failure'

def save(name,value):
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/name).write_text(json.dumps(w.clean(value),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8',newline='\n')

def reproduce():
    case,prefix,history,previous,seed=r.seed()
    exp=r.load(r.OUT/'result.json')['expansions'][-1]
    active=np.array(exp['active']);weights=r.g.harmonic_weights(case.top,exp['centres'],active)
    plan=r.plan(case.s,history,'receding',5,weights)
    targets=history[-1]+plan['height']*(1-(1-.125)*weights)
    targets[weights==0]=case.s.targets[4]
    high,record,metrics,timing=r.evaluate(case,prefix,previous,targets,active)
    assert [e['column'] for e in record['gate']['exact_failures']]==[1232]
    return case,prefix,history,previous,seed,targets,high,record,timing

class LocalProblem:
    """Three controls per vertex: two existing tangent modes and log height.

    Bounds limit computation, not scientific validity or an impossibility claim.
    All untouched upper vertices and every lower vertex remain bitwise fixed.
    """
    def __init__(self,case,prefix,history,previous,targets,high,active):
        if len(prefix)!=4 or len(history)!=4:raise ValueError('Layer5StudyOnly')
        self.case=case;self.prefix=prefix;self.history=history;self.previous=previous
        self.targets=targets;self.high=high;self.low=prefix[-1];self.active=np.array(sorted(active),int)
        self.patch=q.bounded_patch(case.top,self.active);self.basis=w.tangent_basis(case.sdf.evaluate(high[self.active]).normal)
        _,self.radii=w.search_fields(SimpleNamespace(coordinates=case.x),case.top,self.patch,self.low,float(case.s.targets[-1]))
        self.ancestry=b.owner(case.x[self.active],case.spec);self.n=len(self.active);self.calls=0
        self.heights=targets[self.active]-history[-1][self.active]
        if not np.all(self.heights>0):raise ValueError('NonpositiveStudyHeight')
        area=np.linalg.norm(np.cross(self.low[case.f[:,1]]-self.low[case.f[:,0]],self.low[case.f[:,2]]-self.low[case.f[:,0]]),axis=1)
        self.scale=area*case.s.increments[4]

    def project(self,u):
        u=np.asarray(u).reshape(self.n,3);targets=self.targets.copy();high=self.high.copy()
        targets[self.active]=self.history[-1][self.active]+self.heights*np.exp(u[:,2])
        delta=np.einsum('nij,nj->ni',self.basis,u[:,:2]*self.radii[:,None]);points=self.high[self.active]+delta
        for j,v in enumerate(self.active):
            # A zero trial is exact rollback, not another floating-point projection.
            if np.all(u[j]==0):
                targets[v]=self.targets[v]
                continue
            high[v:v+1],_=b.sphere_project(points[j:j+1],float(targets[v]),self.case.sdf,self.case.spec,self.ancestry[j:j+1])
        sample=self.case.sdf.evaluate(high)
        if not np.all(abs(sample.phi-targets)<=sample.source_error_bound):raise ValueError('TargetAdmissionFailure')
        return high,targets,delta

    def constraints(self,u,labels=False):
        high,_,_=self.project(u);self.calls+=1
        m=q.immediate_metrics(self.case.top,self.low,high,5,self.previous);vals=[];names=[]
        def add(kind,name,ids,v):
            vals.extend(v);names.extend((kind,name,int(i)) for i in ids)
        ids=self.patch.detailed.core_columns;c=m['cells']
        for name in ('volume','signed_volume','jacobian'):add('cell',name,ids,c[name][ids]/self.scale[ids])
        add('cell','determinant',ids,c['determinant'][ids]/.0011-1)
        add('cell','aspect',ids,1-c['aspect'][ids]/1000)
        for kind,ids in [('side',self.patch.detailed.side_edges),('cap',self.patch.detailed.core_columns)]:
            a=m['sides' if kind=='side' else 'caps']
            add(kind,'weight',ids,a['weight'][ids]/.055-1);add(kind,'skewness',ids,1-a['skewness'][ids]/3.63636)
            add(kind,'symmetric_volume_ratio',ids,a['symmetric_volume_ratio'][ids]/.01-1)
        vals=np.asarray(vals)
        if not np.all(np.isfinite(vals)):raise ValueError('NonfiniteMetric')
        return (vals,names) if labels else vals

    def verify(self,u):
        high,targets,delta=self.project(u);t=time.perf_counter()
        gate,_,ct=w.checked(self.case.x,self.case.f,self.case.top,self.case.verify,self.prefix,high,self.previous)
        metrics=q.immediate_metrics(self.case.top,self.low,high,5,self.previous)
        failed=q.exact_wedge(np.r_[self.low[self.case.f[1232]],high[self.case.f[1232]]])
        return dict(gate=gate,numeric_events=metrics['events'],residual_cell=failed,coordinate_hash=q.array_hash(high),
            target_hash=q.array_hash(targets),heights=targets[self.active]-self.history[-1][self.active],
            max_tangent_displacement=float(np.linalg.norm(delta,axis=1).max()),
            maximum_unrecovered=float(max(self.case.s.targets[4]-targets)),u=u),dict(seconds=time.perf_counter()-t,certificate_seconds=ct)

    def solve(self,mode,iterations=40):
        n=3*self.n;u=np.zeros(n);f=self.constraints(u);history=[]
        bounds=[]
        for h in self.heights:
            bounds.extend([(-1/np.sqrt(2),1/np.sqrt(2)) if mode!='compression' else (0.,0.)]*2)
            bounds.append((np.log(1/256),np.log(self.case.s.increments[4]/h)) if mode!='tangent' else (0.,0.))
        def callback(z):
            history.append(dict(iteration=len(history),slack=float(min(self.constraints(z[:-1]))),u=z[:-1].copy()))
        # Standard deterministic SQP maximizes common local constraint slack.
        # Solver ftol is computational only. Production gates decide acceptance.
        tick=time.perf_counter()
        result=minimize(lambda z:-z[-1],np.r_[u,min(f)],method='SLSQP',bounds=bounds+[(None,None)],
            constraints=[dict(type='ineq',fun=lambda z:self.constraints(z[:-1])-z[-1])],callback=callback,
            options=dict(maxiter=iterations,ftol=1e-10))
        best=result.x[:-1];rec,perf=self.verify(best)
        return dict(mode=mode,success=bool(result.success),message=result.message,iterations=int(result.nit),
            history=history,result=rec,search_bounds=bounds),dict(seconds=time.perf_counter()-tick,calls=self.calls,verification=perf)

    def jacobian_probe(self,mode,iterations=60):
        """Feasibility restoration of all affected signs; no acceptance shortcut."""
        n=3*self.n;u=np.zeros(n);f,labels=self.constraints(u,True)
        mask=np.array([x[0]=='cell' and x[1] in ('jacobian','signed_volume') for x in labels])
        bounds=[]
        for h in self.heights:
            bounds.extend([(-1/np.sqrt(2),1/np.sqrt(2)) if mode!='compression' else (0.,0.)]*2)
            bounds.append((np.log(1/256),np.log(self.case.s.increments[4]/h)) if mode!='tangent' else (0.,0.))
        history=[]
        def callback(z):history.append(dict(u=z[:-1].copy(),minimum_sign_precursor=float(min(self.constraints(z[:-1])[mask]))))
        tick=time.perf_counter()
        result=minimize(lambda z:-z[-1],np.r_[u,min(f[mask])],method='SLSQP',bounds=bounds+[(None,None)],
            constraints=[dict(type='ineq',fun=lambda z:self.constraints(z[:-1])[mask]-z[-1])],callback=callback,
            options=dict(maxiter=iterations,ftol=1e-10))
        rec,perf=self.verify(result.x[:-1])
        return dict(mode=mode,search='sign restoration diagnostic only',history=history,result=rec,
            success=bool(result.success),message=result.message),dict(seconds=time.perf_counter()-tick,verification=perf)

def probes():
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    problem=LocalProblem(case,prefix,history,previous,targets,high,case.f[1232]);u=np.zeros(3*problem.n)
    f,labels=problem.constraints(u,True);h=np.cbrt(np.finfo(float).eps);J=[];G=[]
    for j in range(len(u)):
        a=u.copy();z=u.copy();a[j]+=h;z[j]-=h
        J.append((problem.constraints(a)-problem.constraints(z))/(2*h))
        G.append(((problem.project(a)[0]-problem.project(z)[0])/(2*h))[problem.active].ravel())
    J=np.array(J).T;G=np.array(G).T
    save('SENSITIVITY.json',dict(labels=labels,values=f,J=J,geometry_response=G,geometry_rank=int(np.linalg.matrix_rank(G)),
        constraint_rank=int(np.linalg.matrix_rank(J)),step=float(h),method='central finite differences of admitted branch projector; guidance only'))
    results=[];times=[]
    for mode in ('compression','tangent','coupled'):
        row,perf=problem.solve(mode);results.append(row);times.append(perf)
        print(mode,row['result']['residual_cell'],len(row['result']['numeric_events']),row['result']['gate']['passed'],flush=True)
        save('PROBES.json',results);save('PROBE_TIMING.json',times)
    save('TOTAL_TIMING.json',dict(seconds=time.perf_counter()-tick,peak_rss=q.psutil.Process().memory_info().peak_wset))

def sign_probes():
    case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    p=LocalProblem(case,prefix,history,previous,targets,high,case.f[1232]);rows=[];times=[]
    for mode in ('compression','tangent','coupled'):
        row,t=p.jacobian_probe(mode);rows.append(row);times.append(t)
        print(mode,row['result']['residual_cell'],len(row['result']['numeric_events']),flush=True)
        save('SIGN_PROBES.json',rows);save('SIGN_TIMING.json',times)

def neighbors_probe():
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    centres=case.f[1232];active=set(map(int,centres))
    for v in centres:active.update(map(int,case.top.vertex_neighbors.row(v)))
    p=LocalProblem(case,prefix,history,previous,targets,high,active)
    save('NEIGHBOR_SUPPORT.json',dict(active=p.active,tangent_dofs=2*p.n,height_dofs=p.n,
        affected_columns=p.patch.affected_columns,verification_ring=p.patch.verification_ring,
        fixed_boundary=p.patch.fixed_boundary,halo=p.patch.halo,reason='One topological ring around failed cell; local height blending and collateral constraints'))
    rows=[];times=[]
    # Bounded common compression and convex graph smoothing, no schedule recovery.
    for scale in (1/2,1/4,1/16,1/64,1/256):
        u=np.zeros((p.n,3));u[:,2]=np.log(scale);rec,t=p.verify(u.ravel())
        rows.append(dict(mode='common_local_compression',scale=scale,result=rec));times.append(t)
    for blend in (.25,.5,1.):
        heights=targets-history[-1];desired=heights.copy()
        for v in p.active:desired[v]=(1-blend)*heights[v]+blend*np.mean(heights[case.top.vertex_neighbors.row(v)])
        u=np.zeros((p.n,3));u[:,2]=np.log(desired[p.active]/p.heights)
        rec,t=p.verify(u.ravel());rows.append(dict(mode='neighbor_height_smoothing',blend=blend,result=rec));times.append(t)
    save('LOCAL_SMOOTHING.json',rows)
    row,t=p.solve('coupled',30);save('NEIGHBOR_COUPLED.json',row);times.append(t)
    save('NEIGHBOR_TIMING.json',dict(seconds=time.perf_counter()-tick,details=times,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print('neighbor',p.n,row['result']['residual_cell'],len(row['result']['numeric_events']),row['result']['gate']['passed'],flush=True)

def branch_escape():
    case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    p=LocalProblem(case,prefix,history,previous,targets,high,case.f[1232]);rows=[];times=[]
    # Search direction comes from the diagnosed source-1/source-2 interaction.
    # Only its components in the authoritative tangent basis are applied.
    source=np.array(case.spec['centres'])[p.ancestry];normal=high[p.active]-source
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    other=high[p.active]-np.array(case.spec['centres'][1]);other/=np.linalg.norm(other,axis=1)[:,None]
    away=other-normal*np.sum(other*normal,axis=1)[:,None]
    uv=np.einsum('nji,nj->ni',p.basis,away);uv/=np.linalg.norm(uv,axis=1)[:,None]
    for scale in (-1.,-.5,-.25,-.125,.125,.25,.5,1.):
        u=np.zeros((p.n,3));u[:,:2]=scale*uv
        rec,t=p.verify(u.ravel());rows.append(dict(scale=scale,result=rec));times.append(t)
    save('BRANCH_ESCAPE.json',rows);save('BRANCH_ESCAPE_TIMING.json',times)
    print([(z['scale'],z['result']['residual_cell']['jacobian_sign'],len(z['result']['gate'].get('exact_failures',[])),len(z['result']['numeric_events'])) for z in rows],flush=True)

def run():
    start=time.perf_counter()
    case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    low=prefix[-1];ids=case.f[1232];patch=q.bounded_patch(case.top,ids)
    metrics=q.immediate_metrics(case.top,low,high,5,previous)
    normals=np.cross(high[ids[1]]-high[ids[0]],high[ids[2]]-high[ids[0]])
    direct=dict(vertices=ids,low=low[ids],high=high[ids],targets=targets[ids],heights=targets[ids]-history[-1][ids],
        upper_area_vector=normals,corner_displacements=high[ids]-low[ids],
        upper_corner_jacobians=(high[ids]-low[ids])@normals,
        lower_source=b.owner(case.x[ids],case.spec),lower_owner=b.owner(low[ids],case.spec),upper_owner=b.owner(high[ids],case.spec))
    seed.pop('certificate_seconds',None)
    save('BASELINE.json',dict(record=record,direct=direct,numeric_events=metrics['events'],conditioning=r.condition(case,low,high,5),seed=seed))
    save('SUPPORT.json',dict(active_vertices=ids,tangent_dofs=6,height_dofs=3,affected_columns=patch.affected_columns,
        fixed_boundary=patch.fixed_boundary,halo=patch.halo,verification_ring=patch.verification_ring,
        metric_columns=patch.detailed.columns,side_edges=patch.detailed.side_edges))
    np.savez_compressed(OUT/'baseline.npz',high=high,targets=targets,low=low)
    save('BASELINE_TIMING.json',dict(seconds=time.perf_counter()-start,validation=timing,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print(json.dumps(w.clean(dict(direct=direct,exact=record['gate']['exact_failures'],events=metrics['events'])),indent=2),flush=True)

def forensic():
    from fractions import Fraction
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    p=LocalProblem(case,prefix,history,previous,targets,high,case.f[1232]);ids=p.active
    sub=lambda a,b:[x-y for x,y in zip(a,b)]
    cross=lambda a,b:[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    top=[[Fraction(float(x)) for x in v] for v in high[ids]]
    n=cross(sub(top[1],top[0]),sub(top[2],top[0]))
    J=[sum(x*y for x,y in zip(sub(top[i],[Fraction(float(x)) for x in prefix[-1][v]]),n)) for i,v in enumerate(ids)]
    save('EXACT_UPPER_CORNERS.json',dict(values=list(map(str,J)),signs=[(x>0)-(x<0) for x in J],vertices=ids))
    directions=[];dt=[];step=float(np.cbrt(np.finfo(float).eps))
    for j in range(3*p.n):
        for sign in (-1,1):
            u=np.zeros(3*p.n);u[j]=sign*step;rec,t=p.verify(u);directions.append(dict(dof=j,sign=sign,result=rec));dt.append(t)
    save('DIRECTION_CONFLICTS.json',directions)
    c1=np.array(case.spec['centres'][1]);c2=np.array(case.spec['centres'][2]);R1=case.spec['radii'][1];R2=case.spec['radii'][2]
    plane=lambda t:(np.dot(c2,c2)-np.dot(c1,c1)-(R2+t)**2+(R1+t)**2)/2
    area=np.cross(high[ids[1]]-high[ids[0]],high[ids[2]]-high[ids[0]])
    baseline_metrics=q.immediate_metrics(case.top,prefix[-1],high,5,previous)
    other=[e for e in baseline_metrics['events'] if not set(e['columns'])&set(p.patch.affected_columns)]
    neighbor=r.load(OUT/'NEIGHBOR_SUPPORT.json')
    outside_neighbor=[e for e in baseline_metrics['events'] if not set(e['columns'])&set(neighbor['affected_columns'])]
    lower=prefix[-1][case.f];areas=np.cross(lower[:,1]-lower[:,0],lower[:,2]-lower[:,0])
    save('CAUSE.json',dict(source_pair=[1,2],intersection_plane_normal=c2-c1,upper_area_vector=area,
        plane_residuals=high[ids]@(c2-c1)-plane(targets[ids]),orientation_dot=float(area@(c2-c1)),
        equal_local_heights=targets[ids]-history[-1][ids],unchanged_lower_valid=seed['gate']['passed'],
        zero_area_lower_triangles=int(np.sum(np.linalg.norm(areas,axis=1)==0)),
        immutable_failures_three_vertices=other,immutable_failures_one_ring=outside_neighbor,
        conclusion='Projected upper triangle folds on the source-1/source-2 offset intersection. Two exact upper-corner triple products and signed volume are negative. Equal positive increments do not prevent this tangential fold. The valid layer-4 ancestry is fixed, not defective.',
        limitation='This diagnoses these candidates. Other tangent/height configurations are not excluded.'))
    rows=r.load(OUT/'LOCAL_SMOOTHING.json');best=next(z for z in rows if z['mode']=='neighbor_height_smoothing' and z['blend']==.5)
    pp=LocalProblem(case,prefix,history,previous,targets,high,neighbor['active']);bh,bt,delta=pp.project(best['result']['u'])
    np.savez_compressed(OUT/'best_invalid.npz',high=bh,targets=bt,u=best['result']['u'])
    save('THICKNESS_CONDITIONING.json',dict(nominal_increment=float(case.s.increments[4]),nominal_cumulative=float(case.s.targets[4]),
        baseline_height=targets-history[-1],baseline_actual=targets,baseline_debt=case.s.targets[4]-targets,
        best_height=bt-history[-1],best_actual=bt,best_debt=case.s.targets[4]-bt,
        debt_change_from_seed=(case.s.targets[4]-bt)-(case.s.targets[3]-history[-1]),
        seed_conditioning=r.condition(case,prefix[-2],prefix[-1],4),baseline_conditioning=r.condition(case,prefix[-1],high,5),
        best_invalid_conditioning=r.condition(case,prefix[-1],bh,5),accepted=False))
    certificates=[];cts=[]
    for name,hh in [('baseline',high),('best_invalid',bh)]:
        start=time.perf_counter();cert,_=q.certificate(case.x,case.top,case.verify,prefix+[hh]);cts.append(time.perf_counter()-start)
        certificates.append(dict(name=name,coordinate_hash=q.array_hash(hh),certificate=cert))
    save('COLLISION_RESULTS.json',certificates)
    save('FORENSIC_TIMING.json',dict(seconds=time.perf_counter()-tick,certificate_seconds=cts,directions=dt,peak_rss=q.psutil.Process().memory_info().peak_wset))

def complete(replay=False):
    names=['BASELINE.json','SUPPORT.json','SENSITIVITY.json','PROBES.json','SIGN_PROBES.json','NEIGHBOR_SUPPORT.json',
        'LOCAL_SMOOTHING.json','NEIGHBOR_COUPLED.json','BRANCH_ESCAPE.json','EXACT_UPPER_CORNERS.json',
        'DIRECTION_CONFLICTS.json','CAUSE.json','THICKNESS_CONDITIONING.json','COLLISION_RESULTS.json']
    before={n:r.load(OUT/n) for n in names} if replay else None
    tick=time.perf_counter()
    run();probes();sign_probes();neighbors_probe();branch_escape();forensic()
    after={n:r.load(OUT/n) for n in names};identity=q.identity(after)
    perf=dict(seconds=time.perf_counter()-tick,peak_rss=q.psutil.Process().memory_info().peak_wset)
    if replay:
        equality={n:q.identity(before[n])==q.identity(after[n]) for n in names}
        save('DETERMINISM.json',dict(equal=all(equality.values()),files=equality,scientific_hash=identity,first_hash=q.identity(before)))
        save('REPLAY_RUNTIME.json',perf);assert all(equality.values()),equality
    else:save('RUNTIME.json',perf)

def iterate_audit(replay=False):
    """Recover best residual-cell trials, not just optimizer final iterates."""
    from fractions import Fraction
    before=r.load(OUT/'ITERATE_AUDIT.json') if replay else None
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    rows=[]
    for file in ('PROBES.json','SIGN_PROBES.json','NEIGHBOR_COUPLED.json'):
        records=r.load(OUT/file)
        if isinstance(records,dict):records=[records]
        active=r.load(OUT/'NEIGHBOR_SUPPORT.json')['active'] if file=='NEIGHBOR_COUPLED.json' else case.f[1232]
        p=LocalProblem(case,prefix,history,previous,targets,high,active)
        for row in records:
            scores=[];best=None
            for i,z in enumerate(row['history']):
                hh,tt,dd=p.project(z['u']);exact=q.exact_wedge(np.r_[prefix[-1][case.f[1232]],hh[case.f[1232]]])
                value=Fraction(exact['jacobian']);scores.append(dict(iteration=i,jacobian=exact['jacobian'],sign=exact['jacobian_sign']))
                if best is None or value>best[0]:best=(value,i,z['u'])
            rec,perf=p.verify(best[2]);rows.append(dict(source=file,mode=row['mode'],best_iteration=best[1],
                selection='Maximum exact residual-cell Jacobian; does not imply best mesh',scores=scores,result=rec))
    save('ITERATE_AUDIT.json',rows)
    if replay:
        equal=q.identity(before)==q.identity(w.clean(rows));save('ITERATE_DETERMINISM.json',dict(equal=equal,scientific_hash=q.identity(w.clean(rows))));assert equal
    save('ITERATE_REPLAY_TIMING.json' if replay else 'ITERATE_TIMING.json',dict(seconds=time.perf_counter()-tick,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print([(x['mode'],x['result']['residual_cell']['jacobian_sign'],len(x['result']['gate'].get('exact_failures',[])),len(x['result']['numeric_events'])) for x in rows],flush=True)

def compression_from_seed(replay=False):
    """Strict height-only control uses layer 4 as projection predictor."""
    before=r.load(OUT/'SEED_COMPRESSION.json') if replay else None
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=reproduce()
    ids=case.f[1232];rows=[]
    for factor in (.5,.25,.0625,.015625,.00390625):
        tt=targets.copy();tt[ids]=history[-1][ids]+factor*(targets[ids]-history[-1][ids])
        hh,z,mm,perf=r.evaluate(case,prefix,previous,tt,ids)
        mm=q.immediate_metrics(case.top,prefix[-1],hh,5,previous)
        rows.append(dict(factor=factor,record=z,exact_cell=q.exact_wedge(np.r_[prefix[-1][case.f[1232]],hh[case.f[1232]]]),
            numeric_events=mm['events'],heights=tt[ids]-history[-1][ids],maximum_unrecovered=float(max(case.s.targets[4]-tt)),
            unchanged_outside=bool(np.array_equal(hh[np.setdiff1d(np.arange(len(hh)),ids)],high[np.setdiff1d(np.arange(len(hh)),ids)]))))
    save('SEED_COMPRESSION.json',rows)
    if replay:
        equal=q.identity(before)==q.identity(w.clean(rows));save('COMPRESSION_DETERMINISM.json',dict(equal=equal,scientific_hash=q.identity(w.clean(rows))));assert equal
    save('COMPRESSION_REPLAY_TIMING.json' if replay else 'COMPRESSION_TIMING.json',dict(seconds=time.perf_counter()-tick,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print([(x['factor'],x['exact_cell']['jacobian_sign'],len(x['record']['gate'].get('exact_failures',[])),len(x['numeric_events'])) for x in rows],flush=True)

if __name__=='__main__':
    import sys
    if '--probes' in sys.argv:probes()
    elif '--sign-probes' in sys.argv:sign_probes()
    elif '--neighbors' in sys.argv:neighbors_probe()
    elif '--escape' in sys.argv:branch_escape()
    elif '--forensic' in sys.argv:forensic()
    elif '--complete' in sys.argv:complete('--replay' in sys.argv)
    elif '--audit' in sys.argv:iterate_audit('--replay' in sys.argv)
    elif '--seed-compression' in sys.argv:compression_from_seed('--replay' in sys.argv)
    else:run()
