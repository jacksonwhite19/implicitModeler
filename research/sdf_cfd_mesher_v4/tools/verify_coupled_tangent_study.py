"""Independent exact and local-tail diagnostics on rejected coupled candidates."""
import json,time
import coupled_tangent_study as c
q=c.q;np=c.np

def conditioning(case,high):
    ids=case.p.core_columns;prisms=np.concatenate((case.low[case.f[ids]],high[case.f[ids]]),axis=1)
    try:
        m=q.cell_metrics(prisms,np.full(len(ids),case.k+1),required_gain_only=True)
        available=m['G_bound_available'];ix=int(np.argmin(m['sigma']));good=np.flatnonzero(available&np.isfinite(m['G_bound']))
        j=int(good[np.argmax(m['G_bound'][good])]) if len(good) else None
        return dict(minimum_sigma=float(m['sigma'][ix]),sigma_column=int(ids[ix]),maximum_available_G_bound=float(m['G_bound'][j]) if j is not None else None,G_column=int(ids[j]) if j is not None else None,unavailable_G=int(np.sum(~available)),columns=len(ids),hard_thresholds=False)
    except (ValueError,np.linalg.LinAlgError) as exc:return dict(available=False,error=str(exc),hard_thresholds=False)

def run():
    tick=time.perf_counter();rows=[];cert_seconds=0.
    specs=q.definitions(q.load_schedule().targets)
    cohort=[(s,'') for s in specs]+[(specs[-1],'__opposed_z'),(specs[-1],'__opposed_minus_z')]
    for spec,suffix in cohort:
        if spec['kind'] not in ('wedge','frustum','spheres') or spec.get('cavity'):continue
        case=c.Case(spec);case.name+=suffix;r=json.loads((c.OUT/case.name/'result.json').read_text());high=np.load(c.OUT/case.name/'candidate.npy')
        reconstructed,projection=case.project(np.array(r['u']));assert np.array_equal(high,reconstructed)
        gate,_,_=c.w.checked(case.x,case.f,case.top,case.verify,case.prefix,high,case.previous);assert q.identity(c.w.clean(gate))==q.identity(r['best_gate'])
        sample=case.sdf.evaluate(high);assert np.all(abs(sample.phi-case.target)<=sample.source_error_bound)
        record=dict(name=case.name,coordinate_identity=True,gate_identity=True,residual_max=float(max(abs(sample.phi-case.target))),target_admitted=True,maximum_tangent_displacement=float(np.linalg.norm(projection['tangent_displacement'],axis=1).max()),maximum_reprojection_correction=float(np.linalg.norm(projection['reprojection_correction'],axis=1).max()),displacements=np.linalg.norm(projection['tangent_displacement'],axis=1).tolist(),coordinate_hash=q.array_hash(high),capacity_authority=False)
        old_events={(e['kind'],e['metric'],e['entity']) for e in r['baseline_tails']['events']};new_events={(e['kind'],e['metric'],e['entity']) for e in r['best_tails']['events']}
        record['new_numeric_failures']=sorted(new_events-old_events);record['removed_numeric_failures']=sorted(old_events-new_events)
        record['outside_affected_failures']=r['best_tails']['outside_affected_columns']
        if case.spec['kind']=='spheres':
            record['conditioning_before']=conditioning(case,case.high);record['conditioning_after']=conditioning(case,high)
            surfaces,labels=q.surface_arrays(case.x,case.top.edges,case.prefix+[high]);packed=np.array([(7757<<32)|9024],np.uint64);ct=time.perf_counter()
            result,perf=q.prove(surfaces,labels,packed,q.Filter());cert_seconds+=time.perf_counter()-ct
            record['collision_pair']=dict(pair=[7757,9024],surfaces=surfaces[[7757,9024]].tolist(),labels=labels[[7757,9024]].tolist(),result=q.plain(result),complete_current_layer_coverage=False)
            model=np.load(c.OUT/case.name/'model_00.npz');u=model['u'];J=model['J'];h=float(np.cbrt(np.finfo(float).eps));checks=[]
            for j in (0,len(u)//2,len(u)-1):
                pp=u.copy();mm=u.copy();pp[j]+=h/2;mm[j]-=h/2
                half=(case.constraints(pp,model_only=True)-case.constraints(mm,model_only=True))/h
                checks.append(dict(dof=j,maximum_absolute_change=float(max(abs(half-J[:,j]))),norm_change=float(np.linalg.norm(half-J[:,j])),scope='Step-halving sensitivity diagnostic only; no acceptance tolerance'))
            record['finite_difference_consistency']=checks
            jacrows=[i for i,label in enumerate(r['constraint_labels']) if label[0]=='cell' and label[1]=='jacobian']
            record['jacobian_coupling']=[dict(column=r['constraint_labels'][i][2],baseline_scaled_value=float(model['values'][i]),positive_response_dofs=np.flatnonzero(J[i]>0).tolist(),negative_response_dofs=np.flatnonzero(J[i]<0).tolist()) for i in jacrows if model['values'][i]<=0]
        rows.append(c.w.clean(record))
    return rows,dict(seconds=time.perf_counter()-tick,certificate_seconds=cert_seconds,peak_rss=q.psutil.Process().memory_info().peak_wset)

if __name__=='__main__':
    a,t=run();b,u=run();assert q.identity(a)==q.identity(b)
    c.save('EXACT_DIAGNOSTICS.json',a);c.save('DIAGNOSTIC_TIMINGS.json',dict(first=t,replay=u));c.save('DIAGNOSTIC_DETERMINISM.json',dict(equal=True,scientific_hash=q.identity(a)))
