"""Bounded branch-aware cohort, using sealed front inputs and frozen gates."""
import json,time,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import qualify_general_sdf_witness as w
import branch_aware_projection_study as b
q=w.q;np=w.np;OUT=ROOT/'results/branch_aware_capacity_study'

def save(name,v):q.write(OUT/name,w.clean(v))

def prepare(old):
    s=q.load_schedule()
    if old['kind']=='spheres':
        x,f,core=q.build(old);spec=old;sdf=q.FixtureSDF(spec);last=3
    else:
        x,f,core,spec=w.extended_front(old);sdf=w.ExtendedSDF(spec)
        prior=json.loads((w.OUT/old['name']/'result.json').read_text());last=prior['last_valid_layer']
    top,p,base=q.foundation(x,f,core,sdf,spec);prefix=[];previous=None
    for k in range(1,last+1):
        if old['kind']=='spheres' or k>=6:
            folder=q.OUT if old['kind']=='spheres' else w.OUT
            high,_=q.load_checkpoint(folder/old['name'],k,top.sha256)
        else:
            high=q.predict_next_front(prefix[-1] if prefix else x,s.targets[k-1],sdf,iteration_cap=256).coordinates
            sealed,_=q.load_checkpoint(q.OUT/old['name'],k,top.sha256);high[p.vertices]=sealed[p.vertices]
            receipt=json.loads((w.OUT/old['name']/'result.json').read_text())['history'][k-1]
            assert q.array_hash(high)==receipt['coordinate_hash']
        metrics=q.immediate_metrics(top,prefix[-1] if prefix else x,high,k,previous);previous=metrics['cells'];prefix.append(high)
    return x,f,top,p,base,spec,sdf,prefix,previous

def steer(x,f,top,p,prefix,high,previous,sdf,spec,rec,schedule):
    tick=time.perf_counter();events=list(rec.get('failures',[]));k=len(prefix)
    events.extend(dict(kind='cell',metric='jacobian',layer=k+1,entity=e['column'],columns=[e['column']],value=e['jacobian']) for e in rec.get('exact_failures',[]))
    if not events:return [],None,dict(seconds=0.,certificate_seconds=0.)
    _,field,_=q.controlling_support(SimpleNamespace(coordinates=prefix[-1]),top,high,events)
    active=np.unique(np.concatenate([f[e['columns']].ravel() for e in events]));patch=q.bounded_patch(top,active)
    core=np.union1d(p.core_columns,patch.detailed.core_columns);_,verify,_=q.foundation(x,f,core,sdf,spec)
    modes=q.correction_fields(SimpleNamespace(coordinates=prefix[-1]),top,patch,field)
    more,_=w.search_fields(SimpleNamespace(coordinates=x),top,patch,prefix[-1],float(schedule.targets[-1]));modes+=more
    ancestry=b.owner(x[active],spec);history=[];seen=set();ct=0.;original=high.copy()
    for mode,direction in modes:
        for factor in (.5,1.,.25,.125,.0625,.03125,2.,4.):
            row=dict(mode=mode,factor=factor,active_vertices=active.tolist(),verification_columns=core.tolist())
            try:
                z=sdf.evaluate(high[active]);basis=w.tangent_basis(z.normal)
                field=np.einsum('nij,nj->ni',basis,np.einsum('nji,nj->ni',basis,direction));length=np.linalg.norm(field,axis=1)
                field=np.divide(field,length[:,None],out=np.zeros_like(field),where=length[:,None]!=0)
                seed=high[active]+float(factor*schedule.increments[k])*field;trial=high.copy()
                if spec['kind']=='spheres':trial[active],rr=b.sphere_project(seed,float(schedule.targets[k]),sdf,spec,ancestry)
                else:trial[active]=b.continue_front(seed,float(schedule.targets[k]),float(schedule.targets[k]),sdf,spec,ancestry).coordinates
                key=q.array_hash(trial)
                if key in seen:row.update(status='DuplicateCandidate',coordinate_hash=key);history.append(row);continue
                seen.add(key);r,m,c=w.checked(x,f,top,verify,prefix,trial,previous);ct+=c
                row.update(status='Evaluated',coordinate_hash=key,gate=r,passed=r['passed']);history.append(row)
                assert np.array_equal(high,original)
                if r['passed']:return history,(trial,r),dict(seconds=time.perf_counter()-tick,certificate_seconds=ct)
            except (RuntimeError,ValueError,np.linalg.LinAlgError) as exc:
                row.update(status='ProjectionOrEvaluationFailure',error=str(exc));history.append(row)
    assert np.array_equal(high,original)
    return history,None,dict(seconds=time.perf_counter()-tick,certificate_seconds=ct)

def run(persist=True):
    start=time.perf_counter();s=q.load_schedule();rows=[];timings=[]
    for old in q.definitions(s.targets):
        if old['kind'] not in ('spheres','wedge','frustum') or old.get('cavity'):continue
        tick=time.perf_counter();x,f,top,p,base,spec,sdf,prefix,previous=prepare(old);k=len(prefix);target=float(s.targets[k]);ancestry=b.owner(x,spec)
        ordinary=q.predict_next_front(prefix[-1],target,sdf,iteration_cap=256);baseline,_,ct=w.checked(x,f,top,p,prefix,ordinary.coordinates,previous)
        pt=time.perf_counter();projected=b.continue_front(prefix[-1],float(s.targets[k-1]),target,sdf,spec,ancestry);projection_seconds=time.perf_counter()-pt
        high=projected.coordinates;record,metrics,c=w.checked(x,f,top,p,prefix,high,previous);ct+=c
        diagnostic=q.immediate_metrics(top,prefix[-1],high,k+1,previous)
        diagnostic.pop('seconds')
        trials=[];accepted=None;st=dict(seconds=0.,certificate_seconds=0.)
        if not record['passed']:trials,accepted,st=steer(x,f,top,p,prefix,high,previous,sdf,spec,record,s)
        passed=record['passed'] or accepted is not None
        final=accepted[1] if accepted else record
        if passed:classification='BranchAwareSteeringResolved' if accepted else 'BranchAwareContinuationResolved'
        elif old['kind']=='spheres':classification='GeneralSDFWarningOnly'
        else:classification='QualityLimitNotCapacity'
        # Exact tangent derivative evidence is local, never an impossibility proof.
        from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import bottom_corner
        sensitivity=[]
        for e in record.get('exact_failures',[]):
            column=e['column'];ids=f[column];normals=sdf.evaluate(high[ids]).normal;basis=w.tangent_basis(normals)
            for corner in range(3):
                value=bottom_corner(prefix[-1][ids],high[ids],corner,basis[corner])
                sensitivity.append(dict(column=column,corner=corner,evidence=q.plain(value)))
        transitions=[]
        if old['kind']=='spheres':
            frozen=json.loads((w.OUT/'GENERAL_SDF.json').read_text())
            for v in frozen['failed_vertices']:
                tr,status=w.trace(prefix[-1][v],sdf,target,np.array(spec['centres']),np.array(spec['radii']))
                transitions.append(dict(vertex=v,current_layer=k,target=target,source=int(ancestry[v]),branch=b.branch_identity(spec,ancestry[v]),previous_owner=int(b.owner(prefix[-1][v:v+1],spec)[0]),history=tr,status=status,branch_aware_history=[dict(step=h['step'],target=h['target'],coordinates=h['coordinates'][v],owner=h['owners_after'][v],source_record=h['source_records'][v]) for h in projected.trace],classification='BranchAmbiguous',reason='Intersecting offset-sphere branches have multiple admitted source-consistent directions; owner switches alone do not establish an unintended jump.'))
        row=dict(name=old['name'],foundation=base,input_layer=k,target=target,baseline=baseline,branch_aware_gate=record,diagnostic_metrics_after_rejection=diagnostic,projected_coordinate_hash=q.array_hash(high),source_identity=q.identity(spec),source_ancestry=ancestry.tolist(),projection_trace=projected.trace,ownership_transitions=transitions,steering=trials,steering_success=accepted is not None,final_gate=final,classification=classification,last_valid_layer=k+1 if passed else k,first_unadmitted_layer=None if passed else k+1,capacity_authority=False,direct_unavoidable_obstruction=False,tangent_evidence=sensitivity,scope='Bounded source-consistent continuation; failure is not proof over all admissible tangent motions',rollback_input_unchanged=True)
        rows.append(row);timings.append(dict(name=old['name'],seconds=time.perf_counter()-tick,projection_seconds=projection_seconds,steering_seconds=st['seconds'],certificate_seconds=ct+st['certificate_seconds']))
        print(old['name'],classification,'exact failures',len(record.get('exact_failures',[])),'trials',len(trials),flush=True)
        if persist:
            save(old['name']+'.json',row);np.save(OUT/(old['name']+'_candidate.npy'),high)
            if accepted:np.save(OUT/(old['name']+'_accepted.npy'),accepted[0])
    return rows,dict(seconds=time.perf_counter()-start,cases=timings,peak_rss=q.psutil.Process().memory_info().peak_wset)

if __name__=='__main__':
    rows,t=run();save('TIMINGS.json',t);save('SUMMARY.json',[{k:r[k] for k in ('name','classification','last_valid_layer','first_unadmitted_layer','capacity_authority','projected_coordinate_hash')} for r in rows])
    replay,rt=run(False);equal=q.identity(w.clean(rows))==q.identity(w.clean(replay));assert equal
    save('DETERMINISM.json',dict(equal=equal,scientific_hash=q.identity(w.clean(rows))));save('REPLAY_TIMINGS.json',rt)
