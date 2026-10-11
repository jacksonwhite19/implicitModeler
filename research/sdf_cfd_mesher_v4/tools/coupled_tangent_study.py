"""Study-only simultaneous tangent feasibility; no production call site."""
import json,time,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'src')]
import qualify_branch_aware_capacity as d
q=d.q;w=d.w;b=d.b;np=d.np
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import tangent_rank
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import adjust_trust
from scipy.optimize import linprog
OUT=ROOT/'results/coupled_tangent_feasibility'
def save(name,value):q.write(OUT/name,w.clean(value))

def trust_boundary(step,radius):
    # Same 0.8 boundary-use rule as the frozen trust_region.stage_assessment.
    return bool(np.max(abs(step))/(radius/np.sqrt(2))>=.8)

class Case:
    def __init__(self,spec):
        self.name=spec['name'];self.s=q.load_schedule()
        self.x,self.f,self.top,self.p,self.base,self.spec,self.sdf,self.prefix,self.previous=d.prepare(spec)
        self.k=len(self.prefix);self.low=self.prefix[-1];self.target=float(self.s.targets[self.k])
        self.high=np.load(d.OUT/(self.name+'_candidate.npy'));self.frozen= json.loads((d.OUT/(self.name+'.json')).read_text())
        r=self.frozen['branch_aware_gate'];cols={e['column'] for e in r.get('exact_failures',[])}
        for e in r.get('failures',[]):cols.update(e['columns'])
        controlling=set(map(int,self.f[sorted(cols)].ravel()));self.pair=[];self.sides=[]
        if self.spec['kind']=='spheres':
            self.pair=[7757,9024];E=len(self.top.edges)
            assert all(side//E==self.k for side in self.pair)
            for side in self.pair:
                ids=self.top.edges[side%E];controlling.update(map(int,ids));self.sides.append(ids)
        active=set(controlling)
        for v in sorted(controlling):active.update(map(int,self.top.vertex_neighbors.row(v)))
        self.active=np.array(sorted(active),int);self.patch=q.bounded_patch(self.top,self.active)
        self.core=np.union1d(self.p.core_columns,self.patch.detailed.core_columns)
        _,self.verify,_=q.foundation(self.x,self.f,self.core,self.sdf,self.spec)
        self.basis=w.tangent_basis(self.sdf.evaluate(self.high[self.active]).normal)
        self.rank=tangent_rank(self.sdf.evaluate(self.high[self.active]).normal)
        _,self.radii=w.search_fields(SimpleNamespace(coordinates=self.x),self.top,self.patch,self.low,float(self.s.targets[-1]))
        self.ancestry=b.owner(self.x[self.active],self.spec)
        self.local=self.patch.detailed.core_columns
        # Only changed/dependency-ring outputs guide search. All original core
        # and expanded verification columns are checked for final acceptance.
        self.edges=self.patch.detailed.side_edges
        area=np.linalg.norm(np.cross(self.low[self.f[:,1]]-self.low[self.f[:,0]],self.low[self.f[:,2]]-self.low[self.f[:,0]]),axis=1)
        self.scale=area*float(self.s.increments[self.k]);self.scale=np.where(self.scale>0,self.scale,1.)
        self.last_project=None;self.evaluations=0
        if self.pair:
            cs=np.array(self.spec['centres']);own=b.owner(self.x,self.spec)
            axis=cs[own[self.sides[1][0]]]-cs[own[self.sides[0][0]]];self.axis=axis/np.linalg.norm(axis)
        self.support=dict(active_vertices=self.active.tolist(),controlling_vertices=sorted(controlling),fixed_boundary=self.patch.fixed_boundary.tolist(),halo_vertices=self.patch.halo.tolist(),affected_columns=self.patch.affected_columns.tolist(),verification_ring=self.patch.verification_ring.tolist(),verification_columns=self.core.tolist(),metric_columns=self.patch.detailed.columns.tolist(),side_faces=self.edges.tolist(),tangent_dofs=2*len(self.active),rank=self.rank,collision_pair=self.pair,collision_side_vertices=[v.tolist() for v in self.sides],immutable_bottom_vertices=[v.tolist() for v in self.sides],basis=self.basis.tolist(),source_ancestry=self.ancestry.tolist(),geometric_radii=self.radii.tolist(),radius_authority='Unchanged search_fields: min(0.25 stored total thickness, 0.5 shortest wall neighbor edge)',all_controlling_vertices_active=controlling<=active,topology_hash=self.top.sha256)

    def project(self,u,model_only=False):
        u=np.asarray(u).reshape(-1,2);delta=np.einsum('nij,nj->ni',self.basis,u*self.radii[:,None]);seed=self.high[self.active]+delta
        high=self.high.copy()
        if self.spec['kind']=='spheres' and model_only:
            centres=np.array(self.spec['centres'])[self.ancestry];radius=np.array(self.spec['radii'])[self.ancestry]+self.target
            direction=seed-centres;high[self.active]=centres+radius[:,None]*direction/np.linalg.norm(direction,axis=1)[:,None];records=None
        elif self.spec['kind']=='spheres':high[self.active],records=b.sphere_project(seed,self.target,self.sdf,self.spec,self.ancestry)
        else:
            pred=b.continue_front(seed,self.target,self.target,self.sdf,self.spec,self.ancestry);high[self.active]=pred.coordinates;records=None
        return high,dict(tangent_displacement=delta,reprojection_correction=high[self.active]-seed,source_records=records)

    def metrics(self,high):
        with np.errstate(all='ignore'):return q.immediate_metrics(self.top,self.low,high,self.k+1,self.previous)

    def constraints(self,u,labels=False,model_only=False):
        high,_=self.project(u,model_only);m=self.metrics(high);self.evaluations+=1;vals=[];names=[]
        def add(kind,name,ids,value):
            vals.extend(value);names.extend((kind,name,int(i)) for i in ids)
        ids=self.local;c=m['cells']
        for name in ('volume','signed_volume','jacobian'):add('cell',name,ids,c[name][ids]/self.scale[ids])
        add('cell','determinant',ids,c['determinant'][ids]/.0011-1)
        add('cell','aspect',ids,1-c['aspect'][ids]/1000)
        for kind,ids in [('side',self.edges),('cap',self.local)]:
            a=m['sides' if kind=='side' else 'caps']
            add(kind,'weight',ids,a['weight'][ids]/.055-1);add(kind,'skewness',ids,1-a['skewness'][ids]/3.63636);add(kind,'symmetric_volume_ratio',ids,a['symmetric_volume_ratio'][ids]/.01-1)
        if self.pair:
            A=np.r_[self.low[self.sides[0]],high[self.sides[0]]];B=np.r_[self.low[self.sides[1]],high[self.sides[1]]]
            gap=min(B@self.axis)-max(A@self.axis)
            add('diagnostic','fixed_axis_separation',[0],np.array([gap/float(self.s.increments[self.k])]))
            for j,(centre,radius) in enumerate(zip(self.spec['centres'],self.spec['radii'])):
                ids=self.active[self.ancestry!=j]
                add('branch','other_source_clearance_'+str(j),ids,(np.linalg.norm(high[ids]-centre,axis=1)-radius-self.target)/float(self.s.increments[self.k]))
        a=np.array(vals,float)
        # Nonfinite geometry is never accepted. An unavailable local derivative
        # is a numerical-model limitation, not a substituted mesh metric.
        if not np.all(np.isfinite(a)):raise ValueError('NonfiniteSensitivityInput')
        return (a,names) if labels else a

    def verify_candidate(self,u):
        high,projection=self.project(u);tick=time.perf_counter();rec,m,ct=w.checked(self.x,self.f,self.top,self.verify,self.prefix,high,self.previous)
        return high,rec,projection,time.perf_counter()-tick

    def tails(self,high):
        m=self.metrics(high);out={}
        for name in ('jacobian','determinant','signed_volume','volume','aspect'):
            a=m['cells'][name][self.core];out[name]=dict(min=float(np.nanmin(a)),max=float(np.nanmax(a)),nonfinite=int(np.sum(~np.isfinite(a))))
        for name in ('weight','skewness','symmetric_volume_ratio'):
            a=np.r_[m['sides'][name][self.verify.side_edges],m['caps'][name][self.core]];out[name]=dict(min=float(np.nanmin(a)),max=float(np.nanmax(a)),nonfinite=int(np.sum(~np.isfinite(a))))
        events=[e for e in m['events'] if e['entity'] in (self.verify.side_edges if e['kind']=='side' else self.core)]
        out['events']=events;out['outside_affected_columns']=[e for e in events if not set(e['columns'])&set(self.patch.affected_columns.tolist())]
        return out

def model(case,u):
    tick=time.perf_counter();f,names=case.constraints(u,True);h=float(np.cbrt(np.finfo(float).eps));J=np.empty((len(f),len(u)))
    for j in range(len(u)):
        plus=u.copy();minus=u.copy();plus[j]+=h;minus[j]-=h;J[:,j]=(case.constraints(plus,model_only=True)-case.constraints(minus,model_only=True))/(2*h)
    return f,J,names,dict(seconds=time.perf_counter()-tick,difference_step=h,method='Central finite differences of source-radial smooth precursor for spheres, qualified projection for channels; other-source inequalities constrain branch. Model probes are not admitted candidates or exact feasibility proof.')

def solve(case,iterations=16,persist=True,seed='zero'):
    started=time.perf_counter();n=2*len(case.active);u=np.zeros(n);trust=.25;history=[];models=[];sensitivity_seconds=0.;verify_seconds=0.;best=None
    if seed!='zero':
        axis=np.tile([0.,0.,1.],(len(case.active),1));axis[case.ancestry%2==0]*=-1
        if seed=='opposed_minus_z':axis=-axis
        elif seed!='opposed_z':raise ValueError('UnknownDeterministicSeed')
        uv=np.einsum('nji,nj->ni',case.basis,axis);norm=np.linalg.norm(uv,axis=1)
        u=(.25*np.divide(uv,norm[:,None],out=np.zeros_like(uv),where=norm[:,None]!=0)).ravel()
    try:f=case.constraints(u)
    except ValueError as exc:
        high,rec,projection,vt=case.verify_candidate(u)
        row=w.clean(dict(name=case.name,seed=seed,support=case.support,classification='Blocked',feasible=False,infeasibility_proved=False,model_blocker=str(exc),best_gate=rec,history=[],models=[],u=u.tolist(),coordinate_hash=q.array_hash(high),baseline_tails=case.tails(high),best_tails=case.tails(high),projection=projection,last_sealed_valid=case.k,study_committed=False,unchanged_outside_active=True,scope='Nonfinite required model inputs; support audit identifies any immutable lower-face defects. No capacity conclusion.'))
        timing=dict(seconds=time.perf_counter()-started,sensitivity_seconds=0.,verification_seconds=vt,evaluations=case.evaluations,iterations=0,peak_rss=q.psutil.Process().memory_info().peak_wset)
        if persist:save(case.name+'/result.json',row);np.save(OUT/case.name/'candidate.npy',high)
        return row,timing
    score=float(min(f));baseline=score;labels=None;cached_key=None;cached_model=None
    for it in range(iterations):
        key=q.array_hash(u)
        if key==cached_key:
            f,J,labels,oldperf=cached_model;perf=dict(oldperf,seconds=0.)
        else:
            f,J,labels,perf=model(case,u);cached_key=key;cached_model=(f,J,labels,perf)
        sensitivity_seconds+=perf['seconds']
        # Maximize common normalized slack, not a weighted quality objective.
        # The strict hard-gate decision is made only by the frozen validator.
        limit=trust/np.sqrt(2);bounds=[(max(-1/np.sqrt(2)-v,-limit),min(1/np.sqrt(2)-v,limit)) for v in u]+[(None,None)]
        slack_column=np.array([0. if name[0]=='branch' else 1. for name in labels])
        result=linprog(np.r_[np.zeros(n),-1.],A_ub=np.c_[-J,slack_column],b_ub=f,bounds=bounds,method='highs-ds')
        row=dict(iteration=it,trust_before=trust,score_before=score,LP_status=int(result.status),LP_message=result.message)
        if persist:
            folder=OUT/case.name;folder.mkdir(exist_ok=True);np.savez_compressed(folder/f'model_{it:02}.npz',u=u,values=f,J=J)
        models.append(dict(iteration=it,identity=q.identity(dict(u=q.array_hash(u),values=q.array_hash(f),J=q.array_hash(J))),**{k:v for k,v in perf.items() if k!='seconds'}))
        if not result.success:history.append(row);break
        du=result.x[:-1];trial=u+du
        try:
            ft=case.constraints(trial);newscore=float(min(ft));high,rec,projection,vt=case.verify_candidate(trial);verify_seconds+=vt
            progress=newscore>score;gain=float(min(f+J@du)-score);ratio=(newscore-score)/gain if gain>0 else None
            oldtrust=trust;trust,change,why=adjust_trust(trust,progress,prediction_ratio=ratio,boundary=trust_boundary(du,trust))
            row.update(score_after=newscore,search_step_accepted=progress,mesh_accepted=rec['passed'],gate=rec,coordinate_hash=q.array_hash(high),trust_after=trust,trust_action=change,trust_reason=why,linearized_minimum=float(min(f+J@du)),maximum_tangent_displacement=float(np.linalg.norm(projection['tangent_displacement'],axis=1).max()))
            if progress:
                u=trial;score=newscore;best=(high,rec,projection,u.copy())
            if rec['passed']:
                best=(high,rec,projection,trial.copy());history.append(row);break
        except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
            trust,change,why=adjust_trust(trust,False);row.update(projection_or_evaluation_failure=str(exc),search_step_accepted=False,mesh_accepted=False,trust_after=trust,trust_action=change)
        history.append(row)
        print(case.name,it,'score',score,'trust',trust,flush=True)
    if best is None:
        high,rec,projection,vt=case.verify_candidate(u);verify_seconds+=vt;best=high,rec,projection,u
    high,rec,projection,u=best;z=case.sdf.evaluate(high);tails=case.tails(high);base_tails=case.tails(case.high)
    classification=('CoupledTangentFeasible' if case.spec['kind']=='spheres' else 'CoupledTangentResolved') if rec['passed'] else 'CoupledTangentNoSolutionFound'
    row=dict(name=case.name,seed=seed,support=case.support,history=history,models=models,constraint_labels=labels,baseline_score=baseline,best_score=score,classification=classification,feasible=rec['passed'],infeasibility_proved=False,best_gate=rec,baseline_tails=base_tails,best_tails=tails,coordinate_hash=q.array_hash(high),u=u.tolist(),projection=projection,maximum_residual=float(abs(z.phi-case.target).max()),target=case.target,last_sealed_valid=case.k,study_committed=rec['passed'],unchanged_outside_active=bool(np.array_equal(high[np.setdiff1d(np.arange(len(high)),case.active)],case.high[np.setdiff1d(np.arange(len(high)),case.active)])),scope='Computationally bounded local search. LP slack and finite differences are not infeasibility certificates.')
    timing=dict(seconds=time.perf_counter()-started,sensitivity_seconds=sensitivity_seconds,verification_seconds=verify_seconds,evaluations=case.evaluations,iterations=len(history),peak_rss=q.psutil.Process().memory_info().peak_wset)
    if persist:save(case.name+'/result.json',row);np.save(OUT/case.name/'candidate.npy',high)
    return w.clean(row),timing

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--case');parser.add_argument('--iterations',type=int,default=16);parser.add_argument('--replay',action='store_true');parser.add_argument('--seed',default='zero');args=parser.parse_args()
    for spec in q.definitions(q.load_schedule().targets):
        if spec['kind'] not in ('wedge','frustum','spheres') or spec.get('cavity') or (args.case and spec['name']!=args.case):continue
        c=Case(spec)
        if args.seed!='zero':c.name+='__'+args.seed
        print(c.name,len(c.active),c.rank,flush=True)
        a,t=solve(c,args.iterations,seed=args.seed);save(c.name+'/timing.json',t)
        if args.replay:
            cc=Case(spec);cc.name=c.name
            r,rt=solve(cc,args.iterations,False,seed=args.seed);assert q.identity(a)==q.identity(r);save(c.name+'/determinism.json',dict(equal=True,scientific_hash=q.identity(a)));save(c.name+'/replay_timing.json',rt)
