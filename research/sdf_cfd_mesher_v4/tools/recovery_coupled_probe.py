"""Adapter for the existing local HiGHS tangent solver at variable SDF targets."""
import copy
from types import SimpleNamespace
import compressed_layer_recovery as r
c=r.c;q=r.q;np=r.np;g=r.g;b=r.b;w=r.w

class RecoveryCase(c.Case):
    def __init__(self,base,prefix,previous,targets,active):
        self.__dict__.update(copy.copy(base.__dict__))
        self.prefix=prefix;self.previous=previous;self.low=prefix[-1];self.k=len(prefix)
        self.local_targets=targets;self.target=float(self.s.targets[self.k]);self.active=active
        self.high,_,_=r.project(self,self.low,targets,active)
        self.patch=q.bounded_patch(self.top,active);self.local=self.patch.detailed.core_columns;self.edges=self.patch.detailed.side_edges
        self.basis=w.tangent_basis(self.sdf.evaluate(self.high[active]).normal)
        _,self.radii=w.search_fields(SimpleNamespace(coordinates=self.x),self.top,self.patch,self.low,float(self.s.targets[-1]))
        self.ancestry=b.owner(self.x[active],self.spec);self.pair=[];self.evaluations=0
        area=np.linalg.norm(np.cross(self.low[self.f[:,1]]-self.low[self.f[:,0]],self.low[self.f[:,2]]-self.low[self.f[:,0]]),axis=1)
        self.scale=area*float(self.s.increments[self.k]);self.scale=np.where(self.scale>0,self.scale,1.)
        self.support=dict(active_vertices=active,affected_columns=self.patch.affected_columns,verification_ring=self.patch.verification_ring,tangent_dofs=2*len(active),fixed_lower=True,target_hash=q.array_hash(targets))
    def project(self,u,model_only=False):
        u=np.asarray(u).reshape(-1,2);delta=np.einsum('nij,nj->ni',self.basis,u*self.radii[:,None]);points=self.high[self.active]+delta;high=self.high.copy()
        if model_only:
            centres=np.array(self.spec['centres'])[self.ancestry];radius=np.array(self.spec['radii'])[self.ancestry]+self.local_targets[self.active]
            direction=points-centres;high[self.active]=centres+radius[:,None]*direction/np.linalg.norm(direction,axis=1)[:,None]
        else:
            for target in np.unique(self.local_targets[self.active]):
                local=np.flatnonzero(self.local_targets[self.active]==target);ids=self.active[local]
                high[ids],_=b.sphere_project(points[local],float(target),self.sdf,self.spec,self.ancestry[local])
        return high,dict(tangent_displacement=delta,reprojection_correction=high[self.active]-points,source_records=None)
    def constraints(self,u,labels=False,model_only=False):
        vals,names=super().constraints(u,True,model_only);high,_=self.project(u,model_only)
        extra=[]
        for j,(centre,radius) in enumerate(zip(self.spec['centres'],self.spec['radii'])):
            ids=self.active[self.ancestry!=j];extra.extend((np.linalg.norm(high[ids]-centre,axis=1)-radius-self.local_targets[ids])/self.s.increments[self.k])
            names.extend(('branch','other_source_clearance_'+str(j),int(i)) for i in ids)
        vals=np.r_[vals,extra];return (vals,names) if labels else vals

def probe(base,prefix,previous,targets,active,iterations=12):
    case=RecoveryCase(base,prefix,previous,targets,active);row,t=c.solve(case,iterations,persist=False)
    high,projection=case.project(row['u']);sample=case.sdf.evaluate(high)
    row.update(target=targets,maximum_residual=float(max(abs(sample.phi-targets))),source_target_admitted=bool(np.all(abs(sample.phi-targets)<=sample.source_error_bound)))
    return high,w.clean(row),t

if __name__=='__main__':
    import sys
    case,prefix,history,previous,seed=r.seed();old=r.load(r.OUT/'result.json');exp=old['expansions'][-1]
    active=np.array(exp['active']);weights=g.harmonic_weights(case.top,exp['centres'],active)
    p=r.plan(case.s,history,'receding',5,weights);targets=history[-1]+p['height']*(1-(1-.125)*weights);targets[weights==0]=case.s.targets[4]
    high,row,t=probe(case,prefix,previous,targets,active)
    if '--replay' in sys.argv:
        old=r.load(r.OUT/'COUPLED_PROBE.json');equal=q.identity(old)==q.identity(row)
        r.save('COUPLED_DETERMINISM.json',dict(equal=equal,first=q.identity(old),replay=q.identity(row)));assert equal
        r.save('COUPLED_REPLAY_TIMING.json',t)
    else:r.save('COUPLED_PROBE.json',row);r.save('COUPLED_TIMING.json',t);np.save(r.OUT/'coupled_candidate.npy',high)
