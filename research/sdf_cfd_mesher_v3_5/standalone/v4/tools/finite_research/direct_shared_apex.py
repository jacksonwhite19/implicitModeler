"""Bounded generic interior shared-apex repair; source predicates decide commits."""
from fractions import Fraction as F
from itertools import combinations
import math,time,copy
import numpy as np
from scipy.optimize import minimize
from background_mixed_geometry import batch_geometry
from finite_support.transition_front_targets import tetrahedron_ratio,population_report
from sdf_cfd_mesher_v4.common.authority import array_hash,identity
import local_termination_topology as topology
from prism_hotpath_v2 import batch_quality

METHOD=dict(schema='GenericInteriorSharedApexRepair/231',maximum_vertices=32,solver_stages=2,maxiter=120,
    maximum_search_evaluations_per_vertex=8192,maximum_search_seconds_per_vertex=30,maximum_construction_seconds=3600,
    movement='Euclidean ball radius half the shortest incident apex-edge length; fixed before solve; strict stored-coordinate displacement norm',
    solver='SLSQP; stage1 minimum normalized displacement; stage2 interface .6 deficit plus1e-4 displacement penalty',
    search_reserve=float(64*np.sqrt(np.finfo(float).eps)),ftol=1e-12,
    acceptance='Strict fresh batch AND scalar source QA, exact signs/star volume, immutable boundary/prefix, mandatory cumulative mean-ratio minimum and local weight improvement; percentage counts are diagnostics. No solver-status authority.',
    order='Unique eligible apices sorted by vertex index derived from actual failed interface faces; one pass, refresh complete star/halo before each solve')

class Refusal(ValueError):pass
def require(ok,label):
    if not ok:raise Refusal(label)
def key(face):return tuple(sorted(map(int,face)))
def counts(labels):return labels.sum(axis=0).astype(int)
def movement_allowed(before,after,radius):
    before=np.asarray(before,dtype=float);after=np.asarray(after,dtype=float)
    return bool(before.shape==after.shape==(3,) and np.isfinite(before).all() and np.isfinite(after).all()
        and math.isfinite(radius) and radius>0 and float(np.linalg.norm(after-before))<=radius)
def thresholds(policy):return (float(policy['quality_targets']['outer_tetrahedra']['minimum']),*[float(r['minimum']) for r in policy['quality_targets']['outer_tetrahedra']['population_targets']])
def exact_labels(points,tets,limits):
    result=[]
    for row in tets:
        p=[[F(float(x)) for x in points[v]] for v in row]
        u=[[p[j][i]-p[0][i] for i in range(3)] for j in (1,2,3)]
        six=topology.dot(topology.cross(u[0],u[1]),u[2]);edges=sum((p[i][k]-p[j][k])**2 for i,j in combinations(range(4),2) for k in range(3))
        result.append([six>0 and edges>0 and 432*six*six>=F(t)**3*edges**3 for t in limits])
    return np.asarray(result,dtype=bool)

class Context:
    def __init__(self,arrays,state,policy,check=lambda:None):
        from finite_research.direct_contract import validate_policy
        validate_policy(policy)
        self.original=state['points'];self.points=self.original.copy();self.arrays=arrays;self.state=state;self.policy=policy
        self.cells={c['id']:c for c in state['cells']};self.owners=state['owners'];self.tets=arrays['background_tets'];self.ids=arrays['tetrahedron_ids']
        self.row={int(v):i for i,v in enumerate(self.ids)};self.prefix=set(state['prefix_ids']);self.cap={key(f) for f in arrays['cap']}
        self.protected=set(range(state['prefix_points']))|set(map(int,np.r_[arrays['wall'].ravel(),arrays['cap'].ravel(),arrays['farfield'].ravel(),arrays['prisms'].ravel()]))
        interface=set()
        for face in arrays['cap']:
            uses=self.owners[key(face)];old=[i for i,_ in uses if i in self.prefix];new=[i for i,_ in uses if i not in self.prefix]
            require(len(uses)==2 and len(old)==len(new)==1,'ActualInterfaceOwnerRoles');require(new[0] in self.row,'InterfaceTetRequired');interface.add(new[0])
        self.interface=np.asarray([int(i) in interface for i in self.ids]);self.limits=np.asarray(thresholds(policy))
        self.q=tetrahedron_ratio(self.points,self.tets);require(np.isfinite(self.q).all(),'FiniteOriginalShape')
        self.labels=self.q[:,None]>=self.limits;self.original_interface=counts(self.labels[self.interface])
        target=policy['quality_targets']['outer_tetrahedra'];self.required=np.asarray([len(self.tets),*[math.ceil(F(str(r['fraction']))*len(self.tets)) for r in target['population_targets']]])
        require(population_report(self.q,target)['minimum_passed'],'OriginalBulkMinimumMustPass')
        self.version=0;self.expected=array_hash(self.points);self.changed=set();self.affected_union=set()
        self.payload_check=check;self.construction_started=time.monotonic()
    def check(self):
        self.payload_check()
        if time.monotonic()-self.construction_started>=METHOD['maximum_construction_seconds']:raise TimeoutError('FrozenConstructionDeadline')
    def select(self,quality):
        candidates=set();refused=[]
        for failure in quality['failures']:
            if failure['kind']!='face_quality' or not failure['weight']<.055:continue
            face=key(failure['vertices'])
            if face not in self.cap:refused.append(dict(face=face,reason='NotInterfaceWeightFailure'));continue
            uses=self.owners[face];bulk=[self.cells[i] for i,_ in uses if i not in self.prefix]
            require(len(bulk)==1 and bulk[0]['kind']=='tetrahedron','SharedApexTetOwner')
            other=set(bulk[0]['vertices'])-set(face);require(len(other)==1,'UniqueTetApex');v=other.pop()
            if v in self.protected:refused.append(dict(face=face,reason='ProtectedApex'));continue
            candidates.add(v)
        require(len(candidates)<=METHOD['maximum_vertices'],'FrozenMaximum32Vertices')
        return sorted(candidates),refused
    def star(self,vertex):
        require(type(vertex) is int and vertex not in self.protected,'InteriorBulkVertexRequired')
        rows=np.flatnonzero(np.any(self.tets==vertex,axis=1));require(len(rows)>0,'NonemptyCompleteStar')
        selected={int(self.ids[r]) for r in rows}
        actual={i for i,c in self.cells.items() if vertex in c['vertices']};require(actual==selected,'CompleteIncidentBulkOnlyStar')
        faces={key(f) for i in selected for f in self.cells[i]['faces']};require(all(k in self.owners for k in faces),'MissingStarOwnerCycle');owners={k:self.owners[k] for k in sorted(faces)}
        require(all(len(v)==2 for v in owners.values()),'InternalCompleteStarFaceClosure')
        require(all(topology.reverse_cycle(uses[0][1],uses[1][1]) and all(i in self.cells and tuple(f) in self.cells[i]['faces'] and key(f)==k for i,f in uses) for k,uses in owners.items()),'ActualStarOwnerCyclesAndHalo')
        needed={i for uses in owners.values() for i,_ in uses};cells=[self.cells[i] for i in sorted(needed)]
        vertices=np.asarray(sorted({v for c in cells for v in c['vertices']}),dtype='<i8');mapping={int(v):j for j,v in enumerate(vertices)}
        local=[]
        for c in cells:
            d=dict(c,vertices=[mapping[v] for v in c['vertices']],faces=[tuple(mapping[v] for v in f) for f in c['faces']]);local.append(d)
        localowners={key([mapping[v] for v in k]):[(i,tuple(mapping[v] for v in f)) for i,f in uses] for k,uses in owners.items()}
        localtets=np.asarray([[mapping[int(v)] for v in self.tets[r]] for r in rows],dtype='<i8')
        neighbour_vertices=sorted({int(v) for r in rows for v in self.tets[r] if v!=vertex})
        distances=np.linalg.norm(self.points[neighbour_vertices]-self.points[vertex],axis=1);radius=.5*float(distances.min());require(np.isfinite(radius) and radius>0,'PositiveLocalMovementDomain')
        return Star(self,vertex,rows,selected,vertices,local,localowners,localtets,mapping[vertex],radius)

class Star:
    def __init__(self,ctx,vertex,rows,selected,vertices,cells,owners,tets,local_vertex,radius):
        self.ctx=ctx;self.vertex=vertex;self.rows=rows;self.selected=selected;self.vertices=vertices;self.cells=cells;self.owners=owners;self.tets=tets;self.lv=local_vertex;self.radius=radius
        self.base=ctx.points[vertices].copy();self.base_version=ctx.version;self.base_hash=ctx.expected
        self.selected_cells=[c for c in cells if c['id'] in selected];self.face_rows=list(owners.values());self.faceverts=np.asarray([uses[0][1] for uses in self.face_rows],dtype='<i8')
        require(self.faceverts.ndim==2 and self.faceverts.shape[1]==3,'TetStarTriangularFaces')
        self.bulk_unaffected=counts(ctx.labels)-counts(ctx.labels[rows]);self.iface_mask=ctx.interface[rows]
        self.iface_unaffected=counts(ctx.labels[ctx.interface])-counts(ctx.labels[rows][self.iface_mask])
        p=self.base[tets];self.initial_six=np.einsum('ni,ni->n',p[:,1]-p[:,0],np.cross(p[:,2]-p[:,0],p[:,3]-p[:,0]));require(np.all(self.initial_six>0),'PositiveOriginalStar')
        self.old_exact=sum((F(topology.exact_cell(self.base,c)['signed_volume']) for c in self.selected_cells),F())
        self.evaluations=0;self.cache={};self.search_started=time.monotonic()
    def coordinates(self,z):
        x=self.base.copy();x[self.lv]=self.base[self.lv]+self.radius*np.asarray(z,dtype=float);return x
    def search(self,z):
        self.ctx.check()
        z=np.asarray(z,dtype=float);cachekey=z.tobytes()
        if cachekey in self.cache:return self.cache[cachekey]
        if self.evaluations>=METHOD['maximum_search_evaluations_per_vertex']:raise RuntimeError('FrozenSearchEvaluationLimit')
        if time.monotonic()-self.search_started>=METHOD['maximum_search_seconds_per_vertex']:raise TimeoutError('FrozenVertexSearchDeadline')
        self.evaluations+=1;x=self.coordinates(z);geometry={}
        with np.errstate(all='ignore'):
            for arity in {tuple(map(len,c['faces'])) for c in self.cells}:
                group=[c for c in self.cells if tuple(map(len,c['faces']))==arity];geometry.update(batch_geometry(x,group))
            cq=topology.q.legacy_modules()['component_quality'];area,centre=cq.FG.face_area_centre_tri(x[self.faceverts])
            a=[geometry[uses[0][0]] for uses in self.face_rows];b=[geometry[uses[1][0]] for uses in self.face_rows]
            metrics=np.column_stack(cq.face_metrics(x[self.faceverts],area,centre,np.asarray([g['centre'] for g in a]),np.asarray([g['centre'] for g in b]),np.asarray([g['volume'] for g in a]),np.asarray([g['volume'] for g in b])))
            det=[];aspect=[];volume=[]
            for c in self.selected_cells:
                g=geometry[c['id']];S=g['areas'];scaled=S/np.linalg.norm(S,axis=1).mean();det.append(float(np.linalg.det(scaled.T@scaled)));aspect.append(g['aspect']);volume.append(g['volume'])
            p=x[self.tets];six=np.einsum('ni,ni->n',p[:,1]-p[:,0],np.cross(p[:,2]-p[:,0],p[:,3]-p[:,0]));edge=sum(np.sum((p[:,i]-p[:,j])**2,axis=1) for i,j in combinations(range(4),2))
            q=12*np.sign(six)*(abs(six)*.5)**(2/3)/edge
            reserve=METHOD['search_reserve'];residual=[*list(metrics[:,0]/.055-1-reserve),*list(1-metrics[:,1]/3.63636-reserve),*list(metrics[:,2]/.01-1-reserve),*list(np.asarray(det)/.0011-1-reserve),*list(1-np.asarray(aspect)/1000-reserve),*list(six/self.initial_six-reserve),*list(q/self.ctx.limits[0]-1-reserve),1-float(z@z)]
            for j,t in enumerate(self.ctx.limits[:1]):
                for values,unaffected,required in ((q,self.bulk_unaffected[j],self.ctx.required[j]),(q[self.iface_mask],self.iface_unaffected[j],self.ctx.original_interface[j])):
                    need=max(0,int(required-unaffected))
                    if need:residual.append(float(np.sort(values)[-need]/t-1-reserve))
            residual=np.asarray(residual);loss=float(np.mean(np.maximum(0,max(self.ctx.limits)-q[self.iface_mask])**2)) if self.iface_mask.any() else 0.
        valid=np.isfinite(residual).all() and np.isfinite(metrics).all() and np.isfinite(loss)
        data=dict(residual=residual if valid else np.full(len(residual),-1e6),loss=loss if valid else 1e6,metrics=metrics,q=q)
        self.cache={cachekey:data};return data
    def final(self,z):
        self.ctx.check()
        ctx=self.ctx;z=np.asarray(z,dtype=float)
        require(z.shape==(3,) and np.isfinite(z).all() and float(z@z)<=1+8*np.finfo(float).eps,'FrozenMovementDomain')
        x=self.coordinates(z);batch=batch_quality(x,self.cells,self.owners,0,active_cells=self.selected,mixed_cells=True,compact=True,face_batch_size=4096)
        scalar=topology.quality(x,self.cells,self.owners,0,active_cells=self.selected)
        q=tetrahedron_ratio(x,self.tets);require(np.isfinite(q).all(),'FiniteFinalShape');labels=(q[:,None]>=ctx.limits)&exact_labels(x,self.tets,ctx.limits)
        bulk=self.bulk_unaffected+counts(labels);interface=self.iface_unaffected+counts(labels[self.iface_mask])
        volume=sum((F(topology.exact_cell(x,c)['signed_volume']) for c in self.selected_cells),F())
        passed=bool(batch['passed'] and scalar['passed'] and movement_allowed(self.base[self.lv],x[self.lv],self.radius)
            and np.all(q>=ctx.limits[0]) and volume==self.old_exact and bulk[0]>=ctx.required[0] and np.all(labels[:,0]))
        return dict(passed=passed,coordinates=x,z=z,q=q,labels=labels,batch=batch,scalar=scalar,bulk_counts=bulk,interface_counts=interface,exact_star_volume_conserved=volume==self.old_exact,
            study_per_star_count_nonregression=bool(np.all(counts(labels)>=counts(ctx.labels[self.rows])) and np.all(counts(labels[self.iface_mask])>=counts(ctx.labels[self.rows][self.iface_mask]))))
    def commit(self,proposal):
        ctx=self.ctx;require(ctx.version==self.base_version and array_hash(ctx.points)==self.base_hash,'AtomicCommitConflict')
        require(proposal['passed'],'RefusedCandidateCannotCommit')
        # Reauthenticate all result-bearing data before the only coordinate write.
        fresh=self.final(proposal['z']);require(fresh['passed'] and fresh['coordinates'].tobytes()==proposal['coordinates'].tobytes(),'ChangedProposalBeforeCommit')
        initial=self.search(np.zeros(3));require(float(fresh['batch']['summary']['minimum_weight'])>float(initial['metrics'][:,0].min()),'NoWeightImprovement')
        # Allocate every candidate array/container/receipt before the single state-pointer swap.
        before=ctx.points[self.vertex].copy();newpoints=ctx.points.copy();newpoints[self.vertex]=fresh['coordinates'][self.lv]
        newq=ctx.q.copy();newq[self.rows]=fresh['q'];newlabels=ctx.labels.copy();newlabels[self.rows]=fresh['labels']
        changed=ctx.changed|{self.vertex};affected=ctx.affected_union|set(map(int,self.rows))
        updated=dict(ctx.__dict__,points=newpoints,q=newq,labels=newlabels,changed=changed,affected_union=affected,version=ctx.version+1,expected=array_hash(newpoints))
        record=dict(vertex=self.vertex,before_m=before.tolist(),after_m=newpoints[self.vertex].tolist(),displacement_m=float(np.linalg.norm(newpoints[self.vertex]-before)),radius_m=self.radius,
            incident_tet_ids=sorted(self.selected),incident_tets=len(self.rows),checked_faces=len(self.owners),fixed_opposite_faces=sum(self.lv not in uses[0][1] for uses in self.owners.values()),halo_cells=len(self.cells),
            source_batch_summary=fresh['batch']['summary'],source_scalar_passed=fresh['scalar']['passed'],exact_star_volume_conserved=fresh['exact_star_volume_conserved'],bulk_mixed_classification_counts=fresh['bulk_counts'].tolist(),interface_mixed_classification_counts=fresh['interface_counts'].tolist(),study_per_star_count_nonregression=fresh['study_per_star_count_nonregression'])
        ctx.__dict__=updated
        return record

def run(arrays,state,policy,check):
    check();context=Context(arrays,state,policy,check)
    baseline=batch_quality(state['points'],state['cells'],state['owners'],0,mixed_cells=True,compact=True,face_batch_size=4096)
    vertices,selection_refusals=context.select(baseline);records=[]
    for vertex in vertices:
        check();star=context.star(vertex);initial=star.search(np.zeros(3));require(np.isfinite(initial['metrics']).all(),'FiniteInitialSearchGeometry')
        if initial['metrics'][:,0].min()>=.055:
            records.append(dict(vertex=vertex,status='UNCHANGED_ALREADY_FEASIBLE'));continue
        constraints=[dict(type='ineq',fun=lambda z:star.search(z)['residual'])];bounds=[(-1.,1.)]*3
        a=minimize(lambda z:(star.search(z),float(z@z))[1],np.zeros(3),method='SLSQP',bounds=bounds,constraints=constraints,options=dict(maxiter=METHOD['maxiter'],ftol=METHOD['ftol']))
        check();b=minimize(lambda z:star.search(z)['loss']+1e-4*float(z@z),a.x,method='SLSQP',bounds=bounds,constraints=constraints,options=dict(maxiter=METHOD['maxiter'],ftol=METHOD['ftol']))
        check();proposals=[]
        for value in (a.x,b.x):
            try:proposal=star.final(value)
            except Refusal:continue
            if proposal['passed']:proposals.append(proposal)
        trials=[dict(stage=i+1,success=bool(sol.success),status=int(sol.status),message=str(sol.message),iterations=int(sol.nit)) for i,sol in enumerate((a,b))]
        if not proposals:records.append(dict(vertex=vertex,status='REFUSED_ROLLED_BACK',trials=trials,evaluations=star.evaluations));continue
        chosen=min(proposals,key=lambda p:(star.search(p['z'])['loss']+1e-4*float(p['z']@p['z']),tuple(p['z'])))
        record=star.commit(chosen);record.update(status='COMMITTED',trials=trials,evaluations=star.evaluations);records.append(record)
    changed=np.flatnonzero(np.any(context.points.view('<u8')!=context.original.view('<u8'),axis=1));require(set(map(int,changed))==context.changed,'ExactChangedVertexInventory')
    require(not set(changed)&context.protected,'ProtectedCoordinatesChanged')
    return context.points,dict(task=231,classification='UNQUALIFIED',method=METHOD,baseline_quality=baseline,selected_vertices=vertices,selection_refusals=selection_refusals,records=records,changed_vertices=list(map(int,changed)),changed_count=len(changed),union_affected_tets=len(context.affected_union),coordinates_sha256=array_hash(context.points),original_coordinates_sha256=array_hash(context.original),protected_coordinate_bits_unchanged=True,connectivity_changed=False,
        source_float_bulk=population_report(context.q,policy['quality_targets']['outer_tetrahedra']),source_float_interface=population_report(context.q[context.interface],policy['quality_targets']['outer_tetrahedra']),mixed_bulk_counts=counts(context.labels).tolist(),mixed_interface_counts=counts(context.labels[context.interface]).tolist(),classification_scope='Changed rows source-float AND exact binary-coordinate classifications; untouched rows source-float only. No global exact-Q population claim.',scientific_acceptance=False,source_queries=0)
