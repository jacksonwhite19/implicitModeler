"""Owned metric placement with actual-neighbour constraints and local screening.

Every numerical proposal is provisional. Passing cells may trade quality margin
while all existing hard limits remain satisfied. The final acceptance owner is
unchanged; a finite placement budget never implies general infeasibility.
"""
from itertools import combinations
import time
import numpy as np
from background_topology_repair import key
from background_local_repair import LocalRepair,star_index
from background_core_repair import atoms
from local_constraint_plan import prepared
from shape_transition_builder import _anchor,envelope_subset_passed,envelope_binding,measure
from transition_motion_guard import MotionGuard


def edge_energy(points, edges, tensors):
    d=points[edges[:,1]]-points[edges[:,0]]
    lengths=np.sqrt(np.maximum(0,np.einsum('ni,nij,nj->n',d,tensors,d)))
    return float(np.mean((lengths-1.)**2))


def inverse_mean_ratio(points,tets):
    p=points[tets];volume6=np.einsum('ni,ni->n',p[:,1]-p[:,0],np.cross(p[:,2]-p[:,0],p[:,3]-p[:,0]))
    squared=sum(np.sum((p[:,i]-p[:,j])**2,axis=1) for i,j in combinations(range(4),2))
    # Shape objective only. Actual positive-volume constraints remain separate.
    with np.errstate(divide='ignore',invalid='ignore'):
        return squared/(12*np.power(np.abs(volume6)*.5,2/3))


def compact_model(points,cells,owners,selected,outer,vertices):
    """Monotone relabel of the complete star and its actual neighbour halo."""
    ids=np.asarray(sorted({v for c in cells for v in c['vertices']}),dtype='<i8');lookup={int(v):i for i,v in enumerate(ids)}
    localcells=[dict(c,vertices=[lookup[v] for v in c['vertices']],faces=[tuple(lookup[v] for v in f) for f in c['faces']]) for c in cells]
    localowners={key(tuple(lookup[v] for v in f)): [(i,tuple(lookup[v] for v in face)) for i,face in uses] for f,uses in owners.items()}
    localouter=[tuple(lookup[v] for v in f) for f in outer];localstar=[c for c in localcells if c['id'] in selected]
    if np.isscalar(vertices):vertices=[int(vertices)]
    model=LocalRepair(points[ids].copy(),localcells,localowners,selected,set(range(len(ids)))-{lookup[v] for v in vertices},[])
    model.scope(localstar)
    return model,ids,lookup,localowners,localouter,localstar


def row_radius(s,vertex):
    """The copied-row source identity owns each cumulative distance bound."""
    if not s.get('_construction_quality_only',False):return s['controls']['maximum_distance_m']
    n=len(s['cap_ids']);index=vertex-s['old_point_count']
    if index<0 or index>=n*s['controls']['rows']:raise ValueError('InvalidCopiedRowVertex')
    if '_construction_row_radii' in s:return float(s['_construction_row_radii'][index//n])
    return (index//n+1)/s['controls']['rows']*s['controls']['maximum_distance_m']


def copied_envelope_passed(s,vertices,points):
    for vertex in vertices:
        bounded=dict(s,controls=dict(s['controls'],maximum_distance_m=row_radius(s,vertex)))
        if not envelope_subset_passed(bounded,[vertex],points):return False
    return True


def _bad_vertices(s,q):
    return {v for r in q['failures'] for i in (r['owners'] if r['kind']=='face_quality' else [r['id']])
         for v in (s['_custom_byid'][i]['vertices'] if i in s['_custom_byid'] else ()) if v>=s['old_point_count']}


def _purpose(s,q,vertex,outer):
    # Do not spend the shape budget on healed interior members of an old defect
    # queue. Actual outer incidence, not a row number or displacement, owns the
    # handoff. Re-evaluate against the current failure inventory after each edit.
    if q['failures']:return 'quality' if vertex in _bad_vertices(s,q) else None
    return 'outer_shape' if vertex in outer else None


def _rank(s, q, stars, metric_index):
    bad=_bad_vertices(s,q);outer={int(v) for v in s['outer'].ravel()}
    tets=np.asarray([c['vertices'] for c in s['additions']],dtype='<i8')
    worst=np.zeros(len(s['points']))
    M=s['custom_metric'];p=s['points']
    if s.get('_physical_shape_completion',False):
        score=inverse_mean_ratio(p,tets)
        for i in range(4):np.maximum.at(worst,tets[:,i],score)
    else:
        for i,j in combinations(range(4),2):
            a,b=tets[:,i],tets[:,j];d=p[b]-p[a];tensor=(M[metric_index[a]]+M[metric_index[b]])*.5
            score=(np.sqrt(np.maximum(0,np.einsum('ni,nij,nj->n',d,tensor,d)))-1.)**2
            np.maximum.at(worst,a,score);np.maximum.at(worst,b,score)
    return sorted((v for v in stars if v in bad or v in outer),key=lambda v:(v not in bad,-float(worst[v]),v))


def _vertex(s, vertex, stars, guard, metric_index, tick, progress, session, q, *, moving=None):
    from scipy.optimize import minimize
    from tet_layer_adapter import quality,local_outer_faces
    import local_termination_topology as t
    moving=[vertex] if moving is None else list(moving)
    byid=s['_custom_byid'];union={c['id']:c for v in moving for c in stars[v]}
    star=[union[i] for i in sorted(union)];selected=set(union)
    faces={key(f) for c in star for f in c['faces']};owners={f:s['owners'][f] for f in faces}
    needed={i for uses in owners.values() for i,_ in uses};cells=[byid[i] for i in sorted(needed)]
    outer=local_outer_faces(star,s['_custom_outer_keys'])
    x=s['points'];base=x[moving].copy();controls=s['controls']
    with progress.span('custom.local_quality'):before=quality(x,cells,owners,selected,outer)
    session.before(s,q,selected,faces,before)
    # Compact only the complete affected stars and all actual face neighbours.
    model,ids,lookup,localowners,localouter,localstar=compact_model(x,cells,owners,selected,outer,moving);v=[lookup[u] for u in moving]
    volumes=np.asarray([abs(t.geometry(x,c)['volume']) for c in star])
    if np.any(volumes<=0):raise ValueError('CustomPositiveSeedRequired')
    edges=np.asarray(sorted({key(e) for c in star for e in combinations(c['vertices'],2)}),dtype='<i8')
    tensors=(s['custom_metric'][metric_index[edges[:,0]]]+s['custom_metric'][metric_index[edges[:,1]]])*.5
    localedges=np.asarray([[lookup[a],lookup[b]] for a,b in edges],dtype='<i8')
    localtets=np.asarray([c['vertices'] for c in localstar],dtype='<i8')
    physical=s.get('_physical_shape_completion',False)
    def energy():
        return float(inverse_mean_ratio(model.points,localtets).max()) if physical else edge_energy(model.points,localedges,tensors)
    old_energy=energy()
    anchors=[_anchor(s,u)[2:] for u in moving]
    scale=controls['maximum_distance_m']/controls['rows']*controls['max_step_fraction']
    def domain(points):
        out=[]
        for u,point,(anchor,normal) in zip(moving,points,anchors):
            D=row_radius(s,u);delta=point-anchor
            out.extend([1-float(delta@delta)/(D*D),float(normal@delta)/(np.linalg.norm(normal)*D)])
            if s['bounds'] is not None:out.extend((point-s['bounds'][:,0])/D);out.extend((s['bounds'][:,1]-point)/D)
        return out
    cache={}
    with prepared(model,localstar,localowners,needed,volumes,localouter) as evaluate:
        def values(z):
            tick();stamp=tuple(z)
            if stamp not in cache:
                model.points[v]=base+scale*np.asarray(z).reshape(-1,3)
                with progress.span('custom.constraints'):vals=np.r_[evaluate(),domain(model.points[v])]
                cache.clear();cache[stamp]=vals
            return cache[stamp]
        zero=np.zeros(3*len(moving));initial=values(zero);passing=initial>=0
        if before['passed']:
            def objective(z):
                values(z);return energy()
            with progress.span('custom.solve'):
                if physical:
                    def constraints(y):
                        vals=values(y[:-1])
                        return np.r_[vals,y[-1]-inverse_mean_ratio(model.points,localtets)]
                    fit=minimize(lambda y:float(y[-1]),np.r_[zero,old_energy],method='SLSQP',
                        bounds=[(-1.,1.)]*len(zero)+[(0.,None)],constraints=[dict(type='ineq',fun=constraints)],
                        options=dict(maxiter=controls['maxiter'],ftol=controls['ftol']))
                    target=base+scale*fit.x[:-1].reshape(-1,3)
                else:
                    fit=minimize(objective,zero,method='SLSQP',bounds=[(-1.,1.)]*len(zero),
                        constraints=[dict(type='ineq',fun=values)],options=dict(maxiter=controls['maxiter'],ftol=controls['ftol']))
                    target=base+scale*fit.x.reshape(-1,3)
        else:
            with progress.span('custom.solve'):
                fit=minimize(lambda y:-float(y[-1]),np.r_[zero,initial.min()],method='SLSQP',bounds=[(-1.,1.)]*len(zero)+[(None,None)],
                    constraints=[dict(type='ineq',fun=lambda y:np.r_[values(y[:-1])-y[-1],values(y[:-1])[passing]])],
                    options=dict(maxiter=controls['maxiter'],ftol=controls['ftol']))
            target=base+scale*fit.x[:-1].reshape(-1,3)
        report=dict(vertex=int(vertex),adopted=False,solver_success=bool(fit.success),before_energy=old_energy,
            before_failures=len(before['failures']),affected_cells=len(star),neighbour_cells=len(cells),collision_rejections=[])
        report.update(moving_vertices=moving,before_minimum_slack=float(initial.min()),constraint_count=len(initial),
            solver_status=int(getattr(fit,'status',-1)),solver_message=str(getattr(fit,'message','unavailable')),
            energy_kind='WorstAffectedPhysicalInverseMeanRatio' if physical else 'MetricEdgeResidual')
        if not np.isfinite(target).all():report['reason']='NonfiniteProposal';return report,before,selected,faces
        for step in range(controls['line_search_steps']):
            tick();proposed=base+(target-base)*2.**(-step)
            if proposed.tobytes()==base.tobytes():break
            if s['bounds'] is not None and (np.any(proposed<=s['bounds'][:,0]) or np.any(proposed>=s['bounds'][:,1])):continue
            after_slack=values(((proposed-base)/scale).ravel()).min()
            model.points[v]=proposed
            new_energy=energy()
            try:
                x[moving]=proposed
                if not copied_envelope_passed(s,moving,x):continue
                with progress.span('custom.local_quality'):after=quality(x,cells,owners,selected,outer)
            finally:x[moving]=base
            bad=atoms(before);newbad=atoms(after)
            if not newbad<=bad:continue
            improvement=(len(newbad)<len(bad) or (bool(bad) and after_slack>initial.min()) or (not bad and new_energy<old_energy))
            if not improvement:continue
            with progress.span('custom.collision'):collision=guard.screen(moving,proposed)
            if not collision['passed']:
                report['collision_rejections'].append(dict(step=step,**collision));continue
            x[moving]=proposed;guard.update(moving)
            report.update(adopted=True,step=step,after_energy=new_energy,after_failures=len(after['failures']),
                before=(base[0] if len(moving)==1 else base).tolist(),after=(proposed[0] if len(moving)==1 else proposed).tolist(),
                after_minimum_slack=float(after_slack),collision_pairs=collision['required_pairs'])
            return report,after,selected,faces
        report['reason']='NoAdmissibleImprovementWithinTrialDomain'
        return report,before,selected,faces


def place(s,check,handoff,progress):
    from tet_layer_adapter import quality,_placement_result
    from transition_repair_plan import quality_session
    from shape_transition_repair import topology_pass
    started=time.perf_counter();deadline=started+s['controls']['seconds']
    def tick():
        if check:check()
        if time.perf_counter()>=deadline:raise ValueError('CustomTransitionPlacementTimeLimit')
    tick.repair_progress=progress
    tick.repair_diagnostics=getattr(check,'repair_diagnostics',None)
    active={c['id'] for c in s['additions']}
    terminal={i for f in s['cap_ids'][s['cap_faces']] for i,_ in s['owners'][key(f)] if i not in active}
    with progress.span('custom.initial_quality',publish=True):initial=quality(s['points'],s['cells'],s['owners'],active|terminal,s['outer'],handoff)
    before_shape=measure(s);records=[];sweeps=[];trials=0;topology=[];coupled=[];events=[]
    metric_index=np.full(len(s['points']),-1,dtype='<i8');metric_index[s['compact_ids']]=np.arange(len(s['compact_ids']))
    with quality_session(s,initial) as session:
        q=session.require(s)
        # Connectivity changes alter the constraints seen by placement. Settle
        # elementary closed cavities before spending coordinate trials; then
        # rank sites against the actual current owners, not the seed defects.
        if not q['passed'] and not s.get('_construction_quality_only',False):
            with progress.span('custom.initial_topology',publish=True):updated,report=topology_pass(s,tick,allow_expanded=False,_session=session)
            s.update(updated);q=session.require(s);topology.append(dict(report,phase='before_coordinates'))
        for sweep in range(s['controls']['max_sweeps']):
            tick();s['_custom_byid']={c['id']:c for c in s['cells']}
            s['_custom_outer_keys']={key(f) for f in s['outer']}
            outer_vertices={int(v) for v in s['outer'].ravel()}
            with progress.span('custom.index',publish=True):
                stars=star_index(s['additions'],range(s['old_point_count'],len(s['points'])))
                guard=MotionGuard(s['points'],[uses[0][1] for _,uses in sorted(s['owners'].items())],s['old_point_count'],tick)
                order=_rank(s,q,stars,metric_index)
            adopted=0
            for vertex in order:
                tick()
                if q['passed'] and s.get('_construction_quality_only',False) and not s.get('_physical_shape_completion',False):break
                purpose=_purpose(s,q,vertex,outer_vertices)
                if purpose is None:continue
                if trials>=s['controls']['max_vertex_trials']:break
                trials+=1
                report,local,selected,faces=_vertex(s,vertex,stars,guard,metric_index,tick,progress,session,q)
                report.update(purpose=purpose,on_outer_front=vertex in outer_vertices)
                if report['adopted']:
                    q=session.coordinate_commit(s,q,selected,faces,local);adopted+=1
                records.append(report)
                events.append(report)
                progress.set(custom_trials=trials,custom_adopted=sum(r['adopted'] for r in records),remaining_records=len(q['failures']))
                if tick.repair_diagnostics is not None:tick.repair_diagnostics('custom_coordinate_trial',report)
            sweeps.append(dict(sweep=sweep,adopted=adopted,remaining_records=len(q['failures']),trials=trials))
            if not q['passed'] and s.get('_construction_quality_only',False):
                # The same actual-star constraints now move the failed cell's
                # free vertices together. No new topology or relaxed gate.
                groups={tuple(sorted({v for i in r.get('owners',[r.get('id')])
                    for v in s['_custom_byid'][i]['vertices'] if v>=s['old_point_count']})) for r in q['failures']}
                for group in sorted(groups,key=lambda row:(len(row),row)):
                    tick()
                    if q['passed'] or trials>=s['controls']['max_vertex_trials']:break
                    if not group or not set(group)&_bad_vertices(s,q):continue
                    trials+=1
                    report,local,selected,faces=_vertex(s,group[0],stars,guard,metric_index,tick,progress,session,q,moving=group)
                    if report['adopted']:q=session.coordinate_commit(s,q,selected,faces,local)
                    coupled.append(report)
                    events.append(report)
            if not q['passed'] and not s.get('_construction_quality_only',False):
                with progress.span('custom.topology',publish=True):updated,report=topology_pass(s,tick,allow_expanded=False,_session=session)
                s.update(updated);q=session.require(s);topology.append(dict(report,phase='after_coordinates'))
            if trials>=s['controls']['max_vertex_trials'] or (not adopted and not (topology and topology[-1]['edits'])):break
        active={c['id'] for c in s['additions']}
        terminal={i for f in s['cap_ids'][s['cap_faces']] for i,_ in s['owners'][key(f)] if i not in active}
        with progress.span('custom.final_quality',publish=True):final=quality(s['points'],s['cells'],s['owners'],active|terminal,s['outer'],handoff)
        if atoms(final)!=atoms(q):raise ValueError('CustomFinalFailureInventoryMismatch')
        result=_placement_result(s,final,records,sweeps,len(initial['failures']),started)
        from custom_transition_diagnostics import changed_outer
        patch_records=[]
        for event in events:
            vertices=event['moving_vertices']
            if event['adopted'] and len(vertices)>1:
                patch_records.extend(dict(adopted=True,vertex=v,before=b,after=a) for v,b,a in zip(vertices,event['before'],event['after']))
            else:patch_records.append(event)
        result.update(construction=s['construction'],vertex_envelope_binding=envelope_binding(s),shape=measure(s),
            shape_before=before_shape,whole_quality_scans=2,topology=topology,custom_vertex_trials=trials,coupled_groups=coupled,
            shape_schedule='CurrentDefectsThenActualOuterFront/1',outer_patch=changed_outer(s,patch_records),
            external_remesher=False,scientific_mesh_acceptance=False,
            trial_budget_exhausted=trials>=s['controls']['max_vertex_trials'],
            scope='Owned shared-row topology and metric placement. No universal shape convergence or bulk compatibility guarantee.')
        s.pop('_custom_byid',None)
        s.pop('_custom_outer_keys',None)
        return result
