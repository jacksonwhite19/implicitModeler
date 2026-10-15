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


def compact_model(points,cells,owners,selected,outer,vertex):
    """Monotone relabel of the complete star and its actual neighbour halo."""
    ids=np.asarray(sorted({v for c in cells for v in c['vertices']}),dtype='<i8');lookup={int(v):i for i,v in enumerate(ids)}
    localcells=[dict(c,vertices=[lookup[v] for v in c['vertices']],faces=[tuple(lookup[v] for v in f) for f in c['faces']]) for c in cells]
    localowners={key(tuple(lookup[v] for v in f)): [(i,tuple(lookup[v] for v in face)) for i,face in uses] for f,uses in owners.items()}
    localouter=[tuple(lookup[v] for v in f) for f in outer];localstar=[c for c in localcells if c['id'] in selected]
    model=LocalRepair(points[ids].copy(),localcells,localowners,selected,set(range(len(ids)))-{lookup[vertex]},[])
    model.scope(localstar)
    return model,ids,lookup,localowners,localouter,localstar


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
    for i,j in combinations(range(4),2):
        a,b=tets[:,i],tets[:,j];d=p[b]-p[a];tensor=(M[metric_index[a]]+M[metric_index[b]])*.5
        score=(np.sqrt(np.maximum(0,np.einsum('ni,nij,nj->n',d,tensor,d)))-1.)**2
        np.maximum.at(worst,a,score);np.maximum.at(worst,b,score)
    return sorted((v for v in stars if v in bad or v in outer),key=lambda v:(v not in bad,-float(worst[v]),v))


def _vertex(s, vertex, stars, guard, metric_index, tick, progress, session, q):
    from scipy.optimize import minimize
    from tet_layer_adapter import quality,local_outer_faces
    import local_termination_topology as t
    byid=s['_custom_byid'];star=stars[vertex];selected={c['id'] for c in star}
    faces={key(f) for c in star for f in c['faces']};owners={f:s['owners'][f] for f in faces}
    needed={i for uses in owners.values() for i,_ in uses};cells=[byid[i] for i in sorted(needed)]
    outer=local_outer_faces(star,s['_custom_outer_keys'])
    x=s['points'];base=x[vertex].copy();controls=s['controls']
    with progress.span('custom.local_quality'):before=quality(x,cells,owners,selected,outer)
    session.before(s,q,selected,faces,before)
    # Compact only the complete affected stars and all actual face neighbours.
    model,ids,lookup,localowners,localouter,localstar=compact_model(x,cells,owners,selected,outer,vertex);v=lookup[vertex]
    volumes=np.asarray([abs(t.geometry(x,c)['volume']) for c in star])
    if np.any(volumes<=0):raise ValueError('CustomPositiveSeedRequired')
    edges=np.asarray(sorted({key(e) for c in star for e in combinations(c['vertices'],2)}),dtype='<i8')
    tensors=(s['custom_metric'][metric_index[edges[:,0]]]+s['custom_metric'][metric_index[edges[:,1]]])*.5
    localedges=np.asarray([[lookup[a],lookup[b]] for a,b in edges],dtype='<i8')
    old_energy=edge_energy(model.points,localedges,tensors)
    _,_,anchor,normal=_anchor(s,vertex);D=controls['maximum_distance_m'];scale=D/controls['rows']*controls['max_step_fraction']
    def domain(point):
        delta=point-anchor
        out=[1-float(delta@delta)/(D*D),float(normal@delta)/(np.linalg.norm(normal)*D)]
        if s['bounds'] is not None:out.extend((point-s['bounds'][:,0])/D);out.extend((s['bounds'][:,1]-point)/D)
        return out
    cache={}
    with prepared(model,localstar,localowners,needed,volumes,localouter) as evaluate:
        def values(z):
            tick();stamp=tuple(z)
            if stamp not in cache:
                model.points[v]=base+scale*np.asarray(z)
                with progress.span('custom.constraints'):vals=np.r_[evaluate(),domain(model.points[v])]
                cache.clear();cache[stamp]=vals
            return cache[stamp]
        zero=np.zeros(3);initial=values(zero);passing=initial>=0
        if before['passed']:
            def objective(z):
                values(z);return edge_energy(model.points,localedges,tensors)
            with progress.span('custom.solve'):
                fit=minimize(objective,zero,method='SLSQP',bounds=[(-1.,1.)]*3,
                    constraints=[dict(type='ineq',fun=values)],options=dict(maxiter=controls['maxiter'],ftol=controls['ftol']))
            target=base+scale*fit.x
        else:
            with progress.span('custom.solve'):
                fit=minimize(lambda y:-float(y[-1]),np.r_[zero,initial.min()],method='SLSQP',bounds=[(-1.,1.)]*3+[(None,None)],
                    constraints=[dict(type='ineq',fun=lambda y:np.r_[values(y[:-1])-y[-1],values(y[:-1])[passing]])],
                    options=dict(maxiter=controls['maxiter'],ftol=controls['ftol']))
            target=base+scale*fit.x[:-1]
        report=dict(vertex=int(vertex),adopted=False,solver_success=bool(fit.success),before_energy=old_energy,
            before_failures=len(before['failures']),affected_cells=len(star),neighbour_cells=len(cells),collision_rejections=[])
        if not np.isfinite(target).all():report['reason']='NonfiniteProposal';return report,before,selected,faces
        for step in range(controls['line_search_steps']):
            tick();proposed=base+(target-base)*2.**(-step)
            if proposed.tobytes()==base.tobytes():break
            if s['bounds'] is not None and (np.any(proposed<=s['bounds'][:,0]) or np.any(proposed>=s['bounds'][:,1])):continue
            values((proposed-base)/scale)
            after_slack=values((proposed-base)/scale).min()
            model.points[v]=proposed
            new_energy=edge_energy(model.points,localedges,tensors)
            try:
                x[vertex]=proposed
                if not envelope_subset_passed(s,[vertex],x):continue
                with progress.span('custom.local_quality'):after=quality(x,cells,owners,selected,outer)
            finally:x[vertex]=base
            bad=atoms(before);newbad=atoms(after)
            if not newbad<=bad:continue
            improvement=(len(newbad)<len(bad) or (bool(bad) and after_slack>initial.min()) or (not bad and new_energy<old_energy))
            if not improvement:continue
            with progress.span('custom.collision'):collision=guard.screen([vertex],proposed.reshape(1,3))
            if not collision['passed']:
                report['collision_rejections'].append(dict(step=step,**collision));continue
            x[vertex]=proposed;guard.update([vertex])
            report.update(adopted=True,step=step,after_energy=new_energy,after_failures=len(after['failures']),
                before=base.tolist(),after=proposed.tolist(),collision_pairs=collision['required_pairs'])
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
    before_shape=measure(s);records=[];sweeps=[];trials=0;topology=[]
    metric_index=np.full(len(s['points']),-1,dtype='<i8');metric_index[s['compact_ids']]=np.arange(len(s['compact_ids']))
    with quality_session(s,initial) as session:
        q=session.require(s)
        # Connectivity changes alter the constraints seen by placement. Settle
        # elementary closed cavities before spending coordinate trials; then
        # rank sites against the actual current owners, not the seed defects.
        if not q['passed']:
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
                purpose=_purpose(s,q,vertex,outer_vertices)
                if purpose is None:continue
                if trials>=s['controls']['max_vertex_trials']:break
                trials+=1
                report,local,selected,faces=_vertex(s,vertex,stars,guard,metric_index,tick,progress,session,q)
                report.update(purpose=purpose,on_outer_front=vertex in outer_vertices)
                if report['adopted']:
                    q=session.coordinate_commit(s,q,selected,faces,local);adopted+=1
                records.append(report)
                progress.set(custom_trials=trials,custom_adopted=sum(r['adopted'] for r in records),remaining_records=len(q['failures']))
                if tick.repair_diagnostics is not None:tick.repair_diagnostics('custom_coordinate_trial',report)
            sweeps.append(dict(sweep=sweep,adopted=adopted,remaining_records=len(q['failures']),trials=trials))
            if not q['passed']:
                with progress.span('custom.topology',publish=True):updated,report=topology_pass(s,tick,allow_expanded=False,_session=session)
                s.update(updated);q=session.require(s);topology.append(dict(report,phase='after_coordinates'))
            if trials>=s['controls']['max_vertex_trials'] or (not adopted and not (topology and topology[-1]['edits'])):break
        active={c['id'] for c in s['additions']}
        terminal={i for f in s['cap_ids'][s['cap_faces']] for i,_ in s['owners'][key(f)] if i not in active}
        with progress.span('custom.final_quality',publish=True):final=quality(s['points'],s['cells'],s['owners'],active|terminal,s['outer'],handoff)
        if atoms(final)!=atoms(q):raise ValueError('CustomFinalFailureInventoryMismatch')
        result=_placement_result(s,final,records,sweeps,len(initial['failures']),started)
        from custom_transition_diagnostics import changed_outer
        result.update(construction=s['construction'],vertex_envelope_binding=envelope_binding(s),shape=measure(s),
            shape_before=before_shape,whole_quality_scans=2,topology=topology,custom_vertex_trials=trials,
            shape_schedule='CurrentDefectsThenActualOuterFront/1',outer_patch=changed_outer(s,records),
            external_remesher=False,scientific_mesh_acceptance=False,
            trial_budget_exhausted=trials>=s['controls']['max_vertex_trials'],
            scope='Owned shared-row topology and metric placement. No universal shape convergence or bulk compatibility guarantee.')
        s.pop('_custom_byid',None)
        s.pop('_custom_outer_keys',None)
        return result
