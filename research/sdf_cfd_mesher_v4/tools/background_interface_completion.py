"""Bounded generic construction before the existing final joined acceptance.

No shape dispatch, checkpoint admission, gate relaxation or whole-mesh checker.
Exact relative edits retain the existing backend embedding premise. The caller
must still run every current completed-mesh acceptance obligation afterward.
"""
import json,math,time
import numpy as np

SCHEMA='V4CoreInterfaceCompletion/1'
INTEGER_FIELDS=('maxiter','preservation_max_depth','preservation_steps_per_vertex',
    'maximum_modified_vertices','maximum_single_star_cells','maximum_groups',
    'maximum_group_vertices','maximum_union_cells')
FIELDS={'schema','seconds_per_sweep','ftol',*INTEGER_FIELDS}

class CompletionWorkLimit(ValueError):
    """Reporting-only details; preserve the historical ValueError args/text."""
    def __init__(self,phase,details):
        super().__init__('CoreInterfaceCompletionWorkLimit',phase)
        self._details=json.dumps(dict(schema='V4CoreInterfaceWorkLimit/1',phase=phase,**details),
            sort_keys=True,allow_nan=False)

    def details(self):
        return json.loads(self._details)

def single_work_limit(single,controls,selected_failures):
    sizes=[len(single.stars[v]) for v in single.wanted]
    exceeded=[]
    if len(single.wanted)>controls['maximum_modified_vertices']:exceeded.append('maximum_modified_vertices')
    if any(n>controls['maximum_single_star_cells'] for n in sizes):exceeded.append('maximum_single_star_cells')
    return dict(controls=controls,selected_failures=selected_failures,exceeded=exceeded,
        observed=dict(wanted_vertices=len(single.wanted),maximum_single_star_cells=max(sizes,default=0),
            stars=[dict(vertex=int(v),cells=n) for v,n in zip(single.wanted,sizes)]))

def coupled_work_limit(supports,enriched,stars,edits,controls,sweep,remaining_failures,selected_failures,*,pending_single_vertices=0):
    all_vertices={v for row in enriched for v in row}
    edited={r['vertex'] for r in edits}
    groups=[dict(vertices=list(map(int,row)),vertices_count=len(row),
        union_cells=len({c['id'] for v in row for c in stars[v]})) for row in enriched]
    exceeded=[]
    if len(enriched)>controls['maximum_groups']:exceeded.append('maximum_groups')
    if len(edited|all_vertices)>controls['maximum_modified_vertices']:exceeded.append('maximum_modified_vertices')
    if any(r['vertices_count']>controls['maximum_group_vertices'] for r in groups):exceeded.append('maximum_group_vertices')
    if any(r['union_cells']>controls['maximum_union_cells'] for r in groups):exceeded.append('maximum_union_cells')
    return dict(controls=controls,selected_failures=selected_failures,remaining_failures=remaining_failures,
        single_sweep=sweep,pending_single_vertices=pending_single_vertices,
        raw_support_vertices=[list(map(int,row)) for row in supports],exceeded=exceeded,observed=dict(raw_supports=len(supports),enriched_groups=len(enriched),
            single_modified_vertices=len(edited),coupled_vertices=len(all_vertices),total_modified_or_coupled_vertices=len(edited|all_vertices),
            maximum_group_vertices=max((r['vertices_count'] for r in groups),default=0),
            maximum_union_cells=max((r['union_cells'] for r in groups),default=0),groups=groups))
def validate(controls):
    if type(controls) is not dict or set(controls)!=FIELDS or controls['schema']!=SCHEMA:
        raise ValueError('ExplicitCoreInterfaceCompletionControlsRequired')
    for name in INTEGER_FIELDS:
        if type(controls[name]) is not int or controls[name]<1:raise ValueError('InvalidInterfaceCompletionControl',name)
    for name in ('seconds_per_sweep','ftol'):
        if type(controls[name]) not in (int,float) or not math.isfinite(controls[name]) or controls[name]<=0:
            raise ValueError('InvalidInterfaceCompletionControl',name)
    return controls

def interface_failures(points,byid,owners,core_ids,faces,prism_geometry):
    """Only existing interface face metrics select proposals; never acceptance."""
    from background_mixed_geometry import batch_geometry
    import local_termination_topology as t
    cq=t.q.legacy_modules()['component_quality'];result=[]
    for start in range(0,len(faces),4096):
        selected=faces[start:start+4096];paired=[]
        for face in selected:
            rows=owners[tuple(sorted(face))]
            if len(rows)!=2:raise ValueError('UnjoinedCompletionInterface')
            new=[i for i,_ in rows if i in core_ids];old=[i for i,_ in rows if i not in core_ids]
            if len(new)!=1 or len(old)!=1 or byid[old[0]]['kind']!='prism' or byid[new[0]]['kind']!='tetrahedron':
                raise ValueError('UnsupportedCompletionInterfaceOwners')
            paired.append((old[0],new[0]))
        geometry=batch_geometry(points,[byid[i] for i in sorted({b for _,b in paired})])
        old=[prism_geometry[a] for a,b in paired];new=[geometry[b] for a,b in paired]
        vs=points[np.asarray(selected,dtype=np.int64)];S,C=cq.FG.face_area_centre_tri(vs)
        w,sk,ratio,_=cq.face_metrics(vs,S,C,np.array([g['centre'] for g in old]),np.array([g['centre'] for g in new]),
            np.array([g['volume'] for g in old]),np.array([g['volume'] for g in new]))
        if not np.isfinite(np.r_[w,sk,ratio]).all():raise ValueError('NonfiniteCompletionInterfaceMetric')
        for j,(a,b) in enumerate(paired):
            if w[j]<.055 or sk[j]>3.63636 or ratio[j]<.01:
                result.append(dict(kind='face_quality',vertices=tuple(selected[j]),owners=[a,b],weight=float(w[j]),
                    skewness=float(sk[j]),symmetric_volume_ratio=float(ratio[j])))
    return result

def construct(handoff,points,cells,owners,core_ids,fixed,faces,controls):
    """Use issued live custody, private working arrays and the qualified owners."""
    if type(controls) is not dict:raise ValueError('ExplicitCoreInterfaceCompletionControlsRequired')
    controls=validate(controls.copy())
    from rolling_fast_state import BackgroundInputs
    if type(handoff) is not BackgroundInputs:raise ValueError('ProtectedLiveCompletionHandoffRequired')
    handoff.require(handoff.view)
    started=time.perf_counter();byid={c['id']:c for c in cells}
    terminal={i for f in faces for i,_ in owners[tuple(sorted(f))] if i not in core_ids}
    geometry,_,_=handoff.quality_inputs(points,byid,owners,terminal,terminal)
    missing=terminal-geometry.keys()
    if missing:
        from background_interface_geometry import prepass
        from types import SimpleNamespace
        extra=prepass(points,SimpleNamespace(cells=byid),[str(i) for i in sorted(missing)])
        if extra is None:
            import local_termination_topology as t
            extra={i:t.geometry(points,byid[i]) for i in missing}
        geometry.update(extra)
    failures=interface_failures(points,byid,owners,core_ids,faces,geometry)
    selected_seconds=time.perf_counter()-started
    if not failures:
        return points,dict(schema='V4CoreInterfaceConstruction/1',interface_faces=len(faces),selected_failures=0,
            modified_vertices=0,selection_seconds=selected_seconds,seconds=time.perf_counter()-started,
            final_acceptance_required=True)
    from refined_local_repair import RefinedRepair
    from coordinated_local_repair import CoordinatedRepair
    from background_local_repair import star_index,verify_edits
    single=RefinedRepair(points,cells,owners,core_ids,fixed,failures)
    if len(single.wanted)>controls['maximum_modified_vertices'] or any(len(single.stars[v])>controls['maximum_single_star_cells'] for v in single.wanted):
        raise CompletionWorkLimit('single_stars',single_work_limit(single,controls,len(failures)))
    tick=time.perf_counter()
    sweep=single.sweep(seconds=controls['seconds_per_sweep'],maxiter=controls['maxiter'],ftol=controls['ftol'])
    single_seconds=time.perf_counter()-tick;edits=list(single.edits);candidate=single.points;groups=[]
    if single.reports:
        group=CoordinatedRepair(candidate,cells,owners,core_ids,fixed,list(single.reports.values()))
        supports=group.groups();roots=star_index(cells,{v for row in supports for v in row})
        enriched=sorted({tuple(sorted(set(row)|{v for u in row for c in roots[u] for v in c['vertices'] if v not in fixed})) for row in supports})
        all_vertices={v for row in enriched for v in row};stars=star_index(cells,all_vertices)
        if (len(enriched)>controls['maximum_groups'] or len({r['vertex'] for r in edits}|all_vertices)>controls['maximum_modified_vertices']
            or any(len(row)>controls['maximum_group_vertices'] or len({c['id'] for v in row for c in stars[v]})>controls['maximum_union_cells'] for row in enriched)):
            raise CompletionWorkLimit('coupled_stars',coupled_work_limit(supports,enriched,stars,edits,controls,
                sweep,len(single.reports),len(failures),pending_single_vertices=len(getattr(single,'pending',None) or [])))
        for row in enriched:
            groups.append(group.group_trial(list(row),stars,maxiter=controls['maxiter'],ftol=controls['ftol'],
                max_depth=controls['preservation_max_depth'],max_attempts=controls['preservation_steps_per_vertex']*len(row)))
        edits.extend(group.edits);candidate=group.points
    tick=time.perf_counter();preservation=verify_edits(points,candidate,cells,fixed,edits);replay_seconds=time.perf_counter()-tick
    return candidate,dict(schema='V4CoreInterfaceConstruction/1',interface_faces=len(faces),selected_failures=len(failures),
        modified_vertices=len({r['vertex'] for r in edits}),selection_seconds=selected_seconds,single_seconds=single_seconds,
        sweep=sweep,groups=groups,witnesses=edits,relative_preservation=preservation,replay_seconds=replay_seconds,
        seconds=time.perf_counter()-started,final_acceptance_required=True,
        scope='Private current construction only. Original backend embedding premise; no archived prism/restart or replacement absolute acceptance.')
