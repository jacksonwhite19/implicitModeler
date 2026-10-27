"""Explicit source-vertex schedules for variable-depth mixed fronts.

Targets are copied from the binary64 authority. Projection is supplied by the
existing SDF provider; this module introduces no projection or recovery rule.
"""
from copy import deepcopy
import math
from ..common.authority import identity


def source_targets(schedule,source):
    if source in schedule['actual']:return schedule['actual'][source]
    if schedule.get('default')=='nominal':return schedule['nominal']
    raise ValueError('IncompleteSourceSchedule')


def bind_schedule(state, nominal, actual):
    nominal=list(map(float,nominal))
    if not nominal or any(not math.isfinite(v) or v<=0 for v in nominal):
        raise ValueError('InvalidNominalSchedule')
    if any(a>=b for a,b in zip(nominal,nominal[1:])):
        raise ValueError('NonIncreasingNominalSchedule')
    if set(actual)!=set(state['wall']['vertices']):
        raise ValueError('IncompleteSourceSchedule')
    rows={}
    for source,values in actual.items():
        values=list(map(float,values))
        if len(values)!=len(nominal) or any(not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('InvalidActualSchedule')
        if values[0]!=nominal[0]:raise ValueError('FirstLayerScheduleMutation')
        if any(a>=b for a,b in zip(values,values[1:])):
            raise ValueError('NonIncreasingActualSchedule')
        rows[source]=values
    state['local_schedule']=dict(nominal=nominal,actual=rows)
    return identity(state['local_schedule'])


def column_accounting(state,source):
    column=state['columns'][str(source)];k=column['last_prism_layer']
    schedule=state['local_schedule'];nominal=schedule['nominal']
    rows={}
    for vertex in state['wall']['triangles'][str(source)]:
        actual=source_targets(schedule,vertex);current=actual[k-1] if k else 0.
        rows[vertex]=dict(actual_cumulative=current,nominal_cumulative=nominal[k-1] if k else 0.,
            thickness_debt=(nominal[k-1]-current) if k else 0.,
            actual_next_increment=(actual[k]-current) if k<len(actual) else None,
            nominal_next_increment=(nominal[k]-(nominal[k-1] if k else 0.)) if k<len(nominal) else None)
    return dict(layer=k,status=column['status'],vertices=rows)


def project_active_futures(state,project):
    """Build actual future points only for active prism columns.

    project(source_vertex, prior_point, target) must use qualified branch-aware
    SDF projection. Shared current vertices use one deterministic trajectory.
    Transition faces remain explicit screening dependencies, never prisms.
    """
    schedule=state['local_schedule'];trajectories={};columns={}
    for source,face in sorted(state['prism_front'].items()):
        column=state['columns'][source]
        if column['status']!='ActivePrismColumn':raise ValueError('InactivePrismFront')
        k=column['last_prism_layer'];rows=[]
        for vertex in state['wall']['triangles'][source]:
            key=f'layer:{k}:source:{vertex}'
            if key not in state['faces'][face]['vertices']:raise ValueError('FrontScheduleAncestryMismatch')
            targets=source_targets(schedule,vertex)[k:]
            trajectory=identity(dict(vertex=key,point=state['vertices'][key],targets=targets))
            if trajectory not in trajectories:
                point=deepcopy(state['vertices'][key]);points=[]
                for target in targets:
                    point=list(map(float,project(vertex,point,target)))
                    if len(point)!=3 or not all(math.isfinite(v) for v in point):raise ValueError('InvalidForecastProjection')
                    points.append(point)
                trajectories[trajectory]=dict(vertex=key,targets=targets,coordinates=points)
            rows.append(trajectory)
        columns[source]=dict(current_layer=k,trajectories=rows,accounting=column_accounting(state,source))
    return dict(columns=columns,trajectories=trajectories,
                screening_faces=deepcopy(state['outer']),schedule=identity(schedule))
