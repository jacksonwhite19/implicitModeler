"""Explicit platform-neutral tetrahedral transition controls."""
import math
SCHEMA='V4TetLayerAdapter/1'
SHAPE_SCHEMA='V4TetLayerAdapter/2'
CUSTOM_SCHEMA='V4TetLayerAdapter/3'
PHYSICAL_SCHEMA='V4TetLayerAdapter/4'
ENVELOPE='BarycentricCapVertexEnvelope/1'
FINITE_COLLAR_SCHEMA='V4TetLayerAdapter/5'

def finite_collar(c):
    return type(c) is dict and c.get('schema') in (FINITE_COLLAR_SCHEMA,'V4TetLayerAdapter/6','V4TetLayerAdapter/7')

def validate_finite_collar(c):
    fields={'schema','maximum_distance_m','seconds','fluid_depth','fluid_boxes','vertex_envelope'}
    if set(c)!=fields or c['vertex_envelope']!=ENVELOPE:
        raise ValueError('ExplicitFiniteCollarControlsRequired')
    for k in ('maximum_distance_m','seconds'):
        if type(c[k]) not in (int,float) or not math.isfinite(c[k]) or c[k]<=0:
            raise ValueError('InvalidFiniteCollarControl',k)
    for k in ('fluid_depth','fluid_boxes'):
        if type(c[k]) is not int or c[k]<=0:raise ValueError('InvalidFiniteCollarControl',k)
    return c


def seed_controls(c):
    """The copied rows are a construction seed, not V2 accepted row ancestry."""
    validate(c)
    if finite_collar(c):raise ValueError('FiniteCollarHasNoCopiedRows')
    if c['schema']==SCHEMA:return dict(c)
    return dict(schema=SCHEMA,maximum_distance_m=c['maximum_distance_m'],rows=c.get('seed_rows',c.get('rows')),
        maxiter=c['maxiter'],ftol=c['ftol'],seconds=c['seconds'],fluid_depth=c['fluid_depth'],fluid_boxes=c['fluid_boxes'])

def validate_shape(c):
    fields={'schema','maximum_distance_m','seed_rows','shape_progress','shape_power','size_growth',
        'metric_gradation','carrier_tolerance_m','native_seconds','memory_megabytes',
        'protected_seed_rows','maxiter','ftol','seconds','fluid_depth','fluid_boxes','vertex_envelope'}
    if set(c)!=fields or c['vertex_envelope']!=ENVELOPE:raise ValueError('ExplicitShapeTransitionControlsRequired')
    for k in ('maximum_distance_m','shape_progress','shape_power','size_growth','metric_gradation','carrier_tolerance_m','native_seconds','ftol','seconds'):
        if type(c[k]) not in (float,int):raise ValueError('InvalidShapeTransitionControl',k)
        try:value=float(c[k])
        except (ValueError,OverflowError):raise ValueError('InvalidShapeTransitionControl',k) from None
        if not math.isfinite(value) or value<=0 or value!=c[k]:raise ValueError('InvalidShapeTransitionControl',k)
    for k in ('seed_rows','memory_megabytes','maxiter','fluid_depth','fluid_boxes'):
        if type(c[k]) is not int or c[k]<=0:raise ValueError('InvalidShapeTransitionControl',k)
    if type(c['protected_seed_rows']) is not int or not 0<=c['protected_seed_rows']<c['seed_rows']:raise ValueError('InvalidProtectedSeedRows')
    if c['shape_progress']>1 or c['metric_gradation']<1 or c['native_seconds']>=c['seconds']:raise ValueError('InvalidShapeTransitionBudgetOrGradation')
    return c

def variable_shape(c):
    return c.get('schema') in (SHAPE_SCHEMA,CUSTOM_SCHEMA,PHYSICAL_SCHEMA)

def validate_custom(c):
    fields={'schema','maximum_distance_m','rows','shape_progress','shape_power','size_growth',
        'max_sweeps','max_vertex_trials','max_step_fraction','line_search_steps',
        'maxiter','ftol','seconds','fluid_depth','fluid_boxes','vertex_envelope'}
    if set(c)!=fields or c['vertex_envelope']!=ENVELOPE:raise ValueError('ExplicitCustomTransitionControlsRequired')
    for k in ('maximum_distance_m','shape_power','size_growth','max_step_fraction','ftol','seconds'):
        if type(c[k]) not in (float,int) or not math.isfinite(c[k]) or c[k]<=0:raise ValueError('InvalidCustomTransitionControl',k)
    if type(c['shape_progress']) not in (float,int) or not math.isfinite(c['shape_progress']) or not 0<=c['shape_progress']<=1:
        raise ValueError('InvalidCustomTransitionControl','shape_progress')
    if c['max_step_fraction']>1:raise ValueError('InvalidCustomTransitionControl','max_step_fraction')
    for k in ('rows','max_sweeps','max_vertex_trials','line_search_steps','maxiter','fluid_depth','fluid_boxes'):
        if type(c[k]) is not int or c[k]<=0:raise ValueError('InvalidCustomTransitionControl',k)
    return c

def physical_shape(c):
    return c.get('schema')==PHYSICAL_SCHEMA


def validate_physical(c):
    fields={'schema','maximum_distance_m','rows','objective','max_sweeps',
        'max_vertex_trials','max_step_fraction','line_search_steps','maxiter','ftol',
        'seconds','fluid_depth','fluid_boxes','vertex_envelope'}
    if set(c)!=fields or c['objective']!='physical_mean_ratio':
        raise ValueError('ExplicitPhysicalTransitionControlsRequired')
    # Reuse existing type, positivity, resource and envelope checks. Metric
    # controls are not accepted inputs of this profile.
    legacy={k:v for k,v in c.items() if k!='objective'}
    legacy.update(schema=CUSTOM_SCHEMA,shape_progress=1.,shape_power=1.,size_growth=1.)
    validate_custom(legacy)
    return c


def validate(c):
    from finite_research.flexible_controls import selected,validate as validate_flexible
    if selected(c):return validate_flexible(c)
    if finite_collar(c):return validate_finite_collar(c)
    if type(c) is dict and c.get('schema')==PHYSICAL_SCHEMA:return validate_physical(c)
    if type(c) is dict and c.get('schema')==CUSTOM_SCHEMA:return validate_custom(c)
    if type(c) is dict and c.get('schema')==SHAPE_SCHEMA:return validate_shape(c)
    if type(c) is not dict or set(c)!={'schema','maximum_distance_m','rows','maxiter','ftol','seconds','fluid_depth','fluid_boxes'} or c['schema']!=SCHEMA:raise ValueError('ExplicitTetLayerControlsRequired')
    for k in ('maximum_distance_m','ftol','seconds'):
        if type(c[k]) not in (float,int) or not math.isfinite(c[k]) or c[k]<=0:raise ValueError('InvalidTetLayerControl',k)
    for k in ('rows','maxiter','fluid_depth','fluid_boxes'):
        if type(c[k]) is not int or c[k]<=0:raise ValueError('InvalidTetLayerControl',k)
    return c

