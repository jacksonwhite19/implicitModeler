"""Explicit flexible-front contracts; no acceptance or execution authority."""
import math
from sdf_cfd_mesher_v4.common.authority import identity

SCHEMA='V4TetLayerAdapter/6'
GRADED_SCHEMA='V4TetLayerAdapter/7'
GRADED_CONSTRUCTION='GradedSupportAdvance/1'
EXTENT='TransitionExtentPolicy/1'
CONSTRUCTION='SharedFrontDepthPoll/1'
PLACEMENT='CapRelativeVertexPlacement/1'
REQUEST='FiniteCollarDownstreamRequest/5'
NATIVE_REQUEST='PrivateFiniteGmshCollar/2'
NATIVE_RESULT='PrivateFiniteCollarBackend/2'
REPORT='V4TetLayerAcceptance/4'
TARGETS={
    'outer_tetrahedra':{'minimum':.25,'population_targets':[
        {'fraction':.99,'minimum':.4},{'fraction':.9,'minimum':.6}]},
    'outer_triangles':{'minimum':.3,'population_targets':[
        {'fraction':.99,'minimum':.6},{'fraction':.9,'minimum':.8}]}}


def selected(controls):
    return type(controls) is dict and controls.get('schema') in (SCHEMA,GRADED_SCHEMA)


def graded(controls):
    return type(controls) is dict and controls.get('schema')==GRADED_SCHEMA


def positive(value,name):
    if type(value) not in (int,float):raise ValueError('InvalidFlexibleControl',name)
    try:valid=math.isfinite(value) and value>0
    except OverflowError:valid=False
    if not valid:raise ValueError('InvalidFlexibleControl',name)


def extent(value):
    if type(value) is not dict or value.get('schema')!=EXTENT:
        raise ValueError('ExplicitTransitionExtentRequired')
    mode=value.get('mode')
    fields={'schema','mode','nominal_distance_m'}
    if mode=='strict':fields.add('maximum_distance_m')
    if mode not in ('nominal','strict') or set(value)!=fields:
        raise ValueError('ExactTransitionExtentFieldsRequired')
    positive(value['nominal_distance_m'],'nominal_distance_m')
    if mode=='strict':
        positive(value['maximum_distance_m'],'maximum_distance_m')
        if value['maximum_distance_m']<value['nominal_distance_m']:
            raise ValueError('StrictBoundBelowNominal')
    return value


def validate(controls):
    fields={'schema','envelope','construction','quality_targets','seconds',
        'fluid_depth','fluid_boxes','vertex_domain'}
    if not selected(controls) or set(controls)!=fields:
        raise ValueError('ExactFlexibleTransitionControlsRequired')
    extent(controls['envelope'])
    construction=GRADED_CONSTRUCTION if graded(controls) else CONSTRUCTION
    if controls['construction']!={'schema':construction} or controls['vertex_domain']!=PLACEMENT:
        raise ValueError('ExplicitSharedFrontConstructionRequired')
    if graded(controls) and controls['envelope']['mode']!='nominal':
        raise ValueError('GradedNominalCompletionRequired')
    from finite_support.transition_front_targets import validate as target_validate
    target_validate(controls['quality_targets'])
    if identity(controls['quality_targets'])!=identity(TARGETS):
        raise ValueError('UnchangedSixShapeTargetsRequired')
    positive(controls['seconds'],'seconds')
    for name in ('fluid_depth','fluid_boxes'):
        if type(controls[name]) is not int or controls[name]<=0:
            raise ValueError('InvalidFlexibleControl',name)
    return controls


def identities(controls):
    validate(controls)
    result=dict(extent_policy_identity=identity(controls['envelope']),
        quality_targets_identity=identity(controls['quality_targets']),
        controls_identity=identity(controls))
    if graded(controls):result['constructor_schema']=GRADED_CONSTRUCTION
    return result


def validate_request(request,controls):
    expected=identities(controls)
    if request.get('schema')!=REQUEST or any(request.get(k)!=v for k,v in expected.items()):
        raise ValueError('ChangedFlexibleStagePolicyBinding')
    return expected
