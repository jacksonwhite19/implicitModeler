"""Q0 native request/receipt identity; never a proof about moved Q1."""
from finite_research.flexible_controls import NATIVE_REQUEST,NATIVE_RESULT,identities,validate
from sdf_cfd_mesher_v4.common.authority import identity


def version(request):
    if type(request) is not dict:raise ValueError('ExactFiniteNativeRequestVersionRequired')
    schema=request.get('schema')
    if schema==NATIVE_REQUEST:
        validate_request(request)
        return 2
    if schema=='PrivateFiniteGmshCollar/1':
        if any(name in request for name in ('controls','extent_policy_identity','quality_targets_identity',
                'initializer_distance_m','stage_binding','stage_binding_identity','source_release_sha256')):
            raise ValueError('FlexibleFieldsCannotUseLegacyNativeVersion')
        return 1
    raise ValueError('UnknownFiniteNativeRequestVersion')


def validate_request(request):
    base={'schema','source_pins','input_sha256','growth_per_m','maximum_size_m','seed_policy',
        'controls_identity','input_points_hash','prism_centres_hash','prism_volumes_hash',
        'seed_hash','boundary_admission_sha256','control','controls','extent_policy_identity',
        'quality_targets_identity','initializer_distance_m','stage_binding','stage_binding_identity','source_release_sha256'}
    if 'stage_observations' in request:base.add('stage_observations')
    if type(request) is not dict or set(request)!=base or request.get('schema')!=NATIVE_REQUEST:
        raise ValueError('ExactFlexibleNativeRequestRequired')
    controls=validate(request['controls'])
    if (any(request.get(name)!=digest for name,digest in identities(controls).items())
            or request['initializer_distance_m']!=controls['envelope']['nominal_distance_m']):
        raise ValueError('ChangedFlexibleNativeControlBinding')
    binding=request['stage_binding']
    if (type(binding) is not dict or identity(binding)!=request['stage_binding_identity']
            or any(binding.get(name)!=digest for name,digest in identities(controls).items())
            or binding.get('execution_target') not in ('transition','transition_background')
            or binding.get('transition_fluid_verification') not in ('backend_trusted','continuous_source')):
        raise ValueError('ChangedFlexibleNativeStageBinding')
    for name in ('parent_prism_mesh','request_sha256','geometry_source_identity','downstream_policy_identity','prism_schedule_identity'):
        value=binding.get(name)
        if type(value) is not str or len(value)!=64 or any(char not in '0123456789abcdef' for char in value):
            raise ValueError('IncompleteFlexibleNativeParentContext',name)
    for name in ('input_sha256','input_points_hash','prism_centres_hash','prism_volumes_hash',
            'seed_hash','boundary_admission_sha256','stage_binding_identity','source_release_sha256',*identities(controls)):
        value=request[name]
        if type(value) is not str or len(value)!=64 or any(char not in '0123456789abcdef' for char in value):
            raise ValueError('InvalidFlexibleNativeIdentity',name)
    return request


def validate_receipt(receipt,request,request_sha256):
    validate_request(request)
    if (receipt.get('schema')!=NATIVE_RESULT or receipt.get('passed') is not True
            or receipt.get('request_sha256')!=request_sha256
            or receipt.get('input_sha256')!=request['input_sha256']
            or receipt.get('stage_binding_identity')!=request['stage_binding_identity']
            or receipt.get('source_release_sha256')!=request['source_release_sha256']
            or any(receipt.get(name)!=digest for name,digest in identities(request['controls']).items())
            or receipt.get('geometry_scope')!='Q0_INITIALIZER_PROVENANCE_ONLY'):
        raise ValueError('ChangedFlexibleNativeReceipt')
    return receipt
