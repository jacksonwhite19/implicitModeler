"""Authenticated finite research target data; no mesh or live authority issuer."""
TARGETS=('transition','transition_background','direct_complete')

def choice(value):
    target=value.get('execution_target','transition')
    if type(target) is not str or target not in TARGETS:
        raise ValueError('ExplicitFiniteExecutionTargetRequired')
    return target

def prepared(assignment,argument=None):
    target=choice(assignment)
    if argument is not None and (type(argument) is not str or argument!=target):
        raise ValueError('ExecutionTargetArgumentAssignmentMismatch')
    if 'execution_target' in assignment and assignment.get('prism_producer_mode')!='source_directions':
        raise ValueError('OuterBackgroundRequiresIndependentPrismRoute')
    return target

def validate_binding(assignment,planned,stage):
    schema=stage.get('schema')
    if schema not in ('FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
        if any('execution_target' in item for item in (assignment,planned,stage)):
            raise ValueError('LegacyExecutionTargetFieldNotSupported')
        return 'transition'
    target=prepared(assignment)
    if (any(item.get('execution_target')!=target
            for item in (assignment,planned,stage))):
        raise ValueError('ChangedFiniteExecutionTargetBinding')
    return target

def validate_result(result,binding):
    """Require the actual joined consumer report, not backend success alone."""
    if choice(binding)!='transition_background':raise ValueError('ExplicitJoinedTargetRequired')
    assurance=result.get('assurance',{})
    if (result.get('passed') is not True or result.get('execution_target')!='transition_background'
            or result.get('background_invoked') is not True
            or result.get('joined_contract')!='FiniteTransitionJoinedExact/1'
            or assurance.get('completed_mesh_exact_proofs')!='passed'
            or assurance.get('omitted_checks')!=[]
            or result.get('certificate',{}).get('passed') is not True
            or result.get('certificate',{}).get('independent_coverage_passed') is not True
            or result.get('volume',{}).get('exact_volume_balance') is not True
            or result['volume'].get('positive_cells') is not True
            or result['volume'].get('interior_degree',{}).get('passed') is not True
            or result.get('quality',{}).get('passed') is not True
            or result.get('tet_transition',{}).get('passed') is not True
            or type(result['tet_transition'].get('arrays_sha256')) is not str
            or len(result['tet_transition']['arrays_sha256'])!=64
            or type(result['tet_transition'].get('sha256')) is not str or len(result['tet_transition']['sha256'])!=64
            or result.get('inherited_transition_preservation',{}).get('passed') is not True
            or type(result.get('mesh_sha256')) is not str or len(result['mesh_sha256'])!=64):
        raise ValueError('IncompleteJoinedBackgroundConsumer')
    if binding.get('constructor_schema')=='GradedSupportAdvance/1':
        from finite_research.graded_joined import REQUIRED
        proof=result.get('whole_joined_geometry',{})
        if (proof.get('passed') is not True or proof.get('schema')!='V4JoinedGradedGeometry/1'
                or set(proof.get('receipts',{}))!=REQUIRED or proof.get('receipt_readbacks')!=len(REQUIRED)
                or proof.get('post_audit_custody') is not True
                or proof.get('collision_scope')!='ALL_FACES_INCLUDING_OLD_OLD'
                or not proof.get('mesh_arrays')):
            raise ValueError('CompleteFreshGradedJoinedProofRequired')
    return 'FiniteTransitionJoinedExact/1'
