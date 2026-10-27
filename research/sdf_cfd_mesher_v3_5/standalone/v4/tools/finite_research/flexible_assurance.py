"""Post-proof Q1 assurance bindings; generation receipts cannot replace them."""
from finite_research.flexible_controls import identities,validate,REPORT
from sdf_cfd_mesher_v4.common.authority import identity

PROOFS=('collision','filling','interior_degree')


def basis(placement,binding,parent_mesh,proof,controls):
    from finite_research.flexible_controls import graded
    if graded(controls):
        from finite_research.graded_assurance import basis as constructed_basis
        return constructed_basis(placement,binding,parent_mesh,proof,controls)
    validate(controls);expected=identities(controls)
    domain=placement.get('fluid_domain_evidence',{})
    fields={'backend_receipt_sha256','backend_outputs','boundary_receipts','q0','q1',
        'final_boundary_receipts','extent_policy_identity','quality_targets_identity',
        'native_request_sha256','native_input_sha256'}
    if (type(domain) is not dict or set(domain)!=fields
            or any(binding.get(name)!=digest for name,digest in expected.items())
            or placement.get('controls_identity')!=expected['controls_identity']
            or domain.get('extent_policy_identity')!=expected['extent_policy_identity']
            or domain.get('quality_targets_identity')!=expected['quality_targets_identity']
            or placement.get('parent_prism_mesh')!=parent_mesh or binding.get('parent_prism_mesh')!=parent_mesh):
        raise ValueError('ChangedFinalFlexibleDomainPolicy')
    q1=domain['q1'];q0=domain['q0']
    geometry_fields={'points_sha256','tets_sha256','outer_sha256','boundary_sha256'}
    if (type(q0) is not dict or set(q0)!=geometry_fields or type(q1) is not dict or set(q1)!=geometry_fields
            or q1!=placement.get('q1') or q0!=placement.get('q0')
            or q1.get('points_sha256')!=placement.get('points_sha256')
            or q1.get('tets_sha256')!=placement.get('tets_sha256')
            or q0['tets_sha256']!=q1['tets_sha256']):
        raise ValueError('ChangedFinalFlexibleGeometry')
    envelope=placement.get('envelope',{})
    if (envelope.get('schema')!='CapRelativeVertexPlacement/1' or envelope.get('passed') is not True
            or envelope.get('extent_policy_identity')!=expected['extent_policy_identity']
            or envelope.get('strict_bound_performed') is not (controls['envelope']['mode']=='strict')):
        raise ValueError('FreshFlexiblePlacementRequired')
    receipts=domain.get('final_boundary_receipts')
    if type(receipts) is not dict or set(receipts)!={
            'FINAL_BOUNDARY_ADMISSION.json','FINAL_BOUNDARY_COLLISION.json','FINAL_BOUNDARY_DEGREE.json'}:
        raise ValueError('FreshFinalBoundaryReceiptsRequired')
    geometry=dict(q1=q1,controls_identity=expected['controls_identity'],
        extent_policy_identity=expected['extent_policy_identity'],quality_targets_identity=expected['quality_targets_identity'],
        final_boundary_receipts=receipts,vertex_placement_identity=identity(envelope))
    if proof.get('geometry_binding')!=geometry:raise ValueError('ChangedFinalProofGeometryBinding')
    if any(proof.get(name,{}).get('geometry_identity')!=identity(geometry) for name in PROOFS):
        raise ValueError('StaleOrRelabelledFlexibleProofSubrecord')
    if (proof.get('passed') is not True or proof.get('inherited_prism_mesh')!=parent_mesh
            or proof.get('collision',{}).get('passed') is not True
            or proof['collision'].get('independent_coverage_passed') is not True
            or proof.get('filling',{}).get('positive_cells') is not True
            or proof['filling'].get('exact_volume_balance') is not True
            or proof.get('interior_degree',{}).get('passed') is not True
            or (parent_mesh is None and proof.get('collision_scope')!='ALL_FACES_INCLUDING_OLD_OLD')):
        raise ValueError('CompleteFinalFlexibleProofsRequired')
    if (type(domain['backend_outputs']) is not dict or set(domain['backend_outputs'])!=
            {'CORE.npz','OUTER.npz','GMSH.npz','NETGEN.npz'}
            or type(domain['boundary_receipts']) is not dict or set(domain['boundary_receipts'])!=
            {'BOUNDARY_ADMISSION.json','BOUNDARY_COLLISION.json','BOUNDARY_DEGREE.json'}):
        raise ValueError('CompleteQ0GeneratorProvenanceRequired')
    hashes=[*q0.values(),*q1.values(),*receipts.values(),*domain['backend_outputs'].values(),
        *domain['boundary_receipts'].values(),domain.get('backend_receipt_sha256'),
        domain.get('native_request_sha256'),domain.get('native_input_sha256'),
        placement.get('source_manifest_sha256'),binding.get('request_sha256'),*expected.values()]
    if any(type(value) is not str or len(value)!=64 or any(char not in '0123456789abcdef' for char in value)
            for value in hashes):raise ValueError('BoundFinalFlexibleHashesRequired')
    return dict(schema='V4InheritedBackendFluidDomain/2',parent_prism_mesh=parent_mesh,
        stage_request_sha256=binding['request_sha256'],stage_binding_identity=identity(binding),
        source_release_sha256=placement['source_manifest_sha256'],q0_generator_provenance=domain,
        geometry_binding=geometry,proof_identities={name:identity(proof[name]) for name in PROOFS})


def geometry_binding(placement,controls):
    from finite_research.flexible_controls import graded
    if graded(controls):
        from finite_research.graded_assurance import geometry_binding as constructed_binding
        return constructed_binding(placement,controls)
    expected=identities(controls)
    return dict(q1=placement['q1'],**expected,
        final_boundary_receipts=placement['final_boundary_receipts'],
        vertex_placement_identity=identity(placement['envelope']))


def report_schema(controls):
    from finite_research.flexible_controls import graded
    if graded(controls):return 'V4TetLayerAcceptance/5'
    from finite_research.flexible_controls import selected
    return REPORT if selected(controls) else 'V4TetLayerAcceptance/3'
