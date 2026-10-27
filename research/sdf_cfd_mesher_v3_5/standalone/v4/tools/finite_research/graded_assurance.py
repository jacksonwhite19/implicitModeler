"""Fresh constructed-domain assurance, distinct from native collar provenance."""
from sdf_cfd_mesher_v4.common.authority import identity
from finite_research.flexible_controls import graded,validate,identities,GRADED_CONSTRUCTION
from finite_support.stored_wall_contract import SCHEMA as WALL

REPORT='V4TetLayerAcceptance/5'
DOMAIN='V4InheritedConstructedFluidDomain/1'
ASSURANCE='V4TransitionFluidAssurance/3'
CONTRACT='InheritedPrismConstructedFluid/1'
RECEIPTS={'FINAL_BOUNDARY_ADMISSION.json','FINAL_BOUNDARY_COLLISION.json','FINAL_BOUNDARY_DEGREE.json'}

def geometry_binding(placement,controls):
    validate(controls)
    if not graded(controls):raise ValueError('GradedAssuranceControlsRequired')
    return dict(geometry=placement['geometry'],**identities(controls),
        final_boundary_receipts=placement['final_boundary_receipts'],wall_contract=WALL,
        vertex_placement_identity=identity(placement['envelope']),
        construction_identity=identity(placement['construction_status']))

def basis(placement,binding,parent_mesh,proof,controls):
    from finite_research.graded_command import require_complete
    validate(controls);expected=identities(controls)
    if not graded(controls) or parent_mesh is None:raise ValueError('LiveGradedAssuranceRequired')
    if (placement.get('parent_prism_mesh')!=parent_mesh or binding.get('parent_prism_mesh')!=parent_mesh
            or any(binding.get(name)!=value for name,value in expected.items())
            or placement.get('controls_identity')!=expected['controls_identity']):
        raise ValueError('ChangedGradedDomainPolicyOrParent')
    status=placement.get('construction_status',{})
    if (status.get('schema')!='V4GradedConstructionStatus/1' or status.get('status')!='COMPLETE_CONSTRUCTION'
            or status.get('constructor')!=GRADED_CONSTRUCTION or status.get('parent_prism_mesh')!=parent_mesh
            or any(status.get(name)!=value for name,value in expected.items())):
        raise ValueError('CompletedLiveGradedConstructionRequired')
    require_complete(status.get('construction',{}))
    if status['construction'].get('nominal_distance_m')!=controls['envelope']['nominal_distance_m']:
        raise ValueError('ChangedGradedNominalCompletion')
    domain=placement.get('fluid_domain_evidence',{})
    if (set(domain)!={'schema','construction_sha256','preparation_sha256','final_boundary_receipts'}
            or domain.get('schema')!='ConstructedGradedDomain/1'
            or set(domain.get('final_boundary_receipts',{}))!=RECEIPTS
            or domain['final_boundary_receipts']!=placement.get('final_boundary_receipts')):
        raise ValueError('FreshGradedConstructionReceiptsRequired')
    geometry=placement.get('geometry',{})
    if (set(geometry)!={'points_sha256','tets_sha256','outer_sha256','boundary_sha256','wall_sha256'}
            or geometry.get('points_sha256')!=placement.get('points_sha256')
            or geometry.get('tets_sha256')!=placement.get('tets_sha256')
            or placement.get('wall_contract')!=WALL):
        raise ValueError('ChangedGradedFinalGeometry')
    envelope=placement.get('envelope',{})
    if (envelope.get('passed') is not True or envelope.get('schema')!='CapRelativeVertexPlacement/1'
            or envelope.get('strict_bound_performed') is not False
            or envelope.get('extent_policy_identity')!=expected['extent_policy_identity']):
        raise ValueError('FreshNominalGradedPlacementRequired')
    bound=geometry_binding(placement,controls)
    if proof.get('geometry_binding')!=bound:raise ValueError('ChangedGradedProofBinding')
    if (proof.get('passed') is not True or proof.get('inherited_prism_mesh')!=parent_mesh
            or proof.get('collision_scope')!='ALL_FACES_INCLUDING_OLD_OLD'
            or proof.get('collision',{}).get('passed') is not True
            or proof['collision'].get('independent_coverage_passed') is not True
            or proof.get('filling',{}).get('positive_cells') is not True
            or proof['filling'].get('exact_volume_balance') is not True
            or proof.get('interior_degree',{}).get('passed') is not True):
        raise ValueError('CompleteGradedFreshProofsRequired')
    for name in ('collision','filling','interior_degree'):
        if proof[name].get('geometry_identity')!=identity(bound):raise ValueError('ChangedGradedProofSubrecord',name)
    hashes=[*geometry.values(),*domain['final_boundary_receipts'].values(),domain['construction_sha256'],
        domain['preparation_sha256'],placement.get('source_manifest_sha256'),binding.get('request_sha256')]
    if any(type(value) is not str or len(value)!=64 or any(char not in '0123456789abcdef' for char in value) for value in hashes):
        raise ValueError('BoundGradedHashesRequired')
    return dict(schema=DOMAIN,parent_prism_mesh=parent_mesh,stage_request_sha256=binding['request_sha256'],
        stage_binding_identity=identity(binding),source_release_sha256=placement['source_manifest_sha256'],
        construction_provenance=domain,geometry_binding=bound,
        proof_identities={name:identity(proof[name]) for name in ('collision','filling','interior_degree')})

