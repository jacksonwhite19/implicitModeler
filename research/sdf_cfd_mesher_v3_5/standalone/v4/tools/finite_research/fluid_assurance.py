"""Data contracts only; the live prism/stage owners remain the authority."""
from finite_research.input_contract import fluid_choice
from sdf_cfd_mesher_v4.common.authority import identity

REPORT_SCHEMA='V4TetLayerAcceptance/3'
ROUTINE_SCOPE='Accepted discretized prism boundary and supported backend fill; continuous original-source solid exclusion was not independently proved.'


def execute_fluid(mode,fluid,points,cells):
    """Called only after the admission owner authenticates this selection."""
    fluid_choice({'transition_fluid_verification':mode})
    if mode=='backend_trusted':
        return dict(status='NOT_PERFORMED',performed=False,scope=ROUTINE_SCOPE)
    if not callable(fluid):raise ValueError('ContinuousSourceProofRequired')
    result=fluid(points,cells)
    if type(result) is not dict or result.get('passed') is not True:
        raise ValueError('TetLayerFluidSideFailure')
    return result


def basis(placement,binding,parent_mesh):
    domain=placement.get('fluid_domain_evidence')
    fields={'backend_receipt_sha256','backend_outputs','boundary_receipts'}
    if (type(domain) is not dict or set(domain)!=fields
            or domain['backend_receipt_sha256']!=placement.get('backend_receipt_sha256')
            or type(domain['backend_outputs']) is not dict
            or set(domain['backend_outputs'])!={'CORE.npz','OUTER.npz','GMSH.npz','NETGEN.npz'}
            or type(domain['boundary_receipts']) is not dict
            or set(domain['boundary_receipts'])!={'BOUNDARY_ADMISSION.json','BOUNDARY_COLLISION.json','BOUNDARY_DEGREE.json'}
            or placement.get('parent_prism_mesh')!=parent_mesh
            or binding.get('parent_prism_mesh')!=parent_mesh):
        raise ValueError('CurrentBackendFluidDomainEvidenceRequired')
    hashes=[domain['backend_receipt_sha256'],*domain['backend_outputs'].values(),*domain['boundary_receipts'].values(),
        placement.get('points_sha256'),placement.get('tets_sha256'),placement.get('controls_identity'),
        placement.get('source_manifest_sha256'),binding.get('request_sha256')]
    if any(type(x) is not str or len(x)!=64 or any(c not in '0123456789abcdef' for c in x) for x in hashes):
        raise ValueError('BoundFluidDomainIdentitiesRequired')
    return dict(schema='V4InheritedBackendFluidDomain/1',parent_prism_mesh=parent_mesh,
        stage_request_sha256=binding['request_sha256'],stage_binding_identity=identity(binding),
        construction_points_sha256=placement['points_sha256'],construction_tets_sha256=placement['tets_sha256'],
        controls_identity=placement['controls_identity'],source_release_sha256=placement['source_manifest_sha256'],
        **domain)


def assurance(mode,fluid_report,domain):
    fluid_choice({'transition_fluid_verification':mode})
    if mode=='backend_trusted':
        if fluid_report!=dict(status='NOT_PERFORMED',performed=False,scope=ROUTINE_SCOPE):
            raise ValueError('UnperformedContinuousProofMustBeExplicit')
    elif type(fluid_report) is not dict or fluid_report.get('passed') is not True:
        raise ValueError('SuccessfulContinuousSourceProofRequired')
    if domain.get('schema')=='V4InheritedConstructedFluidDomain/1':
        return dict(schema='V4TransitionFluidAssurance/3',mode=mode,
            contract='InheritedPrismConstructedFluid/1' if mode=='backend_trusted' else 'ContinuousSourceFluid/1',
            continuous_source_proof='NOT_PERFORMED' if mode=='backend_trusted' else 'PASSED',
            domain_basis=domain,
            scope='Accepted discretized prism boundary and freshly proved support-cell domain; original-source continuous exclusion is separate.')
    flexible=domain.get('schema')=='V4InheritedBackendFluidDomain/2'
    return dict(schema='V4TransitionFluidAssurance/2' if flexible else 'V4TransitionFluidAssurance/1',mode=mode,
        contract=('InheritedPrismBackendFluid/2' if flexible else 'InheritedPrismBackendFluid/1') if mode=='backend_trusted' else 'ContinuousSourceFluid/1',
        continuous_source_proof='NOT_PERFORMED' if mode=='backend_trusted' else 'PASSED',
        domain_basis=domain,
        scope=ROUTINE_SCOPE if mode=='backend_trusted' else 'Continuous real-source sign proved for every new transition cell; other mandatory admission owners remain separate.')


def validate_report(report,mode,binding,parent_mesh,release_sha256,expected_cells):
    from finite_research.flexible_assurance import report_schema
    from finite_research.flexible_controls import selected
    flexible='extent_policy_identity' in binding
    expected_schema=report_schema(report.get('controls')) if flexible else REPORT_SCHEMA
    if (type(report) is not dict or report.get('schema')!=expected_schema
            or selected(report.get('controls')) is not flexible or report.get('passed') is not True
            or report.get('stage_binding')!=binding or report.get('parent_prism_mesh')!=parent_mesh
            or report.get('transition_fluid_verification')!=mode):
        raise ValueError('ChangedTransitionFluidAcceptanceReport')
    if flexible and identity(report['controls'])!=binding.get('controls_identity'):
        raise ValueError('ChangedAuthenticatedFlexibleReportControls')
    placement=report['placement'];proof=report['proof']
    if proof.get('inherited_prism_mesh')!=parent_mesh:
        raise ValueError('ChangedTransitionProofParent')
    if (placement.get('source_manifest_sha256')!=release_sha256
            or identity(report.get('controls'))!=placement.get('controls_identity')):
        raise ValueError('ChangedTransitionFluidSourceRelease')
    if (placement.get('passed') is not True or placement.get('quality',{}).get('passed') is not True
            or placement.get('shape',{}).get('passed') is not True
            or type(report.get('transition_cells')) is not int or report['transition_cells']<=0
            or type(expected_cells) is not int or report['transition_cells']!=expected_cells):
        raise ValueError('IncompleteTransitionPlacement')
    if mode=='continuous_source' and proof.get('fluid',{}).get('cells')!=report['transition_cells']:
        raise ValueError('IncompleteContinuousSourceCellCoverage')
    if selected(report.get('controls')):
        from finite_research.flexible_assurance import basis as final_basis
        domain=final_basis(placement,binding,parent_mesh,proof,report['controls'])
    else:domain=basis(placement,binding,parent_mesh)
    expected=assurance(mode,proof.get('fluid'),domain)
    if report.get('fluid_assurance')!=expected or proof.get('fluid_assurance')!=expected:
        raise ValueError('ChangedTransitionFluidEvidence')
    if (proof.get('passed') is not True or proof.get('collision',{}).get('passed') is not True
            or proof['collision'].get('independent_coverage_passed') is not True
            or proof.get('filling',{}).get('positive_cells') is not True
            or proof['filling'].get('exact_volume_balance') is not True
            or proof.get('interior_degree',{}).get('passed') is not True):
        raise ValueError('IncompleteTransitionAdmission')
    return expected


def validate_component(result,mode,binding):
    """Keep the accepted contract visible across the final CLI return seam."""
    detail=result.get('fluid_assurance',{})
    flexible='extent_policy_identity' in binding
    from finite_research.flexible_controls import GRADED_CONSTRUCTION
    graded=binding.get('constructor_schema')==GRADED_CONSTRUCTION
    schema='V4TetLayerAcceptance/5' if graded else ('V4TetLayerAcceptance/4' if flexible else REPORT_SCHEMA)
    domain_schema='V4InheritedConstructedFluidDomain/1' if graded else ('V4InheritedBackendFluidDomain/2' if flexible else 'V4InheritedBackendFluidDomain/1')
    if (result.get('passed') is not True or result.get('transition_accepted') is not True
            or result.get('acceptance_schema')!=schema
            or detail.get('domain_basis',{}).get('schema')!=domain_schema
            or result.get('transition_fluid_verification')!=mode
            or detail.get('mode')!=mode
            or detail.get('domain_basis',{}).get('stage_binding_identity')!=identity(binding)
            or detail['domain_basis'].get('parent_prism_mesh')!=binding.get('parent_prism_mesh')
            or detail.get('continuous_source_proof')!=('NOT_PERFORMED' if mode=='backend_trusted' else 'PASSED')):
        raise ValueError('ChangedTransitionFluidComponentReturn')
    if flexible:
        domain=detail['domain_basis'];geometry=domain.get('geometry_binding',{})
        if (detail.get('schema')!=('V4TransitionFluidAssurance/3' if graded else 'V4TransitionFluidAssurance/2')
                or any(geometry.get(name)!=binding.get(name) for name in
                    ('controls_identity','extent_policy_identity','quality_targets_identity'))
                or (graded and geometry.get('constructor_schema')!=GRADED_CONSTRUCTION)
                or detail.get('contract')!=(('InheritedPrismConstructedFluid/1' if graded else 'InheritedPrismBackendFluid/2') if mode=='backend_trusted' else 'ContinuousSourceFluid/1')):
            raise ValueError('ChangedFlexibleComponentDomain')
    return mode
