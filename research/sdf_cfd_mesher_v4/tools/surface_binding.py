"""Bindings to the existing Authority API, not geometry or meshing algorithms."""
from dataclasses import dataclass, field, asdict
import inspect
import copy
import shutil
import numpy as np
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalGeometry, GeometryFailure
from historical_capsule_surface import Authority, sha


def provenance(fn):
    path = inspect.getsourcefile(fn)
    return dict(symbol=fn.__module__+'.'+fn.__qualname__,
                source=path, sha256=sha(path))


def implemented(authority, name):
    fn = getattr(authority, name, None)
    return callable(fn) and (name in ('competing_component_sheet_refs','feature_residual','implicit_value')
        or getattr(fn, '__func__', fn) is not getattr(Authority, name, None))


@dataclass(frozen=True)
class QualificationContract:
    expected_components: int
    expected_euler: int
    expected_boundary_loops: int
    sheet_names: dict
    curve_names: dict
    corner_names: dict
    position_limit_m: float
    normal_limit_rad: float
    projection_tolerance_m: float
    premise: str
    source: str

    def validate(self):
        from procedural_authorities import POSITION_LIMIT, NORMAL_LIMIT_RAD
        if (self.position_limit_m != POSITION_LIMIT or self.normal_limit_rad != NORMAL_LIMIT_RAD
                or self.projection_tolerance_m != 2e-12):
            raise ValueError('UnqualifiedContinuousOrProjectionPolicy')
        # Current audited topology checker is closed-surface only. Do not weaken it.
        if self.expected_boundary_loops != 0:
            raise ValueError('OpenBoundaryQualificationBindingNotEstablished')
        if self.expected_components < 1 or not self.premise or not self.source:
            raise ValueError('QualificationPremiseRequired')


OPERATIONS = {
    'initial_projection': ('project_sheet', 'exact_normal'),
    'carrier_refinement': ('project_sheet', 'exact_normal', 'implicit_value', 'competing_component_sheet_refs'),
    'tensor_sizing': ('_normal_map_principal_data', 'gradient_hessian'),
    'exact_projection': ('project_sheet',),
    'independent_qualification': ('implicit_value', 'exact_normal', 'curvature_upper'),
}


@dataclass
class SurfaceBinding:
    authority: object
    geometry_identity: str
    units: str
    precision: str
    references: dict
    seed_provider: object = None
    sizing_policy: object = None
    qualification: QualificationContract | None = None
    qualifier: object = None
    canonical: CanonicalGeometry | None = None
    limitations: list = field(default_factory=list)
    operation_provenance: dict = field(default_factory=dict)
    storage_budget_bytes: int | None = None
    query_discovery: dict | None = None
    topology_discovery: dict | None = None
    topology_evidence: dict | None = None
    spline_discovery: dict | None = None
    metric_connection: object = None
    corner_restorer: object = None
    seed_route: str = 'CGALImplicitExtraction'

    def verify_metric_connection(self):
        linked=getattr(self.authority,'_surface_metric_connection',None)
        if linked is not None and linked is not self.metric_connection:
            raise GeometryFailure('MetricConnectionDetached','tensor_sizing',
                'authority metric and complete composition must remain bound',self.geometry_identity)
        if self.metric_connection is not None:self.metric_connection.verify(self)
        return self.metric_connection

    def check_storage(self, output):
        if self.storage_budget_bytes is None:return
        used=sum(p.stat().st_size for p in output.rglob('*') if p.is_file())
        if used>self.storage_budget_bytes or shutil.disk_usage(output).free<self.storage_budget_bytes:
            raise GeometryFailure('StorageBudgetExceeded','resource_guard',str(used),self.geometry_identity)

    def __post_init__(self):
        self.references = copy.deepcopy(self.references)
        if not self.operation_provenance:
            names = set(n for row in OPERATIONS.values() for n in row)
            names.update(('project_curve', 'feature_residual', 'metric', 'scalar_value',
                          'numerical_project', 'numerical_normal', 'feature_inventory',
                          'feature_evaluate', 'feature_query'))
            self.operation_provenance = {n: provenance(getattr(self.authority,n))
                for n in sorted(names) if implemented(self.authority,n)}

    def issue(self, stage, operation, required, supplied, considered, **feature):
        return dict(stage=stage, operation=operation, geometry_identity=self.geometry_identity,
                    required_premise=required, supplied_evidence=supplied,
                    existing_implementation_considered=considered, **feature)

    def inspect(self):
        issues=[]
        from polygon_boundary_contract import SourceBoundaryContract
        source_contract=isinstance(self.qualification,SourceBoundaryContract)
        from general_source_contract import GeneralSourceQualification
        general=type(self.qualification) is GeneralSourceQualification
        if general:self.qualification.verify(self.authority)
        if self.canonical is not None:
            actual=self.canonical.owner.request(op='describe',handle=self.canonical.handle)
            if actual != self.canonical.description:
                raise GeometryFailure('GeometryDescriptionMismatch','admission','native descriptor differs',self.geometry_identity)
            actual_id=self.canonical.identity
            if self.query_discovery != self.canonical.discover():
                raise GeometryFailure('OperationBindingMismatch','admission','live query discovery differs',self.geometry_identity)
            if self.topology_discovery is not None and self.topology_discovery != self.canonical.stored_topology():
                raise GeometryFailure('GeometryIdentityMismatch','admission','live stored topology differs',self.geometry_identity)
        else:
            actual_id=getattr(self.authority,'geometry_identity',None)
        if actual_id != self.geometry_identity:
            raise GeometryFailure('GeometryIdentityMismatch','admission','binding does not identify its authority',self.geometry_identity)
        if self.spline_discovery is not None:
            if (self.spline_discovery != self.canonical.stored_spline() or
                    self.spline_discovery != getattr(self.authority,'spline_packet',None)):
                raise GeometryFailure('SplineExportMismatch','admission','live spline evidence detached',self.geometry_identity)
            self.authority.verify()
            if self.seed_provider != self.authority.labelled_seed:
                raise GeometryFailure('OperationBindingMismatch','admission','spline seed provider detached',self.geometry_identity)
        affine_packet=getattr(self.authority,'packet',None)
        if affine_packet is not None:
            if affine_packet!=self.topology_discovery or self.topology_evidence is None:
                raise GeometryFailure('OperationBindingMismatch','admission','stored topology evidence detached',self.geometry_identity)
            for callback,name in ((self.seed_provider,'labelled_seed'),(self.corner_restorer,'restore_corners')):
                if callback!=getattr(self.authority,name,None):
                    raise GeometryFailure('OperationBindingMismatch','admission',name,self.geometry_identity)
        if self.units != 'm' or not self.precision:
            raise GeometryFailure('UnitPrecisionContractRequired','admission','Authority operations must use explicit SI units and precision',self.geometry_identity)
        self.verify_metric_connection()
        for name,record in self.operation_provenance.items():
            if not implemented(self.authority,name) or provenance(getattr(self.authority,name)) != record:
                raise GeometryFailure('OperationBindingMismatch','admission',name,self.geometry_identity)
        for kind in ('sheet_names','curve_names','corner_names'):
            if self.references.get(kind) != getattr(self.authority,kind,None):
                raise GeometryFailure('ReferenceBindingMismatch','admission',kind,self.geometry_identity)
        for stage,names in OPERATIONS.items():
            if general and stage=='tensor_sizing':names=('metric',)
            if general and stage=='independent_qualification':names=('implicit_value','exact_normal')
            for name in names:
                if not implemented(self.authority,name):
                    issues.append(self.issue(stage,name,'geometry-owned branch operation with valid provenance',
                        self.available_evidence(name),self.considered(name)))
        if self.references.get('curve_names'):
            for name in ('project_curve','feature_residual','metric'):
                if not implemented(self.authority,name):
                    issues.append(self.issue('feature_ownership',name,'curve-aware callable operation',
                        ('existing SF2 curve metric template available; coupled case policy not selected'
                         if self.spline_discovery and name=='metric' else 'unbound'),
                        ('SmoothAuthority.metric; shared geometry/feature/shape metric composition'
                         if self.spline_discovery and name=='metric' else self.considered(name)),curves=self.references['curve_names']))
        if not self.references.get('sheet_names'):
            issues.append(self.issue('seed_reference_handoff','persistent_sheet_curve_mapping',
                'complete geometry-owned sheet/curve/corner mapping, including nonsmooth joins',
                'inventory is not a certified face-to-sheet/edge-to-curve mapping',
                'Sdf procedural features; indexed_live_authority; provider SeedFace/SeedEdge'))
        if not callable(self.seed_provider):
            issues.append(self.issue('extraction','labelled_seed_provider','labelled seed with authoritative ownership',
                'canonical CGAL writes OFF only; no certified label handoff bound',
                'canonical_cgal_seed; existing constrained-chart seed; provider reference registry'))
        if self.sizing_policy is None:
            issues.append(self.issue('tensor_sizing','sizing_policy','source-backed policy for this geometry',
                'not supplied; capsule policy not transferred','SurfaceSizing; shape_aware_metric'))
        else:
            self.sizing_policy.validate()
        if self.qualification is None or not callable(self.qualifier):
            issues.append(self.issue('independent_qualification','continuous_geometry_premises',
                'valid sheet distance/normal/curvature bounds, feature ownership and expected topology',
                ('SF2 branch premises connected; caller limits, finite-domain/coverage acceptance and applicable case contract not bound'
                 if self.spline_discovery else 'no qualified contract bound'),'certify_face; feature_bound; audit_adaptive_surface'))
        elif source_contract:
            from audit_adaptive_surface import audit_directory
            if self.qualifier is not audit_directory:
                raise GeometryFailure('OperationBindingMismatch','admission','source-boundary qualifier detached',self.geometry_identity)
            self.qualification.verify(self.authority)
            if self.qualification.accuracy_limits is None:
                issues.append(self.issue('independent_qualification','caller_accuracy_limits',
                    'explicit source-boundary position_m and normal_rad',
                    'absent; bounds can be evaluated but acceptance is not evaluated','SourceBoundaryContract'))
        else:
            self.qualification.validate()
            for kind in ('sheet_names','curve_names','corner_names'):
                if getattr(self.qualification,kind) != self.references[kind]:
                    raise GeometryFailure('QualificationReferenceMismatch','admission',kind,self.geometry_identity)
        if self.topology_evidence is not None:
            if self.topology_evidence.get('failure'):
                failure=self.topology_evidence['failure']
                issues.append(self.issue('polygon_representation','source_topology_premises',
                    'supported stored polygon premises',failure,'CompleteAuthority; corner_authority'))
            elif not source_contract:
                issues.append(self.issue('independent_qualification','canonical_affine_consistency',
                    'qualified relation between canonical f32 field and promoted affine branches',
                    'source bits and topology preserved; continuous evaluation error bound unestablished',
                    'Airfoil.lerped_distance_inline; CompleteAuthority affine operations'))
        return dict(geometry_identity=self.geometry_identity,units=self.units,precision=self.precision,
                    coupled_metric=(dict(source_identity=self.metric_connection.source_identity,
                        length_scale=self.metric_connection.length_scale,
                        prism_policy=asdict(self.metric_connection.prism),metric_override_used=False)
                        if self.metric_connection is not None else None),
                    references=self.references,operations=self.operation_provenance,
                    executable=not issues,issues=issues,limitations=self.limitations,
                    qualification=asdict(self.qualification) if self.qualification else None,
                    source_boundary_status=(dict(target_established=True,finite_domains_available=True,
                        conversion_evidence_verified=True,numerical_scope=self.qualification.record['numerical_scope'],
                        accuracy_limits_supplied=self.qualification.accuracy_limits is not None,
                        curve_metric_supplied=implemented(self.authority,'metric'),surface_sizing_supplied=self.sizing_policy is not None)
                        if source_contract else None),
                    query_discovery=self.query_discovery,
                    spline_binding=(dict(export_identity=self.spline_discovery['spline_identity'],
                        reference_identity=self.authority.reference_identity,premises=self.authority.premises,
                        branch_operations_available=True,continuous_surface_qualified=False,
                        curve_metric_supplied=implemented(self.authority,'metric'),
                        surface_sizing_supplied=self.sizing_policy is not None,
                        qualification_contract_supplied=self.qualification is not None)
                        if self.spline_discovery else None),
                    topology_binding=self.topology_evidence,seed_route=self.seed_route,
                    topology_input=(dict(identity=self.topology_discovery['topology_identity'],
                        status=self.topology_discovery['data']['status'],
                        reason=self.topology_discovery['data'].get('reason'),
                        representation=self.topology_discovery['data'].get('representation'))
                        if self.topology_discovery else None),
                    admission_layers=(dict(scalar_extraction='available_with_enclosure_and_criteria',
                        numerical_geometry_queries='available_unqualified',
                        feature_metadata='live_optional_inventory_and_query_status',
                        stored_topology_ownership=('source_spline_physical_topology' if self.spline_discovery else
                            'complete_for_promoted_affine_source' if affine_packet else 'unestablished'),
                        affine_branch_operations='available' if affine_packet else 'unestablished',
                        qualified_branch_queries='SF2_operations_reused_no_new_surface_qualification' if self.spline_discovery else 'unestablished',continuous_certification='unestablished',
                        case_sizing='supplied' if self.sizing_policy else 'absent')
                        if self.query_discovery else None))

    def available_evidence(self, name):
        if self.query_discovery is None:
            return 'no callable Authority binding'
        return {
            'project_sheet':'numerical_project connected; branch selection, required accuracy and minimum-distance guarantee unestablished',
            'exact_normal':'numerical_normal connected; zero-direction status preserved; qualified branch normal unestablished',
            'implicit_value':'scalar_value and sampled projection residual connected; length-valued owning-sheet residual unestablished',
            'competing_component_sheet_refs':'persistent kernel IDs available where inventoried; complete sheet/component ownership unestablished',
            '_normal_map_principal_data':'numerical normal queries connected; existing principal-data consumer needs branch-consistent projection',
            'gradient_hessian':'normalized numerical direction connected; not unnormalized gradient/Hessian differential data',
            'curvature_upper':'no continuous curvature bound supplied by this query discovery; sampled directions are not bounds',
        }.get(name,'query metadata available; qualified Authority role unestablished')

    @staticmethod
    def considered(name):
        return {
            'project_sheet':'query.rs::closest_point has no branch/convergence certificate; GeometryQueryKernel projection uses conditioning; SmoothAuthority requires a different spline object',
            'project_curve':'Sdf procedural feature query exists; no certified branch projector is bound',
            'exact_normal':'query.rs::gradient is a numerical gradient, not a unique branch normal at a feature',
            'curvature_upper':'SmoothAuthority spline enclosures do not bind a clipped polygon; sampled curvature is not a bound',
            'gradient_hessian':'existing differential sizing requires branch-consistent derivatives',
        }.get(name,'existing procedural Authority/provider operations; no compatible canonical binding supplied')


class CanonicalScalarAuthority:
    """Available canonical queries, deliberately not certified Authority roles."""
    sheet_names={}; curve_names={}; corner_names={}
    def __init__(self,geometry): self.geometry=geometry
    def scalar_value(self,points_m): return np.asarray(self.geometry.query(points_m),dtype=float)
    def numerical_project(self,points_m): return self.geometry.numerical_project(points_m)
    def numerical_normal(self,points_m): return self.geometry.numerical_normal(points_m)
    def feature_inventory(self): return self.geometry.feature_inventory()
    def feature_evaluate(self,requests): return self.geometry.feature_evaluate(requests)
    def feature_query(self,points_m,options=None): return self.geometry.feature_query(points_m,options)


def bind_canonical(geometry, *, accuracy_limits=None):
    if not isinstance(geometry,CanonicalGeometry):
        raise GeometryFailure('CanonicalHandleRequired','admission','canonical object or explicit SurfaceBinding required')
    discovery=geometry.discover()
    inventory=discovery['data']['inventory']['model']
    # Keep original IDs. Diagnostic sectors are NOT converted to smooth sheets.
    identities={kind:sorted(item[key] for item in (inventory or {}).get(kind,[]))
        for kind,key in (('features','feature_id'),('sectors','sector_id'),('junctions','junction_id'))}
    binding=SurfaceBinding(CanonicalScalarAuthority(geometry),geometry.identity,'m',
        geometry.description['precision'],dict(sheet_names={},curve_names={},corner_names={}),
        canonical=geometry,query_discovery=discovery,limitations=[dict(operation='feature_inventory',
            evidence=inventory,persistent_identities=identities,
            scope='automatic kernel IDs; coverage/ownership unestablished; no authoritative integer references inferred'),
            dict(operation='scalar_value',source='CanonicalGeometry.query -> Geometry::query -> canonical Sdf::distance',
                 units=geometry.description['coordinate_rule'],field_rule=geometry.description['field_rule'])])
    packet=geometry.stored_topology()
    binding.topology_discovery=packet
    if packet['data']['status']=='available':
        from canonical_polygon_authority import PolygonAuthority
        try:
            authority=PolygonAuthority(geometry,packet)
        except GeometryFailure as error:
            binding.topology_evidence={'failure':error.record,'ready_for_surface_execution':False}
        else:
            from polygon_boundary_contract import SourceBoundaryContract
            from audit_adaptive_surface import audit_directory
            source_contract=SourceBoundaryContract.automatic(authority,accuracy_limits)
            binding=SurfaceBinding(authority,geometry.identity,'m',
                'stored binary32 topology promoted to binary64 affine branches; canonical scalar stays binary32',
                {name:getattr(authority,name) for name in ('sheet_names','curve_names','corner_names')},
                canonical=geometry,query_discovery=discovery,topology_discovery=packet,
                topology_evidence=dict(operations_available=True,topology_coverage=authority.checks,
                    canonical_consistency=authority.canonical_consistency,continuous_canonical_certificate=False,
                    ready_for_surface_execution=False,source_dependencies=authority.provenance),
                seed_provider=authority.labelled_seed,corner_restorer=authority.restore_corners,
                qualification=source_contract,qualifier=audit_directory,
                seed_route='ExistingCGALConstrainedSourceLoopCharts',
                limitations=[dict(scope='Affine branch operations are not a canonical f32 error certificate',
                    topology_identity=packet['topology_identity'],kernel_feature_ids=packet['data']['kernel_feature_ids'],
                    namespace_relation='diagnostic kernel IDs retained separately; no sector-to-sheet equivalence asserted')])
    elif packet['data'].get('reason') == 'SectionRepresentationNotPolygon':
        spline=geometry.stored_spline()
        if spline['data']['status']=='available':
            from canonical_spline_authority import LiveSplineAuthority
            from audit_adaptive_surface import audit_directory
            if accuracy_limits is not None:
                raise GeometryFailure('SplineQualificationContractBindingRequired','admission',
                    'accuracy numbers alone do not establish a spline qualification contract',geometry.identity)
            authority=LiveSplineAuthority(geometry,spline)
            binding=SurfaceBinding(authority,geometry.identity,'m',
                'stored binary64 cubic sections; binary32 placement/chord; SF2 binary64 branch queries; canonical scalar binary32',
                {n:getattr(authority,n) for n in ('sheet_names','curve_names','corner_names')},
                canonical=geometry,query_discovery=discovery,spline_discovery=spline,
                seed_provider=authority.labelled_seed,qualifier=audit_directory,
                seed_route='ExistingCGALSplineParameterCharts',
                limitations=[dict(scope='unit-chord metre-unit uniform spline extrusion only; natural TE unchanged',
                    premises=authority.premises,metric_policy='not inherited; curve and surface policy required',
                    corner_handling='existing SF2 provider projects complete incident curves and checks agreement; no polygon restorer substituted',
                    next_stage='explicit first prism thickness and metric_override composition remain separate')])
    return binding
