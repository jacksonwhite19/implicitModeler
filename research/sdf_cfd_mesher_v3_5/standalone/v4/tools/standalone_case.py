"""Run-owned requirements at the existing geometry/metric binding boundary.

No historical cases, saved planner decisions or implicit numeric defaults.
Supported geometry is checked by bind_canonical, not by a case name/hash.
"""
import copy
import dataclasses
import json
import math
import hashlib
import shutil
from contextlib import contextmanager, ExitStack
from contextvars import ContextVar
from types import MappingProxyType
from pathlib import Path
from types import MethodType
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, file_hash


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


_import_scopes=ContextVar('standalone_import_scopes',default=None)


@dataclasses.dataclass(frozen=True)
class _ImportedInput:
    case_path: Path
    source_path: Path
    source_sha256: str
    geometry_json: str
    script: str


def _import_snapshot(path):
    scopes=_import_scopes.get()
    if scopes is None:return None
    lexical=str(Path(path).absolute())
    return scopes.get(lexical) or scopes.get(str(Path(path).resolve()))


def input_case_path(path):
    snapshot=_import_snapshot(path)
    return snapshot.case_path if snapshot is not None else Path(path).resolve()


def held_import_sha256(path):
    """Bind the protected source across the existing supervisor/worker handoff."""
    snapshot=_import_snapshot(path)
    if snapshot is not None:return snapshot.source_sha256
    if imported_geometry(path,read(path)) is not None:raise ValueError('ImportedSourceCustodyRequired')
    return None


def imported_geometry(path,case):
    """Resolve explicit Rhai input; optional caller pin strengthens identity."""
    g=case.get('geometry',{})
    if not isinstance(g,dict) or 'rhai_file' not in g:return None
    required=set(('rhai_file','metres_per_model_unit'))
    pin=g.get('rhai_sha256')
    if (set(g) not in (required,required|{'rhai_sha256'})
            or not isinstance(g['rhai_file'],str) or not g['rhai_file'].strip()
            or ('rhai_sha256' in g and (type(pin) is not str or len(pin)!=64
                or any(c not in '0123456789abcdef' for c in pin)))):
        raise ValueError('ExplicitGeometryFileRequired')
    units=g['metres_per_model_unit']
    if type(units) not in (float,int) or not math.isfinite(units) or units<=0:
        raise ValueError('ExplicitPositiveUnitScaleRequired')
    snapshot=_import_snapshot(path)
    if snapshot is not None:
        if json.dumps(g,sort_keys=True)!=snapshot.geometry_json:raise ValueError('ChangedImportedCaseInputs')
        return snapshot.source_path,pin
    source=Path(g['rhai_file'])
    if not source.is_absolute():source=Path(path).resolve().parent/source
    return source.resolve(),pin


def _geometry_script(source,pin):
    from immutable_state_access import guard
    with guard(source):
        # Match the existing canonical transport's bounded root-source input.
        with source.open('rb') as stream:raw=stream.read(8*1024*1024+1)
        if len(raw)>8*1024*1024:raise ValueError('GeometrySourceTooLarge')
        if pin is not None and hashlib.sha256(raw).hexdigest()!=pin:raise ValueError('GeometrySourceIdentityMismatch')
        try:script=raw.decode('utf-8-sig')
        except UnicodeDecodeError as error:raise ValueError('GeometrySourceUTF8Required') from error
        if not script.strip():raise ValueError('CanonicalConstructorRequired')
    return script,hashlib.sha256(raw).hexdigest()


def materialize_geometry(path,case):
    imported=imported_geometry(path,case)
    if imported is None:return case
    snapshot=_import_snapshot(path)
    script=snapshot.script if snapshot is not None else _geometry_script(*imported)[0]
    result=copy.deepcopy(case)
    result['geometry']=dict(rhai=script,metres_per_model_unit=case['geometry']['metres_per_model_unit'])
    return result


@contextmanager
def hold_inputs(path):
    """Use the existing custody owner for case and imported root source."""
    from immutable_state_access import guard
    lexical=str(Path(path).absolute());path=input_case_path(path)
    with ExitStack() as held:
        held.enter_context(guard(path))
        raw=read(path);imported=imported_geometry(path,raw)
        if imported is not None:
            source,pin=imported
            held.enter_context(guard(source))
            script,actual=_geometry_script(source,pin)
            snapshot=_import_snapshot(path)
            if snapshot is not None and actual!=snapshot.source_sha256:raise ValueError('ChangedImportedSourceInputs')
            snapshot=_ImportedInput(path,source,actual,json.dumps(raw['geometry'],sort_keys=True),script)
            scopes=dict(_import_scopes.get() or {});scopes.update({lexical:snapshot,str(path):snapshot})
            token=_import_scopes.set(MappingProxyType(scopes))
            try:yield
            finally:_import_scopes.reset(token)
        else:yield


def copy_inputs(path,target):
    """Keep legacy bytes; capture imported source in the same case format."""
    from path_preserving_deduplication import durable_json
    path,target=input_case_path(path),Path(target).resolve()
    if target.exists():raise ValueError('PreserveCopiedMeshCase')
    with hold_inputs(path):
        case,_,original=load(path)
        imported=imported_geometry(path,read(path))
        if imported is None:
            shutil.copyfile(path,target)
            if file_hash(target)!=original:raise ValueError('CopiedMeshCaseMismatch')
        else:
            if (target.parent/'CASE_IMPORT.json').exists():raise ValueError('PreserveCaseImportProvenance')
            durable_json(target,case)
            # The effective input has inline source. Later owners use this
            # self-contained case, rather than reopen a mutable external path.
            durable_json(target.parent/'CASE_IMPORT.json',dict(schema='V4MeshCaseImport/1',
                authoritative=False,original_case_sha256=original,
                original_case_path=str(path),source_path=str(imported[0]),source_sha256=_import_snapshot(path).source_sha256,
                caller_source_sha256=read(path)['geometry'].get('rhai_sha256'),
                effective_case_sha256=file_hash(target),effective_source='inline Rhai in CASE.json',
                scope='Input provenance only; no mesh acceptance or restart authority'))
    return file_hash(target)


def load(path):
    path=input_case_path(path);case=read(path)
    if case.get('schema') not in ('V4MeshCase/1','V4MeshCase/2','V4MeshCase/3','V4MeshCase/4'):raise ValueError('UnsupportedMeshCase')
    fields=set(('schema','geometry','surface','prisms','resources','purpose'))
    if case['schema']=='V4MeshCase/2':fields.add('enclosure')
    if case['schema']=='V4MeshCase/3' or case['schema']=='V4MeshCase/4' and 'background' in case:fields.add('background')
    if case['schema']=='V4MeshCase/4':fields.add('general_sdf')
    if set(case)!=fields:
        raise ValueError('MeshCaseFieldsMismatch')
    case=materialize_geometry(path,case)
    if case['schema']=='V4MeshCase/4':
        from general_sdf_binding import validate_general_policy
        validate_general_policy(case)
    g=case['geometry']
    if 'rhai' in g:
        if set(g)!=set(('rhai','metres_per_model_unit')) or not isinstance(g['rhai'],str) or not g['rhai'].strip():
            raise ValueError('CanonicalConstructorRequired')
    else:
        from standalone_geometry import validate_analytic
        validate_analytic(g)
    if not math.isfinite(g['metres_per_model_unit']) or g['metres_per_model_unit']<=0:
        raise ValueError('ExplicitPositiveUnitScaleRequired')
    if not isinstance(case['purpose'],str) or not case['purpose'].strip():raise ValueError('CasePurposeRequired')
    if 'enclosure' in case:
        from standalone_geometry import validate_enclosure
        validate_enclosure(case['enclosure'])
    if 'background' in case:
        from standalone_background import validate_policy
        validate_policy(case['background'])
    s=case['surface']
    required=('minimum_size_m','maximum_size_m','position_limit_m','normal_limit_rad',
              'metric_multiplier','shape_response_reserve','normal_qualification_fraction',
              'carrier_maximum_depth','grading','length_scale','prism_host')
    if set(s)!=set(required):raise ValueError('ExplicitSurfaceRequirementsRequired')
    if any(not isinstance(v,(float,int)) or isinstance(v,bool) or not math.isfinite(v)
           for k,v in s.items() if k!='prism_host'):raise ValueError('InvalidSurfaceRequirement')
    p=case['prisms']
    expected=set(('first_full_normal_thickness_m_hex','later_path_lengths_m_hex','fluid_side'))
    scalar='later_normal_increments_m_hex' in p
    if scalar:
        expected.remove('later_path_lengths_m_hex');expected.update(('later_normal_increments_m_hex','growth_hex'))
    if 'trajectory_completion' in p:
        if scalar or 'general_sdf' not in case or 'background' not in case:
            raise ValueError('PrismDirectionCompletionRequiresGeneralSourceBackground')
        from accepted_trajectory_contract import validate_direction_controls
        validate_direction_controls(p['trajectory_completion'])
        expected.add('trajectory_completion')
    if set(p)!=expected:
        raise ValueError('ExplicitPrismRequirementsRequired')
    from production_prism_sizing import _hex64
    increments=[_hex64(p['first_full_normal_thickness_m_hex'])]
    key='later_normal_increments_m_hex' if scalar else 'later_path_lengths_m_hex'
    if not isinstance(p[key],list):raise ValueError('ExplicitPathScheduleRequired')
    increments.extend(_hex64(v) for v in p[key])
    if scalar:_hex64(p['growth_hex'])  # metadata, never used to regenerate stored increments
    if scalar!=('rhai' not in g):raise ValueError('UnsupportedGeometryTrajectoryQuantity')
    if p['fluid_side']!='positive_sdf':raise ValueError('UnsupportedSourceSheetFluidSide')
    if s['prism_host'] is not None:
        host=s['prism_host']
        if set(host)!=set(('first_height_m','minimum_owned_layer_thickness_m',
                           'foundation_qualification_target','metric_response_factor','reference_shape')):
            raise ValueError('ExplicitPrismHostRequirementsRequired')
        if host['first_height_m']!=increments[0]:raise ValueError('DifferentSurfacePrismFirstHeight')
    r=case['resources']
    if set(r)!=set(('minimum_free_bytes','output_limit_bytes')):raise ValueError('ExplicitOperationalPolicyRequired')
    if type(r['minimum_free_bytes']) is not int or r['minimum_free_bytes']<0:
        raise ValueError('InvalidOperationalReserve')
    if r['output_limit_bytes'] is not None and (type(r['output_limit_bytes']) is not int or r['output_limit_bytes']<=0):
        raise ValueError('InvalidOperationalOutputLimit')
    return case,np.asarray(increments,dtype='<f8'),file_hash(path)


@dataclasses.dataclass(frozen=True)
class SourceQualification:
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
    geometry_identity: str
    spline_identity: str
    reference_identity: str
    case_path: str
    case_sha256: str

    def validate(self):
        from surface_binding import QualificationContract
        QualificationContract.validate(self)
        if file_hash(self.case_path)!=self.case_sha256:raise ValueError('ChangedRunCase')

    def verify(self,a):
        self.validate();a.verify()
        if (a.geometry_identity,a.spline_packet['spline_identity'],a.reference_identity)!=(
                self.geometry_identity,self.spline_identity,self.reference_identity):
            raise ValueError('SourceQualificationGeometryMismatch')

    def check_source_coverage(self,a,points,faces,refs,curves,corners):
        from sf2_surface_policy import source_coverage
        self.verify(a)
        return source_coverage(a,points,refs,curves,corners,
            'source-labelled spline charts and preserved incidence; no arbitrary-triangle finite-cap predicate')


def bind(binding,path):
    from historical_capsule_surface import SurfaceSizing
    from canonical_spline_authority import LiveSplineAuthority,existing_authority
    from coupled_surface_metric import CoupledSurfaceMetric
    from shape_aware_metric import ShapeMetricPolicy
    from prism_host_metric import PrismMetricPolicy
    from surface_binding import provenance
    from audit_adaptive_surface import audit_directory
    case,_,pin=load(path);s=case['surface'];a=binding.authority
    if type(a) is not LiveSplineAuthority:
        from standalone_geometry import bind_analytic
        return bind_analytic(binding,path)
    if callable(a.metric):raise ValueError('ExistingMetricBindingMustNotBeOverwritten')
    sizing=SurfaceSizing(minimum_size_m=s['minimum_size_m'],maximum_size_m=s['maximum_size_m'],
        position_limit_m=s['position_limit_m'],normal_limit_deg=math.degrees(s['normal_limit_rad']),
        metric_multiplier=s['metric_multiplier'],shape_response_reserve=s['shape_response_reserve'],
        normal_qualification_fraction=s['normal_qualification_fraction'],
        carrier_maximum_depth=s['carrier_maximum_depth'],grading=s['grading'])
    sizing.validate()
    if s['length_scale']<=0:raise ValueError('InvalidMetricLengthScale')
    prism=PrismMetricPolicy(**s['prism_host']) if s['prism_host'] is not None else None
    if prism is None:raise ValueError('UncoupledStandalonePolicyNotBound')
    # The primitives validate coupled feasibility and SPD constraints themselves.
    from owned_runtime import SURFACE
    q=SourceQualification(1,2,0,copy.deepcopy(a.sheet_names),copy.deepcopy(a.curve_names),
        copy.deepcopy(a.corner_names),s['position_limit_m'],s['normal_limit_rad'],2e-12,
        'Stored cubic source charts, physical incidence, exact corners, native branches and derivative-hull curvature',
        str(SURFACE/'continuous_geometry_certifier.py'),a.geometry_identity,a.spline_packet['spline_identity'],
        a.reference_identity,str(Path(path).resolve()),pin)
    class Connection(CoupledSurfaceMetric):
        def verify(self,b):
            self.sizing.validate();q.verify(a)
            if file_hash(path)!=pin or b.authority is not a or b.metric_connection is not self:
                raise ValueError('RunOwnedMetricConnectionDetached')
            if b.sizing_policy!=sizing or b.qualification!=q or b.qualifier is not audit_directory:
                raise ValueError('RunOwnedQualificationDetached')
            if (self.prism,self.shape,self.length_scale)!=(prism,ShapeMetricPolicy(),s['length_scale']):
                raise ValueError('RunOwnedMetricPolicyChanged')
    connection=Connection(a,sizing,prism,ShapeMetricPolicy(),s['length_scale'],identity(case))
    connection.feature_unit=existing_authority(a.source,maximum_size=sizing.maximum_size_m)
    def metric(owner,vertex,refs,multiplier):
        if multiplier!=sizing.metric_multiplier or vertex.sheet_ref not in owner.sheet_names or set(refs)-set(owner.curve_names):
            raise ValueError('RunOwnedFeatureMetricMismatch')
        return connection.feature_unit.metric(vertex,refs,multiplier)
    a._surface_metric_connection=connection;a.metric=MethodType(metric,a)
    binding.metric_connection=connection;binding.sizing_policy=sizing
    binding.qualification=q;binding.qualifier=audit_directory
    binding.operation_provenance['metric']=provenance(a.metric)
    connection.verify(binding)
    return binding
