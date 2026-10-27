"""Source-locked SF2 development inputs, separate from a prism request.

No surface identity is invented; this binding does not authorize prism creation.
The successful source-chart qualification contract is reused, not replaced by
polygon finite-domain or numerical premises.
"""
import ast
import copy
import dataclasses
import json
import math
from pathlib import Path
from types import MethodType
import numpy as np
from historical_capsule_surface import REPO, SurfaceSizing, sha, identity, provider
from surface_binding import QualificationContract, provenance
from canonical_spline_authority import LiveSplineAuthority, existing_authority
from prism_host_metric import PrismMetricPolicy
from shape_aware_metric import ShapeMetricPolicy
from procedural_authorities import POSITION_LIMIT, NORMAL_LIMIT_RAD, HMAX
from coupled_surface_metric import CoupledSurfaceMetric

ROOT = REPO / 'research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09'
CASE = REPO / 'research/sdf_cfd_mesher_v4/cases/live_spline_sf2_coupled_development.json'


def read(path): return json.loads(Path(path).read_text())


def load_case(path=CASE):
    c = read(path)
    if c.get('schema') != 'V4SF2CoupledDevelopmentPolicy/1' or not c.get('source_files'):
        raise ValueError('UnsupportedCoupledPolicy')
    for file, expected in c['source_files'].items():
        if sha(REPO / file) != expected:
            raise ValueError('SF2PolicySourceMismatch:' + file)
    call = read(ROOT/'SF2/A/wing/FROZEN_SURFACE_CALL.json')
    prior = read(ROOT/'SF2/A/wing/surface/A/provider_result_A.json')
    request = read(ROOT/'SF2/FROZEN_CONTRACT.json')
    written = read(ROOT/'SF2/A/wing/WRITTEN_METRIC_VERIFICATION.json')
    first = read(ROOT/'SF2/A/wing/FIRST_ROW_GATE.json')
    gate = read(ROOT/'SF2/A/wing/SURFACE_GATE_A.json')
    h1 = next(ast.literal_eval(n.value) for n in ast.parse((ROOT/'study.py').read_text()).body
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id=='H1' for t in n.targets))
    p = c['policy']; host = prior['prism_host_metric']
    if not all((first['pass_gate'], gate['pass_gate'], prior['pass_before_geometry_certificate'],
                written['pass_gate'], written['exact_binary64_tensor_agreement'])):
        raise ValueError('SF2RetainedQualificationFailed')
    command = call['command']
    for field, flag in [('first_height_m','--prism-first-height'),
            ('minimum_owned_layer_thickness_m','--prism-minimum-owned-layer-thickness'),
            ('foundation_qualification_target','--prism-foundation-target'),
            ('metric_response_factor','--prism-metric-response-factor')]:
        if p[field] != host[field] or p[field] != float(command[command.index(flag)+1]):
            raise ValueError('SF2PolicyValueMismatch:'+field)
    shape = host['shape_aware_tangent_conditioning']
    expected = dict(length_scale=written['scale'], position_limit_m=POSITION_LIMIT,
        normal_limit_deg=math.degrees(NORMAL_LIMIT_RAD), maximum_size_m=HMAX,
        minimum_size_m=prior['adaptive_sdf_metric_carrier']['minimum_edge_m'],
        metric_multiplier=prior['metric_multiplier'],
        normal_qualification_fraction=shape['normal_qualification_fraction'],
        shape_response_reserve=shape['mmgs_response_reserve'],
        carrier_maximum_depth=prior['adaptive_sdf_metric_carrier']['maximum_depth'],
        reference_shape=host['reference_shape'])
    for field, value in expected.items():
        if p[field] != value: raise ValueError('SF2PolicyValueMismatch:'+field)
    if (h1 != p['first_height_m'] or h1 != request['h1_m'] or
            h1.hex() != p['first_height_m_hex'] or p['first_height_meaning'] != 'full first-layer normal thickness' or
            p['length_scale'] != call['resolution_scale']):
        raise ValueError('SF2HeightOrScaleMismatch')
    if c['schedule']['nominal_growth'] != request['growth'] or c['schedule']['layers'] != request['layers']:
        raise ValueError('SF2ScheduleProvenanceMismatch')
    if (c['qualification']['projection_residual_m'] != 2e-12 or
            (c['qualification']['components'],c['qualification']['euler'],c['qualification']['boundary_loops']) != (1,2,0)):
        raise ValueError('SF2QualificationPolicyMismatch')
    return c


@dataclasses.dataclass(frozen=True)
class SF2Sizing(SurfaceSizing):
    coupled_package: str = 'SF2CoupledSmoothControl/1'
    first_height_m: float = 0.000166527069
    minimum_owned_layer_thickness_m: float = 0.00019373087622166372
    foundation_qualification_target: float = 0.00121
    metric_response_factor: float = 0.7250391657507819
    length_scale: float = 0.64

    def validate(self):
        super().validate()
        if self != SF2Sizing(): raise ValueError('UnqualifiedSF2PolicyChange')


@dataclasses.dataclass(frozen=True)
class SF2Qualification(QualificationContract):
    geometry_identity: str
    spline_identity: str
    reference_identity: str
    policy_path: str
    policy_identity: str

    def validate(self):
        super().validate()
        if identity(load_case(self.policy_path)) != self.policy_identity:
            raise ValueError('SF2QualificationSourceMismatch')

    def verify(self, a):
        self.validate()
        if (a.geometry_identity != self.geometry_identity or
                a.spline_packet['spline_identity'] != self.spline_identity or
                a.reference_identity != self.reference_identity):
            raise ValueError('SF2QualificationGeometryMismatch')

    def check_source_coverage(self, a, points, faces, refs, curves, corners):
        self.verify(a)
        return source_coverage(a,points,refs,curves,corners,
            load_case(self.policy_path)['qualification']['finite_domain_basis'])


def source_coverage(a,points,refs,curves,corners,finite_domain_basis):
    """One unchanged source-coverage predicate shared by all case bindings."""
    if (set(refs) != set(a.sheet_names) or set(curves[:,2]) != set(a.curve_names) or
            set(corners)-{0} != set(a.corner_names)):
        raise ValueError('SF2CompleteSourceCoverageRequired')
    exact = all(np.array_equal(points[i], a.corner_points[int(r)])
                for i,r in enumerate(corners) if r)
    if not exact: raise ValueError('SF2ExactSourceCornerRequired')
    return dict(exact_source_corners=exact, complete_source_coverage=True,
        incidence_check='existing audit_directory provider.topology_report; not duplicated here',
        finite_domain_basis=finite_domain_basis)


def feature_metric(a, vertex, refs, multiplier):
    connection = a._surface_metric_connection
    if multiplier != connection.sizing.metric_multiplier: raise ValueError('SF2MultiplierMismatch')
    if vertex.sheet_ref not in a.sheet_names or set(refs)-set(a.curve_names):
        raise ValueError('SF2MetricReferenceMismatch')
    return connection.feature_unit.metric(vertex, refs, multiplier)


class SF2Connection(CoupledSurfaceMetric):
    def verify(self, binding):
        self.sizing.validate()
        if identity(load_case(self.policy_path)) != self.source_identity:
            raise ValueError('SF2PolicyIdentityMismatch')
        if (binding.authority is not self.authority or binding.sizing_policy != self.sizing or
                binding.qualification != self.qualification or binding.qualifier is not self.qualifier):
            raise ValueError('SF2ConnectionDetached')
        if (self.prism != PrismMetricPolicy(self.sizing.first_height_m,self.sizing.foundation_qualification_target,
                self.sizing.minimum_owned_layer_thickness_m,self.sizing.metric_response_factor) or
                self.shape != ShapeMetricPolicy() or self.length_scale != self.sizing.length_scale):
            raise ValueError('SF2CompositionPolicyMismatch')
        self.qualification.verify(self.authority)


def bind_sf2_policy(binding, path=CASE):
    case = load_case(path); a = binding.authority; a.verify()
    if not isinstance(a,LiveSplineAuthority): raise ValueError('SF2SplineRepresentationRequired')
    if (a.geometry_identity != case['geometry']['canonical_identity'] or
            a.spline_packet['spline_identity'] != case['geometry']['spline_identity'] or
            a.spline_packet['metres_per_model_unit'] != case['geometry']['metres_per_model_unit']):
        raise ValueError('SF2CaseGeometryMismatch')
    if callable(a.metric): raise ValueError('ExistingMetricBindingMustNotBeOverwritten')
    p=case['policy']; sizing=SF2Sizing(); sizing.validate()
    # Values are explicit in this source-locked development package. A changed
    # input needs its own verified package, not a silently inherited default.
    for name in dataclasses.asdict(sizing):
        if name in p and p[name] != getattr(sizing,name): raise ValueError('SF2SizingMismatch:'+name)
    prism=PrismMetricPolicy(p['first_height_m'],p['foundation_qualification_target'],
        p['minimum_owned_layer_thickness_m'],p['metric_response_factor'],p['reference_shape'])
    connection=SF2Connection(a,sizing,prism,ShapeMetricPolicy(),p['length_scale'],identity(case))
    connection.policy_path=str(Path(path).resolve())
    connection.feature_unit=existing_authority(a.source,maximum_size=sizing.maximum_size_m)
    a._surface_metric_connection=connection; a.metric=MethodType(feature_metric,a)
    from audit_adaptive_surface import audit_directory
    q=SF2Qualification(1,2,0,copy.deepcopy(a.sheet_names),copy.deepcopy(a.curve_names),
        copy.deepcopy(a.corner_names),POSITION_LIMIT,NORMAL_LIMIT_RAD,2e-12,
        'SF2 stored cubic source charts, complete physical incidence, exact corners, native spline branches and derivative-hull curvature; adaptive signed-distance Hessian and finite-curve certificates',
        str(ROOT/'complete_surface_gate.py'),a.geometry_identity,a.spline_packet['spline_identity'],
        a.reference_identity,connection.policy_path,identity(case))
    binding.qualification=q; binding.qualifier=audit_directory; binding.sizing_policy=sizing
    binding.metric_connection=connection; connection.qualification=q; connection.qualifier=audit_directory
    binding.operation_provenance['metric']=provenance(a.metric)
    binding.limitations.append(dict(package=case['package'],authorization=case['authorization'],
        qualification_scope=case['qualification'],production_prism_request=False,
        carrier_features='historical differential predictor has no curve ledger; writer intersects all supplied source-curve constraints'))
    connection.verify(binding)
    return binding
