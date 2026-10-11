"""Parameterized pre-CFD estimates. No geometry generation or production defaults.

The finite flat-plate interval is an explicit surrogate-policy input, not an
automatically inferred body length. Endpoint envelopes bound this model only.
"""
import math
from pathlib import Path

from physical_prism_provenance import first_height, verify_sources
from production_prism_sizing import decode_schedule
from sdf_cfd_mesher_v4.common.authority import file_hash, identity

VERSION = 'V4InitialBoundaryLayerEstimate/1'
MODELS = {
    'blasius_laminar': (0.664, 0.5, 5.0),
    'smooth_turbulent_power_law': (0.0592, 0.2, 0.37),
}
ASSUMPTIONS = ('steady', 'incompressible', 'constant_properties', 'smooth_wall',
               'zero_pressure_gradient', 'attached_flat_plate_surrogate')


def positive(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError('PositiveFinitePhysicalInputRequired')
    return float(value)


def policy_identity():
    return identity(dict(version=VERSION, implementation=file_hash(__file__),
                         arithmetic=file_hash(Path(__file__).with_name('physical_prism_provenance.py'))))


def estimate(*, model, speed, density, viscosity, yplus, shear_start_m,
             coverage_end_m, assumptions, height_scope):
    """Evaluate a declared model over [shear_start_m, coverage_end_m].

No universal transition Reynolds number or numerical validity range is added.
The caller must justify the regime and interval before using this estimate.
"""
    if height_scope != 'global':
        raise ValueError('LocalHeightNotQualified')
    if model not in MODELS:
        raise ValueError('ExplicitSupportedModelRequired')
    if set(assumptions) != set(ASSUMPTIONS):
        raise ValueError('ExplicitModelAssumptionsRequired')
    speed, density, viscosity, yplus, x0, x1 = map(positive,
        (speed, density, viscosity, yplus, shear_start_m, coverage_end_m))
    if x1 < x0:
        raise ValueError('InvalidSurrogateInterval')
    nu = positive(viscosity / density)
    re0 = positive(speed * x0 / nu)
    re1 = positive(speed * x1 / nu)
    coefficient, exponent, thickness_coefficient = MODELS[model]
    cf = positive(coefficient / re0**exponent)
    height = first_height(speed, density, viscosity, yplus, cf)
    thickness = positive(thickness_coefficient * x1 / re1**exponent)
    values = dict(nu=nu, re_shear_start=re0, re_coverage_end=re1,
                  cf_design=cf, tau_design_pa=positive(0.5*density*speed*speed*cf),
                  u_tau=height['u_tau'], full_first_height_m=height['h1'],
                  physical_coverage_estimate_m=thickness)
    return dict(values=values, binary64={k:v.hex() for k,v in values.items()},
                claim='FiniteIntervalModelEnvelopeOnly', true_yplus_guarantee=False,
                geometry_generation_authorized=False, policy_identity=policy_identity())


def bind_estimate(declaration, *, geometry_identity, surface_identity, evidence_root):
    """Bind an engineering declaration, never certify its physical applicability."""
    required = {'model','speed','density','viscosity','yplus','shear_start_m',
        'coverage_end_m','assumptions','height_scope','geometry_identity','surface_identity',
        'owner','wall_treatment','flow_direction','scale_basis','regime_basis',
        'intent','sources'}
    if set(declaration) != required:
        raise ValueError('MissingOrUnsupportedPolicyFields')
    for key in ('owner','wall_treatment','scale_basis','regime_basis'):
        if not isinstance(declaration[key], str) or not declaration[key].strip():
            raise ValueError('ExplicitPolicyBasisRequired:' + key)
    if declaration['intent'] != 'InitialEngineeringEstimate':
        raise ValueError('UnsupportedGuaranteeClaim')
    for digest in (geometry_identity, surface_identity):
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError('QualifiedIdentityRequired')
    if declaration['geometry_identity'] != geometry_identity or declaration['surface_identity'] != surface_identity:
        raise ValueError('GeometrySurfaceBindingMismatch')
    direction = declaration['flow_direction']
    if not isinstance(direction, list) or len(direction) != 3 or any(
        type(v) not in (int,float) or not math.isfinite(v) for v in direction
    ) or not math.isfinite(math.hypot(*direction)) or math.hypot(*direction) == 0:
        raise ValueError('DeclaredFlowDirectionRequired')
    sources_id = verify_sources(declaration['sources'], evidence_root)
    arguments = {k:declaration[k] for k in ('model','speed','density','viscosity','yplus',
        'shear_start_m','coverage_end_m','assumptions','height_scope')}
    result = estimate(**arguments)
    return dict(result, evidence_identity=sources_id,
        declaration_identity=identity(dict(declaration=declaration, policy=policy_identity())),
        physical_applicability='OwnerDeclarationNotValidated', schedule_status='Unbound')


def explicit_progression(*, h1, growth, count, physical_total_intent_m):
    """Arithmetic only. Does not choose count/growth or claim coverage suffices."""
    h1, growth, physical_total = map(positive, (h1,growth,physical_total_intent_m))
    if type(count) is not int or count < 1:
        raise ValueError('ExplicitPositiveLayerCountRequired')
    increments = []
    for i in range(count):
        increments.append(positive(h1 * growth**i))
    total = 0.
    for value in increments:
        total = positive(total + value)
    request = dict(height_scope='global',scalar_quantity='normal_height_m',
        requested_layer_count=count, increments_m_hex=[v.hex() for v in increments],
        schedule_total_m_hex=total.hex())
    _, _, key = decode_schedule(request)
    return dict(request, schedule_identity=key, physical_total_intent_m_hex=physical_total.hex(),
                nominal_minus_physical_m=total-physical_total,
                progression_qualified=False, geometry_generation_authorized=False)
