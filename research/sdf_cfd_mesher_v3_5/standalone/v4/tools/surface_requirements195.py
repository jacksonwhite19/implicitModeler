"""Explicit sampled numerical surface requirements; no acceptance waiver."""
from copy import deepcopy
from dataclasses import dataclass
import math

PROFILE='explicit_numerical_surface_requirements/1'
POLICY='ExplicitNumericalSurfaceRequirements/1'


def validate_accuracy(position, angle, fraction=.9):
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0
           for v in (position,angle,fraction)):
        raise ValueError('FinitePositiveSurfaceAccuracyRequired')
    # Both carrier tan(angle) and shape tan(fraction*angle) must stay on
    # their positive branch, away from pi/2. No arbitrary degree cap.
    if not 0<angle<math.pi/2 or not 0<fraction<=1:
        raise ValueError('SurfaceNormalAngleRequiresZeroToNinetyDegreesExclusive')
    sine=math.sin(angle/2);shape=math.tan(fraction*angle);carrier=math.tan(angle)
    if not all(math.isfinite(v) and v>0 for v in (sine,shape,carrier,carrier/shape)):
        raise ValueError('UnrepresentableSurfaceAccuracyFormula')
    # Position tolerance is not a projector step or an inverse-square size.
    # Keep those independent requirements and existing metric conflict checks.


def configure(case, angle_degrees=None, position_error=None):
    if angle_degrees is None and position_error is None:
        return case,None
    try:
        surface=case['surface'];g=case['general_sdf'];manual=g['manual']
        original_position=surface['position_limit_m'];original_angle=surface['normal_limit_rad']
        fraction=surface['normal_qualification_fraction']
        if g['profile'] not in ('smooth_star_shaped_real_source/1',PROFILE):
            raise ValueError('UnsupportedSurfaceAccuracyProfile')
        if (surface['minimum_size_m']!=manual['minimum_size_m'] or
                surface['maximum_size_m']!=manual['maximum_size_m'] or
                surface['length_scale']!=1 or surface['metric_multiplier']!=1):
            raise ValueError('SurfaceAccuracyManualSIConflict')
    except (KeyError,TypeError) as error:
        raise ValueError('SurfaceAccuracyRequiresGenericManualSIProfile') from error
    angle=original_angle
    if angle_degrees is not None:
        if type(angle_degrees) not in (int,float) or not math.isfinite(angle_degrees) or angle_degrees<=0:
            raise ValueError('FinitePositiveSurfaceNormalDegreesRequired')
        angle=math.radians(angle_degrees)  # Exactly one conversion into the effective contract.
    position=original_position if position_error is None else position_error
    validate_accuracy(position,angle,fraction)
    effective=deepcopy(case);effective['general_sdf']['profile']=PROFILE
    effective['surface']['position_limit_m']=position
    effective['surface']['normal_limit_rad']=angle
    summary=dict(schema='V4ExplicitSurfaceAccuracyRequest/1',source='explicit_cli',
        supplied_flags={k:v for k,v in (('--surface-normal-angle-deg',angle_degrees),
            ('--surface-position-error',position_error)) if v is not None},
        requested_normal_angle_deg=angle_degrees,requested_position_error_m=position_error,
        original=dict(position_limit_m=original_position,normal_limit_rad=original_angle),
        effective=dict(position_limit_m=position,position_limit_m_hex=float(position).hex(),
            normal_limit_rad=angle,normal_limit_rad_hex=float(angle).hex(),
            normal_limit_deg=math.degrees(angle)),
        effective_profile=PROFILE,nominal_surface_size_m=manual['size_m'],
        effective_manual=deepcopy(manual),effective_surface=deepcopy(effective['surface']),
        unchanged_controls=dict(normal_map_step_m=1e-5,curvature_reserve=2.5,
            projection_tolerance_m=manual['projection_tolerance_m'],
            maximum_projection_displacement_m=manual['maximum_projection_displacement_m'],
            normal_estimate_bound_rad=manual['normal_estimate_bound_rad']),
        mapping='Same declared limits bind curvature sizing, shape/carrier allowance, numerical contract and final audit/readback.',
        supported_angle_domain='finite 0 < effective radians < pi/2; finite positive sin/tan formula values',
        position_domain='finite positive metres; projection controls remain separate, existing finite/SPD/minimum-size and final accuracy checks still apply',
        assurance='sampled_numerical_only',
        limitation='Larger allowances can permit coarser surfaces and lower geometric accuracy. No exact nominal edge size, face-count, continuous accuracy, CFD, y-plus or solver guarantee.')
    return effective,summary


def numerical_sizing(surface):
    """Keep the legacy SurfaceSizing validator restrictive for legacy profiles."""
    from historical_capsule_surface import SurfaceSizing
    @dataclass(frozen=True)
    class NumericalSurfaceSizing(SurfaceSizing):
        normal_limit_rad: float = 0.
        def validate(self):
            defaults=SurfaceSizing()
            legacy={k:getattr(self,k) for k in SurfaceSizing.__dataclass_fields__}
            for k in ('position_limit_m','normal_limit_deg','policy_source'):
                legacy[k]=getattr(defaults,k)
            SurfaceSizing(**legacy).validate()
            if self.policy_source!=POLICY or self.normal_limit_deg!=math.degrees(self.normal_limit_rad):
                raise ValueError('ConflictingNumericalSurfaceSizing')
            validate_accuracy(self.position_limit_m,self.normal_limit_rad,self.normal_qualification_fraction)
    value=NumericalSurfaceSizing(**{k:surface[k] for k in ('minimum_size_m','maximum_size_m',
        'position_limit_m','metric_multiplier','shape_response_reserve','normal_qualification_fraction',
        'carrier_maximum_depth','grading')},normal_limit_deg=math.degrees(surface['normal_limit_rad']),
        normal_limit_rad=surface['normal_limit_rad'],policy_source=POLICY)
    value.validate();return value


def verify_numerical_evidence(case,audit,*,geometry_identity=None):
    """Validate the emitted numerical envelope; live issuer custody remains separate."""
    if case.get('general_sdf',{}).get('profile')!=PROFILE:
        return
    from fractions import Fraction
    from pathlib import Path
    import json
    surface=case['surface'];position=surface['position_limit_m'];angle=surface['normal_limit_rad']
    validate_accuracy(position,angle,surface['normal_qualification_fraction'])
    def reject():raise ValueError('DetachedDeclaredSurfaceAccuracyEvidence')
    def number(value):return type(value) in (float,int) and math.isfinite(value)
    def digest(value):return type(value) is str and len(value)==64 and all(c in '0123456789abcdef' for c in value)
    envelope=audit.get('acceptance_contract');geometry=audit.get('numerical_geometry')
    if (type(envelope) is not dict or set(envelope)!={'assurance','continuous_target','limits'}
            or envelope['assurance']!='numerical' or type(geometry) is not dict
            or audit.get('passed') is not True or audit.get('surface_assurance')!='numerical'
            or audit.get('acceptance_custody_completed') is not True
            or geometry.get('passed') is not True or geometry.get('status')!='NumericallyAccepted'):
        reject()
    limits=envelope['limits'];target=envelope['continuous_target']
    if (type(limits) is not dict or set(limits)!={'expected_components','expected_euler','expected_boundary_loops',
            'sheet_names','curve_names','corner_names','position_limit_m','normal_limit_rad',
            'projection_tolerance_m','premise','source'}):reject()
    for values in (limits,geometry):
        for key,expected in (('position_limit_m',position),('normal_limit_rad',angle)):
            value=values.get(key)
            if not number(value) or float(value).hex()!=float(expected).hex():reject()
    if (limits['expected_components'] is not None or limits['expected_euler'] is not None
            or type(limits['expected_boundary_loops']) is not int or limits['expected_boundary_loops']!=0
            or type(limits['sheet_names']) is not dict or len(limits['sheet_names'])!=1
            or any(type(k) is not str or not k.isdecimal() or str(int(k))!=k or int(k)<1
                or v!='canonical.root_zero_field' for k,v in limits['sheet_names'].items())
            or type(limits['curve_names']) is not dict or limits['curve_names']!={}
            or type(limits['corner_names']) is not dict or limits['corner_names']!={}
            or not number(limits['projection_tolerance_m']) or float(limits['projection_tolerance_m']).hex()!=float(2e-12).hex()
            or limits['premise']!='Authenticated source expression; sampled numerical geometry acceptance of the closed oriented mesh. No whole-source completeness or continuous error proof.'
            or type(limits['source']) is not str or not limits['source']):reject()
    if type(target) is not dict or set(target)!={'schema','geometry_identity','kernel_source_sha256','units',
            'unit_scale_exact','expression','semantics','native_role','residual_role','features'}:reject()
    scale=case['geometry']['metres_per_model_unit']
    if not number(scale) or scale<=0:reject()
    manifest=json.loads((Path(__file__).resolve().parents[1]/'runtime/MANIFEST.json').read_text(encoding='utf-8'))
    if (target['schema']!='V4CanonicalRealExpressionTarget/1' or target['units']!='m'
            or target['unit_scale_exact']!=str(Fraction(float(scale)))
            or not digest(target['geometry_identity']) or target['geometry_identity']!=audit.get('geometry_identity')
            or (geometry_identity is not None and target['geometry_identity']!=geometry_identity)
            or target['kernel_source_sha256']!=manifest['native_host_installation']['kernel_source_sha256']
            or type(target['expression']) is not str or not target['expression']
            or target['semantics']!='Exact real operations on authenticated binary32 source coefficients; SI unit scale is the exact real value of the captured binary64 coefficient. No binary32 coordinate quantization in this continuous target.'
            or target['native_role']!='Proposal extraction and separately labelled scalar diagnostics only. Native zero set, normals and topology are not claimed equivalent.'
            or target['residual_role']!='Numerical real-target field residual estimate; no upper bound, native equivalence or Euclidean distance proof.'
            or target['features']!='One root-field reference; sharp feature identities are not supplied or certified.'):
        reject()
    for key,limit in (('maximum_position_estimate_m',position),('maximum_normal_error_estimate_rad',angle)):
        value=geometry.get(key)
        if not number(value) or not 0<=value<=limit:
            raise ValueError('DeclaredSurfaceAccuracyGateFailed:'+key)
