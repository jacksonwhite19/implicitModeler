"""Explicit bounded qualification envelopes. No production defaults or geometry."""
import itertools
from pathlib import Path
from external_flow_profile_diagnostics import mph_to_mps
from initial_boundary_layer_policy import estimate, ASSUMPTIONS, positive, policy_identity
from physical_prism_provenance import verify_sources
from sdf_cfd_mesher_v4.common.authority import identity, file_hash


def interval(values):
    if not isinstance(values,(list,tuple)) or len(values)!=2:
        raise ValueError('ExplicitIntervalRequired')
    low,high=map(positive,values)
    if low>high:raise ValueError('ReversedInterval')
    return low,high


def envelope(*, speed_mph, length_m, alpha, density, viscosity, yplus):
    """Endpoint extrema of the fixed-property turbulent surrogate with x_end=L.

alpha is an explicit diagnostic parameter, not a selected universal convention.
All returned values are model estimates. Corners and their locations are retained.
"""
    us=interval(speed_mph);ls=interval(length_m);a=positive(alpha)
    if a>1:raise ValueError('StartAfterCoverageEnd')
    rho,mu,yp=map(positive,(density,viscosity,yplus))
    corners=[]
    for u,l in itertools.product(us,ls):
        speed=mph_to_mps(u)
        result=estimate(model='smooth_turbulent_power_law',speed=speed,density=rho,
            viscosity=mu,yplus=yp,shear_start_m=a*l,coverage_end_m=l,
            assumptions=ASSUMPTIONS,height_scope='global')
        corners.append(dict(location=dict(speed_mph=u,speed_mps=speed,L_m=l,
            alpha=a,x_start_m=a*l,x_end_m=l),values=result['values'],binary64=result['binary64']))
    extrema={key:{'minimum':min(corners,key=lambda r:r['values'][key]),
                  'maximum':max(corners,key=lambda r:r['values'][key])}
             for key in corners[0]['values']}
    return dict(corners=corners,extrema=extrema,alpha_selected=False,
                geometry_generation_authorized=False,true_yplus_bound=False,
                progression=dict(status='Unresolved',growth=None,count=None,increments=None))


def bind_qualification_profile(profile, evidence_root):
    """A named test profile never supplies missing production-case authority."""
    required={'kind','name','owner','speed_mph','length_m','density','viscosity','temperature_K',
        'yplus','wall_treatment','sources','development_alpha','coverage_convention'}
    if set(profile)!=required:raise ValueError('MissingOrUnknownProfileFields')
    if profile['kind']!='QualificationProfile':raise ValueError('NotQualificationProfile')
    for k in ('name','owner','wall_treatment'):
        if not isinstance(profile[k],str) or not profile[k].strip():raise ValueError('ProfileProvenanceRequired')
    interval(profile['speed_mph']);interval(profile['length_m'])
    for k in ('density','viscosity','temperature_K','yplus'):positive(profile[k])
    if profile['development_alpha'] is not None:
        raise ValueError('DevelopmentConventionRequiresSeparateQualification')
    if profile['coverage_convention']!='x_end=L_projected; engineering surrogate only':
        raise ValueError('CoverageMeaningRequired')
    evidence=verify_sources(profile['sources'],evidence_root)
    digest=identity(dict(profile=profile,evidence=evidence,prior_policy=policy_identity(),
        implementation=file_hash(__file__),diagnostics=file_hash(Path(__file__).with_name('external_flow_profile_diagnostics.py'))))
    return dict(profile_identity=digest,kind='QualificationProfile',production_default=False,
        yplus_profile_bound=True,development_convention='Unresolved',
        complete_prism_request=False,geometry_generation_authorized=False)
