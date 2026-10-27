"""Bind explicitly adopted engineering physics; never complete a missing schedule.

This is upstream intent evidence, not V4ExternalPrismRequest/1 admission.
"""
import math
from fractions import Fraction
from pathlib import Path
from external_flow_profile_diagnostics import mph_to_mps, projected_extent
from initial_boundary_layer_policy import estimate, ASSUMPTIONS, positive, policy_identity
from physical_prism_provenance import verify_sources
from production_surface_authority import load_production_wall
from production_prism_sizing import contract_identity
from sdf_cfd_mesher_v4.common.authority import identity, file_hash


def progression_freedom(h1, coverage):
    """Algebraic diagnostic under an explicitly CONDITIONAL r>=1 assumption.

For integer N>=2, sum(h1*r^j) increases strictly with r>0. A nondecreasing
solution exists iff N*h1<=T. This does not make r>=1 a production gate.
"""
    h,t=map(positive,(h1,coverage))
    exact=Fraction(t)/Fraction(h)
    largest=math.floor(exact)
    return dict(ratio_binary64=t/h,ratio_exact=str(exact),
        conditional_assumption='r>=1 only for this algebraic diagnostic; not a new hard gate',
        conditional_count_range=[2,largest] if largest>=2 else None,
        conditional_number_of_count_choices=max(0,largest-1),
        selected_count=None,selected_growth=None,increments=None,nominal_total=None)


def bind_physical_intent(declaration, *, surface_directory, evidence_root):
    required={'schema','kind','owner','physical_basis','geometry_identity','surface_identity',
        'speed_mph','density','viscosity','temperature_K','flow_direction','fluid_side',
        'wall_treatment','yplus','alpha','end_ratio','model','assumptions','height_scope',
        'expected_projected_extent_hex','source_documents','limitations'}
    if set(declaration)!=required:raise ValueError('MissingOrUnsupportedIntentFields')
    if declaration['schema']!='V4AdoptedPrismPhysicalIntent/1' or declaration['kind']!='DeclaredQualificationIntent':
        raise ValueError('ExplicitQualificationIntentRequired')
    if declaration['height_scope']!='global':raise ValueError('LocalHeightNotQualified')
    for key in ('owner','physical_basis','wall_treatment','limitations'):
        if not isinstance(declaration[key],str) or not declaration[key].strip():raise ValueError('IntentProvenanceRequired')
    if declaration['fluid_side'] not in ('positive_sdf','negative_sdf'):raise ValueError('FluidSideRequired')
    positive(declaration['temperature_K'])
    sources=verify_sources(declaration['source_documents'],evidence_root)
    x,faces,wall=load_production_wall(surface_directory)
    for key in ('geometry_identity','surface_identity'):
        if declaration[key]!=wall[key]:raise ValueError('QualifiedSourceMismatch')
    projection=projected_extent(x,declaration['flow_direction'])
    length=positive(projection['extent_m'])
    if length.hex()!=declaration['expected_projected_extent_hex']:raise ValueError('ProjectedExtentBindingMismatch')
    alpha=positive(declaration['alpha']);end=positive(declaration['end_ratio'])
    u=mph_to_mps(declaration['speed_mph']);start=alpha*length;stop=end*length
    calc=estimate(model=declaration['model'],speed=u,density=declaration['density'],
        viscosity=declaration['viscosity'],yplus=declaration['yplus'],shear_start_m=start,
        coverage_end_m=stop,assumptions=declaration['assumptions'],height_scope='global')
    values=dict(U=u,L_projected=length,x_start=start,x_end=stop,**calc['values'])
    binary64={k:v.hex() for k,v in values.items()}
    scientific=dict(declaration=declaration,values_binary64=binary64,evidence_identity=sources,
        projection=projection,prior_policy_identity=policy_identity(),prism_contract_identity=contract_identity(),
        projection_implementation_sha256=file_hash(Path(__file__).with_name('external_flow_profile_diagnostics.py')),
        implementation_sha256=file_hash(__file__))
    return dict(schema='V4BoundPhysicalIntentOnly/1',profile_identity=identity(scientific),
        scientific=scientific,values=values,binary64=binary64,wall_vertices=len(x),wall_triangles=len(faces),
        height_scope='global',physical_total_thickness_intent='Engineering surrogate coverage at declared x_end',
        physical_values_bound=True,production_default=False,true_yplus_guarantee=False,
        progression=progression_freedom(values['full_first_height_m'],values['physical_coverage_estimate_m']),
        complete_request=False,generation_authorized=False,prism_vertices_generated=0,prisms_generated=0)
