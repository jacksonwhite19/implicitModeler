"""Record read-only external authorities. Never invoke their executables."""
from prepare_source_fork import ROOT, REPO, V3, H, G, P, sha, put
import json

def reference(path,role,expected=None,**extra):
    assert path.exists(),path
    measured=sha(path) if path.is_file() else None
    if expected and measured:assert measured==expected,(path,measured,expected)
    return dict(path=path.relative_to(REPO).as_posix(),sha256=expected or measured,role=role,copied=False,size_bytes=path.stat().st_size if path.is_file() else None,verification='file SHA-256 checked' if measured else 'directory exists; member identity referenced, not rehashed',**extra)

refs=[]
refs.append(reference(H/'SF2/A/wing/WING_AUTHORITY_A.json','common geometry','f453b7430703e9ae2c4938cd10f600c565d3fb7a0ce6f8e22e1161561326d1a3'))
refs.append(reference(H/'SF2/A/wing/surface/A/QUALIFICATION_ARRAYS.npz','SF2 surface','dc390bedc2198cb315e15961f848cd67b47641c30ae4ec0b96944996d1f8196f'))
identity=H/'SF2/A/CANONICAL_MESH_IDENTITY.json'
j=json.loads(identity.read_text())
mesh=H/'SF2/A/04_complete/openfoam_case/constant/polyMesh'
r=reference(mesh,'SF2 accepted polyMesh',j['canonical_sha256'],hash_kind=j['encoding'],members=j['canonical_members'],identity_record_path=identity.relative_to(REPO).as_posix(),identity_record_sha256=sha(identity))
r['size_bytes']=sum((mesh/name).stat().st_size for name in j['canonical_members'])
refs.append(r)
refs.append(reference(G/'complete_bilinear_certificate_integration_20260914/FINAL_RESEARCH_AUTHORITY.json','certificate integration authority','bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f'))
binding=json.loads((H/'scoped_operator_production_numerical_counterfactual_20260911/boundary_only_remaining_family_20260912/results/SF2_A/CASE_BINDING.json').read_text())
for path,expected in binding['numerical_source_manifest'].items():
    if path.endswith('/bin/potentialFoamObserved'):
        from pathlib import Path
        refs.append(reference(Path('C:/'+path[7:]),'numerical executable',expected))
refs.append(reference(P/'two_way_planner.py','user-bound V3 development planner','c3813b1539af079e2aeb49c08f6cb034945c0dee388849b5a3a0a990d38e497b',also_source_forked=True))
refs.append(reference(G/'revised_conditioning_planner_full_shell_development_20260916/conditioning_planner.py','newer unpromoted V3 development source; informational reference only'))
refs.append(reference(H/'FRESH_BASELINE_CONTRACT.json','physical constants authority'))
refs.append(reference(H/'geometry/bound_input.rhai','existing Rhai geometry input'))
refs.append(reference(G/'FIELD_AUTHORITY.json','legacy synthetic field diagnostic; never a V4 gate'))
put(ROOT/'authorities/DATA_AUTHORITIES.json',dict(schema='V4.DataAuthorities/1',path_base='repository root',references=refs,physical_contract=dict(requested_layers=20,first_layer_m=0.000166527069,nominal_growth_user_text='163359674',nominal_growth_v3_authority=1.163359674,nominal_growth_resolution='Preserve user text; retain verified V3 physical contract for comparison. No growth performed.',exact_wall_geometry=True,frozen_surface_topology=True,deterministic=True,local_layer_termination=False),accepted_sf2_cells=dict(total=3781383,wing_prisms=2126040,support_prisms=52272,core_tetrahedra=1603071),hard_requirements=dict(positive_volume=True,positive_jacobian=True,determinant_min=0.0011,interpolation_weight_min=0.055,skewness_max=3.63636,symmetric_volume_ratio_min=0.01,aspect_ratio_max=1000,no_prism_side_intersections=True,no_unresolved_continuous_geometry=True,valid_topology_and_ownership=True),quality_rounding_note='Frozen V3 code uses 4/1.1 for skew (3.636363636...); requested retained limit is 3.63636. Both recorded; source unchanged. Future admission must respect the stated hard limit.',conditioning_policy=dict(sigma_floor=None,gain_ceiling=None,weighted_operator='A=sqrt(W) N',exact_weighted_gain='1/sigma_min',historical_targets='diagnostic only')))
print('Verified',len(refs),'authority paths; referenced bytes',sum(r['size_bytes'] or 0 for r in refs))
