"""Read-only V3 contract recovery and fresh production wall admission evidence.

No prism coordinates, normals, prospective targets, or layer-2 forecasts are
constructed. Run this study only after its read-only storage preflight.
"""
from __future__ import annotations

import ast
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import psutil

from production_adaptive_surface import REPO, V4, V3, save, sha
from production_prism_sizing import CONTRACT, contract_identity, inspect_first_layer_inputs

OUT = V4 / 'results/fresh_capsule_first_layer_adaptive_surface'
SURFACE = V4 / 'results/production_adaptive_capsule_surface_foundation/fresh'
FAMILY = 'fresh_supported_refinement_family_qualification_2026-09-09/'
PHYSICS = 'physics_driven_prism_75mph/practical_near_wall_retry/'

# Exact inspected sources. V3 modules with import-time writers are never imported.
TRACE = [
    (PHYSICS+'analyze_policy.py', ['xfoil_distribution','solve_growth','main'],
     'External NACA physics: U, rho, mu, selected y+, weighted p75 Cf; h1=2*yplus*nu/u_tau. Declared total and growth/count domain.'),
    (PHYSICS+'freeze_schedule.py', ['main'], 'Freeze the external policy arrays and its SHA-256; no geometry-derived physical request.'),
    ('physics_driven_prism_75mph/derive_physics_schedule.py', ['main','solve_growth'],
     'Earlier y+=0.8 NACA development request; not the later practical/SF2 policy and not a capsule request.'),
    (FAMILY+'bootstrap_fresh.py', [], 'Fresh SF2 baseline explicitly supplies wing h1, growth and 20 layers.'),
    (FAMILY+'study.py', [], 'SF2 H1=.000166527069; frozen qualification constants.'),
    (FAMILY+'first_row.py', ['main','determinant'],
     'First-row entry: incident source normals, compatible displacement, top=wall+H1*direction, exact local audit and global collision/topology audits. No offset re-projection.'),
    (FAMILY+'smooth_authority.py', ['exact_normal','project_sheet','implicit_value'],
     'Live source normals and wall projection. Projection here supplies the wall; it is not a first-row target solve.'),
    ('finite_wing_trailing_tip_qualification_2026-09-06/corner_authority.py', ['height_compatibility'],
     'Solve N*v=1 at finite incident sheets. Do not normalize the resulting compatible displacement vector.'),
    (FAMILY+'bind_planner.py', ['bind'], 'Preserve accepted X1, bind persistent source keys; later unit-normal trajectories and nominal increments.'),
    (FAMILY+'produce_layer_input.py', [], 'Materialize a frozen displacement contract from baseline request values.'),
    ('sdf_guided_advancing_front_qualification/run_naca_staged_planner.py',
     ['nominal_increments','audit_faces','select_layer2_component','select_late_component','select_staged_schedule'],
     'H1*R**arange(N); bounded geometry-driven later-layer selection. First increment immutable. Legacy case constants are not production defaults.'),
    ('sdf_guided_advancing_front_qualification/accepted_front_state.py', ['__init__','positions','determinant','save','load'],
     'Source ownership, unit source normals, fixed first height, binary64 arrays, immutable X1 and request hash checks.'),
    ('foundation_aware_prism_shell_qualification/run_weight_aware_planner.py', ['complete_weight_audit','select_correction'],
     'Complete side/horizontal weight guards and bounded later compression; not physical height derivation.'),
    ('foundation_skew_increment_qualification/run_skew_aware_planner.py', ['complete_face_audit','select_field'],
     'Skew guards and later schedule selection; no waiver of first-height contract.'),
    ('prism_feasibility_contract/prospective_commit_adapter.py', ['_foundation_determinant','_exact_wedge_jacobian_and_volume'],
     'Wedge Jacobian/volume and Foundation area-tensor predicates.'),
    ('foundation_aware_prism_shell_qualification/foundation_geometry.py', ['layer_geometry','internal_face_skewness','unique_manifold_edges'],
     'Actual prism-neighbour geometry, weights, skewness and manifold edges.'),
    ('adaptive_surface_generic_qualification/prism_host_metric.py', ['tangent_to_height_ratio','intersect_tangent_metric'],
     'Consumes h1 and owned-layer thickness to cap tangent sizes. Does not solve h1 from surface edges.'),
    (FAMILY+'native_binding.py', ['execute'], 'Binds the qualified native displacement implementation.'),
    (FAMILY+'materialize.py', [], 'Publication entry after complete accepted plan; contractDirect first then contractStaged. Not executed here.'),
    ('short_wing_displacement_contract_2026-09-06/native_io.py', ['prepare_first','copy_first_to_physical'],
     'One layer per point/face; explicit displacement vectors; stock kernel; persistent point/cell ancestry.'),
    ('short_wing_displacement_contract_2026-09-06/displacement_record.py', ['save_record','load_record'],
     'Expected X, binary64 delta and source ownership; no hidden vector normalization.'),
    ('short_wing_displacement_contract_2026-09-06/prepare.py', [], 'Displacement, serialization and tolerance contract.'),
    ('held_out_geometry_cohort/run_layer_stage.py', ['heights'],
     'Separate development fixture rule min(.04*S,.20*clearance), 15 layers, r=1.06; not production physics. Frozen policy file unavailable here.'),
]


def source_record(relative, symbols, role):
    path = V3 / relative
    text = path.read_text(encoding='utf-8-sig')
    tree = ast.parse(text)
    locations = {name: [n.lineno for n in ast.walk(tree)
                       if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name]
                 for name in symbols}
    return dict(path=str(path.relative_to(REPO)).replace('\\','/'), sha256=sha(path),
                bytes=path.stat().st_size, symbols=locations, role=role)


def parameters():
    # Category is the origin of this input, not an acceptance threshold.
    rows = [
        ('Physical first normal thickness', 'A', None, 'm', 'External request owner / CFD policy', 'Required; no default'),
        ('Fluid side and wall treatment', 'A', None, 'declaration', 'External domain/CFD authority', 'Required; no inferred sign'),
        ('Requested layer count, growth or explicit increments, total intent', 'A', None, 'count / ratio / m', 'External request owner', 'Required, supplied or explicitly derived outside mesher'),
        ('Velocity, viscosity, density, y+ and shear/model basis when physics-derived', 'A', None, 'm/s, Pa s, kg/m^3, dimensionless', PHYSICS+'analyze_policy.py', 'Capsule inputs absent; direct justified schedule may supply their result'),
        ('SDF sheet normals and orientation', 'B', 'live source', 'unit vector', FAMILY+'smooth_authority.py', 'Geometry does not choose thickness'),
        ('Incident sheet compatible direction', 'B', 'N*v=1', 'dimensionless vector', 'finite_wing_trailing_tip_qualification_2026-09-06/corner_authority.py', 'Finite incident-sheet constraint, vector not renormalized'),
        ('Curvature, clearance, fold/collision feasibility', 'B', 'candidate dependent', '1/m and m', 'sdf_guided_advancing_front_qualification/run_naca_staged_planner.py', 'Constraints on request, not replacement physics'),
        ('Wall triangles, persistent vertices and adjacency', 'C', 'sealed surface', 'topology', FAMILY+'bind_planner.py', 'Determines columns and lateral neighbours'),
        ('Tangential sizes and prism-host metric response', 'C', 'given request + surface tensor', 'm / SPD tensor', 'adaptive_surface_generic_qualification/prism_host_metric.py', 'Diagnostic/construction response; no inversion to h1'),
        ('Unit normal absolute check', 'D', 1e-13, 'dimensionless', 'sdf_guided_advancing_front_qualification/accepted_front_state.py', 'rtol=0; source normals only'),
        ('First-height equality reserve', 'D', 1e-12, 'm', 'sdf_guided_advancing_front_qualification/accepted_front_state.py', 'Numerical safeguard; never height compression'),
        ('Native coordinate roundoff envelope', 'D', 'gamma_(2K+8)*max_axis(abs(X0)+sum(abs(delta)))', 'm', 'short_wing_displacement_contract_2026-09-06/prepare.py', 'Also bounded by existing 1e-12 m reserve'),
        ('Formula comparison tolerances', 'D', {'centre_m':1e-12,'face_area_m2':1e-13,'volume_m3':1e-14,'determinant':1e-11,'weight':1e-11,'skewness':1e-10,'relative':1e-9}, 'as labelled', 'short_wing_displacement_contract_2026-09-06/prepare.py', 'Comparison only; hard gates unchanged'),
        ('SF2 first height', 'E', .000166527069, 'm', FAMILY+'FRESH_BASELINE_CONTRACT.json', 'Not capsule authority'),
        ('SF2 growth', 'E', 1.163359674, 'dimensionless', FAMILY+'FRESH_BASELINE_CONTRACT.json', 'Not capsule authority'),
        ('SF2 count', 'E', 20, 'layers', FAMILY+'FRESH_BASELINE_CONTRACT.json', 'Not capsule authority'),
        ('SF2 nominal total from preserved binary64 increments', 'E', .020000000131704564, 'm', FAMILY+'SF2/A/wing/planner/input_state/increments.npy', 'Control only; do not round to .020'),
        ('Practical NACA velocity / density / dynamic viscosity', 'E', [33.528,1.225,1.7894e-5], 'm/s, kg/m^3, Pa s', PHYSICS+'analyze_policy.py', 'Not capsule physical inputs'),
        ('Practical NACA shear statistic and selected y+', 'E', 'surface-weighted p75 attached Cf; historical y+10', 'dimensionless', PHYSICS+'analyze_policy.py', 'Case shear and wall-treatment evidence; no capsule transfer'),
        ('Practical total / count search / growth search', 'E', {'total_m':.020,'count':[20,40],'growth':[1.08,1.25],'preferred_growth_max':1.20}, 'as labelled', PHYSICS+'analyze_policy.py', 'Policy choices, not universal numerical bounds'),
        ('SF2 surface-prism construction target / response / owned thickness', 'E', [.00121,.7250391657507819,.00019373087622166372], 'ratio, ratio, m', FAMILY+'run_surface.py', 'Surface construction settings, not first-height selector'),
        ('Legacy planner risk / taper / numerical-search settings', 'E', {'condition':.25,'spread_deg':5.,'interior_fraction':.02,'max_rings':20,'independent_compression_floor_fraction':.05}, 'as labelled', 'sdf_guided_advancing_front_qualification/run_naca_staged_planner.py', 'Later candidate controls; not promoted for capsule'),
        ('Held-out fixture development schedule', 'E', 'N=15, r=1.06, H=min(.04*S,.20*clearance)', 'count, ratio, m', 'held_out_geometry_cohort/run_layer_stage.py', 'Different case policy; no capsule production applicability'),
        ('Edge/height and radius/height ratios', 'F', None, 'dimensionless', 'Current task requested diagnostics', 'No numeric acceptance threshold'),
        ('Legacy exposed-cap determinant', 'F', None, 'dimensionless', V4.relative_to(REPO).as_posix()+'/tools/first_layer_metric_state.py', 'Not authoritative first-layer determinant'),
    ]
    return [dict(parameter=n,category=c,value=v,units=u,source=s,scope=d) for n,c,v,u,s,d in rows]


def main():
    started = time.perf_counter()
    trace = [source_record(*row) for row in TRACE]
    extra = [V3/(FAMILY+'FRESH_BASELINE_CONTRACT.json'), V3/(FAMILY+'SF2/A/wing/FIRST_ROW_GATE.json'),
             V3/(FAMILY+'fresh_inputs/layers/FROZEN_CONTRACT.json'),
             V3/(FAMILY+'SF2/A/wing/planner/input_state/increments.npy'),
             V3/(FAMILY+'SF2/A/wing/planner/input_state/state.json'),
             V3/(FAMILY+'SF2/A/wing/FIRST_ROW_BRANCH_CERTIFICATE.json'),
             V3/'prism_aware_surface_co_design/FINAL_AUTHORITATIVE_DECISION_REPORT.md',
             V3/'constant_count_adaptive_growth_qualification/AUTHORITATIVE_DECISION_REPORT.md',
             V3/(FAMILY+'run_surface.py'),
             V3/'pmp_failure_forensics/surface_acceptance_contract/test_bounded_prism_metrics.py',
             V3/'pmp_failure_forensics/surface_acceptance_contract/test_bounded_prism_incidence.py',
             V4/'authorities/TASK1_SOURCE_BINDING.json', V4/'src/sdf_cfd_mesher_v4/forecast/schedule.py',
             V4/'tools/geometry_family_authority.py', V4/'tools/production_scale_authority.py',
             V4/'tools/mixed_front_fixture_provider.py', V4/'tools/first_layer_metric_state.py',
             V4/'results/production_scale_resume_fast/capsule_completion/GEOMETRY_AUTHORITY.json']
    bindings = [dict(path=str(p.relative_to(REPO)).replace('\\','/'),sha256=sha(p),bytes=p.stat().st_size)
                for p in extra if p.is_file()]
    missing = [str(p.relative_to(REPO)).replace('\\','/') for p in extra if not p.is_file()]
    save(OUT/'V3_IMPLEMENTATION_TRACE.json',dict(sources=trace,additional_bindings=bindings,unavailable_optional_references=missing,
        first_row_entry=FAMILY+'first_row.py::main',publication_entry=FAMILY+'materialize.py -> native_io.prepare_first -> contractDirect',
        no_v3_study_modules_imported=True,reason='Several V3 study modules create directories on import; source was inspected with AST/text only.'))
    save(OUT/'PARAMETER_PROVENANCE.json',dict(categories={'A':'User or CFD-physics input','B':'Geometry-derived','C':'Surface-mesh-derived','D':'Generic numerical safeguard','E':'V3 case-specific','F':'Diagnostic only'},parameters=parameters()))
    save(OUT/'PRODUCTION_PRISM_CONTRACT.json',dict(contract=CONTRACT,identity=contract_identity(),implementation_sha256=sha(V4/'tools/production_prism_sizing.py')))
    save(OUT/'GENERIC_VERSUS_CASE_SPECIFIC.json', dict(
        generic=['explicit physical request consumption', 'binary64 schedule persistence',
                 'source-bound unit normals and finite-sheet compatibility', 'source-column ownership',
                 'applicable quality and continuous-geometry validation', 'failure blocks publication'],
        case_specific=['NACA shear and wall-treatment policy', 'SF2 h1/growth/count/total',
                       'SF2 tangent metric construction settings', 'later selector study bounds',
                       'held-out fixture geometry-scaled schedule'],
        intentionally_not_ported=['NACA physical schedule generator as a capsule default',
                                 'wing-specific source-sheet geometry', 'fixture schedule fallback',
                                 'unrequested future-layer planning'],
        operations=dict(first_height='Externally supplied full prism normal thickness',
            growth='Externally declared or external policy output; never inferred from wall edges',
            total_thickness='Declared physical intent; separate explicit binary64 nominal total',
            count='External policy output/request', normal='Oriented exact source normals; N*v=1 at incident sheets',
            column_ownership='Persistent source vertex keys and source triangles',
            top='wall + H1*compatible displacement',
            first_top_offset_sdf_projection='Not used by V3 in the recovered accepted SF2 first-row path',
            collision='Complete column candidate set, conservative SAT, CGAL exact-predicate edge/boundary intersections',
            quality='Exact wedge Jacobian/volume; area tensor with internal top role; actual lateral weight/skew; orientation/height',
            storage='Explicit binary64 increments, targets, expected coordinates and native deltas with hashes',
            stopping='Fail closed at gate failure or exhaustion of bounded later candidates; no partial full-plan publication',
            local_compression='Later-layer bounded candidate mechanism; first height immutable',
            first_height_modification='Not used by V3 in this production path',
            tangential_correction='Not used by V3 in the accepted ordinary SF2 first row; separate affine corner fallback has restricted authority')))
    unavailable=[V3/(PHYSICS+'FROZEN_PRACTICAL_SCHEDULE.json'),V3/(PHYSICS+'PRACTICAL_NEAR_WALL_POLICY.json'),V3/'held_out_geometry_cohort/FROZEN_COHORT_POLICY.json']
    search = dict(decision='C',authoritative_capsule_request_found=False,
        candidates=[
            dict(source=FAMILY+'FRESH_BASELINE_CONTRACT.json',result='SF2 wing request. Wrong geometry; reject transfer.'),
            dict(source=PHYSICS+'analyze_policy.py',result='External NACA physics producer; missing capsule shear/flow/wall-treatment request.'),
            dict(source='held_out_geometry_cohort/run_layer_stage.py',result='Fixture-only geometry-derived development policy, not capsule production input.'),
            dict(source='V4 tools/geometry_family_authority.py::definitions',result='Capsule dimensions use design T=.02; no independent physical normal schedule.'),
            dict(source='V4 tools/production_scale_authority.py::cohort',result='Resolution expansion explicitly retains prior fixture first-layer schedule; no new CFD authority.'),
            dict(source='V4 tools/mixed_front_fixture_provider.py::Provider.__init__',result='Calls legacy load_schedule, which imports the SF2 increments source.'),
            dict(source='V4 production adaptive capsule SURFACE_AUTHORITY.json',result='SurfaceQualified only; no wall-normal request.')],
        missing_historical_outputs=[str(p.relative_to(REPO)).replace('\\','/') for p in unavailable if not p.exists()],
        limitation='The archived practical-policy output is absent. Its producer and historical report are available. SF2 values are independently preserved; exact historical physics re-derivation is not claimed.',
        request_search_scope='V3 production/physics/planner source and preserved SF2 controls; V4 sources/authorities/production-case geometry metadata; new surface authority. No old capsule prism arrays loaded.',
        physics_calculation_run=False,old_capsule_prism_arrays_loaded=False)
    # Preserve an inspectable search over production V4 code and configuration.
    command=['rg','-n','-i','capsule|first_height|h1_m|yplus|y_plus|Reynolds|load_schedule',
             str(V4/'tools'),str(V4/'src'),str(V4/'authorities'),'-g','*.py','-g','*.json','-g','!**/legacy_full_stack_planner/**']
    found=subprocess.run(command,capture_output=True,text=True,timeout=120)
    if found.returncode not in (0,1):raise RuntimeError(found.stderr)
    (OUT/'REQUEST_SEARCH.txt').write_text(found.stdout)
    search['search_command']=command;search['search_result_sha256']=sha(OUT/'REQUEST_SEARCH.txt')
    save(OUT/'CAPSULE_REQUEST_SEARCH.json',search)
    source_rows=json.loads((V4/'authorities/TASK1_SOURCE_BINDING.json').read_text())['files']
    schedule_row=next(r for r in source_rows if r['path'].endswith('/input_state/increments.npy'))
    schedule_path=REPO/schedule_row['path']
    assert sha(schedule_path)==schedule_row['sha256']
    historical=np.load(schedule_path,mmap_mode='r');increments=np.array(historical[0],dtype='<f8')
    assert np.array_equal(historical,np.broadcast_to(increments,historical.shape))
    targets=np.cumsum(increments,dtype=np.float64)
    assert len(increments)==20 and increments[0]==.000166527069 and targets[-1]==.020000000131704564
    save(OUT/'SF2_SCHEDULE_CONTROL.json',dict(source=schedule_row['path'],source_sha256=schedule_row['sha256'],
        input_shape=list(historical.shape),uniform_nominal_rows=True,
        increments_m_hex=[float(v).hex() for v in increments],targets_m_hex=[float(v).hex() for v in targets],
        first_height_m=float(increments[0]),layers=len(increments),nominal_total_m=float(targets[-1]),
        used_as_capsule_request=False,scope='Read-only SF2 schedule provenance control; no coordinates or targets generated.'))
    del historical
    t=time.perf_counter();decision=inspect_first_layer_inputs(SURFACE);load_seconds=time.perf_counter()-t
    assert decision['surface_identity']=='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
    assert decision['geometry_identity']=='a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65'
    assert decision['wall_vertices']==16895 and decision['wall_triangles']==33786
    save(OUT/'GENERATION_AUTHORIZATION.json',decision)
    t=time.perf_counter();replayed=inspect_first_layer_inputs(SURFACE);replay_seconds=time.perf_counter()-t
    assert replayed==decision
    save(OUT/'DETERMINISM.json',dict(scope='Independent production authority load and missing-request decision; no generated layer',
        passed=True,decisions_exactly_equal=True,contract_identity=contract_identity(),
        load_decision_seconds=load_seconds,replay_load_decision_seconds=replay_seconds,
        first_layer_replay='Not measured.'))
    p=json.loads((OUT/'PREFLIGHT.json').read_text())
    for row in p['surface_input_files']:
        assert sha(row['path'])==row['sha256'],row['path']
    a=json.loads((SURFACE/'AUDIT.json').read_text());s=json.loads((SURFACE/'SURFACE_AUTHORITY.json').read_text())
    save(OUT/'WALL_VERIFICATION.json',dict(loader='production_surface_authority.load_production_wall',passed=True,
        surface_identity=s['surface_identity'],geometry_identity=s['geometry_identity'],coordinate_hash=s['coordinate_hash'],
        topology_hash=s['topology_hash'],sizing_field_hash=s['sizing_field_identity'],wall_vertices=a['vertices'],
        wall_triangles=a['triangles'],wall_edges=a['edges'],preserved_edge=a['edge'],preserved_triangle=a['triangle'],
        all_input_file_hashes_unchanged=True,wall_coordinates_exact=True,wall_topology_exact=True,
        inherited_surface_audit_passed=a['passed'],fresh_prism_measurements='Not measured.'))
    gate=json.loads((V3/(FAMILY+'SF2/A/wing/FIRST_ROW_GATE.json')).read_text())
    save(OUT/'SF2_CONTROL.json',dict(source=FAMILY+'SF2/A/wing/FIRST_ROW_GATE.json',sha256=sha(V3/(FAMILY+'SF2/A/wing/FIRST_ROW_GATE.json')),
        ordinary=gate['ordinary'],global_checks=gate['global_checks'],regenerated=False,scope='Preserved first-row prospective control only; not a capsule request.'))
    save(OUT/'RUNTIME_MEMORY.json',dict(authority_load_and_missing_request_decision_seconds=load_seconds,
        authority_load_and_missing_request_replay_seconds=replay_seconds,
        study_seconds=time.perf_counter()-started,peak_rss_bytes=psutil.Process().memory_info().peak_wset,
        retained_rss_bytes=psutil.Process().memory_info().rss,cold_generation='Not measured.',replay_generation='Not measured.',
        normal_construction='Not measured.',target_construction='Not measured.',projection='Not measured.',
        prism_quality='Not measured.',continuous_prism_geometry='Not measured.',native_solvers_invoked=0))
    print(json.dumps(decision,indent=2));print(json.dumps(json.loads((OUT/'RUNTIME_MEMORY.json').read_text()),indent=2))


if __name__=='__main__':main()
