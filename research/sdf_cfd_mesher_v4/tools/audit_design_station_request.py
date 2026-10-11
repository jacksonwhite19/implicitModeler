"""Bind adopted physics and inspect progression evidence; no geometry generation."""
import json,time
from pathlib import Path
import psutil
from adopted_prism_physical_intent import bind_physical_intent
from initial_boundary_layer_policy import ASSUMPTIONS
from production_prism_sizing import contract_identity,read_request
from production_surface_authority import load_production_wall
from sdf_cfd_mesher_v4.common.authority import file_hash,identity

V4=Path(__file__).resolve().parents[1];REPO=V4.parents[1];V3=REPO/'research/sdf_cfd_mesher_v3'
OUT=V4/'results/initial_design_station_progression_request'
PREVIOUS=V4/'results/bounded_external_flow_initial_profile/corrected_envelope'
SURFACE=V4/'results/production_adaptive_capsule_surface_foundation/fresh'


def save(name,data):
    (OUT/name).write_text(json.dumps(data,indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf8')


def main():
    begin=time.perf_counter();files=[]
    def bind(p):
        row=dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p));files.append(row);return row
    for name,h in [('report.md','7b1dd31957e3cc0c1e5dbce578162a37709b8c9362821ba33320eec199d918e4'),
                   ('AUTHORITY.json','1872f5dc0d7d31d595cedbf54c349dec08f919a991227be43bc212de2db11ccd')]:
        assert file_hash(PREVIOUS/name)==h;bind(PREVIOUS/name)
    assert contract_identity()=='7de6b4527da1cd55659c1796927394cbd9af2f581cffbd85d2f891705abbf7dd'
    _,_,wall=load_production_wall(SURFACE)
    sources=[bind(OUT/'ADVISOR_ADOPTION.md'),bind(PREVIOUS/'PROFILE.json'),bind(PREVIOUS/'ENVELOPE.json')]
    declaration=dict(schema='V4AdoptedPrismPhysicalIntent/1',kind='DeclaredQualificationIntent',
        owner='User/advisor: QualifyV4InitialDesignStationAndProgressionRequest',
        physical_basis='Explicitly adopted V4 engineering qualification surrogate; no claim of actual capsule wall shear',
        geometry_identity=wall['geometry_identity'],surface_identity=wall['surface_identity'],
        speed_mph=75.,density=1.225,viscosity=1.7894e-5,temperature_K=288.15,
        flow_direction=[0.,0.,1.],fluid_side='positive_sdf',
        wall_treatment='Foundation13 incompressibleFluid; kOmegaSST; U noSlip; nutUSpaldingWallFunction; omegaWallFunction blended=true; kLowReWallFunction',
        yplus=10.,alpha=.1,end_ratio=1.,model='smooth_turbulent_power_law',assumptions=list(ASSUMPTIONS),
        height_scope='global',expected_projected_extent_hex=float(.15999741537843415).hex(),source_documents=sources,
        limitations='Initial engineering estimate only; no actual maximum-shear or y+ bound; no arbitrary-body flat-plate claim; no production default')
    save('ADOPTED_PROFILE.json',declaration)
    first=bind_physical_intent(declaration,surface_directory=SURFACE,evidence_root=REPO)
    replay=bind_physical_intent(json.loads((OUT/'ADOPTED_PROFILE.json').read_text()),surface_directory=SURFACE,evidence_root=REPO)
    assert first==replay
    save('BOUND_PHYSICAL_INTENT.json',first)
    save('DETERMINISM.json',dict(exact=True,profile_identity=first['profile_identity'],scope='independent authority reload and binary64 calculation; no mesh'))
    try:read_request(OUT/'BOUND_PHYSICAL_INTENT.json',wall)
    except ValueError as exc:rejection=str(exc)
    else:raise AssertionError('PartialIntentEnteredProductionRequest')
    save('CONTRACT_STATUS.json',dict(complete=False,rejection=rejection,
        bound=['geometry_identity','surface_identity','production_test_intent','owner','physical_basis','hash_bound_evidence',
               'fluid_side','full_normal_height_definition','global_scope','physical_total_thickness_intent'],
        unbound=['explicit_positive_increments','layer_count','nominal_schedule_total'],
        nominal_growth=None,generation_authorized=False,contract_identity=contract_identity()))
    rows=[]
    def rule(key,category,text,path,measurement,control):
        source=bind(path);rows.append(dict(key=key,category=category,rule=text,source=source,
            measured_evidence=measurement,can_select_production_N_r=control))
    phys=V3/'physics_driven_prism_75mph'
    rule('practical_count_search','D','N in20..40; choose fewest preferred feasible count',phys/'practical_near_wall_retry/analyze_policy.py',
         'Selected20 for the historical SF2 request; no independent count optimum for capsule',False)
    rule('practical_growth_preference','C','Prefer r<=1.20',phys/'practical_near_wall_retry/analyze_policy.py',
         'Historical engineering preference, not a universally qualified hard gate',False)
    rule('practical_root_bracket','D','r in1.08..1.25,100 bisections for each count',phys/'practical_near_wall_retry/analyze_policy.py',
         'Deterministic numerical search domain; no physical derivation of endpoints recovered',False)
    rule('earlier_search','D','Earlier N30..45,r1.08..1.20,y+0.8 policy',phys/'derive_physics_schedule.py',
         'Different earlier request, not the practical profile adopted here',False)
    rule('historical_total','B','TARGET_TOTAL=.020m',phys/'practical_near_wall_retry/analyze_policy.py',
         'Declared SF2 physical intent; not current capsule coverage',False)
    rule('sf2_loader','B','Load exact20 increments, first.000166527069, stored growth1.163359674',V4/'src/sdf_cfd_mesher_v4/forecast/schedule.py',
         'Reference-specific assertions; retained schedule identity, not a selector',False)
    rule('constant_count_representation','A','Fixed count and per-point geometric ratios preserve first displacement in tested fixtures',
         V3/'constant_count_adaptive_growth_qualification/AUTHORITATIVE_DECISION_REPORT.md',
         'Four240-prism fixture outputs; max first displacement error1.408e-19m; ratio errors1.110e-15 uniform and2.442e-15 variable; determinant minima.042672193 and.018882489. Scoped representation evidence only',False)
    rule('ratio_box','C','Declared1.00<=r_i<=1.20 with N20',V3/'constant_count_adaptive_growth_qualification/run_qualification.py',
         'Domain chosen for compression-only study; not a universal interval',False)
    rule('geometry_dependence','A','Quality imposes geometry-dependent lower as well as upper ratio constraints',
         V3/'constant_count_adaptive_growth_qualification/AUTHORITATIVE_DECISION_REPORT.md',
         'Two layer2 interval-box maxima.001043861331 and.000983600260 below.0011; expanded scalar roots1.217272944 and1.237194130 diagnostic only; other later-layer maxima1.015384588,1.056872474,1.148817308',False)
    rule('geometric_recovery','E','Fit future geometric heights with Brent root; no growth gate',V4/'tools/general_local_correction.py',
         'Unvalidated future proposal; local_schedule explicitly recovery_qualified=False',False)
    rule('compression_recovery','E','Immediate/uniform/smooth/neighbor/receding horizon recovery',V4/'results/compressed_layer_recovery/report.md',
         'Immediate and uniform26 layer5 exact failures; smooth horizon2..5:21,19,17,16; best scalar trial1; no post-seed layer accepted; not growth-selection qualification',False)
    rule('schedule_storage','A','Positive finite increments, strictly increasing sequential targets, exact nominal total, global scope',
         V4/'tools/production_prism_sizing.py','Retained request-decoder tests; consistency constraints do not select N/r',False)
    rule('mixed_schedule','A','Source-complete explicit schedules preserve first target and require increasing finite targets',
         V4/'src/sdf_cfd_mesher_v4/advancing_front/mixed_schedule.py','Explicit mapping/accounting rules; no count/growth selector, no new global production authority',False)
    for p in [V4/'tools/initial_boundary_layer_policy.py',V4/'tests/test_general_local_correction.py',
        V4/'results/compressed_layer_recovery/LAYER_STATUS.json',
        V3/'constant_count_adaptive_growth_qualification/interval_monotonicity.py']:
        bind(p)
    save('PROGRESSION_PROVENANCE.json',dict(categories=dict(A='Independently qualified within stated scope',B='Case-specific physical policy',
        C='Historical engineering convention',D='Search/implementation convenience',E='Diagnostic only',F='Unknown provenance'),
        rows=rows,selection_rule_found=False))
    save('EVIDENCE_MANIFEST.json',dict(files=files,identity=identity(files)))
    mem=psutil.Process().memory_info();save('RUNTIME.json',dict(audit_seconds=time.perf_counter()-begin,
        peak_rss_bytes=mem.peak_wset,retained_rss_bytes=mem.rss,whole_task_peak_rss='NotMeasured'))
    print(json.dumps(dict(values=first['values'],binary64=first['binary64'],profile_identity=first['profile_identity'],
                         progression=first['progression'],contract_rejection=rejection),indent=2))


if __name__=='__main__':main()
