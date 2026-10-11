"""Continue the sealed partial study with corrected inputs; no meshing."""
import json
from pathlib import Path
import time
import psutil
from bounded_initial_flow_profile import envelope, bind_qualification_profile
from external_flow_profile_diagnostics import atmosphere, mph_to_mps
from production_surface_authority import load_production_wall
from sdf_cfd_mesher_v4.common.authority import file_hash

V4=Path(__file__).resolve().parents[1];REPO=V4.parents[1]
BASE=V4/'results/bounded_external_flow_initial_profile'
OUT=BASE/'corrected_envelope'


def save(name,data):
    (OUT/name).write_text(json.dumps(data,indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf8')


def main():
    start=time.perf_counter()
    for name,h in [('report.md','63e00e45de11b53e7e813e2cee520165357e639a4d8b4ab8b5b2d2b68ce75f83'),
                   ('AUTHORITY.json','31d82aaf8829c2b0ba5d9570c6f37342b6b1ae4eed6d826d12fa1f4cc66793d4')]:
        assert file_hash(BASE/name)==h
    old=json.loads((BASE/'CAPSULE_PROJECTION_DIAGNOSTICS.json').read_text())
    _,_,wall=load_production_wall(V4/'results/production_adaptive_capsule_surface_foundation/fresh')
    assert old['surface_identity']==wall['surface_identity']
    assert old['coordinate_hash']==wall['coordinate_hash'] and old['topology_hash']==wall['topology_hash']
    reference=json.loads((V4/'results/capsule_physical_prism_request/V3_REFERENCE_CONDITION.json').read_text())
    source_paths=[OUT/'CORRECTED_INPUT.md',V4/'results/capsule_physical_prism_request/report.md',
        REPO/'research/sdf_cfd_mesher_v3/physics_driven_prism_75mph/practical_near_wall_retry/analyze_policy.py']
    profile=dict(kind='QualificationProfile',name='V4_25_to_150mph_air_V3_Spalding_yplus10',
        owner='User-authorized V4 qualification study',speed_mph=[25.,150.],length_m=[.5,3.],
        density=1.225,viscosity=1.7894e-5,temperature_K=288.15,yplus=10.,
        wall_treatment='Foundation13 incompressibleFluid; kOmegaSST; U noSlip; nutUSpaldingWallFunction; omegaWallFunction blended=true; kLowReWallFunction',
        sources=[dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p)) for p in source_paths],
        development_alpha=None,coverage_convention='x_end=L_projected; engineering surrogate only')
    binding=bind_qualification_profile(profile,REPO)
    save('PROFILE.json',dict(declaration=profile,binding=binding))
    candidates=[dict(name='Historical_V3_candidate',alpha=.1,
        evidence='derive_physics_schedule.py lines80-83; claimed NASA provenance not independently recovered; not selected'),
        dict(name='Mean_shear_equivalent_diagnostic',alpha=(.0592/.074)**5,
        evidence='Solve 0.0592*(alpha*Re_L)^(-1/5)=0.074*Re_L^(-1/5); mean is not maximum shear; not selected'),
        dict(name='Plate_end_diagnostic',alpha=1.,
        evidence='Local coefficient at the declared end of model development; not a global shear bound; not selected')]
    length=old['results'][0]['extent_m']
    results=[]
    for c in candidates:
        args=dict(speed_mph=profile['speed_mph'],length_m=profile['length_m'],alpha=c['alpha'],
            density=profile['density'],viscosity=profile['viscosity'],yplus=profile['yplus'])
        production=envelope(**args);replay=envelope(**json.loads(json.dumps(args)))
        assert production==replay
        args['length_m']=[length,length]
        capsule=envelope(**args)
        results.append(dict(candidate=c,production=production,capsule=capsule,replay_exact=True))
    save('ENVELOPE.json',dict(results=results,capsule_length_m=length,capsule_flow=[0,0,1],
        capsule_role='Outside primary production scale; actual sampled axial extent; conditional calculations only',
        coverage_adopted_as_engineering_surrogate=True,development_alpha_selected=None))
    flow=[dict(speed_mph=u,speed_mps=mph_to_mps(u),**atmosphere(speed_mps=mph_to_mps(u),
        density=profile['density'],viscosity=profile['viscosity'],temperature=profile['temperature_K'],
        gamma=reference['gamma'],gas_constant=reference['gas_constant'])) for u in profile['speed_mph']]
    save('ATMOSPHERE.json',dict(endpoints=flow,gamma=reference['gamma'],gas_constant=reference['gas_constant']))
    bindings=[]
    for p in [*source_paths,BASE/'AUTHORITY.json',BASE/'report.md',BASE/'CAPSULE_PROJECTION_DIAGNOSTICS.json',
        REPO/'research/sdf_cfd_mesher_v3/physics_driven_prism_75mph/derive_physics_schedule.py',
        V4/'results/geometry_independent_initial_boundary_layer_sizing/sources/mit_turbulent.pdf']:
        bindings.append(dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p)))
    save('INPUT_BINDINGS.json',bindings)
    mem=psutil.Process().memory_info()
    save('RUNTIME.json',dict(audit_seconds=time.perf_counter()-start,peak_rss_bytes=mem.peak_wset,
                           retained_rss_bytes=mem.rss,whole_task_peak_rss='NotMeasured'))
    print('profile',binding['profile_identity'])
    print('atmosphere',flow)
    for r in results:
        print(r['candidate'])
        for domain in ('production','capsule'):
            print(domain)
            for k in ('re_coverage_end','re_shear_start','cf_design','u_tau','full_first_height_m','physical_coverage_estimate_m'):
                z=r[domain]['extrema'][k]
                print(k,[(mode,z[mode]['values'][k],z[mode]['location']) for mode in ('minimum','maximum')])


if __name__=='__main__':main()
