"""Read-only physical provenance reconstruction. Never generate a request or mesh."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import psutil

from production_adaptive_surface import REPO,V3,V4,save,sha
from production_prism_sizing import inspect_first_layer_inputs,contract_identity
from physical_prism_provenance import first_height

OUT=V4/'results/capsule_physical_prism_request'
PHYS=V3/'physics_driven_prism_75mph'
PRACT=PHYS/'practical_near_wall_retry'
SURFACE=V4/'results/production_adaptive_capsule_surface_foundation/fresh'
OLD=V4/'results/fresh_capsule_first_layer_adaptive_surface'


def load(name,path):
    # Both inspected producers only define constants/functions at import.
    sys.dont_write_bytecode=True
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def reduce_rows(rows, policy):
    stack=np.stack(rows);s=stack[0,:,0]
    cf=np.max(np.maximum(stack[:,:,6],0.),axis=0)
    valid=np.isfinite(s)&np.isfinite(cf)&(cf>0)
    s,cf=s[valid],cf[valid];ds=np.abs(np.diff(s))
    w=np.r_[ds[0]*.5,(ds[:-1]+ds[1:])*.5,ds[-1]*.5]
    return float(policy.weighted_quantile(cf,w,.75)),int(valid.sum())


def main():
    begin=time.perf_counter();bindings=[]
    def bind(path,role):
        path=Path(path);row=dict(path=path.relative_to(REPO).as_posix(),sha256=sha(path),bytes=path.stat().st_size,role=role)
        if not any(r['path']==row['path'] for r in bindings):bindings.append(row)
        return row
    for file,digest in [('report.md','475ff0253404961f1e2935bd476954635cb8d589f22233f0f4ec28556e74bedf'),('AUTHORITY.json','41b154dc20b5d44284155834d99054797bcc3ae2b0400465e74f2fcdc05a9314')]:
        assert sha(OLD/file)==digest;bind(OLD/file,'Prior qualified contract')
    assert contract_identity()=='7de6b4527da1cd55659c1796927394cbd9af2f581cffbd85d2f891705abbf7dd'
    decision=inspect_first_layer_inputs(SURFACE)
    assert decision['surface_identity']=='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
    for p in SURFACE.iterdir():
        if p.is_file():bind(p,'Unchanged qualified adaptive surface input')
    save(OUT/'WALL_AND_CONTRACT_VERIFICATION.json',decision)
    xp=PHYS/'run_xfoil_envelope.py';ap=PRACT/'analyze_policy.py'
    bind(xp,'XFOIL reference condition, command producer, convergence filter and dump reduction')
    bind(ap,'Practical physical request and wall-treatment selection')
    x=load('v3_xfoil_provenance',xp);a=load('v3_practical_provenance',ap)
    for p,role in [(PRACT/'freeze_schedule.py','Original physical schedule freeze'),
        (PHYS/'derive_physics_schedule.py','Earlier different physical policy'),
        (V3/'prism_aware_surface_co_design/FINAL_AUTHORITATIVE_DECISION_REPORT.md','Historical selected y+10 and SF2 rounded constants'),
        (V3/'fresh_supported_refinement_family_qualification_2026-09-09/FRESH_BASELINE_CONTRACT.json','Explicit later SF2 numerical request'),
        (V3/'fresh_supported_refinement_family_qualification_2026-09-09/bootstrap_fresh.py','SF2 numeric literal producer'),
        (V4/'tools/production_prism_sizing.py','Unchanged production admission contract')]:bind(p,role)
    bind(V4/'tools/geometry_family_authority.py','Capsule primitive phi=distance-to-axial-segment minus radius')
    bind(V4/'tools/production_adaptive_surface.py','Qualified capsule authority uses that primitive directly; positive SDF is exterior')
    reference=dict(speed_mph=x.SPEED_MPH,mph_to_mps=x.MPH_TO_MPS,U=x.SPEED_MPS,rho=x.DENSITY_KG_M3,
        mu=x.DYNAMIC_VISCOSITY_PA_S,nu=x.KINEMATIC_VISCOSITY_M2_S,temperature_K=x.TEMPERATURE_K,
        gamma=x.GAMMA,gas_constant=x.AIR_GAS_CONSTANT_J_KG_K,speed_of_sound=x.SPEED_OF_SOUND_MPS,
        mach=x.MACH,reynolds=x.REYNOLDS,length_m=x.CHORD_M,length_definition='NACA airfoil chord',
        reynolds_definition='rho*U*chord/mu',geometry='built-in two-dimensional NACA 0012',
        ncrit=9.,transition_cases=['natural_ncrit9','tripped_1pct'],trip_x_over_c=.01,
        retained_incidence_degrees=list(x.ALPHAS_DEG),continuation_step_degrees=.25,
        panel_rows_consumed=x.PANEL_NODES,source=xp.relative_to(REPO).as_posix())
    reference['binary64_hex']={k:v.hex() for k,v in reference.items() if type(v) is float}
    reference['earlier_policy_diagnostics']=dict(
        turbulent_flat_plate_delta99_m=.37*x.CHORD_M/x.REYNOLDS**.2,
        laminar_flat_plate_delta99_m=5*x.CHORD_M/math.sqrt(x.REYNOLDS),
        nasa_style_flatplate_cf_design=.455/math.log(.06*(.1*x.REYNOLDS))**2,
        status='Earlier source formulas evaluated on NACA inputs; not the practical Cf producer and not capsule models')
    save(OUT/'V3_REFERENCE_CONDITION.json',reference)
    # Recompute original flat-plate guides from retained fields, not a new solve.
    plate=[];samples={}
    for target in (1,5,10,30):
        case=a.CASES/f'yplus_{target:02d}';latest=a.latest_written_time(case)
        for name in ('C','wallShearStress','yPlus'):bind(latest/name,'Retained flat-plate comparison field')
        xx=a.values_for_patch(latest/'C','plate','vector')[:,0]
        cf=2*np.linalg.norm(a.values_for_patch(latest/'wallShearStress','plate','vector'),axis=1)/a.U**2
        order=np.argsort(xx);xx,cf=xx[order],cf[order];mask=xx>=.10;samples[target]=(xx[mask],cf[mask])
        row=dict(target=target,time=latest.name,comparison_samples=int(mask.sum()),mean_cf=float(cf[mask].mean()),
            original_primary_solver_log_present=(case/'log.foamRun').is_file(),continuation_log_present=(case/'log.foamRun.continue').is_file())
        for name in ('log.foamRun','log.foamRun.continue'):
            if (case/name).is_file():bind(case/name,'Retained solver log; full original concatenation may be incomplete')
        plate.append(row)
    anchor=samples[5][1]
    for row in plate:
        xx,cf=samples[row['target']];assert np.array_equal(xx,samples[5][0])
        row.update(integrated_error_percent=100*(float(cf.mean()/anchor.mean())-1),local_p95_error_percent=float(np.quantile(100*abs(cf/anchor-1),.95)))
        row['numeric_original_guides_pass']=abs(row['integrated_error_percent'])<=2 and row['local_p95_error_percent']<=5
    c=a.CASES/'yplus_10';latest=a.latest_written_time(c)
    for p in [c/'constant/momentumTransport',c/'constant/physicalProperties',latest/'U',latest/'k',latest/'omega',latest/'nut']:bind(p,'Stored solver/fluid/wall-treatment configuration')
    save(OUT/'FLAT_PLATE_RECOMPUTATION.json',dict(rows=plate,historical_selected_yplus=10,
        wall_treatment=dict(solver='Foundation 13 incompressibleFluid',model='kOmegaSST',U='noSlip',nut='nutUSpaldingWallFunction',omega='omegaWallFunction blended=true',k='kLowReWallFunction'),
        full_historical_solver_acceptance='NotEstablished: primary logs/manifest unavailable; numerical guides and historical selected value corroborated.'))
    arrays=[];finite=[];inventory=[]
    for p in sorted((PHYS/'xfoil_results_A').rglob('*.dump')):
        d=x.parse_dump(p);arrays.append(d);ok=bool(np.isfinite(d).all())
        if ok:finite.append(d)
        b=bind(p,'Raw XFOIL dump; existence/finite values do not establish convergence admission')
        inventory.append(dict(**b,finite=ok,finite_cf_rows=int(np.isfinite(d[:,6]).sum()),rows=len(d),original_convergence_admission='NotEstablished'))
    for p in sorted((PHYS/'xfoil_results_A').rglob('xfoil.in')):bind(p,'Exact retained XFOIL command input')
    saved=np.load(V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/wing/planner/input_state/increments.npy',mmap_mode='r')
    sf2=np.array(saved[0]);del saved
    bind(V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/wing/planner/input_state/increments.npy','Preserved nominal SF2 binary64 schedule, never used as capsule request')
    recon=[]
    for label,rows in [('all_ten_retained_dumps',arrays),('eight_finite_retained_dumps',finite)]:
        cf,n=reduce_rows(rows,a);calc=first_height(a.U,a.RHO,a.MU,10,cf);delta=calc['h1']-float(sf2[0])
        growth=a.solve_growth(calc['h1'],20,a.TARGET_TOTAL)
        recon.append(dict(label=label,classification='ConditionalDiagnostic_NotHistoricalAuthority',cf_design=cf,valid_reduced_samples=n,
            **calc,sf2_h1=float(sf2[0]),signed_difference_m=delta,absolute_difference_m=abs(delta),relative_difference=abs(delta)/float(sf2[0]),
            relative_difference_percent=100*abs(delta)/float(sf2[0]),conditional_growth_20=growth,
            conditional_nominal_total=float(np.cumsum(calc['h1']*growth**np.arange(20))[-1])))
    implied=2*(2*10*a.NU/(float(sf2[0])*a.U))**2
    save(OUT/'H1_RECONSTRUCTION.json',dict(exact_historical_cf='NotEstablished',exact_historical_h1='NotEstablished',
        exact_historical_h1_difference='NotEstablished',original_envelope_present=False,original_convergence_records_present=False,
        coordinate_rows_identical=all(np.array_equal(d[:,:3],arrays[0][:,:3]) for d in arrays),
        raw_inventory=inventory,conditional_reconstructions=recon,cf_implied_by_sf2=implied,
        implied_cf_status='Circular inversion diagnostic, not independent shear evidence',
        finding='Two high-incidence tripped dumps contain nonfinite values. Finite-only reconstruction is consistent with the displayed SF2 precision but finite is not proof of original convergence admission.'))
    save(OUT/'SCHEDULE_PROVENANCE.json',dict(sf2_h1=float(sf2[0]),sf2_growth=1.163359674,sf2_count=len(sf2),sf2_nominal_total=float(np.cumsum(sf2)[-1]),
        increments_hex=[float(v).hex() for v in sf2],physical_total_intent=.020,
        total_origin='TARGET_TOTAL=.020 declared in practical study; no capsule or general boundary-layer derivation supplied',
        count_growth_origin='Fewest count in 20..40; solve geometric sum in r=1.08..1.25, prefer r<=1.20; 100 bisections',
        capsule_transfer=False,earlier_total_policy='Different earlier y+=0.8 workflow used turbulent flat-plate delta99=0.37*L/Re**0.2; not the practical 20 mm origin',
        literal_lineage='Later SF2 bootstrap stores decimal h1/growth literals; finite-only reconstruction is consistent with their displayed precision, but original freeze output is missing.'))
    save(OUT/'EVIDENCE_MANIFEST.json',dict(files=bindings,missing=[
        str(PHYS/'xfoil_results_A/surface_envelope.npz'),str(PHYS/'xfoil_results_A/XFOIL_ENVELOPE.json'),
        str(PRACT/'PRACTICAL_NEAR_WALL_POLICY.json'),str(PRACT/'FROZEN_PRACTICAL_SCHEDULE.json'),str(PRACT/'FLAT_PLATE_CASE_MANIFEST.json'),
        'Both original XFOIL stdout/return-code/RMS admission records'],
        external_reference=dict(url='https://web.mit.edu/drela/Public/web/xfoil/xfoil_doc.txt',role='Primary author documentation: isolated subsonic airfoil method; limited separation and Karman-Tsien correction. Not capsule applicability evidence.')))
    save(OUT/'RUNTIME.json',dict(seconds=time.perf_counter()-begin,peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss,
        prism_vertices_generated=0,prisms_generated=0,layer_1_generated=False,layer_2_generated=False,core_generated=False))
    print(json.dumps(dict(reference=reference,reconstructions=recon,flat_plate=plate),indent=2))


if __name__=='__main__':main()
