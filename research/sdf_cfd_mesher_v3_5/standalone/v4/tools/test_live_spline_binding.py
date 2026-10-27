"""Bounded integration checks. No seed, adaptation, or mesh qualification run."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import time
import numpy as np
import psutil

sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1]; ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from surface_binding import bind_canonical
from canonical_spline_authority import LiveSplineAuthority, digest, sha
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher


def main():
    start=time.perf_counter()
    out=V4/'results/live_spline_binding';out.mkdir(parents=True,exist_ok=True)
    def save(name,value): (out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',newline='\n')
    script_path=V4/'results/smooth_wing_recovery/recovered_sf2.rhai'
    script=script_path.read_text()
    host=ROOT/'target/debug/canonical_kernel_host.exe'
    historical=ROOT/'research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09/geometry/WING_AUTHORITY_A.json'
    old=json.loads(historical.read_text())
    failures=[]
    def rejects(label,fn):
        try: fn()
        except (GeometryFailure,ValueError) as e:
            failures.append(dict(test=label,error=getattr(e,'record',str(e))))
        else: raise AssertionError('Required rejection absent: '+label)
    with CanonicalKernel([str(host)],timeout=60) as k:
        g=k.load_rhai(script,1.)
        assert g.identity=='079606dc36582ed38e51e5485e1f853e247a2ae40b544414c9e649552ae85311'
        t=time.perf_counter();binding=bind_canonical(g);bind_seconds=time.perf_counter()-t
        a=binding.authority;packet=a.spline_packet
        assert len(a.sheet_names)==4 and len(a.curve_names)==6 and len(a.corner_names)==4
        sections=packet['data']['sections']
        historical_geometry_equal=all(all(s[n]==o[n] for n in ('spline','position','chord','twist')) for s,o in zip(sections,old['sections']))
        assert historical_geometry_equal
        inventory_equal=packet['data']['inventory']==old['inventory']
        # Locations obtained from the existing branch evaluator, not profile math.
        points=[]
        for u in (-1.,-.6,0.,.6,1.):
            q=a.unit.eval(u)
            for y in (0.,-.25,.25):points.append([float(q[0]),y,float(q[1])])
        points += [[1.,0.,0.],[.3,0.,0.],[.3,0.,.1],[1.1,0.,0.],[.3,.3,.02]]
        save('POINTS.json',points)
        t=time.perf_counter()
        run=subprocess.run([str(ROOT/'target/debug/spline_binding_oracle.exe'),str(script_path),str(out/'POINTS.json')],
            capture_output=True,text=True,timeout=60,check=True)
        oracle=json.loads(run.stdout);oracle_seconds=time.perf_counter()-t
        assert oracle['all_exact'] and oracle['spline_identity']==packet['spline_identity']
        direct=np.array([r['scalar'] for r in oracle['queries']])
        t=time.perf_counter();transported=np.array(g.query(points));bound=a.scalar_value(points);query_seconds=time.perf_counter()-t
        assert np.array_equal(direct.view('u8'),transported.view('u8'))
        assert np.array_equal(bound.view('u8'),transported.view('u8'))
        direct_skin=[];branch=[];residual=[];corner=[]
        t=time.perf_counter()
        for p,row in zip(points,oracle['queries']):
            q=a.unit.query(p[0],p[2]);d=row['direct_skin']
            direct_skin.append(dict(point=p,closest_difference=float(np.max(np.abs(np.array(q[:2])-d['point']))),
                normal_difference=float(np.max(np.abs(np.array(q[2:4])-d['outward_normal']))),
                signed_distance_difference=abs(q[6]-d['signed_distance']),
                direct_error_bound=d['distance_error_bound'],bound_error_bound=q[7]))
            for ref in a.sheet_names:
                projected=a.project_sheet(ref,p);direct_project=a.unit.project_sheet(ref,p)
                assert np.array_equal(projected,direct_project)
                normal=a.exact_normal(ref,p)
                assert np.array_equal(normal,a.unit.exact_normal(ref,p))
                dg,dh=a.gradient_hessian(ref,p);eg,eh=a.unit.gradient_hessian(ref,p)
                assert np.array_equal(dg,eg) and np.array_equal(dh,eh)
                residual.append(abs(a.implicit_value(ref,projected)))
                branch.append(dict(ref=ref,point=p,projected=projected.tolist(),normal=normal.tolist(),
                    projected_residual=residual[-1],curvature_bound=a.curvature_upper(ref,p)))
            for ref in a.curve_names:
                assert np.array_equal(a.project_curve(ref,p),a.unit.project_curve(ref,p))
        for ref,p in a.corner_points.items():
            j=a.unit.junctions[a.corner_names[ref]]
            curve_refs=[a.unit.crefs[n] for n in j['connected_feature_ids']]
            matches=[np.array_equal(a.project_curve(c,p),p) for c in curve_refs]
            corner.append(dict(ref=ref,point=p.tolist(),incident_curves=curve_refs,exact=all(matches)))
        assert all(r['exact'] for r in corner)
        branch_seconds=time.perf_counter()-t
        admission=ProductionAdaptiveSdfSurfaceMesher().admission(binding)
        assert not admission['executable']
        assert {r['operation'] for r in admission['issues']}=={'metric','sizing_policy','continuous_geometry_premises'}
        # No mesh stages are invoked by these rejection checks.
        bad=copy.deepcopy(packet);bad['data']['sections'][0]['spline']['spans'][0]['control'][0][0]+=1
        rejects('altered coefficient',lambda:LiveSplineAuthority(g,bad))
        bad=copy.deepcopy(packet);bad['geometry_identity']='altered'
        rejects('altered export identity',lambda:LiveSplineAuthority(g,bad))
        bad=copy.deepcopy(packet);bad['metres_per_model_unit']=.001
        rejects('altered export units',lambda:LiveSplineAuthority(g,bad))
        binding.references['sheet_names'][99]='fabricated'
        rejects('altered binding references',binding.inspect)
        del binding.references['sheet_names'][99]
        old_unit=g.description['metres_per_model_unit'];g.description['metres_per_model_unit']=.001
        rejects('edited native descriptor',binding.inspect)
        g.description['metres_per_model_unit']=old_unit
        rejects('nonfinite query point',lambda:a.project_sheet(a.unit.skin,[float('nan'),0,0]))
        rejects('unknown branch',lambda:a.exact_normal(999,[0,0,0]))
        rejects('no sizing for seed',lambda:a.labelled_seed(a,None,out))
        rejects('accuracy alone is not qualification',lambda:bind_canonical(g,accuracy_limits={'position_m':1.,'normal_rad':1.}))
        with CanonicalKernel([str(host)],timeout=60) as k2:
            g2=k2.load_rhai(script,1.)
            b2=bind_canonical(g2)
            assert g2.identity==g.identity and g2.stored_spline()==packet
            assert b2.authority.reference_identity==a.reference_identity
            assert np.array_equal(np.array(g2.query(points)).view('u8'),direct.view('u8'))
        saved_id=g.identity;g.release()
        rejects('released handle',lambda:k.request(op='stored_spline',handle=g.handle))
    variants=[('unit conversion',script,.001),
              ('taper',script.replace('1.0, 1.0, 0.5','1.0, 0.8, 0.5'),1.),
              ('twist',script.replace('0.5, 0.0, 0.0, 0.0','0.5, 0.0, 0.0, 1.0'),1.),
              ('slanted trailing plane',script.replace('0012','2412'),1.),
              ('different profile in immutable DLL',script.replace('0012','0008'),1.)]
    for name,text,scale in variants:
        with CanonicalKernel([str(host)],timeout=60) as k:
            g=k.load_rhai(text,scale)
            rejects(name,lambda:bind_canonical(g))
    for name,text in [('polygon',script.replace('false','true')),('transform','translate('+script.strip()+', 1.0, 0.0, 0.0)')]:
        with CanonicalKernel([str(host)],timeout=60) as k:
            try: g=k.load_rhai(text,1.)
            except GeometryFailure as e:
                raise AssertionError('test constructor must exist: '+str(e))
            p=g.stored_spline();assert p['data']['status']=='unsupported'
            rejects(name,lambda:LiveSplineAuthority(g,p))
    save('ORACLE.json',oracle)
    save('ADMISSION.json',admission)
    save('BRANCH_CHECKS.json',dict(rows=branch,comparisons=direct_skin,corners=corner))
    result=dict(classification='V4LiveStoredSplineAuthorityBindingVerifiedForUnitChordUniformExtrusion',
        geometry_identity=saved_id,spline_identity=packet['spline_identity'],reference_identity=a.reference_identity,
        kernel_digest=packet['kernel_source_sha256'],retained_export_hash=sha(historical),
        current_export_data_hash=digest(packet['data']),export_bytes=len(json.dumps(packet,separators=(',',':')).encode()),
        historical_geometry_exact=historical_geometry_equal,historical_inventory_exact=inventory_equal,
        metadata_difference='live packet adds identity, binary storage bits and provenance; previews/features/query samples not geometry inputs',
        coefficients_checked=oracle['coefficient_bit_comparisons'],placements_checked=oracle['placement_bit_comparisons'],
        scalar_count=len(points),scalar_bitwise_equal=True,fresh_reconstruction_exact=True,
        topology=dict(sheets=4,curves=6,corners=4,spans_per_section=a.unit.N),
        sheet_comparisons=len(branch),curve_comparisons=len(points)*len(a.curve_names),corner_checks=corner,
        maximum_projection_residual=max(residual),max_direct_skin_differences={key:max(r[key] for r in direct_skin)
            for key in ('closest_difference','normal_difference','signed_distance_difference')},
        direct_skin_error_bound_max=max(r['direct_error_bound'] for r in direct_skin),
        bound_skin_error_bound_max=max(r['bound_error_bound'] for r in direct_skin),
        curvature_bound_per_m=a.premises['curvature_max_per_m'],rejections=failures,
        remaining_inputs=admission['issues'],timings=dict(binding_seconds=bind_seconds,oracle_seconds=oracle_seconds,
            scalar_comparison_seconds=query_seconds,branch_seconds=branch_seconds,total_seconds=time.perf_counter()-start),
        python_peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
        memory_scope='Python peak; child host/oracle and build peak not included',
        host_hash=sha(host),oracle_binary_hash=sha(ROOT/'target/debug/spline_binding_oracle.exe'),
        no_seed_or_meshing=True,no_qualification_or_publication=True)
    save('RESULTS.json',result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
