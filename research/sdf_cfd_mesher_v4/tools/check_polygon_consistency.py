"""Task-local error analysis; queries existing geometry, never generates a mesh.

Fractions check the actual retained coordinates. Existing interval primitives
enclose final square roots/divisions. No geometry/projector implementation here.
"""
import argparse
from collections import Counter
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import struct
import sys
import time

import numpy as np
import psutil

sys.dont_write_bytecode = True
V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
from surface_binding import bind_canonical
from canonical_polygon_authority import validate_topology, sha
from replay_surface_dispatch import verify_kernel_sources
from historical_capsule_surface import provider
from continuous_geometry_certifier import certify_face
from shape_operator_support_envelope import _out, _sqrt, _div


from polygon_conversion import enclosure, dot, cross, exact_error_record, derive_conversion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    start = time.perf_counter()
    source, direct = verify_kernel_sources()
    case = next(c for c in direct['cases'] if c['name'] == 'plain_wing')
    retained = V4/'results/canonical_polygon_connection'
    host = ROOT/'target/debug/canonical_kernel_host.exe'
    with CanonicalKernel([str(host)]) as owner:
        t = time.perf_counter()
        g = owner.load_rhai(case['script'], case['metres_per_model_unit'])
        load_seconds = time.perf_counter()-t
        t = time.perf_counter()
        binding = bind_canonical(g)
        binding_seconds = time.perf_counter()-t
        a = binding.authority
        packet = g.stored_topology()
        assert packet == json.loads((retained/'STORED_TOPOLOGY.json').read_text())
        v = validate_topology(packet, g.identity)
        mapped = a.unit.points*a.scale
        conversion, branch_rows = derive_conversion(a, v, case['metres_per_model_unit'])

        # Retained seed: verify source ancestry, not mesh qualification.
        with np.load(retained/'labelled_seed/surface/A/seed/SEED.npz') as seed:
            assert np.array_equal(seed['points'], a.unit.points)
            faces, refs = seed['faces'].copy(), seed['refs'].copy()
        membership = []
        for tri, ref in zip(faces, refs):
            loop = set(a.unit.brep.loops[a.unit.sheet_names[int(ref)]])
            membership.append(all(a.unit.point_names[int(i)] in loop for i in tri))
        assert all(membership)
        # A translated cap face shows the domain premise of certify_face. This
        # deliberately violates that premise; its return is NOT a valid certificate.
        cap_ref = next(r for r,n in a.unit.sheet_names.items() if '/cap/positive' in n)
        cap_tri = mapped[faces[np.flatnonzero(refs == cap_ref)[0]]].copy()
        cap_tri[:,0] += 2*a.scale
        conditional = certify_face(a,cap_ref,cap_tri)
        finite_cap_lower = min(cap_tri[:,0])-max(mapped[:,0])
        assert finite_cap_lower > 0 and conditional[0] == 'Certified'
        domain_counterexample = dict(ref=cap_ref,points_m=cap_tri.tolist(),
            conditional_routine_return=list(conditional), finite_panel_distance_lower_m=float(finite_cap_lower),
            qualified=False, violated_premise='owning finite panel domain; correct plane and normal alone are insufficient')

        old = json.loads((retained/'CONSISTENCY.json').read_text())
        t = time.perf_counter()
        phi = g.query([r['point_m'] for r in old['surface_samples']])
        repeat_seconds = time.perf_counter()-t
        assert all(struct.pack('d',x)==struct.pack('d',r['canonical_field_m']) for x,r in zip(phi,old['surface_samples']))
        residuals,displacements,normal_errors = [],[],[]
        for row in old['surface_samples']:
            p=np.asarray(row['point_m'])
            for ref in row['sheet_refs']:
                residuals.append(abs(a.implicit_value(ref,p)))
                displacements.append(float(np.linalg.norm(a.project_sheet(ref,p)-p)))
                normal_errors.append(abs(float(np.linalg.norm(a.exact_normal(ref,p)))-1))
        curve_displacements=[]
        for ref,indices in a.unit.curve_vertices.items():
            p=mapped[list(indices)].mean(0)
            curve_displacements.append(float(np.linalg.norm(a.project_curve(ref,p)-p)))
        signs = g.query([r['point_m'] for r in old['sign_samples']])
        assert all(struct.pack('d',x)==struct.pack('d',r['canonical_field_m']) for x,r in zip(signs,old['sign_samples']))

        # Near each real cap: use quarter of the actual f32 cap-coordinate ULP,
        # then straddle the first rounding-cell boundary with adjacent f64 inputs.
        cap_tests = []
        for side, si in (('negative',0), ('positive',-1)):
            y = v['sections'][si]['position'][1]
            direction = -np.inf if side == 'negative' else np.inf
            adjacent = float(np.nextafter(np.float32(y),np.float32(direction)))
            ref = next(r for r,n in a.unit.sheet_names.items() if '/cap/'+side in n)
            loop = a.unit.brep.loops[a.unit.sheet_names[ref]]
            centre = np.array([a.unit.brep.points[k] for k in loop]).mean(0)*a.scale
            rows = []
            for fraction in (-.25,0.,.25):
                p = centre.copy();p[1]=(y+fraction*(adjacent-y))*case['metres_per_model_unit']
                rows.append(dict(role='cap_rounding_cell',fraction_of_outward_ulp=fraction,point_m=p.tolist()))
            threshold = ((y+adjacent)/2)*case['metres_per_model_unit']
            q = threshold
            around = [q]
            for d in (-np.inf,np.inf):
                q = threshold
                for _ in range(4):
                    q = float(np.nextafter(q,d));around.append(q)
            for q in sorted(set(around)):
                p=centre.copy();p[1]=q
                rows.append(dict(role='cap_cast_boundary',point_m=p.tolist()))
            values = g.query([r['point_m'] for r in rows])
            for row,value in zip(rows,values):
                p=np.array(row['point_m'])
                row.update(canonical_field_m=value,model_f32=np.asarray(p/case['metres_per_model_unit'],np.float32).astype(float).tolist(),
                    authority_plane_residual_m=a.implicit_value(ref,p),
                    exact_source_cap_signed_distance_m=float((F(float(p[1]))-F(y)/1000)*(1 if side=='positive' else -1)))
            plateau=rows[2]
            assert plateau['canonical_field_m']==0 and plateau['exact_source_cap_signed_distance_m']>0
            boundary=rows[3:];jumps=[]
            for r,s in zip(boundary,boundary[1:]):
                dx=abs(s['point_m'][1]-r['point_m'][1]);df=abs(s['canonical_field_m']-r['canonical_field_m'])
                if df>dx:jumps.append(dict(from_m=r['point_m'],to_m=s['point_m'],delta_y_m=dx,delta_phi_m=df,ratio=df/dx))
            assert jumps
            cap_tests.append(dict(side=side,ref=ref,rows=rows,one_lipschitz_counterexamples=jumps))

        # Exact arithmetic check of the implemented identical-input interpolation.
        # This is arithmetic analysis on stored scalars, not a geometry constructor.
        profile=np.asarray(v['points'][:-1],np.float32)
        u=F(1,2**24); eta=F(1,2**150)
        # RN(1-t), two products, final sum. For |t|<=1 and 0<=RN(1-t)<=1:
        # |r-a| <= |a|*(3u+3u^2+u^3) + |a|*eta*(1+u)^2 + (3+2u)*eta.
        B=lambda x:abs(F(float(x)))*(3*u+3*u*u+u*u*u)+abs(F(float(x)))*eta*(1+u)**2+(3+2*u)*eta
        interpolation=[]; transitions=[]
        for left,right in zip(v['sections'],v['sections'][1:]):
            for fraction in (0.,1/3,.5,2/3,1.):
                y64=(left['position'][1]+fraction*(right['position'][1]-left['position'][1]))*case['metres_per_model_unit']
                y32=np.float32(y64/case['metres_per_model_unit'])
                t32=np.float32(np.float32(y32-np.float32(left['position'][1]))/np.float32(right['position'][1]-left['position'][1]))
                s32=np.float32(np.float32(1)-t32)
                interpolated=np.float32(np.float32(profile*s32)+np.float32(profile*t32))
                err=[abs(F(float(b))-F(float(a0))) for a0,b in zip(profile.flat,interpolated.flat)]
                assert all(e<=B(a0) for e,a0 in zip(err,profile.flat))
                index=int(np.argmax([float(e) for e in err]))
                interpolation.append(dict(y_m=y64,t_f32=float(t32),changed_coordinates=int(np.count_nonzero(interpolated!=profile)),
                    maximum_normalized_difference=float(max(err)),worst_flat_index=index,
                    source_value=float(profile.flat[index]),evaluated_value=float(interpolated.flat[index])))
                transitions.extend([[float(p[0])*a.scale,y64,float(p[1])*a.scale] for p in profile])
        transition_phi=g.query(transitions)
        # Just-adjacent coordinates around all three stored section positions.
        section_neighbours=[]
        for section in v['sections']:
            y=section['position'][1]*case['metres_per_model_unit']
            for yy in (np.nextafter(y,-np.inf),y,np.nextafter(y,np.inf)):
                section_neighbours.extend([[float(p[0])*a.scale,float(yy),float(p[1])*a.scale] for p in profile])
        neighbour_phi=g.query(section_neighbours)
        assert np.isfinite(transition_phi).all() and np.isfinite(neighbour_phi).all()
        interp_bound=max(B(x) for x in profile.flat)
        # Establish that the original mathematical section does not invoke the
        # small-segment shortcut. This is not a full floating-point winding proof.
        edge_squared = []
        for i,p in enumerate(v['points'][:-1]):
            q=v['points'][(i+1)%v['segments']]
            edge_squared.append(sum((F(x)-F(y))**2 for x,y in zip(q,p)))
        assert min(edge_squared) > F(float(np.float32(1e-10)))
        incident=[set() for _ in mapped]
        for ref,indices in a.unit.curve_vertices.items():
            for i in indices:incident[i].add(ref)
        restored=np.nextafter(mapped,np.inf)
        restoration=a.restore_corners(restored,incident)
        assert np.array_equal(restored,mapped) and len(restoration)==len(mapped)
        admission=binding.inspect()
        assert not admission['executable']
        assert {r['operation'] for r in admission['issues']}=={'metric','sizing_policy','caller_accuracy_limits'}
        host_peak=psutil.Process(owner.transport.process.pid).memory_info().peak_wset
        result=dict(classification='V4StoredPolygonConversionBoundedCanonicalCertificatePremiseUnresolved',
            geometry_identity=g.identity,topology_identity=packet['topology_identity'],source=source,
            dependencies=dict(authority_sources=a.provenance,
                checker_sha256=sha(Path(__file__)),
                script_sha256=hashlib.sha256(case['script'].encode()).hexdigest(),
                topology_artifact_sha256=sha(retained/'STORED_TOPOLOGY.json'),
                seed_artifact_sha256=sha(retained/'labelled_seed/surface/A/seed/SEED.npz')),
            host_sha256=sha(host),stored_chord_model=v['sections'][0]['chord'],affine_length_scale_m=a.scale,
            topology_checks=a.checks,conversion=conversion,branch_bounds=branch_rows,
            maximum_branch_normal_angle_bound_rad=max(r['normal_angle_upper_rad'] for r in branch_rows),
            maximum_source_to_returned_plane_bound_m=max(r['exact_plane_distance_upper_m'] for r in branch_rows),
            seed_source_ancestry=dict(vertices=len(mapped),triangles=len(faces),finite_domain_checks=len(membership),failures=0,
                all_seed_coordinates_equal_retained=True,interpretation='convex source-loop membership, not continuous certification or intersection audit'),
            finite_panel_counterexample=domain_counterexample,cap_rounding_counterexamples=cap_tests,
            retained_boundary_recheck=dict(count=len(phi),bit_equal=True,max_abs_field_m=max(map(abs,phi)),
                source_roles=dict(Counter(r['role'] for r in old['surface_samples'])),sign_count=len(signs),sign_values_bit_equal=True),
            identical_section_interpolation=dict(checks=interpolation,conditional_normalized_bound=exact_error_record(interp_bound),
                assumptions='IEEE RN binary32, separate operations, gradual underflow, no overflow; stored t in [0,1]; not a full field-error bound',
                formula='|a|(3u+3u^2+u^3)+|a| eta (1+u)^2+(3+2u) eta; u=2^-24, eta=2^-150',
                conditional_dimensional_coordinate_bound_m=exact_error_record(interp_bound*F(v['sections'][0]['chord'])/1000)),
            ideal_section_minimum_edge_squared_normalized=str(min(edge_squared)),
            ideal_section_minimum_edge_squared_normalized_float=float(min(edge_squared)),
            total_explicit_scalar_queries=len(phi)+len(signs)+len(transitions)+len(section_neighbours)+sum(len(c['rows']) for c in cap_tests),
            transition_checks=dict(interpolation_positions=len(transitions),adjacent_section_positions=len(section_neighbours),
                max_abs_field_interpolation_m=max(map(abs,transition_phi)),max_abs_field_adjacent_m=max(map(abs,neighbour_phi)),
                values_sha256=hashlib.sha256(np.asarray([*transition_phi,*neighbour_phi],'<f8').tobytes()).hexdigest(),nonfinite=0),
            authority_checks=dict(branch_queries=len(residuals),maximum_residual_m=max(residuals),
                maximum_projection_displacement_m=max(displacements),maximum_normal_length_error=max(normal_errors),
                curve_queries=len(curve_displacements),maximum_curve_projection_displacement_m=max(curve_displacements)),
            exact_corner_restorations=len(restoration),admission=dict(executable=admission['executable'],
                issues=[{k:val for k,val in issue.items() if k!='curves'} for issue in admission['issues']]),
            timings_seconds=dict(load=load_seconds,binding=binding_seconds,retained_boundary_query=repeat_seconds,total=time.perf_counter()-start),
            memory=dict(checker_peak_working_set_bytes=psutil.Process().memory_info().peak_wset,host_peak_working_set_bytes=host_peak),
            production_changes=False,new_mesh=False,continuous_evaluator_error_bound=None,
            limits=['No global f32 scalar-error enclosure or exact-distance claim',
                    'No admission acceptance; target/error accounting contract not bound',
                    'Plane residual is not finite-panel distance without domain premise',
                    'Finite precision certifier arithmetic itself is not a formally outward-rounded implementation'])
        (out/'CHECK.json').write_text(json.dumps(result,indent=2,allow_nan=False,default=lambda x:x.tolist())+'\n',encoding='utf8')
        print(json.dumps({k:result[k] for k in ('classification','conversion','maximum_branch_normal_angle_bound_rad',
            'maximum_source_to_returned_plane_bound_m','retained_boundary_recheck','transition_checks','timings_seconds','memory')},indent=2))


if __name__=='__main__':
    main()
