"""Bounded connection checks. Uses existing geometry/Authority/CGAL operations."""
import argparse, copy, hashlib, json, struct, sys, time, shutil, subprocess, os
from collections import Counter
from pathlib import Path
import numpy as np
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from surface_binding import bind_canonical
from canonical_polygon_authority import validate_topology, f32, sha, COMPLETE, CORNER
from surface_execution import validate_references
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from historical_capsule_surface import provider
from replay_surface_dispatch import verify_kernel_sources

def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False,default=lambda x:x.tolist())+'\n')

def expect(code,fn):
    try:fn()
    except GeometryFailure as e:
        assert e.record['code']==code,e.record
        return e.record
    raise AssertionError('Expected '+code)

def seed_connection(binding,geometry,out):
    a=binding.authority
    seed=binding.seed_provider(a,None,out)
    validate_references(binding,seed['vertices'],seed['faces'],seed['edges'],'diagnostic_labelled_seed')
    x=seed['coordinates'];tri=seed['triangles'];face_refs=np.array([f.sheet_ref for f in seed['faces']])
    edges=np.array([e.vertices for e in seed['edges']]);edge_refs=np.array([e.curve_ref for e in seed['edges']])
    projected,faces_at,curves_at,curve_edges=provider.canonicalize_and_project(a,x,tri,face_refs,edges,edge_refs,set(range(len(x))))
    names=binding.corner_restorer(projected,curves_at)
    assert len(names)==len(x) and np.array_equal(projected,x)
    topology=provider.topology_report(projected,tri,face_refs,curve_edges,curves_at,a)
    assert topology['topology_pass']
    centroids=projected[tri].mean(1);phi=geometry.query(centroids.tolist())
    residual=[a.implicit_value(int(ref),p) for ref,p in zip(face_refs,centroids)]
    result=dict(route=seed['seed_route'],controls=seed['controls'],seconds=seed['timing']['seconds'],
        vertices=len(x),faces=len(tri),edges=len({tuple(sorted((int(f[i]),int(f[j])))) for f in tri for i,j in ((0,1),(1,2),(2,0))}),
        persistent_curves=len(edges),corners_restored=len(names),sheets=len(set(face_refs)),
        restored_bits_equal=True,topology=topology,affine_centroid_residual_max_m=max(map(abs,residual)),
        canonical_centroid_field_max_m=max(map(abs,phi)),bytes=seed['output_bytes'],
        coordinates_sha256=hashlib.sha256(x.tobytes()).hexdigest(),triangles_sha256=hashlib.sha256(tri.tobytes()).hexdigest(),
        cgal_sha256=sha(V4.parent/'sdf_cfd_mesher_v3/singular_sector_charts/sector_cdt'),
        scope='Diagnostic labelled affine-source seed; not global implicit extraction or qualified production surface')
    save(out/'SEED_CONNECTION.json',result)
    print(json.dumps(result),flush=True)
    return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',action='store_true');parser.add_argument('--seed-only',action='store_true')
    parser.add_argument('--checks-only',action='store_true');args=parser.parse_args()
    out=args.output.resolve()
    if args.checks_only:
        env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(V4/'src'),str(V4/'tools')]),PYTHONDONTWRITEBYTECODE='1',CARGO_TARGET_DIR=str(ROOT/'target'))
        commands=[[sys.executable,'-B',str(V4/'tools'/name)] for name in ('test_canonical_polygon.py','test_surface_dispatch.py','test_canonical_transport.py')]
        commands.append(['cargo','test','--offline','--manifest-path',str(V4/'canonical_kernel/Cargo.toml'),'--test','contract','--test','stored_topology','--','--nocapture'])
        commands.append([sys.executable,'-B','-m','pytest',str(V4/'tests/test_production_adaptive_surface.py'),'--noconftest','-q','-k',
            'verified_sealed_surface_loads or exact_projection_and_source_branch or sizing_spd_bounds_and_curvature_response or v3_trace_and_dependency_bindings or seed_scale_defect_reproducer or manifold_topology_orientation_and_boundary or failed_audit_cannot_publish','-p','no:cacheprovider'])
        checks=[]
        for command in commands:
            start=time.perf_counter();p=subprocess.run(command,capture_output=True,text=True,env=env,timeout=90,cwd=ROOT)
            checks.append(dict(command=command,exit_code=p.returncode,seconds=time.perf_counter()-start,stdout=p.stdout,stderr=p.stderr))
            save(out/'CHECKS.json',checks);print(json.dumps(checks[-1]),flush=True)
            assert p.returncode==0,p.stdout+p.stderr
        # Refresh admission metadata only; keep original measured samples and timings.
        c=next(c for c in json.loads((V4/'results/generic_canonical_bridge/DIRECT.json').read_text())['cases'] if c['name']=='plain_wing')
        with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
            g=owner.load_rhai(c['script'],c['metres_per_model_unit'])
            save(out/'ADMISSION.json',ProductionAdaptiveSdfSurfaceMesher().admission(g))
        return
    if not args.seed_only:out.mkdir(parents=True,exist_ok=False)
    start=time.perf_counter();source,direct=verify_kernel_sources()
    cases=direct['cases'];case=next(c for c in cases if c['name']=='plain_wing')
    host=ROOT/'target/debug/canonical_kernel_host.exe';failures=[]
    try:
        with CanonicalKernel([str(host)]) as owner:
            geometry=owner.load_rhai(case['script'],case['metres_per_model_unit'])
            assert geometry.identity==case['description']['geometry_identity']
            t=time.perf_counter();binding=bind_canonical(geometry);binding_seconds=time.perf_counter()-t
            a=binding.authority;packet=geometry.stored_topology();validated=validate_topology(packet,geometry.identity)
            if args.seed_only:
                assert packet==json.loads((out/'STORED_TOPOLOGY.json').read_text())
                prior=json.loads((out/'TEST_RESULTS.json').read_text());assert prior['passed']
                seed_connection(binding,geometry,out)
                return
            save(out/'STORED_TOPOLOGY.json',packet)
            assert packet==geometry.stored_topology()
            other=bind_canonical(geometry)
            assert other.references==binding.references and other.topology_evidence==binding.topology_evidence
            scalar=geometry.query(case['points_m'])
            assert all(struct.pack('d',x)==struct.pack('d',y) for x,y in zip(scalar,case['direct_m']))
            # Similarity check against original source ancestry (not geometry generation).
            mapped=a.unit.points*a.scale;coordinate_errors=[]
            section=validated['sections'][0];profile=validated['points'][:-1]
            for key,point in zip(a.unit.point_names,mapped):
                i=int(key.split('/')[-2]);side=key.split('/')[-1]
                stored=validated['sections'][0 if side=='negative' else -1]
                expected=np.array([profile[i][0]*stored['chord'],stored['position'][1],profile[i][1]*stored['chord']])*case['metres_per_model_unit']
                coordinate_errors.append(float(np.max(abs(point-expected))))
            sheet_refs={name:ref for ref,name in a.unit.sheet_names.items()}
            curve_refs={tuple(a.unit.point_names[i] for i in pair):ref for ref,pair in a.unit.curve_vertices.items()}
            rows=[]
            def add(role,key,point,refs):rows.append(dict(role=role,source=key,point_m=np.asarray(point).tolist(),sheet_refs=sorted(refs)))
            for key,point in zip(a.unit.point_names,mapped):
                add('source_corner',key,point,[sheet_refs[s] for s in a.unit.brep.incident[key]])
            for ref,indices in a.unit.curve_vertices.items():
                keys=tuple(a.unit.point_names[i] for i in indices)
                refs=[sheet_refs[u[0]] for u in a.unit.brep.edges[tuple(sorted(keys))]]
                add('source_edge_midpoint',str(ref),mapped[list(indices)].mean(0),refs)
            for ref,name in a.unit.sheet_names.items():
                q=np.array([a.unit.brep.points[k] for k in a.unit.brep.loops[name]])*a.scale
                add('cap_interior' if '/cap/' in name else 'panel_interior',name,q.mean(0),[ref])
            # Internal section is not a cap. Its boundary points still lie on panels.
            for i,point in enumerate(profile):
                refs=[sheet_refs[s] for s,loop in a.unit.brep.loops.items() if '/panel/' in s and 'live.section/node/'+str(i)+'/negative' in loop]
                add('internal_guide_vertex',str(i),np.array([point[0],0.,point[1]])*a.scale,refs)
            for i in range(len(profile)):
                q=(np.array(profile[i])+np.array(profile[(i+1)%len(profile)]))/2
                name=next(s for s in a.unit.sheet_names.values() if s.endswith('/'+str(i)+'~'+str((i+1)%len(profile))))
                add('internal_guide_edge_midpoint',str(i),np.array([q[0],0.,q[1]])*a.scale,[sheet_refs[name]])
            t=time.perf_counter();values=geometry.query([r['point_m'] for r in rows]);query_seconds=time.perf_counter()-t
            for row,value in zip(rows,values):
                p=np.array(row['point_m']);refs=row['sheet_refs']
                row.update(canonical_field_m=value,affine_residuals_m=[a.implicit_value(r,p) for r in refs],
                    projection_displacements_m=[float(np.linalg.norm(a.project_sheet(r,p)-p)) for r in refs],
                    branch_normal_lengths=[float(np.linalg.norm(a.exact_normal(r,p))) for r in refs])
            # Points derived from source-loop centroids and the solid centroid, no threshold.
            centre=mapped.mean(0);sign_rows=[]
            for ref,name in a.unit.sheet_names.items():
                q=np.array([a.unit.brep.points[k] for k in a.unit.brep.loops[name]]).mean(0)*a.scale
                for factor in (.5,1.5):
                    p=centre+factor*(q-centre)
                    sign_rows.append(dict(source_sheet=ref,centroid_ray_factor=factor,point_m=p.tolist(),
                        affine_support_envelope_m=max(a.implicit_value(r,p) for r in a.sheet_names)))
            for row,value in zip(sign_rows,geometry.query([r['point_m'] for r in sign_rows])):
                row['canonical_field_m']=value
                row['sign_equal']=bool(np.sign(value)==np.sign(row['affine_support_envelope_m']))
            assert all(r['sign_equal'] for r in sign_rows)
            # Exercise unchanged finite-curve projection and complete corner stars.
            incident=[set() for _ in mapped]
            for ref,indices in a.unit.curve_vertices.items():
                for i in indices:incident[i].add(ref)
            perturbed=np.nextafter(mapped,np.inf)
            restored=a.restore_corners(perturbed,incident)
            assert len(restored)==len(mapped) and np.array_equal(perturbed,mapped)
            try:a.restore_corners(mapped[:1].copy(),[set(sorted(incident[0])[:2])])
            except ValueError as error:failures.append(dict(code='IncompleteCornerStar',detail=str(error)))
            else:raise AssertionError('Incomplete corner accepted')
            curve_displacements=[]
            for ref,indices in a.unit.curve_vertices.items():
                p=mapped[list(indices)].mean(0)
                curve_displacements.append(float(np.linalg.norm(a.project_curve(ref,p)-p)))
            # Malformed/unsupported inputs: copies only; live geometry unchanged.
            for name,code,edit in [
                ('closure','InvalidSectionClosure',lambda p:[s['point_bits'].__setitem__(-1,s['point_bits'][1]) for s in p['data']['sections']]),
                ('duplicate','DegenerateSection',lambda p:[s['point_bits'].__setitem__(1,s['point_bits'][0]) for s in p['data']['sections']]),
                ('taper','UniformExtrusionPremiseFailed',lambda p:p['data']['sections'][1].__setitem__('chord_bits',1065353216)),
                ('twist','UniformExtrusionPremiseFailed',lambda p:p['data']['sections'][1].__setitem__('twist_bits',1065353216)),
                ('sweep','UniformExtrusionPremiseFailed',lambda p:p['data']['sections'][1]['position_bits'].__setitem__(0,1065353216)),
                ('nonfinite','NonfiniteStoredTopology',lambda p:p['data']['sections'][1].__setitem__('twist_bits',2139095040)),
                ('identity','GeometryIdentityMismatch',lambda p:p.__setitem__('geometry_identity','wrong')),
                ('dimension','InvalidStoredTopology',lambda p:p['data']['sections'][0].__setitem__('position_bits',[0,0]))]:
                bad=copy.deepcopy(packet);edit(bad)
                failure=expect(code,lambda:validate_topology(bad,geometry.identity));failure['test']=name;failures.append(failure)
            saved=binding.references['sheet_names'];binding.references['sheet_names']={}
            failures.append(expect('ReferenceBindingMismatch',binding.inspect));binding.references['sheet_names']=saved
            saved=binding.corner_restorer;binding.corner_restorer=None
            failures.append(expect('OperationBindingMismatch',binding.inspect));binding.corner_restorer=saved
            saved=binding.topology_evidence;binding.topology_evidence=None
            failures.append(expect('OperationBindingMismatch',binding.inspect));binding.topology_evidence=saved
            admission=ProductionAdaptiveSdfSurfaceMesher().admission(binding)
            assert not admission['executable']
            failures.append(expect('AuthorityOperationUnbound',lambda:ProductionAdaptiveSdfSurfaceMesher().generate(binding,out/'FORBIDDEN_SURFACE')))
            assert not (out/'FORBIDDEN_SURFACE').exists()
            save(out/'ADMISSION.json',admission)
            stats=dict(source_checks=a.checks,length_scale_m=a.scale,coordinate_mapping_max_difference_m=max(coordinate_errors),
                coordinate_mapping_bit_equal_count=sum(x==0 for x in coordinate_errors),
                sample_roles=dict(Counter(r['role'] for r in rows)),query_count=len(rows),
                canonical_field_max_abs_m=max(abs(r['canonical_field_m']) for r in rows),
                affine_residual_max_abs_m=max(abs(v) for r in rows for v in r['affine_residuals_m']),
                projection_max_displacement_m=max(v for r in rows for v in r['projection_displacements_m']),
                branch_normal_max_unit_error=max(abs(v-1) for r in rows for v in r['branch_normal_lengths']),
                normal_branch_count=sum(len(r['sheet_refs']) for r in rows),
                surface_field_sign_counts=dict(Counter(str(int(np.sign(r['canonical_field_m']))) for r in rows)),
                off_surface_sign_samples=len(sign_rows),off_surface_sign_failures=sum(not r['sign_equal'] for r in sign_rows),
                curve_projection_count=len(curve_displacements),curve_projection_max_displacement_m=max(curve_displacements),
                exact_corner_restorations=len(restored),restored_coordinates_bit_equal=True,
                scalar_reference_samples=len(scalar),scalar_reference_bit_equal=True,binding_seconds=binding_seconds,
                canonical_query_seconds=query_seconds,extents_m=np.ptp(mapped,axis=0),bounds_m=[mapped.min(0),mapped.max(0)])
            save(out/'CONSISTENCY.json',dict(summary=stats,surface_samples=rows,sign_samples=sign_rows,
                scope='Affine branches and finite samples; no continuous canonical f32 evaluation-error bound. Support envelope is not Euclidean distance.'))
            save(out/'SUMMARY.json',stats)
            print(json.dumps(stats,default=lambda x:x.tolist()),flush=True)
            seed_result=None
            if args.seed:
                seed_result=seed_connection(binding,geometry,out)
            host_peak=psutil.Process(owner.transport.process.pid).memory_info().peak_wset
            geometry.release()
            failures.append(expect('InvalidOrReleasedHandle',geometry.stored_topology))
        with CanonicalKernel([str(host)]) as owner:
            fresh=owner.load_rhai(case['script'],case['metres_per_model_unit'])
            assert fresh.stored_topology()==packet
            assert bind_canonical(fresh).references==binding.references
        unsupported=[]
        for name in ('sphere','transformed_sphere','scaled_sphere'):
            c=next(c for c in cases if c['name']==name)
            with CanonicalKernel([str(host)]) as owner:
                g=owner.load_rhai(c['script'],c['metres_per_model_unit'])
                d=g.stored_topology();b=bind_canonical(g);assert d['data']['status']=='unsupported'
                assert g.query(c['points_m'])==c['direct_m']
                unsupported.append(dict(name=name,topology=d['data'],scalar_samples=len(c['points_m']),binding_executable=b.inspect()['executable']))
        save(out/'TEST_RESULTS.json',dict(passed=True,failures_expected=failures,unsupported=unsupported,
            source=source,host_sha256=sha(host),source_dependencies={str(p):sha(p) for p in (COMPLETE,CORNER)},
            repeated_and_fresh_host_topology_equal=True,fresh_host_references_equal=True,
            process_peak_working_set_bytes=psutil.Process().memory_info().peak_wset,host_peak_working_set_bytes=host_peak,
            seconds=time.perf_counter()-start,seed_ran=args.seed,production_surface_generated=False,
            final_free_bytes=shutil.disk_usage(out).free))
    except Exception as error:
        save(out/'FAILURE.json',dict(type=type(error).__name__,detail=str(error),seconds=time.perf_counter()-start))
        raise

if __name__=='__main__':main()
