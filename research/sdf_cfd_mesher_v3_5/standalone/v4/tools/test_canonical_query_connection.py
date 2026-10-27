"""Direct kernel versus live bridge; no extraction, conditioning or geometry math."""
import concurrent.futures, copy, hashlib, json, math, os, struct, subprocess, sys, time
from pathlib import Path
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1]; ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel, GeometryFailure
from surface_binding import bind_canonical
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from replay_surface_dispatch import verify_kernel_sources

OUT=V4/'results/canonical_query_connection'
HOST=[str(ROOT/'target/debug/canonical_kernel_host.exe')]
def digest(data): return hashlib.sha256(data).hexdigest()
def save(name,data): (OUT/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf8')
def bits(x): return struct.pack('>d',x)
def same(a,b):
    if isinstance(a,dict):
        assert a.keys()==b.keys(),(a.keys(),b.keys())
        for k in a:same(a[k],b[k])
    elif isinstance(a,list):
        assert len(a)==len(b)
        for x,y in zip(a,b):same(x,y)
    elif isinstance(a,float): assert bits(a)==bits(b),(a,b)
    else: assert a==b,(a,b)

def si_feature(raw,scale):
    # Unit conversion oracle, not a geometry operation.
    r=copy.deepcopy(raw)
    for k in ('query_position','closest_point'):
        if r[k] is not None:r[k]=[v*scale for v in r[k]]
    if r['distance_to_feature'] is not None:r['distance_to_feature']*=scale
    for k in ('query_tolerance','feature_to_sdf_residual'):
        if r['diagnostics'][k] is not None:r['diagnostics'][k]*=scale
    for m in r['all_matches']:
        m['closest_point']=[v*scale for v in m['closest_point']];m['distance']*=scale
    return r

def expect(code,fn):
    try:fn()
    except GeometryFailure as e:
        assert e.record['code']==code,e.record
        return e.record
    raise AssertionError('missing '+code)

def main():
    OUT.mkdir(exist_ok=True);start=time.perf_counter()
    source,_=verify_kernel_sources()
    inputs=V4/'results/generic_canonical_bridge/DIRECT.json'
    retained=V4/'results/automatic_field_ingestion/QUERY_RESULTS.json'
    cases=json.loads(inputs.read_text())['cases'];results=[];failures=[]
    for name in ('plain_wing','sphere','transformed_sphere','scaled_sphere'):
        case=next(c for c in cases if c['name']==name)
        command=[str(ROOT/'target/debug/query_oracle.exe'),str(inputs),name,str(retained)]
        t=time.perf_counter();p=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        peak=0
        while True:
            try:peak=max(peak,psutil.Process(p.pid).memory_info().peak_wset)
            except psutil.NoSuchProcess:pass
            try:stdout,stderr=p.communicate(timeout=.02);break
            except subprocess.TimeoutExpired:
                if time.perf_counter()-t>60:p.kill();raise RuntimeError('OracleTimeout')
        assert p.returncode==0,stderr
        oracle=json.loads(stdout);oracle_wall=time.perf_counter()-t
        s=case['metres_per_model_unit'];points=oracle['points_m']
        with CanonicalKernel(HOST) as owner:
            g=owner.load_rhai(case['script'],s);same(g.description,oracle['description'])
            assert g.identity==case['description']['geometry_identity']
            binding=bind_canonical(g);same(binding.query_discovery,g.discover())
            same(g.feature_inventory()['data']['model'],oracle['inventory'])
            inventory_si=copy.deepcopy(oracle['inventory'])
            if inventory_si:
                for feature in inventory_si['features']:
                    if feature['finite_trailing_edge_thickness'] is not None:
                        feature['finite_trailing_edge_thickness']*=s
            same(g.feature_inventory()['data']['si'],inventory_si)
            same(binding.authority.scalar_value(points).tolist(),oracle['scalar']);same(g.query(points[:64]),case['direct_m'])
            t=time.perf_counter();projection=binding.authority.numerical_project(points);bridge_project_seconds=time.perf_counter()-t
            normal=binding.authority.numerical_normal(points)
            for actual,expected in zip(projection['data']['results'],oracle['projected']):
                same({k:actual[k] for k in expected},expected)
            same([r['normal_direction'] for r in normal['data']['results']],oracle['normal'])
            eval_requests=[{k:r[k] for k in ('feature_id','parameter')} for r in oracle['evaluations']]
            evaluated=binding.authority.feature_evaluate(eval_requests)
            for a,b in zip(evaluated['data']['results'],oracle['evaluations']):
                same(a['model'],b['model']);same(a['si'],si_feature(b['model'],s))
            queried=binding.authority.feature_query(oracle['feature_points_m'])
            same(queried['data']['model']['results'],oracle['batch']['results'])
            same(queried['data']['results_si'],[si_feature(r,s) for r in oracle['batch']['results']])
            same(g.feature_query(oracle['feature_points_m'])['data']['model']['results'],oracle['batch']['results'])
            # Explicit SI options preserve the existing 6 model-unit diagnostic band.
            opts=dict(closest_feature=True,all_features_within_tolerance=True,surface_sector=True,local_parameters=True,tolerance_m=6*s)
            same(g.feature_query(oracle['feature_points_m'],opts)['data']['model']['results'],oracle['batch']['results'])
            unknown=g.feature_evaluate([dict(feature_id='nonexistent',parameter=.5)])
            assert unknown['data']['results'][0]['status']=='unsupported_or_unknown_feature'
            assert not hasattr(binding.authority,'project_sheet') and not hasattr(binding.authority,'exact_normal')
            admission=ProductionAdaptiveSdfSurfaceMesher().admission(binding)
            assert not admission['executable'] and len(admission['issues'])==16
            err=expect('AuthorityOperationUnbound',lambda:ProductionAdaptiveSdfSurfaceMesher().generate(binding,OUT/'must_not_exist'))
            assert not (OUT/'must_not_exist').exists()
            if name=='plain_wing':save('WING_ADMISSION.json',admission)
            # Live descriptor/discovery tampering and protocol failures cannot promote roles.
            old=copy.deepcopy(g.description);g.description['capabilities']['projection']='supported'
            failures.append(expect('GeometryDescriptionMismatch',g.discover));g.description=old
            binding.query_discovery['data']['operations']['numerical_project']='qualified'
            failures.append(expect('OperationBindingMismatch',binding.inspect))
            failures.append(expect('InvalidRequest',lambda:owner.request(op='numerical_project',handle=g.handle,points_m=[[0,0]])))
            failures.append(expect('InvalidRequest',lambda:g.numerical_normal([[float('nan'),0,0]])))
            failures.append(expect('CoordinateOutOfKernelRange',lambda:g.numerical_project([[1e300,0,0]])))
            failures.append(expect('InvalidFeatureTolerance',lambda:g.feature_query(points[:1],dict(opts,tolerance_m=-1))))
            failures.append(expect('InvalidRequest',lambda:owner.request(op='feature_evaluate',handle=g.handle,requests=[dict(feature_id='x',parameter=0,extra=True)])))
            if name=='sphere':
                failures.append(expect('BatchTooLarge',lambda:g.numerical_normal([[0,0,0]]*65537)))
                sample=g.feature_query([[0,0,0]])['data']
                row_bytes=sum(len(json.dumps(sample[k][0] if k=='results_si' else sample[k]['results'][0],separators=(',',':'))) for k in ('model','results_si'))
                oversized_count=(8*1024*1024)//row_bytes+1
                assert oversized_count<=65536
                failures.append(expect('ResponseTooLarge',lambda:g.feature_query([[0,0,0]]*oversized_count)))
                assert g.query([[0,0,0]])==[-.04]  # host recovers after oversized response
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                threaded=list(pool.map(lambda _:g.numerical_normal(points)['data']['results'],range(4)))
            for value in threaded:same(value,normal['data']['results'])
            t=time.perf_counter()
            for _ in range(20):g.numerical_project(points)
            timed_bridge=time.perf_counter()-t
            process_peak=psutil.Process(owner.transport.process.pid).memory_info().peak_wset
            status=[r['normal_status'] for r in normal['data']['results']]
            projection_status=[r['normal_status'] for r in projection['data']['results']]
            rows=queried['data']['model']['results']
            result=dict(name=name,identity=g.identity,points=len(points),scalar_comparisons=len(points),
                projection_comparisons=len(points),normal_comparisons=len(points),feature_evaluations=len(eval_requests),
                feature_queries=len(rows),all_compared_values_bit_equal=True,maximum_transport_difference=0,
                features=len((oracle['inventory'] or {}).get('features',[])),sectors=len((oracle['inventory'] or {}).get('sectors',[])),
                junctions=len((oracle['inventory'] or {}).get('junctions',[])),
                ambiguities=sum(bool(r['diagnostics']['ambiguity']) for r in rows),
                retained_feature_location_ambiguities=sum(bool(r['diagnostics']['ambiguity']) for r in rows[:oracle['feature_point_count']]),
                unsupported_feature_queries=sum(r['support_status']=='unsupported' for r in rows),
                input_zero_directions=status.count('zero_direction'),projected_zero_directions=projection_status.count('zero_direction'),
                maximum_projection_field_residual_m=max(abs(r['signed_field_at_return_m']) for r in projection['data']['results']),
                retained64_projection_field_residual_m=max(abs(r['signed_field_at_return_m']) for r in projection['data']['results'][:64]),
                max_feature_residual_m=max((abs(r['si']['diagnostics']['feature_to_sdf_residual']) for r in evaluated['data']['results']),default=None),
                direct_oracle_sha256=digest(stdout.encode()),oracle_command=command,oracle_stderr=stderr,
                oracle_wall_seconds=oracle_wall,oracle_peak_working_set_lower_bound=peak,
                host_peak_working_set_bytes=process_peak,load_seconds=g.load_seconds,
                direct_project_seconds=oracle['project_seconds'],bridge_project_seconds=bridge_project_seconds,
                repeated_projection_count=20*len(points),direct_repeated_projection_seconds=oracle['repeated_projection_seconds'],
                bridge_repeated_projection_seconds=timed_bridge,threaded_batches_equal=4)
            g.release()
            for fn in (g.discover,lambda:g.numerical_project(points),g.release):
                failures.append(expect('InvalidOrReleasedHandle',fn))
        with CanonicalKernel(HOST) as fresh:
            reload=fresh.load_rhai(case['script'],s)
            same(reload.description,oracle['description'])
            same(reload.numerical_project(points)['data']['results'],projection['data']['results'])
        results.append(result);save('RESULTS.json',dict(cases=results,source=source,failures=failures))
        print(json.dumps(result),flush=True)
    commands=[
        [sys.executable,'-B',str(V4/'tools/test_surface_dispatch.py')],
        [sys.executable,'-B',str(V4/'tools/test_canonical_transport.py')],
        [sys.executable,'-B','-m','pytest',str(V4/'tests/test_production_adaptive_surface.py'),'--noconftest','-q','-k',
         'verified_sealed_surface_loads or exact_projection_and_source_branch or sizing_spd_bounds_and_curvature_response or v3_trace_and_dependency_bindings or seed_scale_defect_reproducer or manifold_topology_orientation_and_boundary or failed_audit_cannot_publish','-p','no:cacheprovider'],
        ['cargo','test','--offline','--manifest-path',str(V4/'canonical_kernel/Cargo.toml'),'--test','contract','--quiet']]
    env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(V4/'src'),str(V4/'tools')]),PYTHONDONTWRITEBYTECODE='1',CARGO_TARGET_DIR=str(ROOT/'target'))
    checks=[]
    for command in commands:
        t=time.perf_counter();p=subprocess.run(command,capture_output=True,text=True,env=env,timeout=90,cwd=ROOT)
        checks.append(dict(command=command,seconds=time.perf_counter()-t,exit_code=p.returncode,stdout=p.stdout,stderr=p.stderr))
        save('CHECKS.json',checks);assert p.returncode==0,p.stdout+p.stderr
    save('RESULTS.json',dict(classification='V4CanonicalQueryConnectionQualifiedForTestedScope',cases=results,source=source,
        failures=failures,seconds=time.perf_counter()-start,python_peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
        corrected_test_setup=dict(error='6000 unsupported rows fit inside 8 MiB; expected oversized-response assertion failed',
            resolution='derive oversized batch count from measured serialized row bytes and existing MAX_LINE; no resource limit changed'),
        no_mesh_or_conditioning=True,all_checks_passed=True))

if __name__=='__main__':main()
