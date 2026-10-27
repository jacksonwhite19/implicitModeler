"""Focused interface checks; all shapes come from existing kernel constructors."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time
import psutil

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[3]
V4 = ROOT/'research/sdf_cfd_mesher_v4'
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel, EnclosingRegion, GeometryFailure
from sdf_cfd_mesher_v4.sdf.cgal import run_cgal, CgalInvocation
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher

OUT = V4/'results/generic_canonical_bridge'
HOST = ROOT/'target/debug/canonical_kernel_host.exe'
CASES = ROOT/'target/debug/bridge_cases.exe'
REGION = EnclosingRegion((0.,0.,0.),4.,'diagnostic caller enclosure; includes listed kernel bounds')


def save(name, data):
    (OUT/name).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''): h.update(block)
    return h.hexdigest()


def linux(p): return '/mnt/'+str(p)[0].lower()+str(p)[2:].replace('\\','/')


def command(points, repeats=10, label='callback'):
    return CgalInvocation(tuple(['wsl','-e','/usr/bin/time','-v','-o',linux(OUT/(label+'.resources.txt')),
            'timeout','--kill-after=2','45',linux(OUT/'canonical_cgal_seed')]),
            '--probe',linux(points),.001,repetitions=repeats)


def same_bits(a,b): return len(a)==len(b) and all(struct.pack('d',x)==struct.pack('d',y) for x,y in zip(a,b))


def expect(code, action):
    try: action()
    except GeometryFailure as e:
        assert e.record['code']==code, e.record
        return e.record
    raise AssertionError('Expected '+code)


def measured_process(command):
    start=time.perf_counter(); peak=0
    with subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE) as p:
        process=psutil.Process(p.pid)
        while p.poll() is None:
            try: peak=max(peak,process.memory_info().peak_wset)
            except psutil.Error: pass
            time.sleep(.02)
        stdout,stderr=p.communicate()
        assert p.returncode==0,(p.returncode,stderr.decode(errors='replace'))
    return dict(seconds=time.perf_counter()-start,peak_working_set_bytes=peak,
                peak_scope='Windows process peak sampled every 20 ms; last sub-interval may be missed',
                stdout=stdout.decode(errors='replace'),stderr=stderr.decode(errors='replace'))


def main():
    start=time.perf_counter()
    resource=dict(initial_c_free_bytes=shutil.disk_usage(ROOT).free)
    resource['direct_process']=measured_process([str(CASES),str(OUT/'DIRECT.json')])
    direct=json.loads((OUT/'DIRECT.json').read_text())
    results=[]
    mesher=ProductionAdaptiveSdfSurfaceMesher()
    for case in direct['cases']:
        native_command=[str(CASES),'--serve-capsule'] if case['name']=='capsule' else [str(HOST)]
        with CanonicalKernel(native_command) as owner:
            geom=owner.initial_geometry() if case['name']=='capsule' else owner.load_rhai(case['script'],case['metres_per_model_unit'])
            assert geom.description==case['description']
            points=case['points_m']; values=geom.query(points)
            assert same_bits(values,case['direct_m'])
            assert geom.query([])==[]
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                repeated=list(pool.map(lambda _:geom.query(points),range(16)))
            assert all(same_bits(v,values) for v in repeated)
            t=time.perf_counter()
            for _ in range(100): assert same_bits(geom.query(points),values)
            ipc_seconds=time.perf_counter()-t
            point_file=OUT/(case['name']+'.points.txt')
            point_file.write_text(''.join(' '.join(format(x,'.17g') for x in p)+'\n' for p in points))
            cgal=mesher.cgal(geom,command(point_file,label=case['name']),REGION,timeout_seconds=50,maximum_queries=2*10*len(points))
            assert same_bits(cgal['values_m'],values)
            assert cgal['labels']==[int(x<0) for x in values],cgal['labels']
            admission=mesher.admission(geom,REGION)
            blocked=expect('UnsupportedCapability',lambda:mesher.generate(geom,OUT/'NOT_CREATED',enclosing_region=REGION))
            assert not (OUT/'NOT_CREATED').exists()
            errors=[expect('InvalidRequest',lambda:owner.request(op='query',handle=geom.handle,points_m=[[0,0]])),
                    expect('InvalidRequest',lambda:geom.query([[float('nan'),0,0]])),
                    expect('CoordinateOutOfKernelRange',lambda:geom.query([[1e300,0,0]])),
                    expect('BatchTooLarge',lambda:geom.query([[0,0,0]]*65537)),
                    expect('InvalidOrReleasedHandle',lambda:owner.request(op='query',handle=99999,points_m=points)),
                    expect('InvalidEnclosingRegion',lambda:EnclosingRegion((0,0,0),-1,'test').validate(geom))]
            if case['script']:
                errors.append(expect('FreshHostRequired',lambda:owner.load_rhai(case['script'],.001)))
            memory=psutil.Process(owner.transport.process.pid).memory_info().peak_wset
            geom.release()
            errors.append(expect('InvalidOrReleasedHandle',lambda:geom.query(points)))
            errors.append(expect('InvalidOrReleasedHandle',geom.release))
        with CanonicalKernel(native_command) as owner:
            replay=owner.initial_geometry() if case['name']=='capsule' else owner.load_rhai(case['script'],case['metres_per_model_unit'])
            assert replay.identity==geom.identity and same_bits(replay.query(points),values)
            # Exercise callback failure after CGAL starts, not merely admission.
            original_handle=replay.handle; replay.handle=99999
            callback_runtime=expect('InvalidOrReleasedHandle',lambda:run_cgal(replay,command(point_file,1,'invalid_callback'),REGION,timeout_seconds=15,maximum_queries=100))
            replay.handle=original_handle
            budget=expect('QueryBudgetExceeded',lambda:run_cgal(replay,command(point_file,1,'budget'),REGION,timeout_seconds=15,maximum_queries=1))
        results.append(dict(name=case['name'],geometry_identity=geom.identity,points=len(points),
                            direct_bridge_cgal_bits_equal=True,max_abs_difference_m=0.,
                            direct_query_seconds=case['direct_seconds'],native_query_seconds=case['native_bridge_seconds'],
                            direct_native_query_count=case['timed_query_count'],ipc_batch_size=len(points),ipc_queries=100*len(points),
                            ipc_seconds=ipc_seconds,load_seconds=case['load_seconds'],host_load_seconds=geom.load_seconds,
                            host_peak_working_set_bytes=memory,threaded_batches=16,repeat_construction_identity_equal=True,
                            cgal=cgal,admission=admission,blocked_generate=blocked,
                            failures=errors,callback_runtime_failure=callback_runtime,callback_budget_failure=budget))
        print(json.dumps(dict(case=case['name'],points=len(points),equal=True)),flush=True)
    with CanonicalKernel([str(HOST)]) as owner:
        constructor=expect('ScriptConstructionFailed',lambda:owner.load_rhai('no_such_geometry()',1))
    resource.update(final_c_free_bytes=shutil.disk_usage(ROOT).free,seconds=time.perf_counter()-start)
    save('PROBE_RESULT.json',dict(classification='V4GenericCanonicalGeometryBridgeQualifiedForTestedScope',
          cases=results,constructor_failure=constructor,resources=resource,
          kernel_source_sha256=direct['kernel_source_sha256'],
          binaries={p.name:sha(p) for p in [HOST,CASES,OUT/'canonical_cgal_seed']},
          scope='Scalar and actual CGAL domain callback only. No extraction, surface, prism, core or CFD qualification.'))


if __name__=='__main__': main()
