"""Live retained-point transport checks; no surface generation or qualification run."""
import argparse,json,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import psutil
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from historical_capsule_surface import sha,save
from polygon_boundary_contract import scalar_diagnostics,scalar_batch_limit
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from replay_surface_dispatch import verify_kernel_sources


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError('OutputMustBeNew')
    start=time.perf_counter();source,direct=verify_kernel_sources();limit=scalar_batch_limit()
    old=V4/'results/two_geometry_surface_integration_20260928'
    manifest=json.loads((old/'MANIFEST.json').read_text())
    for n,r in manifest['files'].items():assert sha(old/n)==r['sha256'],n
    pre=json.loads((old/'PREFLIGHT.json').read_text());host=ROOT/'target/debug/canonical_kernel_host.exe'
    assert sha(host)==next(r['sha256'] for r in pre['binaries'] if Path(r['path'])==Path('target/debug/canonical_kernel_host.exe'))
    with np.load(old/'wing/fresh/SURFACE.npz') as z:
        points=np.concatenate([z['coordinates'],z['coordinates'][z['triangles']].mean(1)])
    case=next(c for c in direct['cases'] if c['name']=='plain_wing');events=[]
    def reject(name,code,fn):
        try:fn()
        except GeometryFailure as e:
            assert e.record['code']==code,(name,e.record)
            events.append(dict(name=name,expected=code,actual=e.record,aggregate_returned=False));return
        raise AssertionError('Expected failure: '+name)
    with CanonicalKernel([str(host)]) as owner:
        g=owner.load_rhai(case['script'],case['metres_per_model_unit'])
        assert g.identity==pre['admissions']['wing']['geometry_identity']
        original=g.query
        for n in (0,17,limit,limit+1,len(points)):
            calls=[];t=time.perf_counter()
            def logged(chunk):calls.append(len(chunk));return original(chunk)
            with patch.object(g,'query',logged):values,sizes=scalar_diagnostics(g,points[:n])
            alternative=[];partition=[]
            for i in range(0,max(1,n),limit//2):
                chunk=points[i:min(i+limit//2,n)].tolist();partition.append(len(chunk));alternative.extend(original(chunk))
            equal=np.array_equal(np.asarray(values,np.float64).view(np.uint64),np.asarray(alternative,np.float64).view(np.uint64))
            assert equal and calls==sizes and sum(sizes)==n
            events.append(dict(name='live_equivalence',points=n,chunk_sizes=sizes,alternative_partition=partition,
                bitwise_equal=equal,maximum_absolute_difference=float(np.max(abs(np.array(values)-alternative))) if n else 0.,seconds=time.perf_counter()-t))
        reject('low_level_oversize','BatchTooLarge',lambda:original(points[:limit+1].tolist()))
        bad=points[:limit+1].copy();bad[-1]=[1e300,0,0]
        reject('later_native_chunk_failure','CoordinateOutOfKernelRange',lambda:scalar_diagnostics(g,bad))
        for name,answer in [('nonfinite',[float('nan')]),('wrong_count',[])]:
            calls=[]
            def broken(chunk):
                calls.append(len(chunk))
                return original(chunk) if len(calls)==1 else answer
            with patch.object(g,'query',broken):
                reject('later_'+name,'InvalidScalarDiagnosticResponse',lambda:scalar_diagnostics(g,points[:limit+1]))
            assert calls==[limit,1]
        bad=points[:limit+1].copy();bad[-1]=[float('nan'),0,0]
        reject('later_nonfinite_coordinate','InvalidRequest',lambda:scalar_diagnostics(g,bad))
        prior=g.description;g.description=dict(prior,sign='tampered')
        reject('descriptor_tampering','GeometryDescriptionMismatch',lambda:scalar_diagnostics(g,points[:1]))
        g.description=prior
        # Exercise error propagation at the real qualifier boundary on the retained
        # tiny seed. A smaller TEST partition forces a later chunk, not a new limit.
        from surface_binding import bind_canonical
        from audit_adaptive_surface import audit_directory
        b=bind_canonical(g)
        with np.load(V4/'results/canonical_polygon_connection/labelled_seed/surface/A/seed/SEED.npz') as z:
            a=b.authority;x=z['points']*a.scale;faces=z['faces'];fr=z['refs']
        data=dict(x=x,faces=faces,face_refs=fr,
            curve_edges=np.array([(*edge,r) for r,edge in a.unit.curve_vertices.items()],int),
            corner_refs=np.array([a.unit.corner_ref[n] for n in a.unit.point_names],int))
        calls=[]
        def injected(chunk):
            calls.append(len(chunk))
            if len(calls)==2:raise GeometryFailure('InjectedChunkFailure','test','expected')
            return original(chunk)
        with patch('polygon_boundary_contract.scalar_batch_limit',return_value=17),patch.object(g,'query',injected):
            reject('qualifier_later_chunk_no_result','InjectedChunkFailure',lambda:audit_directory(args.output,a,b.qualification,source_arrays=data))
        assert calls==[17,17]
        g.release()
        reject('released_handle','InvalidOrReleasedHandle',lambda:scalar_diagnostics(g,points[:0]))
    result=dict(passed=True,source=source,geometry_identity=g.identity,limit=limit,limit_source='canonical_kernel/src/lib.rs::MAX_BATCH',
        host_sha256=sha(host),boundary_implementation_sha256=sha(V4/'tools/polygon_boundary_contract.py'),
        canonical_client_sha256=sha(V4/'src/sdf_cfd_mesher_v4/sdf/canonical.py'),events=events,
        original_manifest_files_verified=len(manifest['files']),seconds=time.perf_counter()-start,
        peak_working_set_bytes=psutil.Process().memory_info().peak_wset)
    save(args.output,result);print(json.dumps(result),flush=True)


if __name__=='__main__':main()
