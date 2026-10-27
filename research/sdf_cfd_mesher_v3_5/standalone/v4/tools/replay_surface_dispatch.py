"""One real capsule replay and an unchanged canonical-wing admission check."""
import argparse, hashlib, json, shutil, struct, subprocess, sys, time, zipfile
from pathlib import Path
import numpy as np
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from historical_capsule_surface import OUT as SEALED, CapsuleGeometry,capsule_binding,sha,save,identity
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from production_surface_authority import scientific_identity,load_production_wall,BOUND_FIELDS
from surface_binding import bind_canonical
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure


def verify_kernel_sources():
    old=V4/'results/generic_canonical_bridge'
    manifest=json.loads((old/'KERNEL_SOURCE_MANIFEST.json').read_text());h=hashlib.sha256()
    with zipfile.ZipFile(old/'KERNEL_SOURCE_SNAPSHOT.zip') as archive:
        for row in manifest['files']:
            raw=(ROOT/row['path']).read_bytes()
            assert hashlib.sha256(raw).hexdigest()==row['sha256'],row['path']
            assert archive.read(row['path'])==raw,row['path']
            h.update(row['path'].encode());h.update(b'\0');h.update(len(raw).to_bytes(8,'little'));h.update(raw)
    direct=json.loads((old/'DIRECT.json').read_text())
    assert h.hexdigest()==direct['kernel_source_sha256']
    return dict(files=len(manifest['files']),source_sha256=h.hexdigest(),
        retained_snapshot_sha256=sha(old/'KERNEL_SOURCE_SNAPSHOT.zip'),all_source_bytes_equal=True),direct


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    out=args.output.resolve()
    if out.exists():raise ValueError('OutputMustBeNew')
    if out.is_relative_to(SEALED.resolve()):raise ValueError('SealedAuthorityProtected')
    out.mkdir(parents=True);start=time.perf_counter()
    comparable=sum(p.stat().st_size for p in (SEALED/'fresh').iterdir() if p.is_file())
    budget=2*comparable
    assert shutil.disk_usage(out).free>budget
    source,direct=verify_kernel_sources()
    _,_,sealed=load_production_wall(SEALED/'fresh')
    definition=json.loads((SEALED/'GEOMETRY.json').read_text())['definition']
    a=CapsuleGeometry(definition);binding=capsule_binding(a);binding.storage_budget_bytes=budget
    assert a.geometry_identity==sealed['geometry_identity']
    save(out/'PREFLIGHT.json',dict(comparable_logical_bytes=comparable,run_budget_bytes=budget,
        initial_free_bytes=shutil.disk_usage(out).free,source=source,definition=definition,
        binding=binding.inspect(),sealed_scientific_identity=sealed['surface_identity']))
    entry=ProductionAdaptiveSdfSurfaceMesher()
    try:
        generation=entry.generate(binding,out/'capsule')
        old=json.loads((SEALED/'fresh/GENERATION.json').read_text())
        keys=('coordinate_hash','topology_hash','sizing_field_identity','residual_hash',
              'geometry_identity','configuration_identity','adaptation_identity','surface_identity',
              'carrier','mmgs_operation_rows','projections','initial_vertices','initial_triangles','final_vertices','final_triangles')
        equal={key:generation[key]==old[key] for key in keys}
        arrays={}
        for name in ('INITIAL.npz','SURFACE.npz','QUALITY.npz','GEOMETRY_CERTIFICATE.npz'):
            with np.load(SEALED/'fresh'/name) as prior,np.load(out/'capsule'/name) as current:
                arrays[name]={k:np.array_equal(prior[k],current[k]) for k in prior.files}
        record={k:generation[k] for k in BOUND_FIELDS if k!='implementation_identity'}
        record['implementation_identity']=sealed['implementation_identity']
        rebound=scientific_identity(record)
        changed=['production_adaptive_surface.py','historical_capsule_surface.py','surface_binding.py','surface_execution.py','audit_adaptive_surface.py']
        new_implementation=identity({p:sha(V4/'tools'/p) for p in changed})
        record['implementation_identity']=new_implementation
        replay=dict(equal=equal,arrays=arrays,reference_bound_scientific_identity=rebound,
            matches_sealed_identity=rebound==sealed['surface_identity'],new_dispatch_implementation_identity=new_implementation,
            new_dispatch_scientific_identity=scientific_identity(record),
            identity_scope='Reference-bound equality checks historical scientific inputs; new source identity is separate. No publication performed.',
            independent_audit=json.loads((out/'capsule/AUDIT.json').read_text())['passed'])
        replay['passed']=all(equal.values()) and all(all(v.values()) for v in arrays.values()) and replay['matches_sealed_identity'] and replay['independent_audit']
        save(out/'CAPSULE_REPLAY.json',replay)
        if not replay['passed']:raise AssertionError('CapsuleReplayMismatch')

        case=next(c for c in direct['cases'] if c['name']=='plain_wing')
        host=ROOT/'target/debug/canonical_kernel_host.exe'
        with CanonicalKernel([str(host)]) as owner:
            g=owner.load_rhai(case['script'],case['metres_per_model_unit'])
            assert g.identity==case['identity'] if 'identity' in case else g.identity==case['description']['geometry_identity']
            assert g.description==case['description']
            bound=bind_canonical(g)
            values=bound.authority.scalar_value(case['points_m']).tolist()
            exact=all(struct.pack('d',a)==struct.pack('d',b) for a,b in zip(values,case['direct_m']))
            assert exact
            admission=entry.admission(bound)
            try:entry.generate(bound,out/'wing')
            except GeometryFailure as e:failure=e.record
            else:raise AssertionError('WingUnexpectedlyExecuted')
            assert failure['code']=='AuthorityOperationUnbound' and not (out/'wing').exists()
            save(out/'WING_BINDING.json',dict(geometry_identity=g.identity,script=case['script'],
                script_sha256=hashlib.sha256(case['script'].encode()).hexdigest(),
                native_host_sha256=sha(host),samples=len(values),scalar_bits_equal=exact,
                maximum_absolute_difference_m=max(abs(a-b) for a,b in zip(values,case['direct_m'])),
                admission=admission,failure=failure,production_surface_generated=False,
                peak_host_working_set_bytes=psutil.Process(owner.transport.process.pid).memory_info().peak_wset))
        save(out/'RESULT.json',dict(classification='V4ProductionSurfaceDispatchRestoredCapsuleReplayEquivalent',
            capsule_replay_passed=True,wing_surface_qualified=False,wing_binding_blocked=True,
            seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
            final_free_bytes=shutil.disk_usage(out).free,new_logical_bytes=sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),
            no_publication=True,source_snapshot_reused=True))
        print(json.dumps(dict(capsule_replay=True,wing_identity=g.identity,wing_issues=len(admission['issues']),seconds=time.perf_counter()-start)))
    except Exception as error:
        save(out/'FAILURE.json',dict(type=type(error).__name__,detail=str(error),seconds=time.perf_counter()-start))
        raise


if __name__=='__main__':main()
