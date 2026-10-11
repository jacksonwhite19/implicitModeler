"""One file-backed qualification of hash-verified retained arrays. Never mesh/publish."""
import argparse,collections,dataclasses,json,math,shutil,subprocess,sys,time,traceback
from pathlib import Path
import numpy as np
import psutil
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from historical_capsule_surface import sha,save,SurfaceSizing,array_hash
from development_surface_policy import load_case,bind_development_policy
from surface_binding import bind_canonical
from audit_adaptive_surface import audit_directory
from replay_surface_dispatch import verify_kernel_sources
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
from run_two_geometry_surface_integration import size,inventory
OLD=V4/'results/two_geometry_surface_integration_20260928'
CASE=V4/'cases/plain_wing_retained_qualification.json'


def read(p):return json.loads(Path(p).read_text())


def verify_old():
    m=read(OLD/'MANIFEST.json')
    for n,r in m['files'].items():
        if sha(OLD/n)!=r['sha256']:raise ValueError('RetainedEvidenceMismatch:'+n)
    return {n:sha(OLD/'wing/fresh'/n) for n in ('SURFACE.npz','REFERENCES.npz','surface.off')}


def worker(out):
    start=time.perf_counter()
    try:
        tests=read(out.parent/'BATCH_TESTS.json')
        if not tests['passed'] or tests['boundary_implementation_sha256']!=sha(V4/'tools/polygon_boundary_contract.py'):
            raise ValueError('BatchTestsStale')
        original=read(V4/'cases/plain_wing_surface_integration_approved.json');case=load_case(CASE)
        if sha(V4/'cases/plain_wing_surface_integration_approved.json')!=case['qualification_revision']['original_sha256']:
            raise ValueError('OriginalApprovalMismatch')
        for k in original:
            if k!='source_files' and original[k]!=case[k]:raise ValueError('ApprovedRequirementChanged:'+k)
        changes={r['path']:r for r in case['qualification_revision']['changes']}
        for p,h in original['source_files'].items():
            if p in changes:
                if changes[p]['before']!=h or changes[p]['after']!=sha(ROOT/p):raise ValueError('QualificationDeltaMismatch')
                prior=subprocess.run(['git','show','73768d8e6651f18b8870619f77c85f82e8f24678:'+p],capture_output=True,check=True).stdout
                import hashlib
                # These existing source files have LF Git/worktree bytes.
                if hashlib.sha256(prior).hexdigest()!=h:raise ValueError('OriginalSourceRevisionMismatch:'+p)
            elif sha(ROOT/p)!=h:raise ValueError('UnreviewedSourceChange:'+p)
        try:load_case(V4/'cases/plain_wing_surface_integration_approved.json')
        except ValueError as e:
            if not str(e).startswith('CasePolicySourceMismatch:'):raise
            stale_check=str(e)
        else:raise ValueError('OriginalStaleSourceProtectionNotActive')
        files=verify_old();source,direct=verify_kernel_sources();pre=read(OLD/'PREFLIGHT.json')
        for r in pre['binaries']:
            if sha(ROOT/r['path'])!=r['sha256']:raise ValueError('BinaryIdentityMismatch')
        target=out/'qualification';target.mkdir()
        for name,digest in files.items():
            shutil.copyfile(OLD/'wing/fresh'/name,target/name)
            if sha(target/name)!=digest:raise ValueError('StagingCopyMismatch')
        generation=read(OLD/'wing/fresh/GENERATION.json')
        with np.load(target/'SURFACE.npz') as z:
            arrays={k:array_hash(z[n]) for k,n in [('coordinate_hash','coordinates'),('topology_hash','triangles'),('sizing_field_identity','metric'),('residual_hash','residual')]}
        if any(generation[k]!=v for k,v in arrays.items()):raise ValueError('ScientificArrayMismatch')
        script=next(c for c in direct['cases'] if c['name']=='plain_wing')
        with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
            g=owner.load_rhai(script['script'],script['metres_per_model_unit'])
            if g.identity!=case['geometry']['canonical_identity'] or g.stored_topology()['topology_identity']!=case['geometry']['topology_identity']:
                raise ValueError('GeometryIdentityMismatch')
            b=bind_development_policy(bind_canonical(g,accuracy_limits=case['accuracy_limits']),SurfaceSizing(**case['sizing_policy']))
            save(out/'CONTRACT.json',dataclasses.asdict(b.qualification))
            save(out/'PREFLIGHT.json',dict(source=source,files=files,arrays=arrays,case_sha256=sha(CASE),
                original_case_sha256=case['qualification_revision']['original_sha256'],intentional_changes=changes,
                stale_original_case_rejected=stale_check,binaries=pre['binaries'],qualification_record=b.qualification.record['source_dependencies'],
                approved_limits=case['accuracy_limits'],production_generation=False))
            print('Resuming existing audit_directory on verified staging copies',flush=True)
            result=audit_directory(target,b.authority,b.qualification)
            s=result.get('source_boundary',result)
            bounds=s.get('bounds',[])
            first=next((r for r in bounds if r['legacy_status']!='Certified' or r['source_normal_bound_rad'] is None
                or r['source_position_bound_m']>case['accuracy_limits']['position_m']
                or r['source_normal_bound_rad']>case['accuracy_limits']['normal_rad']),None)
            summary=dict(classification='V4RetainedWingQualificationCompleted' if result['passed'] else 'V4RetainedWingQualificationStopped',
                passed=result['passed'],source_acceptance=s.get('acceptance'),face_count=s.get('triangles_checked'),
                legacy_counts=dict(collections.Counter(r['legacy_status'] for r in bounds)),unresolved=s.get('unresolved'),
                position_bound_max_m=s.get('position_bound_max_m'),normal_bound_max_rad=s.get('normal_bound_max_rad'),
                curve_source_bound_max_m=s.get('curve_source_bound_max_m'),curve_edges=s.get('curve_count'),corners=s.get('corner_count'),
                finite_domain_failures=s.get('finite_domain_failures'),residual_gate=s.get('residual_gate'),
                routes=dict(collections.Counter(r['position_proof_route'] for r in bounds)),
                strict_outside_vertex_incidences=sum(len(r['strict_footprint_outside_vertices']) for r in bounds),
                scalar={k:v for k,v in s.get('scalar_diagnostics',{}).items() if k not in ('locations_m','values_m')},
                topology=result.get('topology',s.get('topology')),first_failed_face=first,
                continuous_geometry=result.get('continuous_geometry'),intersections=result.get('self_intersection','not reached'),
                feature_ownership=result.get('feature_ownership'),normal=result.get('normal'),
                seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
                no_remeshing=True,no_replay=True,no_capsule=True,no_publication=True)
            save(out/'RESULT.json',summary)
            print(json.dumps({k:v for k,v in summary.items() if k not in ('unresolved','first_failed_face','normal')}),flush=True)
        code=0 if result['passed'] else 1
    except Exception as e:
        save(out/'STOP.json',dict(error=str(e),structured=getattr(e,'record',None),traceback=traceback.format_exc(),
            seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
            no_retry=True,no_publication=True))
        print(traceback.format_exc(),flush=True);code=1
    verify_old()
    return code


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--worker',action='store_true');args=p.parse_args();out=args.output.resolve()
    if not out.is_relative_to((V4/'results/wing_qualification_resume').resolve()):raise ValueError('TaskOutputRequired')
    if args.worker:return worker(out)
    if out.exists():raise ValueError('OutputMustBeNew')
    out.mkdir(parents=True)
    seed=V4/'results/panel_witness/safeguards/VALID_SEED.json';old=read(OLD/'RESOURCE_PLAN.json')
    staged=sum((OLD/'wing/fresh'/n).stat().st_size for n in ('SURFACE.npz','REFERENCES.npz','surface.off'))
    budget=2*math.ceil(seed.stat().st_size*54254/792)+staged
    save(out/'RESOURCES.json',dict(output_limit_bytes=budget,minimum_free_bytes=old['minimum_free_bytes'],
        basis=dict(seed_path=str(seed),seed_bytes=seed.stat().st_size,seed_faces=792,target_faces=54254,staging_bytes=staged),
        rationale='Existing 2x reserve applied to measured certificate bytes per face; source substage and full report may both persist. Operational only.',native_timeout_seconds=120))
    samples=[];start=time.perf_counter()
    with (out/'execution.log').open('w') as log:
        child=subprocess.Popen([sys.executable,'-B',__file__,'--worker','--output',str(out)],stdout=log,stderr=subprocess.STDOUT)
        while child.poll() is None:
            used=size(out);free=shutil.disk_usage(out).free
            try:peak=psutil.Process(child.pid).memory_info().peak_wset
            except psutil.NoSuchProcess:peak=None
            samples.append(dict(seconds=time.perf_counter()-start,logical_bytes=used,free_bytes=free,peak_working_set_bytes=peak))
            if used>budget or free<old['minimum_free_bytes']:
                child.terminate();child.wait();save(out/'STOP.json',dict(error='OperationalStorageLimit',measurement=samples[-1]));break
            time.sleep(1)
    save(out/'MONITOR.json',dict(exit_code=child.returncode,seconds=time.perf_counter()-start,samples=samples))
    save(out/'MANIFEST.json',inventory(out));return child.returncode


if __name__=='__main__':raise SystemExit(main())
