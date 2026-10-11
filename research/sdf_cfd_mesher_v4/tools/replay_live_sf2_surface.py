"""One fresh SF2 replay, exact evidence comparison, then existing publication.

No geometry, projection, metric, meshing or certificate implementation here.
"""
import argparse,json,shutil,subprocess,sys,time,traceback
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from run_live_sf2_surface import read,save,sha,size,inventory,resource_plan,CASE,CASE_HASH
from run_two_geometry_surface_integration import compare
from historical_capsule_surface import identity,array_hash
from replay_surface_dispatch import verify_kernel_sources
from sf2_surface_policy import load_case
from production_surface_authority import publish_surface,load_production_wall,scientific_identity

REFERENCE=V4/'results/live_sf2_surface/utf8_run'
SEED=Path('labelled_spline_seed/surface/live/seed')
EXPECTED={'fresh/SURFACE.npz':'56b29161cee7fb79908bef86152e4149e488baf9b43389aab350eb5bcafc54e5',
 'fresh/AUDIT.json':'922b4d15ab6f798fdafd1e095f9eba8851668d467b9d941acafc4ac0365c5786',
 'fresh/metric.bin':'3523aa2af33b2b09897e973ed20bfda209482f7bfb86a3c2bbd2936386fd7f35'}
NONSCIENTIFIC={'seconds','timing','peak_rss_bytes','retained_rss_bytes','topology_seconds'}


def scientific(value):
    """Drop only named timing/process metadata; retain every scientific field."""
    if isinstance(value,dict):return {k:scientific(v) for k,v in value.items() if k not in NONSCIENTIFIC}
    if isinstance(value,list):return [scientific(v) for v in value]
    return value


def require(ok,code):
    if not ok:raise ValueError(code)


def preflight():
    require(sha(CASE)==CASE_HASH,'CaseIdentityMismatch')
    case=load_case();source,_=verify_kernel_sources();prior=read(REFERENCE/'PREFLIGHT.json')
    require(source==prior['source'],'ReferenceKernelIdentityMismatch')
    manifest=read(REFERENCE/'MANIFEST.json')
    for name,row in manifest.items():
        p=REFERENCE/name
        require(p.stat().st_size==row['bytes'] and sha(p)==row['sha256'],'ReferenceEvidenceMismatch:'+name)
    for name,h in EXPECTED.items():require(sha(REFERENCE/name)==h,'RequiredReferenceMismatch:'+name)
    require(read(REFERENCE/'RESULT.json')['passed'] and read(REFERENCE/'fresh/AUDIT.json')['passed'],'ReferenceNotQualified')
    for name,h in prior['binaries'].items():require(sha(ROOT/name)==h,'ReferenceBinaryMismatch:'+name)
    dependencies={}
    for row in read(REFERENCE/'PRESERVATION_BASELINE.json')['files']:
        expected=prior['runner_identity'] if row['path'].endswith('/run_live_sf2_surface.py') else row['sha256']
        require(sha(ROOT/row['path'])==expected,'ReferenceDependencyChanged:'+row['path'])
        if row['path'].endswith(('.py','.rs')):dependencies[row['path']]=expected
    dependencies.update(case['source_files'])
    for p in (Path(__file__),V4/'tools/production_surface_authority.py',V4/'tools/run_two_geometry_surface_integration.py'):
        dependencies[str(p.relative_to(ROOT)).replace('\\','/')]=sha(p)
    implementation=dict(schema='V4LiveSF2ReplayImplementation/1',dependencies=dependencies,
        kernel_source=source,binaries=prior['binaries'],case_sha256=CASE_HASH,
        geometry_identity=case['geometry']['canonical_identity'],spline_identity=case['geometry']['spline_identity'],
        units=case['geometry']['metres_per_model_unit'],qualification=case['qualification'])
    plan=resource_plan()
    return dict(passed=True,implementation=implementation,implementation_identity=identity(implementation),
        reference_manifest_sha256=sha(REFERENCE/'MANIFEST.json'),reference_audit_sha256=sha(REFERENCE/'fresh/AUDIT.json'),
        reference_files_checked=len(manifest),reference_storage_bytes=size(REFERENCE),
        reference_worker_seconds=read(REFERENCE/'RESULT.json')['seconds'],resources=plan,
        authorization='Current user authorizes exactly one fresh deterministic replay and existing publication only after all required checks pass.',
        no_prisms=True,no_capsule_run=True,no_core=True,no_cfd=True)


def compare_arrays(a,b):
    with np.load(a) as x,np.load(b) as y:
        out={'same_keys':set(x.files)==set(y.files),'arrays':{}}
        for key in sorted(set(x.files)|set(y.files)):
            if key not in x.files or key not in y.files:out['arrays'][key]={'equal':False};continue
            p,q=x[key],y[key];same=p.shape==q.shape and p.dtype==q.dtype
            equal=same and np.array_equal(p,q)
            bits=same and p.tobytes()==q.tobytes()
            row=dict(shape=list(p.shape),dtype=str(p.dtype),equal=bool(equal),stored_bits_equal=bool(bits),
                reference_hash=array_hash(p),replay_hash=array_hash(q))
            if same:
                row['different_elements']=int(np.count_nonzero(p!=q))
                if np.issubdtype(p.dtype,np.number):row['maximum_absolute_difference']=0. if equal else float(np.max(np.abs(p.astype(float)-q.astype(float))))
            out['arrays'][key]=row
        out['passed']=out['same_keys'] and all(r['equal'] and r.get('stored_bits_equal',False) for r in out['arrays'].values())
        return out


def scientific_comparison(reference,replay):
    a,b=reference/'fresh',replay/'fresh'
    result=dict(existing_comparison=compare(a,b),arrays={},files={},records={})
    names=['INITIAL.npz','SURFACE.npz','REFERENCES.npz','QUALITY.npz','GEOMETRY_CERTIFICATE.npz',str(SEED/'SEED.npz')]
    for name in names:result['arrays'][name]=compare_arrays(a/name,b/name)
    result['arrays']['partial_face_and_feature_bounds']=compare_arrays(reference/'PARTIAL_CERTIFICATES.npz',replay/'PARTIAL_CERTIFICATES.npz')
    for name in ['metric.bin','raw.mesh','surface.off']+[str(SEED/f'{i}.{kind}') for i in range(1,5) for kind in ('input','output')]:
        h,k=sha(a/name),sha(b/name);result['files'][name]=dict(reference=h,replay=k,equal=h==k)
    for name in ['CARRIER.json',str(SEED/'SEED_AUDIT.json'),'AUDIT.json']:
        p,q=scientific(read(a/name)),scientific(read(b/name))
        result['records'][name]=dict(equal=p==q,reference_identity=identity(p),replay_identity=identity(q))
    result['excluded_metadata']=sorted(NONSCIENTIFIC)
    result['reference_payloads']={name:sha(a/name) for name in names}
    result['replay_payloads']={name:sha(b/name) for name in names}
    result['reference_audit_sha256']=sha(a/'AUDIT.json');result['replay_audit_sha256']=sha(b/'AUDIT.json')
    result['passed']=result['existing_comparison']['passed'] and all(x['passed'] for x in result['arrays'].values()) and all(x['equal'] for x in result['files'].values()) and all(x['equal'] for x in result['records'].values())
    return result


def publish(out,comparison,pre):
    require(comparison['passed'],'ReplayMismatch')
    replay=out/'run/fresh';authority=out/'authority'
    require(not authority.exists(),'AuthorityOutputMustBeNew')
    names=('SURFACE.npz','AUDIT.json','QUALITY.npz','GEOMETRY_CERTIFICATE.npz','GENERATION.json','REFERENCES.npz','EXECUTION_BINDING.json','WRITTEN_METRIC_POLICY.json')
    needed=sum((replay/n).stat().st_size for n in names)
    require(size(out)+needed<pre['resources']['output_limit_bytes'],'PublicationStorageBudgetExceeded')
    require(shutil.disk_usage(out).free-needed>pre['resources']['minimum_free_bytes'],'PublicationFreeReserveExceeded')
    authority.mkdir()
    for name in names:
        shutil.copyfile(replay/name,authority/name)
        require(sha(replay/name)==sha(authority/name),'PublicationCopyMismatch:'+name)
    # Existing publisher owns and creates the authority record. Sidecars provide
    # inspectable inputs to its implementation and determinism hash bindings.
    save(authority/'IMPLEMENTATION.json',pre['implementation'])
    save(authority/'REPLAY.json',comparison)
    shutil.copyfile(CASE,authority/'CASE.json')
    generation=read(authority/'GENERATION.json')
    record=publish_surface(authority,generation,pre['implementation_identity'],comparison)
    x,f,loaded=load_production_wall(authority)
    require(record==loaded and scientific_identity(loaded)==record['surface_identity'],'PublishedLoaderMismatch')
    require(identity(read(authority/'IMPLEMENTATION.json'))==record['implementation_identity'],'PublishedImplementationBindingMismatch')
    require(identity(read(authority/'REPLAY.json'))==record['determinism_identity'],'PublishedReplayBindingMismatch')
    require(sha(authority/'CASE.json')==CASE_HASH,'PublishedCaseMismatch')
    require(sha(authority/'REFERENCES.npz')==comparison['replay_payloads']['REFERENCES.npz'],'PublishedReferencesMismatch')
    return dict(passed=True,authority=record,loader_verified=True,vertices=len(x),triangles=len(f),
        implementation_sidecar_verified=True,replay_sidecar_verified=True,case_and_references_verified=True,
        scope='Existing loader verifies its five payload files and scientific arrays; task additionally verifies implementation/replay sidecars, case and references against identity-bound evidence. No expanded loader guarantee claimed.')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--preflight-only',action='store_true');args=parser.parse_args()
    out=args.output.resolve();require(out.is_relative_to((V4/'results/live_sf2_surface').resolve()),'TaskOwnedOutputRequired')
    require(not out.exists(),'OutputMustBeNew');start=time.perf_counter();active='preflight';out.mkdir(parents=True)
    try:
        pre=preflight();save(out/'PREFLIGHT.json',pre)
        if args.preflight_only:return 0
        require(shutil.disk_usage(out).free>pre['resources']['minimum_free_bytes'],'InsufficientOperationalReserve')
        active='fresh_replay'
        command=[sys.executable,'-B',str(V4/'tools/run_live_sf2_surface.py'),'--output',str(out/'run')]
        save(out/'INVOCATION.json',dict(command=command,reference_is_not_generation_input=True,fresh_process=True))
        process=subprocess.run(command)
        require(process.returncode==0,'FreshReplayExecutionFailed:see run/STOP.json')
        active='deterministic_comparison'
        after=preflight();require(after['implementation_identity']==pre['implementation_identity'],'ImplementationChangedDuringReplay')
        comparison=scientific_comparison(REFERENCE,out/'run');save(out/'REPLAY.json',comparison)
        require(comparison['passed'],'RequiredScientificReplayMismatch')
        active='publication';published=publish(out,comparison,pre);save(out/'PUBLICATION.json',published)
        save(out/'RESULT.json',dict(classification='V4LiveSmoothSF2SurfaceReplayQualifiedAndPublished',passed=True,
            surface_identity=published['authority']['surface_identity'],seconds=time.perf_counter()-start,
            replay_comparison_passed=True,loader_verified=True,no_prisms=True,no_capsule=True,no_core=True,no_cfd=True))
        return 0
    except Exception as error:
        save(out/'STOP.json',dict(active=active,error_type=type(error).__name__,error=str(error),traceback=traceback.format_exc(),
            seconds=time.perf_counter()-start,no_retry=True,publication_exists=(out/'authority/SURFACE_AUTHORITY.json').exists()))
        print(traceback.format_exc(),flush=True);return 1
    finally:save(out/'MANIFEST.json',inventory(out))


if __name__=='__main__':raise SystemExit(main())
