"""Native metadata-only preflight under the existing authenticated resource owner."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,os,sys,time

SOURCE=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True
sys.path[:0]=[str(SOURCE/'tools'),str(SOURCE/'src'),str(SOURCE/'tools/finite_support')]
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,x):
    with Path(p).open('x',encoding='utf8') as f:json.dump(x,f,indent=2,allow_nan=False)

def worker(spec_path):
    spec=read(spec_path)
    from finite_research import inputs
    with inputs.source_scope(spec['release_sha256'],read(spec['pins'])):
        from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
        from seed_controls import query_controls,validate_plan
        import native_scope
        native_scope.require_transport()
        case=read(spec['case']);g=case['geometry'];settings=case['general_sdf']['seed']
        script=Path(g['rhai_file'])
        if sha(script)!=g['rhai_sha256']:raise ValueError('ChangedImportedRhai')
        with CanonicalKernel([str(SOURCE/'runtime/bin/canonical_kernel_host.exe')],timeout=60) as host:
            obj=host.load_rhai(script.read_text(encoding='utf-8-sig'),g['metres_per_model_unit'])
            packet=obj.surface_seed_plan(**query_controls(settings))
            write(Path(spec['result']).with_name('SEED_METADATA_RAW.json'),dict(description=obj.description,plan=packet))
            validate_plan(settings,packet['data'],obj.description)
            write(spec['result'],dict(schema='V4FileSeedPreflight/1',native_description=obj.description,
                native_plan=packet,native_executable_sha256=sha(SOURCE/'runtime/bin/canonical_kernel_host.exe'),
                source_release_sha256=spec['release_sha256'],case_sha256=sha(spec['case']),
                limits=spec['limits'],scalar_grid_execution=False,scientific_acceptance=False))
    return 0

def run(case_path,folder,release,pins,preparation_deadline):
    """One bounded metadata pass; never calls surface_seed or changes controls."""
    from finite_research import inputs
    folder=Path(folder)
    seconds=min(180.,(preparation_deadline-datetime.now(timezone.utc)).total_seconds())
    if seconds<180:raise TimeoutError('SeedMetadataPreflightRequires180SecondOwnerWindow')
    settings=read(case_path)['general_sdf']['seed']
    limits=dict(declared_seed_timeout_seconds=settings['timeout_seconds'],metadata_transport_timeout_seconds=60,
        metadata_owner_seconds=seconds,metadata_payload_ceiling_seconds=seconds-60,
        windows_job_commit_bytes=16*1024**3,linux_memory_max_bytes=2*1024**3,linux_swap_max_bytes=0,
        output_limit_bytes=read(case_path)['resources']['output_limit_bytes'],minimum_free_bytes=read(case_path)['resources']['minimum_free_bytes'],protocol_limit_bytes=8*1024**2,
        scope='Resource owners remain authoritative. Plan has no calibrated time/peak-memory prediction.')
    spec=folder/'SEED_PREFLIGHT_SPEC.json';manifest=folder/'SEED_PREFLIGHT_PINS.json'
    write(manifest,pins)
    write(spec,dict(case=str(case_path),result=str(folder/'SEED_PREFLIGHT.json'),pins=str(manifest),release_sha256=release,limits=limits))
    extra=dict(pins);extra[str(manifest)]=sha(manifest);extra[str(spec)]=sha(spec)
    with inputs.source_scope(release,extra) as (dependency,unused):
        import native_scope,resource_limits
        dependency.supervisor.HERE=folder
        resources=resource_limits.validate({k:limits[k] for k in resource_limits.APPROVED})
        images=read(SOURCE/'runtime/FINITE_PYTHON_IDENTITY.json')
        result=native_scope.bounded('seed_metadata',[images['child']['executable'],'-B',str(Path(__file__).resolve()),'--worker',str(spec)],seconds,
            {Path(k):v for k,v in extra.items()},dependency.supervisor,resources=resources)
    write(folder/'SEED_PREFLIGHT_TERMINAL.json',result)
    if result['returncode']!=0:raise ValueError('SeedMetadataOwnerFailed',result)
    return read(folder/'SEED_PREFLIGHT.json')

if __name__=='__main__':
    if len(sys.argv)!=3 or sys.argv[1]!='--worker':raise ValueError('MetadataWorkerSpecRequired')
    raise SystemExit(worker(Path(sys.argv[2])))
