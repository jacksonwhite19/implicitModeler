"""Stable owned finite research command. Requests contain data, never generated code."""
from pathlib import Path
import argparse,hashlib,json,sys
V4=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True
sys.path[:0]=[str(V4/'tools'),str(V4/'src'),str(V4/'tools/finite_support')]

def parser():
    root=argparse.ArgumentParser(description=__doc__)
    commands=root.add_subparsers(dest='operation',required=True)
    for name in ('prepare','derive','inspect','run','verify-runtime','verify-native','outer-backend','ownership-probe'):
        p=commands.add_parser(name)
        p.add_argument('--release-sha256',dest='source_manifest_sha256',required=True)
        p.add_argument('--process-context',type=Path)
        p.add_argument('--process-context-sha256')
        p.add_argument('--process-evidence-dir',type=Path)
        if name=='ownership-probe':
            p.add_argument('--probe-spec',type=Path,required=True)
            p.add_argument('--probe-spec-sha256',required=True)
            p.add_argument('--output',type=Path,required=True)
        if name=='outer-backend':
            p.add_argument('--input',type=Path)
            p.add_argument('--probe',action='store_true')
            p.add_argument('--method')
        if name in ('prepare','derive','inspect','run'):p.add_argument('--request-dir',type=Path,required=True)
        if name in ('prepare','derive'):
            p.add_argument('--output',type=Path,required=True)
            p.add_argument('--assignment',type=Path,required=True)
            p.add_argument('--assignment-sha256',required=True)
            p.add_argument('--input-contract',required=True)
        if name=='prepare':
            p.add_argument('--stage-timing',choices=('bounded_operation_caller_v2',))
            p.add_argument('--execution-target',choices=('transition','transition_background','direct_complete'),help='Explicit authenticated research target; must match assignment.')
            p.add_argument('--transition-fluid-verification',choices=('backend_trusted','continuous_source'),
                help='Source-directions FastProd defaults to backend_trusted. Explicit continuous_source retains the full SDF proof; must match the bound assignment choice.')
            p.add_argument('--upstream-forecast-diagnostics',action='store_true',
                help='Explicit bounded diagnostic capture; does not change scientific acceptance.')
            p.add_argument('--case',type=Path,required=True)
            p.add_argument('--policy',type=Path,required=True)
            p.add_argument('--maximum-faces',type=int,required=True)
            p.add_argument('--payload-seconds',type=float,required=True)
            p.add_argument('--owner-seconds',type=float,required=True)
        if name=='derive':
            p.add_argument('--prepared',type=Path,required=True)
            p.add_argument('--prepared-manifest-sha256',required=True)
        if name=='inspect':p.add_argument('--manifest-sha256',required=True)
        if name in ('verify-runtime','verify-native'):p.add_argument('--output',type=Path,required=True)
    return root

def verify_native(args):
    # This role uses Linux identity checks, not Windows handle-custody assertions.
    from finite_research.inputs import sha,native_sources,write
    if sha(V4/'runtime/RELEASE.json')!=args.source_manifest_sha256:raise ValueError('ChangedOwnedReleaseBinding')
    expected=native_sources()
    for name,digest in expected.items():
        if sha(name)!=digest:raise ValueError('ChangedNativeOperationSource',name)
    sys.path.insert(0,str(V4/'tools/finite_support/native'))
    import backend,native_runtime
    if Path(backend.__file__).resolve()!=V4/'tools/finite_support/native/backend.py':raise ValueError('WrongNativeBackendOrigin')
    result=dict(schema='OwnedFiniteNativeInspection/1',passed=True,source_root=str(V4),
        interpreter=str(Path(sys.executable).resolve()),native_backend=str(Path(backend.__file__).resolve()),
        source_files=len(expected),geometry_or_Gmsh_execution=False,
        modules={name:str(Path(module.__file__).resolve()) for name,module in list(sys.modules.items())
                 if getattr(module,'__file__',None) and Path(module.__file__).is_file()})
    missing=sorted(set(result['modules'].values())-set(expected))
    if missing:raise ValueError('UnpinnedNativeOperationImports',missing)
    for name,digest in expected.items():
        if sha(name)!=digest:raise ValueError('ChangedNativeOperationAfterImports',name)
    write(args.output,result)
    return result

def main(argv=None):
    args=parser().parse_args(argv)
    if args.operation=='verify-native':return verify_native(args)
    if args.operation=='outer-backend':
        if args.process_context is None:raise ValueError('OwnedOuterBackendContextRequired')
        if args.probe == (args.input is not None):raise ValueError('OneOuterBackendOperationRequired')
        sys.path.insert(0,str(V4/'tools/finite_support/native'))
        import native_runtime
        native_runtime.verify()
        if native_runtime.sha(V4/'runtime/RELEASE.json')!=args.source_manifest_sha256:raise ValueError('ChangedOwnedReleaseBinding')
        import core_backend
        if Path(core_backend.__file__).resolve()!=V4/'tools/core_backend.py':raise ValueError('WrongOuterBackendOrigin')
        if args.probe:result=core_backend.locks(args.method)
        else:
            core_backend.worker(args.input)
            result=dict(passed=True,backend_sha256=native_runtime.sha(args.input/'BACKEND.json'))
        native_runtime.verify()
        return result
    from finite_research import inputs
    inputs._attest_loaded_roots()
    if args.operation=='ownership-probe':
        from finite_research.ownership_probe import execute
        return execute(args)
    from finite_research import direct_inputs
    direct=(args.operation=='prepare' and args.execution_target=='direct_complete'
        or args.operation=='derive' and direct_inputs.selected(args.prepared)
        or args.operation in ('inspect','run') and direct_inputs.selected(args.request_dir))
    if direct:
        return getattr(direct_inputs,'inspect_request' if args.operation=='inspect' else args.operation)(args)
    if args.operation=='prepare':return inputs.prepare(args)
    if args.operation=='derive':return inputs.derive(args)
    if args.operation=='inspect':return inputs.inspect_request(args)
    if args.operation=='run':
        # Exact no-grant refusal precedes project owners in the stable live entry.
        if inputs.sha(V4/'runtime/RELEASE.json')!=args.source_manifest_sha256:raise ValueError('ChangedOwnedReleaseBinding')
        from finite_research import run
        request=inputs.read(args.request_dir/'LIVE_REQUEST.json')
        return run.main(['--request-dir',str(args.request_dir.resolve()),*request['arguments']])
    with inputs.source_scope(args.source_manifest_sha256) as (d,pins):
        import sealed_front_connection
        first=sealed_front_connection.install_qualified_stack()
        second=sealed_front_connection.install_qualified_stack()
        if set(first)!=set(second):raise ValueError('NormalInitializerIdentity')
        from sdf_cfd_mesher_v4.runtime import frozen_imports
        modules=frozen_imports.legacy_modules()
        if frozen_imports.legacy_modules() is not modules:raise ValueError('LegacyModuleCacheIdentity')
        import path_preserving_deduplication,optimized_durable_json
        if path_preserving_deduplication.durable_json is not optimized_durable_json.durable_json:
            raise ValueError('NormalNativeWriterNotInstalled')
        import background_preparation,background_native_guard,hashlib
        with background_preparation.held() as preparation:
            if preparation is None:raise ValueError('RequiredWindowsPreparationAdmission')
            preparation.require_live()
            preparation_identity=dict(source_sha256=preparation.source_sha256,binary_sha256=preparation.binary_sha256,build_sha256=preparation.build_sha256)
        try:preparation.require_live()
        except ValueError as error:
            if 'ClosedBackgroundPreparation' not in str(error):raise
        else:raise ValueError('PreparationLifetimeDidNotClose')
        source=V4/'runtime/source/background/independent_walk.rs'
        pin=hashlib.sha256(source.read_bytes()).hexdigest()
        library=source.parent/('independent_'+pin+'.dll')
        with background_native_guard.held_library(library,{'independent_walk.rs':pin}) as native:
            if native is None:raise ValueError('RequiredRustAdmission')
            rust_identity=dict(path=str(library),loaded_name=str(native._name),source_sha256=pin,binary_sha256=inputs.sha(library),build_sha256=inputs.sha(library.with_suffix('.BUILD.json')))
        result=dict(schema='OwnedFiniteRuntimeInspection/1',passed=True,source_root=str(V4),
            preparation_admission_closed=True,rust_guard_scope_closed=True,
            native_admissions=dict(preparation=preparation_identity,rust=rust_identity),
            release_sha256=args.source_manifest_sha256,source_files=len(pins),normal_initializer_calls=2,
            geometry_or_Gmsh_execution=False)
        if args.output.exists():raise ValueError('FreshInspectionOutputRequired')
        path_preserving_deduplication.durable_json(args.output,result)
        if inputs.read(args.output)!=result:raise ValueError('NativeWriterReadback')
        return result

if __name__=='__main__':
    from finite_research.process_entry import execute
    options=parser().parse_args()
    raise SystemExit(execute(main,options.process_context,options.process_context_sha256,options.process_evidence_dir))
