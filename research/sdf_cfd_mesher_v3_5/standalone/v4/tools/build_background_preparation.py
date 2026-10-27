"""Pinned C-only preparation build, with actual include/link closure."""
from pathlib import Path
from contextlib import ExitStack
import sys,os,sysconfig,subprocess,time,re,platform
V4=Path(__file__).resolve().parents[1];HERE=V4/'runtime/source/background_preparation'
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from immutable_state_access import guard
from path_preserving_deduplication import durable_json,file_hash

def main():
    if os.name!='nt' or sys.version_info[:2]!=(3,12):raise ValueError('UnsupportedPreparationBuildRuntime')
    from setuptools._distutils._msvccompiler import _get_vc_env
    vc={k.upper():v for k,v in _get_vc_env('x86_amd64').items()}
    compiler=Path(vc['VCTOOLSINSTALLDIR'])/'bin/Hostx64/x64/cl.exe'
    roots=[Path(vc[k]).resolve() for k in ('VCTOOLSINSTALLDIR','WINDOWSSDKDIR')]
    paths=[Path(p) for p in vc['PATH'].split(os.pathsep) if p and any(Path(p).resolve().is_relative_to(r) for r in roots)]
    paths=[compiler.parent,*paths,Path(os.environ['SYSTEMROOT'])/'System32',Path(sys.base_prefix)]
    environment={k:os.environ[k] for k in ('SYSTEMROOT','WINDIR','TEMP','TMP') if k in os.environ}
    environment.update(INCLUDE=vc['INCLUDE'],LIB=vc['LIB'],PATH=os.pathsep.join(dict.fromkeys(map(str,paths))))
    # No ambient CL, _CL_, LINK, LIBPATH, user includes or option injection.
    
    
    source=HERE/'preparation.cpp'
    include=Path(sysconfig.get_path('include'));library=Path(sys.base_prefix)/'libs/python312.lib'
    runtime=Path(sys.base_prefix)/'python312.dll'
    with ExitStack() as held:
        held.enter_context(guard(source));pin=file_hash(source)
        target=source.parent/('_v4_background_preparation_'+pin+sysconfig.get_config_var('EXT_SUFFIX'))
        record=target.with_name(target.name+'.BUILD.json')
        if target.exists() or record.exists():raise ValueError('PreservePreparationBuild')
        initial={source,Path(__file__),library,runtime}
        initial.update(include.rglob('*.h'))
        # Previously traced actual compiler closure is guarded BEFORE this
        # final compile. Any newly discovered dependency rejects the build.
        import json
        if len(sys.argv)!=2:raise ValueError('ExistingPreparationBuildTraceRequired')
        discovery=Path(sys.argv[1]).resolve()
        held.enter_context(guard(discovery))
        traced=json.loads(discovery.read_text(encoding='utf-8'))
        discovery_sha256=file_hash(discovery)
        initial.update(Path(p) for p in traced['included_headers'])
        initial.update(Path(p) for p in traced['link_libraries'])
        for directory in paths:
            if directory.is_dir() and any(directory.resolve().is_relative_to(r) for r in roots):
                initial.update(p for p in directory.iterdir() if p.is_file() and p.suffix.lower() in ('.exe','.dll'))
        initial.update(p for p in Path(sys.base_prefix).glob('*.dll') if p.is_file())
        initial.update(Path(os.environ['SYSTEMROOT'])/'System32'/name for name in
            ('kernel32.dll','kernelbase.dll','ntdll.dll','ucrtbase.dll','msvcrt.dll','vcruntime140.dll'))
        for p in sorted(initial-{source}):held.enter_context(guard(p))
        command=[str(compiler),'/nologo','/LD','/TC','/std:c17','/O2','/fp:strict','/MD','/showIncludes',
            '/I'+str(include),'/Fo'+str(source.parent/'preparation.obj'),str(source),
            '/link','/VERBOSE:LIB',str(library),'/OUT:'+str(target)]
        tick=time.perf_counter();result=subprocess.run(command,cwd=source.parent,env=environment,capture_output=True,text=True)
        headers={Path(line.split('Note: including file:',1)[1].strip()) for line in result.stdout.splitlines() if 'Note: including file:' in line}
        libraries={Path(p) for p in re.findall(r'[A-Za-z]:\\[^\r\n]*?\.lib',result.stdout,re.I)}
        if not (headers|libraries)<=initial:raise ValueError('UnheldPreparationDependency', sorted(map(str,(headers|libraries)-initial)))
        closure=initial
        if not headers or library not in libraries:raise ValueError('IncompletePreparationBuildTrace')
        if file_hash(source)!=pin:raise ValueError('ChangedPreparationSource')
        durable_json(record,dict(schema='V4BackgroundPreparationBuild/1',build_kind='c17-native-preparation',
            sources={source.name:pin},binary_sha256=file_hash(target) if not result.returncode else None,
            command=command,environment=environment,dependencies={str(p):file_hash(p) for p in sorted(closure)},
            precompile_dependencies=[str(p) for p in sorted(initial)],discovery_record=str(discovery),discovery_sha256=discovery_sha256,
            included_headers=[str(p) for p in sorted(headers)],link_libraries=[str(p) for p in sorted(libraries)],
            exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr,seconds=time.perf_counter()-tick,
            python_version=sys.version,implementation=platform.python_implementation(),extension_suffix=sysconfig.get_config_var('EXT_SUFFIX'),
            scope='Equivalent preparation and integer bookkeeping, not geometry/JSON/hash/predicate/acceptance changes.'))
        if result.returncode:raise ValueError('PreparationCompileFailed')
        print(record,flush=True)

if __name__=='__main__':main()
