"""Owned native import identity; fixed installed interpreter, no installer or meshing."""
from pathlib import Path
import sys,json,hashlib,importlib
import stage_observations as obs
HERE=Path(__file__).resolve().parent
V4=HERE.parents[2]
PACKAGES=V4/'runtime/lib/finite_native_packages'
MODULES=('numpy','scipy','sympy','gmsh','core_backend','background_backend_method',
         'standalone_background','capsule_sizing_control','incremental_layer_validation',
         'tet_layer_adapter','shape_transition_topology')
# Hash work is local to each call. No digest, path, or authentication reuse.
_hash_measurements=[]

def _hash_stream(path, cancel=None):
    h=hashlib.sha256();size=0
    with Path(path).open('rb') as stream:
        while True:
            if cancel is not None and cancel.is_set():
                raise RuntimeError('NativeVerificationCancelled')
            block=stream.read(1048576)
            if not block:break
            h.update(block);size+=len(block)
    return h.hexdigest(),size

def sha(path):
    return _hash_stream(path)[0]

def hash_measurements():
    """Bounded scalar diagnostics only; never consumed as authentication."""
    return tuple(dict(row) for row in _hash_measurements)

def _verify_files(rows, error):
    """Same ordered manifest checks; at most four pending independent reads.

    Both pre-import and post-import passes remain. The existing process owner
    retains the aggregate deadline and memory cap. Cancellation stops pending
    reads after a failure; shutdown joins every worker before returning/raising.
    """
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event,get_ident
    from time import perf_counter
    started=perf_counter();cancel=Event();seen=set();count=0;size=0;failure=None
    pending=[];retired=False
    pool=ThreadPoolExecutor(max_workers=4,thread_name_prefix='finite-native-hash')
    def one(row):
        if cancel.is_set():raise RuntimeError('NativeVerificationCancelled')
        path=resolve(row);actual,length=_hash_stream(path,cancel)
        if actual!=row['sha256']:raise ValueError(error,str(path))
        return length,get_ident()
    try:
        for offset in range(0,len(rows),4):
            pending=[]
            for row in rows[offset:offset+4]:pending.append(pool.submit(one,row))
            # Manifest order determines the primary failure, not completion order.
            for future in pending:
                length,worker=future.result();count+=1;size+=length;seen.add(worker)
    except BaseException as caught:
        failure=caught;cancel.set()
        for future in pending:future.cancel()
        raise
    finally:
        cleanup=None
        try:
            pool.shutdown(wait=True,cancel_futures=True);retired=True
        except BaseException as caught:cleanup=caught
        try:
            obs.units('files_verified',count);obs.units('hash_bytes',size)
            _hash_measurements.append(dict(pass_kind=error,files_verified=count,
                verified_bytes=size,workers_configured=4,workers_used=len(seen),
                maximum_pending=4,seconds=perf_counter()-started,
                failed=None if failure is None else type(failure).__name__,
                workers_retired=retired,reuse=False))
            del _hash_measurements[:-16]
        except BaseException as caught:
            if cleanup is None:cleanup=caught
            else:cleanup.__dict__.setdefault('secondary_notes',[]).append(repr(caught))
        if cleanup is not None:
            if failure is None:raise cleanup
            failure.__dict__.setdefault('secondary_notes',[]).append('Native hash cleanup/measurement: '+repr(cleanup))
def resolve(row):
    if 'package_relative' in row:
        path=(V4/row['package_relative']).resolve()
        if V4 not in path.parents:raise ValueError('NativeBindingOutsidePackage')
        return path
    return Path(row['external_path'])
@obs.observed('native.binding','source_lock')
def binding():
    value=json.loads((V4/'runtime/FINITE_NATIVE_BINDING.json').read_text(encoding='utf8'))
    if value.get('schema')!='OwnedFiniteNativeBinding/1':raise ValueError('InvalidOwnedNativeBinding')
    return value

def imports():
    sys.path[:0]=[str(PACKAGES),str(V4/'tools'),str(V4/'src'),str(V4/'runtime/lib/gmsh')]
    for name in MODULES:
        with obs.span('module.'+name,'import'):
            importlib.import_module(name)
    import core_backend
    from background_backend_method import HXT
    with obs.span('imports.core_locks','source_lock'):
        locks=core_backend.locks(HXT)
    with obs.span('imports.sealed_record','source_lock'):
        lock=binding()['lock_reference'];path=resolve(lock)
        if sha(path)!=lock['sha256']:raise ValueError('ChangedSealedRuntimeLock')
        prior=json.loads(path.read_text(encoding='utf8'))
    validate_current_lock(locks,prior['locks'])
    return locks

@obs.observed('native.verify','source_lock')
def verify():
    with obs.span('binding_and_interpreter','source_lock'):
        value=binding()
        if str(Path(sys.executable).resolve())!=value['executable']:
            raise ValueError('ChangedNativeRuntimeInterpreter')
    with obs.span('pre_hash','io_hash'):
        _verify_files(value['files'],'ChangedNativeRuntimeFile')
    with obs.span('imports_and_locks','import_and_lock'):
        imports()
    with obs.span('module_identity','source_lock'):
        observed={name:dict(path=str(Path(sys.modules[name].__file__).resolve()),
            version=getattr(sys.modules[name],'__version__',None)) for name in MODULES+('psutil',)}
        expected={name:dict(path=str(resolve(row)),version=row['version']) for name,row in value['modules'].items()}
        if observed!=expected:raise ValueError('ChangedResolvedNativeModules',observed,expected)
    with obs.span('post_hash','io_hash'):
        _verify_files(value['files'],'ChangedNativeRuntimeAfterImports')
    return value


@obs.observed('native.current_lock_validation','source_lock')
def validate_current_lock(actual,original=None):
    import copy
    value=json.loads((V4/'runtime/FINITE_LOCK_TRANSITION.json').read_text(encoding='utf8'))
    if value.get('schema')!='OwnedNativeLockTransition/1':raise ValueError('InvalidCurrentNativeLockTransition')
    lock=binding()['lock_reference']
    if value['original_record']!=lock or sha(resolve(lock))!=lock['sha256']:raise ValueError('ChangedHistoricalLockTransition')
    historical=json.loads(resolve(lock).read_text(encoding='utf8'))['locks']
    if value['original_lock']!=historical or (original is not None and original!=historical):raise ValueError('ChangedHistoricalNumericalLock')
    required=[(['sizing', 'wrapper_sha256'], 'tools/background_sizing.py'), (['backend_method', 'source_pins', 'tools/background_sizing.py'], 'tools/background_sizing.py'), (['backend_method', 'source_pins', 'tools/background_query_build.py'], 'tools/background_query_build.py'), (['files', 'backend'], 'tools/core_backend.py'), (['files', 'capsule_helpers'], 'tools/capsule_sizing_control.py'), (['backend_method', 'source_pins', 'tools/core_backend.py'], 'tools/core_backend.py'), (['backend_method', 'source_pins', 'tools/background_backend_method.py'], 'tools/background_backend_method.py')]
    if [(r['keys'],r['owned_source']) for r in value['changes']]!=required:raise ValueError('UnreviewedNativeLockDelta')
    expected=copy.deepcopy(historical)
    for row in value['changes']:
        node=expected
        for key in row['keys'][:-1]:node=node[key]
        key=row['keys'][-1]
        if node[key]!=row['original_sha256'] or sha(V4/row['owned_source'])!=row['current_sha256']:raise ValueError('ChangedReviewedNativeWrapper')
        node[key]=row['current_sha256']
    if (sha(V4/'runtime/BUILD_RELOCATION.json')!=value['build_relocation_sha256']
            or sha(V4/'tools/build_provenance.py')!=value['build_provenance_sha256']):raise ValueError('ChangedReviewedBuildProvenance')
    if expected!=value['expected_current_lock'] or actual!=expected:raise ValueError('ChangedExistingNumericalRuntime')
    return expected
