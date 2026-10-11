"""Close out bounded speed evidence; no front/stage/publication executor."""
from contextlib import ExitStack
from pathlib import Path
from contextlib import contextmanager
import ctypes
import sys
import time
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from check_capsule_critical_speed import OUT, INPUT, read, unrelated
from check_current_front_encoding import allocation
from path_preserving_deduplication import durable_json,file_hash,metadata,api,W,handle_hash
from record_receipt_resume import disk
import immutable_state_access as access


@contextmanager
def held_derivative(path,expected):
    """Only task-owned duplicate retirement; keep its verified file identity.

    READ|DELETE with SHARE_READ denies content writes and path replacement.
    FileDispositionInfo retires that handle, never a later pathname occupant.
    Merely leaving this context without retire() preserves the file.
    """
    k=api();h=k.CreateFileW(str(Path(path).resolve()),0x80010000,1,None,3,0,None)
    if h==W.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    try:
        if handle_hash(h)!=expected:raise ValueError('BenchmarkDerivativeNotByteIdentical')
        def retire():
            disposition=ctypes.c_ubyte(1)
            k.SetFileInformationByHandle.argtypes=[W.HANDLE,ctypes.c_int,ctypes.c_void_p,W.DWORD]
            if not k.SetFileInformationByHandle(h,4,ctypes.byref(disposition),ctypes.sizeof(disposition)):
                raise ctypes.WinError(ctypes.get_last_error())
        yield retire
    finally:k.CloseHandle(h)


def collect():
    baseline=read(OUT/'BASELINE.json')
    owned={'research/sdf_cfd_mesher_v4/tools/collect_capsule_critical_speed.py',
           'research/sdf_cfd_mesher_v4/tests/test_critical_speed_collection.py'}
    current=[r for r in unrelated() if r[3:] not in owned]
    if current!=baseline['status']:raise ValueError('ChangedUnrelatedCriticalSpeedStatus')
    if any(file_hash(p)!=h for p,h in baseline['protected'].items()):
        raise ValueError('ChangedProtectedCriticalSpeedInput')
    original=read(OUT/'QUALITY_BOUNDARY.json');private=read(OUT/'QUALITY_PRIVATE.json')
    if any(not r['equivalence'][k] for r in (original,private) for k in
           ('all_rows_exact','logical_identity_exact','readers_exact')):
        raise ValueError('IncompleteQualityBoundaryEquivalence')
    if original['quality']!=private['quality'] or original['rows']!=private['rows']:
        raise ValueError('DifferentQualityBenchmarkPopulation')
    source_envelope=INPUT.parent/'QUALITY_DELTA.json'
    original_source=Path(read(source_envelope)['payload']['path']).resolve()
    inputs=private['inputs']
    before=allocation([p for p in OUT.rglob('*') if p.is_file()])
    derivatives=[];retirements=[]
    with ExitStack() as held:
        for p,h in inputs.items():
            held.enter_context(access.guard(p))
            if file_hash(p)!=h:raise ValueError('ChangedRetainedBenchmarkInput',p)
        expected=file_hash(original_source)
        for folder in ('quality_boundary','quality_private'):
            envelope=OUT/folder/'QUALITY_DELTA.json';body=read(envelope)
            p=Path(body['payload']['path']).resolve()
            if not p.is_relative_to(OUT.resolve()) or p.parent!=(OUT/folder/'quality_objects').resolve():
                raise ValueError('BenchmarkDerivativeOutsideOwnedPath')
            if p.name!=expected+'.sqlite' or body['payload']['sha256']!=expected:
                raise ValueError('BenchmarkDerivativeNotByteIdentical')
            if p==original_source:raise ValueError('ProtectedOriginalIsNotDerivative')
            retirements.append(held.enter_context(held_derivative(p,expected)))
            derivatives.append(dict(path=str(p),sha256=expected,metadata=metadata(p),
                equivalent_retained_source=str(original_source),envelope=str(envelope),
                envelope_sha256=file_hash(envelope),active_ancestry=False,
                recovery='Reconstruct at a NEW path from guarded retained quality rows using the existing writer. Old tokens are never recreated.'))
        manifest=dict(scope='Completed unsealed writer diagnostics only; no candidate/authority was published',
            derivatives=derivatives,inputs=inputs,
            results={str(OUT/n):file_hash(OUT/n) for n in ('QUALITY_BOUNDARY.json','QUALITY_PRIVATE.json','CODEC.json')})
        durable_json(OUT/'DERIVATIVE_MANIFEST.json',manifest)
        # Only freshly generated, proven byte-identical private derivatives.
        # Original authorities and their file tokens stay untouched.
        for retire in retirements:retire()
        if any(file_hash(p)!=h for p,h in inputs.items()):raise ValueError('RetainedInputChangedDuringCleanup')
    after=allocation([p for p in OUT.rglob('*') if p.is_file()])
    result=dict(passed=True,classification='V4CapsuleCriticalProductionReceiptReuseVerified',
        last_qualified_layer=8,last_scientific='e64b53451463bfe764fd2fb5726b5779658419def6eaf37bb2c63de4605be599',
        scope='Actual retained L7 writer/reader performance and shared dispatch; no fresh whole-layer timing or 280 s claim',
        baseline_head=baseline['head'],status_entries_preserved=len(current),protected_files_preserved=len(baseline['protected']),
        seconds=time.time()-baseline['start'],disk_initial=baseline['disk'],disk_final=disk(),
        before={k:v for k,v in before.items() if k!='records'},after={k:v for k,v in after.items() if k!='records'},
        reclaimed_logical_bytes=sum(r['metadata']['bytes'] for r in derivatives),
        reclaimed_allocated_bytes=sum(r['metadata']['cluster_bytes'] for r in derivatives),
        codec=read(OUT/'CODEC.json')['phases'],quality_original=original['phases'],quality_private=private['phases'],
        peak_working_set=max(original['peak_working_set'],private['peak_working_set']),
        preserved_inputs=inputs,
        next='Measure one authorized L6->L7 normal critical production transition with reuse_validated_assembly=True, separating bootstrap/replay and retaining required gates.')
    durable_json(OUT/'CLOSEOUT.json',result)
    print({k:v for k,v in result.items() if k not in ('preserved_inputs','codec')})


if __name__=='__main__':collect()
