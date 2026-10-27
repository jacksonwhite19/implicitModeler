"""Close retained writer controls; retire only exact private duplicate payloads."""
from contextlib import ExitStack
import json
from pathlib import Path
import time

import immutable_state_access as access
from check_current_front_encoding import allocation
from collect_capsule_critical_speed import held_derivative
from path_preserving_deduplication import durable_json, file_hash, metadata
from record_receipt_resume import disk,status

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
OUT=V4/'results/assembly_row_efficiency'
OWN={str(V4/p) for p in ('CURRENT_TASK.md','README.md','ASSEMBLY_ROW_EFFICIENCY_REPORT.md',
    'tools/private_quality_codec_reuse.py','tools/assembly_quality_handoff.py','tools/private_quality_hash_stream.py',
    'tools/profile_assembly_rows.py','tools/check_capsule_hash_stream.py','tools/collect_assembly_row_efficiency.py',
    'tests/test_private_quality_hash_stream.py','tests/test_assembly_row_profile.py','tests/test_assembly_row_collection.py')}


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def validate_runtime(runtime,allowed):
    changes=[p for p,h in runtime.items() if file_hash(p)!=h]
    if set(changes)!=allowed:raise ValueError('UnexpectedRuntimeAssemblyChange',changes)
    return changes


def derivative_record(envelope,keepers,held):
    held.enter_context(access.guard(envelope))
    body=read(envelope);payload=Path(body['payload']['path']).resolve();digest=body['payload']['sha256']
    if (not payload.is_relative_to(OUT) or payload.parent!=envelope.parent/'quality_objects'
        or payload in keepers.values() or digest not in keepers):
        raise ValueError('UnsafePrivateAssemblyRetirement')
    # The handle excludes replacement and content changes before identity is
    # checked. A byte-identical but different pathname occupant is NOT accepted.
    retire=held.enter_context(held_derivative(payload,digest))
    if access.token(payload)!=body['payload']['token']:raise ValueError('ChangedPrivateAssemblyToken')
    record=dict(path=str(payload),sha256=digest,metadata=metadata(payload),envelope=str(envelope),
        envelope_sha256=file_hash(envelope),equivalent_retained_source=str(keepers[digest]),
        active_ancestry=False,recovery='Existing writer reconstructs exact bytes at a NEW path; old file tokens are never recreated.')
    return retire,record


def validate_outcome(comparison,before,after,source,envelopes):
    if not all(comparison.get(k) is True for k in ('exact_logical_hashes','exact_database_bytes')):
        raise ValueError('IncompleteAssemblyWriterControl')
    expected=[dict(seconds=r['seconds'],result=r['result']) for r in (before,after)]
    if comparison.get('measurements')!=expected:raise ValueError('DetachedAssemblyComparison')
    if source['selected_ordinals']!=dict(start=0,step=1,count=source['source_rows']):
        raise ValueError('IncompleteAssemblySourceSelection')
    fields=('identity','sha256','bytes','count')
    for r,envelope,reuse in zip((before,after),envelopes,(False,True),strict=True):
        if r['result']['count']!=source['source_rows'] or r['result']['sha256']!=source['logical']:
            raise ValueError('DifferentAssemblyWriterPopulation')
        if (r['workers']!=4 or r['process'] is not True or r['reuse'] is not reuse
            or r['result']['count']!=envelope['logical']['count']
            or any(r['result'][k]!=envelope['logical'][k] for k in fields)
            or r['payload_sha256']!=envelope['payload']['sha256']):
            raise ValueError('DetachedAssemblyWriterOutcome')
    if any(before['result'][k]!=after['result'][k] for k in fields) or before['payload_sha256']!=after['payload_sha256']:
        raise ValueError('DifferentAssemblyLogicalRows')


def collect():
    if (OUT/'CLOSEOUT.json').exists():raise ValueError('PreserveExistingAssemblyCloseout')
    initial=read(OUT/'INITIAL.json');control=OUT/'full_verified'
    comparison=read(control/'COMPARISON.json');before=read(control/'before.json');after=read(control/'after.json')
    source=read(control/'INPUT.json');review=read(OUT/'SOURCE_REVIEW.json')
    outcome=read(OUT/'OUTCOME_REVIEW.json')
    if (outcome.get('passed') is not True or not outcome.get('reviewer')
        or outcome.get('collector_sha256')!=file_hash(__file__)
        or any(file_hash(p)!=h for p,h in outcome['evidence'].items())):
        raise ValueError('IndependentAssemblyOutcomeReviewRequired')
    validate_outcome(comparison,before,after,source,[read(control/n/'QUALITY_DELTA.json') for n in ('before','after')])
    if not review['passed'] or any(file_hash(p)!=h for p,h in review['sources'].items()):
        raise ValueError('ChangedAssemblySourceReview')
    if file_hash(V4/'tools/profile_assembly_rows.py')!=source['implementation_sha256']:
        raise ValueError('ChangedDiagnosticImplementation')
    changed=[p for p,h in initial['protected'].items() if file_hash(p)!=h]
    allowed={str(V4/'tools'/n) for n in ('private_quality_codec_reuse.py','assembly_quality_handoff.py')}
    if set(changed)!=allowed:raise ValueError('UnexpectedProtectedAssemblyChange',changed)
    def owned(entry):
        path=(ROOT/entry[3:]).resolve()
        return str(path) in OWN or path.is_relative_to(OUT)
    current=status()
    if [r for r in current if not owned(r)]!=initial['status']:
        raise ValueError('ChangedUnrelatedAssemblyStatus')
    files=[p for p in OUT.rglob('*') if p.is_file()];allocated_before=allocation(files)
    keep=OUT/'baseline/before/QUALITY_DELTA.json'
    original_envelope=Path(next(p for p in source['references'] if p.endswith('QUALITY_DELTA.json')))
    originals=[original_envelope,keep];keepers={};retirements=[];records=[]
    old_request=read(V4/'results/generic_critical_production/REVIEW_REQUEST.json')
    with ExitStack() as held:
        # Keep the verified source closure frozen across retirement, not just
        # check it after files have already been deleted.
        for path in old_request['runtime']:held.enter_context(access.guard(path))
        runtime_changes=validate_runtime(old_request['runtime'],allowed)
        for path in originals:
            held.enter_context(access.guard(path));body=read(path)
            payload=Path(body['payload']['path']);held.enter_context(access.guard(payload))
            if file_hash(payload)!=body['payload']['sha256'] or access.token(payload)!=body['payload']['token']:
                raise ValueError('ChangedRetainedAssemblyPayload')
            keepers[body['payload']['sha256']]=payload
        for path,record in source['references'].items():
            if access.token(path)!=record['token'] or file_hash(path)!=record['sha256']:
                raise ValueError('ChangedRetainedAssemblyInput')
        for envelope in sorted(OUT.rglob('QUALITY_DELTA.json')):
            if envelope==keep:continue
            retire,record=derivative_record(envelope,keepers,held)
            retirements.append(retire);records.append(record)
        durable_json(OUT/'DERIVATIVE_MANIFEST.json',dict(scope='Unpublished private writer diagnostics only',
            originals={str(p):file_hash(p) for p in originals},derivatives=records,
            inputs=source['references'],comparison_sha256=file_hash(control/'COMPARISON.json')))
        for retire in retirements:retire()
        for path,record in source['references'].items():
            if access.token(path)!=record['token'] or file_hash(path)!=record['sha256']:
                raise ValueError('RetainedAssemblyInputChangedDuringCleanup')
        validate_runtime(old_request['runtime'],allowed)
    allocated_after=allocation([p for p in OUT.rglob('*') if p.is_file()])
    result=dict(classification='V4PrivateAssemblyHashStreamRetainedRowsVerified',
        scope='690963 retained quality rows; exact writer/cold-reader bytes; no fresh front or whole-layer timing',
        starting_head=initial['head'],branch=initial['branch'],seconds=time.time()-initial['start_time'],
        before_seconds=before['seconds'],after_seconds=after['seconds'],rows=source['source_rows'],
        payload_sha256=before['payload_sha256'],logical=before['result'],
        memory_before=read(control/'before_MEMORY.json'),memory_after=read(control/'after_MEMORY.json'),
        preserved_status_entries=len(initial['status']),protected_files=len(initial['protected']),
        changed_protected=changed,verified_previous_runtime_files=len(old_request['runtime']),runtime_changes=runtime_changes,
        disk_initial=initial['disk'],disk_final=disk(),
        before_allocation={k:v for k,v in allocated_before.items() if k!='records'},
        after_allocation={k:v for k,v in allocated_after.items() if k!='records'},
        reclaimed_logical_bytes=sum(r['metadata']['bytes'] for r in records),
        reclaimed_allocation_bytes=sum(r['metadata']['cluster_bytes'] for r in records),
        no_front_generated=True,no_publication=True,canonical_L8='e64b53451463bfe764fd2fb5726b5779658419def6eaf37bb2c63de4605be599',
        sources={str(Path(p)):file_hash(p) for p in sorted(OWN) if Path(p).is_file()})
    durable_json(OUT/'CLOSEOUT.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('sources','changed_protected','logical','runtime_changes')},indent=2))


if __name__=='__main__':collect()
