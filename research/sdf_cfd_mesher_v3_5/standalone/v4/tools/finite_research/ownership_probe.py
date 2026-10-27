from pathlib import Path
from types import SimpleNamespace
from contextlib import ExitStack
import inspect,json,os,secrets,sys,time

_pending=None
_failed=None

def execute(args):
    global _pending,_failed
    if _pending is not None:
        raise ValueError('OneSyntheticContinuationRequired')
    from finite_research import inputs,process_entry,owner_selection,direct_publication
    from immutable_state_access import guard
    invocation=process_entry.current_invocation()
    if invocation is None or not invocation.get('context_path'):
        raise ValueError('RealOwnedProbeProcessContextRequired')
    context=inputs.read(invocation['context_path'])
    if inputs.sha(args.probe_spec)!=args.probe_spec_sha256:
        raise ValueError('ChangedOwnedProbeSpecification')
    spec=inputs.read(args.probe_spec)
    folder=Path(args.output).resolve()
    if (spec.get('schema')!='Task162NongeneratingEntryProbe/1' or spec.get('scientific_authority') is not False
        or spec.get('output')!=str(folder) or spec.get('fault') not in ('none','precommit','postcommit','ordinary_nonzero')
        or folder.exists() or folder.parent!=Path(context['evidence_dir']).parent):
        raise ValueError('ExactNongeneratingProbeRequired')
    manifest=inputs.read(context['manifest'])
    if manifest.get(str(Path(args.probe_spec).resolve()))!=args.probe_spec_sha256:
        raise ValueError('UnownedProbeSpecification')
    folder.mkdir()
    with guard(args.probe_spec),inputs.source_scope(args.source_manifest_sha256) as (dependency,pins):
        with owner_selection.selected_native(pins) as (native,attest),ExitStack() as held:
            owner=Path(native.__file__).parent
            resource_limits=sys.modules.get('resource_limits')
            if resource_limits is None or Path(resource_limits.__file__).resolve()!=owner/'resource_profile_16gib_3600/resource_limits.py':
                raise ValueError('AttestedSelectedProbeResourceOwnerRequired')
            def check():
                if time.monotonic()>=min(context['deadline'],context['parent_deadline_monotonic']):
                    raise TimeoutError('InheritedProbeDeadline')
            check()
            dependency.supervisor.HERE=folder
            def bounded(label,command,seconds,pins):
                available=min(seconds,context['deadline']-time.monotonic())
                if available<=0:
                    raise TimeoutError('ProbePayloadDeadline')
                return dependency.supervisor.bounded(label,command,available,pins)
            supervisor=SimpleNamespace(HERE=folder,sha=dependency.supervisor.sha,
                publish=dependency.supervisor.publish,bounded=bounded)
            seconds=min(180.,context['parent_deadline_monotonic']-time.monotonic())
            command=[sys.executable,'-B',str(owner/'probe_child.py')]
            result=native.bounded('workload',command,seconds,
                {Path(path):digest for path,digest in pins.items()},supervisor,resources=resource_limits.APPROVED)
            check()
            if result['returncode']!=0:
                raise ValueError('RealOwnedProbeWorkloadFailure')
            retirement=inputs.read(folder/'workload_RESOURCE_RESULT.json')
            if not retirement['passed'] or not retirement['linux_retired'] or not retirement['monitor_retired']:
                raise ValueError('RealOwnedProbeRetirementFailure')
            attest()
            if spec['fault']=='ordinary_nonzero':
                _failed=dict(invocation=invocation,folder=folder)
                return dict(passed=False,returncode=1,phase='WORKLOAD_CLOSED_BEFORE_PUBLICATION',
                    synthetic_workload=True,scientific_authority=False,normal_run_qualified=False)
            _pending=dict(invocation=invocation,folder=folder,spec=spec,context=context,pins=dict(pins))
            return dict(passed=True,phase='WORKLOAD_CLOSED_PUBLICATION_PENDING',synthetic_workload=True,
                scientific_authority=False,normal_run_qualified=False)

def failed_process(code,invocation):
    global _failed
    from finite_research import process_entry
    frame=inspect.currentframe().f_back
    if frame.f_code is not process_entry.execute.__code__ or frame.f_globals is not vars(process_entry):
        raise ValueError('ActualSyntheticFailedContinuationRequired')
    failed=_failed
    _failed=None
    if failed is None:
        return None
    if failed['invocation'] is not invocation or _pending is not None:
        raise ValueError('ChangedSyntheticFailedInvocation')
    return process_entry.failure_outcome(code,invocation,failed['folder'],synthetic=True)


def complete_process(code,invocation):
    global _pending
    from finite_research import process_entry,direct_publication
    from immutable_state_access import guard
    frame=inspect.currentframe().f_back
    if frame.f_code is not process_entry.execute.__code__ or frame.f_globals is not vars(process_entry):
        raise ValueError('ActualSyntheticTerminalContinuationRequired')
    pending=_pending
    _pending=None
    if pending is None:
        return None
    if pending['invocation'] is not invocation or code!=0 or not invocation['completed']:
        raise ValueError('FailedSyntheticOperationCannotPublish')
    folder=pending['folder']
    spec=pending['spec']
    context=pending['context']
    def check():
        if time.monotonic()>=min(context['deadline'],context['parent_deadline_monotonic']):
            raise TimeoutError('SyntheticPublicationInheritedDeadline')
    publication=None
    try:
        with ExitStack() as held:
            for path,digest in invocation['files'].items():
                held.enter_context(guard(path))
                if direct_publication.file_sha(path)!=digest:
                    raise ValueError('ChangedSyntheticOperationClosure')
            publication=publish_synthetic(folder,spec,check,pending['pins'])
        return publication
    except BaseException as error:
        if publication is not None:
            error.direct_publication_outcome=publication
        raise

def publish_synthetic(folder,spec,check,pins):
    from finite_research import direct_publication,process_entry,inputs
    previous=direct_publication.encoded(dict(schema='Task162SyntheticPrevious/1',scientific_authority=False))
    direct_publication.replace_bytes(folder/'CURRENT.json',previous)
    current=dict(schema='Task162SyntheticCurrent/1',publication_id=secrets.token_hex(32),
        scientific_authority=False,workload_retirement_sha256=inputs.sha(folder/'workload_RESOURCE_RESULT.json'))
    calls=[]
    def consume(value,held):
        if value!=current:
            raise ValueError('SyntheticReadbackMismatch')
        calls.append(True)
        if spec['fault']=='precommit' and len(calls)==2:
            raise RuntimeError('InjectedSyntheticPrecommitFailure')
        return dict(synthetic_workload=True,scientific_authority=False,publication_id=current['publication_id'])
    try:
        publication=direct_publication.transaction(folder,current,previous,consume,check,dependency_pins=pins)
    except RuntimeError as error:
        if spec['fault']!='precommit' or str(error)!='InjectedSyntheticPrecommitFailure':
            raise
        if direct_publication.snapshot(folder/'CURRENT.json')!=previous:
            raise ValueError('SyntheticPrecommitRestorationFailed')
        if direct_publication.committed(folder,inputs.read(folder/'DIRECT_PUBLICATION_PREPARED.json')):
            raise ValueError('SyntheticPrecommitFaultCommitted')
        publication=dict(publication_state='NOT_COMMITTED',expected_fault=str(error))
    record=dict(schema='Task162NongeneratingEntryResult/1',phase='TRANSACTION_ONLY_AWAITING_CONTROLLER_TERMINAL',publication=publication,
        synthetic_workload=True,scientific_authority=False,normal_run_qualified=False,
        supervisor_pid=os.getpid(),supervisor_still_live=True,
        workload_retired=True,controller_terminal_required=True,external_readback_required=True)
    try:
        process_entry.write(folder/'PROBE_RESULT.json',record)
        check()
    except BaseException as error:
        error.direct_publication_outcome=publication
        raise
    if spec['fault']=='postcommit':
        if direct_publication.file_sha(folder/'DIRECT_PUBLICATION_COMMITTED.json')!=publication['commit_sha256']:
            error=ValueError('ChangedSyntheticCommittedIdentity')
            error.direct_publication_outcome=publication
            raise error
        error=RuntimeError('InjectedSyntheticPostcommitControllerFailure')
        error.direct_publication_outcome=publication
        raise error
    return publication
