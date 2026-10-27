from contextlib import ExitStack
from pathlib import Path
import base64,hashlib,json,os,uuid
from immutable_state_access import guard
from incremental_transaction import writer
from path_preserving_deduplication import atomic_replace


def encoded(value):
    return json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()


def snapshot(pointer,read_guard=guard):
    if not pointer.exists():
        return None
    with read_guard(pointer):
        return pointer.read_bytes()


def replace_bytes(pointer,content,replace=atomic_replace):
    pending=pointer.parent/(".direct-"+uuid.uuid4().hex+".pending")
    primary=None
    try:
        with pending.open("xb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        replace(pending,pointer)
    except BaseException as error:
        primary=error
        raise
    finally:
        if pending.exists():
            try:
                pending.unlink()
            except BaseException as error:
                if primary is None:
                    raise
                primary.add_note("Direct owned temporary cleanup: "+repr(error))


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def restore_locked(root,prepared):
    pointer=root/'CURRENT.json'
    proposed=encoded(prepared['current'])
    previous=None if prepared['previous_base64'] is None else base64.b64decode(prepared['previous_base64'],validate=True)
    observed=snapshot(pointer)
    if observed==previous:
        return 'PREVIOUS_PRESERVED'
    if observed!=proposed:
        raise ValueError('DirectRollbackCollisionPreservedOtherPointer')
    if previous is None:
        pointer.unlink()
    else:
        replace_bytes(pointer,previous)
    if snapshot(pointer)!=previous:
        raise ValueError('DirectRollbackReadbackMismatch')
    return 'PREVIOUS_RESTORED'

def committed(root,prepared):
    path=root/'DIRECT_PUBLICATION_COMMITTED.json'
    if not path.exists():
        return False
    prepared_sha256=hashlib.sha256(encoded(prepared)).hexdigest()
    if (prepared.get('schema')!='DirectPublicationPrepared/1'
        or prepared.get('candidate_sha256')!=hashlib.sha256(encoded(prepared['current'])).hexdigest()
        or file_sha(root/'DIRECT_PUBLICATION_PREPARED.json')!=prepared_sha256):
        raise ValueError('ChangedDirectPreparedIdentity')
    record=json.loads(path.read_bytes())
    expected=dict(schema='DirectPublicationCommit/1',publication_id=prepared['current']['publication_id'],
        prepared_sha256=prepared_sha256,candidate_sha256=prepared['candidate_sha256'],
        consumed_sha256=prepared['consumed_sha256'],supervisor_terminal_required=True,
        external_readback_required=True,normal_run_qualified=False)
    if record!=expected:
        raise ValueError('IndeterminateDirectCommitRecord')
    return record

def recover(root,prepared_sha256,check):
    import psutil
    root=Path(root).resolve()
    outcome=None
    try:
        with writer(root):
            check()
            path=root/'DIRECT_PUBLICATION_PREPARED.json'
            if file_sha(path)!=prepared_sha256:
                raise ValueError('ChangedDirectRecoveryJournal')
            prepared=json.loads(path.read_bytes())
            controller=prepared['supervisor']
            try:
                candidate=psutil.Process(controller['pid'])
                if candidate.create_time()==controller['created']:
                    try:
                        candidate.wait(timeout=0)
                    except psutil.TimeoutExpired as error:
                        raise ValueError('LiveSupervisorRecoveryRejected') from error
            except psutil.NoSuchProcess:
                pass
            commit=committed(root,prepared)
            state='COMMITTED' if commit else restore_locked(root,prepared)
            outcome=dict(publication_state=state,commit=commit,prepared_sha256=prepared_sha256,
                normal_run_qualified=False,scientific_acceptance=False)
            check()
        return outcome
    except BaseException as error:
        error.direct_publication_outcome=outcome or dict(publication_state='UNRESOLVED',normal_run_qualified=False)
        raise

def transaction(root,current,previous,consume,check,*,dependency_pins=None,expectations_sha256=None):
    root=Path(root).resolve()
    pointer=root/"CURRENT.json"
    proposed=encoded(current)
    publication_id=current.get('publication_id')
    if (type(publication_id) is not str or len(publication_id)!=64
        or any(letter not in '0123456789abcdef' for letter in publication_id)):
        raise ValueError('UniqueDirectPublicationIdentityRequired')
    prepared=None
    commit=None
    recovery='NOT_NEEDED'
    try:
        with writer(root),ExitStack() as source_custody:
            for name,digest in (dependency_pins or {}).items():
                path=Path(name)
                source_custody.enter_context(guard(path))
                with path.open("rb") as stream:
                    if hashlib.file_digest(stream,"sha256").hexdigest()!=digest:
                        raise ValueError("ChangedDirectPublicationDependency")
            if snapshot(pointer)!=previous:
                raise ValueError("ChangedDirectPreviousPointer")
            if previous==proposed:
                raise ValueError("DirectPublicationReplay")
            try:
                with ExitStack() as held:
                    consumed=consume(current,held)
                    check()
                    if snapshot(pointer)!=previous:
                        raise ValueError("ChangedDirectPreviousPointer")
                    import psutil
                    from finite_research.process_entry import creation_filetime
                    prepared=dict(schema='DirectPublicationPrepared/1',current=current,
                        candidate_sha256=hashlib.sha256(proposed).hexdigest(),
                        consumed_sha256=hashlib.sha256(encoded(consumed)).hexdigest(),
                        previous_base64=None if previous is None else base64.b64encode(previous).decode('ascii'),
                        expectations_sha256=expectations_sha256,
                        supervisor=dict(pid=os.getpid(),created=psutil.Process().create_time(),creation_filetime=creation_filetime()))
                    path=root/'DIRECT_PUBLICATION_PREPARED.json'
                    if path.exists() or (root/'DIRECT_PUBLICATION_COMMITTED.json').exists():
                        raise ValueError('DirectPublicationAlreadyAttempted')
                    replace_bytes(path,encoded(prepared))
                    prepared_sha256=hashlib.sha256(encoded(prepared)).hexdigest()
                    if file_sha(path)!=prepared_sha256:
                        raise ValueError('ChangedInstalledDirectPreparation')
                    replace_bytes(pointer,proposed)
                    with guard(pointer):
                        if pointer.read_bytes()!=proposed:
                            raise ValueError("ChangedDirectPublishedPointer")
                        if consume(json.loads(pointer.read_bytes()),held)!=consumed:
                            raise ValueError("ChangedDirectConsumerReadback")
                        check()
                source_custody.close()
                check()
                if file_sha(path)!=prepared_sha256 or snapshot(pointer)!=proposed:
                    raise ValueError('ChangedDirectPrecommitIdentity')
                proposed_commit=dict(schema='DirectPublicationCommit/1',publication_id=publication_id,
                    prepared_sha256=prepared_sha256,candidate_sha256=hashlib.sha256(proposed).hexdigest(),
                    consumed_sha256=hashlib.sha256(encoded(consumed)).hexdigest(),
                    supervisor_terminal_required=True,external_readback_required=True,normal_run_qualified=False)
                replace_bytes(root/'DIRECT_PUBLICATION_COMMITTED.json',encoded(proposed_commit))
                commit=committed(root,prepared)
                if commit!=proposed_commit:
                    raise ValueError('ChangedDirectCommitReadback')
                check()
            except BaseException as error:
                try:
                    commit=None
                    commit=committed(root,prepared) if prepared is not None else False
                    if prepared is not None and not commit:
                        recovery=restore_locked(root,prepared)
                except BaseException as cleanup:
                    recovery='UNRESOLVED'
                    error.add_note('Direct recovery: '+repr(cleanup))
                raise
        check()
        result=dict(publication_state='COMMITTED',publication_id=publication_id,consumer=consumed,
            commit_sha256=file_sha(root/'DIRECT_PUBLICATION_COMMITTED.json'),normal_run_qualified=False,
            controller_state='AWAITING_EXTERNAL_TERMINAL',cold_readback_state='NOT_RUN')
        check()
        return result
    except BaseException as error:
        error.direct_publication_outcome=dict(publication_id=publication_id,
            publication_state='COMMITTED' if commit else ('UNRESOLVED' if recovery=='UNRESOLVED' else 'NOT_COMMITTED'),
            recovery=recovery,commit=commit,normal_run_qualified=False)
        failure=root/("DIRECT_PUBLICATION_FAILURE_"+uuid.uuid4().hex+".json")
        try:
            observed=snapshot(pointer)
            replace_bytes(failure,encoded(dict(passed=False,scientific_acceptance=False,error=repr(error),
                notes=getattr(error,'__notes__',[]),recovery=recovery,
                publication_state='COMMITTED' if commit else ('UNRESOLVED' if recovery=='UNRESOLVED' else 'NOT_COMMITTED'),
                observed_pointer_sha256=None if observed is None else hashlib.sha256(observed).hexdigest(),
                candidate_sha256=hashlib.sha256(proposed).hexdigest())))
        except BaseException as cleanup:
            error.add_note('Direct failure evidence: '+repr(cleanup))
        raise


def publish(root,admission,check):
    from finite_research.direct_admission import require_admission
    from finite_research.direct_consumer import consume
    current,expected,previous=require_admission(admission,root)
    path=Path(root)/'DIRECT_PUBLICATION_EXPECTATIONS.json'
    source=Path(expected['request_root'])/'live01_INPUT_PINS.json'
    reference=dict(path=str(source),sha256=file_sha(source))
    reduced={key:value for key,value in expected.items() if key!='source_pins'}
    reduced['source_pins_reference']=reference
    if path.exists():
        raise ValueError('DirectPublicationExpectationsAlreadyExist')
    replace_bytes(path,encoded(reduced))
    return transaction(root,current,previous,
        lambda value,held:consume(root,value,expected,check,custody=held),check,
        dependency_pins=expected["source_pins"],expectations_sha256=file_sha(path))


def readback(root,expected,check):
    from immutable_state_access import guard
    from finite_research.direct_consumer import consume
    root=Path(root).resolve()
    with ExitStack() as held:
        for name in ('DIRECT_PUBLICATION_PREPARED.json','DIRECT_PUBLICATION_COMMITTED.json'):
            held.enter_context(guard(root/name))
        prepared=json.loads((root/'DIRECT_PUBLICATION_PREPARED.json').read_bytes())
        if not committed(root,prepared) or prepared['current']!=expected['descriptor']:
            raise ValueError('CommittedDirectReadbackRequired')
        held.enter_context(guard(root/"CURRENT.json"))
        current=json.loads((root/"CURRENT.json").read_bytes())
        return consume(root,current,expected,check,custody=held)
