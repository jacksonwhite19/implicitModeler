"""Explicit host-epoch recovery. Never edits sealed scientific objects.

Recovery verifies pinned root bytes and every immutable file under deny-write
handles. New private metadata changes only epoch, tokens and parent access
references. Legacy mode verifies the complete canonical scientific digest;
explicit component mode verifies pinned sealed production roots and acceptance.
The existing recursive reader and its epoch checks are unchanged.
"""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import hashlib
import uuid
import immutable_state_access as access
from incremental_transaction import Store, writer
from composable_state import CommittedState, SCHEMA
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity

RECOVERY='V4RecursiveEpochRecovery/1'
COMPONENT_RECOVERY='V4RecursiveComponentEpochRecovery/1'
# Existing composable_publication.publish acceptance evidence, unchanged gates.
STAGE_EVIDENCE=frozenset(('ADMISSION.json','GENERATION.json','ASSEMBLY.json','VOLUME_DELTA.json','FRESH_CERTIFICATE.json'))


def content_pin(value):
    return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value)


def verified_record(record):
    if record.get('schema')==RECOVERY:
        return record.get('deep_verified') is True
    if record.get('schema')==COMPONENT_RECOVERY:
        basis=record.get('verification_basis',{})
        return (record.get('components_verified') is True and record.get('deep_verified') is False
            and content_pin(record.get('content')) and content_pin(record.get('source',{}).get('root'))
            and record.get('scientific')==record.get('content')
            and basis.get('source')==record.get('source')
            and basis.get('content')==record.get('content')
            and basis.get('legacy_digest_checked') is False)
    return False


def component_source(old,kind):
    """Validate the existing sealed production plan, not a new scientific gate."""
    if kind=='native':return
    p=old['plan'];a=old.get('acceptance')
    if p.get('scope')!='production' or a is None or a.get('passed') is not True:
        raise ValueError('SealedProductionAcceptanceRequired')
    if p['components']!=p['content']['components']:
        raise ValueError('ConflictingSealedComponents')
    if p['implementation']!=file_hash(__import__('composable_state').__file__):
        raise ValueError('IncompatibleRecoveryImplementation')
    refs={**p['references'],**old['produced']}
    if not STAGE_EVIDENCE.issubset(a['evidence']):raise ValueError('IncompleteSealedStageEvidence')
    if any(str(Path(old[name]).resolve()) not in refs for name in ('quality_delta','binding_patch')):
        raise ValueError('UndeclaredSealedColdComponent')
    for name,h in a['evidence'].items():
        path=str((Path(p['output'])/name).resolve())
        if refs.get(path,{}).get('sha256')!=h:raise ValueError('UnboundSealedAcceptanceEvidence')


def restore_tokens(new,old):
    if set(new)!=set(old):raise ValueError('ChangedRecoveryReferencePaths')
    restored={}
    for path,record in new.items():
        row=deepcopy(record);row['token']=old[path]['token']
        if row!=old[path]:raise ValueError('ChangedRecoveryReferenceIdentity')
        restored[path]=row
    return restored


def verify_components(state,expected_content,originals):
    """Same sealed content, expected state and acceptance under refreshed access.

Every referenced file has already been SHA checked under held guards. This is
component-root authentication, not legacy canonical digest recomputation.
"""
    if not originals:raise ValueError('IncompleteRecoveredAncestry')
    if state.content_root!=expected_content:raise ValueError('WrongScientificContentRoot')
    node=state
    for old in reversed(originals):
        kind='native' if old['schema']==access.SCHEMA else 'recursive'
        if node.kind!=kind:raise ValueError('RecoveryAncestryKindMismatch')
        new=node.wrapped.body if kind=='native' else node.body
        # Independent field allowlist. Restoring only access metadata must yield
        # the original root body exactly, including every scientific selector.
        restored=deepcopy(new);restored['epoch']=old['epoch']
        e=restored.pop('epoch_recovery')
        if (e['source_root']!=identity(old) or e['original_acceptance']!=old.get('acceptance')
            or e['original_acceptance_identity']!=old.get('acceptance_identity')):
            raise ValueError('ChangedOriginalAcceptance')
        if 'epoch_recovery' in old:restored['epoch_recovery']=old['epoch_recovery']
        if kind=='native':restored['files']=restore_tokens(new['files'],old['files'])
        else:
            restored['plan']['parent']=old['plan']['parent']
            restored['plan']['references']=restore_tokens(new['plan']['references'],old['plan']['references'])
            restored['produced']=restore_tokens(new['produced'],old['produced'])
            if identity(new['acceptance'])!=new['acceptance_identity'] or new['acceptance']['plan']!=identity(new['plan']):
                raise ValueError('ChangedRecoveredAcceptanceIdentity')
            restored['acceptance']['plan']=old['acceptance']['plan']
            if restored['acceptance']!=old['acceptance']:raise ValueError('ChangedRecoveredAcceptance')
            restored['acceptance_identity']=old['acceptance_identity']
        if restored!=old:raise ValueError('NonAccessRecoveryChange')
        if kind=='native':
            if new['content']!=old['content'] or new['components']!=old['components']:
                raise ValueError('ChangedRecoveredComponents')
        else:
            if (new['plan']['content']!=old['plan']['content'] or new['scientific']!=old['scientific']
                or new['quality_delta']!=old['quality_delta'] or new['binding_patch']!=old['binding_patch']
                or new['epoch_recovery']['original_acceptance']!=old['acceptance']):
                raise ValueError('ChangedRecoveredComponents')
        old_expected=old['expected'] if kind=='native' else old['plan']['expected']
        if node.expected!=old_expected:raise ValueError('ChangedRecoveredExpectedState')
        node.check()
        if kind=='recursive':node=node.parent
    if kind!='native':raise ValueError('IncompleteRecoveredAncestry')
    return expected_content

def sealed_body(reference, held):
    directory=Path(reference['directory']); root=reference['root']
    pointer=directory/'CURRENT.json'; held.enter_context(access.guard(pointer))
    p=access.read(pointer)
    if p['root']!=root or p['file']!='ROOT_'+root+'.json':raise ValueError('StaleRecoveryBase')
    path=directory/p['file'];held.enter_context(access.guard(path))
    if file_hash(path)!=p['sha256']:raise ValueError('CorruptSealedRoot')
    body=access.read(path)
    if identity(body)!=root:raise ValueError('WrongSealedRoot')
    return body

def refresh_records(records, held, authenticated):
    result={}
    for path,record in records.items():
        if path in authenticated:
            if authenticated[path]['sha256']!=record['sha256']:raise ValueError('ConflictingImmutableIdentity')
        else:
            held.enter_context(access.guard(path))
            authenticated[path]=access.attest(path,record['sha256'])
        result[path]=authenticated[path]
    return result

def rebind_body(old, kind, parent, records, produced, current_epoch):
    """Strict allowlist of access-only changes. Scientific content stays exact."""
    body=deepcopy(old);body['epoch']=current_epoch
    if kind=='native':body['files']=records
    elif kind=='recursive':
        p=body['plan'];p['parent']=parent;p['references']=records;body['produced']=produced
        if 'acceptance' in body:
            # Rebind the same acceptance to metadata-only plan differences.
            # Original acceptance is preserved verbatim in the recovery record.
            body['acceptance']['plan']=identity(p)
            body['acceptance_identity']=identity(body['acceptance'])
    else:raise ValueError('UnsupportedRecoveryKind')
    # The recovery statement is host access evidence, not scientific evidence.
    body['epoch_recovery']=dict(schema=RECOVERY,source_root=identity(old),
        old_epoch=old['epoch'],implementation=file_hash(__file__),
        original_acceptance=old.get('acceptance'),original_acceptance_identity=old.get('acceptance_identity'))
    return body

def discover(directory, expected=None):
    directory=Path(directory);p=access.read(directory/'CURRENT.json')
    if expected is not None and p['root']!=expected:raise ValueError('StaleRecoveryWriter')
    if p['file']!='RECOVERY_'+p['root']+'.json':raise ValueError('WrongRecoveryPointer')
    path=directory/p['file']
    if file_hash(path)!=p['sha256']:raise ValueError('CorruptRecoveryRecord')
    record=access.read(path)
    if identity(record)!=p['root'] or not verified_record(record):raise ValueError('IncompleteRecovery')
    if record['epoch']!=access.epoch():raise ValueError('RecoveryEpochChanged')
    if record['implementation']!=file_hash(__file__):raise ValueError('RecoveryImplementationChanged')
    return record

def publish(directory, record, expected=None, interrupt=None):
    """Caller holds recovery writer lock; CAS and idempotent discovery switch."""
    directory=Path(directory);root=identity(record);pointer=directory/'CURRENT.json'
    current=access.read(pointer)['root'] if pointer.exists() else None
    if current==root:
        discover(directory,root);return root
    if current!=expected:raise ValueError('StaleRecoveryWriter')
    if not verified_record(record) or record['epoch']!=access.epoch():raise ValueError('UnverifiedRecovery')
    path=directory/('RECOVERY_'+root+'.json')
    if interrupt:interrupt('before_record')
    durable_json(path,record)
    if interrupt:interrupt('before_pointer')
    durable_json(pointer,dict(root=root,file=path.name,sha256=file_hash(path)))
    if interrupt:interrupt('after_pointer')
    return root

def recover(reference, expected_content, expected_scientific, directory, expected=None, interrupt=None, progress=None, *, component_pinned=False):
    if type(component_pinned) is not bool:raise ValueError('UnsupportedRecoveryVerification')
    if component_pinned and (reference['kind']!='recursive' or expected_scientific!=expected_content
        or not content_pin(expected_content) or not content_pin(reference['root'])):
        raise ValueError('ComponentRecoveryRequiresPinnedRecursiveContent')
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    with writer(directory),ExitStack() as held:
        # Hold implementation identities through the discovery switch too.
        held.enter_context(access.guard(__file__))
        held.enter_context(access.guard(__import__('composable_state').__file__))
        pointer=directory/'CURRENT.json'
        if pointer.exists():
            prior=discover(directory)
            mode=COMPONENT_RECOVERY if component_pinned else RECOVERY
            if prior['schema']==mode and prior['source']==reference and prior['content']==expected_content and prior['scientific']==expected_scientific:
                with CommittedState(prior['reference']) as state:state.check()
                return dict(prior,retry=True)
            if access.read(pointer)['root']!=expected:raise ValueError('StaleRecoveryWriter')
        elif expected is not None:raise ValueError('StaleRecoveryWriter')
        current_epoch=access.epoch();authenticated={};nodes=[];seen=set();originals=[]
        staging=directory/('attempt_'+uuid.uuid4().hex);staging.mkdir()
        def visit(ref):
            if ref['root'] in seen:raise ValueError('AncestorCycle')
            seen.add(ref['root']);old=sealed_body(ref,held);kind=ref['kind']
            if component_pinned:component_source(old,kind)
            if kind=='native':
                if old['schema']!=access.SCHEMA or identity(old['content'])!=old['content_root']:raise ValueError('WrongNativeContent')
                records=refresh_records(old['files'],held,authenticated);parent=None;produced={}
                content=old['content_root']
                for p,h in old['content']['implementation'].items():
                    held.enter_context(access.guard(p))
                    from historical_implementation_access import check_pinned
                    check_pinned(p,h,content,'IncompatibleRecoveryImplementation')
            elif kind=='recursive':
                p=old['plan']
                if old['schema']!='V4IncrementalComponentTransaction/1' or p['schema']!=SCHEMA or identity(p['content'])!=p['scientific'] or old['scientific']!=p['scientific']:raise ValueError('WrongRecursiveContent')
                if 'acceptance' in old:
                    a=old['acceptance']
                    if identity(a)!=old['acceptance_identity'] or a['plan']!=identity(p) or a['base']!=old['parent'] or a.get('passed') is not True:raise ValueError('WrongSealedAcceptance')
                parent=visit(p['parent'])
                records=refresh_records(p['references'],held,authenticated)
                produced=refresh_records(old['produced'],held,authenticated);content=p['scientific']
            else:raise ValueError('UnsupportedRecoveryKind')
            if component_pinned:
                # A duplicated path must never disappear behind a dict merge.
                for path in records.keys() & produced.keys():
                    if records[path]!=produced[path]:raise ValueError('ConflictingImmutableIdentity')
                originals.append(old)
            body=rebind_body(old,kind,parent,records,produced,current_epoch)
            dest=staging/('L'+str(old.get('expected',old.get('plan',{}).get('expected',{}))['layer']))
            if kind=='native':root=access.publish_manifest(dest,body)
            else:root=Store(dest).initialize(body)
            new=dict(kind=kind,directory=str(dest.resolve()),root=root)
            nodes.append(dict(source=ref,reference=new,content=content,component_count=len(old.get('components',old.get('plan',{}).get('content',{}).get('components',{})))))
            if progress:progress('authenticated '+kind+' '+str(len(nodes)))
            return new
        new=visit(reference)
        with CommittedState(new) as state:
            if state.content_root!=expected_content:raise ValueError('WrongScientificContentRoot')
            if progress:progress('sealed component verification' if component_pinned else 'complete scientific verification')
            actual=verify_components(state,expected_content,originals) if component_pinned else state.deep_verify(expected_scientific)
            stats=[];s=state
            while True:
                stats.append(dict(layer=s.layer,stats=dict(s.stats)))
                if s.kind=='native':break
                s=s.parent
            if access.epoch()!=current_epoch:raise ValueError('EpochChangedDuringRecovery')
            # Recheck guarded files and scientific roots at publication boundary.
            for p,r in authenticated.items():
                if access.token(p)!=r['token']:raise ValueError('ChangedRecoveryComponent')
            record=dict(schema=COMPONENT_RECOVERY if component_pinned else RECOVERY,source=reference,reference=new,epoch=current_epoch,
                content=expected_content,scientific=actual,deep_verified=not component_pinned,nodes=nodes,
                files=authenticated,stats=stats,implementation=file_hash(__file__))
            if component_pinned:
                record.update(components_verified=True,verification_basis=dict(source=reference,
                    content=expected_content,legacy_digest_checked=False,
                    scope='Pinned sealed production component plan and acceptance; every declared immutable byte authenticated; no new mesh qualification.'))
            if interrupt:interrupt('deep_verified')
            publish(directory,record,expected,interrupt)
            if progress:progress('published')
        return record


def recover_components(reference,expected_content,directory,expected=None,interrupt=None,progress=None):
    """Explicit component-pinned access renewal; never accepts an absent pin."""
    return recover(reference,expected_content,expected_content,directory,expected,interrupt,progress,component_pinned=True)
