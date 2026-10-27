"""Scoped access-reference resolution using the existing strict recovery proof.

This changes validation anchors, not authorization bytes or trajectory identity.
The original parent validator still checks all admissions and scientific owners.
No digest-only equivalence, state mutation, recovery or publication occurs here.
"""
from contextlib import contextmanager, ExitStack
from contextvars import ContextVar
from copy import deepcopy
from pathlib import Path
from threading import Lock
import immutable_state_access as access
import recursive_epoch_recovery as recovery
from sdf_cfd_mesher_v4.common.authority import identity, file_hash

_active=ContextVar('recovered_parent_authorization',default=None)
_installer=Lock()


class RecoveryAnchors:
    def __init__(self,base,directory,authorization,expected_recovery_identity,held):
        self.closed=False;self.base=base;self.authorization=identity(authorization)
        directory=Path(directory)
        held.enter_context(access.guard(directory/'CURRENT.json'))
        pointer=access.read(directory/'CURRENT.json')
        held.enter_context(access.guard(directory/pointer['file']))
        held.enter_context(access.guard(recovery.__file__))
        if not recovery.content_pin(expected_recovery_identity):raise ValueError('PinnedRecoveryAuthorizationRequired')
        self.record=recovery.discover(directory,expected_recovery_identity)
        self.record_identity=identity(self.record)
        if base.reference!=self.record['reference'] or base.scientific!=self.record['scientific']:
            raise ValueError('WrongRecoveredAuthorizationParent')
        self.paths=[directory/'CURRENT.json',directory/pointer['file'],Path(recovery.__file__)]
        originals=[];self.mapping={};self.live_nodes=[];node=base;rows=list(reversed(self.record['nodes']))
        source=self.record['source'];files={}
        for row in rows:
            if row['source']!=source or row['reference']!=node.reference:
                raise ValueError('WrongRecoveryAuthorizationAncestry')
            old=recovery.sealed_body(source,held)
            d=Path(source['directory']);self.paths.extend((d/'CURRENT.json',d/('ROOT_'+source['root']+'.json')))
            recovery.component_source(old,node.kind)
            originals.append(old)
            body=node.wrapped.body if node.kind=='native' else node.body
            current=body['files'] if node.kind=='native' else {**body['plan']['references'],**body['produced']}
            for p,r in current.items():
                if p in files and files[p]!=r:raise ValueError('ConflictingRecoveryAuthorizationFiles')
                files[p]=r
            self.mapping[node.layer]=dict(source=deepcopy(source),reference=deepcopy(node.reference),scientific=node.scientific)
            self.live_nodes.append((node,node.kind,node.layer,node.content_root))
            if node.kind=='native':
                if row is not rows[-1]:raise ValueError('ExtraRecoveryAuthorizationAncestry')
            else:source=old['plan']['parent'];node=node.parent
        if not rows or node.kind!='native':raise ValueError('IncompleteRecoveryAuthorizationAncestry')
        if files!=self.record['files']:raise ValueError('IncompleteRecoveryAuthorizationFiles')
        recovery.verify_components(base,self.record['content'],list(reversed(originals)))
        for p,r in files.items():
            held.enter_context(access.guard(p))
            if access.token(p)!=r['token']:raise ValueError('ChangedRecoveryAuthorizationFile')
        self.paths.extend(Path(p) for p in files)
        self.tokens={str(p):access.token(p) for p in self.paths}
        self.mapping_identity=identity(self.mapping)
        self.evidence=dict(schema='V4RecoveredAuthorizationAccess/1',authorization=self.authorization,
            recovery=self.record_identity,source=self.record['source'],reference=self.record['reference'],
            scientific=base.scientific,content=base.content_root,nodes=len(rows),files=len(files),
            implementation=file_hash(__file__),scope='Access-only resolution; original authorization and trajectory arithmetic unchanged')
        self.evidence_identity=identity(self.evidence)

    def check(self):
        if self.closed:raise ValueError('ClosedRecoveryAuthorizationScope')
        if identity(self.record)!=self.record_identity or identity(self.evidence)!=self.evidence_identity:
            raise ValueError('AlteredRecoveryAuthorizationEvidence')
        if identity(self.mapping)!=self.mapping_identity:raise ValueError('AlteredRecoveryAuthorizationMapping')
        if (self.base.reference!=self.record['reference'] or self.base.scientific!=self.record['scientific']
            or self.base.content_root!=self.record['content']):raise ValueError('WrongRecoveredAuthorizationParent')
        node=self.base
        for expected,kind,layer,content in self.live_nodes:
            row=self.mapping[layer]
            if (node is not expected or node.kind!=kind or node.layer!=layer or node.content_root!=content
                or node.reference!=row['reference'] or node.root!=row['reference']['root']
                or node.scientific!=row['scientific']):
                raise ValueError('ChangedLiveRecoveryAuthorizationAncestry')
            node.check_live()
            if kind!='native':node=node.parent
        self.base.check()
        if self.record['epoch']!=access.epoch():raise ValueError('RecoveryAuthorizationEpochChanged')
        if any(access.token(p)!=t for p,t in self.tokens.items()):raise ValueError('ChangedRecoveryAuthorizationDependency')

    def resolve(self,record,original):
        self.check()
        if identity(record)!=self.authorization:raise ValueError('WrongRecoveryAuthorizationScope')
        layers,anchors=original(record)
        if not anchors:raise ValueError('RecoveryAuthorizationAnchorRequired')
        result=deepcopy(anchors)
        for layer,anchor in result.items():
            row=self.mapping.get(layer)
            if row is None or anchor['reference']!=row['source'] or anchor['scientific']!=row['scientific']:
                raise ValueError('WrongRecoveredAuthorizationAnchor')
            anchor['reference']=deepcopy(row['reference'])
        return layers,result


@contextmanager
def recovered_authorization(base,directory,authorization,expected_recovery_identity):
    """Install only a guard-held validation view; restore the shared consumer.

Thread-local scope prevents another thread borrowing this parent's resolution.
The original validator and implementation hash remain unchanged.
"""
    import accepted_production_contract as production
    from composable_state import CommittedState
    if type(base) is not CommittedState:raise ValueError('QualifiedCommittedStateRequired')
    if _active.get() is not None or not _installer.acquire(blocking=False):
        raise ValueError('NestedRecoveryAuthorizationScope')
    with ExitStack() as held:
        held.callback(_installer.release)
        original=production.authorization_ancestry
        for p in (__file__,production.__file__,__import__('composable_state').__file__):
            held.enter_context(access.guard(p))
        proof=RecoveryAnchors(base,directory,authorization,expected_recovery_identity,held)
        token=_active.set(proof)
        def resolve(record):
            current=_active.get()
            return original(record) if current is None else current.resolve(record,original)
        production.authorization_ancestry=resolve
        try:
            yield deepcopy(proof.evidence)
            proof.check()
        finally:
            proof.closed=True
            changed=production.authorization_ancestry is not resolve
            production.authorization_ancestry=original
            _active.reset(token)
            if changed:raise ValueError('RecoveryAuthorizationConsumerChanged')
