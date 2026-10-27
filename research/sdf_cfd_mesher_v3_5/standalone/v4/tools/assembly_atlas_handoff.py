"""Reuse a verified private atlas mapping while its original files stay guarded.

No format change or persisted trust. Original write validates the map against
decoded rows. Warm snapshots retain independent blob readbacks; cold snapshots
still use the original full decoder and map digest.
"""
from contextlib import ExitStack,contextmanager
from contextvars import ContextVar
from copy import deepcopy
from pathlib import Path
import threading
from unittest.mock import patch

import compact_front_atlas as atlas
import single_quality_component as quality
import immutable_state_access as access
from incremental_transaction import canonical

_active=ContextVar('v4_assembly_atlas_handoff',default=None)
_write=atlas.write
_snapshot=atlas.snapshot
_durable=atlas.durable_json
_lock=threading.Lock()


def live(active):
    if active['closed'] or active['thread']!=threading.get_ident():
        raise ValueError('ClosedOrForeignAtlasHandoff')


def write(path,values,**kwargs):
    active=_active.get()
    if active is None:return _write(path,values,**kwargs)
    live(active);path=Path(path).resolve()
    if path in active['entries']:raise ValueError('DuplicateAssemblyAtlasWitness')
    captured=[];local=ExitStack();thread=threading.get_ident()
    def durable(target,body):
        _durable(target,body)
        if threading.get_ident()!=thread or Path(target).resolve()!=path:return
        # Original writer calls this only AFTER AtlasWriteMeaningMismatch check.
        local.enter_context(access.guard(path))
        if canonical(access.read(path))!=canonical(body):raise ValueError('ChangedWrittenAtlasEnvelope')
        if body['encoder_sha256']!=atlas.file_hash(atlas.__file__):raise ValueError('ChangedAtlasWriter')
        for p,r in body['references'].items():
            local.enter_context(access.guard(p))
            if access.token(p)!=r['token']:raise ValueError('ChangedWrittenAtlasReference')
        entry=dict(body=deepcopy(body),references={str(path):access.attest(path,atlas.file_hash(path)),**deepcopy(body['references'])})
        entry['binding']=canonical(dict(body=entry['body'],references=entry['references']))
        captured.append(entry)
    try:
        with patch.object(atlas,'durable_json',durable):result=_write(path,values,**kwargs)
        live(active)
        if len(captured)!=1:raise ValueError('IncompleteAssemblyAtlasWriter')
        active['held'].enter_context(local.pop_all());active['entries'][path]=captured[0]
        return result
    finally:local.close()


@contextmanager
def snapshot(path,owner=None):
    active=_active.get();path=Path(path).resolve()
    if active is not None:live(active)
    entry=None if active is None else active['entries'].get(path)
    if entry is None:
        with _snapshot(path,owner) as snap:yield snap
        return
    body=entry['body'];refs=entry['references']
    if canonical(dict(body=body,references=refs))!=entry['binding']:raise ValueError('ChangedAtlasWitness')
    if body['encoder_sha256']!=atlas.file_hash(atlas.__file__):raise ValueError('ChangedAtlasWriter')
    with ExitStack() as held:
        for p,r in refs.items():
            held.enter_context(access.guard(p))
            if owner is not None:owner.verify(Path(p))
            if access.token(p)!=r['token']:raise ValueError('ChangedAssemblyAtlasReference')
        # Never omit the independent row/blob readback, even for a warm atlas.
        inner=held.enter_context(quality.snapshot(Path(body['rows'])))
        if not inner['encoded'] or inner['references']!=body['references']:raise ValueError('AtlasClosureMismatch')
        if owner is not None and owner.components['atlas']['replace']!=body['logical']['sha256']:
            raise ValueError('AtlasSemanticComponentMismatch')
        yield dict(semantic=body['logical']['sha256'],references=deepcopy(refs),view=atlas.AtlasView(inner['rows'],owner),encoded=True)


@contextmanager
def installed():
    if _active.get() is not None or not _lock.acquire(blocking=False):raise ValueError('ConcurrentAtlasHandoff')
    try:
        with ExitStack() as held:
            active=dict(held=held,entries={},closed=False,thread=threading.get_ident())
            token=_active.set(active)
            try:
                with patch.object(atlas,'write',write),patch.object(atlas,'snapshot',snapshot):yield
            finally:active['closed']=True;_active.reset(token);active['entries'].clear()
    finally:_lock.release()
