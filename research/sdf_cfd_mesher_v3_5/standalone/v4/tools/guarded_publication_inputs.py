"""Invocation-local reuse of existing fully checked, deny-write guarded inputs.

Not a persisted trust record or a format. Cold retries still use cold readers.
Only the enclosing live publication owns the guards; borrowers cannot close them.
"""
from contextlib import ExitStack,contextmanager
from contextvars import ContextVar
from copy import deepcopy
from pathlib import Path

import immutable_state_access as access
import single_quality_component as quality
import compact_front_atlas as atlas
from incremental_transaction import canonical

_active=ContextVar('v4_guarded_publication_inputs',default=None)


def _metadata(snap):
    return {k:v for k,v in snap.items() if k not in ('rows','view')}


class _Inputs:
    def __init__(self,out,qsnap,asnap):
        self.out=Path(out).resolve();self.closed=False
        self._quality=qsnap;self._atlas=asnap
        self._qrows=qsnap['rows'];self._aview=asnap['view']
        self._binding=canonical(dict(out=str(self.out),quality=_metadata(qsnap),atlas=_metadata(asnap)))
    def check(self,out):
        if self.closed:raise ValueError('ClosedPublicationInputs')
        if Path(out).resolve()!=self.out:raise ValueError('WrongPublicationInputDirectory')
        if (self._qrows is not self._quality['rows'] or self._aview is not self._atlas['view'] or
            self._binding!=canonical(dict(out=str(self.out),quality=_metadata(self._quality),atlas=_metadata(self._atlas)))):
            raise ValueError('ChangedPublicationInputBinding')
        self._qrows.check();self._aview.check()
        for snap in (self._quality,self._atlas):
            for path,record in snap['references'].items():
                if access.token(path)!=record['token']:raise ValueError('ChangedPublicationInputReference')
    def borrow(self,out,kind):
        self.check(out)
        snap=self._quality if kind=='quality' else self._atlas
        # Metadata is copied; only already-verified guard-owned readers are shared.
        return dict(deepcopy(_metadata(snap)),**({'rows':self._qrows} if kind=='quality' else {'view':self._aview}))


@contextmanager
def scope(out):
    """Hold exact existing cold checks once over prepare -> accept -> commit.

    Explicit opt-in only. Legacy lists/maps do not gain mutable cache trust.
    No witness survives guard closure, crosses a process, or enters a manifest.
    """
    if _active.get() is not None:raise ValueError('NestedPublicationInputScope')
    out=Path(out).resolve()
    with ExitStack() as held:
        qsnap=held.enter_context(quality.snapshot(out/'QUALITY_DELTA.json',retain_verified=True))
        if not qsnap['encoded'] or qsnap['rows']._verified is None:
            raise ValueError('UnsupportedGuardedPublicationQuality')
        asnap=held.enter_context(atlas.snapshot(out/'NEW_ATLAS.json'))
        if not asnap['encoded']:raise ValueError('UnsupportedGuardedPublicationAtlas')
        inputs=_Inputs(out,qsnap,asnap);inputs.check(out)
        token=_active.set(inputs)
        try:yield inputs
        finally:
            inputs.closed=True
            _active.reset(token)


@contextmanager
def quality_snapshot(path,**kwargs):
    inputs=_active.get();path=Path(path).resolve()
    if inputs is None:
        with quality.snapshot(path,**kwargs) as snap:yield snap
        return
    if path.name!='QUALITY_DELTA.json':raise ValueError('WrongPublicationQualityPath')
    # Existing callers all use the default authoritative reader; caller-supplied
    # custom reader/hash hooks are not invoked when the held cold check is reused.
    yield inputs.borrow(path.parent,'quality')


@contextmanager
def atlas_snapshot(path):
    inputs=_active.get();path=Path(path).resolve()
    if inputs is None:
        with atlas.snapshot(path) as snap:yield snap
        return
    if path.name!='NEW_ATLAS.json':raise ValueError('WrongPublicationAtlasPath')
    yield inputs.borrow(path.parent,'atlas')
