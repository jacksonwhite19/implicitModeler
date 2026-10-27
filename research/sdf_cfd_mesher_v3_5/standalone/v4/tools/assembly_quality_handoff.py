"""Keep the capsule's accepted-blob witness across assembly -> publication.

Only the live private writer can supply it. Existing cold validation and file
formats stay unchanged. Independent database readbacks, tokens and guards remain.
"""
from contextlib import ExitStack, closing, contextmanager, nullcontext
from contextvars import ContextVar
from copy import deepcopy
from pathlib import Path
import sqlite3
import threading
from unittest.mock import patch

import immutable_state_access as access
import single_quality_component as quality
from incremental_transaction import canonical

_active = ContextVar('v4_assembly_quality_handoff', default=None)
_write = quality.write
_snapshot = quality.snapshot
_freeze = quality._freeze
_durable = quality.durable_json
_installation = threading.Lock()


def live(active):
    if active['closed']:
        raise ValueError('ClosedAssemblyQualityHandoff')
    if active['thread'] != threading.get_ident():
        raise ValueError('WrongAssemblyQualityHandoffThread')


def write(path, rows, **kwargs):
    active = _active.get()
    if active is None:
        return _write(path, rows, **kwargs)
    live(active)
    path = Path(path).resolve()
    if path in active['entries']:
        raise ValueError('DuplicateAssemblyQualityWitness')
    if active['reuse_private_codec']:
        from compact_front_atlas import AtlasRows
        if type(rows) is AtlasRows:
            # The existing private shortcut requires stable row identities.
            # AtlasRows creates fresh wrappers on indexing; materialize wrappers
            # once, not another atlas payload. Existing snapshot checks remain.
            rows=list(rows)
    captured = []; provisional = []; local = ExitStack()
    thread = threading.get_ident()
    def frozen(witness):
        result = _freeze(witness)
        if threading.get_ident() == thread:
            captured.append(result)
        return result
    def durable(target, body):
        _durable(target, body)
        if threading.get_ident() != thread or Path(target).resolve() != path:
            return
        if len(captured) != 1:
            raise ValueError('MissingPrivateQualityWriterWitness')
        # This hook runs inside the original writer's target deny-write guard,
        # after full logical validation, fsync, rename and independent readback.
        source = Path(body['payload']['path']).resolve()
        local.enter_context(access.guard(source)); local.enter_context(access.guard(path))
        if canonical(access.read(path)) != canonical(body):
            raise ValueError('ChangedWrittenQualityEnvelope')
        if body['encoder_sha256'] != quality.file_hash(quality.__file__):
            raise ValueError('ChangedQualityWriterImplementation')
        refs = {str(path): access.attest(path, quality.file_hash(path)),
                str(source): access.attest(source, body['payload']['sha256'])}
        if access.token(source) != body['payload']['token']:
            raise ValueError('ChangedAssemblyQualityPayload')
        entry = dict(body=deepcopy(body), references=refs, witness=captured[0])
        entry['binding'] = canonical(dict(body=entry['body'], references=refs))
        provisional.append(entry)
    try:
        from private_quality_codec_reuse import verified_private_codec
        from prism_sql_profile import measured_quality_writer
        from quality_storage_layout import installed as packed_layout
        metrics=path.parent/'writer_metrics'/path.stem
        metrics.mkdir(parents=True,exist_ok=True)
        # Preserve the original single-worker reiterable interface. A sized
        # private collection permits the existing adaptive cache estimator.
        try:count=len(rows)
        except TypeError:count=None
        profile=(measured_quality_writer(metrics,count,adaptive=True,private_directory=path.parent/'quality_objects')
                 if count is not None else nullcontext())
        # Retained exact controls select physical packing by component role,
        # not geometry. The atlas has larger records than quality receipts.
        # Unknown helper uses keep the qualified historical 4096-byte layout.
        page_size={'QUALITY_DELTA.json':8192,'atlas_rows.json':16384}.get(path.name,4096)
        with packed_layout(page_size), patch.object(quality, '_freeze', frozen), patch.object(quality, 'durable_json', durable), \
                (verified_private_codec(rows,workers=kwargs.get('workers',1),process=kwargs.get('process',False))
                 if active['reuse_private_codec'] else nullcontext()), \
                profile:
            result = _write(path, rows, **kwargs)
        live(active)
        if len(provisional) != 1:
            raise ValueError('IncompletePrivateQualityWriter')
        entry = provisional[0]
        entry['held'] = local.pop_all()
        entry['borrowers'] = 0
        active['held'].callback(entry['held'].close)
        active['entries'][path] = entry
        active['stats']['writer_witnesses'] += 1
        return result
    finally:
        local.close()


@contextmanager
def snapshot(path, **kwargs):
    active = _active.get(); path = Path(path).resolve()
    if active is not None:
        live(active)
    entry = None if active is None else active['entries'].get(path)
    if entry is None or kwargs.get('legacy_read') is not None or kwargs.get('physical_hash') is not None:
        with _snapshot(path, **kwargs) as snap:
            yield snap
        return
    if set(kwargs)-{'require_rows','legacy_read','physical_hash','retain_verified'}:
        raise TypeError('UnsupportedQualitySnapshotArgument')
    if type(kwargs.get('retain_verified', False)) is not bool:
        raise ValueError('InvalidVerifiedQualityMode')
    body = entry['body']; refs = entry['references']
    if canonical(dict(body=body, references=refs)) != entry['binding']:
        raise ValueError('ChangedAssemblyQualityBinding')
    for name, record in refs.items():
        if access.token(name) != record['token']:
            raise ValueError('ChangedAssemblyQualityReference')
    if quality.file_hash(quality.__file__) != body['encoder_sha256']:
        raise ValueError('ChangedQualityWriterImplementation')
    with ExitStack() as held:
        # Borrowers own overlapping file guards. They can safely outlive the
        # enclosing stage, but their own snapshot exit always revokes the reader.
        for name, record in refs.items():
            held.enter_context(access.guard(name))
            if access.token(name) != record['token']:
                raise ValueError('ChangedAssemblyQualityReference')
        db = held.enter_context(closing(sqlite3.connect(Path(body['payload']['path']).as_uri()+'?mode=ro&immutable=1', uri=True)))
        # Exact independent readback, not merely trusting a pathname/hash.
        quality._verify_encoded(db, body['logical'], entry['witness'])
        active['stats']['independent_blob_readbacks'] += 1
        reader = quality.QualityRows(held, db, Path(body['payload']['path']), body, refs, entry['witness'])
        entry['borrowers'] += 1
        try:
            yield dict(rows=reader, semantic=body['logical']['sha256'], references=deepcopy(refs), encoded=True)
        finally:
            reader.closed = True
            entry['borrowers'] -= 1


def release_directory(directory):
    """End completed writer witnesses; no file writes or deletion authority.

    The caller closes preparation jobs after replay comparison/publication.
    Active borrowers fail closed. Other directories keep their original guards.
    A later read uses the original cold verifier, never a revoked witness.
    """
    active = _active.get()
    if active is None:
        return 0
    live(active)
    directory = Path(directory).resolve()
    selected = [(p, e) for p, e in active['entries'].items() if p.parent == directory]
    if any(e['borrowers'] for _, e in selected):
        raise ValueError('ActiveAssemblyQualityBorrower')
    for path, entry in selected:
        entry['held'].close()
        del active['entries'][path]
    return len(selected)


@contextmanager
def installed(*, reuse_private_codec=False):
    """One same-process stage lifetime; cold/resumed operations retain full checks."""
    if type(reuse_private_codec) is not bool:
        raise ValueError('InvalidPrivateCodecReuseMode')
    if _active.get() is not None:
        raise ValueError('NestedAssemblyQualityHandoff')
    if not _installation.acquire(blocking=False):
        raise ValueError('ConcurrentAssemblyQualityHandoff')
    try:
        with ExitStack() as held:
            active = dict(held=held, entries={}, closed=False, thread=threading.get_ident(),reuse_private_codec=reuse_private_codec,
                          stats=dict(writer_witnesses=0, independent_blob_readbacks=0))
            token = _active.set(active)
            try:
                with patch.object(quality, 'write', write), patch.object(quality, 'snapshot', snapshot):
                    yield active['stats']
            finally:
                active['closed'] = True
                _active.reset(token)
                active['entries'].clear()
    finally:
        _installation.release()
