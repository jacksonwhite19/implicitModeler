"""Opt-in single changed-quality payload using the existing SQLite row codec.

Logical identities remain the original JSON list streams. The small envelope
authenticates order, codec, logical hashes and the immutable indexed payload.
No scientific acceptance is produced here. Legacy JSON and capsule defaults stay.
"""
from collections.abc import Sequence
from copy import deepcopy
from contextlib import ExitStack, closing, contextmanager
import hashlib
from itertools import islice
import json
import os
from pathlib import Path
import sqlite3
import tempfile
from types import MappingProxyType
import zlib

import immutable_state_access as access
from incremental_transaction import canonical
from path_preserving_deduplication import atomic_replace, durable_json, file_hash

SCHEMA = 'V4SingleChangedQuality/1'


def hashes(rows):
    """Original compact identity and indent=2 file hash, without a full buffer."""
    compact = hashlib.sha256(); pretty = hashlib.sha256(); size = 0; count = 0
    def put(data):
        nonlocal size
        pretty.update(data); size += len(data)
    compact.update(b'[')
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'key','inputs','payload'}:
            raise ValueError('UnsupportedChangedQualityRow')
        if not isinstance(row['key'],str) or not isinstance(row['inputs'],dict):
            raise ValueError('InvalidChangedQualityRow')
        if row['payload'] is not None and not isinstance(row['payload'],dict):
            raise ValueError('InvalidChangedQualityPayload')
        if count: compact.update(b','); put(b',\n')
        else: put(b'[\n')
        compact.update(canonical(row))
        text=json.dumps(row,indent=2,sort_keys=True,allow_nan=False)
        put(('  '+text.replace('\n','\n  ')).encode('utf-8')); count += 1
    compact.update(b']'); put(b'\n]\n' if count else b'[]\n')
    return dict(identity=compact.hexdigest(),sha256=pretty.hexdigest(),bytes=size,count=count)


def _iter(db):
    for ordinal,key,inputs,payload in db.execute('SELECT ordinal,key,inputs,payload FROM ordering JOIN updates USING(key) ORDER BY ordinal'):
        yield ordinal,dict(key=key,inputs=json.loads(zlib.decompress(inputs)),
                           payload=None if payload is None else json.loads(zlib.decompress(payload)))


def _verify(db, expected, witness=None):
    if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
        raise ValueError('CorruptSingleQualityDatabase')
    if db.execute('SELECT count(*) FROM updates').fetchone()[0] != expected['count']:
        raise ValueError('IncompleteSingleQualityDatabase')
    if db.execute('SELECT count(*) FROM ordering').fetchone()[0] != expected['count']:
        raise ValueError('IncompleteSingleQualityOrdering')
    def ordered():
        # One ordered decode supplies BOTH the original logical stream and the
        # canonical-blob check. The old second scan decoded the same bytes again.
        sql='SELECT ordinal,key,inputs,payload FROM ordering JOIN updates USING(key) ORDER BY ordinal'
        for i,(ordinal,key,inputs,payload) in enumerate(db.execute(sql)):
            if ordinal != i: raise ValueError('InvalidSingleQualityOrdinal')
            decoded_inputs=json.loads(zlib.decompress(inputs))
            decoded_payload=None if payload is None else json.loads(zlib.decompress(payload))
            if inputs!=zlib.compress(canonical(decoded_inputs)):
                raise ValueError('NoncanonicalSingleQualityInputs')
            if payload is not None and payload!=zlib.compress(canonical(decoded_payload)):
                raise ValueError('NoncanonicalSingleQualityPayload')
            if witness is not None:
                witness['order'].append(key);witness['blobs'][key]=(inputs,payload)
            yield dict(key=key,inputs=decoded_inputs,payload=decoded_payload)
    if hashes(ordered()) != expected: raise ValueError('SingleQualityLogicalMismatch')


def _freeze(witness):
    return MappingProxyType(dict(order=tuple(witness['order']),blobs=MappingProxyType(witness['blobs'])))


def _verify_encoded(db,expected,witness):
    """Capsule's exact accepted-blob readback, plus this format's ordinal checks.

    The witness exists only after full logical/canonical verification: either a
    private writer's closed transaction, or a reader's held deny-write guards.
    Both subsequent independent database readbacks remain. No witness is stored
    in an envelope or reused by a cold reader.
    """
    if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('CorruptSingleQualityDatabase')
    if db.execute('SELECT count(*) FROM updates').fetchone()[0]!=expected['count']:raise ValueError('IncompleteSingleQualityDatabase')
    if db.execute('SELECT count(*) FROM ordering').fetchone()[0]!=expected['count']:raise ValueError('IncompleteSingleQualityOrdering')
    count=0
    for key,inputs,payload in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'):
        if key not in witness['blobs'] or (inputs,payload)!=witness['blobs'][key]:raise ValueError('ChangedVerifiedQualityBlob')
        count+=1
    if count!=expected['count']:raise ValueError('IncompleteVerifiedQualityReadback')
    for i,(ordinal,key) in enumerate(db.execute('SELECT ordinal,key FROM ordering ORDER BY ordinal')):
        if ordinal!=i or key!=witness['order'][i]:raise ValueError('ChangedVerifiedQualityOrdinal')


def write(path, rows, *, progress=None, workers=1, process=False):
    """One private assembly payload. Refuse existing paths; never migrate seals."""
    if type(workers) is not int or workers not in (1,2,4):raise ValueError('InvalidQualityCodecWorkers')
    if type(process) is not bool:raise ValueError('InvalidQualityCodecProcess')
    if workers!=1 and not isinstance(rows,Sequence):raise ValueError('ParallelQualitySequenceRequired')
    path=Path(path).resolve()
    if path.exists(): raise ValueError('PreserveExistingQualityComponent')
    path.parent.mkdir(parents=True,exist_ok=True)
    expected=hashes(iter(rows)); objects=path.parent/'quality_objects'; objects.mkdir(exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='quality-',suffix='.pending',dir=objects);os.close(fd)
    pending=Path(name)
    from critical_quality_codec import encoded
    with closing(sqlite3.connect(pending)) as db:
        db.execute('PRAGMA journal_mode=DELETE');db.execute('PRAGMA synchronous=FULL')
        db.execute('CREATE TABLE updates(key TEXT PRIMARY KEY, inputs BLOB NOT NULL, payload BLOB) WITHOUT ROWID')
        db.execute('CREATE TABLE ordering(ordinal INTEGER PRIMARY KEY, key TEXT UNIQUE NOT NULL)')
        # Same bounded codec and original list order. Duplicate keys fail in SQL.
        iterator=iter(rows); ordinal=0
        def batches():
            if workers==1:
                while chunk:=list(islice(iterator,2048)):
                    yield from zip(chunk,encoded(chunk,1,False),strict=True)
            else:
                # Exact qualified capsule codec: bounded ordered 2048-row jobs.
                with closing(encoded(rows,workers,process)) as compressed:
                    yield from zip(rows,compressed,strict=True)
        for row,blobs in batches():
            db.execute('INSERT INTO updates VALUES(?,?,?)',(row['key'],*blobs))
            db.execute('INSERT INTO ordering VALUES(?,?)',(ordinal,row['key'])); ordinal+=1
            if progress and ordinal%2048==0:progress()
        if progress and ordinal%2048:progress()
        db.commit()
        witness=dict(order=[],blobs={})
        _verify(db,expected,witness)
        witness=_freeze(witness)
    with pending.open('r+b') as stream: os.fsync(stream.fileno())
    digest=file_hash(pending);target=objects/(digest+'.sqlite')
    if target.exists():
        # Existing transaction recovery rule: exact immutable object reuse only.
        # The old object/token stays; only a proven new private duplicate goes.
        with access.guard(target):
            if file_hash(target)!=digest:raise ValueError('CorruptExistingQualityPayload')
            pending.unlink()
    else:atomic_replace(pending,target)
    with access.guard(target), closing(sqlite3.connect(target.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
        # Qualified capsule readback pattern: compare the exact fully validated
        # accepted blobs on a NEW connection after fsync/rename and under guard.
        # Counts, integrity and original ordering are checked independently too.
        _verify_encoded(db,expected,witness)
        descriptor=access.attest(target,digest)
        durable_json(path,dict(schema=SCHEMA,logical=expected,payload=dict(path=str(target),**descriptor),
                               encoder_sha256=file_hash(__file__)))
    return dict(**expected,physical_bytes=target.stat().st_size+path.stat().st_size)


class QualityRows(Sequence):
    """Bounded original-order and indexed reads under held authenticated guards."""
    def __init__(self,held,db,path,body,references,witness=None):
        self._held=held;self._db=db;self._path=path;self._body=deepcopy(body);self._references=deepcopy(references);self.closed=False
        self.__witness=witness
        self._verified_binding=canonical(dict(body=self._body,references=self._references)) if witness is not None else None
    @property
    def _verified(self):return self.__witness
    @property
    def path(self):return self._path
    @property
    def body(self):return deepcopy(self._body)
    @property
    def references(self):return deepcopy(self._references)
    def check(self):
        if self.closed: raise ValueError('ClosedSingleQualitySnapshot')
        path=str(self._path.resolve())
        if path!=self._body['payload']['path'] or path not in self._references:
            raise ValueError('ChangedSingleQualityPayloadBinding')
        if self._verified is not None and self._verified_binding!=canonical(dict(body=self._body,references=self._references)):
            raise ValueError('ChangedVerifiedQualityBinding')
        for p,r in self._references.items():
            if access.token(p)!=r['token']:raise ValueError('ChangedSingleQualityReference')
    def __len__(self): self.check();return self._body['logical']['count']
    def __iter__(self):
        self.check()
        for _,row in _iter(self._db):
            if self.closed: raise ValueError('ClosedSingleQualitySnapshot')
            yield row
    def __getitem__(self,index):
        self.check()
        if isinstance(index,slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        if index<0:index+=len(self)
        result=self._db.execute('SELECT key,inputs,payload FROM ordering JOIN updates USING(key) WHERE ordinal=?',(index,)).fetchone()
        if result is None: raise IndexError(index)
        key,inputs,payload=result
        return dict(key=key,inputs=json.loads(zlib.decompress(inputs)),payload=None if payload is None else json.loads(zlib.decompress(payload)))
    def verify_readback(self):
        self.check()
        with closing(sqlite3.connect(self.path.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
            if self._verified is None:_verify(db,self._body['logical'])
            else:_verify_encoded(db,self._body['logical'],self._verified)


@contextmanager
def snapshot(path, *, require_rows=True, legacy_read=None, physical_hash=None, retain_verified=False):
    path=Path(path).resolve()
    if type(retain_verified) is not bool:raise ValueError('InvalidVerifiedQualityMode')
    with ExitStack() as held:
        held.enter_context(access.guard(path))
        # Preserve the old guarded hash/cache path without expanding a legacy
        # quality list during preparation (which only needs its file identity).
        with path.open('rb') as stream:legacy=stream.read(1)==b'['
        refs={str(path):access.attest(path,(physical_hash or file_hash)(path))}
        if legacy:
            body=(legacy_read or access.read)(path) if require_rows else None
            if require_rows and not isinstance(body,list):raise ValueError('InvalidLegacyQualityList')
            yield dict(rows=body,semantic=refs[str(path)]['sha256'],references=refs,encoded=False)
            return
        body=access.read(path)
        if (not isinstance(body,dict) or set(body)!={'schema','logical','payload','encoder_sha256'}
            or body['schema']!=SCHEMA or body['encoder_sha256']!=file_hash(__file__)):
            raise ValueError('UnsupportedSingleQualityEncoding')
        source=Path(body['payload']['path']).resolve();held.enter_context(access.guard(source))
        if access.token(source)!=body['payload']['token'] or file_hash(source)!=body['payload']['sha256']:
            raise ValueError('ChangedSingleQualityReference')
        refs[str(source)]=access.attest(source,body['payload']['sha256'])
        db=held.enter_context(closing(sqlite3.connect(source.as_uri()+'?mode=ro&immutable=1',uri=True)))
        witness=dict(order=[],blobs={}) if retain_verified else None
        _verify(db,body['logical'],witness)
        if witness is not None:
            witness=_freeze(witness)
        rows=QualityRows(held,db,source,body,refs,witness)
        try:
            yield dict(rows=rows,semantic=body['logical']['sha256'],references=refs,encoded=True)
        finally:rows.closed=True


def quality_identity(rows):
    if isinstance(rows,QualityRows):
        rows.check()
        if rows._verified is not None:return rows._body['logical']['identity']
        return hashes(iter(rows))['identity']
    from sdf_cfd_mesher_v4.common.authority import identity
    return identity(rows)


def verify_transaction_source(rows,plan):
    """The admitted plan must authenticate both files, not just the envelope."""
    if not isinstance(rows,QualityRows): return False
    rows.check()
    for p,r in rows.references.items():
        if plan['references'].get(p)!=r:raise ValueError('UnboundSingleQualityTransaction')
    if str(rows.path.resolve()) not in plan['references']:
        raise ValueError('UnboundSingleQualityPayloadPath')
    return True


def recover_json(path,destination):
    """Recover exact legacy bytes at a NEW path; never recreate old tokens."""
    destination=Path(destination).resolve()
    if destination.exists():raise ValueError('PreserveExistingQualityRecovery')
    with snapshot(path) as snap:
        if not snap['encoded']:raise ValueError('SingleQualityRecoveryRequired')
        fd,name=tempfile.mkstemp(prefix='recover-',suffix='.pending',dir=destination.parent);os.close(fd)
        pending=Path(name);digest=hashlib.sha256();count=0
        with pending.open('wb') as stream:
            def put(data):stream.write(data);digest.update(data)
            for row in snap['rows']:
                put(b',\n' if count else b'[\n')
                text=json.dumps(row,indent=2,sort_keys=True,allow_nan=False)
                put(('  '+text.replace('\n','\n  ')).encode('utf-8'));count+=1
            put(b'\n]\n' if count else b'[]\n');stream.flush();os.fsync(stream.fileno())
        if digest.hexdigest()!=snap['semantic']:raise ValueError('QualityRecoveryMismatch')
        atomic_replace(pending,destination)
    return dict(sha256=file_hash(destination),bytes=destination.stat().st_size)
