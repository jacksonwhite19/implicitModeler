"""Reuse the existing indexed quality-row store for front-atlas records.

Only physical representation changes. The atlas component still names the exact
legacy JSON byte stream. Guards, source locks, readbacks and finite row meaning
are inherited from single_quality_component; no new checkpoint or proof system.
"""
from collections.abc import Mapping,Sequence
from contextlib import ExitStack,contextmanager
import hashlib
import json
from pathlib import Path

import immutable_state_access as access
import single_quality_component as codec
from path_preserving_deduplication import durable_json,file_hash

SCHEMA='V4IndexedFrontAtlas/1'
_installed=False


def encoded(path):
    """Bounded format discovery of our canonical envelope; verify in snapshot.

    Source-keyed legacy maps start with a JSON object value. A scalar metadata
    field identifies an envelope. Unknown envelopes still fail in snapshot.
    """
    import re
    with Path(path).open('rb') as stream:head=stream.read(512)
    return re.match(rb'\s*\{\s*"(?:[^"\\]|\\.)*"\s*:\s*[^\s{]',head) is not None


def digest(rows):
    """Exact original durable_json map, one sorted row at a time."""
    h=hashlib.sha256();size=0;count=0;previous=None
    def put(data):
        nonlocal size
        h.update(data);size+=len(data)
    for row in rows:
        key=row['key'];value=row['payload']
        if (not isinstance(key,str) or row['inputs']!={} or not isinstance(value,dict)
            or set(value)!={'source','front','dependencies','versions'} or value['source']!=key
            or not isinstance(value['front'],str) or not isinstance(value['dependencies'],dict)
            or not isinstance(value['versions'],dict) or set(value['versions'])!=set(value['dependencies'])):
            raise ValueError('UnsupportedAtlasRow')
        if previous is not None and key<=previous:raise ValueError('UnorderedOrDuplicateAtlasSource')
        previous=key
        put(b',\n' if count else b'{\n')
        text=json.dumps(value,indent=2,sort_keys=True,allow_nan=False).replace('\n','\n  ')
        put(('  '+json.dumps(key)+': '+text).encode('utf-8'));count+=1
    put(b'\n}\n' if count else b'{}\n')
    return dict(sha256=h.hexdigest(),bytes=size,count=count)


class AtlasRows(Sequence):
    def __init__(self,values):self.values=values;self.keys=sorted(values)
    def __len__(self):return len(self.keys)
    def __getitem__(self,i):
        if isinstance(i,slice):return [self[j] for j in range(*i.indices(len(self)))]
        key=self.keys[i];return dict(key=key,inputs={},payload=self.values[key])


def write(path,values,*,workers=1,process=False):
    path=Path(path).resolve()
    if path.exists():raise ValueError('PreserveExistingAtlasComponent')
    path.parent.mkdir(parents=True,exist_ok=True)
    rows=AtlasRows(values);logical=digest(rows)
    envelope=path.parent/'atlas_rows.json'
    codec.write(envelope,rows,workers=workers,process=process)
    with codec.snapshot(envelope) as snap:
        if digest(snap['rows'])!=logical:raise ValueError('AtlasWriteMeaningMismatch')
        refs=snap['references']
    body=dict(schema=SCHEMA,logical=logical,rows=str(envelope),references=refs,encoder_sha256=file_hash(__file__))
    durable_json(path,body)
    return dict(**logical,physical_bytes=path.stat().st_size+sum(Path(p).stat().st_size for p in refs))


class AtlasView(Mapping):
    def __init__(self,rows,owner=None):self.rows=rows;self.owner=owner
    def check(self):
        if self.owner is not None:self.owner.check_live()
        self.rows.check()
    def __len__(self):self.check();return len(self.rows)
    def __iter__(self):
        self.check()
        for row in self.rows:
            self.check();yield row['key']
    def __getitem__(self,key):
        self.check()
        # Same indexed table as critical_quality_lookup; one bounded lookup.
        found=self.rows._db.execute('SELECT payload FROM updates WHERE key=?',(key,)).fetchone()
        if found is None:raise KeyError(key)
        return json.loads(codec.zlib.decompress(found[0]))


@contextmanager
def snapshot(path,owner=None):
    path=Path(path).resolve()
    with ExitStack() as held:
        held.enter_context(access.guard(path));body=access.read(path)
        physical=file_hash(path);refs={str(path):access.attest(path,physical)}
        if 'schema' not in body or isinstance(body['schema'],dict):
            if owner is not None and owner.components['atlas']['replace']!=physical:raise ValueError('AtlasSemanticComponentMismatch')
            yield dict(semantic=physical,references=refs,view=body,encoded=False)
            return
        if (set(body)!={'schema','logical','rows','references','encoder_sha256'} or body['schema']!=SCHEMA
            or body['encoder_sha256']!=file_hash(__file__)):
            raise ValueError('UnsupportedAtlasEncoding')
        # Authenticate every physical dependency before opening or expanding it.
        for p,r in body['references'].items():
            held.enter_context(access.guard(p))
            if owner is not None:owner.verify(Path(p))
            if access.token(p)!=r['token'] or file_hash(p)!=r['sha256']:raise ValueError('ChangedAtlasReference')
        source=Path(body['rows']).resolve()
        if str(source) not in body['references']:raise ValueError('UnboundAtlasRows')
        inner=held.enter_context(codec.snapshot(source))
        if inner['references']!=body['references'] or not inner['encoded']:raise ValueError('AtlasClosureMismatch')
        if digest(inner['rows'])!=body['logical']:raise ValueError('AtlasMeaningMismatch')
        refs.update(inner['references'])
        if owner is not None and owner.components['atlas']['replace']!=body['logical']['sha256']:
            raise ValueError('AtlasSemanticComponentMismatch')
        yield dict(semantic=body['logical']['sha256'],references=refs,view=AtlasView(inner['rows'],owner),encoded=True)


def install_reader():
    """Format-aware qualified reader; legacy sealed state code stays pinned."""
    global _installed
    if _installed:return
    from composable_state import CommittedState
    original=CommittedState.__getitem__
    def get(owner,key):
        if key=='atlas' and owner.kind=='recursive' and owner.production:
            owner.check_live();path=owner.verify(owner.out/'NEW_ATLAS.json')
            cached=getattr(owner,'_indexed_atlas',None)
            if cached is None:
                cached=owner.stack.enter_context(snapshot(path,owner));owner._indexed_atlas=cached
            return cached['view']
        return original(owner,key)
    CommittedState.__getitem__=get;_installed=True
