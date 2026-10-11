"""Cold atlas connection to existing native shared-member receipt storage.

The recursive atlas codec is unchanged. Native compact JSON, not its pretty
digest, remains the cold scientific identity. No sealed object is migrated.
"""
from collections.abc import Mapping
from contextlib import closing
import json,sqlite3
from compact_front_atlas import AtlasRows,digest
from compact_initial_quality import bind_shared
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER

_installed=False


def bind_state(state,directory,*,progress=None):
    if type(state['atlas']) is not dict:return False
    values=state['atlas']
    # Reuse the existing complete row validator, including dependencies/versions.
    # Its pretty digest is validation only; bind_shared independently hashes the
    # original native canonical stream and checks reconstructed bytes against it.
    digest(AtlasRows(values))
    table=bind_shared(directory,((k,values[k]) for k in sorted(values)),
                      member_field='dependencies',progress=progress)
    state['atlas']=table
    return True


class NativeAtlas(Mapping):
    """Expanded source rows, guarded by their existing immutable state owner."""
    def __init__(self,owner):
        self.owner=owner;self.access=owner.wrapped;self.row=owner.components['atlas']
        if self.row['encoding']!='receipt_table':raise ValueError('UnsupportedColdAtlasEncoding')
    def _live(self):self.owner.check_live();self.access._live()
    def __len__(self):self._live();return self.row['count']
    def _raw(self,sql,args=()):
        self._live();path=self.access._object_path(self.row)
        self.access._verify_read(path)
        with closing(sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
            for row in db.execute(sql,args):
                self._live();self.owner.stats['atlas_rows_visited']+=1
                yield row
    def __iter__(self):
        for key, in self._raw('SELECT key FROM receipts ORDER BY key'):yield key
    def __getitem__(self,key):
        with closing(self._raw('SELECT value FROM receipts WHERE key=?',(key,))) as rows:
            found=next(rows,None)
        if found is None:raise KeyError(key)
        return json.loads(found[0])
    def items(self):
        for key,value in self._raw('SELECT key,value FROM receipts ORDER BY key'):
            yield key,json.loads(value)
    def native_canonical_chunks(self):
        def chunks():
            yield b'{'
            for i,(key,value) in enumerate(self._raw('SELECT key,value FROM receipts ORDER BY key')):
                if i:yield b','
                yield ENCODER.encode(key).encode();yield b':';yield value.encode()
            yield b'}'
        for chunk in chunks():
            self._live();yield chunk


def install_reader():
    """Opt-in generic format dispatch; historical pinned reader stays unchanged."""
    global _installed
    if _installed:return
    from composable_state import CommittedState
    original=CommittedState.__getitem__
    def get(owner,key):
        if key=='atlas' and owner.kind=='native' and owner.components[key]['encoding']!='json':
            owner.check_live()
            cached=getattr(owner,'_native_atlas_view',None)
            if cached is None:cached=owner._native_atlas_view=NativeAtlas(owner)
            cached._live()
            return cached
        return original(owner,key)
    CommittedState.__getitem__=get;_installed=True
