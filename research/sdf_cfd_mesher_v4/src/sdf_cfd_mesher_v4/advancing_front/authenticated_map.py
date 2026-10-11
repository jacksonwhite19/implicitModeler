"""Deterministic persistent binary Patricia Merkle map over SHA-256 keys.

Leaves bind canonical key and value digests. Branches use the first differing
key bit, so the root depends on content, never insertion or storage order.
SQLite files contain immutable content-addressed nodes, not scientific values.
"""
from contextlib import closing
from functools import lru_cache
from pathlib import Path
import hashlib,os,sqlite3,tempfile
import numpy as np
from collections import OrderedDict
from ..common.authority import file_hash

SCHEMA='V4PatriciaSHA256Map/1'
DOMAIN=SCHEMA.encode()+b'\0'
EMPTY=hashlib.sha256(DOMAIN+b'E').digest()

def digest(data):return hashlib.sha256(DOMAIN+data).digest()
def key_digest(key):return hashlib.sha256(key.encode('utf8')).digest()
def value_digest(canonical_bytes):return hashlib.sha256(canonical_bytes).digest()

class Nodes:
    def __init__(self, directory, files=()):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.files=list(files);self.readers=[];self.read_count=0;self.written=0;self.recent_writes=OrderedDict()
        for row in reversed(self.files):
            if Path(row['file']).name!=row['file']:raise ValueError('MerkleObjectPath')
            path=self.directory/row['file']
            from .verified_objects import verify
            try:verify(path,row['sha256'])
            except ValueError as exc:raise ValueError('MerkleObjectCorrupt') from exc
            reader=sqlite3.connect(path.resolve().as_uri()+'?mode=ro&immutable=1',uri=True)
            # Verified immutable nodes: let the OS page the exact file instead
            # of repeatedly copying random B-tree pages through SQLite's cache.
            reader.execute('PRAGMA mmap_size='+str(path.stat().st_size))
            self.readers.append(reader)
        fd,name=tempfile.mkstemp(prefix='merkle-',suffix='.pending',dir=self.directory);os.close(fd)
        self.pending=Path(name);self.db=sqlite3.connect(self.pending)
        # Bounded writer cache selected by the preserved 2/16/64 MiB node
        # insertion probe. This is a storage budget, not a scientific limit.
        self.db.execute('PRAGMA cache_size=-65536')
        # Append payloads by rowid. The smaller hash index remains directly
        # addressable; random hashes need not relocate the larger node payloads.
        self.db.execute('CREATE TABLE nodes(hash BLOB PRIMARY KEY,data BLOB NOT NULL)')
        self.get=lru_cache(maxsize=32768)(self._get)

    def _get(self,key):
        if key==EMPTY:return None
        if key in self.recent_writes:return self.recent_writes[key]
        self.read_count+=1
        # Recent new nodes are already checked above. Remaining requests are
        # normally historical: do not probe the writable index for every old
        # node before looking in its immutable component.
        for db in (*self.readers,self.db):
            row=db.execute('SELECT data FROM nodes WHERE hash=?',(key,)).fetchone()
            if row is not None:
                data=bytes(row[0])
                if digest(data)!=key:raise ValueError('MerkleNodeCorrupt')
                return data
        raise ValueError('MissingMerkleNode')

    def put(self,data):
        key=digest(data)
        self.db.execute('INSERT OR IGNORE INTO nodes VALUES(?,?)',(key,data));self.written+=1
        self.recent_writes[key]=data;self.recent_writes.move_to_end(key)
        if len(self.recent_writes)>32768:self.recent_writes.popitem(last=False)
        return key

    def close(self):
        if self.get is not None:self.get.cache_clear()
        self.get=None;self.recent_writes.clear();self.db.close()
        for db in self.readers:db.close()

    def abort(self):
        self.close()
        self.pending.unlink(missing_ok=True)

    def seal(self,root):
        data=self.get(root);count=0 if data is None else population(data)
        self.db.commit();self.close()
        h=file_hash(self.pending);name=h+'.merkle.sqlite';target=self.directory/name
        if target.exists():
            if file_hash(target)!=h:raise ValueError('MerkleExistingObjectCorrupt')
            self.pending.unlink()
        else:os.replace(self.pending,target)
        return dict(schema=SCHEMA,root=root.hex(),count=count,
                    files=self.files+[dict(file=name,sha256=h)],
                    work=dict(nodes_read=self.read_count,nodes_written=self.written))

def population(data):return 1 if data[:1]==b'L' else int.from_bytes(data[-8:],'big')
def bounds(data):return (data[1:33],data[1:33]) if data[:1]==b'L' else (data[67:99],data[99:131])

def update(nodes,root,keys,values,actions):
    """Apply sorted unique key hashes. Actions: 0 insert, 1 replace, 2 retire.

    Arrays are compact S32/S32/u1. Only affected trie paths are visited.
    Every untouched subtree is reused by hash without reading its leaves.
    """
    keys=np.asarray(keys,dtype='S32');values=np.asarray(values,dtype='S32');actions=np.asarray(actions,dtype='u1')
    if len(keys)!=len(values) or len(keys)!=len(actions):raise ValueError('MerkleDeltaShape')
    if len(keys)>1 and np.any(keys[1:]<=keys[:-1]):raise ValueError('DuplicateOrUnorderedMerkleKey')
    if np.any(actions>2):raise ValueError('MerkleAction')
    def raw(array,i):return array[i:i+1].tobytes()
    def change(current,a,b):
        if a==b:return current
        old=nodes.get(current)
        low=raw(keys,a);high=raw(keys,b-1)
        if old is not None:
            first,last=bounds(old);low=min(low,first);high=max(high,last)
        if low==high:
            if b-a!=1:raise ValueError('DuplicateMerkleKey')
            action=int(actions[a])
            if old is None and action!=0:raise ValueError('MissingMerkleKey')
            if old is not None and action==0:raise ValueError('DuplicateMerkleKey')
            return EMPTY if action==2 else nodes.put(b'L'+low+raw(values,a))
        bit=256-(int.from_bytes(low,'big')^int.from_bytes(high,'big')).bit_length()
        boundary=((int.from_bytes(low,'big')>>(256-bit))<<(256-bit))|(1<<(255-bit))
        middle=a+int(np.searchsorted(keys[a:b],np.bytes_(boundary.to_bytes(32,'big'))))
        left=right=EMPTY
        if old is not None:
            if old[:1]==b'B' and int.from_bytes(old[1:3],'big')==bit:left,right=old[3:35],old[35:67]
            elif bounds(old)[0]<boundary.to_bytes(32,'big'):left=current
            else:right=current
        left=change(left,a,middle);right=change(right,middle,b)
        if left==EMPTY:return right
        if right==EMPTY:return left
        ld=nodes.get(left);rd=nodes.get(right)
        return nodes.put(b'B'+bit.to_bytes(2,'big')+left+right+bounds(ld)[0]+bounds(rd)[1]+(population(ld)+population(rd)).to_bytes(8,'big'))
    try:return change(root,0,len(keys))
    finally:change=None

def lookup(nodes,root,key):
    target=key_digest(key)
    while root!=EMPTY:
        data=nodes.get(root)
        if data[:1]==b'L':return data[33:65] if data[1:33]==target else None
        bit=int.from_bytes(data[1:3],'big');right=(target[bit//8]>>(7-bit%8))&1
        root=data[35:67] if right else data[3:35]
    return None


def canonical_root(keys,values):
    """Independent full-population auditor; does not read stored Merkle nodes."""
    keys=np.asarray(keys,dtype='S32');values=np.asarray(values,dtype='S32')
    if len(keys)!=len(values):raise ValueError('MerkleAuditShape')
    if len(keys)>1 and np.any(keys[1:]<=keys[:-1]):raise ValueError('MerkleAuditDuplicateOrOrder')
    def raw(a,i):return a[i:i+1].tobytes()
    def subtree(a,b):
        if a==b:return EMPTY
        low=raw(keys,a);high=raw(keys,b-1)
        if b-a==1:return digest(b'L'+low+raw(values,a))
        split=256-(int.from_bytes(low,'big')^int.from_bytes(high,'big')).bit_length()
        boundary=((int.from_bytes(low,'big')>>(256-split))<<(256-split))|(1<<(255-split))
        middle=a+int(np.searchsorted(keys[a:b],np.bytes_(boundary.to_bytes(32,'big'))))
        left=subtree(a,middle);right=subtree(middle,b)
        return digest(b'B'+split.to_bytes(2,'big')+left+right+low+high+(b-a).to_bytes(8,'big'))
    try:return subtree(0,len(keys))
    finally:subtree=None
