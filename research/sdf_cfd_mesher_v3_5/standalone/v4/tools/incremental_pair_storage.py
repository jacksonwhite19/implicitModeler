"""Changed-state proof storage; scientific proof predicates are unchanged."""
from pathlib import Path
from contextlib import ExitStack
import json,sqlite3,time,math,os
import numpy as np
import immutable_state_access as access
from path_preserving_deduplication import file_hash,durable_json
from sdf_cfd_mesher_v4.common.authority import identity

SCHEMA='V4AuthenticatedPairIncidence/1'

def write_proof_index(path,faces,pairs):
    """Same two unique B-trees and insertion order as the qualified writer.

    After each existing 10,000-row insertion batch, project observed allocated
    pages to the known total row count. Cache capacity only grows. This is a
    measured allocation policy, not a scientific threshold or durability change.
    Its effectiveness is workload-measured; spill remains enabled as a fallback.
    """
    path=Path(path)
    if path.exists():raise ValueError('PreserveExistingProofIndex')
    count=len(pairs);samples=[];db=sqlite3.connect(path)
    try:
        db.execute('PRAGMA synchronous=FULL')
        initial={k:db.execute('PRAGMA '+k).fetchone()[0] for k in ('cache_size','cache_spill','journal_mode','synchronous','page_size')}
        page_size=initial['page_size'];cache=math.ceil(-initial['cache_size']*1024/page_size) if initial['cache_size']<0 else initial['cache_size']
        db.execute('CREATE TABLE proof_keys(key BLOB PRIMARY KEY, ordinal INTEGER NOT NULL UNIQUE) WITHOUT ROWID')
        for start in range(0,count,10000):
            values=[(bytes.fromhex(identity(sorted((faces[int(a)],faces[int(b)])))),start+j) for j,(a,b) in enumerate(pairs[start:start+10000])]
            db.executemany('INSERT INTO proof_keys VALUES (?,?)',values)
            done=start+len(values);pages=db.execute('PRAGMA page_count').fetchone()[0]
            projected=(pages*count+done-1)//done
            cache=max(cache,projected);db.execute('PRAGMA cache_size='+str(cache))
            samples.append(dict(rows=done,pages=pages,cache_pages=cache))
        db.commit()
        if db.execute('SELECT COUNT(*) FROM proof_keys').fetchone()[0]!=count:raise ValueError('IncompleteNewProofIndex')
        if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('CorruptNewProofIndex')
        final={k:db.execute('PRAGMA '+k).fetchone()[0] for k in ('cache_size','cache_spill','journal_mode','synchronous','page_size','page_count')}
    finally:db.close()
    with path.open('r+b') as f:os.fsync(f.fileno())
    return dict(rows=count,initial=initial,final=final,samples=samples,old_proof_payloads_read=0,historical_quality_rows=0)


def build_incidence(base,directory):
    """Explicit one-time deep enrollment of existing immutable pair segments."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    if (directory/'MANIFEST.json').exists():raise ValueError('PreserveExistingIncidence')
    segments=[];files={};seen_segments=set();total=0
    for descriptor,objects,verify in base.proof_segments():
        sid=identity(descriptor)
        if sid in seen_segments:raise ValueError('DuplicateProofSegment')
        seen_segments.add(sid)
        fp=objects/descriptor['faces']['file'];pp=objects/descriptor['pairs']['file']
        verify(fp);verify(pp);faces=access.read(fp);pairs=np.load(pp,mmap_mode='r',allow_pickle=False)
        if len(pairs)!=descriptor['count'] or pairs.shape!=(len(pairs),2):raise ValueError('WrongSegmentCount')
        if len(set(faces))!=len(faces):raise ValueError('DuplicateFaceKey')
        if pairs.size and (int(pairs.min())<0 or int(pairs.max())>=len(faces)):raise ValueError('InvalidPairFace')
        if np.any(pairs[:,0]==pairs[:,1]):raise ValueError('SelfPair')
        # Deep enrollment establishes exact pair uniqueness within each segment.
        ordered=np.sort(np.asarray(pairs,dtype=np.uint64),axis=1)
        keys=ordered[:,0]*np.uint64(len(faces))+ordered[:,1]
        if len(np.unique(keys))!=len(pairs):raise ValueError('DuplicatePairKey')
        del ordered,keys
        face_ids=np.asarray(pairs).reshape(-1)
        order=np.argsort(face_ids,kind='stable')
        ordinals=(order//2).astype(np.uint64 if len(pairs)>np.iinfo(np.uint32).max else np.uint32)
        counts=np.bincount(face_ids.astype(np.int64),minlength=len(faces))
        offsets=np.r_[np.uint64(0),np.cumsum(counts,dtype=np.uint64)]
        op=directory/(sid+'.offsets.npy');ip=directory/(sid+'.ordinals.npy');np.save(op,offsets);np.save(ip,ordinals)
        if any(len(f)!=64 or any(c not in '0123456789abcdef' for c in f) for f in faces):raise ValueError('UnsupportedFaceIdentity')
        face_keys=np.asarray(faces,dtype='S64');face_order=np.argsort(face_keys)
        kp=directory/(sid+'.keys.npy');mp=directory/(sid+'.map.npy');np.save(kp,face_keys[face_order]);np.save(mp,face_order)
        # Verify every posting by direct reconstruction before authentication.
        if not np.array_equal(face_ids[order],np.repeat(np.arange(len(faces)),counts)):raise ValueError('IncidenceReconstructionMismatch')
        for p in (op,ip,kp,mp):
            with p.open('r+b') as f:os.fsync(f.fileno())
            files[str(p.resolve())]=access.attest(p,file_hash(p))
        files[str(fp.resolve())]=access.attest(fp,descriptor['faces']['sha256'])
        segments.append(dict(identity=sid,descriptor=descriptor,faces=str(fp.resolve()),offsets=str(op.resolve()),ordinals=str(ip.resolve()),keys=str(kp.resolve()),mapping=str(mp.resolve()),count=len(pairs),face_count=len(faces)))
        total+=len(pairs)
        del pairs,face_ids,order,ordinals,counts,offsets
    if total!=len(base['certificate']):raise ValueError('WrongParentLiveCount')
    body=dict(schema=SCHEMA,parent_content=base.content_root,parent_count=total,segments=segments,files=files,epoch=access.epoch(),implementation=file_hash(__file__),policy='Immutable disjoint append-only qualified proof segments; removals require a separately qualified index update')
    durable_json(directory/'MANIFEST.json',body)
    return identity(body)


class PairIncidence:
    def __init__(self,base,path,root):
        self.stack=ExitStack();self.stats=dict(prior_pair_rows_scanned=0,incidence_postings=0,face_probes=0,components=0)
        try:
            path=Path(path);self.path=path.resolve();self.stack.enter_context(access.guard(path));self.body=access.read(path)
            b=self.body
            if identity(b)!=root or b['schema']!=SCHEMA:raise ValueError('WrongIncidenceRoot')
            if b['epoch']!=access.epoch():raise ValueError('StaleIncidenceEpoch')
            if b['implementation']!=file_hash(__file__):raise ValueError('WrongIncidenceImplementation')
            if b['parent_content']!=base.content_root:raise ValueError('WrongIncidenceParent')
            if b['parent_count']!=len(base['certificate']) or sum(s['count'] for s in b['segments'])!=b['parent_count']:raise ValueError('WrongParentLiveCount')
            actual=[identity(d) for d,_,_ in base.proof_segments()]
            if actual!=[s['identity'] for s in b['segments']] or len(set(actual))!=len(actual):raise ValueError('IncidenceAncestryMismatch')
            for p,r in b['files'].items():
                self.stack.enter_context(access.guard(p))
                if access.token(p)!=r['token']:raise ValueError('IncidenceComponentMutation')
            self.base=base;self.root=root
        except BaseException:self.close();raise
    def close(self):self.stack.close()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def affected(self,bindings,removed=()):
        if removed:raise ValueError('UnqualifiedIncidenceRemoval')
        self.base.check();changed=set()
        for key in bindings:
            if not key.startswith('face:') or key[5:] not in self.base['faces']:raise ValueError('MissingChangedFace')
            changed.add(key[5:])
        result=0
        for s in self.body['segments']:
            keys=np.load(s['keys'],mmap_mode='r',allow_pickle=False);mapping=np.load(s['mapping'],mmap_mode='r',allow_pickle=False)
            offsets=np.load(s['offsets'],mmap_mode='r',allow_pickle=False);ordinals=np.load(s['ordinals'],mmap_mode='r',allow_pickle=False)
            if len(offsets)!=s['face_count']+1 or len(keys)!=s['face_count'] or len(mapping)!=len(keys) or int(offsets[-1])!=2*s['count'] or len(ordinals)!=2*s['count']:raise ValueError('IncompleteIncidence')
            chunks=[]
            for f in changed:
                self.stats['face_probes']+=1
                k=f.encode('ascii');position=int(np.searchsorted(keys,k))
                if position==len(keys) or keys[position]!=k:continue
                i=int(mapping[position])
                values=ordinals[int(offsets[i]):int(offsets[i+1])]
                self.stats['incidence_postings']+=len(values);chunks.append(values)
            result+=len(np.unique(np.concatenate(chunks))) if chunks else 0
            self.stats['components']+=1
        if result>self.body['parent_count']:raise ValueError('InconsistentAffectedCount')
        return dict(changed=result,unchanged=self.body['parent_count']-result,parent_count=self.body['parent_count'],index_root=self.root,index_manifest=str(self.path),**self.stats)
