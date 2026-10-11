"""Immutable compact proof segments for ordinary, unchanged-face extension.

Scientific receipts remain the authority. These authenticated storage indices
avoid decoding every old JSON receipt to render the legacy ordered proof list.
Every new/changed geometric relation still goes through the same exact engine.
"""
from collections import Counter
from copy import deepcopy
from heapq import merge
from pathlib import Path
from itertools import islice
import json
import os
import tempfile
import numpy as np
from compact_proof_rows import CompactProofRows
from sdf_cfd_mesher_v4.common.authority import file_hash, identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import receipt_table_from_rows


def save_object(directory, value, array=False):
    directory=Path(directory)
    fd,name=tempfile.mkstemp(prefix='proof-index-',suffix='.pending',dir=directory)
    os.close(fd)
    temporary=Path(name)
    try:
        with temporary.open('wb') as stream:
            if array:np.save(stream,value,allow_pickle=False)
            else:stream.write(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode())
        digest=file_hash(temporary)
        target=directory/(digest+('.npy' if array else '.json'))
        if target.exists():
            if file_hash(target)!=digest:raise ValueError('CorruptProofIndexObject')
            temporary.unlink()
        else:os.replace(temporary,target)
        return dict(file=target.name,sha256=digest)
    finally:
        if temporary.exists():temporary.unlink()


def persist_segment(rows, faces, directory):
    pairs=np.frombuffer(rows.pairs,dtype=np.uint64).reshape(-1,2)
    used=np.unique(pairs)
    local=np.searchsorted(used,pairs).astype(np.uint64)
    methods,bad=rows.summary()
    if bad:raise ValueError('FailedProofSegment')
    result=dict(count=len(rows),methods=dict(methods),
                faces=save_object(directory,[faces[int(i)] for i in used]),
                pairs=save_object(directory,local,True),
                codes=save_object(directory,np.frombuffer(rows.codes,dtype=np.uint64),True),
                payloads=save_object(directory,rows.payloads))
    result['identity']=identity(result)
    return result


def segment_objects(segment):
    return [segment[k] for k in ('faces','pairs','codes','payloads')]+list(segment.get('dependency',{}).values())


def add_dependency_index(descriptor,directory):
    if 'dependency' in descriptor:return descriptor
    from proof_dependency_index import build
    directory=Path(directory)
    pairs=np.load(directory/descriptor['pairs']['file'],mmap_mode='r',allow_pickle=False)
    faces=json.loads((directory/descriptor['faces']['file']).read_text())
    offsets,incidence=build(pairs,len(faces))
    result={k:v for k,v in descriptor.items() if k!='identity'}
    result['dependency']=dict(offsets=save_object(directory,offsets,True),incidence=save_object(directory,incidence,True))
    codes=np.load(directory/descriptor['codes']['file'],mmap_mode='r',allow_pickle=False)
    result['index_bounds']=dict(pair_max=int(pairs.max()) if len(pairs) else -1,code_max=int(codes.max()) if len(codes) else -1)
    result['identity']=identity(result)
    return result


class LoadedSegment:
    def __init__(self, descriptor, directory, indices, authenticated=False):
        self.descriptor=descriptor
        body=dict(descriptor)
        if body.pop('identity')!=identity(body):raise ValueError('ProofSegmentIdentity')
        directory=Path(directory)
        for row in segment_objects(descriptor):
            if Path(row['file']).name!=row['file']:raise ValueError('ProofSegmentObjectCorrupt')
            if authenticated:
                from sdf_cfd_mesher_v4.advancing_front.verified_objects import verify
                verify(directory/row['file'],row['sha256'])
            elif file_hash(directory/row['file'])!=row['sha256']:raise ValueError('ProofSegmentObjectCorrupt')
        self.faces=json.loads((directory/descriptor['faces']['file']).read_text())
        self.mapping=np.asarray([indices[k] for k in self.faces],dtype=np.uint64)
        if len(self.mapping)>1 and not np.all(self.mapping[1:]>self.mapping[:-1]):
            raise ValueError('ProofSegmentOrderChanged')
        self.pairs=np.load(directory/descriptor['pairs']['file'],mmap_mode='r',allow_pickle=False)
        self.codes=np.load(directory/descriptor['codes']['file'],mmap_mode='r',allow_pickle=False)
        self.payloads=json.loads((directory/descriptor['payloads']['file']).read_text())
        if 'dependency' in descriptor:
            self.dependency={k:np.load(directory/v['file'],mmap_mode='r',allow_pickle=False)
                             for k,v in descriptor['dependency'].items()}
        if self.pairs.dtype!=np.dtype('uint64') or self.codes.dtype!=np.dtype('uint64'):
            raise ValueError('ProofSegmentDtype')
        if self.pairs.shape!=(descriptor['count'],2) or self.codes.shape!=(descriptor['count'],):
            raise ValueError('ProofSegmentShape')
        bounds=descriptor.get('index_bounds') if authenticated else None
        if bounds is None:bounds=dict(pair_max=int(self.pairs.max()) if len(self.pairs) else -1,code_max=int(self.codes.max()) if len(self.codes) else -1)
        if len(self.pairs) and (bounds['pair_max']>=len(self.faces) or bounds['code_max']>=len(self.payloads)):
            raise ValueError('ProofSegmentIndex')

    def rows(self):
        for pair,code in zip(self.pairs,self.codes):
            value={k:deepcopy(v) if isinstance(v,(dict,list)) else v
                   for k,v in self.payloads[int(code)].items()}
            value['pair']=[int(self.mapping[pair[0]]),int(self.mapping[pair[1]])]
            yield value


class SegmentProofRows(list):
    def __init__(self, segments, fresh):
        self.segments=segments
        self.fresh=fresh
    def __len__(self):
        return sum(s.descriptor['count'] for s in self.segments)+len(self.fresh)
    def __iter__(self):
        previous=None
        for row in merge(*(s.rows() for s in self.segments),iter(self.fresh),key=lambda r:r['pair']):
            if previous is not None and row['pair']<=previous:raise ValueError('UnorderedOrDuplicateProofSegment')
            previous=row['pair']
            yield row
    def __getitem__(self,index):
        if isinstance(index,slice):
            start,stop,step=index.indices(len(self))
            if step<0:return list(self)[index]
            return list(islice(iter(self),start,stop,step))
        if index<0:index+=len(self)
        if not 0<=index<len(self):raise IndexError(index)
        return next(islice(iter(self),index,index+1))
    def __eq__(self,other):
        return hasattr(other,'__len__') and len(self)==len(other) and all(a==b for a,b in zip(self,other))
    def __ne__(self,other):return not self==other
    def __deepcopy__(self,memo):return type(self)(self.segments,deepcopy(self.fresh,memo))
    def summary(self):
        methods,bad=self.fresh.summary()
        for segment in self.segments:methods.update(segment.descriptor['methods'])
        return methods,bad
    def pair_iterator(self):
        def pairs(segment):
            for a,b in segment.pairs:
                yield int(segment.mapping[a]),int(segment.mapping[b])
        yield from merge(*(pairs(s) for s in self.segments),self.fresh.pair_iterator())
    def storage(self):
        return dict(records=len(self),disk_backed_historical_records=sum(s.descriptor['count'] for s in self.segments),
                    mapped_pair_bytes=sum(s.pairs.nbytes for s in self.segments),
                    mapped_code_bytes=sum(s.codes.nbytes for s in self.segments),fresh=self.fresh.storage())


def extension_plan(base, before, old_inputs, new_inputs, unchanged, indices):
    if base is None or getattr(base,'qualified_context',None)!=identity(old_inputs):return None
    if hasattr(base,'authenticated') and any(
            old_inputs.get(k)!=new_inputs.get(k)
            for k in ('authority:certificate','authority:certificate_filter')):
        raise ValueError('CertificateAuthorityChangedRequiresRecertification')
    descriptors=getattr(base,'proof_segments',None)
    if not descriptors or not set(before['faces']).issubset(unchanged):return None
    try:
        segments=[LoadedSegment(s,base.proof_objects,indices,hasattr(base,'authenticated')) for s in descriptors]
    except ValueError as exc:
        if str(exc)=='ProofSegmentOrderChanged':return None
        raise
    if sum(s.descriptor['count'] for s in segments)!=len(base):raise ValueError('ProofSegmentCoverage')
    changed={k for k,v in old_inputs.items() if new_inputs.get(k)!=v}
    touched=0;incidences_read=0
    def patches():
        nonlocal touched,incidences_read
        for segment in segments:
            marked=np.asarray(['face:'+f in changed for f in segment.faces],dtype=bool)
            from identity_timing import measure
            if hasattr(base,'authenticated'):
                if not hasattr(segment,'dependency'):raise ValueError('MissingAuthenticatedDependencyIndex')
                from proof_dependency_index import affected as query
                ids=np.flatnonzero(marked);offsets=segment.dependency['offsets']
                incidences_read+=sum(int(offsets[f+1]-offsets[f]) for f in ids)
                with measure('dependency_index_query'):
                    affected=query(segment.dependency['offsets'],segment.dependency['incidence'],np.flatnonzero(marked))
            else:
                with measure('compact_index_scan',len(segment.pairs)):
                    affected=np.flatnonzero(marked[segment.pairs[:,0]]|marked[segment.pairs[:,1]])
            for i in affected:
                a,b=map(int,segment.pairs[i])
                faces=sorted((segment.faces[a],segment.faces[b]))
                deps=['face:'+f for f in faces]+['authority:certificate']
                if 'authority:certificate_filter' in new_inputs:deps.append('authority:certificate_filter')
                value=segment.payloads[int(segment.codes[i])]
                touched+=1
                yield identity(faces),dict(faces=faces,inputs={k:new_inputs[k] for k in deps},value=value)
    from delta_proof_binding import patch_table
    updates,owner=patch_table(base,patches())
    return dict(base=base,patches=updates,retired=[],context=identity(new_inputs),inherited_coverage=True,
                segments=segments,historical_receipts_rebound=touched,dependency_index_entries_read=incidences_read,scratch_owner=owner)


def persist_population(rows, face_keys, directory):
    plan=getattr(rows,'native_delta',{})
    if 'sealed_segments' in plan:return plan['sealed_segments']
    if isinstance(rows,SegmentProofRows):
        descriptors=[s.descriptor for s in rows.segments]
        if len(rows.fresh):
            segment=persist_segment(rows.fresh,face_keys,directory)
            if hasattr(getattr(rows,'native_delta',{}).get('base'),'authenticated'):segment=add_dependency_index(segment,directory)
            descriptors.append(segment)
        return descriptors
    if isinstance(rows,CompactProofRows):
        segment=persist_segment(rows,face_keys,directory)
        if hasattr(plan.get('base'),'authenticated'):segment=add_dependency_index(segment,directory)
        return [segment]
    return None
