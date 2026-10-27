"""Opt-in complete mixed-face certification with unchanged proof semantics.

Uses the sealed Rust predicates, canonical pair order and legacy proof digest.
Only new candidate pairs are stored. No historical proof population is opened.
"""
from array import array
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import numpy as np
from compact_proof_rows import CompactProofRows
from sdf_cfd_mesher_v4.certificate.efficient import SOURCE, Buffer
from sdf_cfd_mesher_v4.certificate.efficient_proof import PROOF_CLASSES

NATIVE=Path(__file__).with_name('optimized_certificate_native.rs')

class Bytes(ctypes.Structure):
    _fields_=[('data',ctypes.c_void_p),('length',ctypes.c_size_t),('capacity',ctypes.c_size_t)]


class Native:
    def __init__(self):
        self.source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
        digest=hashlib.sha256(NATIVE.read_bytes()+SOURCE.read_bytes()).hexdigest()
        directory=NATIVE.parents[1]/'runtime/bin'
        directory.mkdir(parents=True,exist_ok=True)
        path=directory/(digest+('.dll' if os.name=='nt' else '.so'))
        if not path.exists():
            raise ValueError('InstalledOptimizedCertificateBinaryRequired')
        self.dll=ctypes.CDLL(str(path));p=ctypes.c_void_p;s=ctypes.c_size_t
        self.dll.forest_new.argtypes=[p,s,s];self.dll.forest_new.restype=p
        self.dll.forest_free.argtypes=[p];self.dll.forest_free.restype=None
        self.dll.active_pairs.argtypes=[p,p,s];self.dll.active_pairs.restype=Buffer
        self.dll.buffer_free.argtypes=[Buffer];self.dll.buffer_free.restype=None
        self.dll.parallel_screen.argtypes=[p,p,s,p,s,p,s];self.dll.parallel_screen.restype=None
        self.dll.encode_rows.argtypes=[p,p,s,p,s,p,s,p,s];self.dll.encode_rows.restype=Bytes
        self.dll.bytes_free.argtypes=[Bytes];self.dll.bytes_free.restype=None
        self.dll.ieee_environment.argtypes=[];self.dll.ieee_environment.restype=ctypes.c_uint8
        if not self.dll.ieee_environment():raise ValueError('GradualUnderflowRequired')

    @staticmethod
    def workers(value):
        if type(value) is not int or not 1<=value<=64:raise ValueError('InvalidWorkerCount')
        return value

    def pairs(self,q,active,workers):
        workers=self.workers(workers)
        q=np.ascontiguousarray(q,dtype=np.float64)
        if q.ndim!=3 or q.shape[1:]!=(4,3) or not np.isfinite(q).all() or len(q)>=2**32:
            raise ValueError('InvalidFilterGeometry')
        active=sorted(set(active))
        if any(not isinstance(i,(int,np.integer)) for i in active):raise ValueError('InvalidActiveAABB')
        active=np.asarray(active,dtype=np.int64)
        if len(active) and (active.min()<0 or active.max()>=len(q)):raise ValueError('InvalidActiveAABB')
        if not len(q):return np.empty((0,2),dtype=np.uint32)
        mask=np.zeros(len(q),np.uint8);mask[active]=1
        forest=self.dll.forest_new(q.ctypes.data,len(q),len(q))
        try:
            buffer=self.dll.active_pairs(forest,mask.ctypes.data,workers)
            try:packed=np.ctypeslib.as_array(buffer.data,shape=(buffer.length,)).copy()
            finally:self.dll.buffer_free(buffer)
        finally:self.dll.forest_free(forest)
        if len(packed)>1 and np.any(packed[1:]<=packed[:-1]):raise ValueError('DuplicateOrUnorderedProofPair')
        pairs=np.empty((len(packed),2),np.uint32)
        pairs[:,0]=packed>>32;pairs[:,1]=packed&0xffffffff
        return pairs

    def screen(self,q,labels,pairs,workers):
        workers=self.workers(workers)
        q=np.ascontiguousarray(q,dtype=np.float64);labels=np.ascontiguousarray(labels,dtype=np.uint64)
        raw=np.asarray(pairs)
        if raw.size and (np.any(raw<0) or np.any(raw>=len(q)) or np.any(raw!=raw.astype(np.uint32))):raise ValueError('InvalidFilterPair')
        pairs=np.ascontiguousarray(raw,dtype=np.uint32).reshape(-1,2)
        if q.ndim!=3 or q.shape[1:]!=(4,3) or labels.shape!=(len(q),4) or not np.isfinite(q).all():raise ValueError('InvalidFilterGeometry')
        out=np.empty((len(pairs),3),np.uint8)
        self.dll.parallel_screen(q.ctypes.data,labels.ctypes.data,len(q),pairs.ctypes.data,len(pairs),out.ctypes.data,workers)
        return out


class PackedRows(CompactProofRows):
    def __init__(self,rows=(),*,native_factory=None):
        super().__init__(rows)
        self.native_factory=Native if native_factory is None else native_factory

    def append_classified(self,points,faces,pairs,classified,source_hash,*,fallback_results=None,_preparation=None):
        """Reuse the qualified signature/interner assembly in a bounded batch.

        Preserve first occurrence across batches and exact-fallback payloads.
        Pair/code buffers and payload lookup keep their existing representation.
        """
        pairs=np.asarray(pairs);classified=np.asarray(classified)
        if (pairs.ndim!=2 or pairs.shape[1:]!=(2,) or pairs.dtype.kind not in 'iu' or
            classified.shape!=(len(pairs),3) or classified.dtype!=np.uint8 or
            (pairs.size and (np.any(pairs<0) or np.any(pairs>=len(faces))))):
            raise ValueError('InvalidPackedClassifications')
        fallback_indices=np.flatnonzero(classified[:,0]==0)
        if fallback_results is not None and set(fallback_results)!=set(map(int,fallback_indices)):
            raise ValueError('IncompletePackedExactFallback')
        if not len(pairs):return []
        signatures=classified[:,0].astype(np.uint32)*65536+classified[:,1].astype(np.uint32)*256+classified[:,2]
        if _preparation is None:
            unique,first,inverse=np.unique(signatures,return_index=True,return_inverse=True)
        else:
            from background_preparation import require
            require(_preparation)
            unique,first,inverse=_preparation.partition(signatures,return_index=True,return_inverse=True)
        code_map=np.zeros(len(unique),np.uint64);payloads=[];occurrences=[]
        for u in np.argsort(first):
            signature=int(unique[u]);code=signature>>16;witness=(signature>>8)&255;rotation=signature&255
            if not code:continue
            payload=dict(classification='CertifiedSeparated' if code<=2 else 'CertifiedAllowedIncidentContact',
                method=PROOF_CLASSES[code],filter_code=code,filter_witness=witness,filter_source=source_hash)
            if rotation!=255:payload['filter_rotation']=[rotation//4,rotation%4]
            code_map[u]=len(payloads);payloads.append(payload);occurrences.append(int(first[u]))
        codes=code_map[inverse]
        import termination_geometry_certificate as exact
        bad=[]
        for k in fallback_indices:
            i,j=map(int,pairs[k])
            payload=(exact.pair(points,faces[i],faces[j]) if fallback_results is None else fallback_results[int(k)])
            codes[k]=len(payloads);payloads.append(payload);occurrences.append(int(k))
            if payload['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
                bad.append(dict(pair=[i,j],**payload))
        # Same original JSON normalization; only distinct payloads are encoded.
        remap=np.empty(len(payloads),np.uint64)
        for old in np.argsort(occurrences):
            encoded=json.dumps(payloads[old],sort_keys=True,separators=(',',':'),allow_nan=False)
            if encoded not in self.lookup:
                self.lookup[encoded]=len(self.payloads);self.payloads.append(json.loads(encoded))
            remap[old]=self.lookup[encoded]
        self.pairs.frombytes(pairs.astype(np.uint64).tobytes())
        self.codes.frombytes(remap[codes].tobytes())
        return bad

    def templates(self):
        import generalize_local_remesh as g
        templates=[]
        encode=lambda x:json.dumps(g.s.r.w.clean(x),sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        for payload in self.payloads:
            before=[];after=[]
            for key in sorted(payload):
                (before if key<'pair' else after).append(encode(key)+b':'+encode(payload[key]))
            prefix=b'{'+(b','.join(before)+b',' if before else b'')+b'"pair":['
            suffix=b']'+(b','+b','.join(after) if after else b'')+b'}'
            templates.append((prefix,suffix))
        return templates

    def canonical_chunks(self,limit=16384):
        templates=self.templates()
        for start in range(0,len(self),limit):
            lines=[]
            for i in range(start,min(start+limit,len(self))):
                prefix,suffix=templates[self.codes[i]]
                lines.append(prefix+str(self.pairs[2*i]).encode()+b','+str(self.pairs[2*i+1]).encode()+suffix)
            yield lines

    def encoded_chunks(self,separator,limit=16384):
        native=self.native_factory()
        try:
            data=bytearray();bounds=[]
            for prefix,suffix in self.templates():
                start=len(data);data.extend(prefix);mid=len(data);data.extend(suffix)
                bounds.append((start,mid,len(data)))
            text=bytes(data);bounds=np.asarray(bounds,dtype=np.uint64)
            codes=np.frombuffer(self.codes,dtype=np.uint64)
            pairs=np.frombuffer(self.pairs,dtype=np.uint64)
            if len(pairs)!=2*len(codes) or (len(codes) and codes.max()>=len(bounds)):
                raise ValueError('CorruptPackedProofBuffers')
            for start in range(0,len(self),limit):
                count=min(limit,len(self)-start)
                buffer=native.dll.encode_rows(pairs.ctypes.data+start*16,codes.ctypes.data+start*8,count,
                    ctypes.c_char_p(text),len(text),bounds.ctypes.data,len(bounds),ctypes.c_char_p(separator),len(separator))
                try:yield ctypes.string_at(buffer.data,buffer.length)
                finally:native.dll.bytes_free(buffer)
        finally:
            close=getattr(native,'close',None)
            if close is not None:close()

    def digest(self):
        h=hashlib.sha256();h.update(b'[');first=True
        for block in self.encoded_chunks(b','):
            if not first:h.update(b',')
            h.update(block);first=False
        h.update(b']');return h.hexdigest()

    def write_jsonl(self,path,workers=8,*,newline=None):
        import gzip
        from collections import deque
        from concurrent.futures import ThreadPoolExecutor
        Native.workers(workers)
        # Concatenated gzip members are standard gzip. Independent fixed chunks
        # retain every byte of the legacy decompressed stream, in the same order.
        # At most 2*workers chunks are live. No shared writer or completion order.
        newline=os.linesep.encode() if newline is None else newline
        if type(newline) is not bytes or newline not in (b'\n',b'\r\n'):
            raise ValueError('UnsupportedProofStreamNewline')
        def compress(block):return gzip.compress(block,compresslevel=9,mtime=0)
        with open(path,'wb') as stream,ThreadPoolExecutor(max_workers=workers) as pool:
            pending=deque()
            for block in self.encoded_chunks(newline):
                pending.append(pool.submit(compress,block+newline))
                if len(pending)>=2*workers:stream.write(pending.popleft().result())
            while pending:stream.write(pending.popleft().result())
            stream.flush();os.fsync(stream.fileno())


def certify(points,faces,active,*,workers=8,native_factory=None,row_factory=None,calculate_digest=True):
    if type(calculate_digest) is not bool:raise ValueError('InvalidCertificateDigestControl')
    phases={};tick=time.perf_counter()
    points=np.asarray(points,dtype=np.float64)
    if points.ndim!=2 or points.shape[1:]!=(3,) or not np.isfinite(points).all():raise ValueError('InvalidCertificateCoordinates')
    if any(len(f) not in (3,4) or len(set(f))!=len(f) or
           any(not isinstance(v,(int,np.integer)) for v in f) for f in faces):raise ValueError('UnsupportedCertificatePolygon')
    ids=np.asarray([list(f) if len(f)==4 else list(f)+[f[2]] for f in faces],dtype=np.int64).reshape(-1,4)
    if ids.size and (ids.min()<0 or ids.max()>=len(points)):raise ValueError('InvalidCertificateVertex')
    quad=np.asarray([len(f)==4 for f in faces],dtype=bool);ids[quad]=ids[quad][:,[0,1,3,2]]
    ids=np.ascontiguousarray(ids,dtype=np.uint64);q=np.ascontiguousarray(points[ids])
    native=(Native if native_factory is None else native_factory)();phases['geometry']=time.perf_counter()-tick;tick=time.perf_counter()
    try:
        pairs=native.pairs(q,active,workers)
        phases['broad_phase']=time.perf_counter()-tick;tick=time.perf_counter()
        classified=native.screen(q,ids,pairs,workers)
        phases['predicates_and_rotations']=time.perf_counter()-tick;tick=time.perf_counter()
        rows=(PackedRows if row_factory is None else row_factory)(native_factory=native_factory);bad=rows.append_classified(points,faces,pairs,classified,native.source_hash)
        phases['packed_assembly_and_exact_fallback']=time.perf_counter()-tick;tick=time.perf_counter()
        methods,_=rows.summary();digest=rows.digest() if calculate_digest else None
        phases['summary_and_legacy_hash']=time.perf_counter()-tick
        result=dict(passed=not bad,required_pairs=len(rows),methods=methods,failures=bad,proofs=rows,proof_hash=digest,
            coverage='All AABB-overlapping unique-face pairs touching changed geometry; qualified interval hull filter with unchanged exact polygon fallback',
            optimization=dict(workers=workers,phases=phases,pair_bytes=pairs.nbytes,classification_bytes=classified.nbytes))
        if not calculate_digest:result['proof_digest_retained']=False
        return result
    finally:
        close=getattr(native,'close',None)
        if close is not None:close()
