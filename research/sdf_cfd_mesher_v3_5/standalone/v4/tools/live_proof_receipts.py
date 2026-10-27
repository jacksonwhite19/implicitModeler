"""Reconstruct current receipt bindings from immutable compact proofs.

Used at the conservative mixed/correction handoff, never during nominal append.
Random lookup builds a compact digest/ordinal index, not a dictionary of proofs.
"""
from collections.abc import Mapping
from copy import deepcopy
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity


class LiveProofReceipts(Mapping):
    def __init__(self,proofs,face_keys,inputs):
        self.proofs=proofs;self.face_keys=tuple(face_keys);self.inputs=dict(inputs);self.index=None

    def __len__(self):return len(self.proofs)

    def receipt(self,row):
        faces=sorted(self.face_keys[i] for i in row['pair'])
        deps=['face:'+f for f in faces]+['authority:certificate']
        if 'authority:certificate_filter' in self.inputs:deps.append('authority:certificate_filter')
        return identity(faces),dict(faces=faces,inputs={k:self.inputs[k] for k in deps},value={k:v for k,v in row.items() if k!='pair'})

    def items(self):
        for row in self.proofs:yield self.receipt(row)

    def __iter__(self):
        for k,_ in self.items():yield k

    def _ensure_index(self):
        if self.index is None:
            rows=np.empty(len(self),dtype=[('key','S32'),('segment','u4'),('row','u8')]);i=0
            for segment,s in enumerate(self.proofs.segments):
                for j,row in enumerate(s.rows()):
                    k,_=self.receipt(row);rows[i]=(bytes.fromhex(k),segment,j);i+=1
            for j,row in enumerate(self.proofs.fresh):
                k,_=self.receipt(row);rows[i]=(bytes.fromhex(k),len(self.proofs.segments),j);i+=1
            rows.sort(order='key');self.index=rows
        return self.index

    def _row(self,entry):
        segment=int(entry['segment']);j=int(entry['row'])
        if segment==len(self.proofs.segments):return self.proofs.fresh[j]
        s=self.proofs.segments[segment];a,b=s.pairs[j]
        return dict(deepcopy(s.payloads[int(s.codes[j])]),pair=[int(s.mapping[a]),int(s.mapping[b])])

    def __getitem__(self,key):
        self._ensure_index()
        try:digest=bytes.fromhex(key)
        except ValueError:raise KeyError(key) from None
        i=int(np.searchsorted(self.index['key'],np.asarray(digest,dtype='S32')))
        if i==len(self.index) or self.index['key'][i]!=np.asarray(digest,dtype='S32'):raise KeyError(key)
        return self.receipt(self._row(self.index[i]))[1]

    def native_canonical_chunks(self):
        """Exact existing canonical JSON bytes, in scientific-key order.

        Stream indexed proof ordinals directly. Do not sort expanded strings
        and then perform millions of random map lookups to encode the same map.
        This changes neither the digest representation nor its authority version.
        """
        from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER
        yield b'{';previous=None
        for entry in self._ensure_index():
            key,value=self.receipt(self._row(entry))
            if previous is not None:
                if key<=previous:raise ValueError('DuplicateOrMisorderedLiveProofKey')
                yield b','
            yield ENCODER.encode(key).encode();yield b':';yield ENCODER.encode(value).encode()
            previous=key
        yield b'}'

    def __deepcopy__(self,memo):
        # Proof storage and dependency digests are immutable in this adapter.
        return self
