"""Bounded-memory encoding of the existing canonical scientific JSON bytes."""
import hashlib,json,math
import numpy as np

def _tokens(value):
    if isinstance(value,np.ndarray):
        if value.ndim==0:yield from _tokens(value.item());return
        yield '['
        for i,row in enumerate(value):
            if i:yield ','
            yield from _tokens(row)
        yield ']';return
    if isinstance(value,np.generic):yield from _tokens(value.item());return
    if isinstance(value,dict):
        data=value if all(isinstance(k,str) for k in value) else {str(k):v for k,v in value.items()}
        yield '{'
        for i,key in enumerate(sorted(data)):
            if i:yield ','
            yield json.dumps(key);yield ':';yield from _tokens(data[key])
        yield '}';return
    if isinstance(value,(tuple,list)):
        yield '['
        for i,row in enumerate(value):
            if i:yield ','
            yield from _tokens(row)
        yield ']';return
    if isinstance(value,float) and not math.isfinite(value):
        yield from _tokens(dict(nonfinite=repr(value)));return
    yield json.dumps(value,allow_nan=False,separators=(',',':'))

def canonical_chunks(value,buffer_bytes=65536):
    pending=[];size=0
    for token in _tokens(value):
        pending.append(token);size+=len(token)
        if size>=buffer_bytes:
            yield ''.join(pending).encode();pending=[];size=0
    pending.append('\n');yield ''.join(pending).encode()

def scientific_hash(value):
    h=hashlib.sha256()
    for block in canonical_chunks(value):h.update(block)
    return h.hexdigest()
