"""Canonical identities limited to explicit geometry and implementation inputs."""
from pathlib import Path
import hashlib,json
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
REPO=ROOT.parents[1]
def file_hash(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def array_hash(value):
    # hashlib consumes the contiguous buffer directly. tobytes() duplicated
    # complete certificate geometry and forecast arrays only to discard them.
    return hashlib.sha256(np.ascontiguousarray(value)).hexdigest()
def identity(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def frozen(value,dtype):
    a=np.asarray(value,dtype=dtype)
    return np.frombuffer(a.tobytes(),dtype=a.dtype).reshape(a.shape)
def references():return json.loads((ROOT/'authorities/DATA_AUTHORITIES.json').read_text())['references']
def reference(role):
    r=next(r for r in references() if r['role']==role);p=REPO/r['path']
    if p.is_file() and file_hash(p)!=r['sha256']:raise ValueError('AuthorityHashMismatch:'+role)
    return p,r['sha256']
