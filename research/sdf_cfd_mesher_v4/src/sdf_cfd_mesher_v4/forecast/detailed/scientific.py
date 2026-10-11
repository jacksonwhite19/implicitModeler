"""Canonical scientific values, without automatic field exclusion."""
import hashlib,json
import numpy as np

def plain(value):
    if isinstance(value,np.ndarray):return plain(value.tolist())
    if isinstance(value,np.generic):return plain(value.item())
    if isinstance(value,dict):return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    if isinstance(value,float) and not np.isfinite(value):return dict(nonfinite=repr(value))
    return value

def canonical(value):return (json.dumps(plain(value),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
def scientific_hash(value):return hashlib.sha256(canonical(value)).hexdigest()
