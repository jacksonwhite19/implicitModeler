"""Reuse canonical face-string encoding; retain the qualified SQLite writer."""
import hashlib
import json
from unittest.mock import patch
import incremental_pair_storage as storage

QUALIFIED_WRITER=storage.write_proof_index


def write_proof_index(path,faces,pairs):
    encoded={face:json.dumps(face,sort_keys=True,separators=(',',':'),allow_nan=False).encode() for face in faces}
    original=storage.identity
    def identity(value):
        if isinstance(value,list) and len(value)==2 and all(isinstance(x,str) and x in encoded for x in value):
            return hashlib.sha256(b'['+encoded[value[0]]+b','+encoded[value[1]]+b']').hexdigest()
        return original(value)
    with patch.object(storage,'identity',identity):return QUALIFIED_WRITER(path,faces,pairs)
