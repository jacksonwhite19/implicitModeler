"""Byte-identical changed JSON encoding with guarded hash-on-write receipts.

Only immutable changed payloads qualify. Mutable plans, roots, journals and
pointers always use the original durable writer and independent verification.
"""
from pathlib import Path
from contextlib import contextmanager, ExitStack
from unittest.mock import patch
import hashlib
import json
import os
import sys
import time
import immutable_state_access as access
import path_preserving_deduplication as disk

NAMES=frozenset(('GEOMETRY_DELTA.json','CHANGED_INPUTS.json','NEW_QUALITY.json',
    'AFFECTED_INCIDENCE.json','QUALITY_DELTA.json','NEW_ATLAS.json',
    'NEW_FORECAST_DEPENDENCIES.json','BINDING_PATCH.json','NEW_RECEIPT_INPUTS.json'))

@contextmanager
def session():
    old_write=disk.durable_json;old_hash=disk.file_hash
    receipts={};stats={'files':{},'hash_hits':0}
    with ExitStack() as held:
        def write(path,value):
            path=Path(path)
            if path.name not in NAMES:return old_write(path,value)
            key=str(path.resolve())
            if key in receipts:raise ValueError('ImmutableChangedPayloadRewrite')
            start=time.perf_counter()
            # The qualified compact C encoder plus native indent pass preserves every binary64/string/order
            # decision. One bounded changed-component buffer replaces millions
            # of Python TextIO.write calls; this is not a prefix serialization.
            from optimized_durable_json import encoded as canonical_bytes
            data=canonical_bytes(value)
            encoded=time.perf_counter()-start
            digest=hashlib.sha256();pending=path.with_name(path.name+'.pending')
            with pending.open('wb') as stream:
                view=memoryview(data)
                for offset in range(0,len(data),1024*1024):
                    block=view[offset:offset+1024*1024];stream.write(block);digest.update(block)
                stream.flush();os.fsync(stream.fileno())
            disk.atomic_replace(pending,path)
            held.enter_context(access.guard(path));token=access.token(path)
            receipts[key]=(token,digest.hexdigest())
            stats['files'][key]=dict(bytes=len(data),sha256=digest.hexdigest(),encode_seconds=encoded,total_seconds=time.perf_counter()-start)
        def hashed(path):
            key=str(Path(path).resolve()) if Path(path).name in NAMES else None
            if key not in receipts:return old_hash(path)
            token,digest=receipts[key]
            if access.token(path)!=token:raise ValueError('ChangedWriteReceipt')
            stats['hash_hits']+=1
            return digest
        with ExitStack() as patches:
            # Rebind only exact function aliases from repository tools already
            # imported. Later imports get the same scoped disk implementation.
            root=Path(__file__).parent.resolve()
            for module in list(sys.modules.values()):
                file=getattr(module,'__file__',None)
                if not file or Path(file).resolve().parent!=root:continue
                for name,value in list(vars(module).items()):
                    if value is old_write:patches.enter_context(patch.object(module,name,write))
                    elif value is old_hash:patches.enter_context(patch.object(module,name,hashed))
            import composable_transition
            from direct_geometry_delta import append_components
            patches.enter_context(patch.object(composable_transition,'append_components',append_components))
            yield stats
        for key,(token,_) in receipts.items():
            if access.token(key)!=token:raise ValueError('ChangedWriteReceipt')
