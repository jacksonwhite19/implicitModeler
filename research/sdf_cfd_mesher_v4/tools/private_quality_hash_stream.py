"""Bounded private preprocessing for the existing two quality hash streams.

No format, persisted witness or reader changes. Same canonical function and
pretty JSON settings/framing as single_quality_component.hashes. Use the
qualified codec's ordered 2048-row / 2*workers executor pattern.
"""
from collections import deque
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import hashlib
import json
import zlib
from multiprocessing import get_context

from incremental_transaction import canonical


def source_bytes(rows):
    """Bind caller values BEFORE pickle can invoke user-defined reducers."""
    result=[]
    for row in rows:
        if not isinstance(row,dict) or set(row)!={'key','inputs','payload'}:
            raise ValueError('UnsupportedChangedQualityRow')
        if not isinstance(row['key'],str) or not isinstance(row['inputs'],dict):
            raise ValueError('InvalidChangedQualityRow')
        if row['payload'] is not None and not isinstance(row['payload'],dict):
            raise ValueError('InvalidChangedQualityPayload')
        result.append(canonical(row))
    return result


def batch(values, with_codec=False):
    result=[]
    for data in values:
        snapshot=json.loads(data)
        if canonical(snapshot)!=data:raise ValueError('NoncanonicalPrivateQualityInput')
        # Both streams name ONE detached snapshot, not mutable caller aliases.
        text=json.dumps(snapshot,indent=2,sort_keys=True,allow_nan=False)
        pretty=('  '+text.replace('\n','\n  ')).encode('utf-8')
        if with_codec:
            # Independently derive the exact existing codec bytes from the SAME
            # accepted detached snapshot. Never serialize caller aliases here.
            blobs=(zlib.compress(canonical(snapshot['inputs'])),
                   None if snapshot['payload'] is None else zlib.compress(canonical(snapshot['payload'])))
            result.append((data,pretty,(snapshot['key'],blobs)))
        else:result.append((data,pretty))
    return result


def ordered(rows,workers,process,with_codec=False):
    if type(workers) is not int or workers not in (1,2,4):raise ValueError('InvalidQualityCodecWorkers')
    if type(process) is not bool:raise ValueError('InvalidQualityCodecProcess')
    if workers==1:
        for start in range(0,len(rows),2048):yield from batch(source_bytes(rows[start:start+2048]),with_codec)
        return
    executor=ProcessPoolExecutor(workers,mp_context=get_context('spawn')) if process else ThreadPoolExecutor(workers)
    with executor as pool:
        pending=deque()
        for start in range(0,len(rows),2048):
            pending.append(pool.submit(batch,source_bytes(rows[start:start+2048]),with_codec))
            if len(pending)>=2*workers:yield from pending.popleft().result()
        while pending:yield from pending.popleft().result()


def hashes(rows,accepted,workers=1,process=False,*,codec_witness=None):
    compact=hashlib.sha256(); pretty=hashlib.sha256(); size=0;count=0
    def put(data):
        nonlocal size
        pretty.update(data);size+=len(data)
    compact.update(b'[')
    for result in ordered(rows,workers,process,codec_witness is not None):
        data,text=result[:2]
        if count:compact.update(b',');put(b',\n')
        else:put(b'[\n')
        compact.update(data);put(text);accepted.append(data);count+=1
        if codec_witness is not None:codec_witness.append(result[2])
    compact.update(b']');put(b'\n]\n' if count else b'[]\n')
    if count!=len(rows):raise ValueError('IncompletePrivateQualityHashStream')
    return dict(identity=compact.hexdigest(),sha256=pretty.hexdigest(),bytes=size,count=count)
