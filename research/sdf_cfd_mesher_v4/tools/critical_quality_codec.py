"""Bounded ordered canonical compression controls; schema and bytes unchanged."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from multiprocessing import get_context
import zlib
import time
from incremental_transaction import canonical
STATS = {}


def batch(rows):
    cpu = time.thread_time()
    values = [(zlib.compress(canonical(r['inputs'])), None if r['payload'] is None else zlib.compress(canonical(r['payload']))) for r in rows]
    return values, time.thread_time()-cpu


def encoded(rows, workers=1, process=False):
    if type(workers) is not int or workers not in (1, 2, 4):
        raise ValueError('InvalidQualityCodecWorkers')
    STATS.clear(); STATS.update(workers=workers, process=process, batches=0, rows=0, worker_cpu=0.)
    def consume(result):
        values,cpu=result
        STATS['batches']+=1;STATS['rows']+=len(values);STATS['worker_cpu']+=cpu
        return values
    if workers == 1:
        for start in range(0, len(rows), 2048):
            yield from consume(batch(rows[start:start+2048]))
        return
    executor = ProcessPoolExecutor(workers, mp_context=get_context('spawn')) if process else ThreadPoolExecutor(workers)
    with executor as pool:
        pending = deque()
        for start in range(0, len(rows), 2048):
            pending.append(pool.submit(batch, rows[start:start+2048]))
            if len(pending) >= 2*workers:
                yield from consume(pending.popleft().result())
        while pending:
            yield from consume(pending.popleft().result())
