"""Optional aggregate timing hooks; disabled outside the identity study."""
from contextlib import contextmanager
from collections import defaultdict
import time

ROWS=defaultdict(lambda:dict(calls=0,seconds=0.,rows=0))
ENABLED=False

@contextmanager
def measure(name, rows=0):
    if not ENABLED:
        yield
        return
    start=time.perf_counter()
    try:yield
    finally:
        value=ROWS[name];value['calls']+=1
        value['seconds']+=time.perf_counter()-start;value['rows']+=rows

def snapshot():return {k:dict(v) for k,v in ROWS.items()}
