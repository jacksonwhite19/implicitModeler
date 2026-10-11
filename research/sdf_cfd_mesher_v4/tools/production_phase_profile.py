"""Streaming phase/RSS observations; never retains scientific arguments/results."""
import contextlib
import functools
import json
import threading
import time
from pathlib import Path
import psutil


class Profile:
    def __init__(self, path, interval=0.5):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.start = time.perf_counter()
        self.stack = ['process']
        self.process = psutil.Process()
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.interval = interval
        self.thread = threading.Thread(target=self.sample, daemon=True)
        self.thread.start()

    def record(self, event, **extra):
        with self.lock:
            mem = self.process.memory_info()
            io = self.process.io_counters()
            row = dict(seconds=time.perf_counter()-self.start, event=event,
                       phase=list(self.stack), rss_bytes=mem.rss,
                       peak_rss_bytes=getattr(mem, 'peak_wset', mem.rss),
                       io_read_bytes=io.read_bytes,io_write_bytes=io.write_bytes,**extra)
            with self.path.open('a', encoding='utf8') as out:
                out.write(json.dumps(row, separators=(',', ':'))+'\n')

    def sample(self):
        while not self.stop.wait(self.interval):
            self.record('sample')

    @contextlib.contextmanager
    def phase(self, name):
        with self.lock:
            self.stack.append(name)
        self.record('enter')
        started = time.perf_counter()
        try:
            yield
        finally:
            self.record('exit', duration_seconds=time.perf_counter()-started)
            with self.lock:
                self.stack.pop()

    def wrap(self, function, name):
        @functools.wraps(function)
        def measured(*args, **kwargs):
            with self.phase(name):
                return function(*args, **kwargs)
        return measured

    def close(self):
        self.stop.set()
        self.thread.join()
        self.record('end')
