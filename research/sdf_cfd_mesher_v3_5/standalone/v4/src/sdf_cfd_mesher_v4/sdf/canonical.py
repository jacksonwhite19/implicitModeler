"""Owned canonical-kernel connection. No shape construction or sampled fields.

The process protocol is deliberately sequential. A lock covers each request and
its response. Each host admits at most one Rhai construction to isolate existing
kernel helper caches. Trusted standalone scripts only; this is not a sandbox.
"""
from __future__ import annotations
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
import json
import io
import math
import queue
import subprocess
import threading


class GeometryFailure(RuntimeError):
    def __init__(self, code, stage, detail, geometry_identity=None, **fields):
        self.record = dict(code=code, stage=stage, detail=detail,
                           geometry_identity=geometry_identity, **fields)
        super().__init__(json.dumps(self.record, sort_keys=True))


class LineProcess:
    """Bounded lines, drained stderr, deadlines, and explicit process ownership."""
    def __init__(self, command, timeout=30):
        self.timeout = timeout
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, bufsize=0)
        # Preserve interactive raw writes, but read stdout in chunks. FileIO's
        # inherited readline otherwise performs one raw read per byte. The same
        # bounded line framing, deadline and process owner remain in force.
        self.process.stdout = io.BufferedReader(self.process.stdout)
        self.lines = queue.Queue(maxsize=128)
        self.errors = deque(maxlen=32)
        self.closed = threading.Event()
        def stdout():
            try:
                while not self.closed.is_set():
                    line = self.process.stdout.readline(8*1024*1024+1)
                    item = line if len(line) <= 8*1024*1024 else b'PROTOCOL_LINE_TOO_LARGE'
                    while not self.closed.is_set():
                        try:
                            self.lines.put(item, timeout=.1)
                            break
                        except queue.Full:
                            continue
                    if not line or len(line)>8*1024*1024:
                        break
            except (OSError, ValueError):
                pass
        def stderr():
            try:
                while data := self.process.stderr.read(4096):
                    self.errors.append(data.decode('utf8', errors='replace'))
            except (OSError, ValueError):
                pass
        self.readers = [threading.Thread(target=f, daemon=True) for f in (stdout, stderr)]
        for reader in self.readers:
            reader.start()

    def read(self, timeout=None):
        try:
            line = self.lines.get(timeout=self.timeout if timeout is None else max(.001, timeout))
        except queue.Empty:
            self.close()
            raise GeometryFailure('HostTimeout', 'transport', 'process terminated after deadline')
        if not line:
            raise GeometryFailure('HostClosed', 'transport', ''.join(self.errors))
        if line == b'PROTOCOL_LINE_TOO_LARGE':
            self.close()
            raise GeometryFailure('ResponseTooLarge', 'transport', '8 MiB limit')
        return line.decode('utf8').rstrip('\r\n')

    def write(self, line):
        if self.closed.is_set():
            raise GeometryFailure('HostClosed', 'transport', 'owner closed')
        try:
            self.process.stdin.write((line+'\n').encode('utf8'))
            self.process.stdin.flush()
        except (BrokenPipeError, OSError):
            raise GeometryFailure('HostClosed', 'transport', ''.join(self.errors))

    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            stream.close()


class CanonicalKernel:
    def __init__(self, command, timeout=30):
        self.transport = LineProcess(command, timeout)
        self.lock = threading.RLock()
        self._protected_handles = {}
        try:
            self.ready = json.loads(self.transport.read())
            if self.ready.get('ready') != 'V4CanonicalKernel/1':
                raise ValueError('wrong protocol')
        except Exception:
            self.transport.close()
            raise

    def request(self, **request):
        with self.lock:
            if (request.get('op')=='load_rhai' and self._protected_handles or
                    request.get('op')=='release' and request.get('handle') in self._protected_handles):
                raise GeometryFailure('ProtectedGeometryScope','custody','live geometry is held read-only')
            try:
                line = json.dumps(request, allow_nan=False, separators=(',', ':'))
            except (ValueError, TypeError) as e:
                raise GeometryFailure('InvalidRequest', 'client_serialization', str(e))
            if len(line.encode('utf8'))+1 > 8*1024*1024:
                raise GeometryFailure('RequestTooLarge', 'client_serialization', '8 MiB limit')
            self.transport.write(line)
            try:
                reply = json.loads(self.transport.read())
                if not isinstance(reply.get('ok'), bool):
                    raise ValueError('missing status')
            except (ValueError, AttributeError) as e:
                self.transport.close()
                raise GeometryFailure('ProtocolCorruption', 'transport', str(e))
            if not reply['ok']:
                raise GeometryFailure(**reply['error'])
            return reply['result']

    @contextmanager
    def protect_handle(self,handle):
        """Hold the existing request lock and deny native handle mutation."""
        with self.lock:
            self.request(op='describe',handle=handle)
            holds=self._protected_handles
            holds[handle]=holds.get(handle,0)+1
            try:yield
            finally:
                remaining=holds[handle]-1
                if remaining:holds[handle]=remaining
                else:del holds[handle]

    def load_rhai(self, script, metres_per_model_unit):
        result = self.request(op='load_rhai', script=script, metres_per_model_unit=metres_per_model_unit)
        return CanonicalGeometry(self, result['handle'], result['description'], result['load_seconds'])

    def initial_geometry(self):
        handle = self.ready.get('initial_handle')
        if handle is None:
            raise GeometryFailure('NoInitialHandle', 'admission', 'caller must load geometry')
        return CanonicalGeometry(self, handle, self.request(op='describe', handle=handle), None)

    def close(self):
        with self.lock:
            if self._protected_handles:
                raise GeometryFailure('ProtectedGeometryScope','custody','live geometry is held read-only')
            self.transport.close()

    def __enter__(self): return self
    def __exit__(self, *_): self.close()


@dataclass
class CanonicalGeometry:
    owner: CanonicalKernel
    handle: int
    description: dict
    load_seconds: float | None = None

    @property
    def identity(self): return self.description['geometry_identity']

    def query(self, points_m):
        return self.owner.request(op='query', handle=self.handle, points_m=points_m)['values_m']

    def query_report(self, points_m):
        """Same owned request, retaining the native timing for diagnostics."""
        return self.owner.request(op='query', handle=self.handle, points_m=points_m)

    def surface_seed(self, *, padding_m, maximum_queries, timeout_seconds, spacing_m=None, cells_across=None):
        return self._seed_operation('surface_seed',padding_m,maximum_queries,timeout_seconds,spacing_m,cells_across)

    def surface_seed_plan(self, *, padding_m, maximum_queries, timeout_seconds, spacing_m=None, cells_across=None):
        return self._seed_operation('surface_seed_plan',padding_m,maximum_queries,timeout_seconds,spacing_m,cells_across)

    def _seed_operation(self,operation,padding_m,maximum_queries,timeout_seconds,spacing_m,cells_across):
        values=dict(padding_m=padding_m,maximum_queries=maximum_queries,timeout_seconds=timeout_seconds)
        if spacing_m is not None:values['spacing_m']=spacing_m
        if cells_across is not None:values['cells_across']=cells_across
        return self._query_operation(operation,**values)

    def _query_operation(self, operation, **inputs):
        # Live descriptor check prevents caller edits from promoting capabilities.
        actual = self.owner.request(op='describe', handle=self.handle)
        if actual != self.description:
            raise GeometryFailure('GeometryDescriptionMismatch', operation, 'native descriptor differs', self.identity)
        result = self.owner.request(op=operation, handle=self.handle, **inputs)
        if (result.get('geometry_identity') != self.identity or
                result.get('kernel_source_sha256') != actual['kernel_source_sha256'] or
                result.get('operation') != operation or result.get('schema') != 'V4CanonicalQueries/1'):
            raise GeometryFailure('QueryIdentityMismatch', operation, 'query provenance differs', self.identity)
        return result

    def discover(self): return self._query_operation('discover')
    def regional_expression(self): return self._query_operation('regional_expression')
    def stored_topology(self): return self._query_operation('stored_topology')
    def stored_spline(self): return self._query_operation('stored_spline')
    def numerical_project(self, points_m):
        return self._query_operation('numerical_project', points_m=points_m)
    def numerical_normal(self, points_m):
        return self._query_operation('numerical_normal', points_m=points_m)
    def feature_inventory(self): return self._query_operation('feature_inventory')
    def feature_evaluate(self, requests):
        return self._query_operation('feature_evaluate', requests=requests)
    def feature_query(self, points_m, options=None):
        """Options use tolerance_m; None retains the kernel's model-unit defaults."""
        return self._query_operation('feature_query', points_m=points_m, options=options)

    def release(self): return self.owner.request(op='release', handle=self.handle)


@dataclass(frozen=True)
class EnclosingRegion:
    center_m: tuple[float, float, float]
    radius_m: float
    authority: str

    def validate(self, geometry):
        if (len(self.center_m)!=3 or not all(math.isfinite(x) for x in self.center_m)
                or not math.isfinite(self.radius_m) or self.radius_m<=0
                or not math.isfinite(self.radius_m*self.radius_m) or not self.authority):
            raise GeometryFailure('InvalidEnclosingRegion', 'cgal_admission',
                                  'finite center, positive radius, and caller authority required', geometry.identity)
        # Bounds quality is kernel metadata, not an independent certificate.
        # Caller must affirm enclosure; an absent or unqualified bound is never guessed.
        return self


STAGE_REQUIREMENTS = {
    'scalar_query': ('signed_scalar',),
    'cgal_callback': ('signed_scalar',),
    'carrier_refinement': ('projection', 'derivatives', 'feature_query'),
    'tensor_sizing': ('derivatives', 'curvature_bounds', 'feature_query'),
    'mmgs_adaptation': ('projection', 'feature_query'),
    'exact_projection': ('projection',),
    'independent_qualification': ('continuous_certificates', 'feature_query'),
}


def require_capabilities(geometry, stage):
    if stage not in STAGE_REQUIREMENTS:
        raise GeometryFailure('UnknownStage', stage, 'no admission contract', geometry.identity)
    caps = geometry.description['capabilities']
    missing = {key: caps.get(key, 'absent') for key in STAGE_REQUIREMENTS[stage] if caps.get(key) != 'supported'}
    if missing:
        raise GeometryFailure('UnsupportedCapability', stage, 'stage not admitted', geometry.identity,
                              required=list(STAGE_REQUIREMENTS[stage]), supplied=missing)
    return True
