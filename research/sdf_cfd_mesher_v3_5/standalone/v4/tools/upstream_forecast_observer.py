"""Private bounded scalar diagnostics. No geometry, acceptance or restart owner."""
from contextlib import contextmanager
from pathlib import Path
import hashlib
import json
import math
import sys
import os
import ctypes
import time

from repair_timing import RepairProgress
from background_core_repair import evidence_sink
import path_preserving_deduplication as writer_module
import immutable_state_access as access
import background_core_repair as sink_module
import optimized_durable_json as optimized
import optimized_mixed_certificate as native_module
import owned_runtime

# Normal imports retain the exact canonical definitions, never substitute a writer.
_EXPECTED_WRITER = optimized.durable_json
_EXPECTED_ENCODER = optimized.encoded
_EXPECTED_ATOMIC = writer_module.atomic_replace
_EXPECTED_GUARD = access.guard
_EXPECTED_NATIVE = native_module.Native
_EXPECTED_BYTES = native_module.Bytes
_EXPECTED_NATIVE_INIT = native_module.Native.__init__
_EXPECTED_GUARD_BODY = access.guard.__wrapped__
_EXPECTED_DUMPS = json.dumps
_EXPECTED_FSYNC = os.fsync
_EXPECTED_CDLL = ctypes.CDLL
_EXPECTED_STRING_AT = ctypes.string_at
_EXPECTED_NATIVE_PATH = native_module.NATIVE
_EXPECTED_SOURCE_PATH = native_module.SOURCE
_MODULES = (writer_module, sink_module, optimized, native_module, access, owned_runtime, json, os, ctypes)
_FUNCTION_CODES = tuple((fn, fn.__code__) for fn in
                        (_EXPECTED_WRITER, _EXPECTED_ENCODER, _EXPECTED_ATOMIC,
                         _EXPECTED_GUARD, _EXPECTED_GUARD.__wrapped__,
                         _EXPECTED_NATIVE.__init__, evidence_sink, _EXPECTED_DUMPS))


def callable_record(fn):
    code = getattr(fn, '__code__', None)
    module = sys.modules.get(getattr(fn, '__module__', ''))
    return dict(module=getattr(fn, '__module__', None), name=getattr(fn, '__qualname__', None),
                code_source=None if code is None else str(Path(code.co_filename).resolve()),
                module_source=None if not getattr(module, '__file__', None) else str(Path(module.__file__).resolve()))


def installed_sources():
    # The caller owns the existing immutable source/RELEASE holds for this lifetime.
    root = owned_runtime.V4
    release = root/'runtime/RELEASE.json'
    declared = {str((root/r['path']).resolve()): r['sha256']
                for r in json.loads(release.read_text(encoding='utf8'))['sources']}
    files = [Path(m.__file__).resolve() for m in (writer_module, sink_module, optimized, native_module, access, owned_runtime)]
    for m in (writer_module, sink_module, optimized, native_module, access, owned_runtime):
        if Path(m.__file__).resolve() != root/'tools'/(m.__name__+'.py'):
            raise ValueError('NoncanonicalObserverModulePath')
    for fn, module in [(_EXPECTED_WRITER, optimized), (_EXPECTED_ENCODER, optimized),
                       (_EXPECTED_ATOMIC, writer_module), (evidence_sink, sink_module),
                       (_EXPECTED_GUARD.__wrapped__, access), (_EXPECTED_NATIVE.__init__, native_module)]:
        if fn.__globals__ is not vars(module) or Path(fn.__code__.co_filename).resolve() != Path(module.__file__).resolve():
            raise ValueError('NoncanonicalObserverFunction')
    native = Path(native_module.NATIVE).resolve(); source = Path(native_module.SOURCE).resolve()
    digest = hashlib.sha256(native.read_bytes()+source.read_bytes()).hexdigest()
    binary = native.parents[1]/'runtime/bin'/(digest+('.dll' if os.name=='nt' else '.so'))
    files += [native, source, binary]
    result = {}
    for path in files:
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if declared.get(str(path)) != actual:
            raise ValueError('UnboundObserverSource', str(path))
        result[str(path)] = actual
    return dict(files=result, release=str(release), release_sha256=hashlib.sha256(release.read_bytes()).hexdigest(),
                native_binary=str(binary), native_binary_sha256=result[str(binary)],
                caller_source_holds_required=True)

RESERVED = frozenset(('numerical', 'terminal', 'placement1_close', 'placement2_close',
                      'upstream_close', 'truncation', 'secondary', 'final'))
SCOPE = 'Durable confirmed private edit prefix only; no accepted mesh or complete hard-killed state.'


def scalar_copy(value, depth=0):
    if depth > 8:
        raise ValueError('ObserverScalarDepth')
    if value is None or type(value) is bool:
        return value
    if type(value) is int:
        if abs(value) > 2**63 - 1:
            raise ValueError('ObserverIntegerRange')
        return value
    if type(value) is float:
        if not math.isfinite(value):
            raise ValueError('ObserverNonfinite')
        return value
    if type(value) is str:
        if len(value) > 20000:
            raise ValueError('ObserverStringLength')
        return value
    if type(value) is list and len(value) <= 128:
        return [scalar_copy(v, depth + 1) for v in value]
    if type(value) is dict and len(value) <= 128 and all(type(k) is str and len(k) <= 128 for k in value):
        return {k: scalar_copy(v, depth + 1) for k, v in value.items()}
    raise ValueError('ObserverScalarTypesRequired')


class Journal:
    event_limit = 256
    ordinary_limit = 248
    event_bytes = 16384
    total_bytes = 4194304
    interval = 2.0

    def __init__(self, folder, *, clock=time.perf_counter, held=None):
        self.folder = Path(folder)
        self.clock = clock
        self._sink = evidence_sink(self.folder, held=held)
        captured = dict(zip(self._sink.__code__.co_freevars,
                            (c.cell_contents for c in self._sink.__closure__)))
        self._writer = captured['durable_json']
        self._guard = captured['guard']
        self._atomic = _EXPECTED_ATOMIC
        self._encoder = _EXPECTED_ENCODER
        self._dumps = _EXPECTED_DUMPS
        self._check_bindings()
        self.source_binding = installed_sources()
        self.attempts = self.attempted_bytes = self.confirmed_bytes = self.ordinary = 0
        self.omissions = self.sampled_out = 0
        self.last = -math.inf
        self.used = set()
        self.first = {}
        self.failure = None
        self.incomplete = False
        self.placements = 0
        self.group_serial = 0
        self.group = {}
        self._upstream = None
        self._upstream_used = False

    def _check_bindings(self):
        if (self._writer is not _EXPECTED_WRITER or writer_module.durable_json is not _EXPECTED_WRITER
                or optimized.durable_json is not _EXPECTED_WRITER or self._encoder is not _EXPECTED_ENCODER
                or optimized.encoded is not _EXPECTED_ENCODER or self._atomic is not _EXPECTED_ATOMIC
                or optimized.atomic_replace is not _EXPECTED_ATOMIC or writer_module.atomic_replace is not _EXPECTED_ATOMIC
                or self._guard is not _EXPECTED_GUARD or access.guard is not _EXPECTED_GUARD
                or sink_module.evidence_sink is not evidence_sink
                or optimized.Native is not _EXPECTED_NATIVE or native_module.Native is not _EXPECTED_NATIVE
                or optimized.Bytes is not _EXPECTED_BYTES or native_module.Bytes is not _EXPECTED_BYTES
                or json.dumps is not _EXPECTED_DUMPS or self._dumps is not _EXPECTED_DUMPS
                or os.fsync is not _EXPECTED_FSYNC or ctypes.CDLL is not _EXPECTED_CDLL
                or ctypes.string_at is not _EXPECTED_STRING_AT
                or native_module.NATIVE is not _EXPECTED_NATIVE_PATH or native_module.SOURCE is not _EXPECTED_SOURCE_PATH
                or optimized.json is not json or optimized.os is not os or optimized.ctypes is not ctypes
                or native_module.ctypes is not ctypes
                or _EXPECTED_NATIVE.__init__ is not _EXPECTED_NATIVE_INIT or self._guard.__wrapped__ is not _EXPECTED_GUARD_BODY
                or any(sys.modules.get(m.__name__) is not m for m in _MODULES)
                or any(fn.__code__ is not code for fn, code in _FUNCTION_CODES)):
            raise ValueError('ChangedOptimizedObserverBinding')

    def bindings(self):
        return dict(callables={name: callable_record(fn) for name, fn in
                    [('writer', self._writer), ('encoder', self._encoder), ('atomic_replace', self._atomic),
                     ('guard_wrapper', self._guard), ('guard_body', self._guard.__wrapped__),
                     ('sink_factory', evidence_sink), ('native_constructor', _EXPECTED_NATIVE.__init__),
                     ('json_dumps', self._dumps), ('fsync', _EXPECTED_FSYNC)]},
                    source_binding=self.source_binding, mode='actual_post_initializer_optimized_writer')

    def status(self):
        return dict(attempts=self.attempts, confirmed=len(self._sink.receipts),
                    attempted_bytes=self.attempted_bytes, confirmed_bytes=self.confirmed_bytes,
                    ordinary=self.ordinary, used=sorted(self.used), omissions=self.omissions,
                    sampled_out=self.sampled_out, failure=self.failure, incomplete=self.incomplete,
                    first=scalar_copy(self.first), scientific_acceptance=False)

    def _omit(self, reason, category):
        self.omissions += 1
        self.incomplete = True
        if 'truncation' not in self.used and category != 'truncation':
            self.event('truncation', dict(reason=reason, omitted_category=category))
        return False

    def event(self, category, payload):
        if category != 'progress' and category not in RESERVED:
            raise ValueError('ObserverEventCategory')
        private = scalar_copy(payload)
        if self.failure is not None:
            raise RuntimeError('ObserverJournalStopped', self.failure)
        if category in self.used:
            return False
        if category == 'progress':
            now = self.clock()
            if now - self.last < self.interval:
                self.sampled_out += 1
                return False
            if self.ordinary >= self.ordinary_limit:
                return self._omit('ordinary_event_limit', category)
        else:
            self.used.add(category)
        sequence = len(self._sink.receipts) + 1
        body = dict(schema='V4CoreRepairDiagnosticEvent/1', sequence=sequence,
                    event=category, payload=private, provisional=True, scientific_authority=False,
                    restart_supported=False, scope=SCOPE)
        try:
            self._check_bindings()
            data = self._encoder(body)
        except BaseException as error:
            self.incomplete = True
            self.failure = dict(type=type(error).__name__, text=str(error)[:512], sequence=sequence,
                                attempted_bytes=0, receipts=len(self._sink.receipts), prepublication=True)
            raise
        if len(data) > self.event_bytes:
            return self._omit('encoded_event_bytes', category)
        if self.attempts >= self.event_limit or self.attempted_bytes + len(data) > self.total_bytes:
            return self._omit('aggregate_capacity', category)
        # Debit before invoking a writer which may publish without issuing a receipt.
        self.attempts += 1
        self.attempted_bytes += len(data)
        if category == 'progress':
            self.ordinary += 1
            self.last = self.clock()
        try:
            self._check_bindings()
            self._sink(category, private)
            receipt = self._sink.receipts[-1]
            path = self.folder / receipt['file']
            with self._guard(path):
                actual = path.read_bytes()
                digest = hashlib.sha256(actual).hexdigest()
                if actual != data or digest != receipt['sha256'] or receipt['file'] != f'{sequence:06d}.json':
                    raise ValueError('ObserverCommittedEncodingMismatch')
            self.confirmed_bytes += len(data)
        except BaseException as error:
            self.incomplete = True
            self.failure = dict(type=type(error).__name__, text=str(error)[:512], sequence=sequence,
                                attempted_bytes=len(data), receipts=len(self._sink.receipts))
            raise
        return True

    def numerical(self, payload):
        if 'numerical' in self.first:
            return False
        private = scalar_copy(payload)
        self.first['numerical'] = dict(kind=private.get('kind', 'unclassified')[:128]
                                       if type(private) is dict and type(private.get('kind')) is str else 'unclassified',
                                       full_payload_in_journal=False)
        saved = self.event('numerical', private)
        self.first['numerical']['full_payload_in_journal'] = saved
        return saved

    def terminal(self, error):
        if 'terminal' in self.first:
            return False
        value = dict(type=type(error).__name__, text=str(error)[:512])
        self.first['terminal'] = value
        return self.event('terminal', value)

    def secondary(self, primary, error):
        self.incomplete = True
        detail = f'Observer evidence failure: {type(error).__name__}: {str(error)[:512]}'
        if self.failure is None:
            self.failure = dict(type=type(error).__name__, text=str(error)[:512],
                                sequence=len(self._sink.receipts)+1, receipts=len(self._sink.receipts),
                                observer_failure=True)
        if primary is not None:
            try:
                primary.add_note(detail)
            except BaseException:
                pass  # The independent outer result/status remains the fallback.
            return
        raise error

    def progress(self, scope, snapshot):
        event = scope + '_close' if snapshot.get('closed') else 'progress'
        return self.event(event, dict(scope=scope, snapshot=snapshot))

    def placement(self):
        if self.placements >= 2:
            raise ValueError('ObserverPlacementOwnerLimit')
        self.placements += 1
        scope = f'placement{self.placements}'
        return lambda snapshot: self.progress(scope, snapshot)

    @contextmanager
    def upstream(self):
        if self._upstream_used:
            raise ValueError('ObserverUpstreamOwnerLimit')
        self._upstream_used = True
        progress = RepairProgress(lambda snapshot: self.progress('upstream', snapshot), clock=self.clock)
        self._upstream = progress
        primary = None
        try:
            with progress.span('upstream', publish=True):
                yield progress
        except BaseException as error:
            primary = error
            progress.outcome = 'raised'
            progress.terminal_error = repr(error)
            try:
                self.terminal(error)
            except BaseException as secondary:
                self.secondary(error, secondary)
            raise
        else:
            progress.outcome = 'returned'
        finally:
            try:
                progress.close()
            except BaseException as secondary:
                if primary is None:
                    raise
                self.secondary(primary, secondary)
            finally:
                self._upstream = None

    @contextmanager
    def recorded(self):
        primary = None
        try:
            yield
        except BaseException as error:
            primary = error
            try:
                self.terminal(error)
            except BaseException as secondary:
                self.secondary(error, secondary)
            raise
        finally:
            try:
                self.final()
            except BaseException as secondary:
                if primary is None:
                    raise
                self.secondary(primary, secondary)

    def quality(self, failures, phase):
        if not failures or 'numerical' in self.first:
            return
        counts = {}
        for row in failures:
            kind = row.get('kind', 'unclassified')
            counts[kind] = counts.get(kind, 0) + 1
        self.numerical(dict(kind='quality_rejection', phase=phase,
                            records=len(failures), record_kinds=counts))

    def phase(self, name):
        if self._upstream is None:
            raise ValueError('ObserverUpstreamScopeRequired')
        return self._upstream.span(name)

    def group_begin(self, vertices):
        if type(vertices) is not int or vertices < 0:
            raise ValueError('ObserverGroupSize')
        self.group_serial += 1
        self.group = dict(visit=self.group_serial, vertices=vertices, closure_valid=False,
                          cells=None, neighbours=None, solver_nit=None, solver_nfev=None,
                          solver_result_valid=False)
        return scalar_copy(self.group)

    def group_closure(self, cells, neighbours):
        if not self.group or any(type(v) is not int or v < 0 for v in (cells, neighbours)):
            raise ValueError('ObserverGroupClosure')
        self.group.update(cells=cells, neighbours=neighbours, closure_valid=True)
        return scalar_copy(self.group)

    def group_solver_returned(self, nit, nfev):
        if not self.group or any(v is not None and (type(v) is not int or v < 0) for v in (nit, nfev)):
            raise ValueError('ObserverSolverResult')
        self.group.update(solver_nit=nit, solver_nfev=nfev, solver_result_valid=True)
        return scalar_copy(self.group)

    def group_progress(self):
        if not self.group:
            raise ValueError('ObserverGroupRequired')
        return dict(group_visit=self.group['visit'], group_cells=self.group['cells'],
                    group_neighbours=self.group['neighbours'], group_closure_valid=self.group['closure_valid'],
                    solver_nit=self.group['solver_nit'], solver_nfev=self.group['solver_nfev'],
                    solver_result_valid=self.group['solver_result_valid'])

    def final(self):
        return self.event('final', self.status())
