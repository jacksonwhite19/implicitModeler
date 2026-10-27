"""Opt-in bounded exact segment receipts. No geometry predicate or evaluator."""
from pathlib import Path
from contextlib import contextmanager
from contextvars import ContextVar
from fractions import Fraction
import hashlib, json, math, os, threading
import time as _time

class ReceiptError(ValueError):
    pass

class ReceiptLimitError(ReceiptError):
    pass
_active = ContextVar('segment_receipt_limits', default=None)
SCALAR_BYTES = 16384
RECORD_BYTES = 2 * 1024 * 1024
LEDGER_BYTES = 3 * 1024 * 1024 * 1024
CLOSURE_RESERVE = 1024 * 1024
CHUNK_BYTES = 8192
ROUTES = ('gradient_region', 'source_blend_progress', 'parametric_rational')

def check_rational_size(value):
    limits = _active.get()
    if limits is None:
        return
    for integer in (value.numerator, value.denominator):
        if (abs(integer).bit_length() + 3) // 4 + 3 > limits:
            raise ReceiptLimitError('ReceiptRationalScalarLimit')

def _tree(value, scalar, depth=0):
    if depth > 32:
        raise ReceiptLimitError('ReceiptNestingLimit')
    if value is None or type(value) is bool:
        return
    if type(value) is str:
        if len(value) > scalar or len(value.encode('utf-8')) > scalar:
            raise ReceiptLimitError('ReceiptScalarLimit')
    elif type(value) is int:
        if abs(value).bit_length() > 1024:
            raise ReceiptLimitError('ReceiptIntegerLimit')
    elif type(value) is float:
        if not math.isfinite(value):
            raise ReceiptError('ReceiptFiniteValueRequired')
    elif type(value) in (list, tuple):
        if len(value) > 4096:
            raise ReceiptLimitError('ReceiptContainerLimit')
        for x in value:
            _tree(x, scalar, depth + 1)
    elif type(value) is dict:
        if len(value) > 4096:
            raise ReceiptLimitError('ReceiptContainerLimit')
        for k, v in value.items():
            if type(k) is not str:
                raise ReceiptError('ReceiptStringKeysRequired')
            _tree(k, scalar, depth + 1)
            _tree(v, scalar, depth + 1)
    else:
        raise ReceiptError('ReceiptUnsupportedValue')

def rational(value):
    return Fraction(int(value['numerator_hex'], 16), int(value['denominator_hex'], 16))

def validate_proof(proof, expression):
    counts = ('segment_leaves', 'segment_boxes', 'maximum_depth_used')
    if any((type(proof.get(k)) is not int for k in counts)):
        raise ReceiptError('ReceiptIntegerMetadataRequired')
    if proof['status'] != 'Certified' or proof['endpoint_positive'] is not True or proof['strict_field_progress'] is not True or (proof['source_expression_sha256'] != expression):
        raise ReceiptError('ReceiptProofBinding')
    cursor = Fraction(0)
    minimum = None
    routes = dict.fromkeys(ROUTES, 0)
    leaves = proof['accepted_leaf_proofs']
    if type(leaves) is not list or proof['segment_leaves'] != len(leaves):
        raise ReceiptError('ReceiptLeafCountMismatch')
    if not 1 <= len(leaves) <= 64 or not 1 <= proof['segment_boxes'] <= 127 or proof['maximum_depth_used'] > 6:
        raise ReceiptError('ReceiptProofCountLimit')
    for leaf in leaves:
        lo, hi = map(Fraction, leaf['parameter_interval_exact'])
        bound = rational(leaf['lower_bound_exact'])
        depth = leaf['depth']
        if type(depth) is not int or not 0 <= depth <= 6 or hi - lo != Fraction(1, 2 ** depth) or ((lo * 2 ** depth).denominator != 1):
            raise ReceiptError('ReceiptDyadicDepth')
        if lo != cursor or not lo < hi <= 1 or bound <= 0 or (leaf['route'] not in routes):
            raise ReceiptError('ReceiptPositiveOrderedCoverage')
        cursor = hi
        routes[leaf['route']] += 1
        minimum = bound if minimum is None else min(minimum, bound)
    if cursor != 1 or proof['maximum_depth_used'] != max((x['depth'] for x in leaves)) or proof['segment_boxes'] != 2 * len(leaves) - 1:
        raise ReceiptError('ReceiptIncompleteCoverage')
    if proof['minimum_progress_encoding'] not in ('rational_hex','decimal_fraction'):raise ReceiptError('ReceiptMinimumEncoding')
    claimed = rational(proof['minimum_progress_lower_exact']) if proof['minimum_progress_encoding'] == 'rational_hex' else Fraction(proof['minimum_progress_lower_exact'])
    if claimed != minimum:
        raise ReceiptError('ReceiptMinimumMismatch')
    return (minimum, routes)

def _replace_checkpoint(temporary, target):
    """Retry only Windows replacement denials, on the same fsynced bytes.

    Six attempts and a 250ms scheduling budget; syscall/scheduler stalls remain
    subject to the enclosing owner deadline. No row/write/proof is retried.
    """
    started = _time.monotonic()
    for attempt in range(6):
        try:
            os.replace(temporary, target)
            return
        except OSError as failure:
            if os.name != 'nt' or getattr(failure, 'winerror', None) not in (5, 32, 33) or attempt == 5:
                raise
            delay = .005 * (2 ** attempt)
            if _time.monotonic() - started + delay >= .25:
                raise
            _time.sleep(delay)
            if _time.monotonic() - started >= .25:
                raise

class SegmentReceiptSink:
    __slots__ = ('path', '_binding', 'record_limit', 'ledger_limit', 'scalar_limit', 'fd', 'offset', 'chain', 'rows', 'maximum_height_error', 'minimum', 'routes', 'leaves', 'boxes', 'maximum_depth', 'maximum_record_bytes', 'thread', 'state', 'secondary_errors', 'poisoned')

    def __setattr__(self, name, value):
        raise ReceiptError('ReceiptStateMutationForbidden')

    @property
    def binding(self):
        return dict(self._binding)

    def _note(self, value):
        object.__setattr__(self, 'secondary_errors', (self.secondary_errors + (value,))[:4])
    'One synchronous writer. Identity binding comes from live source custody.'

    def __init__(self, path, binding, *, record_bytes=RECORD_BYTES, ledger_bytes=LEDGER_BYTES, scalar_bytes=SCALAR_BYTES):
        if type(record_bytes) is not int or not 256 <= record_bytes <= RECORD_BYTES or type(ledger_bytes) is not int or (not record_bytes <= ledger_bytes <= LEDGER_BYTES) or (type(scalar_bytes) is not int) or (not 128 <= scalar_bytes <= SCALAR_BYTES):
            raise ReceiptError('ReceiptLimitsInvalid')
        if type(binding) is not dict or len(binding) > 20 or any((type(k) is not str or len(k) > 64 or type(v) not in (str, int, float) or (type(v) is str and len(v) > 256) for k, v in binding.items())):
            raise ReceiptError('ReceiptFlatBindingRequired')
        _tree(binding, scalar_bytes)
        object.__setattr__(self, 'path', Path(path))
        object.__setattr__(self, '_binding', tuple(sorted(binding.items())))
        object.__setattr__(self, 'record_limit', record_bytes)
        object.__setattr__(self, 'ledger_limit', ledger_bytes)
        object.__setattr__(self, 'scalar_limit', scalar_bytes)
        object.__setattr__(self, 'fd', None)
        object.__setattr__(self, 'offset', 0)
        object.__setattr__(self, 'chain', '0' * 64)
        object.__setattr__(self, 'rows', 0)
        object.__setattr__(self, 'maximum_height_error', 0.0)
        object.__setattr__(self, 'minimum', None)
        object.__setattr__(self, 'routes', (0, 0, 0))
        object.__setattr__(self, 'leaves', 0)
        object.__setattr__(self, 'boxes', 0)
        object.__setattr__(self, 'maximum_depth', 0)
        object.__setattr__(self, 'maximum_record_bytes', 0)
        object.__setattr__(self, 'thread', threading.get_ident())
        object.__setattr__(self, 'state', 'new')
        object.__setattr__(self, 'secondary_errors', ())
        object.__setattr__(self, 'poisoned', False)

    def _check(self):
        validate_sink(self)
        if threading.get_ident() != self.thread:
            raise ReceiptError('ReceiptThreadMismatch')

    @contextmanager
    def resource_scope(self):
        self._check()
        token = _active.set(self.scalar_limit)
        try:
            yield
        finally:
            _active.reset(token)

    def _emit(self, value, *, closure=False):
        self._check()
        _tree(value, self.scalar_limit)
        value = dict(value, previous_sha256=self.chain)
        start = self.offset
        count = 0
        digest = hashlib.sha256(bytes.fromhex(self.chain))
        encoder = json.JSONEncoder(sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False)
        try:
            for token in encoder.iterencode(value):
                data = token.encode('ascii')
                for pos in range(0, len(data), CHUNK_BYTES):
                    chunk = data[pos:pos + CHUNK_BYTES]
                    count += len(chunk)
                    if count + 1 > self.record_limit:
                        raise ReceiptLimitError('ReceiptRecordByteLimit')
                    if start + count + 1 > self.ledger_limit + (CLOSURE_RESERVE if closure else 0):
                        raise ReceiptLimitError('ReceiptAggregateByteLimit')
                    view = memoryview(chunk)
                    while view:
                        written = os.write(self.fd, view)
                        if written <= 0:
                            raise OSError('ReceiptShortWrite')
                        view = view[written:]
                    digest.update(chunk)
            if os.write(self.fd, b'\n') != 1:
                raise OSError('ReceiptNewlineShortWrite')
            digest.update(b'\n')
            os.fsync(self.fd)
            self._checkpoint(value['kind'],start+count+1,digest.hexdigest())
        except BaseException as failure:
            try:
                os.ftruncate(self.fd, start)
                os.lseek(self.fd, start, os.SEEK_SET)
                os.fsync(self.fd)
            except BaseException:
                self._note('ReceiptRollbackFailed')
                object.__setattr__(self, 'poisoned', True)
            if isinstance(failure, OSError):
                raise ReceiptError('ReceiptIOFailure') from failure
            raise
        object.__setattr__(self, 'offset', start + count + 1)
        object.__setattr__(self, 'chain', digest.hexdigest())
        object.__setattr__(self, 'maximum_record_bytes', max(self.maximum_record_bytes, count + 1))

    def _checkpoint(self,kind,offset,chain):
        state='complete' if kind=='complete' else 'incomplete' if kind=='incomplete' else 'active'
        value=dict(schema='V4SegmentCheckpoint/1',committed_bytes=offset,chain_sha256=chain,
            completed_rows=self.rows+(kind=='row'),state=state,
            binding_sha256=hashlib.sha256(json.dumps(self.binding,sort_keys=True,separators=(',',':')).encode()).hexdigest())
        encoded=json.dumps(value,sort_keys=True,separators=(',',':')).encode('ascii')+b'\n'
        if len(encoded)>4096:raise ReceiptLimitError('ReceiptCheckpointLimit')
        temporary=self.path.with_suffix('.checkpoint.tmp');target=self.path.with_suffix('.checkpoint.json')
        with temporary.open('wb',buffering=0) as stream:
            if stream.write(encoded)!=len(encoded):raise OSError('ReceiptCheckpointShortWrite')
            os.fsync(stream.fileno())
        _replace_checkpoint(temporary,target)

    def begin(self, actual):
        self._check()
        if self.state != 'new':
            raise ReceiptError('ReceiptAlreadyStarted')
        for key, value in actual.items():
            if self.binding.get(key) != value:
                raise ReceiptError('ReceiptInputBindingMismatch:' + key)
        for key in ('source_release_sha256', 'request_identity'):
            v = self.binding.get(key)
            if type(v) is not str or len(v) != 64 or any((c not in '0123456789abcdef' for c in v)):
                raise ReceiptError('ReceiptCustodyBindingRequired')
        if type(actual['segments']) is not int or actual['segments'] <= 0 or actual['first_height_m'] is None:
            raise ReceiptError('ReceiptExplicitHeightRequired')
        if actual.get('maximum_depth',6)>6 or actual.get('maximum_boxes',1000)>1000 or actual.get('precision_bits',80)>80:raise ReceiptError('ReceiptUnsupportedBudgets')
        try:
            object.__setattr__(self, 'fd', os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_BINARY', 0), 384))
        except OSError as failure:
            raise ReceiptError('ReceiptOpenFailure') from failure
        object.__setattr__(self, 'state', 'active')
        try:
            self._emit(dict(kind='header', schema='V4SegmentReceipts/1', binding=self.binding, caps=dict(record_bytes=self.record_limit, ledger_bytes=self.ledger_limit, scalar_bytes=self.scalar_limit, chunk_bytes=CHUNK_BYTES, closure_reserve=CLOSURE_RESERVE)))
        except BaseException:
            self.close()
            raise

    def append(self, index, proof, height_error):
        self._check()
        if type(index) is not int or self.state != 'active' or index != self.rows or (index >= self.binding['segments']):
            raise ReceiptError('ReceiptRowOrder')
        _tree(proof, self.scalar_limit)
        minimum, routes = validate_proof(proof, self.binding['source_expression_sha256'])
        if type(height_error) not in (float, int) or not math.isfinite(height_error) or (not 0 <= height_error < 1e-12):
            raise ReceiptError('ReceiptHeightRequired')
        self._emit(dict(kind='row', index=index, proof=proof, height_error_m=height_error))
        object.__setattr__(self, 'rows', self.rows + 1)
        object.__setattr__(self, 'maximum_height_error', max(self.maximum_height_error, height_error))
        object.__setattr__(self, 'minimum', minimum if self.minimum is None else min(self.minimum, minimum))
        object.__setattr__(self, 'routes', tuple((self.routes[i] + routes[k] for i, k in enumerate(ROUTES))))
        object.__setattr__(self, 'leaves', self.leaves + proof['segment_leaves'])
        object.__setattr__(self, 'boxes', self.boxes + proof['segment_boxes'])
        object.__setattr__(self, 'maximum_depth', max(self.maximum_depth, proof['maximum_depth_used']))

    def summary(self):
        minimum = None if self.minimum is None else dict(numerator_hex=hex(self.minimum.numerator), denominator_hex=hex(self.minimum.denominator))
        return dict(schema='V4SegmentReceiptSummary/1', state=self.state, completed_rows=self.rows, expected_rows=self.binding['segments'], ledger_bytes=self.offset, chain_sha256=self.chain, route_counts=dict(zip(ROUTES, self.routes)), segment_leaves=self.leaves, segment_boxes=self.boxes, maximum_depth_used=self.maximum_depth, minimum_lower_exact=minimum, maximum_first_height_error_m=self.maximum_height_error, maximum_record_bytes=self.maximum_record_bytes, secondary_errors=list(self.secondary_errors), binding=dict(self.binding))

    def complete(self):
        self._check()
        if self.state != 'active' or self.rows != self.binding['segments']:
            raise ReceiptError('ReceiptBatchIncomplete')
        self._emit(dict(kind='complete', completed_rows=self.rows, expected_rows=self.binding['segments']), closure=True)
        object.__setattr__(self, 'state', 'complete')
        self.close()
        return self.summary()

    def abort(self, primary):
        self._check()
        if self.state == 'active':
            try:
                if self.poisoned:
                    raise ReceiptError('ReceiptDamagedTail')
                self._emit(dict(kind='incomplete', completed_rows=self.rows, expected_rows=self.binding['segments'], primary_error_type=type(primary).__name__), closure=True)
            except BaseException as secondary:
                self._note(type(secondary).__name__)
            object.__setattr__(self, 'state', 'incomplete')
            self.close()

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            finally:
                object.__setattr__(self, 'fd', None)

def verify(path, expected_binding, *, require_complete=True, expected_chain=None, committed_bytes=None):
    """Bounded offline replay; a crash may leave one incomplete trailing line."""
    chain = '0' * 64
    rows = 0
    status = 'incomplete'
    header = False
    footer = False
    minimum = None
    routes = dict.fromkeys(ROUTES, 0)
    height = 0.0
    bytes_read = 0
    caps = None
    prefix_bytes = 0
    if committed_bytes is not None and (type(committed_bytes) is not int or not 0 <= committed_bytes <= Path(path).stat().st_size):
        raise ReceiptError('ReceiptCommittedOffset')
    with Path(path).open('rb') as stream:
        while True:
            if committed_bytes is not None and bytes_read == committed_bytes:
                break
            remaining = RECORD_BYTES + 1 if committed_bytes is None else min(RECORD_BYTES + 1, committed_bytes - bytes_read)
            raw = stream.readline(remaining)
            if not raw:
                break
            bytes_read += len(raw)
            if len(raw) > RECORD_BYTES or bytes_read > LEDGER_BYTES + CLOSURE_RESERVE:
                raise ReceiptLimitError('ReceiptReplayByteLimit')
            if not raw.endswith(b'\n'):
                if require_complete:
                    raise ReceiptError('ReceiptTruncated')
                break
            value = json.loads(raw)
            _tree(value, SCALAR_BYTES)
            if caps is not None:
                if len(raw) > caps['record_bytes'] or bytes_read > caps['ledger_bytes'] + (caps['closure_reserve'] if value.get('kind') in ('complete', 'incomplete') else 0):
                    raise ReceiptLimitError('ReceiptDeclaredByteLimit')
                _tree(value, caps['scalar_bytes'])
            if footer or value.get('previous_sha256') != chain:
                raise ReceiptError('ReceiptHashOrder')
            chain = hashlib.sha256(bytes.fromhex(chain) + raw).hexdigest()
            kind = value.get('kind')
            if not header:
                if kind != 'header' or value['schema'] != 'V4SegmentReceipts/1' or value['binding'] != expected_binding:
                    raise ReceiptError('ReceiptReplayBinding')
                caps = value['caps']
                if set(caps) != set(('record_bytes', 'ledger_bytes', 'scalar_bytes', 'chunk_bytes', 'closure_reserve')) or any((type(z) is not int for z in caps.values())) or (not 256 <= caps['record_bytes'] <= RECORD_BYTES) or (not caps['record_bytes'] <= caps['ledger_bytes'] <= LEDGER_BYTES) or (not 128 <= caps['scalar_bytes'] <= SCALAR_BYTES) or (caps['chunk_bytes'] != CHUNK_BYTES) or (caps['closure_reserve'] != CLOSURE_RESERVE) or (len(raw) > caps['record_bytes']) or (bytes_read > caps['ledger_bytes']):
                    raise ReceiptError('ReceiptDeclaredCaps')
                _tree(value, caps['scalar_bytes'])
                header = True
                prefix_bytes = bytes_read
                continue
            if kind == 'row':
                if type(value['index']) is not int or value['index'] != rows or rows >= expected_binding['segments']:
                    raise ReceiptError('ReceiptReplayOrder')
                bound, pop = validate_proof(value['proof'], expected_binding['source_expression_sha256'])
                err = value['height_error_m']
                if type(err) not in (float, int) or not math.isfinite(err) or (not 0 <= err < 1e-12):
                    raise ReceiptError('ReceiptReplayHeight')
                minimum = bound if minimum is None else min(minimum, bound)
                height = max(height, err)
                for k, v in pop.items():
                    routes[k] += v
                rows += 1
            elif kind in ('complete', 'incomplete'):
                if type(value['completed_rows']) is not int or type(value['expected_rows']) is not int or value['completed_rows'] != rows or (value['expected_rows'] != expected_binding['segments']):
                    raise ReceiptError('ReceiptFooterCount')
                if kind == 'complete' and rows != expected_binding['segments']:
                    raise ReceiptError('ReceiptFalseCompletion')
                status = kind
                footer = True
            else:
                raise ReceiptError('ReceiptRecordKind')
            prefix_bytes = bytes_read
    if not header or (require_complete and status != 'complete'):
        raise ReceiptError('ReceiptCompletionRequired')
    if require_complete and expected_chain is None:
        raise ReceiptError('ReceiptAuthenticatedChainRequired')
    if expected_chain is not None and chain != expected_chain:
        raise ReceiptError('ReceiptFinalChainMismatch')
    trailing = Path(path).stat().st_size - prefix_bytes
    if require_complete and trailing:
        raise ReceiptError('ReceiptTrailingBytes')
    return dict(state=status, completed_rows=rows, chain_sha256=chain, route_counts=routes, minimum_lower_approx=float(minimum) if minimum is not None else None, maximum_first_height_error_m=height, bytes_read=prefix_bytes, trailing_bytes=trailing)

_STANDARD_SINK_MEMBERS=tuple((name,value) for name,value in SegmentReceiptSink.__dict__.items() if callable(value) or isinstance(value,property))
def validate_sink(sink):
    if type(sink) is not SegmentReceiptSink or any(SegmentReceiptSink.__dict__.get(name) is not value for name,value in _STANDARD_SINK_MEMBERS):raise ReceiptError('ChangedSegmentReceiptSink')

def verify_checkpoint(path,expected_binding,*,require_complete=False):
    path=Path(path)
    with path.with_suffix('.checkpoint.json').open('rb') as stream:
        raw=stream.read(4097)
    if len(raw)>4096:raise ReceiptLimitError('ReceiptCheckpointLimit')
    checkpoint=json.loads(raw)
    expected=hashlib.sha256(json.dumps(expected_binding,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    if checkpoint.get('schema')!='V4SegmentCheckpoint/1' or checkpoint.get('binding_sha256')!=expected or type(checkpoint.get('completed_rows')) is not int or checkpoint.get('state') not in ('active','complete','incomplete'):raise ReceiptError('ReceiptCheckpointBinding')
    result=verify(path,expected_binding,require_complete=require_complete,expected_chain=checkpoint['chain_sha256'],committed_bytes=checkpoint['committed_bytes'])
    if result['completed_rows']!=checkpoint['completed_rows'] or result['bytes_read']!=checkpoint['committed_bytes'] or (checkpoint['state']=='complete')!=(result['state']=='complete'):raise ReceiptError('ReceiptCheckpointCoverage')
    return dict(result,checkpoint_state=checkpoint['state'])

def bounded_receipts_active():return _active.get() is not None
