"""Shared schedule encoding at the existing assembly/preparation/read boundary.

Scientific component identity is the exact old indent=2 JSON byte stream.
Physical files use the existing durable writer, attestation, reference guards
and transaction closure. Old sealed files and pinned readers remain unchanged.
"""
from collections.abc import Mapping
from contextlib import ExitStack, contextmanager
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import immutable_state_access as access
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity

SCHEMA = 'V4CurrentFrontSharedSchedule/1'
SCHEDULE_SCHEMA = 'V4SharedDependencySchedule/1'
NAME = 'NEW_FORECAST_DEPENDENCIES.json'
_installed = False


def _json(value):
    return json.dumps(value, indent=2, sort_keys=True, allow_nan=False)


def _digest_rows(rows, schedules):
    """Exact original durable_json stream, bounded by one row and shared suffix.

    The schemas below require exactly three original row fields. Sort order
    therefore stays current_layer, region, remaining_schedule. JSON encoding
    and four-space nesting are the original standard-library operations.
    """
    h = hashlib.sha256()
    if not rows:
        h.update(b'{}\n')
        return h.hexdigest(), 3
    tails = {k: _json(v).replace('\n', '\n    ').encode('utf-8')
             for k, v in schedules.items()}
    size = 0
    def put(data):
        nonlocal size
        h.update(data); size += len(data)
    put(b'{\n')
    for i, key in enumerate(sorted(rows)):
        row = rows[key]
        if i: put(b',\n')
        text = ('  ' + json.dumps(key) + ': {\n'
                '    "current_layer": ' + json.dumps(row['current_layer']) + ',\n'
                '    "region": ' + json.dumps(row['region']) + ',\n'
                '    "remaining_schedule": ')
        put(text.encode('utf-8')); put(tails[row['schedule']]); put(b'\n  }')
    put(b'\n}\n')
    return h.hexdigest(), size


def _row(key, row):
    if not isinstance(key, str) or not isinstance(row, dict) or set(row) != {
            'current_layer', 'region', 'remaining_schedule'}:
        raise ValueError('UnsupportedForecastDependencyRow')
    if type(row['current_layer']) is not int or row['current_layer'] < 0:
        raise ValueError('InvalidDependencyLayer')
    if not isinstance(row['region'], str) or not isinstance(row['remaining_schedule'], list):
        raise ValueError('InvalidDependencyValue')
    # Encoding preserves all schedule values, including vertex-local records.
    # It does not validate or reinterpret trajectory/schedule science.
    _json(row)


def encode_rows(values):
    rows = {}; schedules = {}; seen = set()
    iterator = values.items() if isinstance(values, Mapping) else values
    for key, row in iterator:
        _row(key, row)
        if key in seen: raise ValueError('DuplicateDependencySource')
        seen.add(key)
        schedule = row['remaining_schedule']; digest = identity(schedule)
        if digest in schedules and schedules[digest] != schedule:
            raise ValueError('ConflictingSharedSchedule')
        if digest not in schedules: schedules[digest] = deepcopy(schedule)
        rows[key] = dict(current_layer=row['current_layer'], region=row['region'], schedule=digest)
    expanded, size = _digest_rows(rows, schedules)
    return rows, schedules, expanded, size


def write(path, values, *, shared_directory=None):
    """Write a new component, never rewrite any previously sealed pathname."""
    path = Path(path).resolve()
    if path.exists(): raise ValueError('PreserveExistingDependencyComponent')
    path.parent.mkdir(parents=True, exist_ok=True)
    rows, schedules, expanded, size = encode_rows(values)
    shared = dict(schema=SCHEDULE_SCHEMA, schedules=schedules)
    root = identity(shared)
    directory = Path(shared_directory).resolve() if shared_directory else path.parent/'shared_dependencies'
    directory.mkdir(parents=True, exist_ok=True)
    source = directory/(root + '.json')
    if not source.exists():
        durable_json(source, shared)
    with access.guard(source):
        if access.read(source) != shared: raise ValueError('ConflictingSharedScheduleObject')
        record = access.attest(source, file_hash(source))
    body = dict(schema=SCHEMA, rows=rows, shared=dict(path=str(source), root=root, **record),
                expanded_sha256=expanded, expanded_bytes=size, encoder_sha256=file_hash(__file__))
    durable_json(path, body)
    return dict(expanded_sha256=expanded, expanded_bytes=size, rows=len(rows),
                unique_schedules=len(schedules), physical_bytes=path.stat().st_size+source.stat().st_size)


class ForecastView(Mapping):
    """Owned lazy expansion. Returned lists cannot mutate shared evidence."""
    def __init__(self, owner, body, schedules):
        self.owner = owner; self.rows = body['rows']; self.schedules = schedules
        self.expanded_sha256 = body['expanded_sha256']
    def __len__(self): self.owner.check_live(); return len(self.rows)
    def __iter__(self): self.owner.check_live(); return iter(self.rows)
    def __getitem__(self, key):
        self.owner.check_live(); row = self.rows[key]
        return dict(current_layer=row['current_layer'], region=row['region'],
                    remaining_schedule=deepcopy(self.schedules[row['schedule']]))


def _validated(body, shared):
    if set(body) != {'schema','rows','shared','expanded_sha256','expanded_bytes','encoder_sha256'}:
        raise ValueError('InvalidDependencyEncoding')
    if body['schema'] != SCHEMA or body['encoder_sha256'] != file_hash(__file__):
        raise ValueError('UnsupportedDependencyEncoder')
    if set(shared) != {'schema','schedules'} or shared['schema'] != SCHEDULE_SCHEMA:
        raise ValueError('InvalidSharedDependencyObject')
    if identity(shared) != body['shared']['root']: raise ValueError('SharedScheduleRootMismatch')
    schedules = shared['schedules']
    for key, value in schedules.items():
        if not isinstance(value, list) or identity(value) != key:
            raise ValueError('SharedScheduleIdentityMismatch')
    used = set()
    for key, row in body['rows'].items():
        if set(row) != {'current_layer','region','schedule'} or row['schedule'] not in schedules:
            raise ValueError('MissingSharedSchedule')
        _row(key, dict(current_layer=row['current_layer'],region=row['region'],
                       remaining_schedule=schedules[row['schedule']]))
        used.add(row['schedule'])
    if used != set(schedules): raise ValueError('UnboundExtraSchedule')
    expanded, size = _digest_rows(body['rows'], schedules)
    if expanded != body['expanded_sha256'] or size != body['expanded_bytes']:
        raise ValueError('ExpandedDependencyIdentityMismatch')
    return schedules


def _encoded(body):
    """Unknown envelopes are not legacy row maps; validate genuine legacy rows."""
    if not isinstance(body, dict): raise ValueError('InvalidDependencyEncoding')
    if 'schema' in body and not isinstance(body['schema'], dict):
        if body['schema'] != SCHEMA: raise ValueError('UnsupportedDependencyEncoding')
        return True
    for key, row in body.items(): _row(key, row)
    return False


@contextmanager
def authenticated_snapshot(path):
    """Keep semantic derivation and physical closure on one guarded snapshot.

    Preparation retains both handles until its candidate plan is written.
    Hash helpers used alone remain bounded diagnostic/read operations.
    """
    path = Path(path).resolve()
    with ExitStack() as held:
        held.enter_context(access.guard(path)); body = access.read(path)
        physical = file_hash(path)
        refs = {str(path): access.attest(path, physical)}
        if not _encoded(body):
            yield dict(semantic=physical, references=refs, encoded=False)
            return
        source = Path(body['shared']['path']).resolve()
        held.enter_context(access.guard(source))
        if access.token(source) != body['shared']['token'] or file_hash(source) != body['shared']['sha256']:
            raise ValueError('ChangedSharedDependencyReference')
        _validated(body, access.read(source))
        refs[str(source)] = access.attest(source, body['shared']['sha256'])
        yield dict(semantic=body['expanded_sha256'], references=refs, encoded=True)


def reference_records(path):
    """Authenticated shared closure; preparation uses the held snapshot instead."""
    with authenticated_snapshot(path) as snapshot:
        return {k:v for k,v in snapshot['references'].items() if k != str(Path(path).resolve())}


def semantic_hash(path):
    with authenticated_snapshot(path) as snapshot: return snapshot['semantic']


def read_for_owner(owner, path):
    """Both physical files must already be in the authenticated plan closure."""
    body = access.read(owner.verify(Path(path)))
    if not _encoded(body):
        if owner.components['forecast']['replace'] != file_hash(path):
            raise ValueError('ForecastSemanticComponentMismatch')
        return body
    source = Path(body['shared']['path']).resolve()
    # verify does not permit an unbound file, even if its claimed hash matches.
    shared = access.read(owner.verify(source))
    if access.token(source) != body['shared']['token'] or file_hash(source) != body['shared']['sha256']:
        raise ValueError('ChangedSharedDependencyReference')
    schedules = _validated(body, shared)
    if owner.components['forecast']['replace'] != body['expanded_sha256']:
        raise ValueError('ForecastSemanticComponentMismatch')
    return ForecastView(owner, body, schedules)


def install_reader():
    """Extend the pinned reader through the established opt-in installer style."""
    global _installed
    if _installed: return
    from composable_state import CommittedState
    original = CommittedState.__getitem__
    def get(owner, key):
        if key == 'forecast' and owner.kind == 'recursive' and owner.production:
            owner.check_live()
            return read_for_owner(owner, owner.out/NAME)
        return original(owner, key)
    CommittedState.__getitem__ = get
    _installed = True
