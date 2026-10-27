"""Bind the existing exact historical-reader relation to a native enrollment.

The capsule's two-role source contract stays unchanged. This connection supports
only its same reader hook; every other original implementation must match bytes.
It does not accept a new source difference or authorize mesh advancement.
"""
from contextlib import ExitStack, contextmanager
from copy import deepcopy
from pathlib import Path
import immutable_state_access as access
import historical_implementation_access as existing
from recursive_epoch_recovery import sealed_body
from path_preserving_deduplication import file_hash
from sdf_cfd_mesher_v4.common.authority import identity


def derive(reference, historical):
    if reference.get('kind') != 'native':
        raise ValueError('NativeEnrollmentSourceRequired')
    with ExitStack() as held:
        body = sealed_body(reference, held)
        if body['schema'] != access.SCHEMA or identity(body['content']) != body['content_root']:
            raise ValueError('NativeEnrollmentSourceRequired')
        implementations = body['content']['implementation']
        current = str(Path(access.__file__).resolve())
        expected = implementations.get(current)
        historical = str(Path(historical).resolve())
        if expected is None:
            raise ValueError('OriginalReaderPinRequired')
        unchanged = {}
        for p, h in implementations.items():
            held.enter_context(access.guard(p))
            if p != current:
                if file_hash(p) != h:
                    raise ValueError('UnsupportedNativeSourceDifference', p)
                unchanged[p] = h
        for p in (historical, __file__, existing.__file__):
            held.enter_context(access.guard(p))
        if file_hash(historical) != expected:
            raise ValueError('HistoricalSourcePinMismatch')
        relation = existing.source_relation('reader_historical_access',
            Path(historical).read_text(encoding='utf-8'), Path(current).read_text(encoding='utf-8'))
        root = Path(reference['directory'])/('ROOT_'+reference['root']+'.json')
        return dict(schema='V4NativeHistoricalReaderConnection/1', native_reference=deepcopy(reference),
            native_root_sha256=file_hash(root), content_root=body['content_root'],
            original_implementation=deepcopy(implementations), unchanged=unchanged,
            rows=[dict(current=current, current_sha256=file_hash(current), historical=historical,
                historical_sha256=expected, relation=relation)],
            checker=str(Path(__file__).resolve()), checker_sha256=file_hash(__file__),
            existing_checker=str(Path(existing.__file__).resolve()), existing_checker_sha256=file_hash(existing.__file__),
            scope='Only existing historical-reader hooks; unchanged remaining native source pins; access only.')


@contextmanager
def qualified_reader_access(record_path, record_identity, review_path, review_sha256):
    if existing._active.get() is not None:
        raise ValueError('NestedHistoricalSourceAccess')
    with ExitStack() as held:
        for p in (record_path, review_path, __file__, existing.__file__):
            held.enter_context(access.guard(p))
        record, review = access.read(record_path), access.read(review_path)
        if (identity(record) != record_identity or record.get('schema') != 'V4NativeHistoricalReaderConnection/1'
            or file_hash(review_path) != review_sha256 or review.get('passed') is not True
            or not review.get('reviewer') or review.get('record') != record_identity
            or review.get('record_sha256') != file_hash(record_path)):
            raise ValueError('ReviewedHistoricalSourceAccessRequired')
        if (record['checker'] != str(Path(__file__).resolve()) or record['checker_sha256'] != file_hash(__file__)
            or record['existing_checker'] != str(Path(existing.__file__).resolve())
            or record['existing_checker_sha256'] != file_hash(existing.__file__)):
            raise ValueError('ChangedHistoricalSourceChecker')
        if len(record['rows']) != 1 or record['rows'][0]['relation']['role'] != 'reader_historical_access':
            raise ValueError('UnsupportedHistoricalSourceRole')
        for row in record['rows']:
            for path_key, hash_key in (('current','current_sha256'), ('historical','historical_sha256')):
                held.enter_context(access.guard(row[path_key]))
                if file_hash(row[path_key]) != row[hash_key]:
                    raise ValueError('ChangedHistoricalSourceDependency')
        for p, h in record['unchanged'].items():
            held.enter_context(access.guard(p))
            if file_hash(p) != h:
                raise ValueError('UnsupportedNativeSourceDifference', p)
        # Keep the sealed enrollment metadata held through resolver activation.
        sealed_body(record['native_reference'], held)
        actual = derive(record['native_reference'], record['rows'][0]['historical'])
        if actual != record:
            raise ValueError('AlteredHistoricalSourceDerivation')
        # Retain original pin/content selection, closed-scope checks and resolver.
        proof = existing.SourceAccess(record)
        token = existing._active.set(proof)
        try:
            yield proof
            proof.check()
        finally:
            proof.closed = True
            existing._active.reset(token)
