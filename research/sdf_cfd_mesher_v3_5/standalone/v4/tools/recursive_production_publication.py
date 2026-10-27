"""Qualified optimized publication mechanics with a generic incidence parent."""
import sqlite3
from unittest.mock import patch
import immutable_state_access as access
import composable_publication as publication
import incremental_pair_storage as storage
from qualify_recursive_layer6 import measured, metrics
from path_preserving_deduplication import durable_json, file_hash, metadata
from sdf_cfd_mesher_v4.common.authority import identity


def publish(base, adapter, out, parent):
    original_index = storage.write_proof_index
    original_prepare = publication.prepare
    original_connect = sqlite3.connect
    events = []

    def measured_index(*args):
        tick = measured()
        result = original_index(*args)
        durable_json(out/'INDEX_MEASURED.json', dict(metrics=metrics(*tick), writer=result,
            file=metadata(args[0]), sha256=file_hash(args[0])))
        return result

    def prepare(base, adapter, out):
        plan = original_prepare(base, adapter, out)
        tick = measured()
        cert = access.read(out/'FRESH_CERTIFICATE.json')
        descriptor = cert['segment']

        def verify(path):
            expected = next(v['sha256'] for v in descriptor.values()
                if isinstance(v, dict) and v.get('file') == path.name)
            if file_hash(path) != expected:
                raise ValueError('ChangedNewProofSegment')

        class NewSegmentInput:
            content_root = identity(dict(new_segments=[descriptor]))

            def proof_segments(self):
                yield descriptor, out/'objects', verify

            def __getitem__(self, key):
                if key != 'certificate':
                    raise KeyError(key)
                return range(descriptor['count'])

        root = storage.build_incidence(NewSegmentInput(), out/'new_incidence')
        added = access.read(out/'new_incidence/MANIFEST.json')
        if parent.body['parent_count'] + descriptor['count'] != cert['combined_pairs']:
            raise ValueError('WrongChildLiveCount')
        merged = dict(added, parent_content=plan['scientific'], parent_count=cert['combined_pairs'],
            segments=parent.body['segments']+added['segments'],
            files={**parent.body['files'], **added['files']}, inherited_index_root=parent.root)
        path = out/'INCIDENCE_MANIFEST.json'
        durable_json(path, merged)
        plan['pair_incidence_root'] = identity(merged)
        plan['pair_incidence_manifest'] = str(path.resolve())
        plan['references'].update(merged['files'])
        plan['references'][str(path.resolve())] = access.attest(path, file_hash(path))
        durable_json(out/'CANDIDATE_PLAN.json', plan)
        durable_json(out/'INCIDENCE_MEASURED.json', dict(root=identity(merged),
            new_segment_root=root, inherited_segments=len(parent.body['segments']),
            new_pairs=descriptor['count'], prior_pair_rows_scanned=0, metrics=metrics(*tick)))
        return plan

    class MeasuredConnection(sqlite3.Connection):
        def __init__(self, *args, **kwargs):
            self.filename = str(args[0])
            super().__init__(*args, **kwargs)

        def commit(self):
            tick = measured()
            result = super().commit()
            events.append(dict(file=self.filename, operation='commit', metrics=metrics(*tick)))
            return result

        def executemany(self, *args, **kwargs):
            tick = measured()
            result = super().executemany(*args, **kwargs)
            events.append(dict(file=self.filename, operation='executemany', metrics=metrics(*tick)))
            return result

    def connect(*args, **kwargs):
        kwargs['factory'] = MeasuredConnection
        return original_connect(*args, **kwargs)

    with patch.object(storage, 'write_proof_index', measured_index), \
            patch.object(publication, 'prepare', prepare), patch.object(sqlite3, 'connect', connect):
        root = publication.publish(base, adapter, out)
    durable_json(out/'SQL_COMMITS.json', events)
    reference = dict(kind='recursive', directory=str((out/'candidate').resolve()), root=root)
    durable_json(out/'REFERENCE.json', reference)
    return reference
