"""Renew index access metadata with the capsule's guarded byte authentication.

Original manifests and payloads are never written. Normal readers, predicates,
source locks and epoch guards are unchanged. Only explicit pinned inputs can
create a new current-epoch access graph; this is not scientific qualification.
"""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import immutable_state_access as access
from recursive_epoch_recovery import refresh_records
from sdf_cfd_mesher_v4.common.authority import identity, file_hash
from path_preserving_deduplication import durable_json


def dependency_union(primary, ancestry):
    """Same immutable bytes may carry different epoch-access tokens."""
    result = deepcopy(ancestry)
    for p, r in primary.items():
        if p in result and result[p]['sha256'] != r['sha256']:
            raise ValueError('ConflictingIndexRenewalDependency', p)
        result[p] = deepcopy(r)
    return result


class IndexRenewal:
    def __init__(self, directory, records, expected_records):
        if identity(records) != expected_records:
            raise ValueError('WrongIndexRenewalDependencies')
        self.directory = Path(directory)
        self.directory.mkdir(exist_ok=False)
        self.records = deepcopy(records)
        self.records_identity = expected_records
        self.held = ExitStack()
        self.authenticated = {}
        self.manifests = {}
        self.segments = {}
        self.graphs = {}
        self.epoch = access.epoch()
        self.closed = False
        self.events = []

    def __enter__(self):
        try:
            for module in (__file__, refresh_records.__code__.co_filename,
                           access.__file__):
                self.held.enter_context(access.guard(module))
        except BaseException:
            self.closed = True
            self.held.close()
            raise
        return self

    def check(self):
        if self.closed:
            raise ValueError('ClosedIndexRenewal')
        if identity(self.records) != self.records_identity:
            raise ValueError('ChangedIndexRenewalDependencies')
        if access.epoch() != self.epoch:
            raise ValueError('IndexRenewalEpochChanged')
        for p, r in self.authenticated.items():
            if access.token(p) != r['token']:
                raise ValueError('ChangedRenewedIndexDependency')

    def __exit__(self, *exc):
        try:
            if exc[0] is None:
                self.check()
        finally:
            self.closed = True
            self.held.close()

    def authenticate(self, path, expected_hash=None):
        self.check()
        p = str(Path(path).resolve())
        r = self.records.get(p)
        if r is None or expected_hash is not None and r['sha256'] != expected_hash:
            raise ValueError('UnboundIndexRenewalDependency', p)
        return refresh_records({p: r}, self.held, self.authenticated)[p]

    def manifest(self, path, root, schema):
        p = str(Path(path).resolve())
        self.authenticate(p)
        body = access.read(p)
        if identity(body) != root or body['schema'] != schema:
            raise ValueError('WrongIndexRenewalSourceRoot', p)
        self.manifests[p] = deepcopy(body)
        return body

    def files(self, body, named):
        if named:
            return {name: dict(path=r['path'], **self.authenticate(r['path'], r['sha256']))
                    for name, r in body['files'].items()}
        return {p: self.authenticate(p, r['sha256']) for p, r in body['files'].items()}

    def write(self, body, kind, source):
        self.check()
        dest = self.directory / (kind + '_' + str(len(self.events)))
        dest.mkdir()
        path = dest / 'MANIFEST.json'
        durable_json(path, body)
        self.held.enter_context(access.guard(path))
        if access.read(path) != body:
            raise ValueError('RenewedIndexReadbackMismatch')
        event = dict(source=str(Path(source).resolve()), source_root=identity(self.manifests[str(Path(source).resolve())]),
                     path=str(path.resolve()), root=identity(body), sha256=file_hash(path))
        self.events.append(event)
        return dict(directory=str(dest.resolve()), root=identity(body))

    def geometry(self, reference):
        """Only epoch, tokens, and corresponding access links may change."""
        self.check()
        key = identity(reference)
        if key in self.graphs:
            return deepcopy(self.graphs[key])
        ref = deepcopy(reference)
        if 'child_content' in ref:
            parent = self.geometry(ref['parent'])
            delta = self.segment(ref['delta'], ref['parent'], parent)
            ref['parent'], ref['delta'] = parent, delta
            version = ref.get('version_closure')
            if version is not None and 'path' in version:
                p = Path(version['path'])
                body = self.manifest(p, version['root'], 'V4GeometryVersionClosure/1')
                if (body['parent_content'] != reference['parent']['content'] or
                    body['numeric_root'] != reference['parent']['root'] or not body['closed']):
                    raise ValueError('WrongIndexRenewalVersionClosure')
                body['numeric_root'] = parent['root']
                written = self.write(body, 'version', p)
                ref['version_closure'] = dict(path=str(Path(written['directory'])/'MANIFEST.json'), root=written['root'])
            self.graphs[key] = deepcopy(ref)
            return ref
        result = self.segment(ref, None, None)
        self.graphs[key] = deepcopy(result)
        return result

    def segment(self, reference, old_parent, new_parent):
        if set(reference) not in ({'directory','root','content'}, {'directory','root','content','lookup'}):
            raise ValueError('UnsupportedIndexRenewalReference')
        key = identity(dict(reference=reference, parent=old_parent))
        if key in self.segments:
            return deepcopy(self.segments[key])
        from persistent_numeric_incidence import SCHEMA
        ref = deepcopy(reference)
        p = Path(ref['directory'])/'MANIFEST.json'
        body = self.manifest(p, ref['root'], SCHEMA)
        if body['parent_content'] != ref['content'] or body['parent_segment'] != old_parent:
            raise ValueError('WrongIndexRenewalGeometryAncestry')
        body['files'] = self.files(body, True)
        body['epoch'] = self.epoch
        body['parent_segment'] = deepcopy(new_parent)
        new = dict(self.write(body, 'numeric', p), content=ref['content'])
        if 'lookup' in ref:
            lp = Path(ref['lookup']['directory'])/'MANIFEST.json'
            lookup = self.manifest(lp, ref['lookup']['root'], 'V4SourceFaceLookup/1')
            if lookup['index_root'] != ref['root'] or lookup['content'] != ref['content']:
                raise ValueError('WrongIndexRenewalLookupBinding')
            lookup['files'] = self.files(lookup, True)
            lookup['epoch'], lookup['index_root'] = self.epoch, new['root']
            new['lookup'] = self.write(lookup, 'lookup', lp)
        self.segments[key] = deepcopy(new)
        return new

    def pairs(self, reference, content):
        from incremental_pair_storage import SCHEMA, __file__ as implementation
        path = Path(reference['path'])
        body = self.manifest(path, reference['root'], SCHEMA)
        if body['parent_content'] != content or body['implementation'] != file_hash(implementation):
            raise ValueError('WrongIndexRenewalPairBinding')
        body['files'] = self.files(body, False)
        body['epoch'] = self.epoch
        result = self.write(body, 'pairs', path)
        return dict(path=str(Path(result['directory'])/'MANIFEST.json'), root=result['root'])

    def evidence(self):
        self.check()
        return dict(schema='V4PersistentIndexAccessRenewal/1', epoch=self.epoch,
                    dependencies=self.records_identity, implementation=file_hash(__file__),
                    files=len(self.authenticated), bytes_hashed=sum(Path(p).stat().st_size for p in self.authenticated),
                    events=deepcopy(self.events), payloads_copied=0,
                    scope='Pinned original index roots and bytes; access-only metadata with current guards/tokens; no new scientific proofs.')
