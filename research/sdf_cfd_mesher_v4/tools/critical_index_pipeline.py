"""Private proof-index preparation overlaps assembly; publication still waits."""
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from contextlib import ExitStack
from pathlib import Path
import time
import os
import numpy as np
import immutable_state_access as access
from path_preserving_deduplication import file_hash, durable_json, metadata

JOBS = {}


def process_build(path, faces_path, pairs_path):
    """One isolated qualified writer; no shared Python interpreter/GIL."""
    from prism_proof_index import write_proof_index
    import psutil
    start = time.perf_counter()
    cpu = time.process_time()
    result = write_proof_index(path, access.read(faces_path), np.asarray(np.load(pairs_path, mmap_mode='r', allow_pickle=False)))
    return result, dict(wall=time.perf_counter()-start, cpu=time.process_time()-cpu,
                       peak_rss=psutil.Process().memory_info().peak_wset), file_hash(path), access.token(path)


class IndexJob:
    def __init__(self, base, out, writer, *, retained_sha256=None):
        self.base = base
        self.root = base.root
        self.out = Path(out)
        self.held = ExitStack()
        self.process = os.environ.get('V4_INDEX_PROCESS') == '1'
        if self.process:
            from multiprocessing import get_context
            self.pool = ProcessPoolExecutor(1, mp_context=get_context('spawn'))
        else:
            self.pool = ThreadPoolExecutor(1)
        self.tokens = {}
        self.future = None
        self.retained = None
        try:
            cert_path = self.out / 'FRESH_CERTIFICATE.json'
            self.guard(cert_path)
            self.cert = access.read(cert_path)
            self.faces_path = self.out / 'objects' / self.cert['segment']['faces']['file']
            self.pairs_path = self.out / 'objects' / self.cert['segment']['pairs']['file']
            for key, path in (('faces', self.faces_path), ('pairs', self.pairs_path)):
                self.guard(path)
                if file_hash(path) != self.cert['segment'][key]['sha256']:
                    raise ValueError('ChangedIndexSource')
            self.faces = access.read(self.faces_path)
            self.pairs = np.asarray(np.load(self.pairs_path, mmap_mode='r', allow_pickle=False))
            self.path = self.out / 'objects/new-proof-key-index.sqlite'
            if retained_sha256 is not None:
                # Restart has no live producer witness. Re-establish it through
                # the unchanged writer, not a caller assertion about an index.
                self.retained = self.path
                self.guard(self.retained)
                if file_hash(self.retained) != retained_sha256:
                    raise ValueError('ChangedRetainedProofIndex')
                self.retained_sha256 = retained_sha256
                self.path = self.out / 'objects/restart-proof-key-index.sqlite'
            if self.path.exists():
                raise ValueError('PreserveExistingProofIndex')
            self.future = self.pool.submit(process_build, self.path, self.faces_path, self.pairs_path) if self.process else self.pool.submit(self.build, writer)
        except BaseException:
            self.close()
            raise

    def guard(self, path):
        self.held.enter_context(access.guard(path))
        self.tokens[path] = access.token(path)

    def check(self):
        self.base.check()
        if self.base.root != self.root:
            raise ValueError('WrongIndexParent')
        for path, token in self.tokens.items():
            if access.token(path) != token:
                raise ValueError('ChangedPreparedIndex')

    def build(self, writer):
        start = time.perf_counter()
        cpu = time.thread_time()
        result = writer(self.path, self.faces, self.pairs)
        self.guard(self.path)
        self.sha256 = file_hash(self.path)
        self.timing = dict(wall=time.perf_counter()-start, thread_cpu=time.thread_time()-cpu,
                           scope='Index thread; process I/O belongs to overlapping complete control')
        durable_json(self.out / 'INDEX_PIPELINE.json', dict(metrics=self.timing, writer=result,
                     sha256=self.sha256, file=metadata(self.path)))
        return result

    def consume(self, base, faces, pairs):
        if base is not self.base or faces != self.faces or not np.array_equal(pairs, self.pairs):
            raise ValueError('WrongPreparedIndexInput')
        result = self.future.result()
        if self.process:
            result, self.timing, self.sha256, token = result
            if self.path not in self.tokens:
                self.guard(self.path)
            if access.token(self.path) != token or file_hash(self.path) != self.sha256:
                raise ValueError('ChangedPreparedIndex')
            durable_json(self.out / 'INDEX_PIPELINE.json', dict(metrics=self.timing, writer=result,
                         sha256=self.sha256, file=metadata(self.path), isolated_process=True))
        self.check()
        if result['rows'] != self.cert['required_pairs']:
            raise ValueError('IncompleteNewProofIndex')
        if self.retained is not None:
            if self.sha256 != self.retained_sha256 or file_hash(self.retained) != self.sha256:
                raise ValueError('RetainedProofIndexReconstructionMismatch')
            durable_json(self.out / 'INDEX_RESTART.json', dict(
                passed=True, parent=self.root, retained=str(self.retained),
                reconstruction=str(self.path), sha256=self.sha256,
                source_segment=self.cert['segment'], rows=result['rows'],
                scope='Exact qualified-writer reconstruction; no geometry or proof recomputation'))
        return result

    def close(self):
        self.pool.shutdown(wait=True)
        self.held.close()
        # The private mmap is independent of the held file guards. A completed
        # job may still be referenced by its caller when retention starts.
        value = getattr(self, 'pairs', None)
        while value is not None:
            mapping = getattr(value, '_mmap', None)
            if mapping is not None:
                if not mapping.closed:
                    mapping.close()
                break
            value = getattr(value, 'base', None)
        self.pairs = None


def prepared(base, out, faces, pairs):
    job = JOBS.get(str(out))
    return None if job is None else job.consume(base, faces, pairs)


def install():
    import composable_assembly as assembly
    import recursive_production_publication as recursive
    import composable_publication as publication
    import incremental_pair_storage as storage
    import critical_prepare
    original_assembly = assembly.assemble
    original_publish = recursive.publish
    publication.prepare = critical_prepare.prepare

    def assemble(base, adapter, out, *args, **kwargs):
        if str(out) in JOBS:
            raise ValueError('DuplicateIndexJob')
        retained_sha256 = kwargs.pop('retained_index_sha256', None)
        job = IndexJob(base, out, storage.write_proof_index, retained_sha256=retained_sha256)
        JOBS[str(out)] = job
        try:
            return original_assembly(base, adapter, out, *args, **kwargs)
        except BaseException:
            JOBS.pop(str(out)).close()
            raise

    def publish(base, adapter, out, parent):
        try:
            return original_publish(base, adapter, out, parent)
        finally:
            job = JOBS.pop(str(out), None)
            if job is not None:
                job.close()
    assembly.assemble = assemble
    recursive.publish = publish
