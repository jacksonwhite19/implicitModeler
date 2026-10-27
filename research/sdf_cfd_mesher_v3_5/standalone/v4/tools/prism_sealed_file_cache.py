"""Transaction-scoped reuse of immutable changed files; original checks remain."""
from contextlib import ExitStack, contextmanager
from pathlib import Path
from unittest.mock import patch
import json
import immutable_state_access as access


@contextmanager
def sealed_file_cache(directory):
    """Hash each immutable component once while an NTFS deny-write handle is held.

    Mutable pointer, plan, journal and pending files are deliberately excluded.
    A cache entry cannot outlive the guard or be used after context exit.
    """
    import composable_publication as publication
    import recursive_production_publication as recursive
    import path_preserving_deduplication as disk
    directory=Path(directory).resolve()
    names={'GEOMETRY_DELTA.json','QUALITY_DELTA.json','BINDING_PATCH.json',
           'NEW_ATLAS.json','NEW_FORECAST_DEPENDENCIES.json','CANDIDATE_HOT.json',
           'GENERATION.json','ASSEMBLY.json','FRESH_CERTIFICATE.json','VOLUME_DELTA.json',
           'ADMISSION.json','AFFECTED_INCIDENCE.json','NEW_RECEIPT_INPUTS.json'}
    original_hash=disk.file_hash;original_read=publication.read
    hashes={};decoded={};tokens={};stats={'hash_hits':0,'decode_hits':0,'files':0}
    with ExitStack() as held:
        def eligible(path):
            p=Path(path)
            return p.parent==directory and p.name in names and p.exists()
        def secure(path):
            p=Path(path)
            if p not in tokens:
                held.enter_context(access.guard(p));tokens[p]=access.token(p)
                stats['files']+=1
            return p
        def hashed(path):
            p=Path(path)
            if not eligible(p):return original_hash(p)
            secure(p)
            if p in hashes:stats['hash_hits']+=1
            else:hashes[p]=original_hash(p)
            return hashes[p]
        def read(path):
            p=Path(path)
            if p.name not in ('QUALITY_DELTA.json','BINDING_PATCH.json') or not eligible(p):
                return original_read(p)
            secure(p)
            if p in decoded:stats['decode_hits']+=1
            else:decoded[p]=original_read(p)
            return decoded[p]
        with patch.object(publication,'file_hash',hashed),patch.object(recursive,'file_hash',hashed),patch.object(access,'file_hash',hashed),patch.object(publication,'read',read):
            yield stats
        for p,token in tokens.items():
            if access.token(p)!=token:raise ValueError('ChangedSealedPublicationInput')


def install():
    """Opt-in only; no global file cache and no change to transaction logic."""
    import recursive_production_publication as recursive
    from prism_hotpath_v2 import install as previous
    previous()
    import incremental_pair_storage as storage
    import numpy as np
    writer=storage.write_proof_index
    def array_writer(path,faces,pairs):
        # ndarray view retains the mapping and exact dtype/order, without
        # allocating a memmap subclass for each of millions of pair rows.
        return writer(path,faces,np.asarray(pairs))
    storage.write_proof_index=array_writer
    import composable_publication as publication
    from prism_encoded_transaction import Store
    publication.Store=Store
    original=recursive.publish
    def publish(base,adapter,out,parent):
        with sealed_file_cache(out) as stats:
            result=original(base,adapter,out,parent)
        from path_preserving_deduplication import durable_json
        durable_json(out/'SEALED_FILE_CACHE.json',stats)
        return result
    recursive.publish=publish
