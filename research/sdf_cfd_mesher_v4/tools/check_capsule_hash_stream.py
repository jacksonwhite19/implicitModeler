"""Read-only retained capsule receipt control; no front or checkpoint writes."""
from contextlib import ExitStack, closing
import json
from pathlib import Path
import sqlite3
import zlib

import immutable_state_access as access
from incremental_transaction import Store
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity
import single_quality_component as quality
from critical_quality_codec import encoded
from private_quality_hash_stream import hashes


def main():
    root=Path(__file__).resolve().parents[1]
    # The old speed-control payload was retired after qualification. Use the
    # retained enrolled capsule front, not deep recovery or rebuilt geometry.
    cases=root/'results/sealed_front_connection/CASES.json'
    case=access.read(cases)['capsule'];folder=Path(case['reference']['directory'])
    out=root/'results/assembly_row_efficiency/CAPSULE_CONTROL.json'
    if out.exists():raise ValueError('PreserveExistingCapsuleControl')
    with ExitStack() as held:
        held.enter_context(access.guard(folder/'CURRENT.json'))
        store=Store(folder);pointer=store.current()
        if pointer['root']!=case['reference']['root']:raise ValueError('WrongCapsuleControlParent')
        held.enter_context(access.guard(folder/pointer['file']))
        body=store.root(pointer['root'])
        source=Path(body['quality_delta']);held.enter_context(access.guard(source))
        expected=body['produced'][str(source.resolve())]
        if access.token(source)!=expected['token'] or file_hash(source)!=expected['sha256']:
            raise ValueError('ChangedRetainedCapsuleQualitySource')
        db=held.enter_context(closing(sqlite3.connect(source.as_uri()+'?mode=ro&immutable=1',uri=True)))
        if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('CorruptCapsuleQualityControl')
        count=db.execute('SELECT count(*) FROM updates').fetchone()[0]
        if count!=body['acceptance']['quality_count']:raise ValueError('IncompleteCapsuleQualityControl')
        rows=[];blobs=[]
        for i,(key,inputs,payload) in enumerate(db.execute('SELECT key,inputs,payload FROM updates ORDER BY key')):
            if i%21 or len(rows)>=10001:continue
            rows.append(dict(key=key,inputs=json.loads(zlib.decompress(inputs)),
                payload=None if payload is None else json.loads(zlib.decompress(payload))))
            blobs.append((inputs,payload))
        old=quality.hashes(rows);results=[]
        for workers,process in ((1,False),(2,False),(4,True)):
            accepted=[]
            if hashes(rows,accepted,workers,process)!=old:raise ValueError('CapsuleQualityHashMismatch')
            if list(encoded(rows,workers,process))!=blobs:raise ValueError('CapsuleQualityBlobMismatch')
            results.append(dict(workers=workers,process=process,hashes_exact=True,blobs_exact=True))
        durable_json(out,dict(passed=True,scope='10001 key-ordered capsule receipt fixtures, not original list ordering or a new capsule campaign',
            parent_root=pointer['root'],scientific=body['scientific'],source=str(source),source_reference=expected,
            case_reference=case['reference'],case_file_sha256=file_hash(cases),
            source_rows=count,sampled_rows=len(rows),logical=old,results=results,
            sources={str(p):file_hash(p) for p in (Path(__file__),Path(quality.__file__),Path(__file__).with_name('private_quality_hash_stream.py'))}))
    print('Capsule source authenticated; hashes and compressed blobs exact for',len(rows),'rows',flush=True)


if __name__=='__main__':main()
