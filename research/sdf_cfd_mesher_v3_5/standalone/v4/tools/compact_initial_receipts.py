"""Cold binding of existing compact proofs to the native ReceiptTable interface.

The SQL view renders the SAME canonical receipt bytes. Shared face dependencies
and proof payloads are stored once; keys and relations remain individually owned.
No certificate, dependency, authentication root or checkpoint format changes.
"""
from contextlib import closing
from pathlib import Path
import os,sqlite3,tempfile
import numpy as np
from compact_proof_rows import CompactProofRows
from validation_row_stream import rows
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ReceiptTable,ENCODER,scientific_digest


def bind_compact(directory,population,face_keys,inputs):
    """Return None for unsupported populations; the existing binder handles them."""
    source=getattr(population,'source',population)
    from optimized_mixed_certificate import PackedRows
    if type(source) not in (CompactProofRows,PackedRows):return None
    methods,bad=source.summary()
    if bad:raise ValueError('FailedCompactInitialProof')
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='compact-binding-',suffix='.pending',dir=directory)
    os.close(fd);pending=Path(name)
    try:
        with closing(sqlite3.connect(pending)) as db:
            db.execute('PRAGMA synchronous=FULL')
            db.execute('PRAGMA foreign_keys=ON')
            db.executescript('''
                CREATE TABLE source_faces(id INTEGER PRIMARY KEY,name TEXT NOT NULL UNIQUE,
                    quoted TEXT NOT NULL,dependency TEXT NOT NULL);
                CREATE TABLE payloads(id INTEGER PRIMARY KEY,value TEXT NOT NULL);
                CREATE TABLE authorities(value TEXT NOT NULL);
                CREATE TABLE relations(key TEXT PRIMARY KEY,a INTEGER NOT NULL REFERENCES source_faces(id),
                    b INTEGER NOT NULL REFERENCES source_faces(id),payload INTEGER NOT NULL REFERENCES payloads(id),
                    CHECK(a<>b)) WITHOUT ROWID;
                CREATE VIEW receipts AS SELECT r.key AS key,
                    '{"faces":['||a.quoted||','||b.quoted||'],"inputs":{'||t.value||','||
                    a.dependency||','||b.dependency||'},"value":'||p.value||'}' AS value
                    FROM relations r JOIN source_faces a ON a.id=r.a
                    JOIN source_faces b ON b.id=r.b JOIN payloads p ON p.id=r.payload
                    CROSS JOIN authorities t;
            ''')
            if len(set(face_keys))!=len(face_keys):raise ValueError('DuplicateCompactFaceIdentity')
            for i,f in enumerate(face_keys):
                key='face:'+f
                db.execute('INSERT INTO source_faces VALUES(?,?,?,?)',
                    (i,f,ENCODER.encode(f),ENCODER.encode(key)+':'+ENCODER.encode(inputs[key])))
            dependencies=['authority:certificate']
            if 'authority:certificate_filter' in inputs:dependencies.append('authority:certificate_filter')
            db.execute('INSERT INTO authorities VALUES(?)',(','.join(
                ENCODER.encode(k)+':'+ENCODER.encode(inputs[k]) for k in dependencies),))
            for i,p in enumerate(rows(source.payloads)):
                db.execute('INSERT INTO payloads VALUES(?,?)',(i,ENCODER.encode(p)))
            pairs=np.frombuffer(source.pairs,dtype=np.uint64).reshape(-1,2)
            codes=np.frombuffer(source.codes,dtype=np.uint64)
            try:
                for pair,code in zip(pairs,codes,strict=True):
                    a,b=map(int,pair)
                    if not (0<=a<len(face_keys) and 0<=b<len(face_keys)):
                        raise ValueError('InvalidCompactInitialFace')
                    if face_keys[a]>face_keys[b]:a,b=b,a
                    db.execute('INSERT INTO relations VALUES(?,?,?,?)',
                        (identity([face_keys[a],face_keys[b]]),a,b,int(code)))
            except sqlite3.IntegrityError as error:
                raise ValueError('InvalidOrDuplicateCompactInitialProof') from error
            db.commit()
            if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('CompactInitialIntegrity')
            if db.execute('SELECT COUNT(*) FROM receipts').fetchone()[0]!=len(source):
                raise ValueError('CompactInitialCoverage')
        with pending.open('r+b') as stream:os.fsync(stream.fileno())
        table=ReceiptTable(pending,len(source),'')
        digest=scientific_digest(table);target=directory/(digest+'.sqlite')
        if target.exists():
            if scientific_digest(ReceiptTable(target,len(source),digest))!=digest:
                raise ValueError('ExistingCompactInitialTableCorrupt')
            pending.unlink()
        else:os.replace(pending,target)
        return ReceiptTable(target,len(source),digest)
    finally:
        # Only this unpublished private transaction. Never old interrupted files.
        pending.unlink(missing_ok=True)
        Path(str(pending)+'-journal').unlink(missing_ok=True)
