"""Lossless cold quality binding using the existing native receipt SQL view.

Only repeated input-member text is shared. Each original receipt, result,
dependency and prospective-interface distinction is reconstructed unchanged.
ReceiptTable readers, scientific hashes, imports and authentication stay native.
"""
from contextlib import closing
from pathlib import Path
import hashlib,os,sqlite3,tempfile
from collections.abc import Mapping
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER,ReceiptTable,scientific_digest,canonical_chunks


def parts(value,*,member_field='inputs'):
    if member_field not in ('inputs','dependencies'):raise ValueError('UnsupportedColdMemberField')
    if not isinstance(value,Mapping):raise ValueError('UnsupportedColdQualityRow')
    if any(not isinstance(k,str) for k in value):raise ValueError('NativeScientificKeysMustBeStrings')
    if member_field not in value:return ENCODER.encode(dict(value)),[],''
    inputs=value[member_field]
    if not isinstance(inputs,Mapping) or any(not isinstance(k,str) for k in inputs):
        raise ValueError('UnsupportedColdQualityInputs')
    keys=sorted(value);i=keys.index(member_field)
    member=lambda k:ENCODER.encode(k)+':'+ENCODER.encode(value[k])
    before=','.join(member(k) for k in keys[:i])
    after=','.join(member(k) for k in keys[i+1:])
    prefix='{'+(before+',' if before else '')+ENCODER.encode(member_field)+':{'
    suffix='}'+(','+after if after else '')+'}'
    return prefix,[ENCODER.encode(k)+':'+ENCODER.encode(inputs[k]) for k in sorted(inputs)],suffix


def bind_shared(directory,rows,*,progress=None,member_field='inputs'):
    """Stream sorted cold rows; independently bind original canonical bytes.

    Native identity is key-ordered. Unordered or duplicate inputs fail closed.
    """
    if member_field not in ('inputs','dependencies'):raise ValueError('UnsupportedColdMemberField')
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='quality-binding-',suffix='.pending',dir=directory)
    os.close(fd);pending=Path(name)
    try:
        with closing(sqlite3.connect(pending)) as db:
            db.execute('PRAGMA synchronous=FULL');db.execute('PRAGMA foreign_keys=ON')
            db.executescript('''
                CREATE TABLE dependency_members(id INTEGER PRIMARY KEY,fragment TEXT NOT NULL UNIQUE);
                CREATE TABLE quality_rows(id INTEGER PRIMARY KEY,key TEXT NOT NULL UNIQUE,prefix TEXT NOT NULL,suffix TEXT NOT NULL);
                CREATE TABLE dependency_uses(row INTEGER NOT NULL REFERENCES quality_rows(id),
                    ordinal INTEGER NOT NULL,member INTEGER NOT NULL REFERENCES dependency_members(id),
                    PRIMARY KEY(row,ordinal)) WITHOUT ROWID;
                CREATE VIEW receipts AS SELECT r.key AS key,r.prefix || COALESCE(
                    (SELECT group_concat(fragment,',') FROM
                        (SELECT d.fragment FROM dependency_uses u JOIN dependency_members d ON d.id=u.member
                         WHERE u.row=r.id ORDER BY u.ordinal)), '') || r.suffix AS value
                    FROM quality_rows r;
            ''')
            count=0;previous=None;accepted=hashlib.sha256();accepted.update(b'{')
            for key,value in rows:
                if not isinstance(key,str):raise ValueError('NativeScientificKeysMustBeStrings')
                if previous is not None and key<=previous:raise ValueError('UnorderedOrDuplicateColdQualityReceipt')
                previous=key
                if count:accepted.update(b',')
                accepted.update(ENCODER.encode(key).encode());accepted.update(b':')
                for chunk in canonical_chunks(value):accepted.update(chunk)
                prefix,members,suffix=(parts(value) if member_field=='inputs' else parts(value,member_field=member_field))
                try:
                    db.execute('INSERT INTO quality_rows VALUES(?,?,?,?)',(count,key,prefix,suffix))
                    for ordinal,fragment in enumerate(members):
                        db.execute('INSERT OR IGNORE INTO dependency_members(fragment) VALUES(?)',(fragment,))
                        member=db.execute('SELECT id FROM dependency_members WHERE fragment=?',(fragment,)).fetchone()[0]
                        db.execute('INSERT INTO dependency_uses VALUES(?,?,?)',(count,ordinal,member))
                except sqlite3.IntegrityError as error:raise ValueError('DuplicateColdQualityReceipt') from error
                count+=1
                if progress and count%2048==0:progress()
            db.commit()
            accepted.update(b'}')
            if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('ColdQualityIntegrity')
            if db.execute('PRAGMA foreign_key_check').fetchone() is not None:raise ValueError('ColdQualityDependencies')
            if db.execute('SELECT count(*) FROM receipts').fetchone()[0]!=count:raise ValueError('ColdQualityCoverage')
        with pending.open('r+b') as stream:os.fsync(stream.fileno())
        if progress:progress()
        table=ReceiptTable(pending,count,'');digest=scientific_digest(table)
        if digest!=accepted.hexdigest():raise ValueError('ColdQualityAcceptedInputMismatch')
        target=directory/(digest+'.sqlite')
        if target.exists():
            if scientific_digest(ReceiptTable(target,count,digest))!=digest:raise ValueError('ExistingColdQualityCorrupt')
            pending.unlink()
        else:os.replace(pending,target)
        # A fresh read connection validates the durable materialized table.
        result=ReceiptTable(target,count,digest)
        if scientific_digest(result)!=digest:raise ValueError('ColdQualityReadbackMismatch')
        return result
    finally:
        pending.unlink(missing_ok=True);Path(str(pending)+'-journal').unlink(missing_ok=True)


def bind_state(state,directory):
    """Cold-only connection; preserve inherited native/delta/overlay behavior."""
    if type(state['quality']) is not dict:return False
    values=state['quality']
    state['quality']=bind_shared(directory,((k,values[k]) for k in sorted(values)))
    return True
