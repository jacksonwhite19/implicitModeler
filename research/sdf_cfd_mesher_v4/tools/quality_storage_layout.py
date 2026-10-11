"""Physical SQLite packing at the existing private quality-writer boundary.

The row codec, schema, logical identities and all durable readbacks stay owned
by single_quality_component. Old sealed files are never changed or migrated.
This context changes only the page size of NEW empty private databases. It is
not a scientific tolerance, an evidence-retention waiver or a new store.
"""
from contextlib import contextmanager
from pathlib import Path
import os
import threading
from unittest.mock import patch

import single_quality_component as quality

_lock = threading.Lock()


@contextmanager
def installed(page_size):
    if type(page_size) is not int or page_size not in (4096,8192,16384,32768,65536):
        raise ValueError('UnsupportedQualityPageSize')
    if not _lock.acquire(blocking=False):
        raise ValueError('ConcurrentQualityLayoutContext')
    thread = threading.get_ident()
    original_connect = quality.sqlite3.connect
    stats = dict(page_size=page_size,new_private_databases=0)

    def connect(database,*args,**kwargs):
        path = None
        if threading.get_ident()==thread and isinstance(database,(str,os.PathLike)):
            name = os.fspath(database)
            if not name.startswith('file:') and name != ':memory:':
                candidate = Path(name).resolve()
                if (candidate.parent.name=='quality_objects' and candidate.name.startswith('quality-')
                        and candidate.suffix=='.pending' and candidate.exists()
                        and candidate.stat().st_size==0 and candidate.stat().st_nlink==1):
                    path=candidate
        db=original_connect(database,*args,**kwargs)
        if path is not None:
            try:
                # This is before CREATE TABLE/BEGIN. Never VACUUM a live store,
                # change synchronous/journal settings or rewrite an old file.
                if db.execute('SELECT count(*) FROM sqlite_master').fetchone()[0]!=0:
                    raise ValueError('NonemptyPrivateQualityDatabase')
                db.execute('PRAGMA page_size='+str(page_size))
                if db.execute('PRAGMA page_size').fetchone()[0]!=page_size:
                    raise ValueError('QualityPageSizeNotApplied')
                stats['new_private_databases']+=1
            except BaseException:
                db.close();raise
        return db

    try:
        with patch.object(quality.sqlite3,'connect',connect):yield stats
    finally:
        _lock.release()
