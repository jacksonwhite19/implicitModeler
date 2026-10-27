"""Measure private quality SQLite writes without changing transaction controls."""
from contextlib import contextmanager
from unittest.mock import patch
import sqlite3
import threading
from pathlib import Path
import capsule_user_domain_handoff as h


@contextmanager
def measured_quality_writer(output,rows,adaptive=False,*,private_directory=None):
    original=sqlite3.connect;records=[]
    thread=threading.get_ident()
    directory=None if private_directory is None else Path(private_directory).resolve()
    def connect(*args,**kwargs):
        parent=kwargs.get('factory',sqlite3.Connection)
        # Only instrument this thread's existing private writer. Delegate foreign,
        # keyword-only and callable-factory connections without changing them.
        if (threading.get_ident()!=thread or not args or not isinstance(parent,type)
            or not issubclass(parent,sqlite3.Connection)):
            return original(*args,**kwargs)
        if directory is not None:
            source=Path(args[0])
            if (source.parent.resolve()!=directory or not source.name.startswith('quality-')
                or not source.name.endswith('.pending')):return original(*args,**kwargs)
        parent=kwargs.pop('factory',sqlite3.Connection)
        class Measured(parent):
            def __init__(self,*a,**kw):
                super().__init__(*a,**kw)
                self.profile=str(a[0]).endswith('.pending');self.inserted=0;self.samples=[]
                if self.profile:
                    self.tick=h.measured();self.name=str(a[0]);self.initial=self.properties()
            def properties(self):
                return {k:super(Measured,self).execute('PRAGMA '+k).fetchone()[0]
                    for k in ('cache_size','cache_spill','journal_mode','synchronous','page_size','page_count')}
            def execute(self,sql,*a,**kw):
                result=super().execute(sql,*a,**kw)
                if self.profile and self.inserted==0 and sql.startswith('PRAGMA page_size='):
                    # Physical packing runs after connection construction but
                    # before schema creation. Bind telemetry/cache conversion
                    # to the selected layout, not the connection's old default.
                    self.initial=self.properties()
                if self.profile and sql.startswith('INSERT INTO updates'):
                    self.inserted+=1
                    if self.inserted%10000==0:
                        pages=super().execute('PRAGMA page_count').fetchone()[0]
                        if adaptive:
                            projected=(pages*rows+self.inserted-1)//self.inserted
                            current=super().execute('PRAGMA cache_size').fetchone()[0]
                            current=current if current>=0 else (-current*1024+self.initial['page_size']-1)//self.initial['page_size']
                            super().execute('PRAGMA cache_size='+str(max(current,projected)))
                        self.samples.append(dict(rows=self.inserted,pages=pages,cache_size=super().execute('PRAGMA cache_size').fetchone()[0]))
                return result
            def close(self):
                if self.profile:
                    records.append(dict(file=self.name,rows=self.inserted,initial=self.initial,final=self.properties(),samples=self.samples,
                        adaptive=adaptive,write_window_metrics=h.metrics(*self.tick)))
                    self.profile=False
                return super().close()
        kwargs['factory']=Measured
        return original(*args,**kwargs)
    with patch.object(sqlite3,'connect',connect):
        try:yield
        finally:h.durable_json(Path(output)/'QUALITY_SQL_PROFILE.json',records)
