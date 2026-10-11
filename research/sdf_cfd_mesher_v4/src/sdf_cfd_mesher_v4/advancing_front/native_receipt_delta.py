"""Immutable receipt overlays stored as native delta tables.

The canonical scientific map is identical to a flat table. Historical SQLite
pages are shared read-only; only updates and explicit removals are published.
"""
from contextlib import closing
import hashlib
import json
import sqlite3
from .native_checkpoint import ReceiptTable, ENCODER


def raw_items(table):
    if isinstance(table, ReceiptDelta):
        yield from table.raw_items()
    else:
        with closing(table._connect()) as db:
            yield from db.execute('SELECT key,value FROM receipts ORDER BY key')


class ReceiptDelta(ReceiptTable):
    def __init__(self, base, updates, removed=(), *, count=None, scientific=None):
        self.base = base
        self.updates = updates
        self.removed = frozenset(removed)
        self.path = updates.path
        if count is None or scientific is None:
            count, scientific = self.measure_identity()
        self.count = count
        self.scientific = scientific

    def measure_identity(self):
        digest = hashlib.sha256()
        digest.update(b'{')
        count = 0
        for key, value in self.raw_items():
            if count:
                digest.update(b',')
            digest.update(ENCODER.encode(key).encode())
            digest.update(b':')
            digest.update(value.encode())
            count += 1
        digest.update(b'}')
        return count, digest.hexdigest()

    def raw_items(self):
        old = iter(raw_items(self.base))
        new = iter(raw_items(self.updates))
        a, b = next(old, None), next(new, None)
        while a is not None or b is not None:
            if a is not None and (b is None or a[0] < b[0]):
                row = a
                a = next(old, None)
                if row[0] not in self.removed:
                    yield row
            else:
                if a is not None and a[0] == b[0]:
                    a = next(old, None)
                row = b
                b = next(new, None)
                if row[0] not in self.removed:
                    yield row

    def __iter__(self):
        for key, _ in self.raw_items():
            yield key

    def items(self):
        for key, value in self.raw_items():
            yield key, json.loads(value)

    def __getitem__(self, key):
        if key in self.removed:
            raise KeyError(key)
        try:
            return self.updates[key]
        except KeyError:
            return self.base[key]

    def canonical_chunks(self):
        yield b'{'
        for index, (key, value) in enumerate(self.raw_items()):
            if index:
                yield b','
            yield ENCODER.encode(key).encode()
            yield b':'
            yield value.encode()
        yield b'}'

    def _connect(self):
        """Expose the same SQL lookup/audit interface, without copying rows."""
        db = sqlite3.connect(':memory:', uri=True)
        aliases = {}
        def view(table):
            if not isinstance(table, ReceiptDelta):
                path = str(table.path.resolve())
                if path not in aliases:
                    name = 'part' + str(len(aliases))
                    db.execute('ATTACH DATABASE ? AS '+name,
                               (table.path.resolve().as_uri()+'?mode=ro',))
                    aliases[path] = name
                return aliases[path] + '.receipts'
            base = view(table.base)
            updates = view(table.updates)
            suffix = str(len(aliases)) + '_' + str(id(table))
            removed = 'removed_' + suffix
            db.execute('CREATE TEMP TABLE '+removed+'(key TEXT PRIMARY KEY) WITHOUT ROWID')
            db.executemany('INSERT INTO '+removed+' VALUES(?)', ((k,) for k in table.removed))
            name = 'merged_' + suffix
            db.execute('CREATE TEMP VIEW '+name+' AS SELECT key,value FROM '+updates+
                       ' WHERE key NOT IN (SELECT key FROM '+removed+') UNION ALL '+
                       'SELECT key,value FROM '+base+' WHERE key NOT IN (SELECT key FROM '+updates+
                       ') AND key NOT IN (SELECT key FROM '+removed+')')
            return name
        try:
            source = view(self)
            db.execute('CREATE TEMP VIEW receipts AS SELECT key,value FROM '+source)
            return db
        except BaseException:
            db.close()
            raise
