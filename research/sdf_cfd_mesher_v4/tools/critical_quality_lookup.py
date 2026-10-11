"""Bounded indexed quality probes under the existing immutable database guards."""
from pathlib import Path
from contextlib import closing
import json
import sqlite3
import zlib


def selected(base, keys):
    keys = list(keys)
    if len(set(keys)) != len(keys):
        raise ValueError('DuplicateQualityRequest')
    base.check()
    if base.kind != 'recursive' or not base.production:
        return {key: base['quality'][key] for key in keys}
    removed = set(base.delta['removed'].get('quality', []))
    if removed.intersection(keys):
        raise KeyError(next(iter(removed.intersection(keys))))
    path = base.verify(Path(base.body['quality_delta']))
    result = {}
    with closing(sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1', uri=True)) as db:
        for start in range(0, len(keys), 512):
            batch = keys[start:start+512]
            sql = 'SELECT key,inputs,payload FROM updates WHERE key IN ('+','.join('?' for _ in batch)+')'
            for key, inputs, payload in db.execute(sql, batch):
                base.stats['quality_rows_visited'] += 1
                if payload is None:
                    value = base.parent['quality'][key]
                    base.stats['quality_parent_lookups'] += 1
                else:
                    value = json.loads(zlib.decompress(payload))
                value['inputs'] = json.loads(zlib.decompress(inputs))
                result[key] = value
            base.stats['quality_sql_batches'] += 1
    for key in keys:
        if key not in result:
            result[key] = base.parent['quality'][key]
            base.stats['quality_parent_lookups'] += 1
    base.stats['quality_lookups'] += len(keys)
    base.stats['quality_components_touched'] += len(keys)
    base.stats['quality_point_queries'] += len(keys)
    base.verify(path)
    base.check()
    return result
