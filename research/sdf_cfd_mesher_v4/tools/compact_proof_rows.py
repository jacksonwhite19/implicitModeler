"""Lossless column storage for repeated exact proof records.

Pair indices and interned payload identifiers are unsigned 64-bit integers.
Payloads retain the same normalized JSON values; no scientific float changes.
This is list-compatible for the existing evidence streaming interfaces.
"""
from array import array
from copy import deepcopy
import json
import numpy as np


class CompactProofRows(list):
    def __init__(self, rows=()):
        self.pairs = array('Q')
        self.codes = array('Q')
        self.payloads = []
        self.lookup = {}
        for row in rows:
            self.append(row)

    def __len__(self):
        return len(self.codes)

    def append(self, row):
        pair = row['pair']
        if len(pair) != 2 or any(int(v) != v or not 0 <= v < 2**64 for v in pair):
            raise ValueError('InvalidCompactProofPair')
        value = {k: v for k, v in row.items() if k != 'pair'}
        encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
        code = self.lookup.get(encoded)
        if code is None:
            code = len(self.payloads)
            self.lookup[encoded] = code
            self.payloads.append(json.loads(encoded))
        self.pairs.extend(map(int, pair))
        self.codes.append(code)

    def extend(self, rows):
        for row in rows:
            self.append(row)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        if index < 0:
            index += len(self)
        if not 0 <= index < len(self):
            raise IndexError(index)
        result = {k: deepcopy(v) if isinstance(v, (dict, list)) else v
                  for k, v in self.payloads[self.codes[index]].items()}
        result['pair'] = [self.pairs[2*index], self.pairs[2*index+1]]
        return result

    def __iter__(self):
        for index in range(len(self)):
            yield self[index]

    def __eq__(self, other):
        if not hasattr(other, '__len__') or len(self) != len(other):
            return False
        return all(a == b for a, b in zip(self, other))

    def __ne__(self, other):
        return not self == other

    def __deepcopy__(self, memo):
        result = type(self)()
        result.pairs = self.pairs[:]
        result.codes = self.codes[:]
        result.payloads = deepcopy(self.payloads, memo)
        result.lookup = self.lookup.copy()
        return result

    def ordered_unique(self):
        pairs = np.frombuffer(self.pairs, dtype=np.uint64).reshape(-1, 2)
        order = np.lexsort((pairs[:, 1], pairs[:, 0]))
        ordered = pairs[order]
        if len(ordered) > 1 and np.any(np.all(ordered[1:] == ordered[:-1], axis=1)):
            raise ValueError('DuplicateCertificateProof')
        codes = np.frombuffer(self.codes, dtype=np.uint64)[order]
        self.pairs = array('Q', ordered.ravel())
        self.codes = array('Q', codes)
        return self

    def pair_iterator(self):
        for i in range(len(self)):
            yield self.pairs[2*i], self.pairs[2*i+1]

    def storage(self):
        from retained_state_storage import retained_storage
        owned=retained_storage(dict(pair_buffer=self.pairs,code_buffer=self.codes,
                                    payloads=self.payloads,payload_lookup=self.lookup))
        return dict(records=len(self), pair_bytes=self.pairs.buffer_info()[1]*self.pairs.itemsize,
                    code_bytes=self.codes.buffer_info()[1]*self.codes.itemsize,
                    distinct_payloads=len(self.payloads),
                    canonical_payload_bytes=sum(len(x.encode()) for x in self.lookup),
                    owned_python_bytes=owned['total_unique_python_bytes'])

    def summary(self):
        from collections import Counter
        counts=Counter(self.codes)
        methods=Counter()
        invalid=set()
        for code,count in counts.items():
            value=self.payloads[code]
            methods[value['method']]+=count
            if value['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
                invalid.add(code)
        failures=[self[i] for i,code in enumerate(self.codes) if code in invalid] if invalid else []
        return methods,failures
