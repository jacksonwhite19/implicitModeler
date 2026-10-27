"""Fresh bounded integer population reduction; no retained summary cache."""
from collections import Counter
import numpy as np
from compact_proof_rows import CompactProofRows

def summary(rows):
    codes=np.frombuffer(rows.codes,dtype=np.uint64)
    counts=np.zeros(len(rows.payloads),dtype=np.int64)
    for start in range(0,len(codes),1048576):
        block=codes[start:start+1048576]
        if block.size and block.max()>=len(counts):return CompactProofRows.summary(rows)
        counts+=np.bincount(block.astype(np.intp),minlength=len(counts))
    if any(not isinstance(rows.payloads[i],dict) or 'method' not in rows.payloads[i] or
            'classification' not in rows.payloads[i] or type(rows.payloads[i]['method']) is not str or
            type(rows.payloads[i]['classification']) is not str for i in np.flatnonzero(counts)):
        return CompactProofRows.summary(rows)
    methods=Counter();invalid=set()
    for code in np.flatnonzero(counts):
        value=rows.payloads[code];methods[value['method']]+=int(counts[code])
        if value['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):invalid.add(int(code))
    failures=[rows[i] for i,code in enumerate(rows.codes) if code in invalid] if invalid else []
    return methods,failures
