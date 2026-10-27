"""Complete saved certificate stream regression, without new certification."""
from pathlib import Path
import gzip
import hashlib
import capsule_user_domain_handoff as h

if __name__ == '__main__':
    out = Path(__file__).resolve().parents[1] / 'results/remaining_prism_critical_path'
    source = out.parent / 'production_hotpath_optimization/raw_release_optimized_certificate/PROOFS.jsonl.gz'
    tick = h.measured()
    digest = hashlib.sha256(b'[')
    count = 0
    with gzip.open(source, 'rb') as stream:
        for line in stream:
            if count:
                digest.update(b',')
            digest.update(line.rstrip(b'\r\n'))
            count += 1
    digest.update(b']')
    passed = count == 19941573 and digest.hexdigest() == '9ce3df428c1784391429ebf18a3385429fde0b679c0fc85b32ec9499ac715c1a'
    h.durable_json(out / 'CERTIFICATE_REGRESSION.json', dict(passed=passed, rows=count, canonical_hash=digest.hexdigest(), metrics=h.metrics(*tick)))
    assert passed
