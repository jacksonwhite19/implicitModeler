"""Avoid a baseline proof when a complete candidate has no forbidden pairs.

This is set comparison, not a new certificate predicate. If the complete set
of forbidden candidate pairs is empty, its difference from any baseline set
is empty. Nonempty candidate sets still require the original baseline proof.
Candidate completeness and exactness are mandatory in both cases.
"""
def baseline_required(candidate):
    if not candidate.get('complete',False) or candidate.get('unresolved'):
        raise RuntimeError('FutureCertificateCannotClose')
    return bool(candidate.get('forbidden'))

def compare_forbidden(baseline,candidate):
    from .local_certificate import new_intersections
    if not baseline_required(candidate):return []
    if not baseline.get('complete',False) or baseline.get('unresolved'):
        raise RuntimeError('FutureCertificateCannotClose')
    return new_intersections(baseline,candidate)
