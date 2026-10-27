"""Exact scientific comparison across the qualified certificate storage change.

This is not a relaxed mesh comparator. Every geometry, quality, decision and
conditioning field must remain exact. Certificate outcomes use the complete
pair and outcome hashes already qualified one-for-one against the old format.
Raw storage-chain hashes are reported separately and are never called equal.
"""
from ..forecast.detailed.scientific import scientific_hash

ENVELOPE_FIELDS={'state_hash','previous','certificate_hash','certificate'}
FINAL_ENVELOPE_FIELDS={'shell_scientific_hash','transaction_state_hash','certificate'}

def compare_checkpoint(a,b,qualified_proof):
    for key in ('candidate_pairs','completed_pairs','pair_hash','outcome_hash'):
        if b['certificate'][key]!=qualified_proof[key]:raise ValueError('CertificateMathematicalOutcomeMismatch:'+key)
    if any(b['certificate'][k] for k in ('forbidden','unresolved','invalid_surfaces')):raise ValueError('NonpassingCertificate')
    if not b['certificate']['complete']:raise ValueError('IncompleteCertificate')
    if a['certificate']['candidate_pairs']!=qualified_proof['candidate_pairs']:raise ValueError('FrozenPopulationMismatch')
    if scientific_hash({k:v for k,v in a.items() if k not in ENVELOPE_FIELDS})!=scientific_hash({k:v for k,v in b.items() if k not in ENVELOPE_FIELDS}):raise ValueError('TransactionScientificMismatch')
    return dict(layer=a['layer'],pair_hash=qualified_proof['pair_hash'],outcome_hash=qualified_proof['outcome_hash'],
        candidate_pairs=qualified_proof['candidate_pairs'],scientific_state_hash=scientific_hash({k:v for k,v in a.items() if k not in ENVELOPE_FIELDS}))

def compare_final_values(a,b,coverage):
    for x in (a,b):
        if not x['certificate']['complete'] or any(x['certificate'][k] for k in ('forbidden','unresolved','invalid_surfaces')):raise ValueError('FinalCertificateNotPassing')
        if sum(r['candidate_pairs'] for r in coverage)!=x['certificate']['candidate_pairs']:raise ValueError('FinalCoverageMismatch')
    aa={k:v for k,v in a.items() if k not in FINAL_ENVELOPE_FIELDS};bb={k:v for k,v in b.items() if k not in FINAL_ENVELOPE_FIELDS}
    if scientific_hash(aa)!=scientific_hash(bb):raise ValueError('FinalScientificMismatch')
    aa['complete_mathematical_certificate']=coverage;bb['complete_mathematical_certificate']=coverage
    return scientific_hash(aa),scientific_hash(bb)
