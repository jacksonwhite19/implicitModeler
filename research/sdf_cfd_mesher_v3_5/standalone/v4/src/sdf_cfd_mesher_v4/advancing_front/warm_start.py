"""One-step transported proposal; no accepted geometry and no future commit.

The caller must evaluate ordinary gates, future admission and exact immediate
certification. The performance-equivalence runner also requires equality with
the first canonical accepted proposal. A different valid proposal is not an
equivalence result and cannot short-circuit that runner.
"""
import numpy as np
from .private_rolling import try_tangent_proposal
from ..common.authority import array_hash

def warm_proposal(planner,top,patch,nominal,sdf,target,increment,hazard=None):
    advice=planner.advisory(top,patch,nominal,sdf,hazard=hazard)
    if advice is None:return None,dict(eligible=False,reason='NoCompatibleConsecutiveRegion')
    x,diagnostic=try_tangent_proposal(nominal,advice['direction'],np.full(len(nominal),advice['factor']*increment),sdf,target)
    return x,dict(eligible=True,source_layer=advice['source_layer'],source_mode=advice['mode'],factor=advice['factor'],
        projection_failed=x is None,proposal_hash=None if x is None else array_hash(x),projection=diagnostic,
        accepted=False,reason='RequiresOrdinaryValidationAndCanonicalEquivalence',plan_steps=1)

def validated_warm_choice(incumbent,proposal,*,numeric,exact,certificate,admission,canonical_hash):
    """Fail-closed adapter: rollback is bitwise and certification is mandatory."""
    complete=all(k in certificate for k in ('complete','candidate_pairs','completed_pairs','forbidden','unresolved','invalid_surfaces'))
    complete=complete and certificate['complete'] and certificate['candidate_pairs']==certificate['completed_pairs'] and not any(certificate[k] for k in ('forbidden','unresolved','invalid_surfaces'))
    accepted=proposal is not None and numeric and exact and complete and admission and array_hash(proposal)==canonical_hash
    return np.asarray(proposal if accepted else incumbent).copy(),bool(accepted)
