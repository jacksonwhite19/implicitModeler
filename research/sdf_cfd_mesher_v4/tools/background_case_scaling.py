"""Similarity scaling candidate. g is dh/dd, dimensionless, despite wire name."""
import copy,math
from standalone_background import validate_policy
def scale_background_policy(policy,factor):
    validate_policy(policy)
    if type(factor) not in (int,float) or not math.isfinite(factor) or factor<=0:
        raise ValueError('PositiveFiniteSimilarityScaleRequired')
    result=copy.deepcopy(policy)
    result['bounds_m']=[[v*factor for v in row] for row in policy['bounds_m']]
    result['maximum_size_m']=policy['maximum_size_m']*factor
    # h(a*x) = a*min(hmax, s + g*d(x)): scale lengths, retain g.
    validate_policy(result)
    return result
