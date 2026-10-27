"""Apply the existing Task-3 pair-work bound before exact cap traversal.

The frozen SideIndex.certify primitive has a time bound but no pair-count
argument. The original local_certificate.certify_all caller caps work at
250000 pairs. This adapter preserves that caller policy on expanded supports.
No pair is accepted using this broad-phase calculation.
"""
import numpy as np
from ..certificate.continuous_prism_certificate import kernel
from ..forecast.detailed.scientific import scientific_hash

def pair_population(index,core_edges,max_edges=8192):
    edges=index.candidates(core_edges,max_edges=max_edges);ep=index.topology.edges[edges]
    quads=[];labels=[];previous=index.front.coordinates;core=set(map(int,core_edges))
    for k,x in enumerate(index.forecast.vertex['coordinates'],1):
        quads.extend(np.stack((previous[ep[:,0]],previous[ep[:,1]],x[ep[:,0]],x[ep[:,1]]),axis=1))
        labels.extend((k,int(e)) for e in edges);previous=x
    lo,hi=kernel.enumerator.boxes(np.asarray(quads));pairs,_=kernel.enumerator.bvh_pairs(lo,hi)
    pairs=sorted((a,b) for a,b in pairs if labels[a][1] in core or labels[b][1] in core)
    return dict(candidate_edges=len(edges),core_edges=len(core),candidate_pairs=len(pairs),
        pair_identity_hash=scientific_hash([[labels[a],labels[b]] for a,b in pairs]))

def over_budget_record(population,max_pairs=250000):
    if population['candidate_pairs']<=max_pairs:return None
    return dict(classification='LocalContinuousEvidenceIncomplete',reason='ComputationalPairBound',
        **population,pair_bound=max_pairs,completed_pairs=0,exact_kernel_calls=0,
        forbidden_count=None,unresolved_count=None,
        interpretation='Unexamined scope. This is neither a collision witness nor an intersection-free conclusion.')

def certify_bounded(index,scope,session,*,max_pairs=250000,max_edges=8192):
    population=pair_population(index,scope.side_edges,max_edges=max_edges)
    blocked=over_budget_record(population,max_pairs)
    if blocked:return blocked
    r=index.certify(scope,session=session,max_edges=max_edges,seconds_budget=3600)
    r.pop('seconds',None)
    return r|dict(pair_bound=max_pairs,pair_identity_hash=population['pair_identity_hash'])
