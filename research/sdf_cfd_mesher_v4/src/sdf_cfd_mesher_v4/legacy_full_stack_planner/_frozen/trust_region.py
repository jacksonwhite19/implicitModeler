"""Fixed two-way globalization of the existing lexicographic finite search."""
import numpy as np

def stage_assessment(current, modeled, finite, baseline, z, candidate, radii,
                     columns, targets, groups, step, radius, tau):
    """Assess only the first controlling modeled stage; never weight stages."""
    mask=np.isin(columns,targets)
    order=sorted(np.flatnonzero(mask),key=lambda i:(float(baseline.reshape(-1,20)[i].min()),int(columns[i])))
    def values(s,q):
        minima=s.reshape(-1,20).min(1)
        return np.r_[minima[mask].min(),minima[order],
                     -max(0.,float(np.max(baseline-s))),-float(np.sum((q/radii)**2))]
    old=values(current,z);pred=values(modeled,candidate);actual=values(finite,candidate)
    names=['common_target_minimum']+['ordered_target_'+str(int(columns[i]))for i in order]+['remaining_weak_prism_loss','direction_displacement']
    changed=np.flatnonzero(np.abs(pred-old)>tau)
    index=int(changed[0])if len(changed)else None
    pg=float(pred[index]-old[index])if index is not None else 0.
    ag=float(actual[index]-old[index])if index is not None else 0.
    meaningful=index is not None and pg>tau
    usage=0.
    for group in groups:
        for j in group:
            bound=radius*radii[j]/np.sqrt(len(group))
            usage=max(usage,float(abs(step[j])/bound))
    return dict(stage=names[index]if index is not None else 'ArithmeticZeroModel',
                stage_index=index,modeled_gain=pg,actual_gain=ag,
                ratio=ag/pg if meaningful else None,meaningful_prediction=meaningful,
                radius_used=float(radius),trust_bound_usage=usage,trust_boundary_active=usage>=.8)

def next_radius(radius,accepted,assessment):
    if not accepted:return radius*.5,'shrink','RejectedPrefixFamily'
    ratio=assessment['ratio']
    if ratio is None:return radius,'hold','NonpositiveOrArithmeticZeroPrediction'
    if ratio<.25:return radius*.5,'shrink','PoorStageAgreement'
    if ratio>=.75 and assessment['trust_boundary_active']:
        new=min(2*radius,1.)
        return new,'grow'if new>radius else'hold','StrongBoundaryStep'if new>radius else'MaximumRadius'
    return radius,'hold','StageAgreementWithoutGrowth'
