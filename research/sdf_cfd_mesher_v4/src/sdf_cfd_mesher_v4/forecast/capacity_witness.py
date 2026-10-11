"""Fail-closed distinction between capacity evidence and valid-prefix evidence.

This module cannot commit geometry. Ownership preflight mirrors the qualified
rolling metric adapter; it does not invent a boundary-face metric convention.
"""
from .capacity import schedule_demand


def ownership_preflight(topology):
    missing=[]
    for edge in range(len(topology.edges)):
        owners=list(map(int,topology.edge_triangles.row(edge)))
        if len(owners)!=2:
            missing.append(dict(edge=edge,vertices=topology.edges[edge].tolist(),owners=owners,
                                required_owner_count=2,actual_owner_count=len(owners)))
    return dict(complete=not missing,missing=missing,
                reason=None if not missing else 'IncompleteSurfaceOwnership',
                scope='Necessary production face-metric dependency check; not full topology qualification')


def blocked_prefix(region,schedule,capacity,ownership):
    if ownership['complete']:raise ValueError('NotOwnershipBlocked')
    return dict(region_id=region,requested_final_layer=len(schedule.targets),committed_fixture_layers=0,
                last_committed_layer=0,last_fully_valid_layer=None,first_invalid_layer=None,
                first_attempted_layer=1,status='ValidPrefixAssessmentBlocked',
                blocker=ownership,capacity_prediction=capacity,
                layer_zero_demand=schedule_demand(schedule,0),
                warning_first_observed_layer=0 if capacity['screen']['active'] else None,
                predicted_limit_first_observed_layer=0 if capacity['status']=='CapacityLimitPredicted' else None,
                hard_failure_lead_time=None,termination_authority=False,
                mechanism_classification='UnresolvedLimitMechanism')


def classify_boundary(*,geometric_first,hard_first,full_gate_coverage,steering_validated=False,completed=False):
    if not full_gate_coverage:return 'UnresolvedLimitMechanism'
    if completed:return 'SteerableBeforeCapacity' if steering_validated else 'CapacityClear'
    if hard_first is None:return 'UnresolvedLimitMechanism'
    if geometric_first is not None and hard_first==geometric_first:return 'GeometricLimitControlsFailure'
    if geometric_first is None or hard_first<geometric_first:return 'QualityGateControlsFailure'
    return 'UnresolvedLimitMechanism'
