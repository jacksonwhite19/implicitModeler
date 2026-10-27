"""Evidence-bound zero-risk admission to attempt generation, never publication.

The existing production selector makes detailed work conditional on events.
This adapter closes only the sealed zero-risk branch. Nonzero risk remains
blocked for its required detailed work; no metric or final commit gate changes.
"""
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.production_risk import select_supports

REQUIRED=('restored_authority','request_schedule','cheap_forecast','direct_population',
          'direct_metrics','topology_ownership','collision_inventory','collision_proofs','risk_supports')
CHEAP_REQUIRED={'nonfinite_coordinate','sdf_evaluation_failure','zero_required_gradient','projection_failure',
                'zero_area_triangle','orientation_reversal','invalid_topology_reference'}


def evaluate(evidence,seal,binding,coordinates,top,candidate,*,schedule=None,trajectory=None):
    if identity(evidence)!=seal:raise ValueError('EvidenceIdentityMismatch')
    if evidence['binding']!=binding:raise ValueError('AuthorityBindingMismatch')
    typed=trajectory is not None
    if typed:
        from accepted_production_contract import ProductionTrajectory
        if type(trajectory) is not ProductionTrajectory:raise ValueError('QualifiedTrajectoryContractRequired')
        trajectory.check()
        if (schedule is not trajectory.schedule or binding['committed']!=trajectory.parent or
            binding['trajectory_owner']!=trajectory.source_identity or binding['authorization']!=trajectory.authorization_identity or
            binding['schedule']!=schedule.sha256 or binding['continuation']!=schedule.source_sha256 or
            binding['layer']!=trajectory.layer or binding['next_layer']!=trajectory.layer+1):raise ValueError('ScheduleAuthorityMismatch')
        if evidence.get('increment_record')!=schedule.row_record(binding['next_layer']):raise ValueError('ScheduleMismatch')
        if evidence.get('trajectory_premises')!=trajectory.trajectory_premises() or not evidence['trajectory_premises']['passed']:
            raise ValueError('MissingTrajectoryPremises')
        trajectory.validate_target(candidate,schedule.targets[trajectory.layer])
    elif binding.get('version')=='V4AcceptedFrontProductionBinding/1':raise ValueError('QualifiedTrajectoryContractRequired')
    elif schedule is None:
        # Preserve the sealed legacy entry point; later layers need an exact
        # externally bound schedule, not a guessed increment.
        if binding['layer']!=1 or binding['next_layer']!=2:raise ValueError('Layer2Only')
        expected_increment='0x1.6b844d56c388ep-13'
    else:
        layer=binding['layer']
        if not 1<=layer<len(schedule.increments) or binding['next_layer']!=layer+1:raise ValueError('NextLayerMismatch')
        if schedule.sha256!=binding['schedule'] or schedule.source_sha256!=binding['request_file']:raise ValueError('ScheduleAuthorityMismatch')
        expected_increment=float(schedule.increments[layer]).hex()
    if not typed and evidence['increment_hex']!=expected_increment:raise ValueError('ScheduleMismatch')
    next_layer=binding['next_layer']
    if 'forecast_scope' in evidence:
        if schedule is None or evidence['forecast_scope']!=dict(kind='NextTransitionOnly/1',layers=[next_layer],schedule=schedule.sha256):
            raise ValueError('ForecastScopeMismatch')
    scope=({'forecast_scope':dict(evidence['forecast_scope'],future_layers='NotEvaluated')}
           if 'forecast_scope' in evidence else {})
    checks=evidence['completeness']
    if set(checks)!=set(REQUIRED) or not all(checks[k] is True for k in REQUIRED):raise ValueError('IncompleteEvidence')
    reasons=[]
    if set(evidence['cheap_hard_events'])!=(CHEAP_REQUIRED|({'invalid_declared_target'} if typed else set())):raise ValueError('MissingCheapEventCoverage')
    if any(type(v) is not int or v<0 for v in evidence['cheap_hard_events'].values()):raise ValueError('InvalidCheapEventCount')
    if any(evidence['cheap_hard_events'].values()):reasons.append('CheapHardEvent')
    if evidence['direct_hard_events']:reasons.append('DirectHardEvent')
    accounting=evidence['collision_accounting']
    if set(accounting)!={'candidates','certified','unresolved','forbidden','failure','missing','duplicate'} or any(type(v) is not int or v<0 for v in accounting.values()):
        raise ValueError('IncompleteCollisionAccounting')
    if accounting['candidates']!=accounting['certified']+accounting['unresolved']+accounting['forbidden']+accounting['failure']:
        raise ValueError('CollisionAccountingMismatch')
    if any(accounting[k] for k in ('unresolved','forbidden','failure','missing','duplicate')):reasons.append('CollisionEvidenceNotClear')
    if evidence['risk_seeds'] or evidence['patches']:reasons.append('DetailedRiskWorkRequired')
    if evidence['mandatory_detail_requirements']:reasons.append('MandatoryDetailedEvidenceRequired')
    if reasons:
        return dict(status=f'ProductionLayer{next_layer}AdvanceBlocked',generation_attempt_authorized=False,publication_authorized=False,reasons=reasons,**scope)
    # No known future hard warning is promoted from a historical fixture cohort.
    # This is not a claim of complete direct future-layer hard-metric coverage.
    patches,traces=select_supports(coordinates,top,candidate,[],[],next_layer,retained=())
    if patches or traces:raise ValueError('ZeroRiskSelectorSemanticsChanged')
    noop=dict(status='ConditionalDetailedNoOp',patches=0,detailed_work_count=0,
              mandatory_independent_requirements=[],future_direct_metrics='NotMeasured; optional guidance, not unconditional admission requirement')
    return dict(status=f'ProductionLayer{next_layer}AdvanceAuthorized',generation_attempt_authorized=True,publication_authorized=False,**scope,
        next_layer=next_layer,**({'increment_record':evidence['increment_record']} if typed else {'increment_hex':evidence['increment_hex']}),binding_identity=identity(binding),evidence_identity=seal,
        completeness_identity=identity(checks),zero_risk_identity=identity({k:evidence[k] for k in ('cheap_hard_events','direct_hard_events','collision_accounting','risk_seeds','patches')}),
        detailed_noop=noop,detailed_noop_identity=identity(noop),
        reason='Complete bound immediate screening and collision evidence is clear; existing event-driven selector requires no detailed patch.',
        final_geometry_certification_required=True,transactional_commit_required=True)
