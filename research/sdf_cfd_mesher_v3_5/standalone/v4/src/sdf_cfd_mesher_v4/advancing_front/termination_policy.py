"""Evidence-driven local escalation. No geometry mutation or runner integration.

Inputs are compact receipts admitted by current geometry/certificate adapters.
The policy never interprets a numerical quality surrogate. A context binds the
base geometry, topology, schedule and requested layer. Every receipt carries it.
"""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import json
from ..common.authority import identity
from .rolling_state import atomic_json

VERSION = 'V4TerminationTrigger/1'
STAGES = ('TangentialCorrection', 'CompressionCorrection', 'GrowthRedistribution',
          'BranchAwareCorrection', 'ContinuationAttempt')
REQUIRED_DIRECT = frozenset(('PositiveVolumeOverlap', 'TargetLevelSetAbsent',
                            'ExactZeroThickness', 'QualifiedCapacityLimit', 'FoldWithNegativeVolume'))
IMMEDIATE_CHECKS = ('hard_quality', 'exact_validity', 'ownership', 'continuous', 'sdf_target')
REMESH_CHECKS = IMMEDIATE_CHECKS + ('volume_conformity', 'outer_interface', 'core_compatibility',
                                  'neighbor_continuation', 'wall_first_layer', 'outside_preserved')


def _receipt(receipt, context):
    if receipt.get('context') != context:
        raise ValueError('StaleEvidenceContext')
    if not isinstance(receipt.get('proof'), str) or len(receipt['proof']) != 64:
        raise ValueError('UnboundEvidence')


def _valid(receipt, checks):
    if receipt is not None and tuple(checks)==REMESH_CHECKS:
        from terminal_capacity import remesh_admitted
        return remesh_admitted(receipt)
    return receipt is not None and all(receipt.get('checks', {}).get(k) is True for k in checks)


def cavity_seed(evidence):
    """Union actual source entities, not historical fixture IDs or guessed rings."""
    result = {key: set() for key in ('vertices', 'columns', 'cells', 'faces')}
    for item in [evidence['immediate']] + evidence.get('witnesses', []):
        if item.get('scope') == 'future':
            continue
        for key in result:
            values = item.get('seed', {}).get(key, [])
            if any(type(v) is not int or v < 0 for v in values):
                raise ValueError('InvalidSeedEntity')
            result[key].update(values)
    return {k: sorted(v) for k, v in result.items()}


def evaluate(evidence):
    """Replay the local state machine from its complete current evidence ledger.

    New correction receipts extend the ledger. Re-evaluation does not inherit a
    stale terminal decision; restarting the same ledger gives the same trace.
    Trial counts, height ratios, debt and conditioning are diagnostic data only.
    """
    e = deepcopy(evidence);ctx = e['context']
    if ctx.get('layer', 0) < 1 or not all(ctx.get(k) for k in ('region', 'base', 'topology', 'schedule')):
        raise ValueError('IncompleteLocalBinding')
    for item in [e['support'], e['immediate']] + e.get('witnesses', []) + e.get('attempts', []):
        _receipt(item, ctx)
    if e.get('accepted') is not None:
        _receipt(e['accepted'], ctx)
    immediate = e['immediate']['status']
    if immediate not in ('unknown', 'invalid', 'valid'):
        raise ValueError('UnknownImmediateStatus')
    attempts = e.get('attempts', [])
    if len({a['stage'] for a in attempts}) != len(attempts):
        raise ValueError('DuplicateCorrectionStage')
    for a in attempts:
        if a['stage'] not in STAGES or a['status'] not in ('failed', 'succeeded', 'inapplicable'):
            raise ValueError('UnknownCorrectionReceipt')
        if a['status'] == 'inapplicable' and not a.get('reason'):
            raise ValueError('UnjustifiedStageSkip')
    trace = [];eid = identity(e)
    def emit(state, reason, proofs=()):
        trace.append(dict(state=state, reason=reason, evidence=eid, proofs=sorted(set(proofs))))
    def finish(disposition, next_action=None, seed=None):
        return dict(context=ctx, evidence=eid, state=trace[-1]['state'], disposition=disposition,
                    next_action=next_action, seed=seed, trace=trace)
    emit('PassiveNominal', 'Current local evidence ledger')
    if e['support']['status'] != 'clean':
        emit('Blocked', 'Repair or expand contaminated/unknown support before remeshing', [e['support']['proof']])
        return finish('RemeshBlocked', 'RepairSupport')
    if e.get('warnings'):
        emit('ActiveWarning', 'Forecast is diagnostic until immediate evidence justifies escalation', e['warnings'])
    direct = [w for w in e.get('witnesses', []) if w.get('qualified') is True and
              w.get('scope') in ('current_stack', 'immediate') and
              (w['kind'] in REQUIRED_DIRECT or w['kind'] == 'ExactCollision')]
    stack_bad = [w for w in direct if w.get('scope') == 'current_stack']
    # Newly discovered current-stack overlap invalidates an older acceptance.
    # A prospective failed trial, however, does not override a valid correction.
    accepted = e.get('accepted')
    if not stack_bad and _valid(accepted, IMMEDIATE_CHECKS):
        stage = accepted.get('stage', 'NominalAdvance')
        if stage not in STAGES and stage != 'NominalAdvance':
            raise ValueError('UnknownAcceptedStage')
        for name in STAGES:
            if name == stage:
                emit(name, 'Fully validated correction', [accepted['proof']]);break
            a = next((a for a in attempts if a['stage'] == name), None)
            if a is not None:
                emit(name, a['status'], [a['proof']])
        emit('Resolved', 'Current candidate passes every immediate acceptance check', [accepted['proof']])
        return finish('NoRemesh')
    if immediate != 'invalid' and not stack_bad:
        if immediate == 'valid':
            emit('Blocked', 'Validity claim lacks complete admission receipt', [e['immediate']['proof']])
            return finish('WarningOnly', 'ValidateImmediate')
        if trace[-1]['state'] == 'PassiveNominal':
            emit('ActiveWarning', 'Immediate validation still required')
        return finish('WarningOnly', 'ValidateImmediate')
    reason = None;disposition = None;proofs = []
    if direct:
        required = [w for w in direct if w['kind'] in REQUIRED_DIRECT]
        disposition = 'RemeshRequiredByDirectGeometry' if required else 'RemeshEligible'
        reason = ('Direct geometry makes the current construction inappropriate; not an optimizer infeasibility claim'
                  if required else 'Exact collision and unacceptable immediate geometry justify a bounded remesh attempt')
        proofs = [w['proof'] for w in direct]
    else:
        for stage in STAGES:
            a = next((a for a in attempts if a['stage'] == stage), None)
            emit(stage, 'Next simplest uncompleted correction' if a is None else a['status'], [] if a is None else [a['proof']])
            if a is None:
                return finish('WarningOnly', stage)
            if a['status'] == 'succeeded':
                # A numerical/projection success is not full geometry acceptance.
                emit('Blocked', 'Successful stage lacks complete immediate acceptance receipt', [a['proof']])
                return finish('WarningOnly', 'ValidateImmediate')
        disposition = 'RemeshEligible';reason = 'Every applicable practical correction stage has an invalid continuation receipt'
        proofs = [a['proof'] for a in attempts]
    seed = cavity_seed(e)
    if not seed['vertices']:
        emit('Blocked', 'Direct source-vertex mapping required before cavity discovery', proofs)
        return finish('RemeshBlocked', 'MapEvidenceToSupport')
    emit('RemeshCandidate', reason, proofs)
    return finish(disposition, 'RollbackRemesh', seed)


@dataclass
class LocalTerminationPolicy:
    evidence: dict
    decision: dict
    invocation: dict | None = None

    @classmethod
    def start(cls, evidence):
        return cls(deepcopy(evidence), evaluate(evidence))

    def observe(self, evidence):
        if evidence['context'] != self.evidence['context']:
            raise ValueError('NewLocalStateRequiresNewPolicy')
        if self.invocation is not None:
            raise ValueError('CannotChangeEvidenceDuringRemeshTransaction')
        self.evidence = deepcopy(evidence);self.decision = evaluate(evidence)
        return self.decision

    def invoke(self, current_context, backend, checkpoint=None):
        if current_context != self.evidence['context']:
            raise ValueError('StaleRemeshRequest')
        if self.invocation and self.invocation.get('receipt') is not None:
            return self.decision
        if self.decision['next_action'] != 'RollbackRemesh':
            raise ValueError('RemeshNotEligible')
        request = dict(context=current_context, seed=self.decision['seed'], evidence=self.decision['evidence'])
        request['request_id'] = identity(request)
        self.invocation = dict(request=request, receipt=None)
        if checkpoint is not None:
            self.save(checkpoint)  # Durable request ID; backend must be private/idempotent.
        try:
            receipt = backend(deepcopy(request))
            _receipt(receipt, current_context)
            if receipt.get('request_id') != request['request_id']:
                raise ValueError('WrongRemeshReceipt')
        except Exception as exc:
            receipt = dict(context=current_context, request_id=request['request_id'],
                           proof=identity(dict(error_type=type(exc).__module__+'.'+type(exc).__qualname__)),
                           checks={}, failure=type(exc).__module__+'.'+type(exc).__qualname__)
        self.invocation['receipt'] = receipt
        ok = _valid(receipt, REMESH_CHECKS)
        resolved = ('TerminalCapacityResolved' if ok and receipt.get('terminal_capacity') else 'Resolved')
        self.decision = deepcopy(self.decision)
        self.decision['trace'] += [dict(state='RollbackRemesh', reason='Invoke qualified bounded mechanics',
                                       evidence=self.decision['evidence'], proofs=[request['request_id']]),
                                  dict(state=resolved if ok else 'Blocked', reason='All remesh gates passed' if ok else 'Remesh failed; invalid prism continuation remains uncommitted',
                                       evidence=self.decision['evidence'], proofs=[receipt['proof']])]
        self.decision.update(state=resolved if ok else 'Blocked', next_action=None,
                             disposition='RemeshResolved' if ok else 'RemeshBlocked')
        if checkpoint is not None:
            self.save(checkpoint)
        return self.decision

    def save(self, path):
        data = dict(version=VERSION,evidence=self.evidence,decision=self.decision,invocation=self.invocation)
        atomic_json(Path(path), dict(payload=data,sha256=identity(data)))

    @classmethod
    def restart(cls, path, current_context):
        z=json.loads(Path(path).read_text());data=z['payload']
        if identity(data)!=z['sha256'] or data['version']!=VERSION:
            raise ValueError('CorruptPolicyCheckpoint')
        obj=cls.start(data['evidence'])
        if current_context!=obj.evidence['context']:
            raise ValueError('StalePolicyCheckpoint')
        if data['invocation']:
            saved=data['invocation'];expected=dict(context=current_context,seed=obj.decision['seed'],evidence=obj.decision['evidence'])
            expected['request_id']=identity(expected)
            if obj.decision['next_action']!='RollbackRemesh' or saved['request']!=expected:
                raise ValueError('CheckpointRequestMismatch')
            if saved['receipt'] is not None:
                obj.invoke(current_context,lambda request:saved['receipt'])
            else:
                obj.invocation=saved
        if obj.decision!=data['decision']:
            raise ValueError('CheckpointDecisionMismatch')
        return obj
