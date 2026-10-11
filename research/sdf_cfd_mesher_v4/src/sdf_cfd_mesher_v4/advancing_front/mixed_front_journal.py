"""Atomic mixed-front publication and durable topology transaction journal.

Only CURRENT publishes state. Uncommitted journal stages are diagnostic and
are replayed from the previous committed snapshot, never exposed as geometry.
The caller supplies the existing scientific validators; this module has no
geometric acceptance tolerances or remesh trigger rules.
"""
from copy import deepcopy
from pathlib import Path
from ..common.authority import identity
from .rolling_state import atomic_json
from .mixed_front import VERSION, scientific_hash

STAGES = ('Prepared', 'RollbackApplied', 'RemeshConstructed', 'NumericallyValidated',
          'CertificateGenerated', 'StateRebuilt', 'Committed')


class MixedFrontJournal:
    def __init__(self, path):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=True)

    def initialize(self, state):
        if (self.path/'CURRENT.json').exists():
            current = self.restart()
            if current['wall'] != state['wall'] or current['authority'] != state['authority']:
                raise ValueError('MixedFrontAuthorityMismatch')
            return current
        self.publish(state)
        return self.restart()

    def publish(self, state):
        if state['version'] != VERSION:
            raise ValueError('MixedFrontSchemaMismatch')
        from .native_checkpoint import NativeCheckpointStore
        return NativeCheckpointStore(self.path).publish(state)

    def restart(self):
        from .native_checkpoint import NativeCheckpointStore
        return NativeCheckpointStore(self.path).restart()

    def record_stage(self,event,log):
        atomic_json(self.path/('JOURNAL_'+event+'.json'),log)

    def transact(self, evidence, cavity, construct, validate, rebuild, interrupt=None,
                 *, numerical_validate=None):
        """Explicit stages; validate must reject before any publication.

        evidence carries the pre-state identity. Cavity contains source-keyed
        removed cells and preserved boundary identity from qualified discovery.
        Restarting an already committed event never invokes callbacks again.
        """
        before = self.restart()
        event = identity(dict(evidence=evidence, cavity=cavity))
        if event in before['events']:
            return deepcopy(before['events'][event])
        if evidence['base'] != scientific_hash(before):
            raise ValueError('StaleMixedFrontTransaction')
        log = dict(event=event, base=evidence['base'], evidence=evidence, cavity=cavity, stages=[])

        def stage(name):
            log['stages'].append(name)
            self.record_stage(event,log)
            if interrupt:
                interrupt(name)

        stage('Prepared')
        private = {k:(v.transaction_copy() if hasattr(v,'transaction_copy') else deepcopy(v)) for k,v in before.items()}
        removed = cavity['removed_cells']
        if len(removed) != len(set(removed)) or any(k not in private['cells'] for k in removed):
            raise ValueError('InvalidRemovedCellManifest')
        if any(private['cells'][k]['layer'] == 1 for k in removed):
            raise ValueError('FirstLayerRollbackNotQualified')
        for key in removed:
            del private['cells'][key]
        stage('RollbackApplied')
        replacement = construct(private)
        stage('RemeshConstructed')
        if replacement['wall'] != before['wall'] or replacement['first_layer'] != before['first_layer']:
            raise ValueError('WallFirstLayerMutation')
        for key, cell in before['cells'].items():
            if key not in removed and (replacement['cells'].get(key) != cell or any(
                    replacement['vertices'].get(v) != before['vertices'][v] for v in cell['vertices'])):
                raise ValueError('OutsideCavityMutation')
        if numerical_validate is not None:
            numeric = numerical_validate(replacement)
            if numeric.get('state') != scientific_hash(replacement) or numeric.get('passed') is not True:
                raise ValueError('UnboundMixedFrontNumericalValidation')
            log['numerical_receipt'] = deepcopy(numeric)
            stage('NumericallyValidated')
        receipt = validate(replacement)
        if receipt.get('state') != scientific_hash(replacement) or receipt.get('passed') is not True:
            raise ValueError('UnboundMixedFrontValidation')
        # Combined historical callbacks remain readable, but cannot claim a
        # separate numerical/certificate failure boundary.
        stage('CertificateGenerated' if numerical_validate is not None else 'RemeshValidated')
        geometry_before = {k: deepcopy(replacement[k]) for k in ('wall', 'first_layer', 'vertices', 'cells', 'faces', 'columns', 'outer', 'prism_front')}
        replacement = rebuild(replacement, receipt)
        if any(replacement[k] != v for k, v in geometry_before.items()):
            raise ValueError('StateRebuildMutatedValidatedGeometry')
        stage('StateRebuilt')
        if scientific_hash(self.restart()) != evidence['base']:
            raise ValueError('ConcurrentMixedFrontWriter')
        scientific_receipt={k:receipt[k] for k in ('state','passed','proof','checks') if k in receipt}
        record = dict(event=event, evidence=identity(evidence), cavity=identity(cavity),
                      cavity_manifest=deepcopy(cavity), validation=scientific_receipt)
        replacement['events'][event] = record
        if interrupt:
            interrupt('BeforeCommit')
        self.publish(replacement)
        stage('Committed')
        return deepcopy(record)


class InMemoryMixedFrontJournal(MixedFrontJournal):
    """Same transaction predicates, without durable stage or checkpoint writes."""
    def __init__(self,state):
        self.current=deepcopy(state)

    def restart(self):
        return deepcopy(self.current)

    def publish(self,state):
        if state['version']!=VERSION:raise ValueError('MixedFrontSchemaMismatch')
        value=deepcopy(state)
        digest=scientific_hash(value)
        self.current=value
        return digest

    def record_stage(self,event,log):
        pass
