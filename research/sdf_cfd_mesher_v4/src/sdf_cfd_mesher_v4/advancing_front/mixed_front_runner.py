"""Rolling mixed-front orchestration with qualified geometry-provider contracts.

The provider owns SDF construction, ordinary correction, cavity discovery and
exact validation. The runner owns committed mixed state and atomic publication.
No fixture identifiers or geometry-specific trigger rules belong here.
"""
from copy import deepcopy
from .mixed_front import scientific_hash
from .mixed_front_journal import MixedFrontJournal
from .termination_policy import LocalTerminationPolicy, REMESH_CHECKS


def preserves_committed_cells(candidate,before):
    cells=candidate['cells'];vertices=candidate['vertices']
    if hasattr(cells,'preserves') and hasattr(vertices,'preserves'):
        return cells.preserves(before['cells']) and vertices.preserves(before['vertices'])
    return all(cells.get(k)==c and all(vertices.get(v)==before['vertices'][v] for v in c['vertices'])
               for k,c in before['cells'].items())


class MixedFrontRunner:
    def __init__(self, path, provider, profile='Qualification'):
        from .execution_profile import ExecutionProfile
        self.profile=ExecutionProfile(profile)
        self.diagnostics=[]
        self.store=MixedFrontJournal(path)
        self.provider=provider
        self.fast_session=None
        self.fast_conservative=False

    def initialize(self, state):
        validate=getattr(self.provider,'validate_initial_commit',None)
        if validate is not None:
            if (self.store.path/'CURRENT.json').exists():
                current=self.store.initialize(state)
                if validate(current) is not True:raise ValueError('InitialCheckpointValidationFailed')
                return current
            if validate(state) is not True:raise ValueError('InitialCandidateValidationFailed')
            from .native_checkpoint import NativeCheckpointStore
            NativeCheckpointStore(self.store.path).publish(state,validate=validate)
            pointer=self.store.path/'CURRENT.json'
            published=pointer.read_bytes()
            try:
                committed=self.store.restart()
                if validate(committed) is not True:
                    raise ValueError('InitialPostCommitValidationFailed')
                return committed
            except Exception:
                # This is fresh initialization: there was no prior published
                # state. Keep staged evidence, but revoke only our pointer.
                if pointer.exists() and pointer.read_bytes()==published:
                    pointer.unlink()
                raise
        return self.store.initialize(state)

    def step(self, interrupt=None):
        from .execution_profile import ExecutionProfile
        if self.profile is ExecutionProfile.ProductionFast:
            if self.fast_conservative:return self._step(interrupt)
            if self.fast_session is None:
                current=self.store.restart()
                # The optimized adapter shares AnalyticProvider's exact
                # ordinary construction. Other providers keep their own
                # scientific callbacks; never substitute an analytic builder.
                from run_geometry_family import AnalyticProvider
                if (type(self.provider) is not AnalyticProvider
                        or current['events'] or any(c['kind']!='prism' for c in current['cells'].values())
                        or set(current['prism_front'])!=set(current['columns'])
                        or len({c['last_prism_layer'] for c in current['columns'].values()})!=1):
                    from .mixed_front_journal import InMemoryMixedFrontJournal
                    self.store=InMemoryMixedFrontJournal(current);self.fast_conservative=True
                    return self._step(interrupt)
                from production_fast_layer import FastLayer
                self.fast_session=FastLayer(current,self.provider.spec,self.provider.folder)
            try:return self.fast_session.step()
            except ValueError as exc:
                message=str(exc)
                if not (message.startswith('RequiresQualified') or message.startswith('IncrementalHardQualityFailure')
                        or message in ('ChangedRoleDeterminantFailure','ProspectiveDeterminantFailure','ContinuousGeometryFailure','GlobalDegreeFailure')):
                    raise
                # Re-enter the SAME production policy/correction/remesh code.
                # No failed nominal candidate has been committed. The journal
                # changes persistence only; its scientific guards are shared.
                from .mixed_front_journal import InMemoryMixedFrontJournal
                self.store=InMemoryMixedFrontJournal(self.fast_session.binding_state())
                self.fast_session=None;self.fast_conservative=True
                return self._step(interrupt)
        if self.profile is ExecutionProfile.Diagnostic:
            import time
            tick=time.perf_counter()
            result=self._durable_step(interrupt)
            self.diagnostics.append(dict(seconds=time.perf_counter()-tick,operation=result['operation']))
            return result
        return self._durable_step(interrupt)

    def _durable_step(self, interrupt=None):
        # Block collection must not race an unpublished candidate's construction.
        # Hold the storage lock for the complete transaction, including remesh.
        import json
        from .canonical_blocks import AUTHORITY
        from .native_checkpoint import FORMAT
        pointer=json.loads((self.store.path/'CURRENT.json').read_text())
        if pointer.get('format')==FORMAT:
            from .block_retention import current,locked
            with locked(self.store.path):
                root=current(self.store.path)
                if root and root.get('authority')==AUTHORITY:return self._step(interrupt)
        return self._step(interrupt)
    def _step(self, interrupt=None):
        before=self.store.restart()
        from terminal_capacity import prevent_terminal_advance
        prevent_terminal_advance(before)
        proposal=self.provider.propose(deepcopy(before))
        evidence=self.provider.evidence(before,proposal)
        if evidence['context']['base']!=scientific_hash(before):
            raise ValueError('StaleRollingEvidence')
        policy=LocalTerminationPolicy.start(evidence)
        if policy.decision['next_action']=='RollbackRemesh':
            request=policy.decision
            cavity=self.provider.discover(before,proposal,request)
            def construct(private):
                return self.provider.remesh(before,private,proposal,cavity)
            def validate(state):
                receipt=self.provider.certify_validation(state,proposal,cavity)
                from terminal_capacity import remesh_admitted
                if not remesh_admitted(receipt):
                    raise ValueError('IncompleteRemeshAcceptance')
                return receipt
            def rebuild(state,receipt):
                return self.provider.rebuild(before,state,receipt)
            record=self.store.transact(dict(base=scientific_hash(before),policy=evidence),
                                       cavity,construct,validate,rebuild,interrupt,
                                       numerical_validate=lambda state:self.provider.numerical_validation(state,proposal,cavity))
            return dict(operation='RollbackRemesh',event=record,state=scientific_hash(self.store.restart()),
                        policy=policy.decision)
        # Ordinary correction remains a provider operation in the same state
        # model. No failed nominal candidate is silently published.
        candidate=self.provider.correct(before,proposal,policy.decision)
        receipt=self.provider.validate(candidate,proposal,None)
        if receipt.get('passed') is not True or receipt.get('state')!=scientific_hash(candidate):
            raise ValueError('OrdinaryMixedAdvanceRejected')
        geometry={k:deepcopy(candidate[k]) for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front')}
        candidate=self.provider.rebuild(before,candidate,receipt)
        if any(candidate[k]!=v for k,v in geometry.items()):
            raise ValueError('StateRebuildMutatedValidatedGeometry')
        first_committed=bool(before['first_layer']) or bool(before['cells'])
        if candidate['wall']!=before['wall'] or (first_committed and candidate['first_layer']!=before['first_layer']):
            raise ValueError('OrdinaryWallFirstLayerMutation')
        if not preserves_committed_cells(candidate,before):
            raise ValueError('OrdinaryAdvanceRequiresRollbackJournal')
        if scientific_hash(self.store.restart())!=scientific_hash(before):
            raise ValueError('ConcurrentMixedFrontWriter')
        # Publication already hashes the exact native evidence representation.
        # Reuse that verified identity instead of rewalking expanded proofs.
        committed=self.store.publish(candidate)
        return dict(operation='PrismCommit',state=committed,policy=policy.decision)
