"""Authoritative ordinary-prism production orchestration.

One installer selection and one stage sequence for every supported bound owner.
Qualification adds an independent realization around the SAME production stages.
Geometry mathematics, acceptance, indexed storage and transactions remain owned
by the existing qualified implementations. This module grants no authorization.
"""
from contextlib import ExitStack, contextmanager
from enum import Enum
from pathlib import Path
import os
import json
import shutil
import time

import sealed_front_connection as connection

PRODUCTION_CONTROLS = dict(V4_SINGLE_QUALITY='1', V4_INDEXED_ATLAS='1',
                          V4_VERIFIED_QUALITY='1', V4_GUARDED_PUBLICATION='1')


class ExecutionProfile(str, Enum):
    PRODUCTION = 'production'
    QUALIFICATION = 'qualification'


@contextmanager
def production_controls():
    prior = {name: os.environ.get(name) for name in PRODUCTION_CONTROLS}
    for name, value in PRODUCTION_CONTROLS.items():
        if prior[name] is not None and prior[name] != value:
            raise ValueError('ConflictingProductionProfileControl:' + name)
    os.environ.update(PRODUCTION_CONTROLS)
    try:
        yield
    finally:
        for name, value in prior.items():
            if value is None:os.environ.pop(name, None)
            else:os.environ[name] = value


@contextmanager
def qualified_dispatch(*, reuse_validated_assembly=True, production_step=True):
    """Single selection of the qualified capsule stack; legacy flags preserved."""
    if type(reuse_validated_assembly) is not bool:
        raise ValueError('InvalidValidatedAssemblyReuseMode')
    if type(production_step) is not bool:raise ValueError('InvalidProductionStepMode')
    with ExitStack() as held:
        if production_step:held.enter_context(production_controls())
        executors = held.enter_context(connection.qualified_dispatch())
        if reuse_validated_assembly:
            from assembly_quality_handoff import installed as quality_handoff
            from shared_schedule_hotpath import installed as shared_schedule
            from assembly_atlas_handoff import installed as atlas_handoff
            held.enter_context(quality_handoff(reuse_private_codec=True))
            held.enter_context(atlas_handoff())
            held.enter_context(shared_schedule())
        if production_step:
            from functools import wraps
            admit=executors['admission']
            @wraps(admit)
            def admission(*args,**kwargs):
                if 'forecast_scope' in kwargs:raise ValueError('ConflictingProductionForecastScope')
                return admit(*args,**kwargs,forecast_scope='next_transition')
            yield dict(executors,admission=admission)
        else:yield executors


@contextmanager
def initial_front_dispatch():
    """Cold-front entry to the SAME optimized predicates and production stack.

    Initial coverage has no inherited proofs. Native publication and independent
    readbacks remain required. Cache only a private completed incidence result
    for identical coordinates, cells, ordered interface and active-cell scope.
    Each caller still validates its own receipt, versions and restored state.
    """
    from copy import deepcopy
    from unittest.mock import patch
    import numpy as np
    import final_shell_incidence as incidence
    import first_layer_metric_state as metric
    import native_mixed_face_certificate as certificate
    import optimized_mixed_certificate as optimized
    import cavity_volume_audit as volume
    import prism_degree
    import local_termination_topology as topology
    from prism_hotpath_v2 import batch_quality
    from sdf_cfd_mesher_v4.common.authority import array_hash, identity
    original = incidence.check
    original_quality = topology.quality
    saved = None
    counts = dict(incidence_evaluations=0, incidence_reuses=0)

    def cold_quality(points, cells, owners, base_layer, *, active_cells=None):
        # No scientific cache crosses calls. Output geometry is fresh and local.
        # The defining module's bindings distinguish an earlier custom owner,
        # including one installed before this dispatcher was imported.
        coordinates = points
        native = (original_quality is topology._STANDARD_QUALITY and
                  topology.geometry is topology._STANDARD_GEOMETRY and
                  type(points) is np.ndarray and coordinates.dtype == np.dtype(np.float64) and
                  coordinates.ndim == 2 and coordinates.shape[1:] == (3,) and
                  type(cells) is list and bool(cells) and
                  (active_cells is None or type(active_cells) in (set, frozenset)) and
                  all(type(c) is dict and type(c.get('id')) is int and
                      c.get('kind') == 'prism' and len(c.get('faces', ())) == 5 and
                      all(len(f) in (3,4) for f in c['faces']) for c in cells) and
                  len({c['id'] for c in cells}) == len(cells) and
                  all(tuple(map(len,c['faces'])) == tuple(map(len,cells[0]['faces']))
                      for c in cells))
        if native:
            return batch_quality(points,cells,owners,base_layer,
                                 active_cells=active_cells,_geometry_out={})
        return original_quality(points,cells,owners,base_layer,active_cells=active_cells)

    def checked(points, cells, outer, *, active_cells=None):
        nonlocal saved
        coordinates = np.asarray(points)
        if (not metric.eligible(cells) or coordinates.dtype != np.dtype(np.float64)
                or coordinates.ndim != 2 or coordinates.shape[1:] != (3,)):
            return original(points, cells, outer, active_cells=active_cells)
        key = (coordinates.dtype.str, coordinates.shape, array_hash(coordinates),
               identity(cells), identity(outer),
               None if active_cells is None else identity(sorted(active_cells)))
        if saved is not None and saved[0] == key:
            counts['incidence_reuses'] += 1
            return deepcopy(saved[1])
        result = original(points, cells, outer, active_cells=active_cells)
        counts['incidence_evaluations'] += 1
        # Failed calculations are not reuse credentials. Retain only one result.
        saved = (key, deepcopy(result)) if result['passed'] else None
        return result

    with qualified_dispatch(), patch.object(certificate, 'certify', optimized.certify), \
            patch.object(volume, 'degree', prism_degree.degree), \
            patch.object(topology, 'quality', cold_quality), \
            patch.object(incidence, 'check', checked):
        yield counts


def close_preparation_jobs(folder):
    """Release only completed process-local jobs/guards; no file deletion."""
    from critical_index_pipeline import JOBS
    from prism_receipt_witness import _WITNESSES
    job=JOBS.pop(str(folder),None)
    if job:job.close()
    witness=_WITNESSES.pop(str(folder),None)
    if witness:witness['held'].close()
    from assembly_quality_handoff import release_directory
    release_directory(folder)


def publish_checked(base,adapter,folder,pairs,publisher,check):
    """An additional comparison may reject, never replace publisher acceptance."""
    from copy import deepcopy
    from unittest.mock import patch
    import composable_publication as publication
    original=publication.prepare;matched=[]
    def checked(parent,owner,out):
        if parent is not base or owner is not adapter or out!=folder or matched:
            raise ValueError('WrongReplayedPublicationOwner')
        primary=original(parent,owner,out)
        check(deepcopy(primary))  # Detached diagnostic view; exception stops BEFORE commit.
        matched.append(primary['scientific']);return primary
    with patch.object(publication,'prepare',checked):reference=publisher(base,adapter,folder,pairs)
    if len(matched)!=1:raise ValueError('MissingReplayedPublicationComparison')
    return reference,matched[0]


def publish_replayed(base,adapter,folder,pairs,replay_plan,publisher,*,extra_check=None):
    from sdf_cfd_mesher_v4.common.authority import identity
    from path_preserving_deduplication import durable_json
    def check(primary):
        comparisons={k:replay_plan['components'].get(k)==v for k,v in primary['components'].items()}
        passed=(set(primary['components'])==set(replay_plan['components'])
            and all(comparisons.values()) and primary['content']==replay_plan['content']
            and identity(primary['content'])==primary['scientific']==replay_plan['scientific']
            and identity(replay_plan['content'])==replay_plan['scientific'])
        durable_json(folder/'REPLAY.json',dict(passed=passed,comparisons=comparisons,
            content=primary['scientific'],replay_content=replay_plan['scientific']))
        if not passed:raise ValueError('DeterministicScientificComponentMismatch')
        if extra_check is not None:extra_check(primary)
    return publish_checked(base,adapter,folder,pairs,publisher,check)


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def execution_scope(authorization, selected_layers=None):
    """Select an existing authorized contiguous subset; never extend its scope."""
    layers=authorization['authorized_layers'] if selected_layers is None else selected_layers
    if (not isinstance(layers,list) or not layers or any(type(k) is not int for k in layers)
        or layers!=list(range(layers[0],layers[-1]+1))
        or any(k not in authorization['authorized_layers'] for k in layers)):
        raise ValueError('OutsideAuthorizedExecutionScope')
    return layers


def continuation_scope(control,authorization,authorization_identity):
    """Select a validated successor or resume; never create authorization.

    Resumed parents are authenticated again by CommittedState and the original
    ProductionTrajectory ancestry validator before forecast or generation.
    """
    from sdf_cfd_mesher_v4.common.authority import identity
    layers=execution_scope(authorization,control['layers'])
    if (control.get('schema')!='V4AuthorizedContinuationExecution/1'
        or authorization.get('schema') not in ('V4ContinuationAuthorizationSuccessor/1','V4CurrentFrontContinuationAuthorization/1','V4WingContinuationUserAuthorization/1')
        or identity(authorization)!=authorization_identity
        or control['authorization']!=authorization_identity
        or control['authorization_record']!=authorization):
        raise ValueError('InvalidSuccessorExecutionControl')
    resumed=control.get('resume',False)
    if type(resumed) is not bool:raise ValueError('InvalidSuccessorResumeMode')
    parent=control['parent']
    if authorization.get('schema')=='V4WingContinuationUserAuthorization/1':
        if (resumed or not control.get('fresh_inputs') or parent.get('passed') is not True
            or parent.get('layer')!=1 or parent.get('scientific')!=authorization['initial_committed_parent']
            or parent.get('reference',{}).get('kind')!='native' or layers!=authorization['authorized_layers']):
            raise ValueError('InvalidFreshInitialExecutionControl')
        return layers
    if resumed:
        if (parent.get('passed') is not True or parent.get('layer')!=layers[0]-1
            or parent['reference'].get('kind')!='recursive'
            or not isinstance(parent.get('scientific'),str) or len(parent['scientific'])!=64):
            raise ValueError('InvalidResumedCommittedParent')
    elif parent!=authorization['approved_parent'] or layers!=authorization['authorized_layers']:
        raise ValueError('InvalidSuccessorExecutionControl')
    return layers


def execution_resources(original,policy=None):
    """Explicit monitored-only policy; preserve all other operational limits."""
    if policy is None:return dict(original)
    if policy!=dict(total_storage_policy='monitor_without_output_cap',
                    output_limit_bytes=None,minimum_free_bytes=original['minimum_free_bytes']):
        raise ValueError('UnsupportedContinuationResourcePolicy')
    return dict(original,output_limit_bytes=None,total_storage_policy=policy['total_storage_policy'],
                inherited_output_limit_bytes=original['output_limit_bytes'])


def recovered_execution_access(base,control,authorization):
    """Reuse the existing strict resolver on the actual recovered ancestor."""
    from contextlib import nullcontext
    spec=control.get('recovered_access') if control is not None else None
    if spec is None:return nullcontext(None)
    node=base
    while node.reference!=spec['reference']:
        if node.kind!='recursive':raise ValueError('MissingRecoveredExecutionAncestor')
        node=node.parent
    from recovered_parent_authorization import recovered_authorization
    if authorization.get('schema')=='V4CurrentFrontContinuationAuthorization/1':
        return current_recovered_access(node,spec,authorization)
    return recovered_authorization(node,Path(spec['directory']),authorization,spec['identity'])


def current_recovered_access(base,spec,authorization):
    """Verify existing strict recovery, without resolving obsolete grant anchors."""
    from contextlib import contextmanager,ExitStack
    from recovered_parent_authorization import RecoveryAnchors
    from composable_state import CommittedState
    @contextmanager
    def held_access():
        if type(base) is not CommittedState:raise ValueError('QualifiedCommittedStateRequired')
        with ExitStack() as held:
            proof=RecoveryAnchors(base,Path(spec['directory']),authorization,spec['identity'],held)
            try:
                yield dict(proof.evidence,current_front_authorization=True)
                proof.check()
            finally:proof.closed=True
    return held_access()


def committed_index_session(base,adapter,control):
    """Select existing renewed indices explicitly; never mutate sealed plans."""
    from contextlib import contextmanager,ExitStack
    from prism_persistent_hotpath import bind_committed_geometry,geometry_session
    from incremental_pair_storage import PairIncidence
    import immutable_state_access as access
    from sdf_cfd_mesher_v4.common.authority import identity,file_hash
    @contextmanager
    def selected():
        with ExitStack() as held:
            spec=control.get('current_indices') if control is not None else None
            prior=getattr(base,'numeric_incidence',None)
            try:
                if spec is not None and base.reference==control['parent']['reference']:
                    path=Path(spec['path']);held.enter_context(access.guard(path))
                    if file_hash(path)!=spec['sha256']:raise ValueError('ChangedCurrentIndexAccess')
                    record=read(path)
                    if identity(record)!=spec['identity'] or base.scientific!=control['parent']['scientific']:
                        raise ValueError('WrongCurrentIndexAccess')
                    bind_committed_geometry(base,record['geometry'],identity(record['geometry']))
                    pairspec=record['pairs']
                else:pairspec=dict(path=base.plan['pair_incidence_manifest'],root=base.plan['pair_incidence_root'])
                numeric=held.enter_context(geometry_session(base,adapter))
                pairs=held.enter_context(PairIncidence(base,pairspec['path'],pairspec['root']))
                yield numeric,pairs
                base.check()
            finally:
                if prior is None:
                    if hasattr(base,'numeric_incidence'):del base.numeric_incidence
                else:base.numeric_incidence=prior
    return selected()


def validate_execution_parent(base,control,layer):
    if base.layer!=layer-1:raise ValueError('NonSequentialCommittedParent')
    if control is not None and layer==control['layers'][0] and (
        base.scientific!=control['parent']['scientific'] or base.reference!=control['parent']['reference']):
        raise ValueError('WrongSuccessorExecutionParent')


class ProductionStack:
    """A live installed session. Bound cases cannot replace stage executors."""
    def __init__(self,executors,profile):
        from types import MappingProxyType
        try:self._profile=ExecutionProfile(profile)
        except (TypeError,ValueError):raise ValueError('UnsupportedProductionProfile') from None
        self._executors=MappingProxyType(dict(executors));self._closed=False;self._busy=False

    @property
    def profile(self):return self._profile

    @property
    def closed(self):return self._closed

    def step(self,base,adapter,folder,pairs,*,observe=None,prepare_check=None,
             retained=None):
        """One authenticated-parent transition, mandatory acceptance and reopen.

        observe reports stage boundaries; prepare_check is an additional diagnostic
        rejection, not an acceptance alternative. Neither is read from case JSON.
        retained is the existing hash-bound assembly-resume triple, not skip flags.
        """
        if self.closed:raise ValueError('ClosedProductionStack')
        if self._busy:raise ValueError('ConcurrentProductionStep')
        self._busy=True
        try:return self._step(base,adapter,folder,pairs,observe,prepare_check,retained)
        finally:self._busy=False

    def _step(self,base,adapter,folder,pairs,observe,prepare_check,retained):
        from composable_state import CommittedState
        from path_preserving_deduplication import durable_json,file_hash
        from recursive_replay_retention import retain
        from retained_transition_input import stage as stage_retained
        if type(base) is not CommittedState:raise ValueError('QualifiedCommittedStateRequired')
        if any(f is not None and not callable(f) for f in (observe,prepare_check)):
            raise ValueError('InvalidProductionObserver')
        folder=Path(folder).resolve()
        if not folder.is_dir():raise ValueError('ProductionOutputDirectoryRequired')
        if any((folder/n).exists() for n in ('RESULT.json','STOP.json','PRODUCTION_STOP.json','PRODUCTION_STEP.json','REOPEN.json','REPLAY.json','CANDIDATE_PLAN.json')):
            raise ValueError('PreserveProductionStepEvidence')
        if adapter.binding['layer']!=base.layer or adapter.binding['next_layer']!=base.layer+1:
            raise ValueError('NonSequentialTransition')
        base.check();phases={};current='preflight';expected_science=None
        def stage(name,operation):
            nonlocal current
            current=name
            if observe is not None:observe(name,'before',None)
            tick=time.perf_counter();result=operation();seconds=time.perf_counter()-tick
            phases[name]=seconds
            if observe is not None:observe(name,'after',seconds)
            return result
        with ExitStack() as held:
            pending={folder,folder/'replay'}
            def release(path):
                if path in pending:
                    close_preparation_jobs(path);pending.remove(path)
            held.callback(release,folder)
            held.callback(release,folder/'replay')
            try:
                if retained is None:
                    stage('admission',lambda:self._executors['admission'](base,adapter,adapter.trajectory_spec,folder))
                    stage('generation',lambda:self._executors['generation'](base,adapter,adapter.trajectory_spec,folder))
                else:
                    if not isinstance(retained,tuple) or len(retained)!=3:raise ValueError('CompleteRetainedManifestBindingRequired')
                    stage('retained_input_validation',lambda:stage_retained(retained[0],folder,retained[1],retained[2],base,adapter,held))
                stage('assembly',lambda:self._executors['assembly'](base,adapter,folder,pair_index=pairs))
                publisher=self._executors['publication']
                if self.profile is ExecutionProfile.QUALIFICATION:
                    replay=folder/'replay'
                    def replay_operation():
                        import composable_publication as publication
                        replay.mkdir(exist_ok=False)
                        for name in ('ADMISSION.json','DIRECT.json','LOCALIZATION.json','SCREEN_ARRAYS.npz'):
                            shutil.copyfile(folder/name,replay/name)
                            if file_hash(folder/name)!=file_hash(replay/name):raise ValueError('ReplayInputMismatch')
                        self._executors['generation'](base,adapter,adapter.trajectory_spec,replay)
                        self._executors['assembly'](base,adapter,replay,pair_index=pairs)
                        return publication.prepare(base,adapter,replay)
                    replay_plan=stage('replay',replay_operation)
                    reference,expected_science=stage('publication',lambda:publish_replayed(
                        base,adapter,folder,pairs,replay_plan,publisher,extra_check=prepare_check))
                    release(replay)
                elif prepare_check is not None:
                    reference,expected_science=stage('publication',lambda:publish_checked(
                        base,adapter,folder,pairs,publisher,prepare_check))
                else:
                    reference=stage('publication',lambda:publisher(base,adapter,folder,pairs))
                base.check()
                def reopen():
                    with CommittedState(reference) as child:
                        child.check()
                        if (child.kind!='recursive' or child.layer!=base.layer+1
                            or child.parent.reference!=base.reference or child.parent.scientific!=base.scientific
                            or expected_science is not None and child.scientific!=expected_science):
                            raise ValueError('PublishedReopenMismatch')
                        durable_json(folder/'REOPEN.json',dict(passed=True,reference=reference,scientific=child.scientific))
                        return child.scientific
                science=stage('guarded_reopen',reopen)
                if self.profile is ExecutionProfile.QUALIFICATION:
                    stage('successful_replay_retention',lambda:retain(folder))
                result=dict(passed=True,profile=self.profile.value,layer=base.layer+1,reference=reference,
                    scientific=science,phases=phases,replay=self.profile is ExecutionProfile.QUALIFICATION)
                durable_json(folder/'PRODUCTION_STEP.json',result)
                return result
            except Exception as error:
                import traceback
                durable_json(folder/'PRODUCTION_STOP.json',dict(stage=current,code=str(error),
                    exception=type(error).__name__,traceback=traceback.format_exc(),phases=phases))
                raise


@contextmanager
def session(profile=ExecutionProfile.PRODUCTION):
    """Install once; keep indexed/cache scopes warm across sequential steps."""
    try:profile=ExecutionProfile(profile)
    except (TypeError,ValueError):raise ValueError('UnsupportedProductionProfile') from None
    with qualified_dispatch() as executors:
        stack=ProductionStack(executors,profile)
        try:yield stack
        finally:stack._closed=True
