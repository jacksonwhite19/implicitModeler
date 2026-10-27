"""Fresh FastProd binding into existing FastLayer, never a second mesher.

Initialization receives the actual checked first-layer provider, not a retained
checkpoint path or a caller-written passed receipt. Live acceptance is private
to the process. Qualification/publication/restart retain their separate owners.
"""
import stage_observations as obs
from types import SimpleNamespace
from pathlib import Path
import json
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, frozen, array_hash
from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction, ExecutionProfile
from incremental_layer_validation import Topology, Quality, ExactVolume, key, t
from rolling_fast_state import LiveAcceptance, mesh_identity, numeric_identity, cache_identity, verify_fresh_coverage, current_quality


class RollingGeometry:
    def __init__(self, initial, increments, state, *, _preparation=None, completed_front=False):
        if type(completed_front) is not bool:raise ValueError('InvalidRollingFaceRoleScope')
        self.completed_front=completed_front
        from standalone_geometry import is_scalar
        self.x = frozen(initial.x, '<f8'); self.tri = frozen(initial.tri, '<i8')
        self.authority = dict(state['authority']); self.binding = initial.binding
        self.binding.qualification.verify(self.binding.authority)
        h = np.asarray(increments, dtype='<f8')
        if h.ndim != 1 or not len(h) or not np.isfinite(h).all() or np.any(h <= 0) or h[0] != initial.h1:
            raise ValueError('InvalidRollingSchedule')
        self.increments = frozen(h, '<f8')
        self.scalar = is_scalar(self.binding)
        self._transition_policy_json=getattr(initial,'_rolling_transition_json',None)
        self._transition_plan=None
        if self._transition_policy_json is not None:
            from standalone_background import validate_policy
            policy=json.loads(self._transition_policy_json)
            if self.scalar or policy.get('schema')!='V4GradedBackground/6' or not completed_front:
                raise ValueError('RollingProspectiveTransitionScopeMismatch')
            validate_policy(policy)
            plan=getattr(initial,'_trajectory_plan',None)
            from accepted_trajectory_contract import _SourcePrismPlan
            if type(plan) is not _SourcePrismPlan or plan.summary()['inputs'].get('transition_policy_identity')!=identity(policy):
                raise ValueError('RollingProspectiveTransitionPolicyMismatch')
            plan.check(initial.source,self.x,initial.high,self.tri,initial.refs['face_refs'],
                np.broadcast_to(self.increments[:,None],(len(h),len(self.x))))
            self._transition_plan=plan
            self._transition_first=frozen(initial.high,initial.high.dtype.str)
            self._transition_refs=frozen(initial.refs['face_refs'],initial.refs['face_refs'].dtype.str)
        self.spec = dict(initial.spec) if self.scalar else dict(owner=self.binding.geometry_identity, units='m', quantity='path_length_m')
        self.schedule = SimpleNamespace(targets=frozen(np.cumsum(h, dtype=np.float64), '<f8'))
        self.source = initial.source
        self.geometry_source = None
        if self.scalar:
            import geometry_family_authority
            self.geometry_source = geometry_family_authority
            self.owner = None
        elif len(h) > 1:
            from accepted_trajectory_contract import AcceptedTrajectoryOwner
            plan=getattr(initial,'_trajectory_plan',None)
            if plan is not None:
                from accepted_trajectory_contract import _SourcePrismPlan
                if type(plan) is not _SourcePrismPlan:raise ValueError('LiveSourcePrismPlanRequired')
                if plan.summary()['completed_front'] is not self.completed_front:raise ValueError('WrongSourcePrismPlanScope')
                expected_policy=None if self._transition_policy_json is None else identity(json.loads(self._transition_policy_json))
                if plan.summary()['inputs'].get('transition_policy_identity')!=expected_policy:
                    raise ValueError('RollingProspectiveTransitionPolicyMismatch')
            self.owner = AcceptedTrajectoryOwner.from_inputs(self.source, self.x, initial.high,
                self.tri, initial.refs['face_refs'], self.increments,
                parent=mesh_identity(state,_preparation=_preparation), surface=initial.wall_authority['surface_identity'],
                request=__import__('production_prism_sizing').read_request(initial.request, initial.wall_authority),
                case_identity=identity(dict(geometry=self.binding.geometry_identity, increments=array_hash(self.increments))),
                _plan=getattr(initial,'_trajectory_plan',None))
        else:
            self.owner = None  # Exactly L1: no continuation premise is requested.
        self._inputs = self.input_identity()
        self.layer = 1; self.session = None

    def input_identity(self):
        inputs=dict(authority=self.authority, geometry=self.binding.geometry_identity,
            surface_assurance=getattr(self.binding.qualification,'assurance','qualification'),
            completed_front=self.completed_front,
            wall=array_hash(self.x), faces=array_hash(self.tri), increments=array_hash(self.increments),
            targets=array_hash(self.schedule.targets), scalar=self.scalar, spec=self.spec,
            source_spec=self.source.spec if self.scalar else self.source.spline_packet if hasattr(self.source,'spline_packet') else self.binding.qualification.target)
        transition_json=getattr(self,'_transition_policy_json',None)
        if transition_json is not None:
            inputs['transition_policy_identity']=identity(json.loads(transition_json))
            plan=getattr(self,'_transition_plan',None)
            inputs['transition_plan']=None if plan is None else dict(instance=id(plan),identity=plan.plan_identity)
        return identity(inputs)

    def check(self):
        if self.input_identity() != self._inputs: raise ValueError('ChangedRollingCaseInputs')
        self.binding.qualification.verify(self.binding.authority)
        if self.owner is not None: self.owner.check()
        if self._transition_plan is not None:
            self._transition_plan.check(self.source,self.x,self._transition_first,self.tri,self._transition_refs,
                np.broadcast_to(self.increments[:,None],(len(self.increments),len(self.x))))

    def transition_proposal(self,actual_cap,triangles,controls):
        """Live numerical proposal only; current transition admission remains."""
        self.check()
        if self._transition_policy_json is None:return None
        policy=json.loads(self._transition_policy_json)
        if identity(controls)!=identity(policy['tet_transition']):raise ValueError('RollingTransitionControlsMismatch')
        if self._transition_plan is None:raise ValueError('LiveSourcePrismPlanRequired')
        return self._transition_plan.transition_proposal(actual_cap,triangles,controls)

    def geometry(self, state, extra=0):
        self.check()
        if self.session is None or self.session.current is not state:
            raise ValueError('WrongRollingMemoryParent')
        n = len(self.x)
        if len(state['vertices']) != (self.layer+1)*n:
            raise ValueError('RollingFrontInventoryMismatch')
        fronts = [np.asarray([state['vertices'][f'layer:{k}:source:{v}'] for v in range(n)], dtype='<f8')
                  for k in range(1, self.layer+1)]
        if extra not in (0, 1): raise ValueError('UnsupportedRollingForecastScope')
        if extra:
            if self.layer >= len(self.increments): raise ValueError('NoPendingIncrement')
            if self.scalar:
                from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
                high = predict_next_front(fronts[-1], self.schedule.targets[self.layer], self.source,
                    iteration_cap=256).coordinates
            else:
                positions = self.owner.positions()
                if fronts[-1].tobytes() != positions[self.layer].tobytes():
                    raise ValueError('RollingDeclaredParentMismatch')
                high = positions[self.layer+1]
            fronts.append(high)
        return fronts, np.full(n, self.layer, dtype=int)

    def validate_fast_target(self, before, high, layer):
        self.check()
        if before is not self.session.current or layer != self.layer+1:
            raise ValueError('StaleRollingTargetParent')
        if self.scalar:
            return self.geometry_source.sdf_record(self.spec, high, self.schedule.targets[layer-1])
        if np.asarray(high).tobytes() != self.owner.positions()[layer].tobytes():
            raise ValueError('RollingDeclaredTargetMismatch')
        # Execute the SAME existing supporting-normal premise, no new predictor.
        from accepted_production_contract import ProductionTrajectory, ProductionSchedule
        parent = mesh_identity(before)
        bound_owner = identity(dict(root=self.owner.source_identity, parent=parent,
            generation=self.session.generation, layer=self.layer, units='m'))
        schedule = ProductionSchedule(self.owner.schedule, bound_owner, parent)
        view = SimpleNamespace(check=self.check, premises=None, original=self.owner,
            record=self.owner.record, state=self.owner.state, positions=self.owner.positions,
            layer=self.layer, source_identity=bound_owner,
            parent=parent, schedule=schedule)
        premise = ProductionTrajectory.trajectory_premises(view)
        return dict(admitted=premise['passed'], premises=premise,
            quantity='path_length_m', not_transition_qualification=True)

    def prepare_commit(self, before, candidate, generation, layer):
        self.check()
        if (self.session.current is not before or self.session.generation != generation
                or layer != self.layer+1 or layer > len(self.increments)):
            raise ValueError('StaleRollingGeometryCommit')
        return candidate, generation, layer

    def verify_prepared(self, prepared, candidate, generation):
        self.check()
        if prepared[0] is not candidate or prepared[1:] != (generation, self.layer+1):
            raise ValueError('WrongRollingPreparedTarget')
        return True

    def finish_commit(self, prepared):
        self.layer = prepared[2]

    def fast_dependency_schedule(self):
        return self.increments  # Rolling mode has no archived forecast table.


@obs.observed('prism.initial_handoff','prism')
def from_initial(initial, increments, folder, *, _preparation=None, completed_front=False):
    """Initialize rolling caches after actual first-layer checks, before archive.

    Initial provider must stay live until this detached numeric/cache owner is
    returned. No proof_objects, sealed base, parent path or receipt handles remain.
    """
    from production_first_layer_run import ProductionFirstLayerProvider
    from live_spline_first_layer import SourceSheetFirstLayerProvider
    if type(initial) not in (ProductionFirstLayerProvider, SourceSheetFirstLayerProvider):
        raise ValueError('UnsupportedRollingInitialProvider')
    if initial.verify_wall() is not True or initial.assigned.get('admitted') is not True:
        raise ValueError('UnqualifiedRollingInitialTarget')
    from owned_source_segments import verify_provider,fresh_required
    segment_context=verify_provider(initial,required=fresh_required(initial))
    checked = initial.check
    if (checked.get('passed') is not True or any(checked[k].get('passed') is not True
            for k in ('quality', 'ownership', 'vertex_links'))
            or not checked['volume']['positive_cells'] or not checked['volume']['exact_volume_balance']
            or not checked['volume']['interior_degree']['passed']):
        raise ValueError('UnqualifiedRollingInitialGeometry')
    if any(c['layer'] != 1 or c['kind'] != 'prism' for c in initial.cells):
        raise ValueError('RollingRequiresFirstUniformOrdinaryLayer')
    owners, topology_check = t.ownership(initial.cells, initial.outer,
        [tuple(reversed(f)) for f in initial.tri])
    if not topology_check['passed']: raise ValueError('RollingInitialOwnershipMismatch')
    faces = [owners[k][0][1] for k in sorted(owners)]
    methods = verify_fresh_coverage(initial.points, faces, range(len(faces)), checked['certificate'])
    q = checked['quality']
    from final_shell_incidence import check
    prospective = check(initial.points, initial.cells, initial.outer)
    if not prospective['passed']: raise ValueError('RollingInitialIncidenceFailure')
    state = initial.pack([initial.high], np.ones(len(initial.x), dtype=int))
    state.update(quality={}, certificate={}, atlas={}, forecast={}, _production_unsealed=True)
    topology = Topology(initial.cells, owners)
    topology.boundary(initial.outer, [tuple(reversed(f)) for f in initial.tri])
    quality = Quality({r['id']: r for r in q['cells']},
        {key(r['vertices']): {k: v for k, v in r.items() if k != 'vertices'} for r in q['faces']},
        {r['id']: r for r in q['exact']}, {r['id']: r for r in prospective['cells']})
    quality.bind_masks(topology, initial.outer)
    from production_fast_layer import FastLayer
    run = object.__new__(FastLayer); run.rolling = True; run.profile = ExecutionProfile.ProductionFast
    run.folder = Path(folder); run.folder.mkdir(parents=True, exist_ok=True)
    run.initial_segment_context=segment_context
    run.tx = MemoryTransaction(state); run.provider = RollingGeometry(initial, increments, state,
        _preparation=_preparation,completed_front=completed_front)
    run.provider.session = run.tx; run.spec = run.provider.spec
    run.points = np.array(initial.points, copy=True); run.outer = list(initial.outer)
    run.wall = [tuple(reversed(f)) for f in initial.tri]; run.topology = topology
    run.quality = current_quality(quality, topology, 1)
    run.volume = ExactVolume(run.points, initial.cells, run.outer, initial.tri, quality.exact)
    if run.volume.total != run.volume.boundary_total: raise ValueError('RollingInitialExactVolumeFailure')
    from incremental_degree import DegreeState
    run.degree_state = DegreeState(run.points, run.wall+run.outer, run.volume.witness,
        checked['volume']['interior_degree'])
    run.proofs = None; run.face_keys = None; run.inputs = {}; run.vertex_incidence = {}
    run.live_acceptance = LiveAcceptance(mesh_identity(state,_preparation=_preparation), numeric_identity(run.points,topology,run.outer,run.wall),
        cache_identity(run.quality,run.volume,run.degree_state), identity(state['authority']),
        checked['certificate']['required_pairs'], tuple(sorted(methods.items())), 0)
    run.live_acceptance.check(run,_preparation=_preparation)
    return run


@obs.observed('prism.mesh_serialization','io_serialization')
def write_mesh(run, output, *, _handoff=None):
    """Keep one accumulated mesh export, not one mesh copy per layer."""
    import os
    from rolling_fast_state import export_mesh
    from path_preserving_deduplication import durable_json, file_hash
    temporary = output/'MESH.pending.npz'
    record = export_mesh(run, temporary, _handoff=_handoff)
    with temporary.open('r+b') as stream: os.fsync(stream.fileno())
    record['sha256'] = file_hash(temporary)
    os.replace(temporary, output/'MESH.npz')
    durable_json(output/'CURRENT.json', dict(record, layer=run.provider.layer,
        restart_supported=False, archived_certificate_available=False,
        certified_relations=run.live_acceptance.count, methods=dict(run.live_acceptance.methods)))
    return record


def write_progress(run,output):
    """Accepted-layer telemetry only; no mesh bytes or reusable authority."""
    from path_preserving_deduplication import durable_json
    durable_json(output/'CURRENT.json',dict(schema='V4FastProdProgress/1',
        layer=run.provider.layer,generation=run.tx.generation,cells=len(run.topology.cells),
        mesh_export_available=False,restart_supported=False,published_scientific_authority=False))


def _terminal_assurance_mode(policy,*,independent):
    """Select full policy validation before acquiring accepted-parent custody."""
    if type(independent) is not bool:raise ValueError('ExplicitTerminalPolicyRouteRequired')
    if independent:
        from finite_research.prism_connection import terminal_assurance
        return terminal_assurance(policy)
    from standalone_background import assurance_mode
    return assurance_mode(policy)


def _terminal_background_handoff(run,case_path,through_layer,expected,scope,*,_downstream_policy=None):
    """Existing custody owner shared only by terminal export and routine core."""
    if expected is None or run.provider.layer!=through_layer:return None,None
    from standalone_case import load
    policy=load(case_path)[0].get('background') if _downstream_policy is None else _downstream_policy
    if _terminal_assurance_mode(policy,independent=_downstream_policy is not None)!='backend_trusted':return None,None
    from background_preparation import held
    preparation=scope.enter_context(held())
    handoff=scope.enter_context(run.live_acceptance.background_inputs(run,_preparation=preparation,_capture_volume=policy['schema']=='V4GradedBackground/6'))
    return handoff,preparation


@obs.observed('engine.execute','engine')
def execute(case_path, output, through_layer, *, intermediate_meshes=False, _upstream_diagnostics=False):
    """Same standalone command and stage owners; rolling FastLayer retention.

    L0 still uses the existing qualified surface entry. L1 still uses the
    existing generation/coverage predicates, not native checkpoint publication.
    No retained mesh, parent or historical campaign is an execution input.
    """
    import shutil, time, traceback
    if type(intermediate_meshes) is not bool:raise ValueError('InvalidIntermediateMeshExport')
    import standalone_tool as cli
    from standalone_case import load
    from path_preserving_deduplication import durable_json, file_hash
    from production_prism_stack import initial_front_dispatch
    import run_live_sf2_first_layer as initial
    from production_surface_authority import load_production_wall,fastprod_surface_scope
    output = Path(output).resolve()
    if output.exists(): raise ValueError('OutputMustBeNew')
    total_started=time.perf_counter();timing={};background_result=None;transition_result=None;direct_pending=False
    def measured(name,fn):
        t=time.perf_counter()
        try:
            with obs.span('engine.'+name,'engine_stage'):
                return fn()
        finally:timing[name]=time.perf_counter()-t
    if cli.stop_after()=='seed':
        from finite_research.partial_stages import seed_stage
        return seed_stage(case_path,output,through_layer)
    pre = measured('public_preflight',lambda:cli.preflight(case_path, 0, through_layer))
    producer_case, increments, pin = load(case_path)
    independent=cli.prism_producer_mode()=='source_directions'
    downstream=None
    if independent:
        from finite_research.prism_connection import require
        require(producer_case,increments,through_layer)
    output.mkdir(parents=True); started = time.perf_counter(); run = None
    from standalone_case import copy_inputs
    copy_inputs(case_path,output/'CASE.json')
    durable_json(output/'PREFLIGHT.json', dict(pre, execution_profile='fastprod', restart_supported=False))
    stage = 'surface'
    scope=fastprod_surface_scope(output/'surface/A/fresh');scope.__enter__()
    try:
        surface, surface_record = measured('surface_stage',lambda:cli.surface_stage(output, output/'CASE.json',profile='fastprod'))
        if through_layer == 0 or cli.stop_after()=='surface':
            from production_surface_authority import FASTPROD_NUMERICAL_ASSURANCE
            numerical_surface=surface_record['assurance']==FASTPROD_NUMERICAL_ASSURANCE
            durable_json(output/'RESULT.json', dict(passed=True, last_accepted_layer=0,last_qualified_layer=None if numerical_surface else 0,
                surface_assurance=surface_record['assurance'],surface_formally_qualified=not numerical_surface,
                surface_authority=surface_record,surface_readback_verified=True,
                restart_supported=False,published_scientific_authority=False,
                replay_performed=False,seconds=time.perf_counter()-started)); return
        stage = 'initial_front'
        root = output/'initial'; root.mkdir()
        measured('first_request',lambda:cli.first_request(root, surface, output/'CASE.json'))
        measured('initial_wall_handoff',lambda:initial.configure_inputs(root, surface, root/'REQUEST.json',load_production_wall(surface)[2]['surface_identity']))
        initial.STANDALONE_INPUTS = output/'CASE.json'
        def accept(provider, result, evidence):
            nonlocal run
            # Reuse the existing qualified native row selector. Every hash and
            # predicate still reads its actual inputs; no acceptance is cached.
            from background_preparation import held
            with held() as preparation:
                run = from_initial(provider, increments[:through_layer], output/'work',
                    _preparation=preparation,completed_front=pre.get('background') is not None)
            # All initial rows were independently replayed from actual AABBs
            # by from_initial, before dropping their archival representation.
            return run.live_acceptance.mesh
        from contextlib import ExitStack
        with ExitStack() as owned, initial_front_dispatch():
            terminal_handoff=None;terminal_preparation=None
            def prepare_terminal_handoff():
                nonlocal terminal_handoff,terminal_preparation,downstream,stage
                if cli.stop_after()=='prisms':return
                if run.provider.layer!=through_layer or (pre.get('background') is None and not independent):return
                if terminal_handoff is not None:
                    terminal_handoff.require_origin(run)
                    timing['terminal_handoff_producer_capture']=terminal_handoff.summary['capture_seconds']
                    timing['terminal_handoff_producer_issue']=terminal_handoff.summary['producer_issue_seconds']
                    return
                if independent:
                    stage='accepted_prism_handoff'
                    from finite_research.prism_connection import begin_terminal
                    downstream=dict(policy=begin_terminal(load(output/'CASE.json')[0],increments,through_layer),backend=None)
                terminal_scope=ExitStack()
                # Register before capture so failures also close the scope.
                # Account for destruction where the original background owner
                # paid it; do not hide relocated cleanup outside stage totals.
                owned.callback(lambda:measured('terminal_handoff_cleanup',terminal_scope.close))
                terminal_handoff,terminal_preparation=measured('terminal_handoff_setup',lambda:
                    _terminal_background_handoff(run,output/'CASE.json',through_layer,
                        downstream if independent else pre['background'],terminal_scope,
                        _downstream_policy=downstream['policy'] if independent else None))
                if independent:
                    stage='independent_transition_preflight'
                    from finite_research.prism_connection import terminal_preflight
                    downstream=measured('independent_transition_preflight',lambda:
                        terminal_preflight(run,terminal_handoff,load(output/'CASE.json')[0],increments,through_layer))
                    stage='accepted_prism_export'
            host, binding = measured('initial_geometry_binding',lambda:cli.geometry(output/'CASE.json'))
            owned.callback(host.close)
            def first_worker():
                from general_sdf_authority import GeneralSdfAuthority
                background=load(output/'CASE.json')[0].get('background')
                transition=(background if background is not None and background['schema']=='V4GradedBackground/6' else None)
                from finite_research.prism_connection import initial_arguments
                arguments=initial_arguments(increments[:through_layer],
                    load(output/'CASE.json')[0]['prisms'].get('trajectory_completion'),
                    pre.get('background') is not None,transition,_upstream_diagnostics,cli.prism_producer_mode())
                if type(binding.authority) is GeneralSdfAuthority and binding.authority.stock_prism_queries():
                    # First normals and continuation normals have the same live
                    # source, wall bits and refs. Keep only this call's existing
                    # qualified cache until the acceptance callback completes.
                    with binding.authority.prism_query_scope() as statistics:
                        initial.worker('candidate',True,commit_action=accept,geometry_session=(host,binding),**arguments)
                        durable_json(output/'INITIAL_SOURCE_QUERY_ARITHMETIC.json',statistics())
                else:
                    initial.worker('candidate',True,commit_action=accept,geometry_session=(host,binding),**arguments)
            measured('initial_worker',first_worker)
            from canonical_spline_authority import LiveSplineAuthority,verified_export_scope
            if type(run.provider.binding.authority) is LiveSplineAuthority:
                source_scope=ExitStack()
                owned.callback(lambda:measured('source_custody_cleanup',source_scope.close))
                measured('source_export_custody_setup',lambda:source_scope.enter_context(verified_export_scope(run.provider.binding.authority)))
                if run.provider.owner is not None:
                    measured('trajectory_input_custody_setup',lambda:source_scope.enter_context(run.provider.owner.verified_input_scope()))
            prepare_terminal_handoff()
            if independent and terminal_handoff is not None:
                from finite_research.prism_connection import check_terminal
                check_terminal()
            if intermediate_meshes or through_layer==1:
                measured('L1_mesh_export',lambda:write_mesh(run, output,_handoff=terminal_handoff))
            else:measured('L1_progress',lambda:write_progress(run,output))
            if independent and terminal_handoff is not None:check_terminal()
            # Current engine owns no provider.check/receipt/path to this scratch.
            # Keep compact initial diagnostics/hashes before retiring its unique
            # verbose FastProd-only proof stream. Never delete older run inputs.
            scratch = (root/'candidate').resolve()
            if scratch != output/'initial/candidate' or not scratch.is_relative_to(output):
                raise ValueError('RollingScratchScopeMismatch')
            segment_archive=None
            if getattr(run,'initial_segment_context',None) is not None:
                from owned_source_segments import archive
                segment_archive=measured('initial_source_segment_evidence',lambda:archive(run.initial_segment_context,output))
            files = measured('initial_scratch_manifest',lambda:{p.relative_to(scratch).as_posix():dict(bytes=p.stat().st_size,sha256=file_hash(p))
                     for p in scratch.rglob('*') if p.is_file()})
            durable_json(output/'INITIAL_SUMMARY.json', dict(result=cli.read(scratch/'RESULT.json'),
                files=files, mesh=run.live_acceptance.mesh, proof_coverage_independently_replayed=True,
                **({'owned_source_segments':segment_archive} if segment_archive is not None else {})))
            measured('initial_scratch_retirement',lambda:shutil.rmtree(scratch))
            for layer in range(2, through_layer+1):
                stage = f'L{layer}'
                cli.check_resources(output, load(output/'CASE.json')[0]['resources'])
                if cli.stop_after()=='complete' and layer==through_layer and pre.get('background') is not None:
                    from standalone_background import assurance_mode
                    if assurance_mode(load(output/'CASE.json')[0]['background'])=='backend_trusted':
                        from background_preparation import held
                        from rolling_fast_state import _producer_background_inputs
                        def arm_terminal():
                            nonlocal terminal_handoff,terminal_preparation
                            terminal_scope=ExitStack()
                            owned.callback(lambda:measured('terminal_handoff_cleanup',terminal_scope.close))
                            terminal_preparation=terminal_scope.enter_context(held())
                            terminal_handoff=terminal_scope.enter_context(_producer_background_inputs(run,_preparation=terminal_preparation,_capture_volume=load(output/'CASE.json')[0]['background']['schema']=='V4GradedBackground/6'))
                        measured('terminal_handoff_setup',arm_terminal)
                result = measured(f'L{layer}_transition',run.step)
                prepare_terminal_handoff()
                if independent and terminal_handoff is not None:
                    from finite_research.prism_connection import check_terminal
                    check_terminal()
                if intermediate_meshes or layer==through_layer:
                    measured(f'L{layer}_mesh_export',lambda:write_mesh(run, output,_handoff=terminal_handoff))
                else:measured(f'L{layer}_progress',lambda:write_progress(run,output))
                if independent and terminal_handoff is not None:check_terminal()
                # Replace latest telemetry. No parent file/path participates.
                durable_json(output/'LAST_STEP.json', result)
                cli.check_resources(output, load(output/'CASE.json')[0]['resources'])
            if cli.stop_after()=='complete' and (pre.get('background') is not None or independent):
                transition_only=cli.transition_only()
                stage='tet_transition' if transition_only else 'background'
                from standalone_background import execute as background_stage
                component_folder='transition' if transition_only else 'background'
                produced=measured(stage+'_stage',lambda:background_stage(run,output/'CASE.json',output/component_folder,(downstream if independent else pre['background'])['backend'],
                    _handoff=terminal_handoff,_preparation=terminal_preparation,_transition_only=transition_only))
                if independent and require(producer_case,increments,through_layer).get('execution_target')=='direct_complete':
                    from finite_research.direct_runtime import evidence_only
                    evidence_only(produced,output)
                    direct_pending=True
                else:
                    direct_pending=False
                if not direct_pending:
                    current=cli.read(output/'CURRENT.json')
                    if transition_only:
                        if produced.get('transition_accepted') is not True or produced.get('background_invoked') is not False:
                            raise ValueError('TransitionOnlyPublicationMismatch')
                        transition_result=dict(path='transition/TRANSITION.npz',acceptance='transition/TRANSITION.json',
                            sha256=produced['arrays_sha256'],acceptance_sha256=produced['report_sha256'],tetrahedra=produced['tetrahedra'],
                            outer_bulk_neighbours_required=True,restart_supported=False)
                        if 'transition_fluid_verification' in produced:
                            transition_result.update(transition_fluid_verification=produced['transition_fluid_verification'],
                                fluid_assurance=produced['fluid_assurance'],acceptance_schema=produced['acceptance_schema'])
                        current.update(output_role='prism_shell',transition=transition_result,complete_mesh=None)
                    else:
                        background_result=produced
                        if independent:
                            if (produced.get('execution_target')!='transition_background' or produced.get('transition_accepted') is not True
                                    or produced.get('background_invoked') is not True):raise ValueError('JoinedFinitePublicationMismatch')
                            current.update(execution_target='transition_background',transition_accepted=True,
                                transition_fluid_assurance=produced['fluid_assurance'],transition=produced['tet_transition'])
                        current.update(output_role='prism_shell',complete_mesh=dict(path='background/COMPLETE_MESH.npz',
                            sha256=background_result['mesh_sha256'],qualification='background/QUALIFICATION.json',
                            assurance=background_result['assurance']))
                    graded_publication=produced.get('acceptance_schema')=='V4TetLayerAcceptance/5'
                    if graded_publication:
                        from finite_research.published_consumer import publish
                        from finite_research.prism_connection import terminal_execution_context,terminal_fluid_verification
                        from owned_runtime import V4
                        selected=require(producer_case,increments,through_layer)
                        publication_binding=produced['publication_binding']
                        current.update(execution_target=publication_binding['execution_target'],transition_accepted=True,
                            background_invoked=not transition_only)
                        terminal_fluid_verification(selected['downstream_policy'],publication_binding,terminal_handoff)
                        unused,publication_clock=terminal_execution_context()
                        if not transition_only:
                            current['complete_mesh']['qualification_sha256']=produced['qualification_sha256']
                        current['stage_binding_sha256']=file_hash(output/component_folder/'STAGE_BINDING.json')
                        current['stage_binding_path']=component_folder+'/STAGE_BINDING.json'
                        def consumer_check():
                            publication_clock()
                            terminal_handoff.require_origin(run)
                            terminal_fluid_verification(selected['downstream_policy'],publication_binding,terminal_handoff)
                            cli.check_resources(output,load(output/'CASE.json')[0]['resources'])
                        publish(output,current,consumer_check,dict(stage_binding=publication_binding,
                            source_release_sha256=file_hash(V4/'runtime/RELEASE.json'),parent_prism_mesh=run.live_acceptance.mesh))
                    else:durable_json(output/'CURRENT.json',current)
                    cli.check_resources(output,load(output/'CASE.json')[0]['resources'])
        durable_json(output/'RESULT.json', dict(passed=True, last_qualified_layer=run.provider.layer,
            direct_evidence_provisional=direct_pending,
            surface_assurance=surface_record['assurance'],
            surface_formally_qualified=surface_record['qualification']['geometry'] is True,
            mesh=run.live_acceptance.mesh, restart_supported=False, published_scientific_authority=False,
            mesh_identity_scope='accepted prism shell',
            **(dict(execution_target='transition',transition_accepted=transition_result is not None,
                transition=transition_result) if cli.transition_only() else {}),
            **(dict(execution_target='transition_background',transition_accepted=True,
                transition_fluid_assurance=background_result['fluid_assurance'],transition=background_result['tet_transition'],joined_contract=background_result['joined_contract'])
                if independent and background_result is not None else {}),
            background_accepted=background_result is not None,
            background_qualified=(background_result is not None and
                background_result['assurance']['mode']=='qualification'),
            complete_mesh=(None if background_result is None else dict(path='background/COMPLETE_MESH.npz',
                sha256=background_result['mesh_sha256'],qualification='background/QUALIFICATION.json',
                assurance=background_result['assurance'])),
            seconds=time.perf_counter()-started))
    except BaseException as error:
        durable_json(output/'STOP.json', dict(stage=stage, error=str(error), traceback=traceback.format_exc(),
            last_accepted_layer=run.provider.layer if run is not None else 0,
            no_retry=True, restart_supported=False, seconds=time.perf_counter()-started))
        raise
    finally:
        import sys
        primary=sys.exc_info()[1]
        try:
            scope.__exit__(type(primary) if primary is not None else None,primary,
                primary.__traceback__ if primary is not None else None)
        except BaseException as cleanup_error:
            if primary is None:
                raise
            primary.add_note('Rolling owner cleanup: '+repr(cleanup_error))
        try:
            durable_json(output/'TIMING.json',dict(stages=timing,seconds=time.perf_counter()-total_started,
                nested_stage_notes={
                    'terminal_handoff_producer_issue':'Includes capture and accepted token issuance within final prism transition. Add once to conservative inclusive background/terminal comparison; do not also add capture.',
                    'terminal_handoff_producer_capture':'Subset of producer_issue; already included in final prism transition.'},
                replay_performed=False,qualification_of_new_surface_required=True))
        except Exception as telemetry_error:
            if primary is None:raise
            # STOP already preserves the required first failure. Telemetry is
            # not allowed to replace that scientific/resource exception.
            primary.add_note('TimingWriteFailure: '+repr(telemetry_error))
