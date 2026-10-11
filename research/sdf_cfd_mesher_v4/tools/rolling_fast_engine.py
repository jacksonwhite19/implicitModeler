"""Fresh FastProd binding into existing FastLayer, never a second mesher.

Initialization receives the actual checked first-layer provider, not a retained
checkpoint path or a caller-written passed receipt. Live acceptance is private
to the process. Qualification/publication/restart retain their separate owners.
"""
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, frozen, array_hash
from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction, ExecutionProfile
from incremental_layer_validation import Topology, Quality, ExactVolume, key, t
from rolling_fast_state import LiveAcceptance, mesh_identity, numeric_identity, cache_identity, verify_fresh_coverage, current_quality


class RollingGeometry:
    def __init__(self, initial, increments, state):
        from standalone_geometry import is_scalar
        self.x = frozen(initial.x, '<f8'); self.tri = frozen(initial.tri, '<i8')
        self.authority = dict(state['authority']); self.binding = initial.binding
        self.binding.qualification.verify(self.binding.authority)
        h = np.asarray(increments, dtype='<f8')
        if h.ndim != 1 or not len(h) or not np.isfinite(h).all() or np.any(h <= 0) or h[0] != initial.h1:
            raise ValueError('InvalidRollingSchedule')
        self.increments = frozen(h, '<f8')
        self.scalar = is_scalar(self.binding)
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
            self.owner = AcceptedTrajectoryOwner.from_inputs(self.source, self.x, initial.high,
                self.tri, initial.refs['face_refs'], self.increments,
                parent=mesh_identity(state), surface=initial.wall_authority['surface_identity'],
                request=__import__('production_prism_sizing').read_request(initial.request, initial.wall_authority),
                case_identity=identity(dict(geometry=self.binding.geometry_identity, increments=array_hash(self.increments))))
        else:
            self.owner = None  # Exactly L1: no continuation premise is requested.
        self._inputs = self.input_identity()
        self.layer = 1; self.session = None

    def input_identity(self):
        return identity(dict(authority=self.authority, geometry=self.binding.geometry_identity,
            surface_assurance=getattr(self.binding.qualification,'assurance','qualification'),
            wall=array_hash(self.x), faces=array_hash(self.tri), increments=array_hash(self.increments),
            targets=array_hash(self.schedule.targets), scalar=self.scalar, spec=self.spec,
            source_spec=self.source.spec if self.scalar else self.source.spline_packet if hasattr(self.source,'spline_packet') else self.binding.qualification.target))

    def check(self):
        if self.input_identity() != self._inputs: raise ValueError('ChangedRollingCaseInputs')
        self.binding.qualification.verify(self.binding.authority)
        if self.owner is not None: self.owner.check()

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


def from_initial(initial, increments, folder):
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
    run.tx = MemoryTransaction(state); run.provider = RollingGeometry(initial, increments, state)
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
    run.live_acceptance = LiveAcceptance(mesh_identity(state), numeric_identity(run.points,topology,run.outer,run.wall),
        cache_identity(run.quality,run.volume,run.degree_state), identity(state['authority']),
        checked['certificate']['required_pairs'], tuple(sorted(methods.items())), 0)
    run.live_acceptance.check(run)
    return run


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


def _terminal_background_handoff(run,case_path,through_layer,expected,scope):
    """Existing custody owner shared only by terminal export and routine core."""
    if expected is None or run.provider.layer!=through_layer:return None,None
    from standalone_background import assurance_mode
    from standalone_case import load
    policy=load(case_path)[0].get('background')
    if assurance_mode(policy)!='backend_trusted':return None,None
    from background_preparation import held
    preparation=scope.enter_context(held())
    handoff=scope.enter_context(run.live_acceptance.background_inputs(run,_preparation=preparation))
    return handoff,preparation


def execute(case_path, output, through_layer, *, intermediate_meshes=False):
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
    total_started=time.perf_counter();timing={};background_result=None
    def measured(name,fn):
        t=time.perf_counter()
        try:return fn()
        finally:timing[name]=time.perf_counter()-t
    pre = measured('public_preflight',lambda:cli.preflight(case_path, 0, through_layer))
    _, increments, pin = load(case_path)
    output.mkdir(parents=True); started = time.perf_counter(); run = None
    from standalone_case import copy_inputs
    copy_inputs(case_path,output/'CASE.json')
    durable_json(output/'PREFLIGHT.json', dict(pre, execution_profile='fastprod', restart_supported=False))
    stage = 'surface'
    scope=fastprod_surface_scope(output/'surface/A/fresh');scope.__enter__()
    try:
        surface, surface_record = measured('surface_stage',lambda:cli.surface_stage(output, output/'CASE.json',profile='fastprod'))
        if through_layer == 0:
            from production_surface_authority import FASTPROD_NUMERICAL_ASSURANCE
            numerical_surface=surface_record['assurance']==FASTPROD_NUMERICAL_ASSURANCE
            durable_json(output/'RESULT.json', dict(passed=True, last_accepted_layer=0,last_qualified_layer=None if numerical_surface else 0,
                surface_assurance=surface_record['assurance'],surface_formally_qualified=not numerical_surface,
                restart_supported=False,published_scientific_authority=False,
                replay_performed=False,seconds=time.perf_counter()-started)); return
        stage = 'initial_front'
        root = output/'initial'; root.mkdir()
        measured('first_request',lambda:cli.first_request(root, surface, output/'CASE.json'))
        measured('initial_wall_handoff',lambda:initial.configure_inputs(root, surface, root/'REQUEST.json',load_production_wall(surface)[2]['surface_identity']))
        initial.STANDALONE_INPUTS = output/'CASE.json'
        def accept(provider, result, evidence):
            nonlocal run
            run = from_initial(provider, increments[:through_layer], output/'work')
            # All initial rows were independently replayed from actual AABBs
            # by from_initial, before dropping their archival representation.
            return run.live_acceptance.mesh
        from contextlib import ExitStack
        with ExitStack() as owned, initial_front_dispatch():
            terminal_handoff=None;terminal_preparation=None
            def prepare_terminal_handoff():
                nonlocal terminal_handoff,terminal_preparation
                if run.provider.layer!=through_layer or pre.get('background') is None:return
                if terminal_handoff is not None:
                    terminal_handoff.require_origin(run)
                    timing['terminal_handoff_producer_capture']=terminal_handoff.summary['capture_seconds']
                    timing['terminal_handoff_producer_issue']=terminal_handoff.summary['producer_issue_seconds']
                    return
                terminal_scope=ExitStack()
                # Register before capture so failures also close the scope.
                # Account for destruction where the original background owner
                # paid it; do not hide relocated cleanup outside stage totals.
                owned.callback(lambda:measured('terminal_handoff_cleanup',terminal_scope.close))
                terminal_handoff,terminal_preparation=measured('terminal_handoff_setup',lambda:
                    _terminal_background_handoff(run,output/'CASE.json',through_layer,pre['background'],terminal_scope))
            host, binding = measured('initial_geometry_binding',lambda:cli.geometry(output/'CASE.json'))
            owned.callback(host.close)
            def first_worker():
                from general_sdf_authority import GeneralSdfAuthority
                if type(binding.authority) is GeneralSdfAuthority and binding.authority.stock_prism_queries():
                    # First normals and continuation normals have the same live
                    # source, wall bits and refs. Keep only this call's existing
                    # qualified cache until the acceptance callback completes.
                    with binding.authority.prism_query_scope() as statistics:
                        initial.worker('candidate',True,commit_action=accept,geometry_session=(host,binding))
                        durable_json(output/'INITIAL_SOURCE_QUERY_ARITHMETIC.json',statistics())
                else:
                    initial.worker('candidate',True,commit_action=accept,geometry_session=(host,binding))
            measured('initial_worker',first_worker)
            from canonical_spline_authority import LiveSplineAuthority,verified_export_scope
            if type(run.provider.binding.authority) is LiveSplineAuthority:
                source_scope=ExitStack()
                owned.callback(lambda:measured('source_custody_cleanup',source_scope.close))
                measured('source_export_custody_setup',lambda:source_scope.enter_context(verified_export_scope(run.provider.binding.authority)))
                if run.provider.owner is not None:
                    measured('trajectory_input_custody_setup',lambda:source_scope.enter_context(run.provider.owner.verified_input_scope()))
            prepare_terminal_handoff()
            if intermediate_meshes or through_layer==1:
                measured('L1_mesh_export',lambda:write_mesh(run, output,_handoff=terminal_handoff))
            else:measured('L1_progress',lambda:write_progress(run,output))
            # Current engine owns no provider.check/receipt/path to this scratch.
            # Keep compact initial diagnostics/hashes before retiring its unique
            # verbose FastProd-only proof stream. Never delete older run inputs.
            scratch = (root/'candidate').resolve()
            if scratch != output/'initial/candidate' or not scratch.is_relative_to(output):
                raise ValueError('RollingScratchScopeMismatch')
            files = measured('initial_scratch_manifest',lambda:{p.relative_to(scratch).as_posix():dict(bytes=p.stat().st_size,sha256=file_hash(p))
                     for p in scratch.rglob('*') if p.is_file()})
            durable_json(output/'INITIAL_SUMMARY.json', dict(result=cli.read(scratch/'RESULT.json'),
                files=files, mesh=run.live_acceptance.mesh, proof_coverage_independently_replayed=True))
            measured('initial_scratch_retirement',lambda:shutil.rmtree(scratch))
            for layer in range(2, through_layer+1):
                stage = f'L{layer}'
                cli.check_resources(output, load(output/'CASE.json')[0]['resources'])
                if layer==through_layer and pre.get('background') is not None:
                    from standalone_background import assurance_mode
                    if assurance_mode(load(output/'CASE.json')[0]['background'])=='backend_trusted':
                        from background_preparation import held
                        from rolling_fast_state import _producer_background_inputs
                        def arm_terminal():
                            nonlocal terminal_handoff,terminal_preparation
                            terminal_scope=ExitStack()
                            owned.callback(lambda:measured('terminal_handoff_cleanup',terminal_scope.close))
                            terminal_preparation=terminal_scope.enter_context(held())
                            terminal_handoff=terminal_scope.enter_context(_producer_background_inputs(run,_preparation=terminal_preparation))
                        measured('terminal_handoff_setup',arm_terminal)
                result = measured(f'L{layer}_transition',run.step)
                prepare_terminal_handoff()
                if intermediate_meshes or layer==through_layer:
                    measured(f'L{layer}_mesh_export',lambda:write_mesh(run, output,_handoff=terminal_handoff))
                else:measured(f'L{layer}_progress',lambda:write_progress(run,output))
                # Replace latest telemetry. No parent file/path participates.
                durable_json(output/'LAST_STEP.json', result)
                cli.check_resources(output, load(output/'CASE.json')[0]['resources'])
            if pre.get('background') is not None:
                stage='background'
                from standalone_background import execute as background_stage
                background_result=measured('background_stage',lambda:background_stage(run,output/'CASE.json',output/'background',pre['background']['backend'],
                    _handoff=terminal_handoff,_preparation=terminal_preparation))
                current=cli.read(output/'CURRENT.json')
                current.update(output_role='prism_shell',complete_mesh=dict(path='background/COMPLETE_MESH.npz',
                    sha256=background_result['mesh_sha256'],qualification='background/QUALIFICATION.json',
                    assurance=background_result['assurance']))
                durable_json(output/'CURRENT.json',current)
                cli.check_resources(output,load(output/'CASE.json')[0]['resources'])
        durable_json(output/'RESULT.json', dict(passed=True, last_qualified_layer=run.provider.layer,
            surface_assurance=surface_record['assurance'],
            surface_formally_qualified=surface_record['qualification']['geometry'] is True,
            mesh=run.live_acceptance.mesh, restart_supported=False, published_scientific_authority=False,
            mesh_identity_scope='accepted prism shell',
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
        scope.__exit__(None,None,None)
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
