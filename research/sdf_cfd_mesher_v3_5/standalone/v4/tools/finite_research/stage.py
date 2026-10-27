"""Private downstream composition. Never alters upstream prism policy or owners.

This is an uninstalled connection. The live entry remains disabled. The caller
must retain the existing authenticated job/input custody for the whole call.
"""
from contextlib import contextmanager, ExitStack
from contextvars import ContextVar
from pathlib import Path
from types import FunctionType
import hashlib, importlib, json, math, sys, threading, time

HERE = Path(__file__).resolve().parent
V4 = HERE.parents[1]
NAMES = (
    ('finite_stage_controls', 'tet_layer_controls'),
    ('finite_stage_adapter', 'tet_layer_adapter'),
    ('finite_stage_background', 'standalone_background'),
    ('finite_stage_command', 'finite_collar_command'),
    ('finite_stage_handoff', 'tet_layer_handoff'),
)
_active_helpers = ContextVar('finite_stage_active_helpers', default=None)


def active_background():
    """Return the existing selected helper and its exact lifetime check."""
    bound = _active_helpers.get()
    if bound is None:
        raise ValueError('ActivePrivateStageHelpersRequired')
    modules, require_active = bound
    require_active()
    return modules['finite_stage_background'], require_active


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def request(path, pins):
    path = Path(path).resolve()
    if pins.get(str(path)) != sha(path):
        raise ValueError('UnboundDownstreamRequest')
    value = json.loads(path.read_text())
    from finite_research.flexible_controls import selected as flexible
    if flexible(value.get('downstream_policy',{}).get('tet_transition'))!=(value.get('schema')=='FiniteCollarDownstreamRequest/5'):
        raise ValueError('FlexibleTransitionRequestVersionMismatch')
    fields = {'schema', 'upstream_case_sha256', 'upstream_effective_identity',
        'upstream_geometry_identity', 'upstream_flags', 'maximum_faces',
        'downstream_policy', 'downstream_policy_identity', 'live_execution_authorized'}
    independent=value.get('schema') in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5')
    if independent:fields|={'prism_producer_mode','prism_schedule_identity'}
    if value.get('schema') in ('FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):fields.add('transition_fluid_verification')
    if value.get('schema') in ('FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):fields.add('execution_target')
    if value.get('schema')=='FiniteCollarDownstreamRequest/5':
        from finite_research.flexible_controls import validate_request
        fields|={'extent_policy_identity','quality_targets_identity','controls_identity'}
        from finite_research.flexible_controls import graded
        if graded(value['downstream_policy']['tet_transition']):fields.add('constructor_schema')
        validate_request(value,value['downstream_policy']['tet_transition'])
    if 'stage_timing' in value:
        from stage_observations import choice as timing_choice
        timing_choice(value);fields.add('stage_timing')
    if set(value) != fields or value['schema'] not in ('FiniteCollarDownstreamRequest/1','FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
        raise ValueError('ExplicitDownstreamRequestRequired')
    from finite_research.input_contract import fluid_choice
    fluid_choice(value)
    from finite_research.execution_target import choice
    target=choice(value)
    if target=='transition_background' and value['schema'] not in ('FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
        raise ValueError('ChangedFiniteExecutionTargetSchema')
    if identity(value['downstream_policy']) != value['downstream_policy_identity']:
        raise ValueError('ChangedDownstreamPolicyIdentity')
    if type(value['maximum_faces']) is not int or value['maximum_faces'] <= 0:
        raise ValueError('PositiveExplicitFaceBoundRequired')
    from finite_research.input_contract import observer_flag,upstream_flags,producer_mode
    mode=producer_mode(value['upstream_flags'])
    if independent!=(mode=='source_directions') or (independent and value['prism_producer_mode']!=mode):
        raise ValueError('ChangedPrismProducerMode')
    if value['upstream_flags']!=upstream_flags(observer_flag(value['upstream_flags']),value['upstream_flags']['through_layer'],mode,target):
        raise ValueError('ChangedUpstreamObserverFlags')
    return value


def validate_case(value, case, original_pin, increments=None):
    if (original_pin != value['upstream_case_sha256']
            or identity(case) != value['upstream_effective_identity']
            or identity(case['geometry']) != value['upstream_geometry_identity']):
        raise ValueError('ChangedUpstreamCaseOrSource')
    if value.get('schema') in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
        from finite_research.input_contract import producer_flags
        if (increments is None or value.get('prism_producer_mode')!='source_directions'
                or producer_flags(dict(prism_producer_mode='source_directions',
                    upstream_forecast_diagnostics=False,**({'execution_target':value['execution_target']} if 'execution_target' in value else {})),case,increments)!=value['upstream_flags']
                or identity(case['prisms'])!=value.get('prism_schedule_identity')):
            raise ValueError('ChangedIndependentProducerCase')
        return
    # This connection replaces only the downstream transition configuration.
    # All upstream background/sizing requirements retain their original meaning.
    upstream = {k: v for k, v in case['background'].items() if k != 'tet_transition'}
    downstream = {k: v for k, v in value['downstream_policy'].items() if k != 'tet_transition'}
    if identity(upstream) != identity(downstream):
        raise ValueError('ChangedNonTransitionBackgroundPolicy')


@contextmanager
def clock_scope(path, pins, request_root):
    """Consume the already source-bound outer CLOCK; do not trust an env value."""
    from immutable_state_access import guard
    import native_scope
    native_scope.require_transport()
    path = Path(path).resolve()
    if path != Path(request_root).resolve()/'CLOCK.json':
        raise ValueError('ExactOwnedClockPathRequired')
    with guard(path):
        digest = sha(path)
        if pins.get(str(path)) != digest:
            raise ValueError('UnboundOuterClock')
        record = json.loads(path.read_text())
        deadline = record.get('deadline')
        if type(deadline) not in (float, int) or not math.isfinite(deadline):
            raise ValueError('InvalidOuterDeadline')
        active = True
        def check():
            if not active:
                raise ValueError('ClosedOuterClock')
            if time.perf_counter() >= deadline:
                raise TimeoutError('NoSharedStageBudget')
        check.deadline = deadline
        check.remaining_seconds = lambda: max(0., deadline-time.perf_counter())
        check()
        try:
            yield check
            check()
            if sha(path) != digest:raise ValueError('ChangedOuterClock')
        finally:
            active = False


@contextmanager
def selected_helpers(pins, observed=None):
    """Load only private helper names; retain canonical owner objects exactly."""
    from sdf_cfd_mesher_v4.runtime import frozen_imports
    from immutable_state_access import guard
    if _active_helpers.get() is not None:
        raise ValueError('NestedPrivateStageHelpers')
    if threading.current_thread() is not threading.main_thread() or threading.active_count() != 1:
        raise ValueError('SingleThreadedStageHostRequired')
    canonical_names = ('standalone_background', 'tet_layer_controls', 'tet_layer_adapter',
        'tet_layer_planning', 'accepted_trajectory_contract', 'tet_layer_handoff', 'rolling_fast_state')
    canonical = {n: importlib.import_module(n) for n in canonical_names}
    for n, module in canonical.items():
        if Path(module.__file__).resolve() != V4/'tools'/(n+'.py'):
            raise ValueError('OriginalUpstreamModuleRequired', n)
    owner = canonical['tet_layer_handoff']
    objects = (owner.TransitionInputs, owner._issued, owner._capture, owner.fluid_owner)
    background_owner = canonical['rolling_fast_state']
    registry_names = ('BackgroundInputs','LiveAcceptance','_background_issued','_background_active','_background_views','_background_pending')
    background_objects = {n:getattr(background_owner,n) for n in registry_names}
    aliases = tuple(a for name, _ in NAMES for a in (name, 'sdf_cfd_mesher_v4._frozen_'+name))
    if any(a in sys.modules for a in aliases):
        raise ValueError('FreshPrivateStageHelpersRequired')
    path_before = sys.path[:]
    primary = None
    modules = {}
    functions = {}
    active = False
    from path_preserving_deduplication import durable_json as original_writer
    from optimized_durable_json import durable_json as optimized_writer
    import standalone_tool as cli
    def unchanged():
        if any(sys.modules.get(n) is not m for n, m in canonical.items()):
            raise ValueError('CanonicalUpstreamOwnerReplaced')
        if any(a is not b for a,b in zip((owner.TransitionInputs, owner._issued, owner._capture, owner.fluid_owner), objects)):
            raise ValueError('CanonicalTransitionIssuerChanged')
        if any(getattr(background_owner,n) is not obj for n,obj in background_objects.items()):
            raise ValueError('CanonicalBackgroundIssuerChanged')
        producer=modules.get('finite_stage_handoff')
        if producer is not None and any(a is not b for a,b in zip((producer.TransitionInputs,producer._issued,producer._capture,producer.fluid_owner),objects)):
            raise ValueError('DuplicateTransitionIssuer')
        for name, filename in NAMES:
            if name not in modules:continue
            module=modules[name];path=HERE/'candidate'/(filename+'.py')
            if (sys.modules.get(name) is not module or sys.modules.get('sdf_cfd_mesher_v4._frozen_'+name) is not module
                    or Path(module.__file__).resolve()!=path or Path(module.__spec__.origin).resolve()!=path
                    or pins.get(str(path))!=sha(path)):
                raise ValueError('ChangedPrivateStageBinding',name)
        # All function aliases remain exact; the existing canonical installer
        # deliberately updates module-level durable_json aliases in place.
        for (name, alias), (value, defining, symbol) in functions.items():
            expected = cli.durable_json if alias == 'durable_json' else value
            if alias == 'durable_json' and expected not in (original_writer, optimized_writer):
                raise ValueError('UnknownCanonicalStageWriter')
            if getattr(modules[name], alias, None) is not expected:
                raise ValueError('ChangedPrivateStageFunction', name, alias)
            if alias != 'durable_json' and defining is not None:
                if sys.modules.get(defining.__name__) is not defining or getattr(defining, symbol, None) is not value:
                    raise ValueError('ChangedStageDefiningFunction', name, alias)
    def require_active():
        if not active:
            raise ValueError('ClosedPrivateStageHelpers')
        if threading.current_thread() is not threading.main_thread():
            raise ValueError('MainThreadStageConsumerRequired')
        unchanged()
    try:
        with frozen_imports._lock, ExitStack() as held:
            for name, filename in NAMES:
                path = HERE/'candidate'/(filename+'.py')
                held.enter_context(guard(path))
                if pins.get(str(path)) != sha(path):
                    raise ValueError('ChangedPrivateStageHelper', name)
                modules[name] = frozen_imports._load(name, path)
                unchanged()
            producer = modules['finite_stage_handoff']
            if any(a is not b for a,b in zip((producer.TransitionInputs, producer._issued, producer._capture, producer.fluid_owner), objects)):
                raise ValueError('DuplicateTransitionIssuer')
            for name, module in modules.items():
                for alias, value in vars(module).items():
                    if isinstance(value, (FunctionType, type)):
                        defining = sys.modules.get(value.__module__)
                        if defining is not None and getattr(defining, value.__name__, None) is not value:
                            defining = None  # Decorated wrappers retain their own exact identity.
                        functions[name, alias] = (value, defining, value.__name__)
            active = True
            require_active()
            helper_token = _active_helpers.set((modules, require_active))
            try:
                yield modules, require_active
                unchanged()
            finally:
                _active_helpers.reset(helper_token)
    except BaseException as error:
        primary = error
        raise
    finally:
        active = False
        cleanup_error = None
        try:
            unchanged()
        except BaseException as error:
            cleanup_error = error
        try:
            if observed is not None:
                observed.update({alias:str(Path(sys.modules[alias].__file__).resolve()) for alias in aliases if alias in sys.modules})
        except BaseException as error:
            if cleanup_error is None:cleanup_error=error
            else:cleanup_error.add_note('Stage observation cleanup: '+repr(error))
        finally:
            for alias in aliases:
                sys.modules.pop(alias, None)  # Private aliases only.
            sys.path[:] = path_before
        if cleanup_error is not None:
            if primary is None:raise cleanup_error
            primary.add_note('Stage owner cleanup: '+repr(cleanup_error))



@contextmanager
def terminal_dispatch(case_path, request_path, pins, outer_check, observed=None):
    """Return the real downstream result from the existing terminal seam."""
    if json.loads(Path(request_path).read_text()).get('schema')=='DirectEvidenceStageRequest/1':
        from finite_research.direct_runtime import terminal_dispatch as direct_dispatch
        direct_scope=direct_dispatch(case_path,request_path,pins,outer_check,observed)
        with direct_scope:
            yield
        return
    import standalone_background as background
    import standalone_tool as cli
    from standalone_case import hold_inputs, load
    from immutable_state_access import guard
    from production_surface_authority import load_production_wall
    from rolling_fast_state import BackgroundInputs
    from path_preserving_deduplication import durable_json
    if not callable(outer_check) or not callable(getattr(outer_check, 'remaining_seconds', None)):
        raise ValueError('OwnedOuterClockRequired')
    with ExitStack() as held:
        # Load before cli.main opens the canonical SDF reader threads. Keep this
        # exact scope through upstream execution and genuine terminal consumption.
        modules, require_helpers = held.enter_context(selected_helpers(pins, observed))
        held.enter_context(guard(Path(request_path)))
        value = request(request_path, pins)
        held.enter_context(hold_inputs(case_path))
        original_case, original_increments, original_pin = load(case_path)
        validate_case(value, original_case, original_pin,original_increments)
        if value['schema'] in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
            from finite_research.prism_connection import selected
            held.enter_context(selected(request_path,sha(request_path),outer_check))
        original_background, original_surface = background.execute, cli.surface_stage
        called = False
        def surface(*args, **kwargs):
            if cli._upstream_forecast_diagnostics.get() is not value['upstream_flags']['upstream_forecast_diagnostics']:
                raise ValueError('ChangedUpstreamObserverInvocation')
            from finite_research.input_contract import producer_mode
            if cli.prism_producer_mode()!=producer_mode(value['upstream_flags']):
                raise ValueError('ChangedPrismProducerInvocation')
            outer_check()
            result = original_surface(*args, **kwargs)
            if len(load_production_wall(result[0])[1]) > value['maximum_faces']:
                raise ValueError('FixtureSurfaceFaceCap')
            outer_check()
            return result
        def dispatch(run, actual_case_path, folder, expected_backend, *,
                     _handoff, _preparation, _transition_only):
            nonlocal called
            outer_check()
            require_helpers()
            if called or type(_transition_only) is not bool or _transition_only is not value['upstream_flags']['transition_only'] or type(_handoff) is not BackgroundInputs:
                raise ValueError('OneGenuineTerminalHandoffRequired')
            _handoff.require_origin(run)
            if _handoff._open()['preparation'] is not _preparation:
                raise ValueError('ChangedTerminalPreparation')
            actual, _, _ = load(actual_case_path)
            if identity(actual) != identity(original_case):
                raise ValueError('ChangedTerminalCase')
            connection = run.provider.binding.metric_connection
            connection.verify(run.provider.binding)
            if connection.source_identity != identity(actual):
                raise ValueError('ChangedTerminalSourceBinding')
            if len(_handoff.view.outer) > value['maximum_faces']:
                raise ValueError('FixtureTerminalFaceCap')
            flags = value['upstream_flags']
            from finite_research.input_contract import producer_mode
            if (cli._upstream_forecast_diagnostics.get() is not flags['upstream_forecast_diagnostics']
                    or cli.prism_producer_mode()!=producer_mode(flags)
                    or cli.surface_assurance() != flags['surface_assurance']
                    or cli.carrier_assessment_enabled() != (flags['surface_carrier'] == 'adaptive')
                    or cli.transition_only() != flags['transition_only']
                    or run.provider.layer != flags['through_layer']):
                raise ValueError('ChangedUpstreamInvocation')
            binding = dict(request_sha256=sha(request_path), original_case_sha256=original_pin,
                effective_case_identity=identity(actual), geometry_source_identity=identity(actual['geometry']),
                downstream_policy_identity=value['downstream_policy_identity'], parent_prism_mesh=_handoff.view.live_acceptance.mesh)
            if value['schema'] in ('FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):binding['execution_target']=value['execution_target']
            if value['schema']=='FiniteCollarDownstreamRequest/5':
                from finite_research.flexible_controls import identities
                binding.update(**identities(value['downstream_policy']['tet_transition']))
            if value['schema'] in ('FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
                binding['transition_fluid_verification']=value['transition_fluid_verification']
            if value['schema'] in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
                binding.update(prism_producer_mode=value['prism_producer_mode'],
                    prism_schedule_identity=value['prism_schedule_identity'])
            consumer_clock=outer_check
            if value['schema'] in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
                from finite_research.prism_connection import terminal_clock
                consumer_clock=terminal_clock(outer_check)
            called = True
            result = modules['finite_stage_background'].execute(run, actual_case_path, folder,
                expected_backend, _handoff=_handoff, _preparation=_preparation, _transition_only=_transition_only,
                _downstream_policy=json.loads(json.dumps(value['downstream_policy'])),
                _outer_check=consumer_clock, _stage_binding=binding)
            require_helpers()
            consumer_clock()
            _handoff.require_origin(run)
            if value['schema'] in ('FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
                from finite_research.fluid_assurance import validate_component
                validate_component(result,value['transition_fluid_verification'],binding)
            if value['schema'] in ('FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5') and value['execution_target']=='transition_background':
                from finite_research.execution_target import validate_result
                validate_result(result,binding)
            durable_json(Path(folder)/'STAGE_BINDING.json', binding)
            if binding.get('constructor_schema')=='GradedSupportAdvance/1':
                result['publication_binding']=dict(binding)
            consumer_clock()
            return result
        background.execute, cli.surface_stage = dispatch, surface
        primary = None
        try:
            yield
        except BaseException as error:
            primary = error
            raise
        finally:
            try:
                if background.execute is not dispatch or cli.surface_stage is not surface:
                    raise ValueError('ChangedTerminalDispatchDuringCall')
            except BaseException as error:
                if primary is None:
                    raise
                primary.add_note('Terminal dispatch cleanup: '+repr(error))
            finally:
                background.execute, cli.surface_stage = original_background, original_surface


def stage_clock(seconds, outer_check, clock=time.perf_counter):
    if (not callable(outer_check) or not callable(getattr(outer_check,'remaining_seconds',None))
            or type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<=0
            or type(getattr(outer_check,'deadline',None)) not in (int,float)
            or not math.isfinite(outer_check.deadline)):
        raise ValueError('AuthenticatedOuterStageClockRequired')
    outer_check()
    deadline=min(clock()+seconds,outer_check.deadline)
    def check():
        outer_check()
        if clock()>=deadline:raise ValueError('TetTransitionWholeStageTimeLimit')
    check.remaining_seconds=lambda:max(0.,deadline-clock())
    check.deadline=deadline
    return check
