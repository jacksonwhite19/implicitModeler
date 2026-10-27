"""Call-scoped selection of the preserved prism producer; never an acceptance."""
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
import hashlib,json,sys,threading,time

_selection=ContextVar('independent_prism_connection',default=None)
_terminal_budget=ContextVar('independent_terminal_budget',default=None)

def _active_background():
    """Use the exact stage already selected by the common command owner."""
    name='finite_collar_stage_adapter'
    stage=sys.modules.get(name)
    path=Path(__file__).resolve().parent/'stage.py'
    if (stage is None
            or sys.modules.get('sdf_cfd_mesher_v4._frozen_'+name) is not stage
            or Path(stage.__file__).resolve()!=path
            or Path(stage.__spec__.origin).resolve()!=path):
        raise ValueError('SelectedCommandStageRequired')
    return stage.active_background()

def identity(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),
                                    allow_nan=False).encode()).hexdigest()

@contextmanager
def selected(request_path,expected_sha256,outer_check):
    if _selection.get() is not None:raise ValueError('NestedPrismConnection')
    path=Path(request_path).resolve()
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected_sha256:
        raise ValueError('ChangedIndependentPrismSelection')
    value=json.loads(raw)
    if (value.get('schema') not in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5','DirectEvidenceStageRequest/1')
            or value.get('prism_producer_mode')!='source_directions'):
        raise ValueError('ExplicitIndependentPrismSelectionRequired')
    if not callable(outer_check) or not callable(getattr(outer_check,'remaining_seconds',None)):
        raise ValueError('OwnedOuterClockRequired')
    outer_check()
    background,require_helpers=_active_background()
    token=_selection.set((path,expected_sha256,raw,threading.get_ident(),outer_check,
                          background,require_helpers))
    budget_token=_terminal_budget.set(None)
    try:yield
    finally:
        _terminal_budget.reset(budget_token)
        _selection.reset(token)

def require(case,increments,through_layer):
    bound=_selection.get()
    if bound is None:raise ValueError('OwnedIndependentPrismSelectionRequired')
    path,digest,raw,thread,outer_check,background,require_helpers=bound
    outer_check()
    if threading.get_ident()!=thread or path.read_bytes()!=raw:
        raise ValueError('ChangedIndependentPrismSelection')
    value=json.loads(raw)
    from finite_research.input_contract import producer_flags
    flags=producer_flags(dict(prism_producer_mode='source_directions',
        upstream_forecast_diagnostics=False,**({'execution_target':value['execution_target']} if 'execution_target' in value else {})),case,increments)
    if (through_layer!=len(increments) or flags!=value['upstream_flags']
            or identity(case)!=value['upstream_effective_identity']
            or identity(case['geometry'])!=value['upstream_geometry_identity']
            or identity(case['prisms'])!=value['prism_schedule_identity']
            or identity(value['downstream_policy'])!=value['downstream_policy_identity']):
        raise ValueError('ChangedIndependentPrismInputs')
    return value

def begin_terminal(case,increments,through_layer):
    """Start the finite budget before genuine accepted-parent capture."""
    value=require(case,increments,through_layer)
    if _terminal_budget.get() is not None:raise ValueError('OneIndependentTerminalAdmissionRequired')
    direct=value.get('execution_target')=='direct_complete'
    seconds=value['downstream_policy']['downstream_seconds'] if direct else value['downstream_policy']['tet_transition']['seconds']
    if direct and _selection.get()[4].remaining_seconds()<seconds:
        raise TimeoutError('DirectDownstreamReserveRequired')
    if type(seconds) not in (int,float) or not (0<seconds<=1440 or seconds==10800):
        raise ValueError('BoundedWholeIndependentTransitionRequired')
    deadline=min(time.perf_counter()+seconds,_selection.get()[4].deadline)
    _terminal_budget.set((_selection.get(),deadline,(case,increments,through_layer)))
    return value['downstream_policy']

def terminal_assurance(policy):
    """Validate the bound finite policy through its existing live helper owner."""
    check_terminal()
    budget=_terminal_budget.get()
    case,increments,through_layer=budget[2]
    value=require(case,increments,through_layer)
    if identity(policy)!=value['downstream_policy_identity']:
        raise ValueError('ChangedIndependentTerminalPolicy')
    background,require_helpers=_active_background()
    if (background is not _selection.get()[5]
            or require_helpers is not _selection.get()[6]):
        raise ValueError('ChangedIndependentHelperLifetime')
    # active_background has just run this exact helper's require_active check.
    # Preserve a validator's primary failure if the exit check also rejects.
    primary=None
    try:
        if value.get('execution_target')=='direct_complete':
            from finite_research.direct_contract import validate_policy
            validate_policy(policy)
            mode='backend_trusted'
        else:
            mode=background.assurance_mode(policy)
    except BaseException as error:
        primary=error
        raise
    finally:
        try:
            require_helpers()
        except BaseException as error:
            if primary is None:raise
            primary.add_note('Independent policy helper exit: '+repr(error))
    check_terminal()
    value=require(case,increments,through_layer)
    if identity(policy)!=value['downstream_policy_identity']:
        raise ValueError('ChangedIndependentTerminalPolicy')
    return mode

def terminal_fluid_verification(policy,binding,parent):
    """Authenticate the choice against this live terminal and selected request."""
    check_terminal()
    case,increments,through_layer=_terminal_budget.get()[2]
    value=require(case,increments,through_layer)
    from rolling_fast_state import BackgroundInputs
    from finite_research.input_contract import fluid_choice
    if type(parent) is not BackgroundInputs:raise ValueError('GenuineTerminalPrismInputsRequired')
    origin=parent._open()['origin']
    parent.require_origin(origin);parent.require(parent.view)
    if (type(binding) is not dict or origin.provider.layer!=through_layer
            or binding.get('request_sha256')!=_selection.get()[1]
            or binding.get('effective_case_identity')!=identity(case)
            or binding.get('geometry_source_identity')!=identity(case['geometry'])
            or binding.get('prism_schedule_identity')!=identity(case['prisms'])
            or binding.get('parent_prism_mesh')!=parent.view.live_acceptance.mesh
            or binding.get('downstream_policy_identity')!=identity(policy)
            or identity(policy)!=value['downstream_policy_identity']):
        raise ValueError('ChangedTerminalFluidAuthority')
    from finite_research.execution_target import choice
    if choice(binding)!=choice(value):raise ValueError('ChangedTerminalExecutionTarget')
    mode=fluid_choice(value)
    if value['schema']=='FiniteCollarDownstreamRequest/5':
        from finite_research.flexible_controls import identities
        if any(binding.get(name)!=digest for name,digest in identities(policy['tet_transition']).items()):
            raise ValueError('ChangedTerminalFlexiblePolicy')
    if value['schema'] in ('FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
        if ('transition_fluid_verification' not in value
                or binding.get('transition_fluid_verification')!=mode):
            raise ValueError('ChangedTerminalFluidSelection')
    elif 'transition_fluid_verification' in binding or mode!='continuous_source':
        raise ValueError('LegacyTerminalFluidProofRequired')
    if terminal_assurance(policy)!='backend_trusted':
        raise ValueError('RoutineBackgroundHandoffRequired')
    check_terminal();parent.require_origin(origin)
    return mode


def terminal_preflight(run,handoff,case,increments,through_layer):
    """Admit downstream geometry only through a genuinely issued live parent."""
    value=require(case,increments,through_layer)
    from rolling_fast_state import BackgroundInputs
    if type(handoff) is not BackgroundInputs:raise ValueError('GenuineTerminalPrismInputsRequired')
    handoff.require_origin(run)
    handoff.require(handoff.view)
    if run.provider.layer!=through_layer:raise ValueError('AcceptedTerminalPrismRequired')
    budget=_terminal_budget.get()
    if budget is None or budget[0] is not _selection.get():raise ValueError('LiveIndependentTerminalBudgetRequired')
    check_terminal()
    if value.get('execution_target')=='direct_complete':
        from finite_research.direct_runtime import preflight
        result=preflight(run,handoff,value)
        check_terminal()
        handoff.require_origin(run)
        return result
    from finite_stage_background import preflight
    from finite_research.execution_target import choice
    result=preflight(run.provider.binding,value['downstream_policy'],transition_only=value['upstream_flags']['transition_only'],execution_target=choice(value))
    if time.perf_counter()>=budget[1]:raise TimeoutError('IndependentTransitionWholeStageTimeLimit')
    handoff.require_origin(run)
    return result

def check_terminal():
    bound=_terminal_budget.get()
    if bound is None or bound[0] is not _selection.get():
        raise ValueError('LiveIndependentTerminalBudgetRequired')
    bound[0][4]()
    if time.perf_counter()>=bound[1]:raise TimeoutError('IndependentTransitionWholeStageTimeLimit')

def terminal_clock(outer_check):
    """Include preflight, custody capture and finite construction in one bound."""
    bound=_terminal_budget.get()
    if bound is None or bound[0] is not _selection.get():
        raise ValueError('LiveIndependentTerminalBudgetRequired')
    deadline=min(bound[1],outer_check.deadline)
    def check():
        outer_check()
        if _terminal_budget.get() is not bound or bound[0] is not _selection.get():
            raise ValueError('ClosedIndependentTerminalBudget')
        if time.perf_counter()>=deadline:raise TimeoutError('IndependentTransitionWholeStageTimeLimit')
    check.deadline=deadline
    check.remaining_seconds=lambda:max(0.,deadline-time.perf_counter())
    check()
    return check

def terminal_execution_context():
    check_terminal()
    bound=_selection.get()
    return bound[0].parent/'live01',terminal_clock(bound[4])

def initial_arguments(increments,completion,completed,transition,diagnostics,mode):
    """One caller seam, shared by both existing source authority branches."""
    if mode=='source_directions':
        if completion is not None or completed or transition is not None or diagnostics:
            raise ValueError('IndependentPrismProducerOptionsRequired')
        schedule=None
    elif mode=='coupled':schedule=increments
    else:raise ValueError('UnknownPrismProducerMode')
    return dict(_rolling_schedule=schedule,_rolling_completion=completion,
        _rolling_completed=completed,_rolling_transition=transition,
        _rolling_diagnostics=diagnostics)
