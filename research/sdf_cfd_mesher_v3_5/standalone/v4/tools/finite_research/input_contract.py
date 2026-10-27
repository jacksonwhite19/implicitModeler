"""Existing explicit assignment/input predicates, separated from code generation."""

from pathlib import Path

from datetime import datetime,timedelta

import hashlib,json,math

def assignment_window(task):
    version=task.get('schema')
    window=task.get('proposed_live_window_seconds')
    cutoff=task.get('proposed_live_launch_cutoff_seconds')
    if version=='ReusableFiniteInputAssignment/1':
        if window!=7200 or cutoff!=2400:
            raise ValueError('ExactAssignmentAllowancesRequired')
    elif version=='ReusableFiniteInputAssignment/2':
        if task.get('execution_target')!='direct_complete':
            raise ValueError('ExplicitDirectAssignmentVersionRequired')
        if (any(type(value) not in (int,float) or not math.isfinite(value) for value in (window,cutoff))
                or not 0<cutoff<window or window-cutoff<14700):
            raise ValueError('ExplicitAssignmentWindowWithOwnerReserveRequired')
    else:
        raise ValueError('ExactPendingOrOneLiveAssignmentRequired')
    return window,cutoff

def read(path): return json.loads(Path(path).read_text(encoding='utf-8'))

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def transition_allowance(value):
    if type(value) not in (int,float) or value not in (1440,10800):
        raise ValueError('ExplicitApprovedTransitionAllowanceRequired')
    return value

def preparation_allowance(seconds, selected, payload_seconds):
    allowance=transition_allowance(selected)
    if type(seconds) not in (int,float) or not seconds>0:
        return False
    if allowance==1440:
        return seconds<=min(1440,payload_seconds)
    return seconds==10800 and seconds<=payload_seconds

def assignment_intent(task):
    from datetime import datetime, timedelta
    observer_flag(task)
    from stage_observations import choice
    choice(task)
    count=task.get('maximum_live_invocations')
    if (task.get('schema') not in ('ReusableFiniteInputAssignment/1','ReusableFiniteInputAssignment/2')
            or type(count) is not int or count not in (0,1)):
        raise ValueError('ExactPendingOrOneLiveAssignmentRequired')
    permitted=count==1
    window,launch_cutoff=assignment_window(task)
    allowance=transition_allowance(task.get('proposed_transition_seconds'))
    if (task.get('execution_authorized') is not permitted
            or task.get('live_payload_seconds')!=(14400 if permitted else 0)
            or task.get('live_owner_seconds')!=(14700 if permitted else 0)
            or task.get('live_transition_seconds')!=(allowance if permitted else 0)
            or task.get('proposed_live_payload_seconds')!=14400
            or task.get('proposed_live_owner_seconds')!=14700
            or task.get('proposed_transition_seconds')!=allowance):
        raise ValueError('ExactAssignmentAllowancesRequired')
    dates=[task.get(k) for k in ('start_utc','deadline_utc','live_cutoff_utc')]
    if not permitted:
        if any(value is not None for value in dates):
            raise ValueError('PendingAssignmentDatesMustBeNull')
    else:
        if not all(isinstance(value,str) for value in dates):
            raise ValueError('ExactFreshLiveWindowRequired')
        start,end,cutoff=map(datetime.fromisoformat,dates)
        if (any(value.utcoffset() is None for value in (start,end,cutoff))
                or end-start!=timedelta(seconds=window)
                or cutoff-start!=timedelta(seconds=launch_cutoff)):
            raise ValueError('ExactFreshLiveWindowRequired')
    return permitted


def observer_flag(value):
    selected=value.get('upstream_forecast_diagnostics')
    if type(selected) is not bool:
        raise ValueError('ExplicitBooleanUpstreamObserverRequired')
    return selected


def producer_mode(value):
    mode=value.get('prism_producer_mode','coupled')
    if mode not in ('coupled','source_directions'):
        raise ValueError('UnknownPrismProducerMode')
    return mode


def fluid_choice(value, *, default='continuous_source'):
    choice=value.get('transition_fluid_verification',default)
    if type(choice) is not str or choice not in ('backend_trusted','continuous_source'):
        raise ValueError('ExplicitTransitionFluidVerificationRequired')
    return choice


def prepared_fluid_choice(assignment, argument=None):
    mode=producer_mode(assignment)
    choice=fluid_choice(assignment,default='backend_trusted' if mode=='source_directions' else 'continuous_source')
    if argument is not None and (type(argument) is not str or argument!=choice):
        raise ValueError('TransitionFluidArgumentAssignmentMismatch')
    if mode!='source_directions' and choice!='continuous_source':
        raise ValueError('TrustedFluidRequiresIndependentPrismRoute')
    return choice


def validate_fluid_binding(assignment, planned, stage):
    name='transition_fluid_verification'
    if stage.get('schema') not in ('FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5'):
        if any(name in value for value in (assignment,planned,stage)):
            raise ValueError('LegacyTransitionFluidChoiceNotSupported')
        return 'continuous_source'
    if (producer_mode(assignment)!='source_directions' or name not in planned or name not in stage
            or fluid_choice(planned)!=fluid_choice(stage)
            or fluid_choice(stage)!=prepared_fluid_choice(assignment)):
        raise ValueError('ChangedTransitionFluidSelection')
    return fluid_choice(stage)


def producer_flags(assignment,case,increments):
    mode=producer_mode(assignment)
    if case['schema']!='V4MeshCase/4':
        raise ValueError('CurrentSchema4ScopeRequired')
    if mode=='coupled':
        if len(increments)!=6:raise ValueError('CurrentSchema4SixLayerScopeRequired')
    elif ('background' in case or case['prisms'].get('trajectory_completion') is not None):
        raise ValueError('IndependentPrismProducerCaseRequired')
    from finite_research.execution_target import prepared
    return upstream_flags(observer_flag(assignment),len(increments),mode,prepared(assignment))


def upstream_flags(selected,through_layer=6,mode='coupled',target='transition'):
    from finite_research.execution_target import choice
    choice({'execution_target':target})
    if target in ('transition_background','direct_complete') and mode!='source_directions':raise ValueError('OuterBackgroundRequiresIndependentPrismRoute')
    if type(selected) is not bool:
        raise ValueError('ExplicitBooleanUpstreamObserverRequired')
    if mode not in ('coupled','source_directions'):
        raise ValueError('UnknownPrismProducerMode')
    if type(through_layer) is not int or through_layer<1:
        raise ValueError('PositivePrismScheduleEndpointRequired')
    if mode=='coupled' and through_layer!=6:
        raise ValueError('CurrentSchema4SixLayerScopeRequired')
    if mode=='source_directions' and selected:
        raise ValueError('IndependentProducerHasNoForecastObserver')
    flags=dict(surface_assurance='numerical',surface_carrier='nodal',
                through_layer=through_layer,transition_only=target=='transition',upstream_forecast_diagnostics=selected)
    if mode=='source_directions':flags['prism_producer_mode']=mode
    return flags


def validate_observer_binding(assignment, planned, stage):
    if planned.get('transition_stage_seconds')!=transition_allowance(assignment.get('proposed_transition_seconds')):
        raise ValueError('ChangedAssignedTransitionAllowance')
    from finite_research.flexible_controls import selected,identities
    controls=stage.get('downstream_policy',{}).get('tet_transition')
    if selected(controls):
        expected=identities(controls)
        if (stage.get('schema')!='FiniteCollarDownstreamRequest/5'
                or any(planned.get(name)!=digest or stage.get(name)!=digest for name,digest in expected.items())):
            raise ValueError('ChangedPreparedFlexiblePolicyBinding')
    from stage_observations import binding
    binding(assignment,planned,stage)
    from finite_research.execution_target import validate_binding
    target=validate_binding(assignment,planned,stage)
    validate_fluid_binding(assignment,planned,stage)
    selected=observer_flag(assignment)
    arguments=planned['arguments']
    if type(arguments) is not list or any(type(arg) is not str for arg in arguments):
        raise ValueError('ExplicitObserverArgumentListRequired')
    stops=[arg for arg in arguments if arg.split('=',1)[0]=='--stop-after-transition']
    if stops!=(['--stop-after-transition'] if target=='transition' else []):
        raise ValueError('ChangedFiniteExecutionTargetArguments')
    if type(stage['upstream_flags'].get('transition_only')) is not bool:
        raise ValueError('ExplicitBooleanTransitionTargetRequired')
    observed=[arg for arg in arguments if arg.split('=',1)[0]=='--upstream-forecast-diagnostics']
    if observed!=(['--upstream-forecast-diagnostics'] if selected else []):
        raise ValueError('ChangedUpstreamObserverArguments')
    if (observer_flag(planned) is not selected
            or observer_flag(stage['upstream_flags']) is not selected
            or producer_mode(assignment)!=producer_mode(planned)
            or producer_mode(planned)!=producer_mode(stage['upstream_flags'])
            or stage['upstream_flags']!=upstream_flags(selected,stage['upstream_flags']['through_layer'],producer_mode(assignment),target)
            or planned['arguments'].count('--upstream-forecast-diagnostics')!=int(selected)):
        raise ValueError('ChangedUpstreamObserverSelection')
    mode=producer_mode(assignment)
    marker='--prism-producer-mode'
    present=[i for i,arg in enumerate(arguments) if arg.split('=',1)[0]==marker]
    if mode=='source_directions':
        if len(present)!=1 or arguments[present[0]:present[0]+2]!=[marker,mode]:
            raise ValueError('ChangedPrismProducerArguments')
    elif present:raise ValueError('ChangedPrismProducerArguments')
    endpoint=[i for i,arg in enumerate(arguments) if arg.split('=',1)[0]=='--through-layer']
    if len(endpoint)!=1 or arguments[endpoint[0]:endpoint[0]+2]!=['--through-layer',str(stage['upstream_flags']['through_layer'])]:
        raise ValueError('ChangedPrismEndpointArguments')
    return selected


def assignment_capacity(task):
    value=task.get('maximum_faces')
    if type(value) is not int or value<=0:
        raise ValueError('ExplicitPositiveIntegerAssignmentCapacityRequired')
    return value


def select_input_contract(task, contract_id):
    rows=task.get('input_contracts',{})
    if not isinstance(rows,dict) or contract_id not in rows:
        raise ValueError('UnlistedInputContract')
    keys={'case_path','case_sha256','imported_source_path','imported_source_sha256','policy_path','policy_sha256'}
    signatures=[]
    for row in rows.values():
        if not isinstance(row,dict) or set(row)!=keys:
            raise ValueError('ExactRawInputContractRequired')
        if (row['imported_source_path'] is None)!=(row['imported_source_sha256'] is None):
            raise ValueError('ExplicitImportedSourceOrNullRequired')
        for name in ('case_path','policy_path','imported_source_path'):
            if row[name] is not None and str(Path(row[name]).resolve())!=row[name]:
                raise ValueError('AbsoluteOriginalInputPathRequired',name)
        signatures.append(json.dumps(row,sort_keys=True))
    if len(signatures)!=len(set(signatures)):
        raise ValueError('AmbiguousInputContracts')
    return dict(rows[contract_id])


def validate_bound_inputs(contract, planned, stage, canonical, maximum_faces):
    # Only assignment data selects the raw tuple. Canonical facts were derived
    # under the existing input owner, and the external review binds this record.
    for path_key,hash_key in (('case_path','case_sha256'),('policy_path','policy_sha256'),
                              ('imported_source_path','imported_source_sha256')):
        if contract[path_key] is not None and sha(contract[path_key])!=contract[hash_key]:
            raise ValueError('ChangedAllowedRawInput',path_key)
    if (planned['case']!=contract['case_path'] or planned['case_sha256']!=contract['case_sha256']
            or planned['imported_source_sha256']!=contract['imported_source_sha256']
            or planned['original_policy_path']!=contract['policy_path']
            or planned['original_policy_sha256']!=contract['policy_sha256']):
        raise ValueError('UnlistedPreparedInputTuple')
    if (canonical['case_sha256']!=contract['case_sha256']
            or canonical['imported_source_path']!=contract['imported_source_path']
            or canonical['imported_source_sha256']!=contract['imported_source_sha256']
            or canonical['policy_sha256']!=contract['policy_sha256']
            or stage['upstream_case_sha256']!=canonical['case_sha256']
            or stage['upstream_effective_identity']!=canonical['effective_case_identity']
            or stage['upstream_geometry_identity']!=canonical['geometry_identity']
            or stage['downstream_policy_identity']!=canonical['policy_identity']
            or stage['downstream_policy']!=read(contract['policy_path'])):
        raise ValueError('ChangedCanonicalInputBinding')
    expected=['--finite-collar-selection',planned['selector'],'--case',contract['case_path'],
        '--from-layer','0','--through-layer',str(stage['upstream_flags']['through_layer']),'--mode','fastprod',*(['--stop-after-transition'] if stage['upstream_flags']['transition_only'] else []),
        '--output',planned['output'],'--minimum-free-bytes','1000000000',
        '--surface-carrier','nodal','--surface-assurance','numerical']
    selected=observer_flag(planned)
    if observer_flag(stage['upstream_flags']) is not selected:
        raise ValueError('ChangedUpstreamObserverSelection')
    if selected:expected.append('--upstream-forecast-diagnostics')
    mode=producer_mode(planned)
    if mode=='source_directions':
        expected+=['--prism-producer-mode',mode]
        if (stage.get('schema') not in ('FiniteCollarDownstreamRequest/2','FiniteCollarDownstreamRequest/3','FiniteCollarDownstreamRequest/4','FiniteCollarDownstreamRequest/5')
                or stage.get('prism_producer_mode')!=mode
                or canonical.get('prism_schedule_identity')!=stage.get('prism_schedule_identity')):
            raise ValueError('ChangedPrismScheduleBinding')
    if (type(stage['maximum_faces']) is not int or stage['maximum_faces']!=maximum_faces):
        raise ValueError('CapacityAssignmentMismatch')
    if (planned['arguments']!=expected
            or stage['upstream_flags']!=upstream_flags(selected,stage['upstream_flags']['through_layer'],mode,__import__('finite_research.execution_target',fromlist=['choice']).choice(stage))
            or planned['proposed_payload_seconds']!=14400 or planned['proposed_owner_seconds']!=14700
            or planned['transition_stage_seconds']!=transition_allowance(stage['downstream_policy']['tet_transition']['seconds'])):
        raise ValueError('ChangedPreparedArgumentsOrBounds')
    return True


def live_task_from_assignment(task):
    permitted=assignment_intent(task)
    window,launch_cutoff=assignment_window(task)
    return dict(schema='PreparedFiniteLiveTask/1' if permitted else 'ProposedFiniteLiveTask/1',
        assignment=task['assignment'],human_message=task['human_message'],
        upstream_forecast_diagnostics=observer_flag(task),
        start_utc=task['start_utc'],launch_cutoff_utc=task['live_cutoff_utc'],deadline_utc=task['deadline_utc'],
        authorization_scope='PreparedResearchSeparatelyBoundedReviewedContinuation',
        maximum_invocations=task['maximum_live_invocations'],
        payload_seconds=task['live_payload_seconds'],owner_seconds=task['live_owner_seconds'],
        transition_seconds=task['live_transition_seconds'],proposed_payload_seconds=14400,
        proposed_owner_seconds=14700,proposed_transition_seconds=transition_allowance(task['proposed_transition_seconds']),proposed_window_seconds=window,
        proposed_launch_cutoff_seconds=launch_cutoff,live_execution_authorized=permitted,
        **({'execution_target':__import__('finite_research.execution_target',fromlist=['choice']).choice(task)} if 'execution_target' in task else {}),
        **({'transition_fluid_verification':fluid_choice(task)} if 'transition_fluid_verification' in task else {}))
