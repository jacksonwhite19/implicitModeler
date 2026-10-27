"""Stable finite research input preparation and activation; code is never generated."""
from contextlib import contextmanager,ExitStack
from pathlib import Path
import hashlib,json,math,sys
import stage_observations as obs
from finite_research.input_contract import (assignment_intent,assignment_capacity,
    select_input_contract,validate_bound_inputs,live_task_from_assignment,
    observer_flag,upstream_flags,validate_observer_binding,producer_mode,producer_flags,
    prepared_fluid_choice,fluid_choice,transition_allowance,preparation_allowance)
HERE=Path(__file__).resolve().parent
V4=HERE.parents[1]
TEMPLATE=HERE
FILES=('PYTHON_IDENTITY.json',)
FORBIDDEN=('AUTHORIZATION.json','LAUNCH_REVIEW.json','TASK.json','CLOCK.json',
    'live01_INPUT_PINS.json','CONTROL_AUTHORIZATION.json')

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):h.update(obs.byte_value(block))
    return h.hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf8'))
def encoded(value):return (json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
def write_new(path,raw):
    with Path(path).open('xb') as stream:stream.write(raw)
    if Path(path).read_bytes()!=raw:raise ValueError('PreparedFileReadbackMismatch',str(path))
def write(path,value):write_new(path,encoded(value))

def package_sources():
    import owned_runtime
    if Path(owned_runtime.__file__).resolve()!=V4/'tools/owned_runtime.py':
        raise ValueError('WrongOwnedRuntimeOrigin')
    release=owned_runtime.verify_release()
    values={str(V4/row['path']):row['sha256'] for row in release['sources']+release['independent_reviews']}
    values[str(V4/'runtime/RELEASE.json')]=sha(V4/'runtime/RELEASE.json')
    return values

def windows_sources():
    from finite_research.dependency_inputs import union_checked
    return union_checked(package_sources(),read(V4/'runtime/FINITE_WINDOWS_DEPENDENCIES.json')['files'])

def native_sources():
    """Conservative owned Python/scientific runtime closure, without Windows site-packages/history."""
    release=read(V4/'runtime/RELEASE.json');values={}
    for row in release['sources']:
        name=row['path'];suffix=Path(name).suffix.lower()
        code=name.startswith(('tools/','src/')) and suffix in ('.py','.json','.so')
        runtime=name.startswith(('runtime/surface/','runtime/source/','runtime/bin/','runtime/lib/')) and suffix not in ('.exe','.dll','.pyd','.pyc')
        metadata=name in ('runtime/MANIFEST.json','runtime/MMGS_DEPENDENCY_QUALIFICATION.json','runtime/FINITE_NATIVE_BINDING.json','runtime/BUILD_RELOCATION.json','runtime/FINITE_LOCK_TRANSITION.json','runtime/finite_qualification/INDEX.json')
        if code or runtime or metadata:values[str(V4/name)]=row['sha256']
    for row in release['independent_reviews']:values[str(V4/row['path'])]=row['sha256']
    values[str(V4/'runtime/RELEASE.json')]=sha(V4/'runtime/RELEASE.json')
    binding=read(V4/'runtime/FINITE_NATIVE_BINDING.json')
    for row in binding['files']:
        name=str(V4/row['package_relative']) if 'package_relative' in row else row['external_path']
        if name in values and values[name]!=row['sha256']:raise ValueError('ConflictingNativeDependency',name)
        values[name]=row['sha256']
    lock=binding['lock_reference'];values[str(V4/lock['package_relative'])]=lock['sha256']
    return values

def _attest_loaded_roots():
    expected={p.stem:p.resolve() for p in (V4/'tools').glob('*.py')}
    expected['fixture_study']=V4/'tools/finite_support/fixture_study.py'
    for name,module in list(sys.modules.items()):
        origin=getattr(module,'__file__',None)
        if not origin:continue
        path=Path(origin).resolve()
        if name in expected and path!=expected[name]:raise ValueError('WrongPreloadedOwnedOrigin',name)
        if name=='finite_research' or name.startswith(('finite_research.','sdf_cfd_mesher_v4')):
            if V4 not in path.parents:raise ValueError('WrongPreloadedPackageOrigin',name)


@contextmanager
def source_scope(expected_release,extra=None):
    _attest_loaded_roots()
    from immutable_state_access import guard
    release_path=V4/'runtime/RELEASE.json'
    with ExitStack() as held:
        held.enter_context(guard(release_path))
        if sha(release_path)!=expected_release:raise ValueError('ChangedOwnedReleaseBinding')
        pins=windows_sources()
        for name,digest in (extra or {}).items():
            name=str(Path(name).resolve())
            if name in pins and pins[name]!=digest:raise ValueError('ConflictingOperationIdentity',name)
            pins[name]=digest
        for name,digest in pins.items():
            held.enter_context(guard(name))
            if sha(name)!=digest:raise ValueError('ChangedOwnedInput',name)
        from finite_research import qualification,owner_selection
        qualification.verify()
        with owner_selection.selected_native(pins) as (native,attest):
            import fixture_study as d
            _attest_loaded_roots();attest()
            from sdf_cfd_mesher_v4.runtime import frozen_imports
            profile=HERE/'owner/resource_profile_16gib_3600/selection.py'
            selector=frozen_imports._load('finite_forward_resource_selection',profile)
            with selector.selected(pins):
                attest()
                yield d,pins
                attest()
        if sha(release_path)!=expected_release:raise ValueError('ChangedOwnedReleaseBinding')
        for name,digest in pins.items():
            if sha(name)!=digest:raise ValueError('ChangedOwnedInputAfterUse',name)

def input_pins(args):
    assignment=read(args.assignment);row=assignment.get('input_contracts',{}).get(args.input_contract)
    if not isinstance(row,dict):raise ValueError('ExplicitInputContractRequired')
    values={str(Path(args.assignment).resolve()):args.assignment_sha256,
        str(Path(args.case).resolve()):sha(args.case),str(Path(args.policy).resolve()):sha(args.policy)}
    if row.get('imported_source_path') is not None:
        values[row['imported_source_path']]=row['imported_source_sha256']
    return values

def copy_sources(packet,selector,payload,owner):
    # Operation outputs are data. The same reviewed Python entry handles every case.
    write_new(packet/'PYTHON_IDENTITY.json',(V4/'runtime/FINITE_PYTHON_IDENTITY.json').read_bytes())

def paths_and_bounds(args):
    case, policy, packet, output = (getattr(args, n).resolve() for n in
                                   ('case', 'policy', 'request_dir', 'output'))
    if packet.exists() or output.exists():
        raise ValueError('FreshRequestAndOutputRequired')
    if not packet.parent.is_dir() or packet/'live01' not in output.parents:
        raise ValueError('ExistingParentAndContainedOutputRequired')
    if packet in case.parents or packet in policy.parents:
        raise ValueError('InputsMustPreexistOutsideRequest')
    if type(args.maximum_faces) is not int or args.maximum_faces<=0:
        raise ValueError('ExplicitPositiveIntegerCapacityRequired')
    if (not all(math.isfinite(x) and x > 0 for x in
                (args.payload_seconds, args.owner_seconds))
            or args.payload_seconds > 14400 or args.owner_seconds > 14700
            or args.owner_seconds < args.payload_seconds+60):
        raise ValueError('ExplicitBoundedPayloadAndCleanupReserveRequired')
    return case, policy, packet, output

def prepare(args):
    case_path, policy_path, packet, output = paths_and_bounds(args)
    with source_scope(args.source_manifest_sha256,input_pins(args)) as (d, pins):
        import standalone_case as case_owner
        from sdf_cfd_mesher_v4.common.authority import identity
        from sdf_cfd_mesher_v4.runtime import frozen_imports
        assignment_path=args.assignment.resolve()
        with d.guard(assignment_path), d.guard(policy_path), case_owner.hold_inputs(case_path):
            if (pins.get(str(assignment_path))!=args.assignment_sha256
                    or sha(assignment_path)!=args.assignment_sha256):
                raise ValueError('ChangedReviewedCapacityAssignment')
            assignment=read(assignment_path)
            import stage_observations as obs
            selected_timing=obs.choice(assignment)
            if getattr(args,'stage_timing',None)!=selected_timing:raise ValueError('StageTimingArgumentAssignmentMismatch')
            capacity=assignment_capacity(assignment)
            assignment_intent(assignment)
            diagnostic=observer_flag(assignment)
            if type(args.upstream_forecast_diagnostics) is not bool or args.upstream_forecast_diagnostics is not diagnostic:
                raise ValueError('ObserverArgumentAssignmentMismatch')
            if (args.payload_seconds!=assignment['proposed_live_payload_seconds']
                    or args.owner_seconds!=assignment['proposed_live_owner_seconds']):
                raise ValueError('ExactAssignmentProposalBoundsRequired')
            if args.maximum_faces!=capacity:
                raise ValueError('CapacityArgumentAssignmentMismatch')
            row=assignment.get('input_contracts',{}).get(args.input_contract)
            keys={'case_path','case_sha256','imported_source_path','imported_source_sha256','policy_path','policy_sha256'}
            if (not isinstance(row,dict) or set(row)!=keys
                    or row['case_path']!=str(case_path) or row['policy_path']!=str(policy_path)
                    or row['case_sha256']!=sha(case_path) or row['policy_sha256']!=sha(policy_path)
                    or (row['imported_source_path'] is None)!=(row['imported_source_sha256'] is None)):
                raise ValueError('ExactAssignmentInputTupleRequired')
            case, increments, original = case_owner.load(case_path)
            flags=producer_flags(assignment,case,increments)
            mode=producer_mode(assignment)
            from finite_research.execution_target import prepared as prepared_target
            target=prepared_target(assignment,getattr(args,'execution_target',None))
            verification=prepared_fluid_choice(assignment,getattr(args,'transition_fluid_verification',None))
            if case['resources'] != dict(minimum_free_bytes=1000000000, output_limit_bytes=4000000000):
                raise ValueError('CurrentExplicitStoragePolicyRequired')
            policy = read(policy_path)
            policy_hash = sha(policy_path)
            if policy.get('backend_method') != 'HXT4DirectSizing095/1':
                raise ValueError('UnsupportedFiniteCollarBackend')
            aliases = ('finite_collar_stage_adapter', 'sdf_cfd_mesher_v4._frozen_finite_collar_stage_adapter')
            if any(name in sys.modules for name in aliases):
                raise ValueError('FreshPreparationStageRequired')
            try:
                stage = frozen_imports._load('finite_collar_stage_adapter', TEMPLATE/'stage.py')
                with stage.selected_helpers(pins) as (helpers, require_active):
                    helpers['finite_stage_background'].validate_policy(policy)
                    helpers['finite_stage_background'].validate_execution_target(policy, flags['transition_only'],execution_target=target)
                    transition = policy['tet_transition']
                    if (transition['schema'] not in ('V4TetLayerAdapter/5','V4TetLayerAdapter/6','V4TetLayerAdapter/7')
                            or not preparation_allowance(transition['seconds'],assignment['proposed_transition_seconds'],args.payload_seconds)):
                        raise ValueError('ExplicitBoundedFiniteTransitionRequired')
                    value = dict(schema='FiniteCollarDownstreamRequest/1',
                        upstream_case_sha256=original, upstream_effective_identity=identity(case),
                        upstream_geometry_identity=identity(case['geometry']),
                        upstream_flags=flags,
                        maximum_faces=args.maximum_faces, downstream_policy=policy,
                        downstream_policy_identity=identity(policy), live_execution_authorized=False)
                    if mode=='source_directions':
                        value.update(schema='FiniteCollarDownstreamRequest/3',prism_producer_mode=mode,
                            prism_schedule_identity=identity(case['prisms']),transition_fluid_verification=verification)
                    if 'execution_target' in assignment:value.update(schema='FiniteCollarDownstreamRequest/4',execution_target=target)
                    from finite_research.flexible_controls import selected as flexible,identities
                    if flexible(transition):
                        if mode!='source_directions' or 'execution_target' not in assignment:
                            raise ValueError('FlexibleTransitionRequiresExplicitIndependentTarget')
                        value.update(schema='FiniteCollarDownstreamRequest/5',**identities(transition))
                    if selected_timing is not None:value['stage_timing']=selected_timing
                    stage.validate_case(value, case, original,increments)
                    require_active()
                imported = case_owner.imported_geometry(case_path, case_owner.read(case_path))
                if ((str(imported[0]) if imported else None)!=row['imported_source_path']
                        or case_owner.held_import_sha256(case_path)!=row['imported_source_sha256']):
                    raise ValueError('ChangedAssignmentImportedSource')
                extra = {str(case_path): original, str(policy_path): policy_hash,
                         str(assignment_path):args.assignment_sha256}
                if imported is not None:
                    extra[str(imported[0])] = case_owner.held_import_sha256(case_path)
                packet.mkdir()  # Exclusive new directory; no overwrite or cleanup on failure.
                write_new(packet/'STAGE_REQUEST.json', encoded(value))
                selector = sha(packet/'STAGE_REQUEST.json')
                copy_sources(packet, selector, args.payload_seconds, args.owner_seconds)
                arguments = ['--finite-collar-selection', selector, '--case', str(case_path),
                    '--from-layer', '0', '--through-layer', str(len(increments)), '--mode', 'fastprod',
                    *(['--stop-after-transition'] if flags['transition_only'] else []), '--output', str(output), '--minimum-free-bytes',
                    '1000000000', '--surface-carrier', 'nodal', '--surface-assurance', 'numerical']
                if diagnostic:arguments.append('--upstream-forecast-diagnostics')
                if mode=='source_directions':arguments+=['--prism-producer-mode',mode]
                request = dict(schema='DisabledLiveRequest/1', case=str(case_path),
                    upstream_forecast_diagnostics=diagnostic,
                    assignment_task=str(assignment_path), assignment_task_sha256=args.assignment_sha256,
                    input_contract_id=args.input_contract, live_payload_seconds=0, live_owner_seconds=0,
                    case_sha256=original, output=str(output), output_must_not_exist=True,
                    entry=str(HERE/'run.py'), execution_authorized=False, selector=selector,
                    stage_request=str(packet/'STAGE_REQUEST.json'), stage_request_sha256=selector,
                    arguments=arguments, proposed_payload_seconds=args.payload_seconds,
                    proposed_owner_seconds=args.owner_seconds, transition_stage_seconds=transition['seconds'],
                    original_policy_path=str(policy_path), original_policy_sha256=policy_hash,
                    preparation_manifest_sha256=args.source_manifest_sha256,
                    source_root=str(V4),source_release_sha256=args.source_manifest_sha256,
                    imported_source_sha256=case_owner.held_import_sha256(case_path),
                    scope='Prepared research inputs only; numerical entry always denies')
                if mode=='source_directions':
                    request.update(prism_producer_mode=mode,transition_fluid_verification=verification)
                if 'execution_target' in assignment:request['execution_target']=target
                if flexible(transition):request.update(**identities(transition))
                if selected_timing is not None:request['stage_timing']=selected_timing
                write_new(packet/'LIVE_REQUEST.json', encoded(request))
                selected = dict(pins, **extra)
                for name in (*FILES, 'STAGE_REQUEST.json', 'LIVE_REQUEST.json'):
                    selected[str(packet/name)] = sha(packet/name)
                write_new(packet/'PINNED_SOURCES.json', encoded(selected))
                if sha(case_path) != original or sha(policy_path) != policy_hash:
                    raise ValueError('ChangedOriginalInputs')
                return dict(request_dir=str(packet), manifest_sha256=sha(packet/'PINNED_SOURCES.json'),
                            selector=selector, execution_authorized=False)
            finally:
                for name in aliases:
                    sys.modules.pop(name, None)

def verify_prepared(packet, expected_manifest):
    """Read-only prepared-input check. Cannot issue launch authority."""
    packet = Path(packet).resolve()
    if any((packet/name).exists() for name in FORBIDDEN):
        raise ValueError('UnexpectedOrConsumedExecutionAuthority')
    if sha(packet/'PINNED_SOURCES.json') != expected_manifest:
        raise ValueError('ChangedPreparedManifest')
    for path, digest in read(packet/'PINNED_SOURCES.json').items():
        if sha(path) != digest:
            raise ValueError('ChangedPreparedSourceOrInput', path)
    value = read(packet/'LIVE_REQUEST.json')
    if (value['source_root']!=str(V4) or value['source_release_sha256']!=sha(V4/'runtime/RELEASE.json')
        or value['entry']!=str(HERE/'run.py') or value['stage_request']!=str(packet/'STAGE_REQUEST.json')
        or value['stage_request_sha256']!=sha(packet/'STAGE_REQUEST.json')):
        raise ValueError('ChangedPreparedOwnedRoot')
    selected=read(packet/'PINNED_SOURCES.json')
    for name in ('LIVE_REQUEST.json','STAGE_REQUEST.json','PYTHON_IDENTITY.json'):
        if selected.get(str(packet/name))!=sha(packet/name):raise ValueError('MissingPreparedDataBinding',name)
    if (value['execution_authorized'] is not False or Path(value['output']).exists()
        or packet/'live01' not in Path(value['output']).resolve().parents):
        raise ValueError('DisabledFreshOutputRequired')
    if sha(packet/'STAGE_REQUEST.json') != value['selector']:
        raise ValueError('ChangedPreparedSelection')
    selected=read(packet/'PINNED_SOURCES.json')
    assignment_path=Path(value['assignment_task'])
    if (selected.get(str(assignment_path))!=value['assignment_task_sha256']
            or sha(assignment_path)!=value['assignment_task_sha256']):
        raise ValueError('ChangedPreparedCapacityAssignment')
    stage=read(packet/'STAGE_REQUEST.json')
    assignment=read(assignment_path)
    assignment_intent(assignment)
    capacity=assignment_capacity(assignment)
    if type(stage['maximum_faces']) is not int or stage['maximum_faces']!=capacity:
        raise ValueError('PreparedCapacityAssignmentMismatch')
    validate_observer_binding(assignment,value,stage)
    # Re-derive the endpoint and producer identity from the held original case.
    # A rehashed request is not authority to alter its caller's prism schedule.
    if producer_mode(assignment)=='source_directions':
        import standalone_case as case_owner
        from finite_research.stage import request as read_stage,validate_case
        from immutable_state_access import guard
        from sdf_cfd_mesher_v4.common.authority import identity
        contract=select_input_contract(assignment,value['input_contract_id'])
        with case_owner.hold_inputs(Path(contract['case_path'])),guard(contract['policy_path']):
            case,increments,original=case_owner.load(Path(contract['case_path']))
            stage=read_stage(packet/'STAGE_REQUEST.json',selected)
            validate_case(stage,case,original,increments)
            if stage['upstream_flags']!=producer_flags(assignment,case,increments):
                raise ValueError('ChangedPreparedPrismSchedule')
            imported=case_owner.imported_geometry(contract['case_path'],case_owner.read(contract['case_path']))
            canonical=dict(case_sha256=original,effective_case_identity=identity(case),
                geometry_identity=identity(case['geometry']),metres_per_model_unit=case['geometry']['metres_per_model_unit'],
                imported_source_path=str(imported[0]) if imported else None,
                imported_source_sha256=case_owner.held_import_sha256(contract['case_path']),
                policy_sha256=sha(contract['policy_path']),policy_identity=identity(read(contract['policy_path'])),
                prism_schedule_identity=identity(case['prisms']))
            validate_bound_inputs(contract,value,stage,canonical,capacity)
    return value


def verify_activation(packet,expected_manifest):
    packet=Path(packet).resolve()
    if sha(packet/'PINNED_SOURCES.json')!=expected_manifest:raise ValueError('ChangedActivatedManifest')
    selected=read(packet/'PINNED_SOURCES.json')
    for name,digest in selected.items():
        if sha(name)!=digest:raise ValueError('ChangedActivatedSourceOrInput',name)
    binding=read(packet/'ACTIVATION.json');planned=read(packet/'LIVE_REQUEST.json')
    prepared=Path(binding['prepared_request'])
    if binding.get('schema')!='OwnedFiniteDerivative/1' or binding['source_root']!=str(V4):
        raise ValueError('ChangedOwnedSourceRoot')
    if (sha(V4/'runtime/RELEASE.json')!=binding['release_sha256']
        or sha(prepared/'PINNED_SOURCES.json')!=binding['prepared_manifest_sha256']
        or sha(prepared/'LIVE_REQUEST.json')!=binding['prepared_request_sha256']
        or sha(packet/'TASK.json')!=binding['task_sha256']
        or sha(packet/'LIVE_REQUEST.json')!=binding['live_request_sha256']
        or sha(packet/'STAGE_REQUEST.json')!=binding['selector']
        or sha(binding['assignment_task'])!=binding['assignment_task_sha256']):
        raise ValueError('ChangedPreparedActivationBinding')
    if any((prepared/name).exists() for name in FORBIDDEN):raise ValueError('OriginalPreparationAuthorityChanged')
    if (planned['entry']!=str(HERE/'run.py') or planned['source_root']!=str(V4)
        or planned['selector']!=binding['selector'] or planned['stage_request_sha256']!=binding['selector']
        or planned['stage_request']!=str(packet/'STAGE_REQUEST.json')
        or planned['arguments'][:2]!=['--finite-collar-selection',binding['selector']]
        or planned['execution_authorized'] is not False
        or packet/'live01' not in Path(planned['output']).resolve().parents):raise ValueError('ChangedActivationSelection')
    assignment=read(binding['assignment_task']);capacity=assignment_capacity(assignment);permitted=assignment_intent(assignment)
    contract=select_input_contract(assignment,binding['input_contract_id'])
    if (contract!=binding['input_contract'] or capacity!=binding['maximum_faces']
        or binding['source_entry']!=str(HERE/'run.py') or binding['source_entry_sha256']!=sha(HERE/'run.py')
        or binding['generated_python_files']!=0):raise ValueError('ChangedOwnedDerivativeContract')
    for name in ('ACTIVATION.json','TASK.json','LIVE_REQUEST.json','STAGE_REQUEST.json','PYTHON_IDENTITY.json'):
        if selected.get(str(packet/name))!=sha(packet/name):raise ValueError('MissingDerivativeDataBinding',name)
    original=verify_prepared(prepared,binding['prepared_manifest_sha256'])
    validate_bound_inputs(contract,original,read(prepared/'STAGE_REQUEST.json'),binding['canonical_inputs'],capacity)
    if observer_flag(binding) is not observer_flag(assignment):
        raise ValueError('ChangedDerivativeObserverBinding')
    for request in (original,planned):
        validate_observer_binding(assignment,request,read(Path(request['stage_request'])))
        if (request['assignment_task']!=binding['assignment_task']
            or request['assignment_task_sha256']!=binding['assignment_task_sha256']
            or request['input_contract_id']!=binding['input_contract_id']
            or request['source_root']!=str(V4) or request['source_release_sha256']!=binding['release_sha256']):
            raise ValueError('ChangedDerivativeAssignmentBinding')
    validate_bound_inputs(contract,planned,read(packet/'STAGE_REQUEST.json'),binding['canonical_inputs'],capacity)
    task=read(packet/'TASK.json')
    if (task!=live_task_from_assignment(assignment)
        or read(packet/'STAGE_REQUEST.json')['live_execution_authorized'] is not permitted
        or original.get('live_payload_seconds')!=0 or original.get('live_owner_seconds')!=0
        or planned.get('live_payload_seconds')!=task['payload_seconds']
        or planned.get('live_owner_seconds')!=task['owner_seconds']
        or selected.get(binding['assignment_task'])!=binding['assignment_task_sha256']):
        raise ValueError('ChangedActivatedTask')
    return planned

def derive(args):
    prepared=Path(args.prepared).resolve();destination=Path(args.request_dir).resolve();output=Path(args.output).resolve()
    if destination.exists() or output.exists() or destination/'live01' not in output.parents or not destination.parent.is_dir():
        raise ValueError('FreshContainedDerivativeRequired')
    assignment_path=Path(args.assignment).resolve()
    assignment=read(assignment_path)
    if sha(assignment_path)!=args.assignment_sha256:raise ValueError('ChangedDerivativeAssignment')
    contract=select_input_contract(assignment,args.input_contract)
    extra=read(prepared/'PINNED_SOURCES.json')
    extra[str(assignment_path)]=args.assignment_sha256
    extra.update({str(prepared/name):sha(prepared/name) for name in ('PINNED_SOURCES.json','LIVE_REQUEST.json','STAGE_REQUEST.json','PYTHON_IDENTITY.json')})
    with source_scope(args.source_manifest_sha256,extra) as (d,pins):
        prior=verify_prepared(prepared,args.prepared_manifest_sha256)
        if (prior['assignment_task']!=str(assignment_path)
            or prior['assignment_task_sha256']!=args.assignment_sha256
            or prior['input_contract_id']!=args.input_contract):raise ValueError('PreparedAssignmentMismatch')
        if prior['source_root']!=str(V4) or prior['source_release_sha256']!=args.source_manifest_sha256:
            raise ValueError('ChangedPreparedPackageRoot')
        assignment_intent(assignment);capacity=assignment_capacity(assignment)
        import standalone_case as case_owner
        with case_owner.hold_inputs(Path(contract['case_path'])),d.guard(contract['policy_path']):
            case,increments,original=case_owner.load(Path(contract['case_path']))
            from sdf_cfd_mesher_v4.common.authority import identity
            flags=producer_flags(assignment,case,increments)
            mode=producer_mode(assignment)
            from finite_research.execution_target import prepared as prepared_target
            target=prepared_target(assignment,getattr(args,'execution_target',None))
            policy=read(contract['policy_path'])
            if (policy['backend_method']!='HXT4DirectSizing095/1'
                or policy['tet_transition']['schema'] not in ('V4TetLayerAdapter/5','V4TetLayerAdapter/6','V4TetLayerAdapter/7')
                or policy['tet_transition']['seconds']!=transition_allowance(assignment['proposed_transition_seconds'])
                or (mode=='coupled' and {k:v for k,v in policy.items() if k!='tet_transition'}
                   !={k:v for k,v in case['background'].items() if k!='tet_transition'})):
                raise ValueError('ChangedNonTransitionBackgroundPolicyOrCapability')
            imported=case_owner.imported_geometry(contract['case_path'],case_owner.read(contract['case_path']))
            canonical=dict(case_sha256=original,effective_case_identity=identity(case),
                geometry_identity=identity(case['geometry']),metres_per_model_unit=case['geometry']['metres_per_model_unit'],
                imported_source_path=str(imported[0]) if imported else None,
                imported_source_sha256=case_owner.held_import_sha256(contract['case_path']),
                policy_sha256=sha(contract['policy_path']),policy_identity=identity(policy))
            if mode=='source_directions':canonical['prism_schedule_identity']=identity(case['prisms'])
            stage=read(prepared/'STAGE_REQUEST.json')
            if stage['upstream_flags']!=flags:raise ValueError('ChangedPreparedPrismSchedule')
            validate_bound_inputs(contract,prior,stage,canonical,capacity)
            stage['live_execution_authorized']=assignment_intent(assignment)
            destination.mkdir()
            write(destination/'STAGE_REQUEST.json',stage);selector=sha(destination/'STAGE_REQUEST.json')
            planned=dict(prior,output=str(output),selector=selector,stage_request=str(destination/'STAGE_REQUEST.json'),stage_request_sha256=selector,
                assignment_task=str(assignment_path),assignment_task_sha256=args.assignment_sha256,input_contract_id=args.input_contract,
                live_payload_seconds=assignment['live_payload_seconds'],live_owner_seconds=assignment['live_owner_seconds'])
            planned['arguments']=list(prior['arguments']);planned['arguments'][1]=selector;planned['arguments'][planned['arguments'].index('--output')+1]=str(output)
            write(destination/'LIVE_REQUEST.json',planned);write(destination/'TASK.json',live_task_from_assignment(assignment))
            write_new(destination/'PYTHON_IDENTITY.json',(V4/'runtime/FINITE_PYTHON_IDENTITY.json').read_bytes())
            binding=dict(schema='OwnedFiniteDerivative/1',source_root=str(V4),release_sha256=args.source_manifest_sha256,
                prepared_request=str(prepared),prepared_manifest_sha256=sha(prepared/'PINNED_SOURCES.json'),prepared_request_sha256=sha(prepared/'LIVE_REQUEST.json'),
                task_sha256=sha(destination/'TASK.json'),live_request_sha256=sha(destination/'LIVE_REQUEST.json'),selector=selector,
                assignment_task=str(assignment_path),assignment_task_sha256=args.assignment_sha256,input_contract_id=args.input_contract,
                input_contract=contract,canonical_inputs=canonical,maximum_faces=capacity,
                upstream_forecast_diagnostics=observer_flag(assignment),
                source_entry=str(HERE/'run.py'),source_entry_sha256=sha(HERE/'run.py'),generated_python_files=0)
            write(destination/'ACTIVATION.json',binding)
            from finite_research.dependency_inputs import union_checked
            selected=union_checked(pins,read(prepared/'PINNED_SOURCES.json'))
            for name in ('STAGE_REQUEST.json','LIVE_REQUEST.json','TASK.json','PYTHON_IDENTITY.json','ACTIVATION.json'):
                selected[str(destination/name)]=sha(destination/name)
            write(destination/'PINNED_SOURCES.json',selected)
            verify_activation(destination,sha(destination/'PINNED_SOURCES.json'))
            return dict(request_dir=str(destination),manifest_sha256=sha(destination/'PINNED_SOURCES.json'),selector=selector,
                execution_authorized=False,generated_python_files=0)

def inspect_request(args):
    packet=Path(args.request_dir).resolve()
    with source_scope(args.source_manifest_sha256,dict(read(packet/'PINNED_SOURCES.json'),**{str(packet/'PINNED_SOURCES.json'):args.manifest_sha256})):
        if (packet/'ACTIVATION.json').exists():value=verify_activation(packet,args.manifest_sha256)
        else:value=verify_prepared(packet,args.manifest_sha256)
        if value['source_root']!=str(V4) or value['source_release_sha256']!=args.source_manifest_sha256:
            raise ValueError('ChangedRequestSourceRoot')
        return dict(schema='OwnedFiniteInspection/1',passed=True,request=str(packet),selector=value['selector'],
            case=value['case'],output=value['output'],execution_authorized=False,source_root=str(V4),
            upstream_forecast_diagnostics=observer_flag(value),
            prism_producer_mode=producer_mode(value),
            transition_fluid_verification=fluid_choice(value),
            **({name:value[name] for name in ('extent_policy_identity','quality_targets_identity','controls_identity')}
                if 'extent_policy_identity' in value else {}),
            **({'constructor_schema':value['constructor_schema']} if 'constructor_schema' in value else {}),
            execution_target=__import__('finite_research.execution_target',fromlist=['choice']).choice(value),
            stage_timing=__import__('stage_observations').choice(value),
            through_layer=read(packet/'STAGE_REQUEST.json')['upstream_flags']['through_layer'],
            proposed_payload_seconds=value['proposed_payload_seconds'],proposed_owner_seconds=value['proposed_owner_seconds'],
            transition_stage_seconds=value['transition_stage_seconds'],generated_python_files=0)
