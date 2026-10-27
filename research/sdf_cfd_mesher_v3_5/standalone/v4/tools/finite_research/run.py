"""Task48 reviewed continuation: one fresh live command, no retry or install."""
import time
_ENTRY_STARTED=time.perf_counter()
_ENTRY_CPU=time.process_time()
from pathlib import Path
from types import SimpleNamespace
from contextlib import ExitStack, contextmanager
import hashlib, json, os, sys, time, traceback
from datetime import datetime, timezone, timedelta

CODE=Path(__file__).resolve().parent
V4=CODE.parents[1]
S=V4/'tools/finite_support'
HERE=None  # Explicit per-operation data root; never generated source.
sys.path[:0]=[str(S),str(V4/'tools'),str(V4/'src')]
import stage_observations as obs
from finite_research.input_contract import assignment_intent,live_task_from_assignment,assignment_capacity,select_input_contract,validate_bound_inputs,transition_allowance

def verify_activation():
    if read(HERE/'LIVE_REQUEST.json').get('execution_target')=='direct_complete':
        from finite_research.direct_activation import verify
        return verify(HERE,sha(HERE/'PINNED_SOURCES.json'))
    from finite_research.inputs import verify_activation as verify
    return verify(HERE,sha(HERE/'PINNED_SOURCES.json'))

sys.dont_write_bytecode = True
ROLES = ('launch', 'coordinator', 'worker')
PROPOSED_PAYLOAD_SECONDS = 14400.0
PROPOSED_OWNER_SECONDS = 14700.0


@obs.observed('role.gate','source_lock')
def gate(role):
    """Fresh prepared derivative under current task, exact source and independent review."""
    if not (HERE/'AUTHORIZATION.json').is_file():
        raise ValueError('MissingSeparateLiveGrant',role)
    if read(HERE/'LIVE_REQUEST.json').get('execution_target')=='direct_complete':
        from finite_research.direct_activation import gate as direct_gate
        return direct_gate(HERE,role)
    task=read(HERE/'TASK.json')
    if task.get('live_execution_authorized') is not True:
        raise ValueError('NumericalExecutionDeniedByAssignment', role)
    now=datetime.now(timezone.utc)
    start=datetime.fromisoformat(task['start_utc']);end=datetime.fromisoformat(task['deadline_utc'])
    cutoff=datetime.fromisoformat(task['launch_cutoff_utc'])
    if (role not in ROLES or task.get('schema')!='PreparedFiniteLiveTask/1'
            or task.get('live_execution_authorized') is not True or task.get('maximum_invocations')!=1
            or task.get('payload_seconds')!=14400 or task.get('owner_seconds')!=14700
            or task.get('transition_seconds')!=transition_allowance(task.get('proposed_transition_seconds')) or end-start!=timedelta(minutes=120)
            or cutoff-start!=timedelta(minutes=40) or not start<=now<end
            or role=='launch' and now>=cutoff):
        raise ValueError('ExactFreshLiveWindowRequired')
    verify_activation()
    grant=read(HERE/'AUTHORIZATION.json');review=read(HERE/'LAUNCH_REVIEW.json')
    if (grant.get('approved') is not True or grant.get('mode')!='fresh_live_once'
            or grant.get('human_message')!=task['human_message']
            or grant.get('assignment')!=task['assignment']
            or grant.get('activation_sha256')!=sha(HERE/'ACTIVATION.json')
            or task.get('authorization_scope')!='PreparedResearchSeparatelyBoundedReviewedContinuation'
            or grant.get('maximum_invocations')!=1 or grant.get('payload_seconds')!=14400
            or grant.get('owner_seconds')!=14700 or grant.get('transition_seconds')!=task['transition_seconds']
            or grant.get('task_sha256')!=sha(HERE/'TASK.json')
            or grant.get('request_sha256')!=sha(HERE/'LIVE_REQUEST.json')
            or grant.get('manifest_sha256')!=sha(HERE/'PINNED_SOURCES.json')
            or review.get('approved') is not True or review.get('mode')!='fresh_live_once'
            or review.get('authorization_sha256')!=sha(HERE/'AUTHORIZATION.json')
            or review.get('task_sha256')!=sha(HERE/'TASK.json')
            or review.get('manifest_sha256')!=sha(HERE/'PINNED_SOURCES.json')):
        raise ValueError('ExactHumanLiveGrantAndReviewRequired')
    from finite_research import qualification
    qualification.verify()
    if (review.get('qualification_index_sha256')!=sha(V4/'runtime/finite_qualification/INDEX.json')
        or review.get('source_release_sha256')!=sha(V4/'runtime/RELEASE.json')
        or grant.get('source_release_sha256')!=sha(V4/'runtime/RELEASE.json')):
        raise ValueError('CurrentOwnedQualificationAndReleaseRequired')
    return task
















def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


_selected_owners = None


@contextmanager
def resource_scope(role):
    """One fixed native bundle before fixture imports; profile after imports."""
    global _selected_owners
    if _selected_owners is not None:raise ValueError('FreshResourceScopeRequired')
    sys.path[:0]=[str(S),str(V4/'tools'),str(V4/'src')]
    from immutable_state_access import guard
    with ExitStack() as held:
        held.enter_context(obs.span('resource_scope_lifetime','owner_lifetime'))
        held.enter_context(guard(HERE/'PINNED_SOURCES.json'))
        gate(role)
        selected=read(HERE/'PINNED_SOURCES.json')
        for path,digest in selected.items():
            held.enter_context(guard(path))
            if sha(path)!=digest:raise ValueError('ChangedOwnedLiveInput',path)
        from finite_research.inputs import package_sources
        if any(selected.get(path)!=digest for path,digest in package_sources().items()):
            raise ValueError('MissingOwnedLiveReleaseSource')
        helper_path=CODE/'owner_selection.py'
        held.enter_context(guard(helper_path))
        if selected.get(str(helper_path))!=sha(helper_path):raise ValueError('ChangedFixedOwnerSelector')
        from finite_research import owner_selection
        if Path(owner_selection.__file__).resolve()!=helper_path:raise ValueError('WrongOwnerSelectorOrigin')
        with owner_selection.selected_native(selected) as (native,attest):
            import fixture_study as d
            attest()
            from sdf_cfd_mesher_v4.runtime import frozen_imports
            profile_dir=CODE/'owner/resource_profile_16gib_3600'
            selector_path=profile_dir/'selection.py';held.enter_context(d.guard(selector_path))
            if selected.get(str(selector_path))!=sha(selector_path):raise ValueError('ChangedForwardProfileSelector')
            selector=frozen_imports._load('finite_forward_resource_selection',selector_path)
            with selector.selected(selected) as (resources,profile):
                if profile['maximum_whole_seconds']!=14700:raise ValueError('ChangedForwardWholeRunCeiling')
                attest();_selected_owners=(d,native,resources)
                try:yield
                finally:_selected_owners=None


def owners():
    if _selected_owners is None:
        raise ValueError('SelectedResourceLifetimeRequired')
    return _selected_owners


def check_clock(deadline):
    clock = read(HERE/'CLOCK.json')
    if deadline != clock['deadline'] or time.perf_counter() >= deadline:
        raise TimeoutError('CommandControlSharedDeadline')


@obs.observed('role.pins','source_lock')
def pins():
    selected = read(HERE/'live01_INPUT_PINS.json')
    for path, digest in selected.items():
        if sha(path) != digest:
            raise ValueError('ChangedCommandControlSource', path)
    selected[str(HERE/'live01_INPUT_PINS.json')] = sha(HERE/'live01_INPUT_PINS.json')
    return selected


@obs.observed('role.closure','source_lock')
def closure(d, folder, selected, observed):
    modules = dict(observed)
    modules.update({k: str(Path(m.__file__).resolve()) for k, m in list(sys.modules.items())
                    if getattr(m, '__file__', None) and Path(m.__file__).suffix.lower() in ('.py','.pyd','.dll')
                    and Path(m.__file__).is_file()})
    modules['explicit_control_entry'] = str(CODE/'run.py')
    missing = sorted(set(modules.values())-set(selected))
    files = {p: sha(p) for p in set(modules.values())}
    d.publish(folder/'LOADED_SOURCE_CLOSURE.json', dict(modules=modules, files=files, unpinned=missing))
    if missing or any(selected[p] != h for p, h in files.items()):
        raise ValueError('CommandControlLoadedSourceClosureFailed',len(missing),dict(path=str(folder/'LOADED_SOURCE_CLOSURE.json'),sha256=sha(folder/'LOADED_SOURCE_CLOSURE.json')))


@obs.observed('role.finish','role')
def finish(d, folder, selected, observed, result, started, deadline):
    try:
        closure(d, folder, selected, observed)
        pins(); check_clock(deadline)
    except BaseException as error:
        result.update(passed=False, closure_error=repr(error))
    result.update(seconds=time.perf_counter()-started, shared_deadline=deadline,
                  acceptance_owner='Existing mesh/transition reports; process closure is not scientific authority', pid=os.getpid(),
                  parent_pid=os.getppid(), argv=sys.argv[:])
    try:
        d.publish(folder/'RESULT.json', result)
        completion = dict(passed=result['passed'], result_sha256=sha(folder/'RESULT.json'),
                          source_closure_sha256=sha(folder/'LOADED_SOURCE_CLOSURE.json'),
                          startup_identity_sha256=sha(folder/'STARTUP_IDENTITY.json'))
        d.publish(folder/'WORKER_COMPLETION.json', completion)
        check_clock(deadline)
        return 0 if completion['passed'] else 1
    except BaseException as error:
        # Preserve any primary failure even when evidence publication itself fails.
        print(json.dumps(dict(primary=result, publication_or_deadline_error=repr(error))), file=sys.stderr, flush=True)
        return 1


@obs.observed('role.child_record','role')
def child_record(folder, expected_code, actual_code):
    completion = read(folder/'WORKER_COMPLETION.json')
    result = read(folder/'RESULT.json')
    source = read(folder/'LOADED_SOURCE_CLOSURE.json')
    selected = pins()
    if (actual_code != expected_code or not result['passed'] or not completion['passed']
            or source['unpinned'] or completion['result_sha256'] != sha(folder/'RESULT.json')
            or completion['source_closure_sha256'] != sha(folder/'LOADED_SOURCE_CLOSURE.json')
            or completion['startup_identity_sha256'] != sha(folder/'STARTUP_IDENTITY.json')
            or any(selected.get(p) != h or sha(p) != h for p, h in source['files'].items())):
        raise ValueError('CommandControlChildFailed', str(folder), actual_code, dict(result_sha256=sha(folder/'RESULT.json'),completion_sha256=sha(folder/'WORKER_COMPLETION.json')))
    return dict(result_sha256=sha(folder/'RESULT.json'),
                completion_sha256=sha(folder/'WORKER_COMPLETION.json'),
                startup_identity_sha256=sha(folder/'STARTUP_IDENTITY.json'),
                source_closure_sha256=sha(folder/'LOADED_SOURCE_CLOSURE.json'))


def python_identity():
    return read(HERE/'PYTHON_IDENTITY.json')


def validate_image(record, bindings):
    """Authenticate each role and its parent against predeclared file identities."""
    try:
        role = bindings['roles'][record['role']]
        expected = bindings[role['own']]
        child = bindings['child']
        path = expected['executable']
        if (record.get('observation_error') or
                not os.path.samefile(record['owner_executable'], child['executable']) or
                not os.path.samefile(record['os_image'], path) or
                record['image_sha256'] != expected['sha256'] or
                record['image_file_identity'] != expected['file_identity'] or
                sha(path) != expected['sha256']):
            raise ValueError('ImageOrOwnerMismatch')
        if role['explicit_argv0'] and (not record['os_command'] or
                not os.path.samefile(record['os_command'][0], path)):
            raise ValueError('UnboundExecutableArgv')
        if role['parent'] is None:
            if record['expected_parent_pid'] is not None:
                raise ValueError('UnexpectedControllerParentContract')
        else:
            parent = bindings[role['parent']]
            if (record['expected_parent_pid'] is None or
                    record['parent_pid'] != record['expected_parent_pid'] or
                    not os.path.samefile(record['parent_image'], parent['executable']) or
                    record['parent_image_sha256'] != parent['sha256'] or
                    record['parent_image_file_identity'] != parent['file_identity']):
                raise ValueError('ImageParentContinuityMismatch')
    except (KeyError, OSError, ValueError) as error:
        raise ValueError('RealOwnedPythonImageRequired', repr(error)) from error


def file_identity(path):
    stat = Path(path).stat()
    return dict(device=stat.st_dev, file_id=stat.st_ino, size=stat.st_size)


def record_image(d, resources, folder, role, bindings, parent_pid):
    """Publish the raw witness before any identity comparison can reject it."""
    expected = bindings[bindings['roles'][role]['own']]
    record = dict(role=role, pid=os.getpid(), parent_pid=os.getppid(),
                  expected_parent_pid=parent_pid, requested_argv=sys.argv[:],
                  sys_executable=sys.executable, base_prefix=sys.base_prefix,
                  expected=expected, role_contract=bindings['roles'][role],
                  expected_parent=None if bindings['roles'][role]['parent'] is None else bindings[bindings['roles'][role]['parent']],
                  identity_contract_sha256=sha(HERE/'PYTHON_IDENTITY.json'),
                  resource_owner=str(Path(resources.__file__).resolve()), resource_limits=resources.APPROVED,
                  native_owner=str(Path(sys.modules['native_scope'].__file__).resolve()),
                  native_here=str(sys.modules['native_scope'].HERE),wire_owner=str(Path(sys.modules['wire'].__file__).resolve()),
                  native_source_sha256=sha(sys.modules['native_scope'].__file__),
                  expected_resolved=str(Path(expected['executable']).resolve()))
    try:
        import psutil
        options = {}; resources.executable_options([sys.executable], options)
        image = psutil.Process().exe()
        record.update(owner_executable=options['executable'], executable_override=options,
            sys_executable_resolved=str(Path(sys.executable).resolve()),
            owner_executable_resolved=str(Path(options['executable']).resolve()),
            os_image=image, os_image_resolved=str(Path(image).resolve()),
            os_command=psutil.Process().cmdline(), image_sha256=sha(image),
            image_file_identity=file_identity(image))
        if parent_pid is not None:
            parent_image = psutil.Process(os.getppid()).exe()
            record.update(parent_image=parent_image, parent_image_sha256=sha(parent_image),
                          parent_image_file_identity=file_identity(parent_image))
    except BaseException as error:
        record['observation_error'] = repr(error)
    d.publish(folder/'STARTUP_IDENTITY.json', record)
    return record


@obs.observed('role.owned_host','role')
def owned_host(d, native, resources, folder, role, parent_pid):
    expected = python_identity()
    record = record_image(d, resources, folder, role, expected, parent_pid)
    native.require_transport()
    validate_image(record, expected)
    return record

def request():
    value = read(HERE/'LIVE_REQUEST.json')
    if value['selector'] != sha(HERE/'STAGE_REQUEST.json'):
        raise ValueError('ChangedPreparedLiveSelection')
    if sha(value['case']) != value['case_sha256']:
        raise ValueError('ChangedPreparedLiveInputs')
    return value


@obs.observed('role.selected_main','role')
def selected_main(d, argv, observed):
    """Common command with unchanged upstream owners and a terminal-only adapter."""
    from sdf_cfd_mesher_v4.runtime import frozen_imports
    connection = frozen_imports._load('finite_collar_live_connection', CODE/'connection.py')
    if argv[:2] != ['--finite-collar-selection', request()['selector']]:
        raise ValueError('ExplicitPinnedCandidateSelectionRequired')
    selected_pins = pins()
    terminal_scope = []
    with connection.selected_command(request_root=HERE,selector=request()['selector'],terminal_scope=terminal_scope) as (cli, stage):
        try:
            # The common parent still owns its parser/worker launch. Numerical
            # work only occurs in the existing worker under the exact live grant.
            with stage.clock_scope(HERE/'CLOCK.json', selected_pins, request_root=HERE) as check:
                manager = stage.terminal_dispatch(request()['case'], HERE/'STAGE_REQUEST.json', selected_pins, check, observed)
                terminal_scope.append(manager)
                try:
                    with manager:
                        return cli.main(argv[2:])
                finally:
                    terminal_scope.clear()
        finally:
            observed.update({name: str(Path(module.__file__).resolve())
                for name, module in list(sys.modules.items())
                if getattr(module, '__file__', None) and Path(module.__file__).suffix == '.py'
                and V4 in Path(module.__file__).resolve().parents})



@obs.observed('role.worker','role')
def worker(argv):
    gate('worker')  # Require exact grant before project owners.
    deadline = float(os.environ['V4_LIVE_PAYLOAD_DEADLINE'])
    check_clock(deadline)
    if '--worker' not in argv:
        raise ValueError('ActualCommonWorkerRequired')
    started = time.perf_counter(); d, native, resources = owners()
    folder = HERE/'live01'/'worker'; folder.mkdir()
    selected = read(HERE/'live01_INPUT_PINS.json'); observed = {}; result = dict(passed=False)
    try:
        owned_host(d, native, resources, folder, 'worker', int(os.environ['V4_STANDALONE_SUPERVISOR']))
        selected = pins(); request()
        code = selected_main(d, argv, observed)
        if code != 0:
            raise ValueError('LiveCommonWorkerDidNotReturnSuccess', code)
        check_clock(deadline)
        result.update(passed=True, common_returncode=code,
                      mesh_result=str(Path(request()['output'])/'RESULT.json'))
        if request().get('execution_target')=='direct_complete':
            name='DIRECT_SCOPE_EXIT.json' if request()['stop_after']=='complete' else 'STAGE_COMPLETION.json'
            result['direct_scope_exit_sha256']=sha(Path(request()['output'])/name)
    except BaseException as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
    return finish(d, folder, selected, observed, result, started, deadline)


@obs.observed('role.parent','role')
def parent(deadline, launch_pid, argv):
    gate('coordinator')  # Require exact grant before project owners.
    check_clock(deadline)
    if argv != request()['arguments']:
        raise ValueError('ChangedPreparedLiveArguments')
    started = time.perf_counter(); d, native, resources = owners()
    folder = HERE/'live01'; folder.mkdir()
    selected = read(HERE/'live01_INPUT_PINS.json'); observed = {}; result = dict(passed=False)
    previous = os.environ.get('V4_LIVE_PAYLOAD_DEADLINE')
    try:
        owned_host(d, native, resources, folder, 'coordinator', launch_pid)
        selected = pins(); os.environ['V4_LIVE_PAYLOAD_DEADLINE'] = repr(deadline)
        code = selected_main(d, argv, observed)
        if code != 0:
            raise ValueError('LiveCommonParentDidNotReturnSuccess', code)
        child = child_record(folder/'worker', 0, code)
        check_clock(deadline)
        result.update(passed=True, common_returncode=code, worker=child,
                      mesh_result=str(Path(request()['output'])/'RESULT.json'))
    except BaseException as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
    finally:
        if previous is None: os.environ.pop('V4_LIVE_PAYLOAD_DEADLINE', None)
        else: os.environ['V4_LIVE_PAYLOAD_DEADLINE'] = previous
    return finish(d, folder, selected, observed, result, started, deadline)


@obs.observed('role.launch','role')
def launch(argv):
    task=gate('launch')
    if datetime.now(timezone.utc)+timedelta(seconds=14700)>=datetime.fromisoformat(task['deadline_utc']):
        raise ValueError('InsufficientLiveMilestone')
    planned = request()
    if argv != planned['arguments'] or Path(planned['output']).exists():
        raise ValueError('ExactPreparedArgumentsAndNewOutputRequired')
    if (HERE/'CLOCK.json').exists() or (HERE/'live01_INPUT_PINS.json').exists():
        raise ValueError('LiveInvocationConsumed')
    d, native, resources = owners()
    selected = read(HERE/'PINNED_SOURCES.json')
    # Qualification archive is already covered by the current owned release.
    for path, digest in selected.items():
        if sha(path) != digest: raise ValueError('ChangedPreparedLiveSource', path)
    # Proposed global payload clock; no new owner or independent stage allowance.
    import psutil
    process_age=max(0.,time.time()-psutil.Process().create_time())
    now=time.perf_counter()
    deadline=min(_ENTRY_STARTED+PROPOSED_PAYLOAD_SECONDS,now+PROPOSED_PAYLOAD_SECONDS-process_age)
    owner_deadline=min(_ENTRY_STARTED+PROPOSED_OWNER_SECONDS,now+PROPOSED_OWNER_SECONDS-process_age)
    inherited_context=None
    if planned.get('execution_target')=='direct_complete':
        from finite_research.process_entry import current_invocation
        invocation=current_invocation()
        if invocation is None or not invocation.get('context_path'):
            raise ValueError('AuthenticatedDirectDeadlineContextRequired')
        context_path=Path(invocation['context_path']).resolve()
        if sha(context_path)!=invocation['context_sha256']:
            raise ValueError('ChangedDirectDeadlineContext')
        context=read(context_path)
        if (context.get('deadline_clock')!='monotonic'
            or context['deadline']>context['parent_deadline_monotonic']):
            raise ValueError('InvalidDirectInheritedDeadlineChain')
        sampled_perf=time.perf_counter()
        sampled_monotonic=time.monotonic()
        absolute_remaining=(datetime.fromisoformat(task['deadline_utc'])-datetime.now(timezone.utc)).total_seconds()
        deadline=min(deadline,sampled_perf+context['deadline']-sampled_monotonic,sampled_perf+absolute_remaining)
        owner_deadline=min(owner_deadline,sampled_perf+context['parent_deadline_monotonic']-sampled_monotonic,
            sampled_perf+absolute_remaining)
        inherited_context=dict(path=str(context_path),sha256=invocation['context_sha256'],
            payload_monotonic=context['deadline'],owner_monotonic=context['parent_deadline_monotonic'])
    if deadline<=now:raise TimeoutError('LiveStartupConsumedPayload')
    d.publish(HERE/'CLOCK.json', dict(deadline=deadline, launch_pid=os.getpid(),
        payload_seconds=PROPOSED_PAYLOAD_SECONDS,owner_deadline=owner_deadline,
        task_deadline_utc=task['deadline_utc'],inherited_context=inherited_context))
    with ExitStack() as held:
        identity_file = HERE/'PYTHON_IDENTITY.json'; held.enter_context(d.guard(identity_file))
        if sha(identity_file) != selected[str(identity_file)]:
            raise ValueError('ChangedPredeclaredExecutableRoles')
        bindings = python_identity()
        for role in ('controller', 'child'):
            expected = bindings[role]; held.enter_context(d.guard(expected['executable']))
            if (selected.get(expected['executable']) != expected['sha256'] or
                    sha(expected['executable']) != expected['sha256'] or
                    file_identity(expected['executable']) != expected['file_identity']):
                raise ValueError('ChangedPredeclaredExecutableFile', role)
        startup = record_image(d, resources, HERE, 'launch', bindings, None)
        validate_image(startup, bindings)
        for name in ('TASK.json','AUTHORIZATION.json','LAUNCH_REVIEW.json','LIVE_REQUEST.json','PINNED_SOURCES.json','CLOCK.json','STARTUP_IDENTITY.json'):
            selected[str(HERE/name)] = sha(HERE/name)
        d.publish(HERE/'live01_INPUT_PINS.json', selected)
        selected[str(HERE/'live01_INPUT_PINS.json')] = sha(HERE/'live01_INPUT_PINS.json')
        d.supervisor.HERE = HERE
        command = [bindings['child']['executable'], '-B', str(CODE/'run.py'),
                   '--request-dir',str(HERE),'parent', repr(deadline), str(os.getpid()), *argv]
        def bounded(label, args, remaining, protected):
            available = min(remaining, deadline-time.perf_counter())
            if available <= 0: raise TimeoutError('NoSharedLivePayloadBudget')
            return d.supervisor.bounded(label, args, available, protected)
        supervisor = SimpleNamespace(HERE=HERE, sha=d.supervisor.sha,
                                     publish=d.supervisor.publish, bounded=bounded)
        # Existing owner reserves cleanup and deducts startup; it owns all descendants.
        owner_remaining=int(min(PROPOSED_OWNER_SECONDS-(time.perf_counter()-_ENTRY_STARTED),
                                PROPOSED_OWNER_SECONDS-max(0.,time.time()-psutil.Process().create_time()),
                                owner_deadline-time.perf_counter()))
        result = native.bounded('live01', command, owner_remaining,
            {Path(path): digest for path,digest in selected.items()}, supervisor, resources=resources.APPROVED)
        if request().get('execution_target')=='direct_complete' and result['returncode']==0:
            from finite_research.direct_admission import capture_retirement
            capture_retirement(sys.modules[__name__])
        return result['returncode']


def main(argv):
    global HERE
    if HERE is not None:raise ValueError('OneFreshOwnedEntryRequired')
    if len(argv)<2 or argv[0]!='--request-dir':raise ValueError('ExplicitRequestDirectoryRequired')
    HERE=Path(argv[1]).resolve();argv=argv[2:]
    role = 'coordinator' if argv[:1] == ['parent'] else ('worker' if '--worker' in argv else 'launch')
    gate(role)
    planned=read(HERE/'LIVE_REQUEST.json')
    owns_observation=not obs.enabled()
    if obs.choice(planned) is not None and owns_observation:
        paths={'launch':HERE/'live01_STAGE_TIMING.json','coordinator':HERE/'live01/STAGE_TIMING.json','worker':HERE/'live01/worker/STAGE_TIMING.json'}
        collector=obs.activate(paths[role],role,{'launch':0,'coordinator':1,'worker':2}[role],sha(HERE/'LIVE_REQUEST.json'))
        if collector is not None:collector.adopt_start(_ENTRY_STARTED,_ENTRY_CPU,'startup_and_initial_admission')
    code=None
    continuation=None
    if role=='launch' and planned.get('execution_target')=='direct_complete':
        from finite_research.direct_admission import begin
        continuation=begin(sys.modules[__name__])
    try:
        with obs.span('selected_role','role'):
            with resource_scope(role):
                if role == 'coordinator':code=parent(float(argv[1]), int(argv[2]), argv[3:])
                elif role == 'worker':code=worker(argv)
                else:code=launch(argv)
    finally:
        primary=sys.exc_info()[1]
        finished=obs.finish('returned:'+str(code)) if owns_observation else None
        if owns_observation and planned.get('execution_target')=='direct_complete' and obs.enabled():
            if finished is not True or obs._active.incomplete or not obs._active.finished:
                failure=ValueError('IncompleteDirectStageObservationClosure')
                if primary is None:
                    raise failure
                primary.add_note(str(failure))
    if role=='launch' and code==0 and planned.get('execution_target')=='direct_complete':
        from finite_research.direct_admission import defer as defer_direct
        defer_direct(sys.modules[__name__],continuation)
    return code


if __name__ == '__main__':
    from finite_research.process_entry import execute
    from finite_research import run as canonical_run
    raise SystemExit(execute(lambda:canonical_run.main(sys.argv[1:])))
