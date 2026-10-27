from pathlib import Path
import base64,hashlib,inspect,json,os,sys,time
from finite_research.direct_evidence_modules import driver
from finite_research.direct_contract import ACCEPTANCE,identity,method

_issued={}
_pending=None
_continuation=None


def owner_identity(owner):
    from finite_research import run
    if owner is not run or Path(owner.__file__).resolve()!=Path(__file__).parent/"run.py":
        raise ValueError("CanonicalDirectLaunchOwnerRequired")
    from types import CodeType
    path=Path(owner.__file__).resolve()
    from finite_research.inputs import package_sources
    if package_sources().get(str(path))!=driver.sha(path):
        raise ValueError("UnboundDirectLaunchCode")
    compiled=compile(path.read_text(encoding="utf8"),str(path),"exec",dont_inherit=True)
    definitions={value.co_name:value for value in compiled.co_consts if isinstance(value,CodeType)}
    for name in ("main","launch","gate","pins","check_clock","request"):
        function=inspect.unwrap(getattr(owner,name))
        if function.__globals__ is not owner.__dict__ or function.__code__!=definitions[name]:
            raise ValueError("ChangedCanonicalDirectLaunchFunction",name)
    return run


def begin(owner):
    global _continuation
    owner_identity(owner)
    frame=inspect.currentframe().f_back
    if frame.f_code is not owner.main.__code__ or frame.f_globals is not owner.__dict__ or _continuation is not None:
        raise ValueError("FreshCanonicalDirectContinuationRequired")
    from finite_research.process_entry import current_invocation
    invocation=current_invocation()
    if invocation is None or invocation["context_path"] is None:
        raise ValueError("AuthenticatedDirectCLIContextRequired")
    token=object()
    _continuation=dict(token=token,owner=owner,pid=os.getpid(),receipts=None,invocation=invocation,
        publication_started=False,admission_issued=False,
        functions={name:getattr(owner,name) for name in ("main","launch","gate","pins","check_clock","request")})
    return token


def capture_retirement(owner):
    owner_identity(owner)
    frame=inspect.currentframe().f_back
    if (_continuation is None or _continuation["owner"] is not owner
        or frame.f_code is not owner.launch.__wrapped__.__code__ or frame.f_globals is not owner.__dict__
        or _continuation["receipts"] is not None):
        raise ValueError("ActualDirectRetirementContinuationRequired")
    packet=Path(owner.HERE)
    retirement(packet)
    paths=[packet/name for name in ("live01_RESOURCE_RESULT.json","live01_EXECUTION.json",
        "live01_EXECUTION_CLOSURE.json","live01_SCOPE_RESULT.json","live01_SCOPE_CLOSURE.json","live01_SCOPE/RETIREMENT.json")]
    for folder in (packet/"live01",packet/"live01/worker"):
        paths.extend(folder/name for name in ("RESULT.json","WORKER_COMPLETION.json","LOADED_SOURCE_CLOSURE.json","STARTUP_IDENTITY.json"))
    root=Path(owner.request()["output"])
    closure="DIRECT_SCOPE_EXIT.json" if owner.request()["stop_after"]=="complete" else "STAGE_COMPLETION.json"
    paths.extend(root/name for name in (closure,"RESULT.json"))
    _continuation["receipts"]={str(path.resolve()):driver.sha(path) for path in paths}


class Admission:
    __slots__=()


def has_pending():
    return _pending is not None


def require_admission(token,root):
    if type(token) is not Admission or token not in _issued:
        raise ValueError("GenuineCompletedDirectAdmissionRequired")
    record=_issued.pop(token)
    if Path(root).resolve()!=record["root"] or record["pid"]!=os.getpid():
        raise ValueError("ChangedDirectAdmissionOwner")
    return record["current"],record["expected"],record["previous"]


def retirement(packet):
    packet=Path(packet)
    resource=driver.read(packet/"live01_RESOURCE_RESULT.json")
    if (not resource["passed"] or not resource["linux_retired"] or not resource["monitor_retired"]
        or resource.get("primary_error") is not None or resource.get("cleanup_error") is not None
        or resource["worker_returncode"]!=0):
        raise ValueError("IncompleteDirectResourceRetirement")
    for name in ("windows_retirement","windows_payload_retirement"):
        value=resource[name]
        if not value["passed"] or value["after"]["active_processes"]!=0:
            raise ValueError("IncompleteDirectWindowsRetirement")
    linux=driver.read(packet/"live01_SCOPE/RETIREMENT.json")
    if not linux["passed"] or not linux["after"]["retired"] or linux["after"]["live_members"]:
        raise ValueError("IncompleteDirectLinuxRetirement")
    for name in ("live01_EXECUTION_CLOSURE.json","live01_SCOPE_CLOSURE.json"):
        if not driver.read(packet/name)["all_guards_released"]:
            raise ValueError("OutstandingDirectOwnerGuards")
    if driver.read(packet/"live01_EXECUTION_CLOSURE.json")["receipt_sha256"]!=driver.sha(packet/"live01_EXECUTION.json"):
        raise ValueError("ChangedDirectExecutionClosure")
    scope=driver.read(packet/"live01_SCOPE_RESULT.json")
    if (not scope["passed"] or not scope["all_linux_retired"] or scope["worker_returncode"]!=0
        or scope["linux_retirement_sha256"]!=driver.sha(packet/"live01_SCOPE/RETIREMENT.json")
        or driver.read(packet/"live01_SCOPE_CLOSURE.json")["receipt_sha256"]!=driver.sha(packet/"live01_SCOPE_RESULT.json")):
        raise ValueError("ChangedDirectRetirementBinding")
    if (packet/"live01_SCOPE_FAILURE.json").exists():
        raise ValueError("OutstandingDirectScopeFailure")


def closed_worker(packet,folder,pins):
    import psutil
    result=driver.read(folder/"RESULT.json")
    completion=driver.read(folder/"WORKER_COMPLETION.json")
    closure=driver.read(folder/"LOADED_SOURCE_CLOSURE.json")
    if not result["passed"] or not completion["passed"] or closure["unpinned"] or psutil.pid_exists(result["pid"]):
        raise ValueError("IncompleteDirectWorkerExit")
    for name,key in (("RESULT.json","result_sha256"),("STARTUP_IDENTITY.json","startup_identity_sha256"),
        ("LOADED_SOURCE_CLOSURE.json","source_closure_sha256")):
        if driver.sha(folder/name)!=completion[key]:
            raise ValueError("ChangedDirectWorkerCompletion")
    for path,digest in closure["files"].items():
        if pins.get(path)!=digest or driver.sha(path)!=digest:
            raise ValueError("ChangedDirectWorkerSource")
    return result,completion


def defer(owner,token):
    global _pending
    owner_identity(owner)
    frame=inspect.currentframe().f_back
    if (frame.f_code is not owner.main.__code__ or frame.f_globals is not owner.__dict__
        or owner._selected_owners is not None or frame.f_locals.get("role")!="launch"
        or frame.f_locals.get("code")!=0):
        raise ValueError("ActualSuccessfulOuterDirectExitRequired")
    if (_continuation is None or _continuation["token"] is not token or _continuation["owner"] is not owner
        or _continuation["pid"]!=os.getpid() or _continuation["receipts"] is None
        or any(getattr(owner,name) is not value for name,value in _continuation["functions"].items())):
        raise ValueError("ChangedDirectContinuationCapability")
    if _pending is not None:
        raise ValueError("OneDirectOuterCompletionRequired")
    _pending=(owner,os.getpid(),dict(_continuation["receipts"]),_continuation["invocation"])


def invalidate_process(invocation):
    global _pending,_continuation
    if _pending is not None and _pending[3] is invocation:
        _pending=None
    if _continuation is not None and _continuation["invocation"] is invocation:
        _continuation=None


def failed_process(code,invocation):
    from finite_research import process_entry
    frame=inspect.currentframe().f_back
    if frame.f_code is not process_entry.execute.__code__ or frame.f_globals is not vars(process_entry):
        raise ValueError('ActualFailedCLIClosureRequired')
    record=_continuation
    if record is None:
        return None
    owner=record['owner']
    owner_identity(owner)
    if (code in (None,0) or not invocation['completed'] or record['invocation'] is not invocation
        or record['pid']!=os.getpid() or record['publication_started'] or record['admission_issued']
        or _pending is not None or owner._selected_owners is not None
        or any(getattr(owner,name) is not value for name,value in record['functions'].items())):
        raise ValueError('UnprovenPrepublicationFailure')
    planned=owner.request()
    root=Path(planned['output']).resolve()
    packet=Path(owner.HERE).resolve()
    if not root.is_relative_to(packet/'live01') or root==packet/'live01':
        raise ValueError('ExactFailedOutputOwnerRequired')
    return process_entry.failure_outcome(code,invocation,root,synthetic=False)


def complete_process(code,invocation):
    global _pending
    from finite_research import process_entry
    frame=inspect.currentframe().f_back
    if frame.f_code is not process_entry.execute.__code__ or frame.f_globals is not process_entry.__dict__:
        raise ValueError("ActualCLIProcessClosureRequired")
    pending=_pending
    _pending=None
    if pending is None:
        return
    owner,pid,receipts,issued_invocation=pending
    if issued_invocation is not invocation or code!=0 or not invocation["completed"]:
        invalidate_process(issued_invocation)
        return
    receipts.update(invocation["files"])
    if not invocation["files"]:
        raise ValueError("DirectOuterProcessClosureRequired")
    if pid!=os.getpid() or owner._selected_owners is not None:
        raise ValueError("ChangedDirectOuterOwner")
    if owner.request()['stop_after']!='complete':
        return finish_partial(owner,receipts)
    _continuation['publication_started']=True
    return finish(owner,receipts)


def finish(owner,receipts):
    frame=inspect.currentframe().f_back
    if frame.f_code is not complete_process.__code__ or frame.f_globals is not globals():
        raise ValueError("DirectCompletionIssuerRequired")
    packet=Path(owner.HERE).resolve()
    planned=owner.request()
    if planned.get("publication_enabled") is not True:
        raise ValueError("SeparateDirectPublicationGrantRequired")
    owner.gate("coordinator")
    selected=owner.pins()
    deadline=driver.read(packet/"CLOCK.json")["deadline"]
    def check():
        owner.check_clock(deadline)
    for path,digest in receipts.items():
        if driver.sha(path)!=digest:
            raise ValueError("ChangedCapturedDirectRetirement")
    retirement(packet)
    parent,parent_completion=closed_worker(packet,packet/"live01",selected)
    worker,worker_completion=closed_worker(packet,packet/"live01/worker",selected)
    if parent["worker"]["completion_sha256"]!=driver.sha(packet/"live01/worker/WORKER_COMPLETION.json"):
        raise ValueError("ChangedDirectChildClosure")
    root=Path(planned["output"]).resolve()
    if not root.is_relative_to(packet/"live01") or root==packet/"live01":
        raise ValueError("ExactDirectOutputOwnerRequired")
    if (root/"STOP.json").exists() or not driver.read(root/"RESULT.json")["passed"]:
        raise ValueError("OutstandingDirectProducerFailure")
    closed=driver.read(root/"DIRECT_SCOPE_EXIT.json")
    stage=driver.read(packet/"STAGE_REQUEST.json")
    if (closed["pid"]!=worker["pid"] or worker.get("direct_scope_exit_sha256")!=driver.sha(root/"DIRECT_SCOPE_EXIT.json")
        or closed["request_sha256"]!=driver.sha(packet/"STAGE_REQUEST.json")
        or closed["source_release_sha256"]!=planned["source_release_sha256"]):
        raise ValueError("ChangedDirectSuccessfulScopeExit")
    result=closed["result"]
    if Path(result["folder"]).resolve()!=root/"background":
        raise ValueError("ChangedDirectEvidenceLocation")
    binding=result["binding"]
    if (binding["request_sha256"]!=closed["request_sha256"] or binding["case_sha256"]!=planned["case_sha256"]
        or binding["policy_identity"]!=stage["downstream_policy_identity"]
        or binding["source_release_sha256"]!=closed["source_release_sha256"]):
        raise ValueError("ChangedIndependentDirectInputs")
    expected_parent=driver.read(root/"background/PARENT_EXPECTATIONS.json")
    previous=base64.b64decode(expected_parent["previous_current_base64"],validate=True)
    if (hashlib.sha256(previous).hexdigest()!=expected_parent["previous_current_sha256"]
        or json.loads(previous).get("mesh")!=binding["parent_prism_mesh"]
        or expected_parent["binding"]!=binding):
        raise ValueError("ChangedIndependentDirectParent")
    from finite_research import direct_consumer,direct_publication
    source_files={str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
        if getattr(module,"__file__",None) and Path(module.__file__).suffix.lower() in (".py",".pyd",".dll")}
    for path in source_files:
        if selected.get(path)!=driver.sha(path):
            raise ValueError("UnpinnedDirectTerminalDependency",path)
    import secrets
    from finite_research.direct_fill_contract import binding as contract_binding
    policy=driver.read(root/"background/POLICY.json")
    current=dict(schema=ACCEPTANCE,acceptance_contract=contract_binding(policy),execution_target="direct_complete",mesh=binding["parent_prism_mesh"],
        publication_id=secrets.token_hex(32),
        source_release_sha256=binding["source_release_sha256"],method_identity=identity(method()),
        validation_sha256=result["validation_sha256"],parent_expectations_sha256=result["parent_expectations_sha256"],
        handoff_sha256=result["handoff_sha256"],evidence_index_sha256=result["evidence_index_sha256"],
        scope_exit_sha256=driver.sha(root/"DIRECT_SCOPE_EXIT.json"),
        worker_completion_sha256=driver.sha(packet/"live01/worker/WORKER_COMPLETION.json"),
        owner_retirement_sha256=driver.sha(packet/"live01_SCOPE_CLOSURE.json"),
        continuous_sdf_fluid="NOT_PERFORMED",stored_boundary_geometry="ALL_TWELVE_VALIDATED",
        publication_enabled=True,provisional=False,scope_exit_verified=True,
        normal_run_qualified=False,solver_ready=False)
    expected=dict(descriptor=current,binding=binding,source_release_sha256=binding["source_release_sha256"],
        source_pins=selected,request_root=str(packet),retirement_files=receipts)
    _continuation['admission_issued']=True
    token=Admission()
    _issued[token]=dict(root=root,pid=os.getpid(),current=current,expected=expected,previous=previous)
    try:
        check()
        return direct_publication.publish(root,token,check)
    finally:
        _issued.pop(token,None)


def finish_partial(owner,receipts):
    frame=inspect.currentframe().f_back
    if frame.f_code is not complete_process.__code__ or frame.f_globals is not globals():
        raise ValueError('PartialCompletionIssuerRequired')
    owner.gate('coordinator'); selected=owner.pins(); planned=owner.request()
    if planned['publication_enabled'] is not False:raise ValueError('PartialPublicationForbidden')
    for path,digest in receipts.items():
        if driver.sha(path)!=digest:raise ValueError('ChangedCapturedPartialRetirement')
    packet=Path(owner.HERE); retirement(packet)
    parent,pc=closed_worker(packet,packet/'live01',selected)
    worker,wc=closed_worker(packet,packet/'live01/worker',selected)
    if parent['worker']['completion_sha256']!=driver.sha(packet/'live01/worker/WORKER_COMPLETION.json'):
        raise ValueError('ChangedPartialChildClosure')
    root=Path(planned['output']).resolve()
    if not root.is_relative_to(packet/'live01') or root==packet/'live01':raise ValueError('ChangedPartialOutput')
    from finite_research.partial_stages import validate
    stage=driver.read(packet/'STAGE_REQUEST.json')
    value=validate(root,planned['stop_after'],planned['case_sha256'],stage['upstream_flags']['through_layer'],
        planned['source_release_sha256'],driver.sha(packet/'STAGE_REQUEST.json'))
    if value['pid']!=worker['pid'] or worker.get('direct_scope_exit_sha256')!=driver.sha(root/'STAGE_COMPLETION.json'):
        raise ValueError('ChangedPartialProducerIdentity')
    owner.check_clock(driver.read(packet/'CLOCK.json')['deadline'])
    return dict(phase='PARTIAL_BY_REQUEST',publication_state='NOT_RUN_BY_REQUEST',
        cold_readback_state='NOT_RUN_BY_REQUEST',root=str(root),stage_completion_sha256=driver.sha(root/'STAGE_COMPLETION.json'),
        retirement_files=receipts,requested_stop_after=planned['stop_after'],full_mesh_complete=False)
