from contextlib import ExitStack
from pathlib import Path
from datetime import datetime,timezone,timedelta
from finite_research import inputs,direct_inputs
from finite_research.input_contract import assignment_intent,assignment_capacity,select_input_contract,live_task_from_assignment

SCHEMA="V4ActivatedDirectComplete/1"

def live_assignment(value):
    from finite_research.partial_stages import choice
    publication=choice(value)=='complete'
    if (assignment_intent(value) is not True or value.get("execution_target")!="direct_complete"
        or value.get("prism_producer_mode")!="source_directions" or value.get("upstream_forecast_diagnostics") is not False
        or value.get("proposed_transition_seconds")!=10800 or value.get("publication_enabled") is not publication
        or value.get("transition_fluid_verification")!="backend_trusted" or value.get("native_seconds")!=7200):
        raise ValueError("ExplicitDirectEvidenceLiveAssignmentRequired")
    return value

def derive(args):
    import standalone_case
    prepared=Path(args.prepared).resolve()
    packet=Path(args.request_dir).resolve()
    output=Path(args.output).resolve()
    assignment_path=Path(args.assignment).resolve()
    if packet.exists() or output.exists() or packet/"live01" not in output.parents or not packet.parent.is_dir():
        raise ValueError("FreshContainedDirectActivationRequired")
    extra=inputs.read(prepared/"PINNED_SOURCES.json")
    extra[str(prepared/"PINNED_SOURCES.json")]=args.prepared_manifest_sha256
    extra[str(assignment_path)]=args.assignment_sha256
    with inputs.source_scope(args.source_manifest_sha256,extra):
        prior=direct_inputs.verify(prepared,args.prepared_manifest_sha256)
        assignment=live_assignment(inputs.read(assignment_path))
        contract=select_input_contract(assignment,args.input_contract)
        original=inputs.read(prior["assignment"])["input_contracts"][prior["input_contract"]]
        if assignment.get("stop_after","complete")!=prior["stop_after"]:raise ValueError("ChangedActivatedStage")
        if contract!=original or assignment_capacity(assignment)!=prior["maximum_faces"]:
            raise ValueError("ChangedDirectActivationInputs")
        packet.mkdir()
        planned=dict(prior,schema=SCHEMA,output=str(output),live_execution_authorized=True)
        with standalone_case.hold_inputs(Path(prior["case"])):
            case,increments,unused=standalone_case.load(Path(prior["case"]))
            stage=direct_inputs.runtime_fields(planned,case,increments,inputs.read(prior["policy_path"]),packet)
        planned["publication_enabled"]=planned["stop_after"]=="complete"
        inputs.write(packet/"STAGE_REQUEST.json",stage)
        inputs.write(packet/"LIVE_REQUEST.json",planned)
        inputs.write(packet/"TASK.json",live_task_from_assignment(assignment))
        inputs.write_new(packet/"PYTHON_IDENTITY.json",(inputs.V4/"runtime/FINITE_PYTHON_IDENTITY.json").read_bytes())
        binding=dict(schema="DirectEvidenceActivation/1",prepared_request=str(prepared),
            prepared_manifest_sha256=args.prepared_manifest_sha256,assignment=str(assignment_path),
            assignment_sha256=args.assignment_sha256,input_contract=args.input_contract,
            release_sha256=args.source_manifest_sha256,publication_enabled=planned["publication_enabled"],
            task_sha256=inputs.sha(packet/"TASK.json"),request_sha256=inputs.sha(packet/"LIVE_REQUEST.json"),
            selector=inputs.sha(packet/"STAGE_REQUEST.json"))
        inputs.write(packet/"ACTIVATION.json",binding)
        selected=dict(inputs.windows_sources(),**extra)
        for name in ("STAGE_REQUEST.json","LIVE_REQUEST.json","TASK.json","PYTHON_IDENTITY.json","ACTIVATION.json"):
            selected[str(packet/name)]=inputs.sha(packet/name)
        inputs.write(packet/"PINNED_SOURCES.json",selected)
        verify(packet,inputs.sha(packet/"PINNED_SOURCES.json"))
        return dict(passed=True,request_dir=str(packet),manifest_sha256=inputs.sha(packet/"PINNED_SOURCES.json"),
            execution_authorized=False,publication_enabled=False,separate_grant_and_reviews_required=True)

def verify(packet,manifest):
    import standalone_case
    packet=Path(packet).resolve()
    if inputs.sha(packet/"PINNED_SOURCES.json")!=manifest:
        raise ValueError("ChangedDirectActivationManifest")
    selected=inputs.read(packet/"PINNED_SOURCES.json")
    for name,digest in selected.items():
        if inputs.sha(name)!=digest:
            raise ValueError("ChangedDirectActivatedInput",name)
    activation=inputs.read(packet/"ACTIVATION.json")
    if activation.get("schema")!="DirectEvidenceActivation/1" or type(activation.get("publication_enabled")) is not bool:
        raise ValueError("DirectEvidenceActivationRequired")
    if (activation["release_sha256"]!=inputs.sha(inputs.V4/"runtime/RELEASE.json")
        or inputs.sha(activation["assignment"])!=activation["assignment_sha256"]):
        raise ValueError("ChangedDirectActivationSource")
    prior=direct_inputs.verify(Path(activation["prepared_request"]),activation["prepared_manifest_sha256"])
    prior_manifest=Path(activation["prepared_request"])/"PINNED_SOURCES.json"
    inherited=inputs.read(prior_manifest)
    inherited[str(prior_manifest)]=activation["prepared_manifest_sha256"]
    if any(selected.get(name)!=digest for name,digest in inherited.items()):
        raise ValueError("UnpinnedDirectActivationAncestry")
    assignment=live_assignment(inputs.read(activation["assignment"]))
    contract=select_input_contract(assignment,activation["input_contract"])
    if (contract!=inputs.read(prior["assignment"])["input_contracts"][prior["input_contract"]]
        or assignment_capacity(assignment)!=prior["maximum_faces"]):
        raise ValueError("ChangedDirectActivationInputs")
    if assignment.get("stop_after","complete")!=prior["stop_after"]:raise ValueError("ChangedActivatedStage")
    planned=inputs.read(packet/"LIVE_REQUEST.json")
    expected=dict(prior,schema=SCHEMA,output=planned["output"],live_execution_authorized=True)
    with standalone_case.hold_inputs(Path(prior["case"])):
        case,increments,unused=standalone_case.load(Path(prior["case"]))
        stage=direct_inputs.runtime_fields(expected,case,increments,inputs.read(prior["policy_path"]),packet)
    expected["publication_enabled"]=expected["stop_after"]=="complete"
    if (activation["publication_enabled"] is not expected["publication_enabled"] or planned!=expected or inputs.read(packet/"STAGE_REQUEST.json")!=stage
        or packet/"live01" not in Path(planned["output"]).resolve().parents
        or inputs.read(packet/"TASK.json")!=live_task_from_assignment(assignment)
        or inputs.read(packet/"PYTHON_IDENTITY.json")!=inputs.read(inputs.V4/"runtime/FINITE_PYTHON_IDENTITY.json")):
        raise ValueError("ChangedDirectActivatedSelection")
    for key,name in (("task_sha256","TASK.json"),("request_sha256","LIVE_REQUEST.json"),("selector","STAGE_REQUEST.json")):
        if activation[key]!=inputs.sha(packet/name):
            raise ValueError("ChangedDirectActivationBinding")
    for name in ("TASK.json","LIVE_REQUEST.json","STAGE_REQUEST.json","ACTIVATION.json","PYTHON_IDENTITY.json"):
        if selected.get(str(packet/name))!=inputs.sha(packet/name):
            raise ValueError("UnpinnedDirectActivation")
    if selected.get(activation["assignment"])!=activation["assignment_sha256"]:
        raise ValueError("UnpinnedDirectAssignment")
    return planned

def gate(packet,role):
    packet=Path(packet)
    planned=inputs.read(packet/"LIVE_REQUEST.json")
    if planned.get("schema")!=SCHEMA:
        raise ValueError("DirectPreparedInputsAreNotLiveAuthority")
    verify(packet,inputs.sha(packet/"PINNED_SOURCES.json"))
    task=inputs.read(packet/"TASK.json")
    now=datetime.now(timezone.utc)
    start=datetime.fromisoformat(task["start_utc"])
    end=datetime.fromisoformat(task["deadline_utc"])
    cutoff=datetime.fromisoformat(task["launch_cutoff_utc"])
    if role not in ("launch","coordinator","worker") or not start<=now<end or role=="launch" and now>=cutoff:
        raise ValueError("ExactDirectLiveWindowRequired")
    grant=inputs.read(packet/"AUTHORIZATION.json")
    review=inputs.read(packet/"LAUNCH_REVIEW.json")
    if (grant.get("approved") is not True or grant.get("mode")!="fresh_live_once"
        or grant.get("human_message")!=task["human_message"] or grant.get("assignment")!=task["assignment"]
        or grant.get("maximum_invocations")!=1 or grant.get("payload_seconds")!=14400
        or grant.get("owner_seconds")!=14700 or grant.get("transition_seconds")!=10800
        or grant.get("native_seconds")!=7200
        or grant.get("native_scope")!="DOWNSTREAM_NATIVE_ONLY"
        or grant.get("activation_sha256")!=inputs.sha(packet/"ACTIVATION.json")
        or grant.get("task_sha256")!=inputs.sha(packet/"TASK.json")
        or grant.get("request_sha256")!=inputs.sha(packet/"LIVE_REQUEST.json")
        or grant.get("manifest_sha256")!=inputs.sha(packet/"PINNED_SOURCES.json")
        or grant.get("source_release_sha256")!=inputs.sha(inputs.V4/"runtime/RELEASE.json")
        or grant.get("publication_enabled") is not planned["publication_enabled"]
        or review.get("approved") is not True or review.get("mode")!="fresh_live_once"
        or review.get("authorization_sha256")!=inputs.sha(packet/"AUTHORIZATION.json")
        or review.get("task_sha256")!=grant["task_sha256"]
        or review.get("manifest_sha256")!=grant["manifest_sha256"]
        or review.get("source_release_sha256")!=grant["source_release_sha256"]):
        raise ValueError("ExactDirectLiveGrantAndReviewRequired")
    import owned_runtime
    release=owned_runtime.verify_release()
    reviews=release["independent_reviews"]
    paths=[(inputs.V4/row["path"]).resolve() for row in reviews]
    reviewers=[inputs.read(path)["reviewer"] for path in paths]
    if len(reviews)<2 or len(set(paths))!=len(paths) or len(set(reviewers))!=len(reviewers):
        raise ValueError("IndependentDirectSourceReviewsRequired")
    return task
