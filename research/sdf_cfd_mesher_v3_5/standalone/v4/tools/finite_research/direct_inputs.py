"""Direct-complete metadata through the existing CLI; never issues a live grant."""
from pathlib import Path
from contextlib import ExitStack
from types import SimpleNamespace
from finite_research import inputs
from finite_research.direct_contract import validate_policy,identity,METHOD

SCHEMA="V4PreparedDirectComplete/2"
FORBIDDEN=inputs.FORBIDDEN+("ACTIVATION.json",)

def selected(directory):
    path=Path(directory)/"LIVE_REQUEST.json"
    return path.is_file() and inputs.read(path).get("schema") in (SCHEMA,"V4ActivatedDirectComplete/1")

def runtime_fields(request,case,increments,policy,packet):
    import hashlib
    from finite_research.direct_runtime import stage_request
    from finite_research.partial_stages import choice
    request['stop_after']=choice(request)
    stage=stage_request(request,case,increments,policy)
    selector=hashlib.sha256(inputs.encoded(stage)).hexdigest()
    arguments=["--finite-collar-selection",selector,"--case",request["case"],"--from-layer","0",
        "--through-layer",str(len(increments)),"--mode","fastprod","--output",request["output"],
        "--minimum-free-bytes","1000000000","--surface-carrier","nodal","--surface-assurance","numerical",
        "--prism-producer-mode","source_directions"]
    arguments += ["--stop-after",request["stop_after"]]
    request.update(selector=selector,stage_request=str(packet/"STAGE_REQUEST.json"),stage_request_sha256=selector,
        arguments=arguments,entry=str(inputs.HERE/"run.py"),publication_enabled=False)
    return stage

def assignment(args,case_path,policy_path):
    value=inputs.read(args.assignment)
    if (inputs.sha(args.assignment)!=args.assignment_sha256 or value.get("schema")!="V4DirectSourceAssignment/1"
        or value.get("execution_authorized") is not False or value.get("maximum_live_invocations")!=0
        or value.get("execution_target")!="direct_complete" or value.get("prism_producer_mode")!="source_directions"
        or value.get("proposed_payload_seconds")!=14400 or value.get("proposed_owner_seconds")!=14700):
        raise ValueError("ExplicitSourceOnlyDirectAssignmentRequired")
    row=value.get("input_contracts",{}).get(args.input_contract)
    if (type(row) is not dict or set(row)!={"case_path","case_sha256","policy_path","policy_sha256","imported_source_path","imported_source_sha256"}
        or row["case_path"]!=str(case_path) or row["policy_path"]!=str(policy_path)
        or row["case_sha256"]!=inputs.sha(case_path) or row["policy_sha256"]!=inputs.sha(policy_path)
        or (row["imported_source_path"] is None)!=(row["imported_source_sha256"] is None)):
        raise ValueError("ExactDirectAssignmentInputsRequired")
    return value

def _prepare(args,case_path,policy_path,prior=None):
    import standalone_case
    from immutable_state_access import guard
    packet=Path(args.request_dir).resolve()
    output=Path(args.output).resolve()
    if packet.exists() or output.exists() or not packet.parent.is_dir() or packet/"live01" not in output.parents:
        raise ValueError("FreshContainedDirectRequestRequired")
    if packet in case_path.parents or packet in policy_path.parents:
        raise ValueError("DirectInputsMustPreexistOutsideRequest")
    extra={str(Path(args.assignment).resolve()):args.assignment_sha256,
        str(case_path):inputs.sha(case_path),str(policy_path):inputs.sha(policy_path)}
    assigned_row=inputs.read(args.assignment).get("input_contracts",{}).get(args.input_contract,{})
    if assigned_row.get("imported_source_path") is not None:
        extra[str(Path(assigned_row["imported_source_path"]).resolve())]=assigned_row["imported_source_sha256"]
    if prior is not None:
        extra.update(inputs.read(Path(args.prepared)/"PINNED_SOURCES.json"))
        extra[str(Path(args.prepared).resolve()/"PINNED_SOURCES.json")]=args.prepared_manifest_sha256
    dependency=inputs.V4/"tools/finite_research/direct_evidence_modules/DEPENDENCIES.json"
    for row in inputs.read(dependency)["inputs"].values():
        extra[row["path"]]=row["sha256"]
    with inputs.source_scope(args.source_manifest_sha256,extra) as (unused,pins):
        with ExitStack() as held:
            held.enter_context(standalone_case.hold_inputs(case_path))
            held.enter_context(guard(policy_path))
            grant=assignment(args,case_path,policy_path)
            policy=validate_policy(inputs.read(policy_path))
            case,increments,case_hash=standalone_case.load(case_path)
            if (case["schema"]!="V4MeshCase/4" or "background" in case
                or case["prisms"].get("trajectory_completion") is not None or len(increments)<1):
                raise ValueError("UnchangedIndependentPrismCaseRequired")
            if case["resources"]!=dict(minimum_free_bytes=1000000000,output_limit_bytes=4000000000):
                raise ValueError("ExplicitDirectResourcePolicyRequired")
            imported=standalone_case.imported_geometry(case_path,standalone_case.read(case_path))
            assigned_row=grant["input_contracts"][args.input_contract]
            if ((str(imported[0]) if imported else None)!=assigned_row["imported_source_path"]
                or standalone_case.held_import_sha256(case_path)!=assigned_row["imported_source_sha256"]):
                raise ValueError("ChangedAssignedDirectImportedSource")
            if imported:
                extra[str(imported[0])]=standalone_case.held_import_sha256(case_path)
            if prior is not None:
                if prior["case_sha256"]!=case_hash or prior["policy_identity"]!=identity(policy):
                    raise ValueError("ChangedDerivedDirectInputs")
            request=dict(schema=SCHEMA,stop_after=grant.get("stop_after","complete"),execution_target="direct_complete",execution_authorized=False,
                live_connection_installed=False,runtime_evidence_connection_installed=True,source_root=str(inputs.V4),source_release_sha256=args.source_manifest_sha256,
                case=str(case_path),case_sha256=case_hash,canonical_input_identity=identity(case["geometry"]),
                prism_schedule_identity=identity(case["prisms"]),policy_path=str(policy_path),
                policy_identity=identity(policy),policy_sha256=inputs.sha(policy_path),backend_method=METHOD,
                assignment=str(Path(args.assignment).resolve()),assignment_sha256=args.assignment_sha256,
                input_contract=args.input_contract,output=str(output),maximum_faces=grant["maximum_faces"],
                proposed_payload_seconds=14400,proposed_owner_seconds=14700,downstream_seconds=policy["downstream_seconds"],
                upstream_flags=dict(surface_assurance="numerical",surface_carrier="nodal",
                    prism_producer_mode="source_directions",through_layer=len(increments),from_layer=0,
                    upstream_forecast_diagnostics=False),
                imported_source_sha256=standalone_case.held_import_sha256(case_path),
                geometry_assurance="all_twelve_fresh",continuous_sdf_fluid="NOT_PERFORMED",
                farfield_preparation="fresh_generic_once_then_authenticated_fixed_boundary",
                prior_prepared_path=str(Path(args.prepared).resolve()) if prior is not None else None,
                prior_prepared_manifest_sha256=args.prepared_manifest_sha256 if prior is not None else None)
            if type(request["maximum_faces"]) is not int or request["maximum_faces"]<=0:
                raise ValueError("ExplicitDirectCapacityRequired")
            if prior is None and (args.maximum_faces!=request["maximum_faces"] or args.payload_seconds!=14400
                or args.owner_seconds!=14700 or args.execution_target!="direct_complete"
                or args.upstream_forecast_diagnostics or args.transition_fluid_verification is not None):
                raise ValueError("DirectArgumentsMustMatchAssignment")
            packet.mkdir()
            stage=runtime_fields(request,case,increments,policy,packet)
            inputs.write(packet/"STAGE_REQUEST.json",stage)
            inputs.write(packet/"LIVE_REQUEST.json",request)
            selected_pins=dict(pins,**extra)
            selected_pins[str(packet/"STAGE_REQUEST.json")]=inputs.sha(packet/"STAGE_REQUEST.json")
            selected_pins[str(packet/"LIVE_REQUEST.json")]=inputs.sha(packet/"LIVE_REQUEST.json")
            inputs.write(packet/"PINNED_SOURCES.json",selected_pins)
            return dict(passed=True,request_dir=str(packet),manifest_sha256=inputs.sha(packet/"PINNED_SOURCES.json"),
                execution_authorized=False,live_connection_installed=False,mesh_generated=False)

def verify(packet,manifest,_seen=None):
    packet=Path(packet).resolve()
    seen=set() if _seen is None else set(_seen)
    if packet in seen:
        raise ValueError("CyclicDirectPreparationLineage")
    seen.add(packet)
    if any((packet/name).exists() for name in FORBIDDEN):
        raise ValueError("DirectSourceRequestCannotCarryLiveAuthority")
    if inputs.sha(packet/"PINNED_SOURCES.json")!=manifest:
        raise ValueError("ChangedDirectPreparedManifest")
    pins=inputs.read(packet/"PINNED_SOURCES.json")
    for name,digest in pins.items():
        if inputs.sha(name)!=digest:
            raise ValueError("ChangedDirectPreparedInput",name)
    request=inputs.read(packet/"LIVE_REQUEST.json")
    if (pins.get(str(packet/"LIVE_REQUEST.json"))!=inputs.sha(packet/"LIVE_REQUEST.json")
        or request.get("schema")!=SCHEMA or request.get("execution_authorized") is not False
        or request.get("live_connection_installed") is not False or request.get("execution_target")!="direct_complete"
        or request.get("source_root")!=str(inputs.V4)
        or request.get("source_release_sha256")!=inputs.sha(inputs.V4/"runtime/RELEASE.json")
        or Path(request["output"]).exists() or packet/"live01" not in Path(request["output"]).resolve().parents):
        raise ValueError("ChangedDisabledDirectRequest")
    if (inputs.sha(request["case"])!=request["case_sha256"]
        or inputs.sha(request["policy_path"])!=request["policy_sha256"]
        or identity(validate_policy(inputs.read(request["policy_path"])))!=request["policy_identity"]):
        raise ValueError("ChangedDirectPreparedCaseOrPolicy")
    semantic(request,packet,pins,seen)
    return request

def semantic(request,packet,pins,seen=None):
    import standalone_case
    args=SimpleNamespace(assignment=Path(request["assignment"]),assignment_sha256=request["assignment_sha256"],
        input_contract=request["input_contract"])
    case_path=Path(request["case"])
    policy_path=Path(request["policy_path"])
    for path,digest in ((case_path,request["case_sha256"]),(policy_path,request["policy_sha256"]),
        (args.assignment,args.assignment_sha256)):
        if not path.is_absolute() or pins.get(str(path))!=digest:
            raise ValueError("UnpinnedDirectSemanticInput")
    grant=assignment(args,case_path,policy_path)
    policy=validate_policy(inputs.read(policy_path))
    with standalone_case.hold_inputs(case_path):
        case,increments,case_hash=standalone_case.load(case_path)
        if case["schema"]!="V4MeshCase/4" or "background" in case or case["prisms"].get("trajectory_completion") is not None:
            raise ValueError("UnchangedIndependentPrismCaseRequired")
        imported=standalone_case.imported_geometry(case_path,standalone_case.read(case_path))
        imported_hash=standalone_case.held_import_sha256(case_path)
        assigned_row=grant["input_contracts"][args.input_contract]
        if ((str(imported[0]) if imported else None)!=assigned_row["imported_source_path"]
            or imported_hash!=assigned_row["imported_source_sha256"]):
            raise ValueError("ChangedAssignedDirectImportedSource")
        if imported and pins.get(str(imported[0]))!=imported_hash:
            raise ValueError("UnpinnedDirectImportedSource")
        expected=dict(schema=SCHEMA,stop_after=grant.get("stop_after","complete"),execution_target="direct_complete",execution_authorized=False,
            live_connection_installed=False,runtime_evidence_connection_installed=True,source_root=str(inputs.V4),source_release_sha256=inputs.sha(inputs.V4/"runtime/RELEASE.json"),
            case=str(case_path),case_sha256=case_hash,canonical_input_identity=identity(case["geometry"]),
            prism_schedule_identity=identity(case["prisms"]),policy_path=str(policy_path),
            policy_identity=identity(policy),policy_sha256=inputs.sha(policy_path),backend_method=METHOD,
            assignment=str(args.assignment),assignment_sha256=args.assignment_sha256,input_contract=args.input_contract,
            output=request["output"],maximum_faces=grant["maximum_faces"],proposed_payload_seconds=14400,
            proposed_owner_seconds=14700,downstream_seconds=policy["downstream_seconds"],
            upstream_flags=dict(surface_assurance="numerical",surface_carrier="nodal",prism_producer_mode="source_directions",
                through_layer=len(increments),from_layer=0,upstream_forecast_diagnostics=False),
            imported_source_sha256=imported_hash,geometry_assurance="all_twelve_fresh",continuous_sdf_fluid="NOT_PERFORMED",
            farfield_preparation="fresh_generic_once_then_authenticated_fixed_boundary",
            prior_prepared_path=request.get("prior_prepared_path"),
            prior_prepared_manifest_sha256=request.get("prior_prepared_manifest_sha256"))
    stage=runtime_fields(expected,case,increments,policy,Path(packet))
    stage_path=Path(packet)/"STAGE_REQUEST.json"
    if (pins.get(str(stage_path))!=inputs.sha(stage_path) or inputs.read(stage_path)!=stage):
        raise ValueError("ChangedDirectStageSelection")
    if request!=expected or type(expected["maximum_faces"]) is not int or expected["maximum_faces"]<=0:
        raise ValueError("ChangedDirectDerivedMetadata")
    prior=expected["prior_prepared_path"]
    prior_hash=expected["prior_prepared_manifest_sha256"]
    if prior is None:
        if prior_hash is not None:
            raise ValueError("ChangedDirectPreparationLineage")
    else:
        path=Path(prior)/"PINNED_SOURCES.json"
        if not path.is_absolute() or pins.get(str(path))!=prior_hash or inputs.sha(path)!=prior_hash:
            raise ValueError("ChangedDirectPreparationLineage")
        prior_pins=inputs.read(path)
        if any(pins.get(name)!=digest for name,digest in prior_pins.items()):
            raise ValueError("UnheldDirectPreparationLineage")
        previous=verify(Path(prior),prior_hash,set(seen or ())|{Path(packet).resolve()})
        if any(previous[name]!=request[name] for name in (
            "case_sha256","policy_identity","canonical_input_identity","prism_schedule_identity","upstream_flags",
            "backend_method","imported_source_sha256","geometry_assurance","continuous_sdf_fluid")):
            raise ValueError("ChangedDirectInheritedInputs")

def packet_scope(args,packet,manifest):
    packet=Path(packet).resolve()
    pins=inputs.read(packet/"PINNED_SOURCES.json")
    pins[str(packet/"PINNED_SOURCES.json")]=manifest
    return inputs.source_scope(args.source_manifest_sha256,pins)

def prepare(args):
    return _prepare(args,Path(args.case).resolve(),Path(args.policy).resolve())

def derive(args):
    if inputs.read(args.assignment).get("schema") in ("ReusableFiniteInputAssignment/1","ReusableFiniteInputAssignment/2"):
        from finite_research.direct_activation import derive as activate
        return activate(args)
    with packet_scope(args,args.prepared,args.prepared_manifest_sha256):
        prior=verify(args.prepared,args.prepared_manifest_sha256)
    return _prepare(args,Path(prior["case"]),Path(prior["policy_path"]),prior)

def inspect_request(args):
    if inputs.read(Path(args.request_dir)/"LIVE_REQUEST.json").get("schema")=="V4ActivatedDirectComplete/1":
        from finite_research.direct_activation import verify as verify_activation
        with packet_scope(args,args.request_dir,args.manifest_sha256):
            request=verify_activation(args.request_dir,args.manifest_sha256)
        return dict(passed=True,request=request,publication_enabled=False,execution_authorized=False)
    with packet_scope(args,args.request_dir,args.manifest_sha256):
        request=verify(args.request_dir,args.manifest_sha256)
    return dict(passed=True,schema=SCHEMA,execution_authorized=False,live_connection_installed=False,
        request=request,mesh_generated=False)

def run(args):
    if inputs.sha(inputs.V4/"runtime/RELEASE.json")!=args.source_manifest_sha256:
        raise ValueError("ChangedOwnedReleaseBinding")
    if not (Path(args.request_dir)/"AUTHORIZATION.json").exists():
        raise ValueError("MissingSeparateLiveGrant")
    from finite_research import run as owned
    request=inputs.read(Path(args.request_dir)/"LIVE_REQUEST.json")
    return owned.main(["--request-dir",str(Path(args.request_dir).resolve()),*request["arguments"]])
