from contextlib import contextmanager,ExitStack
from contextvars import ContextVar
from pathlib import Path
import json,threading
from finite_research.direct_contract import validate_policy,identity,METHOD

SCHEMA="DirectEvidenceStageRequest/1"
_returned_binding=ContextVar("direct_evidence_return_binding",default=None)

@contextmanager
def selected_direct_modules(pins):
    import importlib
    from types import FunctionType
    from finite_research.inputs import V4,sha
    from immutable_state_access import guard
    modules={}
    functions={}
    names=("direct_runtime","direct_evidence","direct_native","native_budget","direct_contract","direct_readback",
        "direct_publication","direct_consumer","direct_admission","direct_fill_contract","direct_shared_apex","direct_repair_custody",
        "direct_evidence_modules.driver","direct_evidence_modules.background_audit",
        "direct_evidence_modules.joined_audit","direct_evidence_modules.shape",
        "direct_evidence_modules.collision",
        "direct_evidence_modules.export_adapter","direct_evidence_modules.variant")
    with ExitStack() as held:
        for name in names:
            module=importlib.import_module("finite_research."+name)
            path=V4/"tools/finite_research"/(name.replace(".","/")+".py")
            held.enter_context(guard(path))
            if Path(module.__file__).resolve()!=path or pins.get(str(path))!=sha(path):
                raise ValueError("UnboundDirectModule",name)
            modules[name]=module
            for symbol,value in vars(module).items():
                if isinstance(value,(FunctionType,type)):
                    functions[name,symbol]=value
        def attest():
            for name,module in modules.items():
                if importlib.import_module("finite_research."+name) is not module:
                    raise ValueError("ChangedDirectModuleIdentity",name)
            for (name,symbol),value in functions.items():
                if getattr(modules[name],symbol,None) is not value:
                    raise ValueError("ChangedDirectFunctionIdentity",name,symbol)
        attest()
        primary=None
        try:
            yield attest
            attest()
        except BaseException as error:
            primary=error
            raise
        finally:
            try:
                attest()
            except BaseException as error:
                if primary is None:raise
                primary.add_note("Direct helper cleanup: "+repr(error))

def stage_request(prepared,case,increments,policy):
    from finite_research.input_contract import producer_flags
    validate_policy(policy)
    from finite_research.partial_stages import choice
    stop=choice(prepared)
    flags=producer_flags(dict(prism_producer_mode="source_directions",execution_target="direct_complete",
        upstream_forecast_diagnostics=False),case,increments)
    from finite_research.direct_fill_contract import binding
    return dict(schema=SCHEMA,acceptance_contract=binding(policy),stop_after=stop,execution_target="direct_complete",prism_producer_mode="source_directions",
        upstream_case_sha256=prepared["case_sha256"],upstream_effective_identity=identity(case),
        upstream_geometry_identity=identity(case["geometry"]),prism_schedule_identity=identity(case["prisms"]),
        upstream_flags=flags,maximum_faces=prepared["maximum_faces"],downstream_policy=policy,
        downstream_policy_identity=identity(policy),source_release_sha256=prepared["source_release_sha256"],
        publication_enabled=False,scientific_acceptance=False,continuous_sdf_fluid="NOT_PERFORMED")

def validate_request(value,case,increments,case_sha256,release_sha256):
    expected=stage_request(dict(case_sha256=case_sha256,maximum_faces=value.get("maximum_faces"),
        source_release_sha256=release_sha256,stop_after=value.get("stop_after")),case,increments,value.get("downstream_policy"))
    if value!=expected or type(value["maximum_faces"]) is not int or value["maximum_faces"]<=0:
        raise ValueError("ChangedDirectStageRequest")
    return value

def preflight(run,handoff,value):
    from rolling_fast_state import BackgroundInputs
    if type(handoff) is not BackgroundInputs:
        raise ValueError("GenuineTerminalPrismInputsRequired")
    handoff.require_origin(run)
    handoff.require(handoff.view)
    validate_policy(value["downstream_policy"])
    if run.provider.layer!=value["upstream_flags"]["through_layer"]:
        raise ValueError("ChangedDirectTerminalLayer")
    return dict(passed=True,backend=dict(method=METHOD),publication_enabled=False,
        continuous_sdf_fluid="NOT_PERFORMED",geometry_assurance="all_twelve_fresh")

def evidence_only(result,output):
    expected_binding=_returned_binding.get()
    if (result.get("schema")!="DirectEvidenceHandoff/1" or result.get("passed") is not True
        or result.get("execution_target")!="direct_complete" or result.get("publication_enabled") is not False
        or result.get("scientific_acceptance") is not False
        or result.get("continuous_sdf_fluid")!="NOT_PERFORMED" or result.get("binding")!=expected_binding
        or expected_binding is None or result.get("scope_exit_verified") is not False or result.get("provisional") is not True):
        raise ValueError("InvalidDirectEvidenceHandoff")
    from path_preserving_deduplication import durable_json
    durable_json(Path(output)/"DIRECT_EVIDENCE_PROVISIONAL.json",result)
    return dict(provisional=True,publication_enabled=False,scientific_acceptance=False)

@contextmanager
def terminal_dispatch(case_path,request_path,pins,outer_check,observed=None):
    from finite_research.prism_connection import _active_background,selected,terminal_clock
    from finite_research import direct_evidence
    import standalone_tool as cli
    import standalone_background as background
    from standalone_case import hold_inputs,load
    from immutable_state_access import guard
    from rolling_fast_state import BackgroundInputs
    from production_surface_authority import load_production_wall
    from finite_research.inputs import V4,sha
    import sys
    stage=sys.modules["finite_collar_stage_adapter"]
    with ExitStack() as held:
        helpers,attest=held.enter_context(stage.selected_helpers(pins,observed))
        held.enter_context(guard(Path(request_path)))
        if pins.get(str(Path(request_path)))!=sha(request_path):
            raise ValueError("UnboundDirectStageRequest")
        held.enter_context(hold_inputs(case_path))
        case,increments,case_hash=load(case_path)
        value=validate_request(json.loads(Path(request_path).read_text()),case,increments,case_hash,sha(V4/"runtime/RELEASE.json"))
        operation=direct_evidence.execute
        direct_attest=held.enter_context(selected_direct_modules(pins))
        held.enter_context(selected(request_path,sha(request_path),outer_check))
        original_background,original_surface=background.execute,cli.surface_stage
        called=False
        completed=None
        binding_token=_returned_binding.set(None)
        def surface(*args,**kwargs):
            outer_check()
            if (cli.prism_producer_mode()!="source_directions" or cli._upstream_forecast_diagnostics.get() is not False
                or cli.surface_assurance()!="numerical" or cli.carrier_assessment_enabled() or cli.transition_only()):
                raise ValueError("ChangedDirectUpstreamInvocation")
            if cli.stop_after()!=value["stop_after"]:raise ValueError("ChangedRequestedStage")
            result=original_surface(*args,**kwargs)
            if len(load_production_wall(result[0])[1])>value["maximum_faces"]:
                raise ValueError("DirectSurfaceFaceCap")
            outer_check()
            return result
        def dispatch(run,actual_case_path,folder,expected_backend,*,_handoff,_preparation,_transition_only):
            nonlocal called,completed
            outer_check()
            attest()
            direct_attest()
            if value["stop_after"]!="complete":raise ValueError("DownstreamDispatchForbiddenByRequest")
            if called or _transition_only is not False or type(_handoff) is not BackgroundInputs:
                raise ValueError("OneGenuineDirectTerminalRequired")
            if direct_evidence.execute is not operation:
                raise ValueError("ChangedDirectEvidenceProducer")
            _handoff.require_origin(run)
            _handoff.require(_handoff.view)
            if _handoff._open()["preparation"] is not _preparation:
                raise ValueError("ChangedDirectPreparation")
            actual,actual_increments,actual_hash=load(actual_case_path)
            if identity(actual)!=identity(case):
                raise ValueError("ChangedDirectTerminalCase")
            connection=run.provider.binding.metric_connection
            connection.verify(run.provider.binding)
            if connection.source_identity!=identity(actual):
                raise ValueError("ChangedDirectCanonicalSource")
            flags=value["upstream_flags"]
            if (cli.prism_producer_mode()!="source_directions" or cli.surface_assurance()!=flags["surface_assurance"]
                or cli.carrier_assessment_enabled() or cli.transition_only()
                or cli._upstream_forecast_diagnostics.get() is not False or run.provider.layer!=len(increments)
                or len(_handoff.view.outer)>value["maximum_faces"]):
                raise ValueError("ChangedDirectUpstreamInvocation")
            called=True
            clock=terminal_clock(outer_check)
            binding=dict(request_sha256=sha(request_path),case_sha256=case_hash,
                canonical_source_identity=identity(case["geometry"]),prism_schedule_identity=identity(case["prisms"]),
                execution_target="direct_complete",effective_case_identity=identity(case),
                geometry_authority_identity=run.provider.binding.geometry_identity,
                parent_prism_mesh=_handoff.view.live_acceptance.mesh,
                source_release_sha256=sha(V4/"runtime/RELEASE.json"),
                policy_identity=value["downstream_policy_identity"])
            expected_binding=json.dumps(binding,sort_keys=True,allow_nan=False)
            result=operation(run,Path(folder),_handoff,_preparation,value["downstream_policy"],json.loads(expected_binding),
                pins,helpers,attest,clock,observed if observed is not None else {})
            clock()
            attest()
            direct_attest()
            _handoff.require(_handoff.view)
            _handoff.require_origin(run)
            if result.get("binding")!=json.loads(expected_binding):
                raise ValueError("ChangedDirectEvidenceReturnBinding")
            _returned_binding.set(json.loads(expected_binding))
            from finite_research.direct_consumer import validate_provisional
            completed=validate_provisional(Path(folder),json.loads(expected_binding),pins,helpers,clock,observed)
            return result
        background.execute,cli.surface_stage=dispatch,surface
        primary=None
        try:
            yield
            if cli.stop_after()!="complete":
                # CLI contexts reset after return; use the pinned request below.
                raise ValueError("UnclosedCLIStageContext")
        except BaseException as error:
            primary=error
            raise
        finally:
            _returned_binding.reset(binding_token)
            changed=background.execute is not dispatch or cli.surface_stage is not surface
            background.execute,cli.surface_stage=original_background,original_surface
            if changed:
                error=ValueError("ChangedDirectScopedDispatch")
                if primary is None:
                    raise error
                primary.add_note(str(error))
    if value["stop_after"]!="complete":
        if called:raise ValueError("UnexpectedPartialBulkDispatch")
        from finite_research.partial_stages import validate
        validate(Path(json.loads((Path(request_path).parent/"LIVE_REQUEST.json").read_text())["output"]),value["stop_after"],case_hash,len(increments),sha(V4/"runtime/RELEASE.json"),sha(request_path))
    if called:
        if completed is None:
            raise ValueError("MissingDirectReadbackCompletion")
        from path_preserving_deduplication import durable_json
        import os
        path=Path(completed["folder"]).parent/"DIRECT_SCOPE_EXIT.json"
        if path.exists():
            raise ValueError("DirectScopeExitReplay")
        durable_json(path,dict(schema="DirectSuccessfulScopeExit/1",pid=os.getpid(),
            result=completed,request_sha256=sha(request_path),source_release_sha256=sha(V4/"runtime/RELEASE.json"),
            provisional=True,scientific_acceptance=False,publication_enabled=False))
