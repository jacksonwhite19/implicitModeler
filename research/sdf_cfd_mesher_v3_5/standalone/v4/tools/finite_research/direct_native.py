from contextlib import ExitStack
from datetime import datetime,timezone,timedelta
from pathlib import Path
import json,subprocess,time
from finite_research.direct_evidence_modules import driver,variant

def run(folder,cap_path,outer_path,policy_path,binding,pins,check,observed,mode,invocation):
    from finite_research import native_budget
    with native_budget.stage(mode) as ticket:
        return _run(folder,cap_path,outer_path,policy_path,binding,pins,check,observed,mode,invocation,ticket)

def _run(folder,cap_path,outer_path,policy_path,binding,pins,check,observed,mode,invocation,ticket):
    from finite_research import native_budget
    from finite_research.inputs import native_sources,V4
    from finite_support.retained_initializer import bounded_command,validate_retirement
    from finite_research.prism_connection import terminal_execution_context
    from immutable_state_access import guard
    import native_scope
    native_scope.require_transport()
    check()
    context,unused=terminal_execution_context()
    request_root=context.parent
    code=Path(__file__).resolve().parent/"direct_evidence_modules"
    spec=driver.read(code/"DEPENDENCIES.json")
    toolchain=Path(spec["inputs"]["LINUX_TOOLCHAIN"]["path"])
    binding_path=V4/"runtime/FINITE_NATIVE_BINDING.json"
    if pins.get(str(binding_path))!=driver.sha(binding_path) or pins.get(str(toolchain))!=driver.sha(toolchain):
        raise ValueError("UnboundDirectNativeDependencyManifest")
    external={row["external_path"]:row["sha256"] for row in driver.read(binding_path)["files"] if "external_path" in row}
    for name,digest in driver.read(toolchain)["files"].items():
        if name in external and external[name]!=digest:
            raise ValueError("ConflictingDirectNativeDependency")
        external[name]=digest
    if mode not in ("farfield","candidate"):
        raise ValueError("DirectNativeModeRequired")
    allowance=native_budget.remaining(ticket)
    if allowance<=30:
        raise TimeoutError("DirectNativeBudgetExhausted")
    selected=native_sources()
    selected[str(policy_path)]=driver.sha(policy_path)
    owners={}
    for name,path in (("task",request_root/"TASK.json"),("authorization",request_root/"AUTHORIZATION.json"),
        ("launch_review",request_root/"LAUNCH_REVIEW.json"),("invocation",invocation),
        ("clock",request_root/"CLOCK.json"),("input_pins",request_root/"live01_INPUT_PINS.json"),
        ("stage",request_root/"STAGE_REQUEST.json")):
        owners[name]=dict(path=str(path),sha256=driver.sha(path))
    inputs={"ORIGINAL_CAP_INPUT":dict(path=str(cap_path),sha256=driver.sha(cap_path),bytes=cap_path.stat().st_size)}
    if outer_path is not None:
        inputs["OUTER"]=dict(path=str(outer_path),sha256=driver.sha(outer_path),bytes=outer_path.stat().st_size)
    request=dict(schema="DirectNativeRequest/1",variant=variant.NAME,mode=mode,generation_calls=1,
        maximum_optimizer_calls=2 if mode=="candidate" else 0,seconds=allowance,
        expires_utc=(datetime.now(timezone.utc)+timedelta(seconds=allowance)).isoformat(),
        source_root=str(V4),pins=selected,inputs=inputs,owners=owners,binding=binding,publication_enabled=False,
        policy_path=str(policy_path),policy_sha256=driver.sha(policy_path),
        toolchain_path=str(toolchain),toolchain_sha256=driver.sha(toolchain),
        native_budget_reservation={key:ticket[key] for key in ('path','sha256')})
    if mode=="candidate":
        frozen=folder.parent/"FROZEN_BOUNDARY.json"
        request["frozen_boundary"]=dict(path=str(frozen),sha256=driver.sha(frozen))
    folder.mkdir()
    variant.write(folder/"OPTIMIZER_REQUEST.json",request)
    digest=driver.sha(folder/"OPTIMIZER_REQUEST.json")
    entry=code/("native_farfield.py" if mode=="farfield" else "native_optimize.py")
    def linux(path):
        value=str(path).replace("\\","/")
        return "/mnt/"+value[0].lower()+value[2:]
    environment=driver.read(toolchain)["execution_environment"]
    command=["wsl.exe","-e","/usr/bin/env",*[name+"="+value for name,value in sorted(environment.items())],
        "PYTHONDONTWRITEBYTECODE=1","/usr/bin/python3.10","-B",linux(code/"native_entry.py"),linux(folder),digest,entry.name]
    with ExitStack() as held:
        held.enter_context(guard(Path(ticket['path'])))
        for path in (folder/"OPTIMIZER_REQUEST.json",cap_path,policy_path,toolchain,invocation):
            held.enter_context(guard(path))
        if outer_path is not None:
            held.enter_context(guard(outer_path))
        for row in owners.values():
            held.enter_context(guard(Path(row["path"])))
        for name,digest_value in selected.items():
            if str(name)!=str(policy_path) and (
                external.get(str(name)) if str(name).startswith("/") else pins.get(str(name)))!=digest_value:
                raise ValueError("ChangedDirectNativeSelectedSource",name)
        primary=None
        linux_deadline=None
        try:
            linux_deadline=native_scope.downstream_deadline(ticket['deadline']-2)
            allowance=native_budget.remaining(ticket)
            command=bounded_command(command,allowance,folder/"NATIVE_RETIREMENT.json",linux_deadline=linux_deadline)
            variant.write(folder/"DISPATCH.json",dict(argv=command,seconds=allowance,mode=mode))
            with (folder/"stdout.txt").open("x") as stdout,(folder/"stderr.txt").open("x") as stderr:
                wait_seconds=ticket['deadline']-time.perf_counter()
                if wait_seconds<=0:raise TimeoutError('NativeHostTicketExpired')
                process=subprocess.run(command,stdout=stdout,stderr=stderr,timeout=wait_seconds)
        except BaseException as error:
            primary=error
            raise
        finally:
            try:
                validate_retirement(driver.read(folder/"NATIVE_RETIREMENT.json"),allowance,linux_deadline=linux_deadline)
            except BaseException as error:
                if primary is None:raise
                primary.add_note("Native retirement: "+repr(error))
        check()
        if process.returncode!=0:
            raise ValueError("DirectNativeFailed",mode,process.returncode)
        if not driver.read(folder/"NATIVE_ENVELOPE_COMPLETION.json")["passed"]:
            raise ValueError("DirectNativeEnvelopeFailed")
        result=driver.read(folder/("FARFIELD_RESULT.json" if mode=="farfield" else "OPTIMIZER_RESULT.json"))
        if not result["passed"] or result["request_sha256"]!=digest:
            raise ValueError("DirectNativeReceiptFailed")
        if driver.sha(folder/"OPTIMIZER_REQUEST.json")!=digest:
            raise ValueError("ChangedDirectNativeRequest")
        for row in inputs.values():
            if driver.sha(Path(row["path"]))!=row["sha256"]:
                raise ValueError("ChangedDirectNativeInput")
        if mode=="candidate":
            if result["generation_calls"]!=1 or result["optimizer_calls"]!=2:
                raise ValueError("DirectNativeCallCount")
            for name,digest_value in result["files"].items():
                if driver.sha(folder/"generated"/name)!=digest_value:
                    raise ValueError("ChangedDirectNativeArtifact",name)
            if not driver.read(folder/"NATIVE_TERMINAL_CHECKS.json")["passed"]:
                raise ValueError("DirectNativeTerminalFailure")
        elif result["generation_calls"]!=1 or not result["terminal"]["passed"]:
            raise ValueError("DirectFarfieldTerminalFailure")
    observed["direct_native_"+mode]=str(entry)
    return result
