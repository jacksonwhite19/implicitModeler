from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,os,subprocess,sys,time
code=Path(__file__).resolve().parent
output=Path(sys.argv[1]).resolve()
request_path=output/"NATIVE_REQUEST.json"
request=json.loads(request_path.read_text())
def local(name):
    value=str(name)
    return Path("/mnt/"+value[0].lower()+value[2:].replace("\\","/")) if value[1:2]==":" else Path(value)
toolchain=local(request["toolchain_path"])
manifest=json.loads(toolchain.read_text())
from native_contract import owners as verify_owners
def sha(path):
    digest=hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda:stream.read(1048576),b""):digest.update(block)
    return digest.hexdigest()
def verify():
    for name,digest in manifest["files"].items():
        if sha(name)!=digest:raise ValueError("ChangedInstalledToolchain",name)
    for name,digest in request["source_pins"].items():
        if sha(local(name))!=digest:raise ValueError("ChangedDirectExportSource",name)
    verify_owners(request)
    if sha(output.parent/"JOINED_EVIDENCE.npz")!=request["joined_archive_sha256"]:
        raise ValueError("ChangedDirectExportJoinedGeometry")
request_path=output/"NATIVE_REQUEST.json"
assert sha(request_path)==sys.argv[2]
request=json.loads(request_path.read_text())
assert request["generation_calls"]==0 and request["entry_sha256"]==sha(__file__)
assert request["toolchain_sha256"]==sha(toolchain)
assert request["transfer_sha256"]==sha(output/"complete_transfer.msh")
assert request["control_sha256"]==sha(output/"openfoam_case/system/controlDict")
with (output/"NATIVE_ENTRY_CONSUMED.json").open("x") as stream:json.dump(dict(request_sha256=sys.argv[2],pid=os.getpid()),stream)
verify()
records={}
# All export commands consume the original shared reservation; no per-command reset.
reservation=json.loads(local(request['native_budget_reservation']['path']).read_text())
command_deadline=time.monotonic()+min(request['seconds'],
    (datetime.fromisoformat(reservation['expires_utc'])-datetime.now(timezone.utc)).total_seconds())

def remaining_command_seconds():
    remaining=command_deadline-time.monotonic()
    if remaining<=0:raise TimeoutError('DirectExportReservationExpired')
    return remaining

environment=manifest["execution_environment"]
assert environment["WM_PROJECT_VERSION"]==manifest["openfoam_version"]
assert environment["LD_LIBRARY_PATH"]==manifest["ld_library_path"]
def runtime_closure(name):
    loaded={str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
        if getattr(module,"__file__",None) and Path(module.__file__).is_file()}
    allowed=dict(manifest["files"]);allowed[str(Path(__file__).resolve())]=request["entry_sha256"]
    for source_name,digest in request["source_pins"].items():
        source_name=str(local(source_name))
        if source_name in allowed and allowed[source_name]!=digest:raise ValueError("ConflictingDirectExportPin",source_name)
        allowed[source_name]=digest
    missing=sorted(loaded-set(allowed))
    files={path:sha(path) for path in loaded}
    changed=[path for path,digest in files.items() if allowed.get(path)!=digest]
    closure=dict(files=files,unpinned=missing,changed=changed,declared_toolchain_files_verified=len(manifest["files"]))
    with (output/name).open("x") as stream:json.dump(closure,stream,indent=2)
    if missing or changed:raise ValueError("UnpinnedNativePythonRuntime",missing,changed)
    return closure
runtime_closure("NATIVE_PREEXEC_CLOSURE.json")
primary=None
try:
    resolved={}
    for executable,expected in manifest["declared_elf_dependencies"].items():
        result=subprocess.run(["/usr/bin/ldd",executable],capture_output=True,text=True,env=environment,timeout=30,check=True)
        actual=sorted({str(Path(word).resolve()) for line in result.stdout.splitlines() for word in line.split() if word.startswith("/") and Path(word).is_file()})
        assert actual==sorted(set(expected["files"])),("NativeDependencyResolutionChanged",executable,actual)
        resolved[executable]=actual
    with (output/"NATIVE_ENVIRONMENT.json").open("x") as stream:json.dump(dict(environment=environment,resolved=resolved),stream,indent=2)
    case=output/"openfoam_case";audit=output/"foundation_audit";audit.mkdir()
    commands=[("gmshToFoam",[manifest["converter"],str(output/"complete_transfer.msh"),"-case",str(case)]),
        ("foundationAudit",[manifest["audit"],"-case",str(case),"-outputDir",str(audit)]),
        ("checkMesh",[manifest["checkmesh"],"-constant","-allGeometry","-allTopology","-case",str(case)])]
    for name,argv in commands:
        before=time.monotonic()
        with (output/(name+".stdout")).open("x") as stdout,(output/(name+".stderr")).open("x") as stderr:
            result=subprocess.run(argv,env=environment,stdout=stdout,stderr=stderr,timeout=remaining_command_seconds())
        records[name]=dict(argv=argv,returncode=result.returncode,seconds=time.monotonic()-before,
            stdout_sha256=sha(output/(name+".stdout")),stderr_sha256=sha(output/(name+".stderr")))
        if name in ("gmshToFoam","foundationAudit") and result.returncode!=0:break
    verify()
    closure=runtime_closure("NATIVE_LOADED_CLOSURE.json")
    with (output/"NATIVE_COMMANDS.json").open("x") as stream:json.dump(dict(commands=records,toolchain_sha256=sha(toolchain),
        mesh_quality="NOT_RUN_NO_APPLICABLE_APPROVED_DICTIONARY",closure_passed=True),stream,indent=2)
    if any(row["returncode"]!=0 for name,row in records.items() if name!="checkMesh"):raise ValueError("ConversionOrAuditExecutionFailed")
    
except BaseException as error:
    primary=error
    raise
finally:
    errors=[]
    for label,operation in (("toolchain",verify),("closure",lambda:runtime_closure("NATIVE_TERMINAL_CLOSURE.json"))):
        try:
            operation()
        except BaseException as error:
            errors.append(dict(stage=label,error=repr(error)))
    with (output/"NATIVE_TERMINAL_CHECKS.json").open("x") as stream:
        json.dump(dict(passed=not errors,errors=errors),stream)
    if errors:
        if primary is None:
            raise ValueError("NativeExportTerminalFailure",errors)
        if hasattr(primary,"add_note"):
            primary.add_note(repr(errors))
        else:
            primary.direct_terminal_errors=errors
