from pathlib import Path
import ast,json,os,shutil,subprocess,sys,time,traceback
code=Path(__file__).resolve().parent
if not __debug__ or sys.flags.optimize:raise RuntimeError("AssertionsRequired")
output=Path(sys.argv[1]).resolve();request_path=output/"OPTIMIZER_REQUEST.json"
import variant
def timed(label,operation):
    started=time.monotonic()
    variant.write(output/("TIMING_"+label+"_BEFORE.json"),dict(stage=label,monotonic=started,pid=os.getpid()))
    try:
        value=operation()
    except BaseException as error:
        variant.write(output/("TIMING_"+label+"_AFTER.json"),dict(stage=label,seconds=time.monotonic()-started,passed=False,error=repr(error)))
        raise
    record=dict(stage=label,seconds=time.monotonic()-started,passed=True)
    if label.startswith("runtime_"):record["hash_measurements"]=native_runtime.hash_measurements()
    variant.write(output/("TIMING_"+label+"_AFTER.json"),record)
    return value

request=json.loads(request_path.read_text())
assert variant.sha(request_path)==sys.argv[2]
timed("request_"+str(len(list(output.glob("TIMING_request_*_BEFORE.json")))),lambda:variant.verify_request(request,request_path,sys.argv[2]))
source=variant.local(request["source_root"])
sys.path[:0]=[str(source/"tools/finite_support/native"),str(source/"tools"),str(source/"src"),str(source/"runtime/lib/gmsh")]
import native_runtime
timed("runtime_initial",native_runtime.verify)
import numpy as np
import core_backend as core
import gmsh
from gmsh_core_setup import setup_profile
from types import SimpleNamespace
from finite_research.inputs import native_sources
expected={str(variant.local(name).resolve()):digest for name,digest in native_sources().items()}
allowed={str(variant.local(name).resolve()):digest for name,digest in request["pins"].items()}
assert all(allowed.get(name)==digest for name,digest in expected.items())
for name,digest in json.loads(variant.local(request["toolchain_path"]).read_text())["files"].items():
    if name in allowed and allowed[name]!=digest:
        raise ValueError("ConflictingDirectRuntimePin",name)
    allowed[name]=digest
for name,digest in allowed.items():assert variant.sha(name)==digest,name
def closure(name):
    files={str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
        if getattr(module,"__file__",None) and Path(module.__file__).is_file()}
    observed,changed=variant.observe_files(files,allowed)
    record=dict(files=observed,unpinned_or_changed=changed)
    variant.write(output/name,record);assert not changed,changed
    return record
timed("closure_initial",lambda:closure("OPTIMIZER_PREEXEC_CLOSURE.json"))
dependencies={}
from background_query_build import record as query_build_record
native_libraries=[query_build_record(kind)[1] for kind in ("query","direct")]
for executable in (str(Path(sys.executable).resolve()),str(Path(gmsh.lib._name).resolve()),*[str(path.resolve()) for path in native_libraries]):
    result=subprocess.run(["/usr/bin/ldd",executable],capture_output=True,text=True,timeout=30,check=True)
    assert "not found" not in result.stdout
    files=sorted({str(Path(word).resolve()) for line in result.stdout.splitlines() for word in line.split() if word.startswith("/") and Path(word).is_file()})
    assert all(path in allowed and variant.sha(path)==allowed[path] for path in files),files
    dependencies[executable]=files
variant.write(output/"OPTIMIZER_ENVIRONMENT.json",dict(environment=dict(os.environ),dependencies=dependencies))
timed("request_"+str(len(list(output.glob("TIMING_request_*_BEFORE.json")))),lambda:variant.verify_request(request,request_path,sys.argv[2]))
variant.write(output/"OPTIMIZER_ENTRY_CONSUMED.json",dict(variant=variant.NAME,mode=request["mode"],request_sha256=sys.argv[2],pid=os.getpid()))
started=time.monotonic();calls=[];settings=[];result=dict(passed=False,variant=variant.NAME,request_sha256=sys.argv[2],generation_calls=0,optimizer_calls=0,scientific_acceptance=False)
original_optimize=gmsh.model.mesh.optimize

import generation_profile
profile=None;generation_calls=0
try:
    tests=dict(reload=variant.focused_tests(),profile=generation_profile.focused_tests())
    assert gmsh.__version__=="4.15.2"
    policy=json.loads(variant.local(request["policy_path"]).read_text())
    from finite_research.direct_contract import validate_policy
    validate_policy(policy)
    shape_policy=dict(tet_transition=dict(quality_targets=policy["quality_targets"]))
    with np.load(variant.local(request["inputs"]["ORIGINAL_CAP_INPUT"]["path"]),allow_pickle=False) as archive:
        cap={name:archive[name].copy() for name in ("points","triangles","seeds")}
    with np.load(variant.local(request["inputs"]["OUTER"]["path"]),allow_pickle=False) as archive:
        outer={name:archive[name].copy() for name in ("points","triangles")}
    if request["mode"]=="preflight":
        faces=np.concatenate((cap["triangles"],outer["triangles"]+len(cap["points"])))
        reference=timed("cycle_reference",lambda:sorted(min(tuple(np.roll(face,offset)) for offset in range(3)) for face in faces))
        repaired=timed("cycle_repaired",lambda:generation_profile.cycle_keys(faces))
        assert reference==repaired
        variant.write(output/"CYCLE_EQUIVALENCE.json",dict(passed=True,faces=len(faces),exact_equal=True))
    cap_bytes={name:value.tobytes() for name,value in cap.items()}
    outer_bytes={name:value.tobytes() for name,value in outer.items()}
    def entering_generation(record):
        global generation_calls
        assert generation_calls==0 and request["mode"]=="candidate"
        timed("request_"+str(len(list(output.glob("TIMING_request_*_BEFORE.json")))),lambda:variant.verify_request(request,request_path,sys.argv[2]))
        for name,value in cap.items():assert value.tobytes()==cap_bytes[name]
        for name,value in outer.items():assert value.tobytes()==outer_bytes[name]
        variant.write(output/"GENERATION_ENTERED.json",dict(record,request_sha256=sys.argv[2],
            cap_sha256=request["inputs"]["ORIGINAL_CAP_INPUT"]["sha256"],outer_sha256=request["inputs"]["OUTER"]["sha256"]))
        generation_calls=1
    profile=generation_profile.Profile(gmsh,cap,policy["growth_per_m"],policy["maximum_size_m"],entering_generation)
    gmsh.initialize([],False);profile.setup(gmsh);gmsh.model.add("retained_farfield_session")
    gmsh.model.addDiscreteEntity(2,2)
    gmsh.model.mesh.addNodes(2,2,np.arange(1,len(outer["points"])+1),outer["points"].ravel())
    gmsh.model.mesh.addElementsByType(2,2,np.arange(1,len(outer["triangles"])+1),(outer["triangles"]+1).ravel())
    profile.install()
    empty=dict(points=np.empty((0,3),dtype="<f8"),triangles=np.empty((0,3),dtype="<i8"))
    reused=timed("farfield_boundary",lambda:generation_profile.surface_check(gmsh,empty,outer))
    variant.write(output/"FARFIELD_REUSE.json",dict(reused,source_sha256=request["inputs"]["OUTER"]["sha256"],
        generation_calls=0,explicit_2d_generation=False,orientation_arithmetic_repeated=False))
    profile.finalize()
    gmsh.initialize([],False);profile.setup(gmsh)
    before=timed("boundary_import",lambda:generation_profile.import_boundary(gmsh,cap,outer))
    profile.install()
    variant.write(output/"BOUNDARY_PRE.json",dict(before,cap_sha256=request["inputs"]["ORIGINAL_CAP_INPUT"]["sha256"],
        outer_sha256=request["inputs"]["OUTER"]["sha256"],method=generation_profile.NAME))
    timed("closure_pregeneration",lambda:closure("GENERATION_PREEXEC_CLOSURE.json"));timed("runtime_pregeneration",native_runtime.verify)
    if request["mode"]=="candidate":
        generated=output/"generated";generated.mkdir()
        shutil.copy2(variant.local(request["inputs"]["OUTER"]["path"]),generated/"OUTER.npz")
        boundary_before=generation_profile.boundary_snapshot(gmsh)
        variant.write(output/"BOUNDARY_SNAPSHOT_BEFORE.json",boundary_before)
        assert generation_profile.boundary_difference(boundary_before,boundary_before)["passed"],"PreGenerationSnapshotInvalid"
        callback_before=profile.callback.report();algorithm_before=gmsh.option.getNumber("Mesh.Algorithm3D")
        gmsh.logger.start()
        generation_error=None
        try:
            tick=time.monotonic();profile.generate();generation_seconds=time.monotonic()-tick
        except BaseException as error:
            generation_error=error
        finally:
            messages=gmsh.logger.get();gmsh.logger.stop()
            trace=dict(messages=messages,callback_before=callback_before,callback_after=profile.callback.report(),
                algorithm_before=algorithm_before,algorithm_after=gmsh.option.getNumber("Mesh.Algorithm3D"))
            variant.write(output/"ALGORITHM_TRACE.json",trace)
        execution=generation_profile.execution_evidence(trace)
        variant.write(output/"ALGORITHM_EXECUTION.json",execution)
        captured={}
        def capture_raw():
            points,tets=core.arrays(gmsh);captured.update(points=points,tets=tets)
            np.savez(generated/"CORE.npz",points=points,tets=tets)
            variant.write(output/"RAW_CORE_CAPTURE.json",dict(accepted_at_capture=False,archive_sha256=variant.sha(generated/"CORE.npz"),
                acceptance_authority="Only a passed generated/GENERATION.json can accept this raw diagnostic archive",points=len(points),tetrahedra=len(tets)))
        def capture_boundary():
            boundary_after=generation_profile.boundary_snapshot(gmsh)
            variant.write(output/"BOUNDARY_SNAPSHOT_AFTER.json",boundary_after)
            difference=generation_profile.boundary_difference(boundary_before,boundary_after);captured["difference"]=difference
            variant.write(output/"BOUNDARY_DIFFERENCE.json",difference)
        captures=generation_profile.terminal_checks([("raw",capture_raw),("boundary",capture_boundary)])
        variant.write(output/"GENERATION_CAPTURE_STATUS.json",dict(captures,generation_api_returned=generation_error is None,
            generation_error=repr(generation_error) if generation_error is not None else None))
        if generation_error is not None:raise generation_error
        assert captures["passed"],"GenerationDiagnosticCaptureFailed"
        points=captured["points"];tets=captured["tets"];difference=captured["difference"]
        assert execution["passed"],"DelaunayExecutionEvidenceRefused"
        assert difference["passed"],"ExactPostGenerationBoundaryRefused"
        stage_shapes=dict(CORE=generation_profile.stage_shape(core,points,tets,cap,outer,shape_policy))
        variant.write(output/"CORE_BOUNDARY_AND_SHAPE.json",stage_shapes["CORE"])
        generation=dict(passed=True,method=generation_profile.NAME,generation_calls=1,dimension=3,factor=.95,algorithm=1,
            boundary_difference_sha256=variant.sha(output/"BOUNDARY_DIFFERENCE.json"),
            boundary_before_sha256=variant.sha(output/"BOUNDARY_SNAPSHOT_BEFORE.json"),boundary_after_sha256=variant.sha(output/"BOUNDARY_SNAPSHOT_AFTER.json"),
            raw_capture_sha256=variant.sha(output/"RAW_CORE_CAPTURE.json"),
            algorithm_execution_sha256=variant.sha(output/"ALGORITHM_EXECUTION.json"),algorithm_trace_sha256=variant.sha(output/"ALGORITHM_TRACE.json"),
            seconds=generation_seconds,core_sha256=variant.sha(generated/"CORE.npz"),
            outer_sha256=variant.sha(generated/"OUTER.npz"),boundary=stage_shapes["CORE"],
            cap_sha256=request["inputs"]["ORIGINAL_CAP_INPUT"]["sha256"],
            growth_per_m=policy["growth_per_m"],maximum_size_m=policy["maximum_size_m"],
            callback_identity=profile.input_identity,pre_boundary_sha256=variant.sha(output/"BOUNDARY_PRE.json"),
            generation_entry_sha256=variant.sha(output/"GENERATION_ENTERED.json"))
        variant.write(generated/"GENERATION.json",generation)
        profile.finalize()
        namespace=dict(Path=Path,np=np,time=time,setup=core.setup,arrays=core.arrays,
            file_hash=core.file_hash,durable_json=core.durable_json,boundary_audit=core.boundary_audit,exact_reload=variant.exact_reload)
        operation=variant.optimizer_function(source/"tools/core_backend.py",namespace)
        def completed_optimizer(name):
            current_points,current_tets=core.arrays(gmsh)
            checked=generation_profile.stage_shape(core,current_points,current_tets,cap,outer,shape_policy)
            variant.write(output/("BOUNDARY_AFTER_"+name.upper()+".json"),checked)
        optimize_once=variant.guarded_optimizer(gmsh,original_optimize,calls,
            lambda row:variant.write(output/("OPTIMIZER_"+row["name"].upper()+"_ENTERED.json"),row),completed_optimizer)
        gmsh.model.mesh.optimize=optimize_once
        optimized=operation(generated,cap["points"],cap["triangles"],execution=SimpleNamespace(setup=profile.setup))
        assert len(calls)==2 and all(row["completed"] for row in calls)
        for name in ("GMSH","NETGEN"):
            with np.load(generated/(name+".npz"),allow_pickle=False) as archive:
                stage_shapes[name]=generation_profile.stage_shape(core,archive["points"],archive["tets"],cap,outer,shape_policy)
        variant.write(generated/"STAGE_SHAPE.json",stage_shapes)
        method=profile.finish(True);settings=method["settings"]
        method["selection"]=dict(request_sha256=sys.argv[2],profile_sha256=variant.sha(code/"generation_profile.py"),
            policy_sha256=variant.sha(variant.local(request["policy_path"])),generation_receipt_sha256=variant.sha(generated/"GENERATION.json"),
            cap_sha256=request["inputs"]["ORIGINAL_CAP_INPUT"]["sha256"],outer_sha256=request["inputs"]["OUTER"]["sha256"])
        variant.write(generated/"METHOD.json",method)
        result.update(optimization=optimized,method=method,files={name:variant.sha(generated/name) for name in
            ("CORE.npz","OUTER.npz","NETGEN.npz","GMSH.npz","PRECONDITIONED.msh","OPTIMIZATION.json","GENERATION.json","STAGE_SHAPE.json","METHOD.json")})
    else:
        assert generation_calls==0 and not calls
        tests["fixed_boundary_import"]=timed("boundary_repeat",lambda:generation_profile.surface_check(gmsh,cap,outer))
        profile.finalize()
        method=profile.finish(False);settings=method["settings"];tests["method"]=method
    for name,value in cap.items():assert value.tobytes()==cap_bytes[name]
    for name,value in outer.items():assert value.tobytes()==outer_bytes[name]
    result.update(passed=True,generation_calls=generation_calls,optimizer_calls=len(calls),calls=calls,settings=settings,focused_tests=tests)
except BaseException as error:
    result.update(error=repr(error),traceback=traceback.format_exc(),generation_calls=generation_calls,optimizer_calls=len(calls),calls=calls,settings=settings)
finally:
    gmsh.model.mesh.optimize=original_optimize
    try:
        if profile is not None:profile.close()
        elif gmsh.isInitialized():gmsh.finalize()
    except BaseException as error:
        result.update(passed=False,cleanup_error=repr(error))
    def check_dependencies():
        for files in dependencies.values():
            for path in files:assert variant.sha(path)==allowed[path],path
    terminal=generation_profile.terminal_checks([
        ("request",lambda:timed("request_final",lambda:variant.verify_request(request,request_path,sys.argv[2],fresh=False))),
        ("runtime",lambda:timed("runtime_final",native_runtime.verify)),
        ("dependencies",check_dependencies),
        ("closure",lambda:timed("closure_final",lambda:closure("OPTIMIZER_LOADED_CLOSURE.json")))])
    variant.write(output/"NATIVE_TERMINAL_CHECKS.json",terminal)
    if not terminal["passed"]:result.update(passed=False,terminal_errors=terminal)
    result["seconds"]=time.monotonic()-started
    variant.write(output/"OPTIMIZER_RESULT.json",result)
raise SystemExit(0 if result["passed"] else 1)
