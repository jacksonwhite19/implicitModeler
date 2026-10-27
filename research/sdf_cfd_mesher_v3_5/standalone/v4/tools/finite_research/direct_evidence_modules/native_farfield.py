from pathlib import Path
import json,sys,time,traceback,subprocess
import variant

output=Path(sys.argv[1]).resolve()
request_path=output/"OPTIMIZER_REQUEST.json"
request=json.loads(request_path.read_text())
variant.verify_request(request,request_path,sys.argv[2])
if request["mode"]!="farfield":
    raise ValueError("SeparateUpstreamFarfieldRequired")
source=variant.local(request["source_root"])
sys.path[:0]=[str(source/"tools/finite_support/native"),str(source/"tools"),str(source/"src"),
    str(source/"runtime/lib/gmsh")]
import native_runtime
native_runtime.verify()
import numpy as np
import gmsh
import core_backend as core
from generic_core_sizing import add_analytic_box,GmshSizingAdapter
from optimized_core_sizing import ExactCachedSizingField
from finite_research.direct_contract import validate_policy
from finite_research.direct_recipe155 import terminal_checks
from finite_research.direct_evidence_modules.farfield_callback import CheckedCallback

allowed={str(variant.local(name).resolve()):digest for name,digest in request["pins"].items()}
for name,digest in json.loads(variant.local(request["toolchain_path"]).read_text())["files"].items():
    if name in allowed and allowed[name]!=digest:
        raise ValueError("ConflictingFarfieldRuntimePin",name)
    allowed[name]=digest
def closure(name):
    files={str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
        if getattr(module,"__file__",None) and Path(module.__file__).is_file()}
    observed,changed=variant.observe_files(files,allowed)
    variant.write(output/name,dict(files=observed,unpinned_or_changed=changed))
    if changed:
        raise ValueError("UnpinnedFarfieldRuntime",changed)

policy=validate_policy(json.loads(variant.local(request["policy_path"]).read_text()))
with np.load(variant.local(request["inputs"]["ORIGINAL_CAP_INPUT"]["path"]),allow_pickle=False) as archive:
    cap={name:archive[name].copy() for name in ("points","triangles","seeds")}
field=ExactCachedSizingField(cap["points"][cap["triangles"]],cap["seeds"],
    g=policy["growth_per_m"],h_max=policy["maximum_size_m"])
callback=CheckedCallback(core.MeasuredCallback(field))
dependencies=set()
for executable in (str(Path(sys.executable).resolve()),str(Path(gmsh.lib._name).resolve())):
    linked=subprocess.run(["/usr/bin/ldd",executable],capture_output=True,text=True,timeout=30,check=True)
    if "not found" in linked.stdout:
        raise ValueError("MissingFarfieldNativeDependency")
    dependencies.update(str(Path(word).resolve()) for line in linked.stdout.splitlines() for word in line.split()
        if word.startswith("/") and Path(word).is_file())
def verify_dependencies():
    for name in dependencies:
        if allowed.get(name)!=variant.sha(name):
            raise ValueError("ChangedFarfieldNativeDependency",name)
verify_dependencies()
closure("FARFIELD_PREEXEC_CLOSURE.json")
result=dict(passed=False,dimension=2,upstream_preparation=True,generation_calls=0,publication_enabled=False)
primary=None
try:
    core.locks()
    gmsh.initialize([],False)
    core.setup(gmsh)
    gmsh.model.add("generic_direct_upstream_farfield")
    add_analytic_box(gmsh,np.asarray(policy["bounds_m"]))
    gmsh.model.occ.synchronize()
    GmshSizingAdapter(field).install(gmsh)
    gmsh.model.mesh.setSizeCallback(callback)
    variant.verify_request(request,request_path,sys.argv[2])
    variant.write(output/"FARFIELD_GENERATION_ENTERED.json",dict(request_sha256=sys.argv[2],dimension=2))
    result["generation_calls"]=1
    gmsh.model.mesh.generate(2)
    callback_record=callback.require()
    points,faces=core.surface_arrays(gmsh)
    centre=np.asarray(policy["bounds_m"]).mean(axis=1)
    triangles=points[faces]
    flip=np.einsum("ij,ij->i",np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),triangles.mean(1)-centre)<0
    faces[flip]=faces[flip][:,[0,2,1]]
    from standalone_background import check_box
    check_box(points,faces,np.asarray(policy["bounds_m"]))
    np.savez(output/"OUTER.npz",points=points,triangles=faces)
    result.update(passed=True,outer_sha256=variant.sha(output/"OUTER.npz"),
        request_sha256=sys.argv[2],callback=callback_record,bounds_m=policy["bounds_m"])
except BaseException as error:
    primary=error
    result.update(error=repr(error),traceback=traceback.format_exc())
finally:
    def finish():
        if gmsh.isInitialized():
            gmsh.model.mesh.removeSizeCallback()
            gmsh.finalize()
    terminal=terminal_checks([("gmsh",finish),("request",lambda:variant.verify_request(request,request_path,sys.argv[2],fresh=False)),
        ("runtime",native_runtime.verify),("dependencies",verify_dependencies),
        ("closure",lambda:closure("FARFIELD_SOURCE_CLOSURE.json"))])
    result["terminal"]=terminal
    result["passed"]=result["passed"] and terminal["passed"]
    variant.write(output/"FARFIELD_RESULT.json",result)
raise SystemExit(0 if result["passed"] else 1)
