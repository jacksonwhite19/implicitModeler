from pathlib import Path
import hashlib,json,sys,traceback

folder=Path(sys.argv[1]).resolve()
expected=sys.argv[2]
target=sys.argv[3]
if target not in ("native_farfield.py","native_optimize.py","native_tools.py"):
    raise ValueError("ExplicitDirectNativeEntryRequired")
entry=Path(__file__).resolve().parent/target
request_path=folder/("NATIVE_REQUEST.json" if target=="native_tools.py" else "OPTIMIZER_REQUEST.json")
def sha(path):
    with Path(path).open("rb") as stream:
        digest=hashlib.sha256()
        for block in iter(lambda:stream.read(1048576),b""):
            digest.update(block)
        return digest.hexdigest()
request=json.loads(request_path.read_text())
result=dict(passed=False,target=target,request_sha256=expected,initial_authentication="NOT_REACHED")
primary=None
try:
    if sha(request_path)!=expected:
        raise ValueError("ChangedDirectNativeEnvelopeRequest")
    pins=request.get("pins",request.get("source_pins",{}))
    def local(name):
        value=str(name)
        return str(Path("/mnt/"+value[0].lower()+value[2:].replace("\\","/"))) if value[1:2]==":" else str(Path(value))
    selected={local(name):digest for name,digest in pins.items()}
    for path in (entry,Path(__file__).resolve()):
        if selected.get(str(path))!=sha(path):
            raise ValueError("UnpinnedDirectNativeEntry",str(path))
    result["initial_authentication"]="PASSED"
    sys.argv=[str(entry),str(folder),expected]
    try:
        namespace=dict(__name__="__main__",__file__=str(entry),__package__=None,__spec__=None)
        exec(compile(entry.read_bytes(),str(entry),"exec"),namespace)
    except SystemExit as stopped:
        if stopped.code not in (None,0):
            raise RuntimeError("DirectNativeChildRefused",stopped.code)
    result["passed"]=True
except BaseException as error:
    primary=error
    result.update(error=repr(error),traceback=traceback.format_exc())
finally:
    result["terminal_artifacts"]={}
    for name in ("NATIVE_TERMINAL_CHECKS.json","FARFIELD_RESULT.json","OPTIMIZER_RESULT.json","NATIVE_COMMANDS.json"):
        path=folder/name
        result["terminal_artifacts"][name]=dict(status="PRESENT",sha256=sha(path)) if path.is_file() else dict(status="NOT_REACHED")
    for path in (entry,Path(__file__).resolve()):
        if "selected" not in globals() or selected.get(str(path))!=sha(path):
            result["passed"]=False
            result["entry_changed"]=True
    with (folder/"NATIVE_ENVELOPE_COMPLETION.json").open("x") as stream:
        json.dump(result,stream,indent=2)
raise SystemExit(0 if result["passed"] else 1)
