from pathlib import Path
import ast,hashlib,json,sys
import numpy as np

NAME="FixedBoundaryDelaunay095GmshNetgen155/1"

def verify_request(request,request_path,expected_hash,fresh=True):
    from native_contract import owners
    from datetime import datetime,timezone
    if (sha(request_path)!=expected_hash or request.get("schema")!="DirectNativeRequest/1"
        or request.get("variant")!=NAME or request.get("mode") not in ("candidate","farfield")
        or request.get("generation_calls")!=1 or request.get("maximum_optimizer_calls")!=(2 if request["mode"]=="candidate" else 0)
        or not 0<request["seconds"]<=3600 or request.get("publication_enabled") is not False):
        raise ValueError("InvalidDirectNativeRequest")
    if fresh and datetime.now(timezone.utc)>=datetime.fromisoformat(request["expires_utc"]):
        raise TimeoutError("ExpiredDirectNativeRequest")
    for name,digest in request["pins"].items():
        if sha(local(name))!=digest:
            raise ValueError("ChangedDirectNativeSource",name)
    for row in request["inputs"].values():
        path=local(row["path"])
        if path.stat().st_size!=row["bytes"] or sha(path)!=row["sha256"]:
            raise ValueError("ChangedDirectNativeInput")
    for row in request["owners"].values():
        if sha(local(row["path"]))!=row["sha256"]:
            raise ValueError("ChangedDirectNativeOwner")
    task=json.loads(local(request["owners"]["task"]["path"]).read_text())
    grant=json.loads(local(request["owners"]["authorization"]["path"]).read_text())
    invocation=json.loads(local(request["owners"]["invocation"]["path"]).read_text())
    if (task.get("schema")!="PreparedFiniteLiveTask/1" or task.get("live_execution_authorized") is not True
        or grant.get("approved") is not True or grant.get("mode")!="fresh_live_once"
        or grant.get("task_sha256")!=request["owners"]["task"]["sha256"]
        or invocation.get("schema")!="DirectRuntimeInvocation/1"
        or invocation.get("binding")!=request["binding"]
        or invocation.get("publication_enabled") is not False
        or datetime.now(timezone.utc)>=datetime.fromisoformat(task["deadline_utc"])):
        raise ValueError("DirectNativeLiveOwnerRequired")
    if (sha(local(request["policy_path"]))!=request["policy_sha256"]
        or sha(local(request["toolchain_path"]))!=request["toolchain_sha256"]
        or sha(local(request["source_root"])/"runtime/RELEASE.json")!=request["binding"]["source_release_sha256"]):
        raise ValueError("ChangedDirectNativeContract")
    owners(request)

def sha(path):
    with Path(path).open("rb") as stream:
        digest=hashlib.sha256()
        for block in iter(lambda:stream.read(1048576),b""):digest.update(block)
        return digest.hexdigest()


def write(path,value):
    with Path(path).open("x",encoding="utf8") as stream:json.dump(value,stream,indent=2)


def local(path):
    value=str(path)
    if sys.platform=="linux" and len(value)>2 and value[1]==":":
        return Path("/mnt/"+value[0].lower()+value[2:].replace("\\","/"))
    return Path(value)


def observe_files(files,expected,hasher=sha):
    observed={path:hasher(path) for path in sorted(files)}
    changed=[path for path,digest in observed.items() if expected.get(path)!=digest]
    return observed,changed


def observation_tests():
    counts={}
    actual={"first":"good","second":"bad"}
    def hash_once(path):
        counts[path]=counts.get(path,0)+1
        return actual[path]
    observed,changed=observe_files(actual,{"first":"good","second":"expected"},hash_once)
    assert observed==actual and changed==["second"] and counts=={"first":1,"second":1}
    _,changed=observe_files(actual,{"first":"good"},hash_once)
    assert changed==["second"]
    actual["first"]="mutated"
    _,changed=observe_files(actual,{"first":"good","second":"bad"},hash_once)
    assert changed==["first"]
    return dict(passed=True,one_fresh_digest_per_checkpoint=True,changed_and_unpinned_refused=True,no_cross_checkpoint_cache=True)


def orientation_key(cell):
    vertices=tuple(map(int,cell))
    assert len(set(vertices))==len(vertices)
    parity=sum(vertices[left]>vertices[right] for left in range(len(vertices)) for right in range(left+1,len(vertices)))%2
    return tuple(sorted(vertices)),parity


def exact_reload(before,after):
    points,cells=before;other,rows=after
    assert points.dtype==other.dtype and points.shape==other.shape and cells.shape==rows.shape
    lookup={point.tobytes():index for index,point in enumerate(points)}
    assert len(lookup)==len(points)
    mapping=np.asarray([lookup[point.tobytes()] for point in other],dtype="<i8")
    assert len(np.unique(mapping))==len(points)
    assert sorted(map(orientation_key,cells))==sorted(map(orientation_key,mapping[rows]))
    return dict(passed=True,points=len(points),cells=len(cells),mapping="Exact binary64 coordinate bijection and oriented tetrahedron connectivity")


def optimizer_function(path,namespace):
    tree=ast.parse(Path(path).read_text())
    function=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=="optimize")
    function.decorator_list=[]
    loops=[node for node in ast.walk(function) if isinstance(node,ast.For) and isinstance(node.target,ast.Name) and node.target.id=="name"]
    assert len(loops)==1 and ast.literal_eval(loops[0].iter)==("Gmsh","Netgen")
    inserted=[]
    for parent in ast.walk(function):
        for field,value in ast.iter_fields(parent):
            if not isinstance(value,list):continue
            for index,node in list(enumerate(value))[::-1]:
                if not isinstance(node,ast.Assign) or len(node.targets)!=1 or not isinstance(node.targets[0],ast.Name):continue
                name=node.targets[0].id
                if name=="before":
                    value.insert(index+1,ast.parse("exact_reload((data['points'], data['tets']), before)").body[0]);inserted.append(name)
                elif name=="a":
                    value.insert(index+1,ast.parse("exact_reload(reload_source, a)").body[0]);inserted.append(name)
    branch=next(node for node in ast.walk(function) if isinstance(node,ast.If) and ast.unparse(node.test)=="name == 'Netgen'")
    assert isinstance(branch.body[0],ast.Expr) and "gmsh.write" in ast.unparse(branch.body[0])
    branch.body.insert(0,ast.parse("reload_source=arrays(gmsh)").body[0])
    assert sorted(inserted)==["a","before"]
    source=ast.unparse(ast.fix_missing_locations(function))
    assert source.count("gmsh.model.mesh.optimize(name, force=True)")==1 and source.count("exact_reload(")==2
    assert "generate(" not in source
    exec(compile(source,str(path)+"::FixedBoundaryDelaunay095GmshNetgen155","exec"),namespace)
    return namespace["optimize"]


def focused_tests():
    points=np.asarray([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[1.,1.,1.]],dtype="<f8")
    cells=np.asarray([[0,1,2,3],[1,2,3,4]],dtype="<i8")
    permutation=np.asarray([4,2,0,3,1]);inverse=np.argsort(permutation)
    after=(points[permutation],inverse[cells[::-1]])
    exact_reload((points,cells),after)
    failures=0
    bad_points=after[0].copy();bad_points[0,0]=np.nextafter(bad_points[0,0],2.)
    reversed_cells=after[1].copy();reversed_cells[0,[0,1]]=reversed_cells[0,[1,0]]
    for bad in ((bad_points,after[1]),(after[0],reversed_cells),(after[0],after[1][:-1]),(after[0],np.repeat(after[1][:1],2,axis=0))):
        try:exact_reload((points,cells),bad)
        except (AssertionError,KeyError):failures+=1
        else:raise AssertionError("InvalidReloadAccepted")
    assert failures==4
    return dict(passed=True,closure=observation_tests(),permuted_rows_and_vertices=True,rejected_coordinate_change=True,rejected_orientation_change=True,rejected_missing_cell=True,rejected_duplicate_cell=True)




def guarded_optimizer(module,original,calls,on_enter=None,on_complete=None):
    def invoke(name,force=False):
        assert len(calls)<2 and name==("Gmsh","Netgen")[len(calls)] and force is True
        assert module.option.getNumber("Mesh.OptimizeThreshold")==0.3
        row=dict(name=name,force=True,iterations="API default",threshold=0.3,completed=False)
        calls.append(row)
        if on_enter is not None:on_enter(row)
        result=original(name,force=force)
        assert module.option.getNumber("Mesh.OptimizeThreshold")==0.3
        row.update(completed=True,restored_threshold=0.3)
        if on_complete is not None:on_complete(name)
        return result
    return invoke
