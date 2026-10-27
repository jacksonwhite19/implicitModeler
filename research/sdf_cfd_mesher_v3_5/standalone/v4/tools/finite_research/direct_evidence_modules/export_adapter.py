from pathlib import Path
from contextlib import ExitStack
import ast,hashlib,json,pathlib,subprocess,sys,tempfile,time
import numpy as np
from types import SimpleNamespace

class RetainedTemporaryDirectory:
    def __init__(self,*,prefix,dir):self.name=tempfile.mkdtemp(prefix=prefix,dir=dir)
    def __enter__(self):return self.name
    def __exit__(self,*unused):return False


def load_functions(path,names,observed):
    from finite_research.direct_evidence_modules import driver
    source=path.read_text(encoding="utf8");tree=ast.parse(source)
    functions=[node for node in tree.body if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef)) and node.name in names]
    assert {node.name for node in functions}==set(names)
    namespace=dict(pathlib=pathlib,sha=driver.sha)
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(path),"exec"),namespace)
    observed["legacy_functions_"+path.stem]=str(path)
    return namespace

def cyclic(first,second):
    first=tuple(map(int,first));second=tuple(map(int,second))
    return any(first==second[index:]+second[:index] for index in range(len(second)))

def checkmesh_result(stdout,stderr,returncode):
    failed=[line.strip() for line in (stdout+"\n"+stderr).splitlines() if "Failed" in line or line.strip().startswith("***")]
    warnings=[line.strip() for line in (stdout+"\n"+stderr).splitlines() if "Warning" in line or "warning" in line or "severely" in line]
    return dict(returncode=returncode,literal_mesh_ok="Mesh OK." in stdout,failed_lines=failed,warnings=warnings,
        mesh_quality="NOT_RUN_NO_APPLICABLE_APPROVED_DICTIONARY",
        passed=returncode==0 and "Mesh OK." in stdout and not failed)

def mapped_cell_order(source_cells,offsets,vertices,point_map,check):
    lookup={tuple(sorted(map(int,cell))):index for index,cell in enumerate(source_cells)}
    assert len(lookup)==len(source_cells) and len(offsets)==len(source_cells)+1
    result=[]
    for index in range(len(source_cells)):
        if index%4096==0:check()
        key=tuple(sorted(map(int,point_map[vertices[int(offsets[index]):int(offsets[index+1])]])))
        result.append(lookup[key])
    result=np.asarray(result,dtype="<i8")
    assert len(np.unique(result))==len(source_cells)
    return result

def prepare(spec,observed):
    from sdf_cfd_mesher_v4.runtime import frozen_imports
    get=lambda name:Path(spec["inputs"][name]["path"])
    module=frozen_imports._load("task147_existing_bounded_conversion",get("CONVERSION_AUDIT"))
    writer=load_functions(get("LEGACY_WRITER"),["write_msh"],observed)
    case_writer=load_functions(get("LEGACY_CASE"),["foam_header","create_case"],observed)
    return module,writer,case_writer

def run(selected,observed,deadline,folder,state,saved,spec,context,check,live_request):
    from finite_research import native_budget
    with native_budget.stage('export') as ticket:
        return _run(selected,observed,deadline,folder,state,saved,spec,context,check,live_request,ticket)

def _run(selected,observed,deadline,folder,state,saved,spec,context,check,live_request,ticket):
    from finite_research import native_budget
    from finite_research.direct_evidence_modules import driver
    from immutable_state_access import guard
    from finite_support.retained_initializer import bounded_command,validate_retirement
    from path_preserving_deduplication import durable_json
    module,writer,case_writer=context
    code=Path(__file__).resolve().parent
    get=lambda name:Path(spec["inputs"][name]["path"])
    with ExitStack() as held:
        held.enter_context(guard(Path(ticket['path'])))
        probe=folder/"writer_test.msh"
        test_points=np.asarray([[.1,-0.,.3],[1.+2**-52,0.,0.],[0.,1.,0.],[0.,0.,1.],[1.,0.,1.],[0.,1.,1.]],dtype="<f8")
        test_prisms=np.asarray([[0,1,2,3,4,5]],dtype="<i8")
        test_tets=np.asarray([[0,1,2,3]],dtype="<i8")
        writer["write_msh"](probe,test_points,np.asarray([[0,2,1]]),np.asarray([[3,4,5]]),test_prisms,test_tets)
        lines=probe.read_text().splitlines();begin=lines.index("$Nodes")
        reread=np.asarray([[float(value) for value in row.split()[1:]] for row in lines[begin+2:begin+8]],dtype="<f8")
        assert reread.tobytes()==test_points.tobytes()
        assert "3 6 2 1 1 1 2 3 4 5 6" in lines and "4 4 2 1 1 1 2 3 4" in lines
        test_mapping=mapped_cell_order([np.asarray([0,1,2,3]),np.asarray([1,2,3,4])],np.asarray([0,4,8]),np.asarray([4,3,2,1,3,2,1,0]),np.arange(5),check)
        assert np.array_equal(test_mapping,[1,0])
        durable_json(folder/"WRITER_FOCUSED_TEST.json",dict(passed=True,point_roundtrip_exact=True,prism_tet_element_types_and_one_based_tags=True,
            permuted_cell_mapping_exact=True,
            scope="Serialization-only synthetic primitives; no generated mesh or geometric candidate"))
        points=saved["points"];tets=saved["background_tets"]
        prisms=np.asarray([cell["vertices"] for cell in state["cells"]],dtype="<i8")
        writer_result=writer["write_msh"](folder/"complete_transfer.msh",points,state["wall"],saved["farfield"],prisms,tets)
        case_writer["create_case"](folder/"openfoam_case")
        before={str(path):driver.sha(path) for path in (folder/"complete_transfer.msh",folder/"openfoam_case/system/controlDict")}
        durable_json(folder/"TRANSFER.json",dict(writer=writer_result,source_functions={str(get(name)):driver.sha(get(name)) for name in ("LEGACY_WRITER","LEGACY_CASE")},files=before,
            points=len(points),cells=len(prisms)+len(tets),prisms=len(prisms),tets=len(tets),wall=len(state["wall"]),farfield=len(saved["farfield"])))
        def linux(path):
            value=str(path).replace("\\","/");return "/mnt/"+value[0].lower()+value[2:]
        allowance=native_budget.remaining(ticket)
        assert allowance>30
        entry=code/"native_tools.py";observed["native_export_entry"]=str(entry)
        held.enter_context(guard(entry));held.enter_context(guard(get("LINUX_TOOLCHAIN")))
        assert selected[str(entry)]==driver.sha(entry)
        assert selected[str(get("LINUX_TOOLCHAIN"))]==driver.sha(get("LINUX_TOOLCHAIN"))
        request=dict(entry_sha256=driver.sha(entry),toolchain_path=str(get("LINUX_TOOLCHAIN")),toolchain_sha256=driver.sha(get("LINUX_TOOLCHAIN")),
            transfer_sha256=driver.sha(folder/"complete_transfer.msh"),control_sha256=driver.sha(folder/"openfoam_case/system/controlDict"),generation_calls=0)
        request.update(owners=live_request["owners"],binding=live_request["binding"],
            source_pins=live_request["pins"],source_root=live_request["source_root"],
            joined_archive_sha256=driver.sha(folder.parent/"JOINED_EVIDENCE.npz"),publication_enabled=False,
            native_budget_reservation={key:ticket[key] for key in ('path','sha256')},seconds=allowance)
        durable_json(folder/"NATIVE_REQUEST.json",request)
        held.enter_context(guard(folder/"NATIVE_REQUEST.json"))
        command=["wsl.exe","-e","/usr/bin/python3.10","-B",linux(code/"native_entry.py"),linux(folder),driver.sha(folder/"NATIVE_REQUEST.json"),entry.name]
        primary=None
        linux_deadline=None
        import native_scope
        try:
            linux_deadline=native_scope.downstream_deadline(ticket['deadline']-2)
            allowance=native_budget.remaining(ticket)
            command=bounded_command(command,allowance,folder/"NATIVE_STAGE_RETIREMENT.json",linux_deadline=linux_deadline)
            durable_json(folder/"DISPATCH.json",dict(argv=command,seconds=allowance,generation_calls=0))
            with (folder/"native.stdout").open("x") as stdout,(folder/"native.stderr").open("x") as stderr:
                wait_seconds=ticket['deadline']-time.perf_counter()
                if wait_seconds<=0:raise TimeoutError('NativeHostTicketExpired')
                process=subprocess.run(command,stdout=stdout,stderr=stderr,timeout=wait_seconds)
        except BaseException as error:
            primary=error
            raise
        finally:
            try:
                validate_retirement(driver.read(folder/"NATIVE_STAGE_RETIREMENT.json"),allowance,linux_deadline=linux_deadline)
            except BaseException as error:
                if primary is None:raise
                primary.add_note("Export retirement: "+repr(error))
        check()
        assert process.returncode==0,dict(error="NativeConversionAuditExecution",returncode=process.returncode)
        assert driver.read(folder/"NATIVE_ENVELOPE_COMPLETION.json")["passed"]
        native=driver.read(folder/"NATIVE_COMMANDS.json")
        assert native["closure_passed"]
        for name,digest in before.items():assert driver.sha(Path(name))==digest
        scratch=folder/"scratch";scratch.mkdir()
        original_tempfile=module.tempfile
        try:
            module.tempfile=SimpleNamespace(TemporaryDirectory=lambda **kwargs:RetainedTemporaryDirectory(dir=str(scratch),**kwargs))
            fidelity=module.conversion_fidelity_bounded(folder/"foundation_audit",folder/"openfoam_case",points,prisms,tets,state["wall"],saved["farfield"],state["outer"])
        finally:
            module.tempfile=original_tempfile
        durable_json(folder/"CONVERSION_FIDELITY.json",fidelity)
        assert fidelity["pass"],"Existing exact conversion fidelity refused"
        audit=folder/"foundation_audit"
        foam_points=np.memmap(audit/"points.f64",mode="r",dtype="<f8").reshape((-1,3))
        lookup={point.tobytes():index for index,point in enumerate(points)}
        point_map=np.asarray([lookup[point.tobytes()] for point in foam_points],dtype="<i8")
        offsets=np.memmap(audit/"face_offsets.u64",mode="r",dtype="<u8")
        raw_faces=np.memmap(audit/"face_points.i32",mode="r",dtype="<i4")
        owners=np.memmap(audit/"owner.i32",mode="r",dtype="<i4")
        neighbours=np.memmap(audit/"neighbour.i32",mode="r",dtype="<i4")
        cell_offsets=np.memmap(audit/"cell_offsets.u64",mode="r",dtype="<u8")
        cell_points=np.memmap(audit/"cell_points.i32",mode="r",dtype="<i4")
        foam_to_source=mapped_cell_order([*prisms,*tets],cell_offsets,cell_points,point_map,check)
        cap_lookup={tuple(sorted(map(int,face))):index for index,face in enumerate(state["outer"])}
        first_tet_id=max(cell["id"] for cell in state["cells"])+1
        source_cell_ids=np.asarray([cell["id"] for cell in state["cells"]]+[first_tet_id+index for index in range(len(tets))],dtype="<i8")
        cell_ids=source_cell_ids[foam_to_source]
        prism_lookup={cell["id"]:index for index,cell in enumerate(state["cells"])}
        cap_map=[]
        for index in range(len(neighbours)):
            if index%8192==0:check()
            face=point_map[raw_faces[int(offsets[index]):int(offsets[index+1])]]
            key=tuple(sorted(map(int,face)));row=cap_lookup.get(key)
            if row is None:continue
            owner=int(owners[index]);neighbour=int(neighbours[index])
            original=state["owners"][key][0][0];expected=prism_lookup[original]
            source_owner=int(foam_to_source[owner]);source_neighbour=int(foam_to_source[neighbour])
            assert (source_owner==expected and source_neighbour>=len(prisms)) or (source_neighbour==expected and source_owner>=len(prisms))
            oriented=face if source_owner==expected else face[::-1]
            assert cyclic(oriented,state["outer"][row])
            cap_map.append((row,index,owner if source_owner==expected else neighbour,neighbour if source_owner==expected else owner))
        assert len(cap_map)==len(state["outer"]) and len({row[0] for row in cap_map})==len(state["outer"])
        np.savez(folder/"OPENFOAM_MAPPING.npz",foam_point_to_original=point_map,foam_cell_to_source_id=cell_ids,
            foam_cell_to_source_row=foam_to_source,
            cap_mapping=np.asarray(cap_map,dtype="<i8"),face_offsets=np.asarray(offsets),face_global_vertices=point_map[raw_faces])
        mapping=dict(passed=True,point_mapping="Exact binary64 coordinates",cell_mapping="Explicit canonical-connectivity bijection; no within-group row-order assumption",cap_faces=len(cap_map),
            prism_cells_preserved=len(prisms),interface_owner_types_and_orientation=True,sha256=driver.sha(folder/"OPENFOAM_MAPPING.npz"))
        durable_json(folder/"MAPPING.json",mapping)
        stdout=(folder/"checkMesh.stdout").read_text(errors="replace")
        stderr=(folder/"checkMesh.stderr").read_text(errors="replace")
        check_record=checkmesh_result(stdout,stderr,native["commands"]["checkMesh"]["returncode"])
        durable_json(folder/"CHECKMESH_RESULT.json",check_record)
        sets=[]
        for path in (folder/"openfoam_case").rglob("*"):
            if path.is_file() and path.parent.name=="sets":sets.append(dict(path=str(path),sha256=driver.sha(path),bytes=path.stat().st_size))
        durable_json(folder/"FAILED_SETS.json",dict(files=sets,mapping="Use retained OPENFOAM_MAPPING.npz; no equivalence assumed between named metrics"))
    
    return dict(passed=check_record["passed"],generation_calls=0,conversion_fidelity=fidelity,mapping=mapping,checkmesh=check_record,scientific_acceptance=False)
