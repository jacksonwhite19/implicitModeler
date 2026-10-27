from contextlib import ExitStack,nullcontext
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
from finite_research.direct_evidence_modules import driver,export_adapter
from finite_research.direct_contract import identity,method,validate_policy


def source_dependency(root,parent,points,held):
    from owned_source_segments import enabled,complete_dependency
    if not enabled():return None
    import base64
    previous=base64.b64decode(parent['previous_current_base64'],validate=True)
    import hashlib
    driver.require(hashlib.sha256(previous).hexdigest()==parent['previous_current_sha256'],'complete source parent pointer')
    current=json.loads(previous)
    driver.require(current.get('mesh')==parent['binding']['parent_prism_mesh'],'complete source parent mesh')
    driver.require('owned_source_dependency' in parent,'required complete source dependency')
    return complete_dependency(root,current,points,expected=parent['owned_source_dependency'],custody=held)


def guarded_files(folder,files,held,check):
    from immutable_state_access import guard
    folder=Path(folder).resolve()
    result={}
    for relative,row in files.items():
        check()
        path=(folder/relative).resolve()
        if not path.is_relative_to(folder) or path==folder or str(Path(relative)).startswith(".."):
            raise ValueError("DirectConsumerPathEscape")
        held.enter_context(guard(path))
        if driver.sha(path)!=row["sha256"] or path.stat().st_size!=row["bytes"]:
            raise ValueError("ChangedDirectConsumerFile",relative)
        result[str(path)]=row["sha256"]
    return result


def foam_readback(folder,arrays,state,observed,check,*,scratch=None):
    folder=Path(folder)
    spec=driver.read(Path(__file__).parent/"direct_evidence_modules/DEPENDENCIES.json")
    module,unused_writer,unused_case=export_adapter.prepare(spec,observed)
    scratch=folder/"readback_scratch" if scratch is None else Path(scratch)
    scratch.mkdir()
    original_tempfile=module.tempfile
    try:
        module.tempfile=SimpleNamespace(TemporaryDirectory=lambda **kwargs:export_adapter.RetainedTemporaryDirectory(dir=str(scratch),**kwargs))
        fidelity=module.conversion_fidelity_bounded(folder/"foundation_audit",folder/"openfoam_case",
            arrays["points"],arrays["prisms"],arrays["background_tets"],arrays["wall"],arrays["farfield"],arrays["cap"])
    finally:
        module.tempfile=original_tempfile
    driver.require(fidelity["pass"] and fidelity==driver.read(folder/"CONVERSION_FIDELITY.json"),"exact OpenFOAM fidelity replay")
    native=driver.read(folder/"NATIVE_COMMANDS.json")
    checkmesh=export_adapter.checkmesh_result((folder/"checkMesh.stdout").read_text(errors="replace"),
        (folder/"checkMesh.stderr").read_text(errors="replace"),native["commands"]["checkMesh"]["returncode"])
    driver.require(checkmesh["passed"] and checkmesh==driver.read(folder/"CHECKMESH_RESULT.json"),"raw OpenFOAM log replay")
    audit=folder/"foundation_audit"
    points=np.memmap(audit/"points.f64",mode="r",dtype="<f8").reshape((-1,3))
    lookup={point.tobytes():index for index,point in enumerate(arrays["points"])}
    driver.require(len(lookup)==len(points)==len(arrays["points"]),"OpenFOAM point bijection")
    point_map=np.asarray([lookup[point.tobytes()] for point in points],dtype="<i8")
    driver.require(len(np.unique(point_map))==len(points),"unique OpenFOAM point mapping")
    offsets=np.memmap(audit/"cell_offsets.u64",mode="r",dtype="<u8")
    vertices=np.memmap(audit/"cell_points.i32",mode="r",dtype="<i4")
    driver.require(len(offsets)==len(state["cells"])+1 and offsets[0]==0 and offsets[-1]==len(vertices)
        and np.all(offsets[1:]>=offsets[:-1]) and np.all(vertices>=0) and np.all(vertices<len(point_map)),"OpenFOAM cell index domain")
    rows=export_adapter.mapped_cell_order([*arrays["prisms"],*arrays["background_tets"]],offsets,vertices,point_map,check)
    face_offsets=np.memmap(audit/"face_offsets.u64",mode="r",dtype="<u8")
    face_vertices=np.memmap(audit/"face_points.i32",mode="r",dtype="<i4")
    owners=np.memmap(audit/"owner.i32",mode="r",dtype="<i4")
    neighbours=np.memmap(audit/"neighbour.i32",mode="r",dtype="<i4")
    driver.require(len(face_offsets)==len(owners)+1 and face_offsets[0]==0 and face_offsets[-1]==len(face_vertices)
        and np.all(face_offsets[1:]>=face_offsets[:-1]) and len(neighbours)<=len(owners)
        and np.all(face_vertices>=0) and np.all(face_vertices<len(point_map))
        and np.all(owners>=0) and np.all(owners<len(rows))
        and np.all(neighbours>=0) and np.all(neighbours<len(rows)),"OpenFOAM face owner index domain")
    ids=np.r_[arrays["prism_ids"],arrays["tetrahedron_ids"]]
    cap_lookup={tuple(sorted(map(int,face))):index for index,face in enumerate(arrays["cap"])}
    mapping=[]
    for index in range(len(neighbours)):
        if index%4096==0:
            check()
        face=point_map[face_vertices[int(face_offsets[index]):int(face_offsets[index+1])]]
        selected=cap_lookup.get(tuple(sorted(map(int,face))))
        if selected is None:
            continue
        owner,neighbour=int(owners[index]),int(neighbours[index])
        owner_prefix=int(rows[owner])<len(arrays["prisms"])
        neighbour_prefix=int(rows[neighbour])<len(arrays["prisms"])
        driver.require(owner_prefix!=neighbour_prefix,"actual cap prism tet ownership")
        oriented=face if owner_prefix else face[::-1]
        driver.require(export_adapter.cyclic(oriented,arrays["cap"][selected]),"actual cap cycle")
        uses=state["owners"][tuple(sorted(map(int,face)))]
        actual_ids=[int(ids[rows[owner]]),int(ids[rows[neighbour]])]
        driver.require(len(uses)==2 and set(actual_ids)=={cell_id for cell_id,cycle in uses},"exact cap source owners")
        cycles=dict(uses)
        driver.require(export_adapter.cyclic(face,cycles[actual_ids[0]])
            and export_adapter.cyclic(face[::-1],cycles[actual_ids[1]]),"both mapped owner cycles")
        mapping.append((selected,index,owner if owner_prefix else neighbour,neighbour if owner_prefix else owner))
    driver.require(len(mapping)==len(cap_lookup)==len(arrays["cap"])
        and len({row[0] for row in mapping})==len(mapping),"complete cap mapping")
    with np.load(folder/"OPENFOAM_MAPPING.npz",allow_pickle=False) as saved:
        expected=dict(foam_point_to_original=point_map,foam_cell_to_source_id=ids[rows],
            foam_cell_to_source_row=rows,cap_mapping=np.asarray(mapping,dtype="<i8"),
            face_offsets=np.asarray(face_offsets),face_global_vertices=point_map[face_vertices])
        driver.require(set(saved.files)==set(expected),"exact mapping fields")
        for name,value in expected.items():
            driver.same(saved[name],value,"recomputed mapping "+name)
    return dict(fidelity=fidelity,checkmesh=checkmesh,mapping_sha256=driver.sha(folder/"OPENFOAM_MAPPING.npz"))


def validate_provisional(folder,binding,pins,helpers,check,observed):
    from immutable_state_access import guard
    from finite_research import direct_readback
    from path_preserving_deduplication import durable_json
    folder=Path(folder).resolve()
    record=folder/"CONSUMER_VALIDATION.json"
    if record.exists():
        raise ValueError("DirectValidationReplay")
    with ExitStack() as held:
        for name in ("EVIDENCE_INDEX.json","EVIDENCE_HANDOFF.json"):
            held.enter_context(guard(folder/name))
        handoff=driver.read(folder/"EVIDENCE_HANDOFF.json")
        index=driver.read(folder/"EVIDENCE_INDEX.json")
        driver.require(index["binding"]==binding==handoff["binding"],"independent provisional binding")
        driver.require(handoff["evidence_index_sha256"]==driver.sha(folder/"EVIDENCE_INDEX.json"),"producer index")
        driver.require(handoff["provisional"] is True and handoff["scope_exit_verified"] is False
            and handoff["scientific_acceptance"] is False and handoff["publication_enabled"] is False,"unadmitted provisional evidence")
        checked=guarded_files(folder,index["files"],held,check)
        parent=driver.read(folder/"PARENT_EXPECTATIONS.json")
        driver.require(parent["binding"]==binding,"parent expectation binding")
        policy=validate_policy(driver.read(folder/"POLICY.json"))
        driver.require(identity(policy)==binding["policy_identity"],"independent policy identity")
        from finite_research.direct_fill_contract import require_binding
        require_binding(handoff["acceptance_contract"],policy);require_binding(index["acceptance_contract"],policy)
        with helpers["finite_stage_command"].reviewed_construction() as construction:
            arrays,state,proof=direct_readback.semantics(folder,parent,policy,helpers,construction,check,folder/"READBACK_AUDIT")
        source_dependency(folder.parent,parent,arrays['points'][:parent['prefix_points']],held)
        foam=foam_readback(folder/"export",arrays,state,observed,check,scratch=folder/"READBACK_AUDIT/conversion_scratch")
        for path,digest in checked.items():
            driver.require(driver.sha(path)==digest,"readback held custody")
        check()
        replay_files={path.relative_to(folder).as_posix():dict(sha256=driver.sha(path),bytes=path.stat().st_size)
            for path in sorted((folder/"READBACK_AUDIT").rglob("*")) if path.is_file()}
        guarded_files(folder,replay_files,held,check)
        value=dict(schema="DirectSemanticReadback/1",binding=binding,acceptance_contract=handoff["acceptance_contract"],shape_applicability=handoff["shape_applicability"],method_identity=identity(method()),
            geometry_identity=proof["geometry_identity"],proof_obligations=sorted(proof["proof_receipts"]),
            parent_expectations_sha256=driver.sha(folder/"PARENT_EXPECTATIONS.json"),
            evidence_index_sha256=driver.sha(folder/"EVIDENCE_INDEX.json"),
            handoff_sha256=driver.sha(folder/"EVIDENCE_HANDOFF.json"),foam=foam,
            replay_files=replay_files,provisional=True,scientific_acceptance=False,publication_enabled=False)
        durable_json(record,value)
        return dict(folder=str(folder),validation_sha256=driver.sha(record),binding=binding,
            parent_expectations_sha256=value["parent_expectations_sha256"],
            handoff_sha256=value["handoff_sha256"],evidence_index_sha256=value["evidence_index_sha256"])


def consume(root,current,expected,check,*,custody=None):
    from immutable_state_access import guard
    from finite_research.direct_contract import ACCEPTANCE,REQUIRED,validate_acceptance
    validate_acceptance(current,expected)
    root=Path(root).resolve()
    if current.get("schema")!=ACCEPTANCE or current.get("execution_target")!="direct_complete":
        raise ValueError("DistinctDirectAcceptanceRequired")
    if current!=expected["descriptor"] or current["source_release_sha256"]!=expected["source_release_sha256"]:
        raise ValueError("IndependentDirectDescriptorRequired")
    with (ExitStack() if custody is None else nullcontext(custody)) as held:
        import sys
        from finite_research.direct_admission import retirement,closed_worker
        packet=Path(expected["request_root"]).resolve()
        selected=expected["source_pins"]
        for path,digest in selected.items():
            held.enter_context(guard(path))
            driver.require(driver.sha(path)==digest,"independent consumer source custody")
        for path,digest in expected["retirement_files"].items():
            held.enter_context(guard(path))
            driver.require(driver.sha(path)==digest,"captured retirement custody")
        retirement(packet)
        parent_process,unused=closed_worker(packet,packet/"live01",selected)
        worker_process,unused=closed_worker(packet,packet/"live01/worker",selected)
        for path,digest in ((root/"DIRECT_SCOPE_EXIT.json",current["scope_exit_sha256"]),
            (packet/"live01/worker/WORKER_COMPLETION.json",current["worker_completion_sha256"]),
            (packet/"live01_SCOPE_CLOSURE.json",current["owner_retirement_sha256"])):
            held.enter_context(guard(path))
            driver.require(driver.sha(path)==digest,"independent scope exit custody")
        driver.require(worker_process["direct_scope_exit_sha256"]==current["scope_exit_sha256"],"worker scoped issuer binding")
        folder=root/"background"
        for name,key in (("CONSUMER_VALIDATION.json","validation_sha256"),("EVIDENCE_HANDOFF.json","handoff_sha256"),
            ("EVIDENCE_INDEX.json","evidence_index_sha256"),("PARENT_EXPECTATIONS.json","parent_expectations_sha256")):
            held.enter_context(guard(folder/name))
            driver.require(driver.sha(folder/name)==current[key],"admitted direct "+name)
        validation=driver.read(folder/"CONSUMER_VALIDATION.json")
        guarded_files(folder,validation["replay_files"],held,check)
        parent=driver.read(folder/"PARENT_EXPECTATIONS.json")
        driver.require(validation["binding"]==parent["binding"]==expected["binding"],"independent admitted parent")
        driver.require(set(validation["proof_obligations"])==set(REQUIRED),"all direct obligations")
        index=driver.read(folder/"EVIDENCE_INDEX.json")
        bound=guarded_files(folder,index["files"],held,check)
        from finite_research.direct_readback import joined
        policy=validate_policy(driver.read(folder/"POLICY.json"))
        arrays,state=joined(folder,parent,policy,check)
        from finite_research.direct_fill_contract import require_binding,require_minima
        for record in (current,validation,index):require_binding(record["acceptance_contract"],policy)
        source_dependency(root,parent,arrays['points'][:parent['prefix_points']],held)
        from finite_research.direct_evidence_modules.joined_audit import geometry_identity,readback
        driver.require(geometry_identity(state)==validation["geometry_identity"],"admitted reconstructed geometry")
        handoff=driver.read(folder/"EVIDENCE_HANDOFF.json")
        require_binding(handoff["acceptance_contract"],policy)
        applicability=require_minima(handoff["shape"],policy)
        driver.require(applicability==handoff["shape_applicability"]==validation["shape_applicability"],"admitted shape applicability")
        readback(folder/"audit",handoff["proof"]["proof_receipts"],validation["geometry_identity"])
        for path,digest in bound.items():
            driver.require(driver.sha(path)==digest,"admitted file custody")
        for module in list(sys.modules.values()):
            name=getattr(module,"__file__",None)
            if name and Path(name).suffix.lower() in (".py",".pyd",".dll"):
                path=str(Path(name).resolve())
                driver.require(selected.get(path)==driver.sha(path),"terminal loaded source closure")
        check()
        return dict(schema="DirectCompleteConsumer/1",acceptance_contract=handoff["acceptance_contract"],shape_applicability=applicability,geometry_identity=validation["geometry_identity"],
            points=len(arrays["points"]),prisms=len(arrays["prisms"]),tetrahedra=len(arrays["background_tets"]),
            continuous_sdf_fluid="NOT_PERFORMED",stored_boundary_geometry="ALL_TWELVE_VALIDATED",
            normal_run_qualified=False,solver_ready=False)
