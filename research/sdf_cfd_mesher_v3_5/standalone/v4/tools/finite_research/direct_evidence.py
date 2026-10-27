from contextlib import contextmanager,ExitStack
from pathlib import Path
import json,threading
import numpy as np
from finite_research.direct_contract import validate_policy,identity,method
from finite_research.direct_evidence_modules import driver,background_audit,joined_audit,shape,export_adapter

@contextmanager
def terminal_lease(run,handoff,check):
    from rolling_fast_state import BackgroundInputs
    from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction
    if type(handoff) is not BackgroundInputs or type(run.tx) is not MemoryTransaction:
        raise ValueError("GenuineDirectTransactionRequired")
    if threading.current_thread() is not threading.main_thread():
        raise ValueError("ExclusiveDirectTerminalRequired")
    transaction=run.tx
    thread=threading.get_ident()
    token=run.live_acceptance
    def require():
        check()
        if run.tx is not transaction or threading.get_ident()!=thread:
            raise ValueError("ChangedDirectTransactionLease")
        handoff.require_origin(run)
        handoff.require(handoff.view)
    require.deadline=check.deadline
    require.remaining_seconds=check.remaining_seconds
    with transaction.exclusive() as require_transaction:
        require()
        token.check(run,_preparation=handoff._open()["preparation"])
        def checked():
            require_transaction()
            require()
        checked.deadline=check.deadline
        checked.remaining_seconds=check.remaining_seconds
        primary=None
        try:
            yield checked
        except BaseException as error:
            primary=error
            raise
        finally:
            try:
                token.check(run,_preparation=handoff._open()["preparation"])
                require_transaction()
                require()
            except BaseException as error:
                if primary is None:
                    raise
                primary.add_note("Direct terminal custody exit: "+repr(error))

def validate_filling(record,cells):
    ids=[cell["id"] for cell in cells]
    exact=record["exact_cells"]
    degree=record["interior_degree"]
    if (len(set(ids))!=len(ids) or [row["id"] for row in exact]!=ids
        or len(degree["values"])!=len(ids) or degree["unresolved"]
        or not all(row["degree"]==1 for row in degree["values"])
        or not record["positive_cells"] or not record["exact_volume_balance"]):
        raise ValueError("IncompleteDirectFillingWitnesses")
    return dict(ordered_cell_ids=ids,witness_order="Existing cavity_volume_audit visits cells in this exact order",
        exact_cells_identity=identity(exact),degree_identity=identity(degree))

def execute(run,folder,handoff,preparation,policy,binding,pins,helpers,attest,check,observed):
    from immutable_state_access import guard
    from path_preserving_deduplication import durable_json
    from background_materialization import build
    import local_termination_topology as topology
    from finite_research import direct_native,native_budget
    from finite_research.prism_connection import terminal_execution_context
    from finite_research.inputs import V4
    validate_policy(policy)
    if binding["policy_identity"]!=identity(policy) or binding["source_release_sha256"]!=driver.sha(V4/"runtime/RELEASE.json"):
        raise ValueError("ChangedDirectEvidenceBinding")
    if handoff._open()["preparation"] is not preparation or binding["parent_prism_mesh"]!=handoff.view.live_acceptance.mesh:
        raise ValueError("ChangedDirectEvidenceParent")
    if folder.exists():
        raise ValueError("FreshDirectEvidenceFolderRequired")
    folder.mkdir()
    with terminal_lease(run,handoff,check) as require,ExitStack() as held:
        expected_artifacts={}
        def retain(path):
            path=Path(path)
            if path not in expected_artifacts:
                held.enter_context(guard(path))
                expected_artifacts[path]=driver.sha(path)
            elif driver.sha(path)!=expected_artifacts[path]:
                raise ValueError("ChangedDirectProducedArtifact",str(path))
        def retain_tree(directory):
            for path in sorted(Path(directory).rglob("*")):
                if path.is_file():
                    retain(path)
        def native_envelope(directory):
            envelope=driver.read(directory/"NATIVE_ENVELOPE_COMPLETION.json")
            if not envelope["passed"]:
                raise ValueError("FailedDirectNativeEnvelope")
            for name,row in envelope["terminal_artifacts"].items():
                if Path(name).name!=name:
                    raise ValueError("ChangedDirectEnvelopePath")
                if row["status"]=="PRESENT" and driver.sha(directory/name)!=row["sha256"]:
                    raise ValueError("ChangedDirectNativeTerminalArtifact",name)
            retain_tree(directory)
        require()
        with helpers["finite_stage_command"].reviewed_construction() as construction:
            view=handoff.view
            token,points,triangles,cap_ids,volumes,terminal,metadata,centres=helpers["finite_stage_background"]._interface(
                view,_preparation=preparation,_handoff=handoff)
            seed_policy=dict(schema="V4GradedBackground/2",
                **{name:policy[name] for name in ("bounds_m","growth_per_m","maximum_size_m","interface_seed_policy")})
            seeds=helpers["finite_stage_background"].interface_seeds(seed_policy,points,triangles,volumes,centres)
            cells=handoff.cells()
            owners={key:list(rows) for key,rows in view.topology.owners.items()}
            state=dict(points=np.asarray(view.points),cells=cells,owners=owners,cap_ids=cap_ids,cap_faces=triangles,
                outer=np.asarray(view.outer,dtype="<i8"),wall=np.asarray(view.wall,dtype="<i8"),
                bounds=np.asarray(policy["bounds_m"],dtype="<f8"))
            from sdf_cfd_mesher_v4.common.authority import array_hash
            from finite_research.direct_publication import snapshot
            import base64
            previous=snapshot(folder.parent/"CURRENT.json")
            if previous is None or json.loads(previous).get("mesh")!=binding["parent_prism_mesh"]:
                raise ValueError("IndependentDirectParentPointerRequired")
            parent=dict(schema="DirectParentExpectations/1",binding=binding,transaction_generation=run.tx.generation,
                prefix_points=len(state["points"]),cells_identity=identity(construction.d.plain(cells)),
                prefix_digest=construction.inherited_digest(cells,owners,require),
                array_hashes={name:array_hash(value) for name,value in dict(points=state["points"],
                    prisms=np.asarray([cell["vertices"] for cell in cells],dtype="<i8"),
                    prism_ids=np.asarray([cell["id"] for cell in cells],dtype="<i8"),wall=state["wall"],cap=state["outer"]).items()},
                previous_current_base64=base64.b64encode(previous).decode("ascii"),
                previous_current_sha256=driver.sha(folder.parent/"CURRENT.json"))
            from owned_source_segments import enabled,complete_dependency
            if enabled():
                parent['owned_source_dependency']=complete_dependency(folder.parent,json.loads(previous),state['points'],custody=held)
            durable_json(folder/"PARENT_EXPECTATIONS.json",parent)
            retain(folder/"PARENT_EXPECTATIONS.json")
            if state["points"][cap_ids].tobytes()!=points.tobytes():
                raise ValueError("ChangedDirectCapCoordinates")
            cap=dict(points=points,triangles=triangles,seeds=seeds)
            cap_path=folder/"CAP.npz"
            np.savez(cap_path,**cap)
            policy_path=folder/"POLICY.json"
            durable_json(policy_path,policy)
            source_context,unused=terminal_execution_context()
            invocation=folder/"DIRECT_RUNTIME_INVOCATION.json"
            durable_json(invocation,dict(schema="DirectRuntimeInvocation/1",binding=binding,publication_enabled=False,
                cap_sha256=driver.sha(cap_path),policy_sha256=driver.sha(policy_path),
                clock_sha256=driver.sha(source_context.parent/"CLOCK.json"),
                source_context=str(source_context),transaction_generation=run.tx.generation))
            for path in (cap_path,policy_path,invocation):
                held.enter_context(guard(path))
            budget=held.enter_context(native_budget.scope(invocation,binding,require))
            require()
            farfield=direct_native.run(folder/"farfield",cap_path,None,policy_path,binding,pins,require,observed,"farfield",invocation)
            native_envelope(folder/"farfield")
            outer_path=folder/"farfield/OUTER.npz"
            held.enter_context(guard(outer_path))
            outer=driver.arrays(outer_path,dict(points="<f8",triangles="<i8"),require)
            helpers["finite_stage_background"].check_box(outer["points"],outer["triangles"],state["bounds"])
            if driver.sha(outer_path)!=farfield["outer_sha256"]:
                raise ValueError("ChangedFreshFarfield")
            frozen=dict(schema="DirectFrozenBoundary/1",cap_sha256=driver.sha(cap_path),
                farfield_sha256=driver.sha(outer_path),farfield_request_sha256=driver.sha(folder/"farfield/OPTIMIZER_REQUEST.json"),
                method_identity=identity(method()),policy_identity=identity(policy),binding=binding)
            durable_json(folder/"FROZEN_BOUNDARY.json",frozen)
            held.enter_context(guard(folder/"FROZEN_BOUNDARY.json"))
            native=direct_native.run(folder/"native",cap_path,outer_path,policy_path,binding,pins,require,observed,"candidate",invocation)
            native_envelope(folder/"native")
            require()
            if driver.sha(outer_path)!=frozen["farfield_sha256"] or driver.sha(cap_path)!=frozen["cap_sha256"]:
                raise ValueError("ChangedFrozenDirectBoundary")
            expected_native={"CORE.npz","OUTER.npz","NETGEN.npz","GMSH.npz","PRECONDITIONED.msh",
                "OPTIMIZATION.json","GENERATION.json","STAGE_SHAPE.json","METHOD.json"}
            if set(native["files"])!=expected_native:
                raise ValueError("IncompleteDirectNativeArtifacts")
            held.enter_context(guard(folder/"native/OPTIMIZER_RESULT.json"))
            for name,digest in native["files"].items():
                path=folder/"native/generated"/name
                held.enter_context(guard(path))
                if driver.sha(path)!=digest:
                    raise ValueError("ChangedDirectNativeResult")
            core=driver.arrays(folder/"native/generated/NETGEN.npz",dict(points="<f8",tets="<i8"),require)
            boundary,inner,far=helpers["finite_stage_background"].compare_boundary(
                core["points"],core["tets"],points,triangles,outer["points"],outer["triangles"])
            joined_points,mapped,far_faces,unused=background_audit.map_join(
                state["points"],cap_ids,core["points"],core["tets"],inner,far,outer["triangles"])
            driver.same(core["points"][far],outer["points"],"fresh farfield coordinate bytes")
            prefix_digest=construction.inherited_digest(cells,owners,require)
            additions=build(mapped,max(cell["id"] for cell in cells)+1)
            all_cells=cells+additions
            joined_owners,incidence=topology.ownership(all_cells,far_faces,list(map(tuple,state["wall"])))
            durable_json(folder/"JOINED_INCIDENCE.json",incidence)
            if not incidence["passed"]:
                raise ValueError("DirectJoinedIncidence")
            preservation=construction.verify_inherited_join(dict(cells=all_cells,owners=joined_owners),cells,owners,prefix_digest,require)
            joined=dict(points=joined_points,bulk_tets=mapped,prefix_front=state["outer"],
                farfield=np.asarray(far_faces,dtype="<i8"),wall=state["wall"],bounds=state["bounds"],
                cells=all_cells,owners=joined_owners,prefix_ids=[cell["id"] for cell in cells],
                prefix_points=len(state["points"]),prefix_digest=prefix_digest,
                provenance=dict(binding=binding,method_identity=identity(method()),frozen_boundary=frozen,
                    optimizer_receipt_sha256=driver.sha(folder/"native/OPTIMIZER_RESULT.json"),
                    generation_invocations=1,live_parent=True))
            from finite_research import direct_repair_custody
            joined_points,repair=direct_repair_custody.apply(folder,joined,state,additions,policy,require)
            retain(folder/'SHARED_APEX_REPAIR.json')
            joined['points']=joined_points
            joined['provenance'].update(repair=repair,construction_invocations=1)
            for name in ("points","bulk_tets","prefix_front","farfield","wall","bounds"):
                joined[name].flags.writeable=False
            saved=dict(points=joined_points,background_tets=mapped,farfield=joined["farfield"],
                prisms=np.asarray([cell["vertices"] for cell in cells],dtype="<i8"),
                prism_ids=np.asarray([cell["id"] for cell in cells],dtype="<i8"),
                tetrahedron_ids=np.asarray([cell["id"] for cell in additions],dtype="<i8"),
                wall=state["wall"],cap=state["outer"])
            np.savez(folder/"JOINED_EVIDENCE.npz",**saved)
            durable_json(folder/"JOINED_RECONSTRUCTION.json",dict(
                cell_metadata=[construction.d.plain({key:value for key,value in cell.items() if key not in ("id","vertices")}) for cell in all_cells],
                provenance=joined["provenance"]))
            retain(folder/"JOINED_RECONSTRUCTION.json")
            retain(folder/"JOINED_EVIDENCE.npz")
            with np.load(folder/"JOINED_EVIDENCE.npz",allow_pickle=False) as archive:
                if set(archive.files)!=set(saved):
                    raise ValueError("ChangedDirectJoinedArchiveKeys")
                for name,value in saved.items():
                    driver.same(archive[name],value,"joined archive "+name)
            def publish(name,value):
                path=folder/(name+".json")
                durable_json(path,value)
                retain(path)
                if driver.read(path)!=value:
                    raise ValueError("ChangedDirectProducedReadback",name)
            populations=shape.diagnostics(state,joined,policy,cap,require,publish)
            audit=folder/"audit"
            audit.mkdir()
            proof=joined_audit.audit(joined,preservation,helpers,construction,require,audit)
            if not proof["whole_joined_geometry_passed"]:
                raise ValueError("DirectWholeGeometryProofFailed",proof.get("failed_stage"))
            from finite_research.direct_evidence_modules.collision import verify_retained
            for name in ("boundary_collision","collision"):
                verify_retained(driver.read(audit/(name.upper()+".json")),audit)
            joined_audit.readback(audit,proof["proof_receipts"],proof["geometry_identity"])
            retain_tree(audit)
            filling=validate_filling(driver.read(audit/"FILLING.json"),all_cells)
            publish("FILLING_WITNESS_BINDING",filling)
            from finite_research.direct_fill_contract import require_minima,binding as contract_binding
            applicability=require_minima(populations,policy)
            require()
            attest()
            spec=driver.read(Path(__file__).parent/"direct_evidence_modules/DEPENDENCIES.json")
            for row in spec["inputs"].values():
                path=Path(row["path"])
                held.enter_context(guard(path))
                if pins.get(str(path))!=row["sha256"] or driver.sha(path)!=row["sha256"]:
                    raise ValueError("UnpinnedDirectExportDependency")
            context=export_adapter.prepare(spec,observed)
            exported=folder/"export"
            exported.mkdir()
            native_request=driver.read(folder/"native/OPTIMIZER_REQUEST.json")
            foam=export_adapter.run(pins,observed,check.deadline,exported,state,saved,spec,context,require,native_request)
            if not foam["passed"] or not driver.read(exported/"NATIVE_TERMINAL_CHECKS.json")["passed"]:
                raise ValueError("DirectOpenFOAMFailure")
            native_envelope(exported)
            budget.finish()
            for path,digest in expected_artifacts.items():
                if driver.sha(path)!=digest:
                    raise ValueError("ChangedDirectProducedArtifact",str(path))
            artifacts={}
            for path in sorted(folder.rglob("*")):
                if path.is_file():
                    held.enter_context(guard(path))
                    artifacts[path.relative_to(folder).as_posix()]=dict(sha256=driver.sha(path),bytes=path.stat().st_size)
            for name,row in proof["proof_receipts"].items():
                if artifacts["audit/"+name.upper()+".json"]["sha256"]!=row["sha256"]:
                    raise ValueError("ChangedDirectProofBeforeHandoff",name)
            if artifacts["export/OPENFOAM_MAPPING.npz"]["sha256"]!=foam["mapping"]["sha256"]:
                raise ValueError("ChangedDirectMappingBeforeHandoff")
            durable_json(folder/"EVIDENCE_INDEX.json",dict(geometry_identity=proof["geometry_identity"],
                binding=binding,acceptance_contract=contract_binding(policy),files=artifacts,publication_enabled=False,scope_exit_verified=False))
            require()
            attest()
            result=dict(schema="DirectEvidenceHandoff/1",passed=True,execution_target="direct_complete",
                publication_enabled=False,scientific_acceptance=False,continuous_sdf_fluid="NOT_PERFORMED",
                scope_exit_verified=False,provisional=True,
                assurance_mode="backend_trusted",geometry_assurance="all_twelve_fresh",binding=binding,
                geometry_identity=proof["geometry_identity"],joined_archive_sha256=driver.sha(folder/"JOINED_EVIDENCE.npz"),
                proof=proof,shape=populations,acceptance_contract=contract_binding(policy),shape_applicability=applicability,openfoam=foam,filling_witnesses=filling,
                farfield=frozen,method_identity=identity(method()),generation_calls=1,
                upstream_farfield_discretizations=1,optimizer_calls=2,normal_run_qualified=False)
            result["evidence_index_sha256"]=driver.sha(folder/"EVIDENCE_INDEX.json")
            durable_json(folder/"EVIDENCE_HANDOFF.json",result)
            if driver.read(folder/"EVIDENCE_HANDOFF.json")!=result:
                raise ValueError("ChangedDirectEvidenceReadback")
            return result
