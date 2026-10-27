from collections import Counter
from pathlib import Path
import numpy as np
from finite_research.direct_evidence_modules import driver,joined_audit,shape,export_adapter
from sdf_cfd_mesher_v4.common.authority import array_hash,identity

def core_metadata(rows,ids,*,conditioned,first_id):
    """Canonical face metadata; conditioned IDs may contain edit-created gaps."""
    from background_materialization import build
    if conditioned:return [cell for row,cid in zip(rows,ids) for cell in build(np.asarray([row],dtype='<i8'),int(cid))]
    return build(rows,first_id)


def joined(folder,parent,policy,check):
    from local_termination_topology import ownership
    folder=Path(folder)
    fields=dict(points="<f8",background_tets="<i8",farfield="<i8",prisms="<i8",
        prism_ids="<i8",tetrahedron_ids="<i8",wall="<i8",cap="<i8")
    arrays=driver.arrays(folder/"JOINED_EVIDENCE.npz",fields,check)
    from finite_research.direct_evidence_modules.background_audit import validate_mesh
    for name,arity in (("background_tets",4),("prisms",6),("farfield",3),("wall",3),("cap",3)):
        validate_mesh(arrays["points"],arrays[name],arity)
    prefix=parent["prefix_points"]
    for name in ("prisms","prism_ids","wall","cap"):
        driver.require(array_hash(arrays[name])==parent["array_hashes"][name],"independent parent "+name)
    driver.require(array_hash(arrays["points"][:prefix])==parent["array_hashes"]["points"],"independent parent points")
    ids=np.r_[arrays["prism_ids"],arrays["tetrahedron_ids"]]
    driver.require(len(ids)==len(np.unique(ids)) and len(arrays["prism_ids"])==len(arrays["prisms"])
        and len(arrays["tetrahedron_ids"])==len(arrays["background_tets"]),"unique direct cell identities")
    metadata=driver.read(folder/"JOINED_RECONSTRUCTION.json")
    cells=[]
    for index,(cell_id,vertices) in enumerate(zip(ids,[*arrays["prisms"],*arrays["background_tets"]])):
        cell=dict(metadata["cell_metadata"][index],id=int(cell_id),vertices=list(map(int,vertices)))
        cell["faces"]=[tuple(face) for face in cell["faces"]]
        cells.append(cell)
    driver.require(len(metadata["cell_metadata"])==len(cells),"complete cell metadata")
    driver.require(identity(cells[:len(arrays["prisms"])])==parent["cells_identity"],"independent parent cells")
    rebuilt=core_metadata(arrays['background_tets'],arrays['tetrahedron_ids'],conditioned='conditioning' in metadata['provenance'],first_id=int(max(arrays['prism_ids']))+1)
    driver.require(rebuilt==cells[len(arrays["prisms"]):],"fresh cell connectivity metadata and identities")
    owners,incidence=ownership(cells,list(map(tuple,arrays["farfield"])),list(map(tuple,arrays["wall"])))
    driver.require(incidence["passed"],"direct reconstructed incidence")
    state=dict(points=arrays["points"],bulk_tets=arrays["background_tets"],prefix_front=arrays["cap"],
        farfield=arrays["farfield"],wall=arrays["wall"],bounds=np.asarray(policy["bounds_m"],dtype="<f8"),
        cells=cells,owners=owners,prefix_ids=arrays["prism_ids"].tolist(),prefix_points=prefix,
        prefix_digest=parent["prefix_digest"],provenance=metadata["provenance"])
    driver.require(state["provenance"]["binding"]==parent["binding"],"independent direct provenance")
    from finite_research.direct_repair_custody import verify
    verify(folder,parent,arrays,state,policy,check)
    return arrays,state


class DecodedProofRows:
    def __init__(self,segment,check):
        self.segment=segment
        self.check=check
    def __len__(self):
        return self.segment.descriptor["count"]
    def __iter__(self):
        for index,row in enumerate(self.segment.rows()):
            if index%4096==0:
                self.check()
            yield row
    def summary(self):
        methods=Counter()
        failures=[]
        for row in self:
            methods[row["method"]]+=1
            if row["classification"] not in ("CertifiedSeparated","CertifiedAllowedIncidentContact"):
                failures.append(row)
        return methods,failures


def collision(folder,name,points,faces,check):
    from finite_research.direct_evidence_modules.collision import verify_retained
    from native_proof_segments import LoadedSegment
    from rolling_fast_state import verify_fresh_coverage
    from background_independent import AABBIndex
    audit=Path(folder)/"audit"
    receipt=driver.read(audit/(name.upper()+".json"))
    verify_retained(receipt,audit)
    directory=audit/receipt["witness_directory"]
    witness=driver.read(directory/"WITNESSES.json")
    driver.require(witness["points_identity"]==array_hash(points),"collision actual points")
    population=driver.arrays(directory/"FACE_POPULATION.npz",dict(offsets="<i8",vertices="<i8",active="<i8"),check)
    offsets=np.r_[0,np.cumsum([len(face) for face in faces])].astype("<i8")
    driver.same(population["offsets"],offsets,"collision offsets")
    driver.same(population["vertices"],np.asarray([vertex for face in faces for vertex in face],dtype="<i8"),"collision face population")
    driver.same(population["active"],np.arange(len(faces),dtype="<i8"),"complete collision active population")
    segment=LoadedSegment(witness["segment"],directory,{str(index):index for index in range(len(faces))})
    proofs=DecodedProofRows(segment,check)
    certificate=dict(receipt,proofs=proofs)
    methods=verify_fresh_coverage(points,faces,list(range(len(faces))),certificate,index_type=AABBIndex)
    driver.require(dict(methods)==witness["segment"]["methods"],"decoded collision method counts")
    return receipt


def semantics(folder,parent,policy,helpers,construction,check,output):
    from finite_research.direct_contract import method
    from finite_research.direct_evidence import validate_filling
    arrays,state=joined(folder,parent,policy,check)
    handoff=driver.read(Path(folder)/"EVIDENCE_HANDOFF.json")
    driver.require(handoff["binding"]==parent["binding"] and handoff["method_identity"]==identity(method()),"independent handoff binding")
    driver.require(joined_audit.geometry_identity(state)==handoff["geometry_identity"],"reconstructed geometry identity")
    joined_audit.readback(Path(folder)/"audit",handoff["proof"]["proof_receipts"],handoff["geometry_identity"])
    prefix_cells=state["cells"][:len(arrays["prisms"])]
    import local_termination_topology as topology
    prefix_owners,unused=topology.ownership(prefix_cells,list(map(tuple,arrays["cap"])),list(map(tuple,arrays["wall"])))
    preservation=construction.verify_inherited_join(state,prefix_cells,prefix_owners,parent["prefix_digest"],check)
    def retained(points,faces,active,require,directory):
        name="boundary_collision" if directory.name=="boundary_collision_witnesses" else "collision"
        driver.require(active==list(range(len(faces))),"complete replay collision population")
        return collision(folder,name,points,faces,require)
    output.mkdir()
    proof=joined_audit.audit(state,preservation,helpers,construction,check,output,collision_reader=retained)
    driver.require(proof["whole_joined_geometry_passed"],"all twelve replayed obligations")
    filling=driver.read(Path(folder)/"audit/FILLING.json")
    repeated=driver.read(output/"FILLING.json")
    driver.require(filling==repeated,"exact filling witnesses and volume replay")
    validate_filling(filling,state["cells"])
    reports=shape.diagnostics(dict(outer=arrays["cap"]),state,policy,{},check,lambda *unused:None)
    from finite_research.direct_fill_contract import require_minima,require_binding
    applicability=require_minima(reports,policy)
    require_binding(handoff["acceptance_contract"],policy)
    driver.require(reports==handoff["shape"] and applicability==handoff["shape_applicability"],"actual shape population readback")
    return arrays,state,proof
