from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash,identity
from finite_research.direct_evidence_modules import driver

def retained(points,faces,active,check,directory):
    from native_mixed_face_certificate import certify
    from rolling_fast_state import verify_fresh_coverage
    from background_independent import AABBIndex
    from native_proof_segments import persist_segment,LoadedSegment,segment_objects
    from path_preserving_deduplication import durable_json
    check()
    faces=list(map(tuple,faces))
    certificate=certify(points,faces,active,workers=4)
    result={name:certificate[name] for name in ("passed","required_pairs","failures","proof_hash")}
    if not certificate["passed"]:
        return dict(result,independent_coverage_passed=False)
    methods=verify_fresh_coverage(points,faces,active,certificate,index_type=AABBIndex)
    check()
    directory=Path(directory)
    directory.mkdir()
    offsets=np.zeros(len(faces)+1,dtype="<i8")
    for index,face in enumerate(faces):
        offsets[index+1]=offsets[index]+len(face)
    vertices=np.asarray([vertex for face in faces for vertex in face],dtype="<i8")
    population=directory/"FACE_POPULATION.npz"
    np.savez(population,offsets=offsets,vertices=vertices,active=np.asarray(active,dtype="<i8"))
    keys=[str(index) for index in range(len(faces))]
    descriptor=persist_segment(certificate["proofs"],keys,directory)
    loaded=LoadedSegment(descriptor,directory,{name:index for index,name in enumerate(keys)})
    original=certificate["proofs"]
    original_pairs=np.frombuffer(original.pairs,dtype=np.uint64).reshape(-1,2)
    original_codes=np.frombuffer(original.codes,dtype=np.uint64)
    if descriptor["count"]!=certificate["required_pairs"] or loaded.payloads!=original.payloads:
        raise ValueError("ChangedDirectCollisionWitnessStorage")
    for start in range(0,len(original_pairs),65536):
        check()
        stop=start+65536
        if (not np.array_equal(loaded.mapping[loaded.pairs[start:stop]],original_pairs[start:stop])
            or not np.array_equal(loaded.codes[start:stop],original_codes[start:stop])):
            raise ValueError("ChangedDirectCollisionWitnessReadback")
    with np.load(population,allow_pickle=False) as saved:
        if (set(saved.files)!={"offsets","vertices","active"} or not np.array_equal(saved["offsets"],offsets)
            or not np.array_equal(saved["vertices"],vertices) or not np.array_equal(saved["active"],active)):
            raise ValueError("ChangedDirectCollisionPopulationReadback")
    for row in segment_objects(descriptor):
        if driver.sha(directory/row["file"])!=row["sha256"]:
            raise ValueError("ChangedDirectCollisionObject")
    bound=dict(schema="DirectCollisionWitnesses/1",points_identity=array_hash(points),
        face_population_sha256=driver.sha(population),faces=len(faces),active_faces=len(active),
        segment=descriptor,proof_hash=certificate["proof_hash"],independent_coverage_methods=dict(methods))
    durable_json(directory/"WITNESSES.json",bound)
    check()
    return dict(result,independent_coverage_passed=True,methods=dict(methods),
        witness_directory=directory.name,witnesses_sha256=driver.sha(directory/"WITNESSES.json"),
        witness_identity=identity(bound))

def verify_retained(receipt,audit):
    from native_proof_segments import segment_objects
    directory=Path(audit)/receipt["witness_directory"]
    if directory.parent!=Path(audit):
        raise ValueError("ChangedDirectWitnessDirectory")
    bound=driver.read(directory/"WITNESSES.json")
    if (driver.sha(directory/"WITNESSES.json")!=receipt["witnesses_sha256"]
        or identity(bound)!=receipt["witness_identity"] or bound["proof_hash"]!=receipt["proof_hash"]
        or driver.sha(directory/"FACE_POPULATION.npz")!=bound["face_population_sha256"]):
        raise ValueError("ChangedDirectCollisionWitnessBinding")
    for row in segment_objects(bound["segment"]):
        if Path(row["file"]).name!=row["file"] or driver.sha(directory/row["file"])!=row["sha256"]:
            raise ValueError("ChangedDirectCollisionWitnessObject")
