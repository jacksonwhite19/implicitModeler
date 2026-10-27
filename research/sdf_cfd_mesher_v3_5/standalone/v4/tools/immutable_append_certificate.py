"""Existing compact proof reuse for a checked immutable-cell append.

The certificate predicate depends on oriented face geometry and vertex labels,
not prism cell type. This adapter rejects retirement/mutation; those cases keep
the general rollback certifier. The independent whole-population audit remains
required during qualification.
"""
import copy
import numpy as np
from production_fast_layer import MemorySegment
from native_proof_segments import SegmentProofRows
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key


def certify(run,points,cells,owners,progress=None):
    before=run.tx.current;n=len(run.provider.x)
    source=lambda v:f'layer:{v//n}:source:{v%n}'
    byid={c['id']:c for c in cells}
    if len(byid)!=len(cells):raise ValueError('DuplicateAppendCertificateCell')
    for i,old in run.topology.cells.items():
        if byid.get(i)!=old:raise ValueError('AppendCertificateCommittedCellChanged')
        if not np.array_equal(points[old['vertices']],run.points[old['vertices']]):
            raise ValueError('AppendCertificateCommittedGeometryChanged')
    faces=[owners[k][0][1] for k in sorted(owners)]
    keys=[face_key(tuple(source(v) for v in f)) for f in faces];indices={k:i for i,k in enumerate(keys)}
    if len(indices)!=len(keys):raise ValueError('DuplicateAppendCertificateFace')
    if not set(run.face_keys)<=indices.keys():raise ValueError('AppendCertificateRetiredFace')
    segments=[]
    for old in run.proofs.segments:
        s=copy.copy(old);s.mapping=np.asarray([indices[k] for k in s.faces],dtype=np.uint64)
        if len(s.mapping)>1 and np.any(s.mapping[1:]<=s.mapping[:-1]):raise ValueError('ProofSegmentOrderChanged')
        segments.append(s)
    segments.append(MemorySegment(run.proofs.fresh,run.face_keys,indices))
    from native_mixed_face_certificate import certify as exact
    active=[i for k,i in indices.items() if k not in before['faces']]
    if progress:progress('scope_start',scope='new_changed_face_certificate',active_faces=len(active),retained_relations=len(run.proofs))
    fresh=exact(points,faces,active)
    proofs=SegmentProofRows(segments,fresh['proofs']);methods,bad=proofs.summary()
    if progress:progress('scope_end',scope='new_changed_face_certificate',new_relations=len(fresh['proofs']),failures=len(bad))
    return dict(passed=not bad,proofs=proofs,failures=bad,required_pairs=len(proofs),
                new=len(fresh['proofs']),reused=len(run.proofs),faces=faces,face_keys=keys,
                scope='Complete old immutable face population plus exact new-vs-all population; no retirement supported')
