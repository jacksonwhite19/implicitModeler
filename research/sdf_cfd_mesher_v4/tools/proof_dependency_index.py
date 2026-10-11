"""Exact compact face-to-relation incidence for immutable proof segments."""
import numpy as np


def build(pairs, face_count):
    pairs=np.asarray(pairs)
    if pairs.ndim!=2 or pairs.shape[1]!=2 or pairs.dtype.kind not in 'iu':
        raise ValueError('InvalidDependencyPairs')
    if len(pairs) and (int(pairs.min())<0 or int(pairs.max())>=face_count):
        raise ValueError('DependencyFaceRange')
    # Stable order makes each face's relation list sorted without another sort.
    flat=pairs.reshape(-1)
    order=np.argsort(flat,kind='stable')
    offsets=np.empty(face_count+1,dtype=np.uint64);offsets[0]=0
    np.cumsum(np.bincount(flat.astype(np.int64),minlength=face_count),out=offsets[1:])
    incidence=(order//2).astype(np.uint64)
    return offsets,incidence


def affected(offsets, incidence, faces):
    faces=np.asarray(faces,dtype=np.int64)
    if len(faces) and (faces.min()<0 or faces.max()+1>=len(offsets)):
        raise ValueError('DependencyQueryRange')
    rows=[incidence[int(offsets[f]):int(offsets[f+1])] for f in faces]
    return np.unique(np.concatenate(rows)) if rows else np.empty(0,dtype=np.uint64)
