"""Exact terminal construction inputs, never a surface acceptance receipt."""
from pathlib import Path
import hashlib,json
import numpy as np

def _hash(array):
    return hashlib.sha256(array.dtype.str.encode()+repr(array.shape).encode()+array.tobytes()).hexdigest()

def _validate(arrays):
    x=arrays['coordinates'];f=arrays['triangles'];n=len(x);m=len(f)
    shapes={'coordinates':(n,3),'triangles':(m,3),'vertex_metadata':(n,3),'face_refs':(m,),
            'roots':(m,),'depths':(m,),'base_nodal_metrics':(n,3,3),'represented_nodal_metrics':(n,3,3)}
    for name,shape in shapes.items():
        a=arrays[name]
        if a.shape!=shape or a.dtype!=np.dtype('<f8' if name in ('coordinates','base_nodal_metrics','represented_nodal_metrics') else '<i8'):
            raise ValueError('CarrierSnapshotArrayShapeOrType',name)
        if not np.isfinite(a).all():raise ValueError('CarrierSnapshotNonfinite',name)
    if f.size and (f.min()<0 or f.max()>=n):raise ValueError('CarrierSnapshotFaceIndex')
    edges=arrays['feature_edges']
    if edges.dtype!=np.dtype('<i8') or edges.ndim!=2 or edges.shape[1]!=4:raise ValueError('CarrierSnapshotFeatureShape')
    if edges.size and (edges[:,:2].min()<0 or edges[:,:2].max()>=n):raise ValueError('CarrierSnapshotFeatureIndex')
    for name in ('base_nodal_metrics','represented_nodal_metrics'):
        a=arrays[name]
        if not np.array_equal(a,a.transpose(0,2,1)) or not np.all(np.linalg.eigvalsh(a)>0):raise ValueError('CarrierSnapshotMetricNotSPD',name)

def read_terminal(path,*,expected_metadata_sha256=None):
    path=Path(path);raw=path.with_suffix('.json').read_bytes()
    # A caller performing authenticated reuse must supply the digest from its
    # trusted enclosing receipt. A self-read alone proves no source identity.
    if expected_metadata_sha256 is not None and hashlib.sha256(raw).hexdigest()!=expected_metadata_sha256:
        raise ValueError('CarrierSnapshotMetadataHash')
    meta=json.loads(raw.decode('utf-8'))
    if meta.get('schema')!='CarrierTerminalSnapshot/1' or meta.get('acceptance_authority') is not False:
        raise ValueError('CarrierSnapshotSchema')
    if hashlib.sha256(path.read_bytes()).hexdigest()!=meta['npz_sha256']:raise ValueError('CarrierSnapshotFileHash')
    with np.load(path,allow_pickle=False) as z:arrays={k:z[k] for k in z.files}
    if set(arrays)!=set(meta['arrays']):raise ValueError('CarrierSnapshotInventory')
    _validate(arrays)
    for name,a in arrays.items():
        if _hash(a)!=meta['arrays'][name]['sha256']:raise ValueError('CarrierSnapshotArrayHash',name)
    return arrays,meta

def write_terminal(path,payload,context):
    path=Path(path);arrays={k:np.asarray(v).copy() for k,v in payload['arrays'].items()};_validate(arrays)
    meta=dict(schema='CarrierTerminalSnapshot/1',acceptance_authority=False,
        role='Exact terminal carrier construction inputs on normal success/refusal; not surface acceptance',
        context=context,representation=payload['representation'],effective_policy=payload['effective_policy'],
        carrier_status=payload['carrier_status'],arrays={k:dict(sha256=_hash(a),dtype=a.dtype.str,shape=list(a.shape)) for k,a in arrays.items()})
    # Both files are fresh within a fresh stage folder; a partial file never
    # grants authority and no older snapshot may be silently overwritten.
    with path.open('xb') as stream:np.savez_compressed(stream,**arrays)
    meta['npz_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    metadata_sha256=hashlib.sha256(path.with_suffix('.json').read_bytes()).hexdigest()
    restored,receipt=read_terminal(path,expected_metadata_sha256=metadata_sha256)
    if any(restored[k].tobytes()!=a.tobytes() for k,a in arrays.items()):raise ValueError('CarrierSnapshotReadback')
    return dict(npz=path.name,metadata=path.with_suffix('.json').name,npz_sha256=meta['npz_sha256'],
                metadata_sha256=metadata_sha256,readback_verified=True,acceptance_authority=False)

def require_supported_handoff(carrier):
    # Current coupled writer composes the base field; it has no authenticated
    # promoted-array input. Do not certify one field and serialize another.
    if carrier.get('interpolation_compatible_transition'):
        raise ValueError('CarrierPromotedMetricConsumerNotBound')
