"""Read-only migration inventory for qualified rectangular checkpoints.

Preserve array-backed proof populations. This inventory is deliberately not a
MixedFrontRunner checkpoint: publishing it as one would omit mixed ownership,
atlas and forecast state. Native publication must follow complete conversion.
"""
from pathlib import Path
import json
import numpy as np
from ..common.authority import identity,array_hash,file_hash
from .rolling_state import connectivity
from .validation_evidence import input_identity,verify_receipt
from ..certificate.production import verify_committed_layer


def read_checkpoint_prefix(store,layer):
    if layer<1 or layer>len(store.schedule.targets):raise ValueError('InvalidMigrationLayer')
    store.restart() # verify the published source chain before selecting a prefix
    rows=[];coordinates=[];previous=None
    for k in range(layer+1):
        meta=json.loads((store.path/f'LAYER_{k:02}.json').read_text())
        body=dict(meta);h=body.pop('state_hash')
        if identity(body)!=h or meta['previous']!=previous:raise ValueError('LegacyChainMismatch')
        previous=h
        if k:
            p=store.path/meta['coordinate_file']
            if file_hash(p)!=meta['coordinate_file_sha256']:raise ValueError('LegacyCoordinateStorageMismatch')
            x=np.load(p)
            if array_hash(x)!=meta['coordinate_hash']:raise ValueError('LegacyCoordinateIdentityMismatch')
            coordinates.append(x)
        rows.append(meta)
    return rows,coordinates


def migrate_inventory(store,layer,sdf_identity,*,verify_proofs=True,read_scientific=None):
    rows,prefix=read_checkpoint_prefix(store,layer);top=store.top;front=store.front
    quality=[];certificates=[];run=store.path.parent
    for k,meta in enumerate(rows[1:],1):
        receipt=json.loads((run/f'LAYER_{k:02}_VALIDATION_RECEIPT.json').read_text())
        sdf_path=run/f'LAYER_{k:02}_SDF.json'
        sdf=read_scientific(sdf_path) if read_scientific else json.loads(sdf_path.read_text())
        low=front.coordinates if k==1 else prefix[k-2]
        prior=None if k==1 else front.coordinates if k==2 else prefix[k-3]
        inputs=input_identity(top,low,prefix[k-1],prior,k,store.schedule.targets,sdf_identity)
        with np.load(run/f'LAYER_{k:02}_CONDITIONING.npz') as z:conditioning={key:z[key] for key in z.files}
        verify_receipt(receipt,top,inputs,meta,conditioning,sdf)
        quality.append(dict(identity=identity(dict(inputs=inputs,populations=receipt['populations'])),
            cell_type='prism',cell_key=dict(layer=k,source_triangle_hash=array_hash(top.triangles)),
            face_key=dict(layer=k,source_edge_hash=array_hash(top.edges)),receipt=receipt))
        ledger=meta['certificate'];path=run/f'certificate_layer_{k:02}'
        if verify_proofs:
            count,_=verify_committed_layer(front,top,prefix[:k],path,ledger)
        else:count=0
        certificates.append(dict(layer=k,verified=verify_proofs,reused_proofs=count,
            scientific_identity=identity(ledger),ledger=ledger,
            source_face_mapping=dict(kind='bilinear_side',edge_vertices_hash=array_hash(top.edges),
                indexing='divmod(side_id, edge_count) gives zero-based layer and source edge',edge_count=len(top.edges)),
            payloads={name:file_hash(path/name) for name in ('COMPACT.npz','EXACT.json.gz','MANIFEST.json')}))
    data=dict(version='V4LegacyMigrationInventory/1',native_checkpoint=False,
        source_state=rows[-1]['state_hash'],wall=front.coordinate_hash,topology=top.sha256,
        first_layer=array_hash(prefix[0]),schedule=array_hash(store.schedule.targets),
        layers=[dict(layer=k,coordinates=array_hash(x),connectivity=array_hash(connectivity(top.triangles,len(x),k)),
            columns=len(top.triangles),status='ActivePrismColumn') for k,x in enumerate(prefix,1)],
        quality=quality,certificate=certificates,remesh_events=[],
        coverage=dict(expected_quality_receipts=layer,migrated_quality_receipts=len(quality),missing_quality_receipts=0,
            reused_proofs=sum(r['reused_proofs'] for r in certificates),recomputed_proofs=0,
            certificate_population_verified=verify_proofs),
        native_publication_gaps=['mixed entity storage','native atlas','native geometric forecast','mixed proof manifest binding'])
    return dict(data,scientific=identity(data))
