"""Prepare one authenticated proof/receipt delta before commit acceptance."""
from itertools import chain
from pathlib import Path
import json
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import receipt_table_from_rows,ENCODER
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import advance,component
from identity_timing import measure


def prepare(plan,inputs):
    return _prepare(plan,inputs)


def _prepare(plan,inputs):
    base=plan['base'];directory=base.auth_objects
    face_keys=plan['face_keys']
    from compact_proof_rows import CompactProofRows
    if not isinstance(plan['fresh'],CompactProofRows):plan['fresh']=CompactProofRows(plan['fresh'])
    def fresh_rows():
        from validation_row_stream import rows
        for row in rows(plan['fresh']):
            faces=sorted(face_keys[i] for i in row['pair'])
            deps=['face:'+f for f in faces]+['authority:certificate']
            if 'authority:certificate_filter' in inputs:deps.append('authority:certificate_filter')
            yield identity(faces),dict(faces=faces,inputs={k:inputs[k] for k in deps},
                                      value={k:v for k,v in row.items() if k!='pair'})
    # Retire/rebind/insert are distinct authenticated operations. Existing
    # scientific keys cannot be silently overwritten by a purported insert.
    with measure('delta_receipt_storage',len(plan['patches'])+len(plan['fresh'])):
        updates=receipt_table_from_rows(directory,chain(plan['patches'].items(),fresh_rows()))
    retired=set(plan['retired'])
    removed=retired-{k for k in updates if k in retired} if retired else set()
    def actions():
        patches=iter(plan['patches']);patch=next(patches,None)
        for k,v in raw_items(updates):
            rebound=k==patch
            if rebound:patch=next(patches,None)
            yield k,v,1 if rebound or k in retired else 0
        if patch is not None:raise ValueError('UnboundCertificatePatch')
        for k in removed:yield k,'null',2
    with measure('incremental_receipt_identity',len(updates)+len(plan['retired'])):
        table=advance(base,updates,removed,actions(),directory)
    if 'segments' in plan:
        from native_proof_segments import persist_segment,add_dependency_index
        with measure('delta_proof_index_publication',len(plan['fresh'])):
            plan['sealed_segments']=[s.descriptor for s in plan['segments']]
            if len(plan['fresh']):
                plan['sealed_segments'].append(add_dependency_index(persist_segment(plan['fresh'],face_keys,directory),directory))
    plan['authenticated_table']=table
    return component(table)
