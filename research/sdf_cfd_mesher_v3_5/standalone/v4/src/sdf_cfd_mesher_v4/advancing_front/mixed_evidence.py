"""Bind qualified mixed-cell receipts to scientific entities, never offsets."""
from copy import deepcopy
from ..common.authority import identity
from .mixed_front import geometry_inputs, face_key


def attach_validation(state, mesh, checked, previous=None, *, receipt_directory=None, bound_certificate=None):
    """Translate the existing complete local validator's exact results.

    checked is the unmodified qualified mixed-cell validation result. This
    adapter adds identities and coverage accounting, not geometric mathematics.
    """
    if checked.get('passed') is not True:
        raise ValueError('MixedValidationFailed')
    n=len(mesh['wall']['coordinates'])
    ids=mesh.get('vertex_ids',[f'layer:{i//n}:source:{i%n}' for i in range(len(mesh['points']))])
    perimeters={tuple(sorted(f)):f for c in mesh['cells'] for f in c['faces']}
    owned=sorted(perimeters)
    mapped={k:face_key(tuple(ids[i] for i in perimeters[k])) for k in owned}
    inputs=geometry_inputs(state)
    quality={};proofs={}
    exact={str(row['id']):row for row in checked['quality'].get('exact',[])}
    for row in checked['quality']['cells']:
        cell=str(row['id']);c=state['cells'][cell]
        deps=['cell:'+cell]+['face:'+face_key(f) for f in c['faces']]
        deps+=['authority:quality']
        if cell not in exact:raise ValueError('MissingExactCellReceipt')
        quality['cell:'+cell]=dict(cell_type=c['kind'],inputs={k:inputs[k] for k in deps},
                                   value=deepcopy(row),exact=deepcopy(exact[cell]))
        if 'first_prism_incidence' in checked:
            quality['cell:'+cell].update(receipt_class='ProspectiveFirstPrismDeterminant',
                metric_version=checked['first_prism_incidence']['metric_version'],
                face_role_state='WallExcludedLateralAndTopInternal',
                complete_neighbor_metrics=False)
            quality['cell:'+cell]['inputs']['authority:first_prism_metric']=inputs['authority:first_prism_metric']
        if c['kind']!='prism':
            quality['cell:'+cell]['receipt_class']='ShellLocalQualityReceipt'
            quality['cell:'+cell]['final_complete_neighbor_authority']=False
    for row in checked['quality']['faces']:
        face=mapped[tuple(sorted(row['vertices']))]
        deps=['face:'+face,'authority:quality']+['cell:'+str(k) for k in row['owners']]
        quality['face:'+face]=dict(inputs={k:inputs[k] for k in deps},
                                  value={k:deepcopy(v) for k,v in row.items() if k!='vertices'})
    certificate=checked['certificate']
    if certificate.get('failures') or not certificate.get('passed'):
        raise ValueError('MixedCertificateFailed')
    def proof_rows(values=None):
        for row in certificate['proofs'] if values is None else values:
            pair=sorted(mapped[owned[i]] for i in row['pair'])
            key=identity(pair)
            deps=['face:'+f for f in pair]+['authority:certificate']
            if 'certificate_filter' in state['authority']:
                deps.append('authority:certificate_filter')
            yield key,dict(faces=pair,inputs={k:inputs[k] for k in deps},
                           value={k:deepcopy(v) for k,v in row.items() if k!='pair'})
    plan=getattr(certificate['proofs'],'native_delta',None)
    delta_retired=None
    if bound_certificate is not None:
        # Resume an already bound, immutable population. This is not a new
        # proof acceptance route: check its canonical content, geometry context
        # and independent compact-row identity before native publication.
        from .native_checkpoint import ReceiptTable,scientific_digest,ENCODER
        from contextlib import closing
        from validation_row_stream import clean_rows_digest
        if (type(bound_certificate) is not ReceiptTable or previous is not None
            or getattr(bound_certificate,'qualified_context',None)!=identity(inputs)
            or scientific_digest(bound_certificate)!=bound_certificate.scientific):
            raise ValueError('RetainedBoundCertificateMismatch')
        seen=set()
        with closing(bound_certificate._connect()) as db:
            if db.execute('SELECT COUNT(*) FROM receipts').fetchone()[0]!=len(bound_certificate):
                raise ValueError('RetainedBoundCertificateCount')
            def matching_rows():
                for raw in certificate['proofs']:
                    key,value=next(proof_rows([raw]))
                    if key in seen:raise ValueError('DuplicateRetainedBoundProof')
                    seen.add(key)
                    found=db.execute('SELECT value FROM receipts WHERE key=?',(key,)).fetchone()
                    if found is None or found[0]!=ENCODER.encode(value):
                        raise ValueError('RetainedBoundCertificateCorrespondence',key)
                    yield raw
            if clean_rows_digest(matching_rows())!=certificate['proof_hash'] or len(seen)!=len(bound_certificate):
                raise ValueError('RetainedBoundCertificateCoverage')
        proofs=bound_certificate
    elif plan is not None and receipt_directory is not None and previous is not None:
        from .native_checkpoint import receipt_table_from_rows,ReceiptTable
        from .native_receipt_delta import ReceiptDelta
        old=previous['certificate']
        if not isinstance(old,ReceiptTable) or old.scientific!=plan['base'].scientific:
            raise ValueError('StaleReceiptDeltaBase')
        if plan['context']!=identity(inputs) or plan['face_keys']!=[mapped[k] for k in owned]:
            raise ValueError('StaleReceiptDeltaGeometry')
        from itertools import chain
        if 'authenticated_table' in plan:
            from .authenticated_receipts import component
            proofs=plan['authenticated_table']
            if certificate.get('authenticated_identity')!=component(proofs):
                raise ValueError('StaleAuthenticatedCertificate')
            if proofs.base is not plan['base']:raise ValueError('StaleAuthenticatedBase')
        else:
            updates=receipt_table_from_rows(receipt_directory,chain(plan['patches'].items(),proof_rows(plan['fresh'])))
            removed=set(plan['retired'])
            for key in updates:
                removed.discard(key)
            proofs=ReceiptDelta(old,updates,removed)
        delta_retired=sorted(set(plan['retired']).union(plan['patches']))
    elif receipt_directory is not None:
        from .native_checkpoint import receipt_table_from_rows
        from compact_initial_receipts import bind_compact
        proofs=(bind_compact(receipt_directory,certificate['proofs'],[mapped[k] for k in owned],inputs)
                if previous is None else None)
        if proofs is None:proofs=receipt_table_from_rows(receipt_directory,proof_rows())
    else:
        for key,value in proof_rows():
            if key in proofs:raise ValueError('DuplicateMixedProof')
            proofs[key]=value
    if len(proofs)!=certificate['required_pairs']:
        raise ValueError('MissingMixedProofCoverage')
    from .native_checkpoint import ReceiptTable
    if isinstance(proofs,ReceiptTable):
        # Storage-envelope attestation, outside the unchanged scientific map.
        # Set only after the complete certifier and native binding both pass.
        proofs.qualified_context=identity(inputs)
        # The provider may attach a lossless storage-index publisher. It does
        # not generate or accept geometry proofs and is outside native identity.
        population=certificate['proofs']
        publisher=getattr(population,'publish_index',None)
        if publisher is not None and bound_certificate is None:
            segments=publisher([mapped[k] for k in owned],receipt_directory)
            if segments:
                proofs.proof_segments=segments
                from pathlib import Path
                proofs.proof_objects=Path(receipt_directory)
    retained={};retired={}
    for family,new in [('quality',quality),('certificate',proofs)]:
        old=(previous or {}).get(family,{})
        if family=='certificate' and delta_retired is not None:
            retained[family]=len(old)-len(delta_retired)
            retired[family]=delta_retired
            state[family]=new
            continue
        # Both native tables already yield sorted rows. Merge once rather than
        # issuing one SQLite lookup per old proof or materializing key sets.
        def ordered(value):
            from .native_checkpoint import ReceiptTable,ReceiptOverlay
            return iter(value.items()) if isinstance(value,(ReceiptTable,ReceiptOverlay)) else iter(sorted(value.items()))
        new_rows=ordered(new);candidate=next(new_rows,None);kept=0;removed=[]
        for key,value in ordered(old):
            while candidate is not None and candidate[0]<key:candidate=next(new_rows,None)
            if candidate is not None and candidate[0]==key and candidate[1]==value:kept+=1
            else:removed.append(key)
        retained[family]=kept;retired[family]=removed
        state[family]=new
    return dict(retained=retained,retired=retired,
                required_relations=len(proofs),missing=0,duplicates=0,
                qualification_scope='Complete qualified local validation population')
