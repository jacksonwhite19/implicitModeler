"""Reuse native exact proofs only for unchanged scientific face geometry.

The exact kernel is unchanged. New/changed faces use its normal certifier.
Coverage is independently enumerated with the same conservative AABB rule.
"""
from validation_row_stream import clean_rows_digest
import numpy as np
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key,geometry_inputs
from sdf_cfd_mesher_v4.common.authority import file_hash
from pathlib import Path


def certify(points,faces,state,before):
    engine=g.s.cert.certify
    if state['authority'].get('certificate_filter'):
        from native_mixed_face_certificate import certify as engine,SOURCE
        if state['authority']['certificate_filter']!=file_hash(SOURCE):raise ValueError('ChangedCertificateFilter')
    if before is None:return engine(points,faces,range(len(faces)))
    if before['authority']['certificate']!=file_hash(Path(g.s.cert.__file__)):
        raise ValueError('ChangedCertificateImplementation')
    if any(before['authority'].get(k)!=state['authority'].get(k) for k in ('certificate','certificate_filter')):
        raise ValueError('ChangedCertificateImplementation')
    n=len(state['wall']['vertices'])
    ids=[f'layer:{i//n}:source:{i%n}' for i in range(len(points))]
    keys=[face_key(tuple(ids[i] for i in f)) for f in faces];index={k:i for i,k in enumerate(keys)}
    if len(index)!=len(faces):raise ValueError('DuplicateCertificateFace')
    unchanged=set()
    for k,i in index.items():
        old=before['faces'].get(k)
        if old is not None and all(v in before['vertices'] and state['vertices'][v]==before['vertices'][v] for v in old['vertices']):
            unchanged.add(k)
    old_inputs=geometry_inputs(before)
    from compact_proof_rows import CompactProofRows
    from delta_proof_binding import retained_proofs,immutable_base
    from native_proof_segments import extension_plan,SegmentProofRows
    rows=CompactProofRows()
    # A private native state contains ReceiptOverlay. Its MutableMapping.values
    # implementation calls __getitem__, promoting every row into mutable RAM
    # and opening a SQLite connection per row. This audit is strictly read-only.
    new_inputs=geometry_inputs(state)
    plan=extension_plan(immutable_base(before['certificate']),before,old_inputs,new_inputs,unchanged,index)
    segmented=plan is not None
    if not segmented:plan=retained_proofs(before,old_inputs,new_inputs,unchanged,index,rows)
    fresh=engine(points,faces,[i for k,i in index.items() if k not in unchanged])
    if segmented:
        values=fresh['proofs']
        if not isinstance(values,CompactProofRows):values=CompactProofRows(values)
        rows=SegmentProofRows(plan['segments'],values)
    else:
        rows.extend(fresh['proofs']);rows.ordered_unique()
    inherited=plan is not None and plan['inherited_coverage']
    if not inherited:
        from identity_timing import measure
        with measure('coverage_validation'):
            lo=np.array([points[list(f)].min(0) for f in faces]);hi=np.array([points[list(f)].max(0) for f in faces])
            from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
            spatial=AABBIndex(lo,hi)
            actual=iter(rows.pair_iterator())
            for i in range(len(faces)):
                for j in spatial.after(i):
                    if next(actual,None)!=(i,int(j)):raise ValueError('MissingReusedProofCoverage')
            if next(actual,None) is not None:raise ValueError('UnexpectedReusedProofCoverage')
    # Induction: authenticated old coverage restricted to unchanged polygons,
    # plus the complete changed-vs-all AABB population, is the new population.
    # Geometry changes never inherit coverage; they are in fresh's active set.
    rows.work=dict(historical_relations=len(before['certificate']),
                   historical_dependency_rows_validated=0 if inherited else len(before['certificate']),
                   historical_payload_rows_read=0 if segmented else len(before['certificate']),
                   compact_index_rows_scanned=len(before['certificate']) if segmented else 0,
                   whole_population_candidate_audit=not inherited,
                   unchanged_faces=len(unchanged),changed_or_new_faces=len(faces)-len(unchanged),
                   new_exact_pairs=len(fresh['proofs']),old_old_exact_pairs=0)
    if plan is not None:
        plan['fresh']=fresh['proofs']
        plan['face_keys']=keys
        rows.native_delta=plan
    methods,bad=rows.summary()
    authenticated=None
    if plan is not None and hasattr(plan['base'],'authenticated') and not bad:
        from authenticated_certificate import prepare
        authenticated=prepare(plan,new_inputs)
        rows.work.update(compact_index_rows_scanned=0,dependency_index_entries_read=plan.get('dependency_index_entries_read',0),historical_receipts_rebound=plan.get('historical_receipts_rebound',len(plan['patches'])),historical_rows_rendered_for_identity=0)
    result=dict(passed=not bad,required_pairs=len(rows),methods=methods,failures=bad,
        proof_hash=authenticated['root'] if authenticated else clean_rows_digest(rows),proofs=rows,
        coverage=fresh['coverage'],reused=len(rows)-len(fresh['proofs']),new=len(fresh['proofs']))
    if authenticated is not None:result['authenticated_identity']=authenticated
    return result
