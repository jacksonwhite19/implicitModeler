"""Use the qualified complete-core hull filter for mixed shell faces too.

Same triangle padding/rotation and exact polygon fallback as
certify_repaired_complete.py. No change to predicates or accepted outcomes.
"""
from validation_row_stream import clean_rows_digest
import numpy as np
import termination_geometry_certificate as exact
from sdf_cfd_mesher_v4.certificate.efficient import Filter,SOURCE
from sdf_cfd_mesher_v4.certificate.efficient_proof import PROOF_CLASSES
from sdf_cfd_mesher_v4.certificate.parallel_screen import screen
from identity_timing import measure


def certify(points,faces,active,*,workers=4):
    # Existing bounded native worker policy; ordered proof handling stays here.
    # One worker retains the direct serial kernel for equivalence controls.
    if type(workers) is not int or not 1<=workers<=24:raise ValueError('InvalidScreenWorkerCount')
    with measure('native_certificate.bounds',len(faces)):
        active=set(active);lo=np.array([points[list(f)].min(0) for f in faces]);hi=np.array([points[list(f)].max(0) for f in faces])
    from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
    from itertools import islice
    with measure('native_certificate.index'):
        pairs=iter(AABBIndex(lo,hi).pairs(active))
    with measure('native_certificate.packing',len(faces)):
        ids=np.asarray([list(f) if len(f)==4 else list(f)+[f[2]] for f in faces],dtype='u8')
        for i,f in enumerate(faces):
            if len(f)==4:ids[i]=np.asarray(f)[[0,1,3,2]]
            elif len(f)!=3:raise ValueError('UnsupportedCertificatePolygon')
    from optimized_mixed_certificate import PackedRows
    native=Filter();q=np.ascontiguousarray(points[ids]);rows=PackedRows();bad=[]
    while True:
        with measure('native_certificate.enumeration'):
            batch=np.asarray(list(islice(pairs,50000)),dtype='u4')
        if not len(batch):break
        with measure('native_certificate.screen',len(batch)):
            codes,witness=(native.screen(q,ids,batch) if workers==1 else screen(native,q,ids,batch,workers=workers))
        with measure('native_certificate.proof_handling',len(batch)):
            classified=np.empty((len(batch),3),dtype=np.uint8)
            classified[:,0]=codes;classified[:,1]=witness;classified[:,2]=255
            fallback_results={}
            for k in np.flatnonzero(codes==0):
                i,j=batch[k];aa=list(faces[i]);bb=list(faces[j]);rot=[]
                for poly,other in [(aa,faces[j]),(bb,faces[i])]:
                    turns=0
                    if len(poly)==3:
                        for _ in range(3):
                            if poly[2] not in other:break
                            poly[:]=poly[1:]+poly[:1];turns+=1
                    rot.append(turns)
                ia=[aa[k] for k in (0,1,3,2)] if len(aa)==4 else aa+[aa[2]]
                ib=[bb[k] for k in (0,1,3,2)] if len(bb)==4 else bb+[bb[2]]
                labels=np.asarray([ia,ib],dtype='u8');c,w=native.screen(points[labels],labels,[[0,1]])
                classified[k]=[int(c[0]),int(w[0]),rot[0]*4+rot[1]]
                # Keep the original rotation -> exact failure boundary/order.
                # Pass only these locally computed results, not fixture proofs.
                if not c[0]:fallback_results[int(k)]=exact.pair(points,faces[int(i)],faces[int(j)])
            bad.extend(rows.append_classified(points,faces,batch,classified,native.source_hash,
                                              fallback_results=fallback_results))
    with measure('native_certificate.summary',len(rows)):methods,_=rows.summary()
    with measure('native_certificate.proof_digest',len(rows)):proof_hash=rows.digest()
    return dict(passed=not bad,required_pairs=len(rows),methods=methods,
        failures=bad,proofs=rows,proof_hash=proof_hash,
        coverage='All AABB-overlapping unique-face pairs touching changed geometry; qualified interval hull filter with unchanged exact polygon fallback')

# Defining-owner binding: custom certifiers keep their original dispatch.
_STANDARD_CERTIFY=certify
