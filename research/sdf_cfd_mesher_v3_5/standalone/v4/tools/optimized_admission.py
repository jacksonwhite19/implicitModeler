"""Exact native candidate enumeration and localization for admission."""
import time
import numpy as np
from optimized_mixed_certificate import Native
from production_collision_localization import validate_pairs, polygon


def conservative_pairs(quads,core):
    core=np.asarray(core,dtype=bool)
    if core.shape!=(len(quads),):raise ValueError('InvalidActiveAABB')
    if len(quads)>np.iinfo(np.int32).max:raise ValueError('PairIndexRange')
    pairs=Native().pairs(quads,np.flatnonzero(core),8).astype(np.int32)
    return pairs,dict(discoveries=len(pairs),duplicates_removed=0,
        implementation='ExactNativeActiveAABB',historical_pair_rows_scanned=0)


def classify(points,labels,pairs,progress=None,exact_fallback=None):
    validate_pairs(pairs,len(labels))
    if exact_fallback is None:
        from termination_geometry_certificate import pair as exact_fallback
    native=Native();tick=time.perf_counter()
    result=native.screen(np.ascontiguousarray(points[labels]),labels,pairs,8)
    codes=result[:,0].copy();witness=result[:,1].copy()
    rotations=np.zeros((len(pairs),2),np.uint8)
    retry=result[:,2]!=255
    rotations[retry,0]=result[retry,2]//4;rotations[retry,1]=result[retry,2]%4
    times=dict(interval_and_rotation=time.perf_counter()-tick,exact=0.)
    exact_rows=[]
    for k in np.flatnonzero(codes==0):
        tick=time.perf_counter();i,j=map(int,pairs[k])
        try:
            z=exact_fallback(points,polygon(labels[i]),polygon(labels[j]))
            accepted=z['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
            codes[k]={'ExactHullSeparated':7,'ExactHullLegalContact':8,'ExactFaceTiling':9}[z['method']] if accepted else (10 if z['classification']=='CertifiedForbiddenIntersection' else 0)
        except Exception as e:
            z=dict(classification='ProofInfrastructureFailure',error=type(e).__name__+':'+str(e));codes[k]=11
        times['exact']+=time.perf_counter()-tick
        exact_rows.append(dict(index=int(k),pair=[i,j],result=z))
    return dict(codes=codes,witness=witness,rotations=rotations,exact_rows=exact_rows,timings=times,filter_source=native.source_hash)


def install():
    """Explicit opt-in; no sealed implementation or authority is rewritten."""
    import production_l2_screening
    import composable_admission
    import native_mixed_face_certificate
    import cavity_volume_audit
    from optimized_global_degree import degree
    from optimized_mixed_certificate import certify
    production_l2_screening.conservative_pairs=conservative_pairs
    composable_admission.classify=classify
    native_mixed_face_certificate.certify=certify
    cavity_volume_audit.degree=degree
    import sys
    import path_preserving_deduplication
    from optimized_durable_json import durable_json
    original=path_preserving_deduplication.durable_json
    for module in tuple(sys.modules.values()):
        if module is not None and getattr(module,'durable_json',None) is original:
            module.durable_json=durable_json
