"""Read-only classification of sealed mixed-face candidates.

Reuses the existing mixed-face interval hull filter and exact polygon fallback.
Repeated triangle corners represent a convex hull only, never a regular patch.
No acceptance is based on topology alone. No publisher is imported or called.
"""
import time
import numpy as np
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from sdf_cfd_mesher_v4.certificate.efficient_proof import PROOF_CLASSES
from sdf_cfd_mesher_v4.common.authority import array_hash

METHODS={0:'Unresolved',**PROOF_CLASSES,7:'ExactHullSeparated',8:'ExactHullLegalContact',9:'ExactFaceTiling',10:'ProvenForbidden',11:'ProofInfrastructureFailure'}


def validate_pairs(pairs, face_count, expected_hash=None):
    if pairs.ndim!=2 or pairs.shape[1]!=2 or not np.issubdtype(pairs.dtype,np.integer):
        raise ValueError('InvalidCandidateShape')
    if np.any(pairs<0) or np.any(pairs>=face_count) or np.any(pairs[:,0]>=pairs[:,1]):
        raise ValueError('InvalidCandidateIndex')
    keys=pairs[:,0].astype(np.uint64)*np.uint64(face_count)+pairs[:,1]
    if np.any(keys[1:]<=keys[:-1]):raise ValueError('DuplicateOrUnorderedCandidates')
    if expected_hash is not None and array_hash(pairs)!=expected_hash:raise ValueError('CandidateIdentityMismatch')


def polygon(labels):
    return list(map(int,labels[:3] if labels[2]==labels[3] else labels[[0,1,3,2]]))


def rotated_labels(a,b):
    out=[];turns=[]
    for labels,other in ((a,b),(b,a)):
        p=polygon(labels);n=0
        if len(p)==3:
            for _ in range(3):
                if p[2] not in other:break
                p=p[1:]+p[:1];n+=1
        out.append(p+[p[2]] if len(p)==3 else [p[i] for i in (0,1,3,2)])
        turns.append(n)
    return out,turns


def topology_relations(labels,pairs):
    """0 disjoint; 1 owned vertex; 2 owned edge; 3 other shared topology."""
    out=np.zeros(len(pairs),np.uint8)
    for start in range(0,len(pairs),50000):
        a,b=labels[pairs[start:start+50000]].transpose(1,0,2)
        common=(a[:,:,None]==b[:,None,:]).any(2)
        common[:,3]&=a[:,3]!=a[:,2]
        count=common.sum(1);r=np.where(count==0,0,np.where(count==1,1,3)).astype(np.uint8)
        # Only actual perimeter edges can own a legal edge contact.
        for i in np.flatnonzero(count==2):
            pa=polygon(a[i]);pb=polygon(b[i]);shared=set(pa)&set(pb)
            ea={frozenset(e) for e in zip(pa,pa[1:]+pa[:1])}
            eb={frozenset(e) for e in zip(pb,pb[1:]+pb[:1])}
            if frozenset(shared) in ea&eb:r[i]=2
        out[start:start+len(r)]=r
    return out


def classify(points,labels,pairs,progress=None,exact_fallback=None):
    validate_pairs(pairs,len(labels))
    if exact_fallback is None:
        from termination_geometry_certificate import pair as exact_fallback
    native=Filter();q=np.ascontiguousarray(points[labels]);codes=np.zeros(len(pairs),np.uint8)
    witness=np.full(len(pairs),255,np.uint8);rotations=np.zeros((len(pairs),2),np.uint8)
    exact_rows=[];times=dict(interval=0.,rotation=0.,exact=0.)
    for start in range(0,len(pairs),50000):
        batch=pairs[start:start+50000];t=time.perf_counter()
        c,w=native.screen(q,labels,batch);times['interval']+=time.perf_counter()-t
        unknown=np.flatnonzero(c==0);t=time.perf_counter()
        if len(unknown):
            rl=[]
            for k in unknown:
                r,turns=rotated_labels(*labels[batch[k]])
                rl.extend(r);rotations[start+k]=turns
            rl=np.asarray(rl,np.uint64)
            cc,ww=native.screen(points[rl],rl,np.arange(len(rl),dtype=np.uint32).reshape(-1,2))
            c[unknown]=cc;w[unknown]=ww
        times['rotation']+=time.perf_counter()-t
        for k in np.flatnonzero(c==0):
            t=time.perf_counter();i,j=map(int,batch[k])
            try:
                z=exact_fallback(points,polygon(labels[i]),polygon(labels[j]))
                accepted=z['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
                c[k]={'ExactHullSeparated':7,'ExactHullLegalContact':8,'ExactFaceTiling':9}[z['method']] if accepted else (10 if z['classification']=='CertifiedForbiddenIntersection' else 0)
            except Exception as e:
                z=dict(classification='ProofInfrastructureFailure',error=type(e).__name__+':'+str(e));c[k]=11
            times['exact']+=time.perf_counter()-t
            exact_rows.append(dict(index=start+int(k),pair=[i,j],result=z))
        codes[start:start+len(c)]=c;witness[start:start+len(c)]=w
        if progress:progress(dict(completed=start+len(c),methods={METHODS[int(v)]:int(n) for v,n in zip(*np.unique(codes[:start+len(c)],return_counts=True))}))
    return dict(codes=codes,witness=witness,rotations=rotations,exact_rows=exact_rows,timings=times,filter_source=native.source_hash)


def risk_columns(codes,pairs,columns):
    risk=np.flatnonzero((codes==0)|(codes>=10))
    seeds=np.unique(columns[pairs[risk]].ravel())
    return risk,seeds[seeds>=0]
