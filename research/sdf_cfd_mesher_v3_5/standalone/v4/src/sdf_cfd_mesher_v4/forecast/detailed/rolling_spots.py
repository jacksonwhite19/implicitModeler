"""Bounded exact future probes selected from current conservative geometry."""
import numpy as np
from ...certificate.incremental import build_scope
from ...certificate.continuous_prism_certificate import repaired_kernel as kernel
from ...certificate.continuous_prism_certificate.repaired_runtime import CertificateSession
from .continuous import SideIndex

def earlier_pair_families(records):
    """Compare source-edge families independently of other future warnings."""
    before={};after={}
    for row in records:
        family=tuple(sorted(edge for layer,edge in row['sides']))
        layer=max(layer for layer,edge in row['sides'])
        for name,out in [('before',before),('after',after)]:
            if row[name]['classification']=='CertifiedForbiddenIntersection':out[family]=min(layer,out.get(family,21))
    return [dict(edges=list(key),before=before.get(key,21),after=after[key])
            for key in sorted(after) if after[key]<before.get(key,21)]

def canonical_spot_record(record):
    """Retain every proof and derive the summary absent in early Task-4 files."""
    derived=earlier_pair_families(record['records'])
    if 'new_earlier_pair_families' in record and record['new_earlier_pair_families']!=derived:
        raise ValueError('InconsistentExactPairFamilySummary')
    return dict(record,new_earlier_pair_families=derived)

def pair_family_improves(records,edges):
    """Direct change in the first exact intersection for one source family."""
    family=tuple(sorted(map(int,edges)));first={name:21 for name in ('before','after')};resolved_after=True
    for row in records:
        if tuple(sorted(edge for layer,edge in row['sides']))!=family:continue
        resolved_after &= row['after']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact','CertifiedForbiddenIntersection')
        for name in first:
            if row[name]['classification']=='CertifiedForbiddenIntersection':
                first[name]=min(first[name],max(layer for layer,edge in row['sides']))
    return dict(improved=resolved_after and first['after']>first['before'],first_before=first['before'],first_after=first['after'],edges=family,resolved_after=bool(resolved_after),scope='Selected exact pair records; not a complete future certificate')

def selected_pairs(front,top,future,patch,after,budget):
    scope=build_scope(SideIndex(front,future,top),patch.side_edges)
    return select_scope_pairs(scope,after,budget)

def select_scope_pairs(scope,after,budget):
    """Exact ID ordering, vectorized without constructing a Python set per pair.

    This selects diagnostic work only. It does not certify excluded pairs.
    The reference implementation in the test uses the original tuple/set rule.
    """
    if budget<0:raise ValueError('NegativeFutureProbeBudget')
    pairs,_=scope.pairs();sides=np.asarray(scope.sides,dtype=np.int64)
    if not len(pairs):return [],0
    pairs=pairs[np.maximum(sides[pairs[:,0],0],sides[pairs[:,1],0])>after];count=len(pairs)
    if not count or not budget:return [],count
    labels=np.asarray(scope.labels,dtype=np.int64);incident=np.zeros(count,bool)
    for i in range(labels.shape[1]):
        for j in range(labels.shape[1]):
            incident|=np.all(labels[pairs[:,0],i]==labels[pairs[:,1],j],axis=1)
    pairs=pairs[~incident];a=sides[pairs[:,0]];b=sides[pairs[:,1]]
    swap=(a[:,0]>b[:,0])|((a[:,0]==b[:,0])&(a[:,1]>b[:,1]))
    rows=np.unique(np.concatenate((np.where(swap[:,None],b,a),np.where(swap[:,None],a,b)),axis=1),axis=0)
    order=np.lexsort((rows[:,3],rows[:,2],rows[:,1],rows[:,0],np.minimum(rows[:,0],rows[:,2]),np.maximum(rows[:,0],rows[:,2])))
    return [((int(a),int(b)),(int(c),int(d))) for a,b,c,d in rows[order[:budget]]],count

def exact_pair(front,top,future,pair,session):
    if hasattr(session,'future_pair'):
        quads=[];labels=[]
        for layer,edge in pair:
            vertices=top.edges[edge];low=front.coordinates[vertices] if layer==1 else future.vertex['coordinates'][layer-2,vertices]
            high=future.vertex['coordinates'][layer-1,vertices]
            quads.append(np.concatenate((low,high)))
            labels.append(tuple((k,int(v)) for k in (layer-1,layer) for v in vertices))
        return session.future_pair(quads,labels)
    patches=[]
    for layer,edge in pair:
        vertices=top.edges[edge];low=front.coordinates[vertices] if layer==1 else future.vertex['coordinates'][layer-2,vertices]
        high=future.vertex['coordinates'][layer-1,vertices]
        labels=((layer-1,int(vertices[0])),(layer-1,int(vertices[1])),(layer,int(vertices[0])),(layer,int(vertices[1])))
        p=kernel.Patch.binary(np.concatenate((low,high)),labels);valid,reason=p.valid()
        if not valid:return dict(classification='InvalidSidePatch',reason=reason)
        patches.append(p)
    return session.pair(*patches)

def compare_spots(front,top,before,after,patch,layer,known=(),budget=32,*,planner=None):
    """The budget is a work bound. Missing future proofs never mean clear."""
    select=selected_pairs if planner is None else planner.select
    a,na=select(front,top,before,patch,layer,budget);b,nb=select(front,top,after,patch,layer,budget)
    pairs=set(a+b)
    for e in known:pairs.add(tuple(tuple(v) for v in e['sides']))
    session=CertificateSession() if planner is None else planner.proofs
    rows=[];warnings=[];first_before=21;first_after=21
    for pair in sorted(pairs):
        old=exact_pair(front,top,before,pair,session);new=exact_pair(front,top,after,pair,session);k=max(s[0] for s in pair)
        rows.append(dict(sides=pair,before=old,after=new))
        if old['classification']=='CertifiedForbiddenIntersection':first_before=min(first_before,k)
        if new['classification']=='CertifiedForbiddenIntersection':first_after=min(first_after,k)
        if new['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
            cols=sorted({int(c) for _,edge in pair for c in top.edge_triangles.row(edge)})
            warnings.append(dict(kind='pair',metric=new['classification'],layer=k,entity=str(pair),columns=cols,sides=pair,value=None))
    return dict(broad_before=na,broad_after=nb,exact_pair_comparisons=len(rows),records=rows,warnings=warnings,
                first_forbidden_before=first_before,first_forbidden_after=first_after,
                new_earlier_pair_families=earlier_pair_families(rows),complete_future_certificate=False)
