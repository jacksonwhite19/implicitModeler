"""Read-only, one-step direct screening and conservative collision risk.

No future prism stack, admission override, geometry publisher or solver occurs.
Broad-phase contact candidates are not discarded merely for sharing topology.
"""
from collections import Counter
from fractions import Fraction
import hashlib
import time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash,identity
from sdf_cfd_mesher_v4.forecast.detailed.metrics import cell_metrics,legacy_modules
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_wedge
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import CELL_LIMITS,FACE_LIMITS
from sdf_cfd_mesher_v4.runtime.regions import extract_regions
from sdf_cfd_mesher_v4.advancing_front.rolling_correction import bounded_patch
from sdf_cfd_mesher_v4.forecast.detailed.patch import LocalityFailure
from sdf_cfd_mesher_v4.certificate.incremental import conservative_pairs


def distributions(values):
    x=np.asarray(values).ravel();finite=x[np.isfinite(x)]
    return dict(count=len(x),nonfinite=int(len(x)-len(finite)),
        **(dict(zip(('min','p1','p5','median','p95','p99','max'),map(float,np.quantile(finite,[0,.01,.05,.5,.95,.99,1])))) if len(finite) else {}))


def direct_screen(top,prefix,high,*,layer,batch=2048,progress=None):
    """One provisional layer; scalar neighbour context is retained, not a stack."""
    start=time.perf_counter();tri=top.triangles;low=prefix[-1]
    if layer!=len(prefix) or layer<2:raise ValueError('CommittedPrefixLayerMismatch')
    if high.shape!=low.shape or not np.isfinite(prefix).all():raise ValueError('InvalidScreeningCoordinates')
    if not np.isfinite(high).all():raise ValueError('NonfiniteProspectiveCoordinates')
    owners=np.asarray([top.edge_triangles.row(i) for i in range(len(top.edges))])
    if owners.shape!=(len(top.edges),2):raise ValueError('InvalidSourceEdgeOwnership')
    cq=legacy_modules()['component_quality'];fg=cq.FG
    prior_C=[];prior_V=[];chunks={};records=[];exact_hash=hashlib.sha256();construction=0.
    for begin in range(0,len(tri),batch):
        ids=tri[begin:begin+batch];t=time.perf_counter()
        p=np.concatenate((low[ids],high[ids]),axis=1);construction+=time.perf_counter()-t
        g=cell_metrics(p,np.full(len(ids),layer),conditioning=False,internal_caps=True)
        exact=[exact_wedge(w) for w in p]
        for j,row in enumerate(exact):
            row=dict(column=begin+j,**row);records.append(row)
            exact_hash.update((identity(row)+'\n').encode())
        g['exact_jacobian']=np.array([float(Fraction(r['jacobian'])) for r in exact])
        g['exact_volume']=np.array([float(Fraction(r['signed_volume'])) for r in exact])
        g['exact_jacobian_sign']=np.array([r['jacobian_sign'] for r in exact],np.int8)
        g['exact_volume_sign']=np.array([r['volume_sign'] for r in exact],np.int8)
        for key,value in g.items():chunks.setdefault(key,[]).append(value)
        C,V=fg.prism_cell_centres(np.concatenate((prefix[-2,ids],low[ids]),axis=1));prior_C.append(C);prior_V.append(V)
        if progress:progress(dict(phase='direct_cells',completed=min(begin+batch,len(tri)),total=len(tri)))
    cells={k:np.concatenate(v) for k,v in chunks.items()};prior_C=np.concatenate(prior_C);prior_V=np.concatenate(prior_V)
    sides={k:[] for k in ('weight','skewness','symmetric_volume_ratio','nonorthogonality')};caps={k:[] for k in sides}
    for begin in range(0,len(top.edges),batch):
        e=top.edges[begin:begin+batch];a,b=owners[begin:begin+batch].T
        vs=np.stack((low[e[:,0]],low[e[:,1]],high[e[:,1]],high[e[:,0]]),axis=1);S,C=fg.face_area_centre_quad(vs)
        vals=cq.face_metrics(vs,S,C,cells['centre'][a],cells['centre'][b],cells['volume'][a],cells['volume'][b])
        for key,value in zip(sides,vals):sides[key].append(value)
    for begin in range(0,len(tri),batch):
        ids=np.arange(begin,min(begin+batch,len(tri)));vs=low[tri[ids]];S,C=fg.face_area_centre_tri(vs)
        vals=cq.face_metrics(vs,S,C,prior_C[ids],cells['centre'][ids],prior_V[ids],cells['volume'][ids])
        for key,value in zip(caps,vals):caps[key].append(value)
    sides={k:np.concatenate(v) for k,v in sides.items()};caps={k:np.concatenate(v) for k,v in caps.items()}
    topo=topology_audit(top,len(high),layer)
    events=hard_events(cells,sides,caps,owners,layer)+topo['events']
    return dict(cells=cells,sides=sides,caps=caps,events=events,exact_records=records,exact_record_hash=exact_hash.hexdigest(),
                topology=topo,construction_seconds=construction,seconds=time.perf_counter()-start,
                counts=dict(provisional_prisms=len(tri),finite_prisms=len(tri),lateral_pairs=len(top.edges),committed_cap_pairs=len(tri),terminal_missing_neighbour_pairs=len(tri),maximum_batch=min(batch,len(tri))))


def topology_audit(top,nvertices,layer):
    tri=top.triangles;events=[]
    _,inv,counts=np.unique(np.sort(tri,axis=1),axis=0,return_inverse=True,return_counts=True)
    duplicates=np.flatnonzero(counts[inv]>1)
    for c in duplicates:events.append(dict(kind='topology',metric='duplicate_triangle',layer=layer,entity=int(c),columns=[int(c)]))
    ordered=tri[:,[[0,1],[1,2],[2,0]]].reshape(-1,2)
    signs=np.where(ordered[:,0]<ordered[:,1],1,-1)
    sums=np.bincount(top.triangle_edges.ravel(),weights=signs,minlength=len(top.edges))
    bad=[]
    for e in range(len(top.edges)):
        owners=top.edge_triangles.row(e)
        if len(owners)!=2 or sums[e]!=0:
            bad.append(e);events.append(dict(kind='topology',metric='edge_incidence_or_orientation',layer=layer,entity=e,columns=owners.tolist()))
    unused=np.setdiff1d(np.arange(nvertices),np.unique(tri))
    if len(unused):raise ValueError('UnusedSourceVerticesRequireOwnershipResolution')
    return dict(events=events,duplicate_triangle_rows=len(duplicates),invalid_edges=len(bad),unused_vertices=len(unused),
                vertices=nvertices,edges=len(top.edges),triangles=len(tri),euler=nvertices-len(top.edges)+len(tri))


def hard_events(cells,sides,caps,owners,layer):
    events=[]
    limits=dict(CELL_LIMITS,exact_jacobian_sign=lambda a:a<=0,exact_volume_sign=lambda a:a<=0)
    for name,predicate in limits.items():
        for col in np.flatnonzero(predicate(cells[name])|~np.isfinite(cells[name])):
            events.append(dict(kind='cell',metric=name,layer=int(layer),entity=int(col),columns=[int(col)],value=float(cells[name][col])))
    for kind,data in [('side',sides),('cap',caps)]:
        for name,predicate in FACE_LIMITS.items():
            for entity in np.flatnonzero(predicate(data[name])|~np.isfinite(data[name])):
                events.append(dict(kind=kind,metric=name,layer=int(layer),entity=int(entity),columns=owners[entity].tolist() if kind=='side' else [int(entity)],value=float(data[name][entity])))
    return events


def face_inventory(top,wall,low,high):
    """Unique physical faces: wall, L1 cap, L2 cap, L1 sides, L2 sides.

    Triangle padding is for AABB extrema only; no bilinear surrogate is claimed.
    L1 caps are shared aliases, not duplicate L2-bottom faces. Only newly changed
    faces are core: old-old geometry is already covered by the sealed L1 state.
    """
    return prefix_face_inventory(top,np.asarray([wall,low]),high)


def prefix_face_inventory(top,prefix,high):
    """Every unique committed face plus the single prospective front/side set.

    No intermediate committed layer may be omitted from collision coverage.
    Prefix coordinates are restored; only the last front is prospective.
    """
    if len(prefix)<2 or high.shape!=prefix[-1].shape:raise ValueError('InvalidCommittedPrefix')
    fronts=[*prefix,high];next_layer=len(prefix)
    n=len(prefix[0]);tri=top.triangles;e=top.edges;owners=np.asarray([top.edge_triangles.row(i) for i in range(len(e))])
    xyz=[];labels=[];columns=[];core=[];kinds=[]
    for layer,x in enumerate(fronts):
        xyz.append(x[tri[:,[0,1,2,2]]]);labels.append((tri[:,[0,1,2,2]]+layer*n))
        columns.append(np.c_[np.arange(len(tri)),np.full(len(tri),-1)])
        core.extend([layer==next_layer]*len(tri));kinds.extend(['cap'+str(layer)]*len(tri))
    for layer,(lo,hi) in enumerate(zip(fronts[:-1],fronts[1:]),1):
        xyz.append(np.stack((lo[e[:,0]],lo[e[:,1]],hi[e[:,0]],hi[e[:,1]]),axis=1))
        labels.append(np.stack(((layer-1)*n+e[:,0],(layer-1)*n+e[:,1],layer*n+e[:,0],layer*n+e[:,1]),axis=1))
        columns.append(owners);core.extend([layer==next_layer]*len(e));kinds.extend(['side'+str(layer)]*len(e))
    return dict(coordinates=np.concatenate(xyz),labels=np.concatenate(labels),columns=np.concatenate(columns),core=np.asarray(core),kinds=np.asarray(kinds))


def collision_screen(inventory):
    start=time.perf_counter();pairs,stats=conservative_pairs(inventory['coordinates'],inventory['core'])
    incident=np.zeros(len(pairs),bool)
    for begin in range(0,len(pairs),65536):
        a,b=pairs[begin:begin+65536].T
        incident[begin:begin+len(a)]=np.any(inventory['labels'][a,:,None]==inventory['labels'][b,None,:],axis=(1,2))
    seed=np.unique(inventory['columns'][pairs].ravel());seed=seed[seed>=0]
    counts=Counter(tuple(sorted((str(inventory['kinds'][a]),str(inventory['kinds'][b])))) for a,b in pairs)
    return dict(pairs=pairs,incident=incident,seed_columns=seed,
        populations=dict(items=len(inventory['core']),new_items=int(inventory['core'].sum()),old_items=int((~inventory['core']).sum()),
            candidate_pairs=len(pairs),shared_topology_candidates=int(incident.sum()),nonincident_candidates=int((~incident).sum()),
            legal_contact_exclusions=0,old_old_excluded_by_scope=True,duplicate_face_aliases_removed=sum(int((inventory['kinds']==k).sum()) for k in np.unique(inventory['kinds']) if k.startswith('cap') and k!='cap0' and not inventory['core'][np.flatnonzero(inventory['kinds']==k)[0]]),
            pair_kinds={'/'.join(k):v for k,v in sorted(counts.items())}),
        stats=stats,seconds=time.perf_counter()-start,
        semantics='All AABB candidates retained as unresolved collision risk, including incident contacts. Shared labels are not a proof that contact is exclusively legal. No forbidden/clear intersection classification.')


def close_risk(top,seeds):
    """Existing zero-ring connected supports and existing bounded closure."""
    start=time.perf_counter();seeds=np.unique(np.asarray(seeds,np.int64));mask=np.zeros(len(top.triangles),bool);mask[seeds]=True
    regions=extract_regions(top,mask,entity='triangle',rings=0,max_vertices=2048);rows=[]
    for r in regions:
        record=dict(seed_columns=sorted(set(map(int,seeds))&set(r.triangle_ids)),vertices=list(r.vertex_ids),edges=list(r.edge_ids),triangles=list(r.triangle_ids),oversized=r.oversized)
        try:
            patch=bounded_patch(top,r.vertex_ids);record.update(status='BoundedClosure',patch=patch.record(),support_identity=identity(patch.record()))
        except LocalityFailure as e:record.update(status='BlockedLocality',reason=str(e),support_identity=None)
        rows.append(record)
    return dict(groups=rows,connected_groups=len(rows),bounded_patches=sum(r['status']=='BoundedClosure' for r in rows),
        blocked_groups=sum(r['status']!='BoundedClosure' for r in rows),seconds=time.perf_counter()-start,
        merging='Existing induced source-vertex connected components; no added rings, partitioning, percentile, or closure radius.',
        support_population_identity=identity(rows))
