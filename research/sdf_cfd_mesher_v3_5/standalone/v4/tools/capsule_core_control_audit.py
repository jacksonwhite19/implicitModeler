"""Existing Foundation geometry metrics applied to experimental core controls.

This is a rejection screen, not a replacement for complete continuous coverage.
No production sizing authority is issued here.
"""
import importlib.util
import time
import numpy as np
from capsule_sizing_control import OUT,HANDOFF,V3,distribution,write,digest,boundary_audit,field_inputs


def foundation():
    from residual_layer5_study import q
    return q.legacy_modules()['component_quality'].FG


def prepare_prisms():
    import capsule_user_domain_handoff as h
    from local_termination_topology import geometry
    authority,_,wall,source,_=h.verify_sources()
    with h.guards(),h.CommittedState(authority['reference']) as state:
        x,tri,rows,lower=h.extract(state,wall,source,True)
        cells=h.read(state.verify(state.out/'GEOMETRY_DELTA.json'))['cells']
        points=np.concatenate([lower,x]);n=len(x)
        def local(v):
            _,layer,_,source=v.split(':');assert int(layer) in (10,11)
            return int(source)+(n if layer=='11' else 0)
        values=[]
        for row in rows:
            c=cells[row['prism_owner']];assert c['source']==row['source_column']
            values.append(geometry(points,dict(c,vertices=[local(v) for v in c['vertices']],faces=[[local(v) for v in f] for f in c['faces']])))
        state.check()
    np.savez(OUT/'TERMINAL_PRISMS.npz',centres=np.array([v['centre'] for v in values]),
             volumes=np.array([v['volume'] for v in values]),areas=np.array([v['areas'] for v in values]))


def face_metrics(fg,fp,a,b,va,vb):
    area,center=fg.face_area_centre_tri(fp)
    da=np.einsum('ij,ij->i',area,center-a);db=np.einsum('ij,ij->i',area,b-center)
    wa=np.abs(da);wb=np.abs(db)
    weight=np.minimum(wa,wb)/(wa+wb)
    ratio=np.minimum(va,vb)/np.maximum(va,vb)
    skew=fg.internal_face_skewness(fp,center,area,a,b)
    cosine=np.einsum('ij,ij->i',area,b-a)/(np.linalg.norm(area,axis=1)*np.linalg.norm(b-a,axis=1))
    nonorth=np.degrees(np.arccos(np.clip(cosine,-1,1)))
    return dict(weight=weight,volume_ratio=ratio,skewness=skew,nonorthogonality=nonorth),da,db


def exact_positive_jacobians(q):
    """Exact dyadic determinant signs. Bounded chunks, no tolerance gate."""
    from fractions import Fraction
    failures=[]
    for i,tet in enumerate(q):
        a=[[Fraction(float(v)) for v in row] for row in tet]
        u=[a[1][k]-a[0][k] for k in range(3)]
        v=[a[2][k]-a[0][k] for k in range(3)]
        w=[a[3][k]-a[0][k] for k in range(3)]
        det=u[0]*(v[1]*w[2]-v[2]*w[1])+u[1]*(v[2]*w[0]-v[0]*w[2])+u[2]*(v[0]*w[1]-v[1]*w[0])
        if det<=0:failures.append(i)
    return failures


def boundary_skewness(fp,owner_centres):
    """Foundation 13 primitiveMeshCheck.C boundaryFaceSkewness, unchanged."""
    area,fc=foundation().face_area_centre_tri(fp)
    cpf=fc-owner_centres;normal=area/(np.linalg.norm(area,axis=1)[:,None]+1e-150)
    d=normal*np.einsum('ij,ij->i',normal,cpf)[:,None]
    sv=cpf-(np.einsum('ij,ij->i',area,cpf)/(np.einsum('ij,ij->i',area,d)+1e-150))[:,None]*d
    mag=np.linalg.norm(sv,axis=1);unit=sv/(mag[:,None]+1e-150)
    denominator=np.maximum(.4*np.linalg.norm(d,axis=1)+1e-150,np.max(np.abs(np.einsum('nvi,ni->nv',fp-fc[:,None],unit)),axis=1))
    return mag/denominator


def outer_quality(candidate):
    data=np.load(OUT/candidate/'NETGEN.npz');p,t=data['points'],data['tets'];far=np.load(OUT/candidate/'OUTER.npz')
    lookup={tuple(v):i for i,v in enumerate(p)};ids=np.array([lookup[tuple(v)] for v in far['points']])[far['triangles']]
    wanted={tuple(sorted(v)):i for i,v in enumerate(ids)};owner=np.full(len(ids),-1,int)
    for i,cell in enumerate(t):
        for local in ((0,2,1),(0,1,3),(1,2,3),(2,0,3)):
            key=tuple(sorted(cell[list(local)]))
            if key in wanted:owner[wanted[key]]=i
    assert np.all(owner>=0)
    values=boundary_skewness(p[ids],p[t[owner]].mean(1))
    result=dict(source='OpenFOAM Foundation 13 meshCheck/primitiveMeshCheck/primitiveMeshCheck.C::boundaryFaceSkewness',
                outer_boundary_skewness=distribution(values),failures=int(sum(values>3.63636)),
                weight_volume_ratio='Not applicable: physical uncoupled outer boundary has no fluid neighbour')
    write(OUT/candidate/'OUTER_QUALITY.json',result)
    return result


def audit(candidate,filename='CORE.npz'):
    started=time.perf_counter();folder=OUT/candidate;data=np.load(folder/filename)
    p=data['points'];t=data['tets'];x,tri,seeds=field_inputs();fg=foundation()
    interface,io,boundary=boundary_audit(p,t,x,tri)
    q=p[t];vol=np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0]))/6
    centres=q.mean(1);patterns=np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    rows=t[:,patterns].reshape(-1,3);owners=np.repeat(np.arange(len(t)),4);keys=np.sort(rows,axis=1)
    order=np.lexsort((keys[:,2],keys[:,1],keys[:,0]));sortedkeys=keys[order]
    starts=np.flatnonzero(np.r_[True,np.any(sortedkeys[1:]!=sortedkeys[:-1],axis=1)]);counts=np.diff(np.r_[starts,len(rows)])
    ir=order[starts[counts==2]];jr=order[starts[counts==2]+1];a=owners[ir];b=owners[jr]
    metrics,_,_=face_metrics(fg,p[rows[ir]],centres[a],centres[b],vol[a],vol[b])
    av=np.stack([.5*np.cross(q[:,j]-q[:,i],q[:,k]-q[:,i]) for i,j,k in patterns],axis=1)
    outer=np.load(folder/'OUTER.npz');lookup={tuple(v):i for i,v in enumerate(p)}
    outer_ids=np.array([lookup[tuple(v)] for v in outer['points']]);outerkeys={tuple(sorted(v)) for v in outer_ids[outer['triangles']]}
    mask=np.ones((len(t),4),bool)
    for idx in order[starts[counts==1]]:
        if tuple(keys[idx]) in outerkeys:mask[idx//4,idx%4]=False
    # Physical uncoupled outer faces excluded; real prism and core neighbours included.
    mean=np.sum(np.linalg.norm(av,axis=2)*mask,axis=1)/mask.sum(1)
    v=av*mask[:,:,None]/mean[:,None,None]
    determinant=np.abs(np.linalg.det(np.einsum('nfi,nfj->nij',v,v)))
    summed=np.abs(av).sum(1)
    aspect=np.maximum(summed.max(1)/summed.min(1),summed.sum(1)/(6*np.abs(vol)**(2/3)))
    prism=np.load(OUT/'TERMINAL_PRISMS.npz')
    im,da,db=face_metrics(fg,x[tri],prism['centres'],centres[io],prism['volumes'],vol[io])
    pa=prism['areas'];pn=pa/np.linalg.norm(pa,axis=2).mean(1)[:,None,None]
    pd=np.abs(np.linalg.det(np.einsum('nfi,nfj->nij',pn,pn)))
    failures=dict(nonpositive_volume=int(sum(vol<=0)),determinant=int(sum(determinant<.0011)),aspect=int(sum(aspect>1000)),
                  interface_side=int(sum((da<=0)|(db<=0))),prism_determinant=int(sum(pd<.0011)),nonmanifold=int(sum(counts>2)),
                  duplicate_tets=len(t)-len(np.unique(np.sort(t,axis=1),axis=0)))
    for prefix,m in [('core',metrics),('interface',im)]:
        for name,bad in [('weight',m['weight']<.055),('volume_ratio',m['volume_ratio']<.01),('skewness',m['skewness']>3.63636)]:
            failures[prefix+'_'+name]=int(sum(bad))
    edges=np.unique(np.sort(t[:,[[0,1],[0,2],[0,3],[1,2],[1,3],[2,3]]].reshape(-1,2),axis=1),axis=0)
    lengths=np.linalg.norm(p[edges[:,0]]-p[edges[:,1]],axis=1)
    first=p[t[io]];first_edges=np.stack([np.linalg.norm(first[:,j]-first[:,i],axis=1) for i,j in ((0,1),(0,2),(0,3),(1,2),(1,3),(2,3))],axis=1)
    interface_edges=np.linalg.norm(x[tri[:,[1,2,0]]]-x[tri],axis=2)
    fe=np.unique(np.sort(outer['triangles'][:,[[0,1],[1,2],[2,0]]].reshape(-1,2),axis=1),axis=0)
    fl=np.linalg.norm(outer['points'][fe[:,0]]-outer['points'][fe[:,1]],axis=1)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    components=connected_components(coo_matrix((np.ones(len(a)),(a,b)),shape=(len(t),len(t))),directed=False,return_labels=False)
    expected=5.625-np.einsum('ij,ij->i',x[tri[:,0]],np.cross(x[tri[:,1]],x[tri[:,2]])).sum()/6
    tolerance=max(4096*np.finfo(float).eps*max(abs(expected),abs(vol).sum()),1e-9)
    failures['volume_balance']=int(abs(vol.sum()-expected)>tolerance)
    failures['disconnected']=int(components!=1)
    innerids=np.array([lookup[tuple(v)] for v in x])
    expected_boundary=outerkeys|{tuple(sorted(v)) for v in innerids[tri]}
    failures['boundary_set']=len(expected_boundary.symmetric_difference({tuple(v) for v in boundary}))
    area_flat=av.reshape(-1,3)
    failures['internal_winding']=int(sum(np.einsum('ij,ij->i',area_flat[ir],area_flat[jr])>=0))
    exact_start=time.perf_counter()
    exact_bad=exact_positive_jacobians(q)
    failures['nonpositive_exact_jacobian']=len(exact_bad)
    exact_seconds=time.perf_counter()-exact_start
    # A bounded deterministic realized-distance sample, separate from full edge statistics.
    import json
    from generic_core_sizing import CoreSizingField
    candidate_record=json.loads((folder/'RESULT.json').read_text())['candidate']
    distance_field=CoreSizingField(x[tri],np.full(len(tri),1e-300),g=1.,h_max=10.)
    target_field=CoreSizingField(x[tri],seeds,g=candidate_record['g'],h_max=candidate_record['h_max'])
    sample_ids=np.unique(np.linspace(0,len(t)-1,256,dtype=int))
    distances=np.array([distance_field.evaluate(v) for v in centres[sample_ids]])
    targets=np.array([target_field.evaluate(v) for v in centres[sample_ids]])
    sample_edges=np.stack([np.linalg.norm(q[sample_ids,j]-q[sample_ids,i],axis=1) for i,j in ((0,1),(0,2),(0,3),(1,2),(1,3),(2,3))],axis=1)
    bins=np.quantile(distances,[0,.25,.5,.75,1]);distance_rows=[]
    for k in range(4):
        chosen=(distances>=bins[k])&((distances<bins[k+1]) if k<3 else (distances<=bins[k+1]))
        if chosen.any():distance_rows.append(dict(distance_range_m=[float(bins[k]),float(bins[k+1])],tet_sample_count=int(chosen.sum()),edges=distribution(sample_edges[chosen]),targets=distribution(targets[chosen])))
    result=dict(scope='Experimental rejection screen; complete continuous certification and native checkMesh not performed',
        status='rejected_hard_gates' if any(failures.values()) else 'screen_pass_only',failures=failures,
        interface=interface,core=dict(vertices=len(p),tetrahedra=len(t),interior_vertices=len(p)-len(x)-len(outer['points'])),
        extrema=dict(volume=distribution(vol),determinant=distribution(determinant),aspect=distribution(aspect),
                     core_faces={k:distribution(v) for k,v in metrics.items()},interface_faces={k:distribution(v) for k,v in im.items()},
                     recertified_prism_determinants=distribution(pd)),
        deferred=dict(determinant_contexts=len(pd),final_incidences=len(io),shared_face_records=len(io),terminal_values_used_as_real_neighbour=False),
        transition=dict(all_core_edges=distribution(lengths),first_core_edges=distribution(first_edges),
            first_core_edge_over_seed=distribution(first_edges/seeds[:,None]),unchanged_shared_edges_over_seed=distribution(interface_edges/seeds[:,None]),
            adjacent_equivalent_size_ratio=distribution(np.cbrt(np.maximum(vol[a],vol[b])/np.minimum(vol[a],vol[b]))),outer_edges=distribution(fl),
            distance_sample=dict(tets=len(sample_ids),definition='256 evenly spaced tetrahedron ordinals; exact triangle-distance BVH; quartile bins',bins=distance_rows)),
        topology=dict(connected_components=int(components),internal_faces=len(a),boundary_faces=len(boundary),expected_boundary_faces=len(tri)+len(outer['triangles'])),
        exact_jacobians=dict(tetrahedra=len(t),method='Fraction exact rational determinant of binary64 vertex coordinates',nonpositive_ids=exact_bad,seconds=exact_seconds),
        volume=dict(actual=float(vol.sum()),expected=float(expected),difference=float(vol.sum()-expected),qualified_tolerance=float(tolerance)),
        failed_ids=dict(determinant=np.flatnonzero(determinant<.0011).tolist(),interface_weight=np.flatnonzero(im['weight']<.055).tolist(),interface_skew=np.flatnonzero(im['skewness']>3.63636).tolist()),
        seconds=time.perf_counter()-started,source_sha256=digest(folder/filename))
    write(folder/(filename.replace('.npz','')+'_AUDIT.json'),result)
    return result


def continuous_screen(candidate):
    """Bounded changed-face screen using existing qualified predicates.

    Covers all pairs touching outer boundary faces or faces of the documented
    sampled cells. This deliberately does not issue a complete core certificate.
    """
    from native_mixed_face_certificate import certify
    import capsule_user_domain_handoff as h
    tick=h.measured();folder=OUT/candidate;data=np.load(folder/'NETGEN.npz')
    p,t=data['points'],data['tets'];patterns=np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    rows=t[:,patterns].reshape(-1,3)
    faces,inverse=np.unique(np.sort(rows,axis=1),axis=0,return_inverse=True)
    # Sorted keys are also legal perimeter orders for triangles only.
    x,tri,_=field_inputs();_,io,_=boundary_audit(p,t,x,tri)
    sample=np.unique(np.r_[np.linspace(0,len(t)-1,256,dtype=int),io[np.linspace(0,len(io)-1,256,dtype=int)]])
    active=set(inverse.reshape(-1,4)[sample].ravel().tolist())
    outer=np.load(folder/'OUTER.npz');lookup={tuple(v):i for i,v in enumerate(p)}
    outside={tuple(sorted(v)) for v in np.array([lookup[tuple(v)] for v in outer['points']])[outer['triangles']]}
    for i,f in enumerate(faces):
        if tuple(f) in outside:active.add(i)
    result=certify(p,[tuple(map(int,f)) for f in faces],active)
    result.pop('proofs')
    result.update(complete_core_certificate=False,unique_faces=len(faces),active_faces=len(active),sampled_tetrahedra=len(sample),
                  sampling='All outer box triangles plus every face of 256 evenly spaced core cells and 256 evenly spaced interface-owner cells',
                  old_prism_proof_payloads_read=0,old_prism_proofs_recomputed=0,metrics=h.metrics(*tick))
    write(folder/'CONTINUOUS_SCREEN.json',result)
    return result


if __name__=='__main__':
    import sys,json
    if sys.argv[1]=='prepare':prepare_prisms()
    else: print(json.dumps(audit(sys.argv[1]),indent=2))
