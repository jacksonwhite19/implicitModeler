"""Complete surface-only qualification and descriptive diagnostics."""
import collections, json, math, sys, time
import numpy as np
import psutil
from production_adaptive_surface import (OUT,V3,V4,REPO,CapsuleGeometry,SurfaceSizing,
    canonical,sha,save,wsl,run_native,metric,identity,array_hash)
from core_handoff_audit import surface
from continuous_geometry_certifier import certify_face, feature_bound

QUANTILES=(0,.01,.05,.5,.9,.95,.99,.995,.999,1)
NAMES=('min','p1','p5','median','p90','p95','p99','p99.5','p99.9','max')

def dist(a):
    a=np.asarray(a)
    if not len(a) or not np.isfinite(a).all():raise ValueError('NonfiniteOrEmptySurfaceAudit')
    return dict(zip(NAMES,map(float,np.quantile(a,QUANTILES))))

def _scalar_topology(x,f,*,expected_components=1,expected_euler=2):
    incidence=collections.defaultdict(list);directed=collections.Counter();parents=list(range(len(x)))
    def root(i):
        while parents[i]!=i:parents[i]=parents[parents[i]];i=parents[i]
        return i
    for i,face in enumerate(f):
        for a,b in zip(face,np.roll(face,-1)):
            a=int(a);b=int(b);incidence[tuple(sorted((a,b)))].append(i);directed[(a,b)]+=1
            parents[root(a)]=root(b)
    used=set(map(int,f.flat));components=len({root(i) for i in used})
    row=dict(vertices=len(x),faces=len(f),edges=len(incidence),connected_components=components,
        unused_vertices=len(x)-len(used),boundary_edges=sum(len(v)==1 for v in incidence.values()),
        nonmanifold_edges=sum(len(v)>2 for v in incidence.values()),
        duplicate_triangles=len(f)-len(np.unique(np.sort(f,axis=1),axis=0)),
        duplicate_vertices=len(x)-len(np.unique(x,axis=0)),
        orientation_edge_failures=sum(directed[(a,b)]!=1 or directed[(b,a)]!=1 for a,b in incidence),
        euler=len(x)-len(incidence)+len(f))
    row['passed']=((expected_components is None or components==expected_components) and (expected_euler is None or row['euler']==expected_euler) and not any(row[k] for k in
        ('unused_vertices','boundary_edges','nonmanifold_edges','duplicate_triangles','duplicate_vertices','orientation_edge_failures')))
    return row


def topology(x,f,*,expected_components=1,expected_euler=2):
    from surface_topology_batch import topology as batched
    return batched(x,f,_scalar_topology,expected_components=expected_components,expected_euler=expected_euler)


def audit_arrays(x,f,authority=None,metrics=None,*,face_refs=None,vertex_refs=None,contract=None):
    t=time.perf_counter();s=surface(x,f)
    out=dict(vertices=len(x),triangles=len(f),edges=len(s['edges']),edge=dist(s['edge_length']),
        triangle={k:dist(s[k]) for k in ('area','shortest_edge','longest_edge','edge_ratio','minimum_angle','maximum_angle','circumradius','inradius','radius_ratio')})
    expected={} if contract is None else dict(expected_components=contract.expected_components,expected_euler=contract.expected_euler)
    out['topology']=topology(x,f,**expected);out['topology_seconds']=time.perf_counter()-t
    if authority is not None:
        face_refs=np.ones(len(f),int) if face_refs is None else face_refs
        vertex_refs=np.ones(len(x),int) if vertex_refs is None else vertex_refs
        from general_sdf_authority import GeneralSdfAuthority
        samples=None
        if type(authority) is GeneralSdfAuthority and authority.numerical_acceptance_active():
            samples=authority.numerical_surface_samples(x,f,face_refs,vertex_refs,s['centre'])
            s['_numerical_samples']=samples
        n=(samples['normals'][len(x)+samples['edge_count']:] if samples is not None else
           np.array([authority.exact_normal(ref,p) for ref,p in zip(face_refs,s['centre'])])); dots=(n*s['normal']).sum(1)
        angles=np.degrees(np.arccos(np.clip(dots,-1,1)))
        residual=(samples['field'][:len(x)] if samples is not None else np.array([authority.implicit_value(ref,p) for ref,p in zip(vertex_refs,x)]))
        if len(set(face_refs))==1:
            vertex_normals=(samples['normals'][:len(x)] if samples is not None else np.array([authority.exact_normal(face_refs[0],p) for p in x]))[f]
        else:
            vertex_normals=np.array([[authority.exact_normal(ref,p) for p in x[face]] for ref,face in zip(face_refs,f)])
        all_vertex_angles=np.degrees(np.arccos(np.clip(np.einsum('fi,fvi->fv',s['normal'],vertex_normals),-1,1)))
        out['normal']=dict(face_centroid_angular_error_deg=dist(angles),face_vertex_angular_error_deg=dist(all_vertex_angles.ravel()),
            inverted_triangles=int((dots<=0).sum()))
        out['sdf_residual']=dist(abs(residual))
        if callable(getattr(authority,'surface_diagnostics',None)):
            out['curvature']=authority.surface_diagnostics(x,s,dist)
    if metrics is not None:
        e=s['edges'];d=x[e[:,1]]-x[e[:,0]];avg=(metrics[e[:,0]]+metrics[e[:,1]])/2
        ratio=np.sqrt(np.einsum('ni,nij,nj->n',d,avg,d))
        out['sizing']=dict(actual_over_target=dist(ratio),requested_directional_size_m=dist(s['edge_length']/ratio),
            max_oversize_edge=e[ratio.argmax()].tolist(),min_ratio_edge=e[ratio.argmin()].tolist(),
            max_oversize_location=x[e[ratio.argmax()]].mean(0).tolist(),min_ratio_location=x[e[ratio.argmin()]].mean(0).tolist(),
            below_mmgs_optimization_length_071=int((ratio<.71).sum()),above_mmgs_optimization_length_14=int((ratio>1.4).sum()),
            interpretation='Endpoint-averaged live-SDF metric-edge length. 0.71/1.4 are MMGS local operation triggers, not a final V3 acceptance interval. MMGS uses graded transported metrics internally. No final hard size-ratio tolerance exists in the recovered V3 gate.',
            minimum_requested_size_m=float(1/np.sqrt(np.linalg.eigvalsh(metrics).max())),
            maximum_requested_size_m=float(1/np.sqrt(np.linalg.eigvalsh(metrics).min())))
        s['actual_over_target']=ratio
    return out,s

def certify_surface_face(authority,ref,triangle,contract,*,regional=None):
    """One face-acceptance owner; specialized owners retain their exact path."""
    if regional is None:return certify_face(authority,ref,triangle)
    from sdf_cfd_mesher_v4.sdf.regional_owner import CanonicalRegionScope
    if type(regional) is not CanonicalRegionScope:raise ValueError('RegionalScopeIssuerRequired')
    # Materialized NumPy scalars become their exact Python scalar values.
    # Do not round Fraction inputs or silently accept nonfinite coordinates.
    coordinates=tuple(tuple(v.item() if isinstance(v,np.generic) else v for v in row) for row in triangle)
    return regional.certify_face(authority,ref,coordinates,contract)


def sample_geometry(a,x,f,fr,vr,arrays,contract):
    """Existing limits at shared vertices/edge midpoints/centroids; no bound."""
    start=time.perf_counter()
    if len(set(fr))!=1 or set(vr)!=set(fr):raise ValueError('GeneralRootReferenceCoverageRequired')
    samples=arrays.pop('_numerical_samples',None)
    if samples is not None:
        position=samples['position'];normals=samples['normals'];sample_faces=samples['sample_faces'];sample_count=samples['sample_count']
    else:
        edges,inverse=np.unique(np.sort(np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]])),axis=1),axis=0,return_inverse=True)
        sites=np.concatenate((x,x[edges].mean(1),arrays['centre']))
        sample_faces=np.column_stack((f,(inverse.reshape(3,len(f)).T+len(x)),np.arange(len(f))+len(x)+len(edges)))
        ref=int(fr[0]);position=np.empty(len(sites));normals=np.empty_like(sites);sample_count=len(sites)
        for i,p in enumerate(sites):
            projected=a.project_sheet(ref,p)
            position[i]=np.linalg.norm(projected-p)
            normals[i]=a.exact_normal(ref,p)
    angles=np.arccos(np.clip(np.einsum('fi,fsi->fs',arrays['normal'],normals[sample_faces]),-1,1))
    estimates=position[sample_faces]
    finite=bool(np.isfinite([position]).all() and np.isfinite(normals).all() and np.isfinite(angles).all())
    passed=bool(finite and estimates.max()<=contract.position_limit_m and angles.max()<=contract.normal_limit_rad)
    return dict(status='NumericallyAccepted' if passed else 'Rejected',passed=passed,face_count=len(f),
        sample_count=sample_count,samples_per_face=7,shared_samples_reused=True,
        maximum_position_estimate_m=float(estimates.max()),maximum_normal_error_estimate_rad=float(angles.max()),
        position_limit_m=contract.position_limit_m,normal_limit_rad=contract.normal_limit_rad,
        method='Numerical projection displacement and source-gradient directions at vertices, unique edge midpoints and face centroids.',
        not_proved=['Whole-source coverage','Continuous face accuracy','Exact root/normal certificates','Native/real-target equivalence'],
        batch_statistics=None if samples is None else samples['batch_statistics'],
        seconds=time.perf_counter()-start)


def audit_directory(directory,a,contract,*,source_arrays=None,regional=None):
    from general_source_contract import GeneralSourceQualification
    if type(contract) is GeneralSourceQualification:
        from contextlib import ExitStack
        from pathlib import Path
        from immutable_state_access import guard
        # The exact intersection traversal and local numeric audit consume the
        # same authenticated files for the entire acceptance call.
        with ExitStack() as held:
            for name in ('SURFACE.npz','REFERENCES.npz','surface.off'):
                held.enter_context(guard(Path(directory)/name))
            contract.verify(a)
            result=_audit_directory(directory,a,contract,source_arrays=source_arrays,regional=regional)
            contract.verify(a)
            return result
    return _audit_directory(directory,a,contract,source_arrays=source_arrays,regional=regional)


def _audit_directory(directory,a,contract,*,source_arrays=None,regional=None):
    from general_source_contract import GeneralSourceQualification,array_binding
    general=type(contract) is GeneralSourceQualification
    if general:
        if a.construction_active():raise ValueError('ConstructionCannotQualifyMesh')
        contract.verify(a)
        if regional is not None and regional is not a.queries._scope:raise ValueError('DetachedRegionalQualification')
        regional=a.queries._scope
    elif regional is not None:raise ValueError('GeneralSurfaceAdmissionNotEstablished')
    from pathlib import Path
    from historical_capsule_surface import provider
    from polygon_boundary_contract import SourceBoundaryContract,enforce_and_compose
    source_result=None
    if getattr(a,'packet',None) is not None:
        if not isinstance(contract,SourceBoundaryContract):
            raise ValueError('SourceBoundaryContractRequired')
        if source_arrays is not None:
            # Diagnostic entry into the SAME post-adaptation geometry qualification
            # boundary. No files written, metric guessed, or publication allowed.
            return enforce_and_compose(a,contract,**source_arrays)
        with np.load(Path(directory)/'SURFACE.npz') as z, np.load(Path(directory)/'REFERENCES.npz') as r:
            source_result=enforce_and_compose(a,contract,z['coordinates'],z['triangles'],r['face_refs'],r['curve_edges'],r['corner_refs'])
        # Retain this completed substage even if a later qualification stage fails.
        save(Path(directory)/'SOURCE_BOUNDARY_AUDIT.json',source_result)
        if not source_result['geometry_requirements_passed']:
            return source_result
    elif source_arrays is not None:
        raise ValueError('SourceBoundaryContractRequired')
    directory=Path(directory);t=time.perf_counter();contract.validate()
    z=np.load(directory/'SURFACE.npz');x=z['coordinates'];f=z['triangles'];m=z['metric']
    if (directory/'REFERENCES.npz').exists():
        refs=np.load(directory/'REFERENCES.npz');fr=refs['face_refs'];vr=refs['vertex_refs'];ce=refs['curve_edges']
    elif len(contract.sheet_names)==1 and not contract.curve_names:
        ref=next(iter(contract.sheet_names));fr=np.full(len(f),ref);vr=np.full(len(x),ref);ce=np.empty((0,3),int)
    else:raise ValueError('PersistentReferenceLedgerRequired')
    if set(fr)!=set(contract.sheet_names) or set(ce[:,2])!=set(contract.curve_names):raise ValueError('QualificationReferenceCoverageFailure')
    cr=refs['corner_refs'] if general else None
    values=(x,f,m,fr,vr,ce,cr)
    if general:
        with a.acceptance_scope(contract,values) as values:
            result=_audit_materialized(directory,a,contract,*values,source_result,general,regional,t)
        # Inner AUDIT is diagnostic until the input owner exits successfully.
        result['acceptance_custody_completed']=True
        save(directory/'AUDIT.json',result)
        return result
    return _audit_materialized(directory,a,contract,*values,source_result,general,regional,t)


def _audit_materialized(directory,a,contract,x,f,m,fr,vr,ce,cr,source_result,general,regional,t):
    from general_source_contract import array_binding,snapshot
    from dataclasses import asdict
    from historical_capsule_surface import provider,read_off
    owner=contract
    numerical=general and a.numerical_acceptance_active()
    contract=owner.batch_contract() if general else contract
    limits_pin=snapshot(asdict(contract)) if general else None
    source_coverage=None;source_coverage_seconds=None
    if numerical:
        source_coverage=dict(status='NotRun',passed=None,reason='Routine numerical surface acceptance; whole-source completeness is not proved.')
        save(directory/'SOURCE_COVERAGE.json',source_coverage)
    elif callable(getattr(owner,'check_source_coverage',None)):
        coverage_start=time.perf_counter()
        source_coverage=owner.check_source_coverage(a,x,f,fr,ce,cr)
        source_coverage_seconds=time.perf_counter()-coverage_start
        save(directory/'SOURCE_COVERAGE.json',dict(source_coverage,seconds=source_coverage_seconds))
    actual=None
    if general:
        off_x,off_f=read_off(directory/'surface.off')
        if off_x.shape!=x.shape or off_f.shape!=f.shape or off_x.tobytes()!=x.tobytes() or not np.array_equal(off_f,f):
            raise ValueError('DetachedGeneralIntersectionSurface')
        actual=dict(zip(('coordinates','faces','face_refs','curve_edges','corner_refs'),
                        (array_binding(v) for v in (x,f,fr,ce,cr))))
        if (x.dtype!=np.dtype('float64') or x.ndim!=2 or x.shape[1]!=3 or not len(x) or not np.isfinite(x).all()
                or f.ndim!=2 or f.shape[1]!=3 or f.dtype.kind not in 'iu' or not len(f)
                or np.any(f<0) or np.any(f>=len(x)) or fr.shape!=(len(f),) or vr.shape!=(len(x),)
                or fr.dtype.kind not in 'iu' or vr.dtype.kind not in 'iu' or set(vr)!=set(contract.sheet_names)
                or ce.shape!=(0,3) or ce.dtype.kind not in 'iu'
                or cr.shape!=(len(x),) or cr.dtype.kind not in 'iu' or np.any(cr)):
            raise ValueError('GeneralRootReferenceCoverageRequired')
        if not numerical and (source_coverage is None or not source_coverage['passed'] or any(
            source_coverage['materialized_surface'][k]!=v for k,v in actual.items())):
            raise ValueError('DetachedFinalSourceCoverage')
        if (m.shape!=(len(x),3,3) or not np.isfinite(m).all() or not np.array_equal(m,m.transpose(0,2,1))):
            raise ValueError('FiniteSymmetricSurfaceMetricsRequired')
        for tensor in m:a.validate_composed_metric(tensor)
    final_points=a.qualify_final_points(x,vr) if general and not numerical else None
    result,arrays=audit_arrays(x,f,a,m,face_refs=fr,vertex_refs=vr,contract=contract)
    # Private handoff inside this final acceptance call. Freeze the same final
    # sampled values; construction queries never participate in this reuse.
    ownership_samples=None
    if numerical:
        from sdf_cfd_mesher_v4.common.authority import frozen
        samples=arrays.get('_numerical_samples')
        if samples is not None:
            ownership_samples=dict(samples,normals=frozen(samples['normals'],samples['normals'].dtype.str))

    if final_points is not None:result['final_point_qualification']=final_points
    elif numerical:result['final_point_qualification']=dict(status='NotRun',passed=None,
        reason='No exact final-bit root or normal-cone certificate in routine numerical acceptance.')
    if source_coverage is not None:
        result['source_coverage']=source_coverage
        result['source_coverage_seconds']=source_coverage_seconds
    np.savez_compressed(directory/'QUALITY.npz',**{k:v for k,v in arrays.items() if isinstance(v,np.ndarray)})
    if numerical:
        geometry=sample_geometry(a,x,f,fr,vr,arrays,contract)
        save(directory/'NUMERICAL_GEOMETRY.json',geometry)
        result['continuous_geometry']=dict(status='NotRun',passed=None,face_count=len(f),
            reason='Sampled numerical geometry acceptance is recorded separately; no continuous bounds.')
        result['numerical_geometry']=geometry
    else:
        result['continuous_geometry']=dict(counts={},maximum_position_bound=0.,maximum_normal_bound_rad=0.)
        counts=collections.Counter();methods=collections.Counter();bounds=np.empty((len(f),2));statuses=[]
        start=time.perf_counter()
        for i,face in enumerate(f):
            status,position,normal,method=certify_surface_face(a,int(fr[i]),x[face],owner,regional=regional);counts[status]+=1;methods[method]+=1
            bounds[i]=[position,normal];statuses.append(status)
            if i%2000==0:print(json.dumps(dict(stage='continuous_surface_geometry',done=i,total=len(f),counts=counts)),flush=True)
        geometry=dict(counts=dict(counts),methods=dict(methods),maximum_position_bound=float(bounds[:,0].max()),
            maximum_normal_bound_rad=float(bounds[:,1].max()),seconds=time.perf_counter()-start,
            passed=counts['Certified']==len(f),face_count=len(f))
        np.savez_compressed(directory/'GEOMETRY_CERTIFICATE.npz',bounds=bounds,status=np.array(statuses))
        result['continuous_geometry']=geometry
    from owned_runtime import BIN, verify as verify_runtime
    verify_runtime()
    timing,stdout=run_native([wsl(BIN/'intersections'),wsl(directory/'surface.off')],directory/'intersections')
    intersections=json.loads(stdout);intersections.update(timing=timing,method='CGAL Polygon_mesh_processing::self_intersections; exact filtered predicates; complete surface pairs; legal shared adjacency excluded',
        missing=0,duplicate=0,coverage='One complete CGAL traversal of every input triangle; no persisted pair-receipt ledger')
    result['self_intersection']=intersections
    feature_bounds=[feature_bound(a,x[int(v)],x[int(w)],int(ref)) for v,w,ref in ce]
    feature_pass=all(math.isfinite(b) and b<=contract.position_limit_m for b in feature_bounds)
    if numerical:
        from surface_ownership_batch import topology_report
        ownership=topology_report(x,f,fr,ce,a,ownership_samples,arrays['centre'])
    else:ownership=provider.topology_report(x,f,fr,ce,None,a)
    result['feature_ownership']=dict(curve_count=len(contract.curve_names),edges=len(ce),
        maximum_position_bound=max(feature_bounds,default=0.),passed=feature_pass and ownership['topology_pass'],topology=ownership)
    result['geometry_identity']=a.geometry_identity
    if source_result is not None:result['source_boundary']=source_result
    result['expected_topology']=dict(components=contract.expected_components,euler=contract.expected_euler,boundary_loops=contract.expected_boundary_loops)
    result['passed']=bool(result['topology']['passed'] and result['normal']['inverted_triangles']==0 and geometry['passed']
        and intersections['forbidden']==0 and intersections['unresolved']==0
        and result['sdf_residual']['max']<=contract.projection_tolerance_m and result['feature_ownership']['passed'])
    if source_result is not None:result['passed']=result['passed'] and source_result['geometry_requirements_passed']
    if general:
        if a.construction_active():raise ValueError('ConstructionCannotQualifyMesh')
        owner.verify(a)
        if snapshot(asdict(contract))!=limits_pin:raise ValueError('ChangedDetachedAcceptanceLimits')
        if any(array_binding(v)!=actual[k] for k,v in zip(actual,(x,f,fr,ce,cr))):raise ValueError('ChangedFinalSourceCoverage')
        if not numerical:result['passed']=result['passed'] and source_coverage['passed'] and final_points['passed']
        result['surface_assurance']=owner.assurance
        result['acceptance_inputs']=dict(actual,metric=array_binding(m),vertex_refs=array_binding(vr))
        result['acceptance_contract']=dict(continuous_target=owner.target,limits=a.acceptance_contract(owner),assurance=owner.assurance)
    result['vertex_projection_tolerance_m']=contract.projection_tolerance_m
    result['vertex_projection_tolerance_source']='V3 held_out_geometry_cohort/src/cohort_surface.cpp: projection_tolerance'
    result['branch_continuity']=dict(persistent_sheets=len(contract.sheet_names),unknown_sheets=0,expected_components=contract.expected_components,measured_components=result['topology']['connected_components'],
        connectivity_authority='CGAL implicit-domain surface facets and MMGS adaptation',directional_sampling=False,
        diagnostic_ray_tests='Not measured.',analytic_fact=contract.premise)
    result['seconds']=time.perf_counter()-t
    result['peak_rss_bytes']=psutil.Process().memory_info().peak_wset
    result['retained_rss_bytes']=psutil.Process().memory_info().rss
    save(directory/'AUDIT.json',result)
    print(json.dumps(dict(label=directory.name,passed=result['passed'],vertices=len(x),triangles=len(f),edge=result['edge'],geometry=geometry,intersections=intersections,seconds=result['seconds'])),flush=True)
    return result


def main():
    from historical_capsule_surface import capsule_binding
    a=CapsuleGeometry(json.loads((OUT/'GEOMETRY.json').read_text())['definition'])
    audit_directory(OUT/sys.argv[1],a,capsule_binding(a).qualification)

if __name__=='__main__':main()
