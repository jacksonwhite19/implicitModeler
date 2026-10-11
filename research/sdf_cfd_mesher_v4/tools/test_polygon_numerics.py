"""Bounded source-boundary numerical checks. No extraction or adaptation."""
import argparse,copy,json,sys,time,tempfile,hashlib
from pathlib import Path
from fractions import Fraction as F
from types import SimpleNamespace
import numpy as np
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from surface_binding import bind_canonical
from audit_adaptive_surface import audit_directory
from replay_surface_dispatch import verify_kernel_sources
from polygon_boundary_contract import exact,rational,identity
from historical_capsule_surface import provider,canonical


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--phase',choices=['before','after'],required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=False);start=time.perf_counter()
    def save(name,value):
        (out/name).write_text(json.dumps(value,indent=2,allow_nan=False,default=lambda v:v.tolist())+'\n',newline='\n')
    source,direct=verify_kernel_sources();case=next(c for c in direct['cases'] if c['name']=='plain_wing')
    results={};events=[]
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
        g=owner.load_rhai(case['script'],case['metres_per_model_unit']);binding=bind_canonical(g)
        a=binding.authority;c=binding.qualification
        with np.load(V4/'results/canonical_polygon_connection/labelled_seed/surface/A/seed/SEED.npz') as z:
            x=z['points']*a.scale;f=z['faces'].copy();fr=z['refs'].copy()
        ce=np.array([(*edge,r) for r,edge in a.unit.curve_vertices.items()],int)
        cr=np.array([a.unit.corner_ref[name] for name in a.unit.point_names],int)
        arrays=dict(x=x,faces=f,face_refs=fr,curve_edges=ce,corner_refs=cr)
        def run(name,data,file_check=False):
            t=time.perf_counter();r=audit_directory(out,a,c,source_arrays=data)
            if file_check:
                with tempfile.TemporaryDirectory(prefix='v4-polygon-numerics-') as d:
                    np.savez(Path(d)/'SURFACE.npz',coordinates=data['x'],triangles=data['faces'])
                    np.savez(Path(d)/'REFERENCES.npz',face_refs=data['face_refs'],curve_edges=data['curve_edges'],corner_refs=data['corner_refs'])
                    assert audit_directory(d,a,c)==r
            results[name]={k:r[k] for k in ('triangles_checked','finite_domain_failures','position_bound_max_m','normal_bound_max_rad',
                'curve_source_bound_max_m','unresolved','residual_gate','acceptance','geometry_requirements_passed','contract_identity')}
            results[name].update(seconds=time.perf_counter()-t,file_backed_equal=file_check)
            assert r['acceptance']=='not_evaluated' and not r['passed']
            print(json.dumps(dict(case=name,unresolved=len(r['unresolved']),seconds=results[name]['seconds'])),flush=True)
            return r
        seed=run('seed',arrays,args.phase=='after')
        prior=json.loads((V4/'results/source_boundary_contract/verified/SUMMARY.json').read_text())['valid_seed']
        assert seed['position_bound_max_m']==prior['position_bound_max_m']
        assert seed['normal_bound_max_rad']==prior['normal_bound_max_rad']
        cap=next(r for r,n in a.unit.sheet_names.items() if '/cap/positive' in n)
        fi=int(np.flatnonzero(fr==cap)[0]);u,v,w=f[fi];j=len(x)
        changed=x[f[fi]].mean(0);changed[1]=np.nextafter(changed[1],np.inf)
        retained=json.loads((V4/'results/source_boundary_contract/verified/NONPARALLEL.json').read_text())
        assert np.array_equal(changed,retained['changed_vertex_m'])
        ulp=copy.deepcopy(arrays);ulp['x']=np.vstack([x,changed]);ulp['corner_refs']=np.append(cr,0)
        ulp['faces']=np.vstack([np.delete(f,fi,axis=0),[u,v,j],[v,w,j],[w,u,j]])
        ulp['face_refs']=np.append(np.delete(fr,fi),[cap,cap,cap])
        np.savez_compressed(out/'ONE_ULP_INPUT.npz',**ulp)
        if args.phase=='after':
            with np.load(V4/'results/polygon_numerics/before/ONE_ULP_INPUT.npz') as before:
                assert all(np.array_equal(value,before[key]) for key,value in ulp.items())
        r=run('one_ulp',ulp,True)
        save('ONE_ULP_FACES.json',r['bounds'][-3:])
        assert len(r['unresolved'])==(3 if args.phase=='before' else 0)
        # Existing finite projector evaluated unchanged on exact rational inputs.
        # This is a reference execution of its method, not a second projector.
        view=SimpleNamespace(points=np.array([rational(p) for p in x],dtype=object),curve_vertices=a.unit.curve_vertices)
        checks=[]
        for ref,(u,v) in a.unit.curve_vertices.items():
            p=(x[u]+x[v])/2;q=a.project_curve(ref,p)
            witness=a.unit.project_curve.__func__(view,np.int64(ref),np.array(rational(q),dtype=object))
            assert exact.onsegment(tuple(witness),tuple(view.points[u]),tuple(view.points[v]))
            checks.append(dict(ref=ref,exact_member=exact.onsegment(rational(q),rational(x[u]),rational(x[v])),
                submitted_m=q,exact_distance_squared=str(exact.dot(exact.sub(rational(q),tuple(witness)),exact.sub(rational(q),tuple(witness))))))
        save('PROJECTED_CURVE_MIDPOINTS.json',checks)
        results['projected_curve_midpoints']=dict(count=len(checks),nonexact=sum(not r['exact_member'] for r in checks))
        if args.phase=='after':
            from polygon_boundary_contract import curve_endpoint_enclosure
            from polygon_conversion import direction_bound
            from continuous_geometry_certifier import feature_bound
            import mpmath as mp
            mp.mp.dps=100
            def real(q):return mp.mpf(q.numerator)/q.denominator
            def independent_face_checks(data,report):
                differences=[];physical=[];float_direction=[]
                for row in report['bounds']:
                    tri=data['x'][data['faces'][row['face']]]
                    P=list(map(rational,tri));T=exact.cross(exact.sub(P[1],P[0]),exact.sub(P[2],P[0]))
                    n=rational(a.exact_normal(row['sheet'],tri.mean(0)))
                    C=exact.cross(T,n);d=exact.dot(T,n);den=exact.dot(T,T)*exact.dot(n,n)
                    sin2=exact.dot(C,C)/den
                    bound=row['exact_triangle_to_branch_enclosure_rad']
                    assert bound is not None and F(bound)**2>=4*sin2 and d>0
                    angle=mp.atan2(mp.sqrt(real(exact.dot(C,C))),real(d))
                    assert angle<=mp.mpf(bound)
                    physical.append(float(angle));differences.append(float(mp.mpf(bound)-angle))
                    fc=np.cross(tri[1]-tri[0],tri[2]-tri[0]);fn=fc/np.linalg.norm(fc)
                    float_direction.append(direction_bound(rational(fn),T))
                return dict(count=len(physical),rational_enclosure_failures=0,
                    high_precision_corroboration_digits=100,max_physical_face_branch_angle_rad=max(physical),
                    min_enclosure_minus_mp_angle_rad=min(differences),max_float_normal_direction_enclosure_rad=max(float_direction))
            results['independent_seed']=independent_face_checks(arrays,seed)
            results['independent_one_ulp']=independent_face_checks(ulp,r)
            def project(data):
                q,ifs,ics,edges=provider.canonicalize_and_project(a,data['x'],data['faces'],data['face_refs'],
                    data['curve_edges'][:,:2],data['curve_edges'][:,2],np.flatnonzero(data['corner_refs']))
                names=binding.corner_restorer(q,ics)
                q,faces,order,inverse,face_order=canonical(q,data['faces'],return_permutations=True)
                return dict(x=q,faces=faces,face_refs=data['face_refs'][face_order],
                    curve_edges=np.array([(inverse[u],inverse[v],ref) for u,v,ref in edges],int),
                    corner_refs=data['corner_refs'][order]),len(names)
            projected,ncorners=project(ulp)
            np.savez_compressed(out/'PROJECTED_FACE_INPUT.npz',**projected)
            rp=run('actual_projected_faces',projected,True)
            assert not rp['unresolved'] and ncorners==398
            results['independent_projected_faces']=independent_face_checks(projected,rp)
            # Select by deterministic source reference, report ALL midpoint domain
            # outcomes, and keep a failing real projection too. No coordinate repair.
            owners={int(ref):set() for _,_,ref in ce}
            for u,v,ref in ce:
                owners[int(ref)]={int(fr[i]) for i,face in enumerate(f) if int(u) in face and int(v) in face}
            domains={}
            for ref,name in a.unit.sheet_names.items():
                loop=[rational(np.array(a.unit.brep.points[k])*a.scale) for k in a.unit.brep.loops[name]]
                N=exact.cross(exact.sub(loop[1],loop[0]),exact.sub(loop[2],loop[0]))
                domains[ref]=[(exact.cross(N,exact.sub(loop[(i+1)%len(loop)],p)),p) for i,p in enumerate(loop)]
            good=[];outside=[];curve_measurements=[]
            for row in checks:
                ref=row['ref'];u,v=a.unit.curve_vertices[ref];p=np.array(row['submitted_m'])
                entry=curve_endpoint_enclosure(a,view,ref,p,[rational(x[u]),rational(x[v])])
                assert entry is not None
                witness=tuple(a.unit.project_curve.__func__(view,ref,np.array(rational(p),dtype=object)))
                D=exact.sub(rational(x[v]),rational(x[u]));P=rational(p)
                # Independent finite-segment optimality conditions, not another projector.
                if witness==rational(x[u]):assert exact.dot(exact.sub(P,witness),D)<=0
                elif witness==rational(x[v]):assert exact.dot(exact.sub(P,witness),D)>=0
                else:assert exact.dot(exact.sub(P,witness),D)==0
                assert F(entry['materialized_distance_upper_m'])**2>=F(entry['distance_squared_exact'])
                fits=all(exact.clip_polygon([P],domains[s]) for s in owners[ref])
                curve_measurements.append(dict(curve=ref,domain_pass=fits,**entry))
                if not row['exact_member']:(good if fits else outside).append(ref)
            save('CURVE_ENCLOSURES.json',curve_measurements)
            results['curve_enclosures']=dict(count=len(curve_measurements),independent_KKT_failures=0,
                nonexact_inside_all_owners=len(good),nonexact_outside_an_owner=len(outside),
                max_materialized_distance_m=max(q['materialized_distance_upper_m'] for q in curve_measurements))
            def split_edge(ref):
                u,v=a.unit.curve_vertices[ref];j=len(x);data=copy.deepcopy(arrays)
                data['x']=np.vstack([x,(x[u]+x[v])/2]);data['corner_refs']=np.append(cr,0)
                faces=[];refs=[]
                for face,s in zip(f,fr):
                    if u in face and v in face:
                        for k in range(3):
                            p,q,z=map(int,[face[k],face[(k+1)%3],face[(k+2)%3]])
                            if {p,q}=={u,v}:faces.extend([[p,j,z],[j,q,z]]);refs.extend([s,s]);break
                    else:faces.append(face);refs.append(s)
                data['faces']=np.array(faces,int);data['face_refs']=np.array(refs,int)
                data['curve_edges']=np.vstack([ce[ce[:,2]!=ref],[u,j,ref],[j,v,ref]])
                return data
            assert good and outside
            pc,n=project(split_edge(good[0]));np.savez_compressed(out/'PROJECTED_CURVE_INPUT.npz',**pc)
            rc=run('actual_projected_curve',pc,True)
            assert not rc['unresolved'] and n==398
            curve_rows=[row for row in rc['curve_evidence'] if row['curve']==good[0]]
            assert any(not e['exact_member'] for row in curve_rows for e in row['endpoints'])
            edge_checks=0
            for row in curve_rows:
                ref=row['curve'];u0,v0=a.unit.curve_vertices[ref]
                P,Q=map(rational,pc['x'][row['edge']])
                endpoint_bound=max(e['endpoint_bound_m'] for e in row['endpoints'])
                for t in (F(0),F(1,4),F(1,2),F(3,4),F(1)):
                    point=tuple((1-t)*p+t*q for p,q in zip(P,Q))
                    witness=tuple(a.unit.project_curve.__func__(view,ref,np.array(point,dtype=object)))
                    delta=exact.sub(point,witness)
                    assert exact.onsegment(witness,rational(x[u0]),rational(x[v0]))
                    assert exact.dot(delta,delta)<=F(endpoint_bound)**2
                    edge_checks+=1
            results['whole_edge_corroboration']=dict(exact_checks=edge_checks,failures=0,
                proof='convex interpolation of finite endpoint witnesses; sampled checks are corroboration only')
            save('SELECTED_CURVE.json',curve_rows)
            results['independent_projected_curve_faces']=independent_face_checks(pc,rc)
            bad,_=project(split_edge(outside[0]))
            np.savez_compressed(out/'OUTSIDE_PROJECTED_CURVE_INPUT.npz',**bad)
            def reject(name,data,expected):
                try:audit_directory(out,a,c,source_arrays=data)
                except GeometryFailure as err:
                    assert err.record['code']==expected,(name,err.record)
                    events.append(dict(case=name,error=err.record));return
                raise AssertionError(name+' was not rejected')
            outside_report=run('actual_projector_outside_finite_owner',bad,True)
            assert not outside_report['unresolved']
            assert sum(bool(b['panel_witnesses']) for b in outside_report['bounds'])==2
            degenerate=copy.deepcopy(ulp);degenerate['x'][-1]=x[int(ulp['faces'][-3,0])]
            exact_degenerate=sum(not exact.nonzero(exact.cross(exact.sub(rational(tri[1]),rational(tri[0])),
                exact.sub(rational(tri[2]),rational(tri[0])))) for tri in degenerate['x'][degenerate['faces']])
            assert exact_degenerate==2
            reject('degenerate_triangle',degenerate,'SurfaceTopologyFailure')
            results['degeneracy']=dict(exact_zero_area_faces=exact_degenerate,expected_rejection='SurfaceTopologyFailure')
            steep=copy.deepcopy(ulp);steep['x'][-1,1]+=a.scale
            rs=run('steep_outside_angle_enclosure',steep)
            assert len(rs['unresolved'])==3 and not rs['residual_gate']['passed']
            # Endpoint, outside-endpoint, and interior evaluations through the
            # unchanged projector. These are diagnostic parameters, not tolerances.
            endpoint_rows=[]
            selected=[min(a.unit.curve_vertices,key=lambda ref:np.linalg.norm(x[a.unit.curve_vertices[ref][1]]-x[a.unit.curve_vertices[ref][0]])),
                      max(a.unit.curve_vertices,key=lambda ref:np.linalg.norm(x[a.unit.curve_vertices[ref][1]]-x[a.unit.curve_vertices[ref][0]]))]
            for ref in selected:
                u,v=a.unit.curve_vertices[ref];ends=[rational(x[u]),rational(x[v])]
                for t in (-1.,0.,1/3,.5,1.,2.):
                    raw=x[u]+t*(x[v]-x[u]);p=a.project_curve(ref,raw)
                    for role,point in [('raw',raw),('projected',p)]:
                        en=curve_endpoint_enclosure(a,view,ref,point,ends)
                        assert en is not None and F(en['materialized_distance_upper_m'])**2>=F(en['distance_squared_exact'])
                        endpoint_rows.append(dict(curve=ref,parameter=t,role=role,point_m=point,**en))
            save('ENDPOINT_CASES.json',endpoint_rows)
            results['endpoint_cases']=dict(count=len(endpoint_rows),nonexact=sum(not q['exact_member'] for q in endpoint_rows))
            zero_ref=next(ref for ref,(u,v) in a.unit.curve_vertices.items() if x[u,0]==x[v,0]==0 and x[u,1]!=x[v,1])
            u,v=a.unit.curve_vertices[zero_ref];tiny=(x[u]+x[v])/2;tiny[0]=np.nextafter(0.,1.)
            assert curve_endpoint_enclosure(a,view,zero_ref,tiny,[rational(x[u]),rational(x[v])]) is None
            results['interval_range_unresolved']=dict(curve=zero_ref,point_m=tiny,
                reason='positive exact squared distance rounds to zero; existing interval sqrt cannot enclose negative lower endpoint',
                result='unsupported, not a zero-distance success')
            # Existing generic Lipschitz certificate is still retained, not replaced.
            ref=selected[0];u,v=a.unit.curve_vertices[ref]
            legacy_feature=feature_bound(a,x[u],x[v],ref)
            assert np.isfinite(legacy_feature)
            results['existing_feature_bound']=dict(curve=ref,bound_m=legacy_feature,scope='shortest source curve; unchanged generic Lipschitz method')
            assert direction_bound((F(0),F(1),F(0)),(F(1),F(1),F(0))) is None
            assert direction_bound((F(0),F(0),F(0)),(F(0),F(1),F(0))) is None
            save('REJECTIONS.json',events)
        save('SUMMARY.json',dict(phase=args.phase,geometry=g.identity,topology=a.packet['topology_identity'],source=source,
            results=results,seconds=time.perf_counter()-start,checker_peak_bytes=psutil.Process().memory_info().peak_wset,
            host_peak_bytes=psutil.Process(owner.transport.process.pid).memory_info().peak_wset,
            implementation_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),V4/'tools/polygon_boundary_contract.py',V4/'tools/polygon_conversion.py']}))


if __name__=='__main__':main()
