"""Bounded finite-panel witness checks; no extraction, adaptation or publication."""
import argparse, copy, hashlib, json, sys, tempfile, time
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
from replay_surface_dispatch import verify_kernel_sources
from polygon_boundary_contract import exact,rational,curve_endpoint_enclosure
from audit_adaptive_surface import audit_directory
from historical_capsule_surface import provider,canonical
from unittest.mock import patch


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--phase',choices=['before','after'],required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=False);start=time.perf_counter()
    def save(name,value):(out/name).write_text(json.dumps(value,indent=2,allow_nan=False,default=lambda x:x.tolist())+'\n',newline='\n')
    source,direct=verify_kernel_sources();case=next(c for c in direct['cases'] if c['name']=='plain_wing')
    old=V4/'results/polygon_numerics/accepted_scope'
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
        g=owner.load_rhai(case['script'],case['metres_per_model_unit']);binding=bind_canonical(g)
        a=binding.authority;c=binding.qualification
        with np.load(old/'OUTSIDE_PROJECTED_CURVE_INPUT.npz') as z:failed={k:z[k].copy() for k in z.files}
        if args.phase=='before':
            try:audit_directory(out,a,c,source_arrays=failed)
            except GeometryFailure as err:
                assert err.record['code']=='FiniteSheetDomainFailure'
                assert [(r['face'],r['sheet'],r['vertices']) for r in err.record['detail']['failures']]==[(780,1,[390]),(787,1,[390])]
                save('FAILURE.json',err.record)
            else:raise AssertionError('Retained failure did not reproduce')
        results={}
        if args.phase=='after':
            import polygon_boundary_contract as boundary
            def run(name,data):
                before={k:v.copy() for k,v in data.items()};t=time.perf_counter()
                r=audit_directory(out,a,c,source_arrays=data)
                with tempfile.TemporaryDirectory(prefix='v4-panel-witness-') as d:
                    np.savez(Path(d)/'SURFACE.npz',coordinates=data['x'],triangles=data['faces'])
                    np.savez(Path(d)/'REFERENCES.npz',**{k:v for k,v in data.items() if k not in ('x','faces')})
                    assert audit_directory(d,a,c)==r
                assert all(np.array_equal(before[k],data[k]) for k in data)
                assert r['acceptance']=='not_evaluated' and not r['passed'] and not r['geometry_requirements_passed']
                results[name]={k:r[k] for k in ('triangles_checked','finite_domain_failures','position_bound_max_m',
                    'normal_bound_max_rad','curve_source_bound_max_m','unresolved','residual_gate','acceptance','contract_identity')}
                results[name].update(file_backed_equal=True,coordinates_unchanged=True,seconds=time.perf_counter()-t,
                    witness_faces=sum(bool(b['panel_witnesses']) for b in r['bounds']))
                return r
            restored=run('retained_failed_projection',failed)
            assert len(restored['bounds'])==794 and not restored['unresolved']
            assert results['retained_failed_projection']['witness_faces']==2
            save('AFFECTED_FACES.json',[b for b in restored['bounds'] if b['panel_witnesses']])
            points=a.unit.points*a.scale;Q=list(map(rational,points));lookup={k:i for i,k in enumerate(a.unit.point_names)}
            view=SimpleNamespace(points=np.array(Q,dtype=object),curve_vertices=a.unit.curve_vertices)
            domains={};planarity=0
            for ref,name in a.unit.sheet_names.items():
                loop=[Q[lookup[k]] for k in a.unit.brep.loops[name]]
                N=exact.cross(exact.sub(loop[1],loop[0]),exact.sub(loop[2],loop[0]))
                hs=[(exact.cross(N,exact.sub(loop[(i+1)%len(loop)],p)),p) for i,p in enumerate(loop)]
                assert exact.nonzero(N) and all(exact.dot(N,exact.sub(p,loop[0]))==0 and exact.clip_polygon([p],hs) for p in loop)
                domains[ref]=(N,loop[0],hs);planarity+=len(loop)
            prior=json.loads((old/'PROJECTED_CURVE_MIDPOINTS.json').read_text());prior={r['ref']:r for r in prior}
            rows=[];enclosures=[];sheet_ids={name:ref for ref,name in a.unit.sheet_names.items()}
            for ref,(u,v) in a.unit.curve_vertices.items():
                raw=(points[u]+points[v])/2;p=a.project_curve(ref,raw)
                assert np.array_equal(p,prior[ref]['submitted_m'])
                ideal=tuple(a.unit.project_curve.__func__(view,ref,np.array(rational(raw),dtype=object)))
                ideal_delta=exact.sub(rational(p),ideal)
                key=tuple(sorted((a.unit.point_names[u],a.unit.point_names[v])))
                owners=[sheet_ids[s[0]] for s in a.unit.brep.edges[key]]
                strict=all(exact.clip_polygon([rational(p)],domains[s][2]) for s in owners)
                exact_member=exact.onsegment(rational(p),Q[u],Q[v])
                owned=[]
                for s in owners:
                    row=boundary.panel_curve_witness(a,view,ref,p,[Q[u],Q[v]],domains[s])
                    assert row is not None
                    w=tuple(map(F,row['witness_exact_m']));N,o,hs=domains[s]
                    assert exact.dot(N,exact.sub(w,o))==0 and exact.clip_polygon([w],hs)
                    owned.append(s);enclosures.append(row['endpoint_bound_m'])
                rows.append(dict(curve=ref,point_m=p,actual_projection_bits_equal=True,exact_segment_member=exact_member,
                    strict_all_panel_footprints=bool(strict),witness_panels=owned,
                    projector_vs_rational_lift_squared_m2=str(exact.dot(ideal_delta,ideal_delta)),
                    materialized_distance_upper_m=row['materialized_distance_upper_m'],endpoint_bound_m=row['endpoint_bound_m']))
            save('MIDPOINTS.json',rows)
            results['midpoints']=dict(count=len(rows),exact_members=sum(r['exact_segment_member'] for r in rows),
                nonexact_strict_pass=sum(not r['exact_segment_member'] and r['strict_all_panel_footprints'] for r in rows),
                nonexact_strict_fail=sum(not r['exact_segment_member'] and not r['strict_all_panel_footprints'] for r in rows),
                supported_witnesses=sum(len(r['witness_panels']) for r in rows),unresolved=0,
                source_loop_vertex_checks=planarity,all_nonexact_differ_from_rational_projector=all(F(r['projector_vs_rational_lift_squared_m2'])>0 for r in rows if not r['exact_segment_member']),
                max_materialized_distance_upper_m=max(r['materialized_distance_upper_m'] for r in rows),
                max_endpoint_bound_m=max(enclosures))
            assert (results['midpoints']['exact_members'],results['midpoints']['nonexact_strict_pass'],results['midpoints']['nonexact_strict_fail'])==(297,100,200)
            # Corroborate whole-triangle bounds with exact convex witnesses. The
            # proof is convexity and the norm triangle inequality, not sampling.
            barycentric=[(F(1),F(0),F(0)),(F(0),F(1),F(0)),(F(0),F(0),F(1)),(F(1,3),)*3,(F(1,2),F(1,4),F(1,4))]
            checks=0
            for row in restored['bounds']:
                if not row['panel_witnesses']:continue
                f=failed['faces'][row['face']];N,o,hs=domains[row['sheet']];nn=exact.dot(N,N)
                P=[rational(failed['x'][j]) for j in f];special={w['vertex']:tuple(map(F,w['witness_exact_m'])) for w in row['panel_witnesses']}
                W=[special[int(j)] if int(j) in special else tuple(p[k]-N[k]*exact.dot(N,exact.sub(p,o))/nn for k in range(3)) for j,p in zip(f,P)]
                assert all(exact.dot(N,exact.sub(w,o))==0 and exact.clip_polygon([w],hs) for w in W)
                for weights in barycentric:
                    p=tuple(sum(t*q[k] for t,q in zip(weights,P)) for k in range(3));w=tuple(sum(t*q[k] for t,q in zip(weights,W)) for k in range(3))
                    d=exact.sub(p,w)
                    assert exact.clip_polygon([w],hs) and exact.dot(N,exact.sub(w,o))==0
                    assert exact.dot(d,d)<=F(row['materialized_panel_distance_upper_m'])**2
                    checks+=1
            results['whole_triangle_exact_checks']=checks
            rejects=[]
            def reject(name,data,code):
                try:audit_directory(out,a,c,source_arrays=data)
                except GeometryFailure as e:
                    assert e.record['code']==code,(name,e.record)
                    rejects.append(dict(name=name,result=e.record));return
                raise AssertionError(name)
            # No projection-provenance Boolean is accepted by the qualifier. A
            # fabricated displacement gets its actual bound, never a roundoff tag.
            displaced=copy.deepcopy(failed);displaced['x'][390,1]-=a.scale/1000
            dr=run('material_displacement',displaced)
            assert dr['position_bound_max_m']>=a.scale/1000 and not dr['residual_gate']['passed']
            wrong=copy.deepcopy(failed)
            slots=np.flatnonzero(np.any(wrong['curve_edges'][:,:2]==390,axis=1))
            ref=int(wrong['curve_edges'][slots[0],2]);other=next(i for i,r in enumerate(wrong['curve_edges'][:,2]) if r!=ref)
            wrong['curve_edges'][slots,2],wrong['curve_edges'][other,2]=int(wrong['curve_edges'][other,2]),ref
            reject('wrong_curve_incidence',wrong,'SourceCurveIncidenceFailure')
            inverted=copy.deepcopy(failed);inverted['faces'][780]=inverted['faces'][780][::-1]
            reject('inverted_triangle',inverted,'SurfaceTopologyFailure')
            # The same finite-panel witness validator rejects corrupted endpoint
            # domains and fabricated out-of-segment witnesses. No geometry change.
            row=next(b for b in restored['bounds'] if b['panel_witnesses']);entry=row['panel_witnesses'][0]
            ref=entry['curve'];u,v=a.unit.curve_vertices[ref];p=failed['x'][entry['vertex']]
            source_key=tuple(sorted((a.unit.point_names[u],a.unit.point_names[v])))
            incident_sheets={sheet_ids[t[0]] for t in a.unit.brep.edges[source_key]}
            other_sheet=next(s for s in domains if s not in incident_sheets)
            try:boundary.panel_curve_witness(a,view,ref,p,[Q[u],Q[v]],domains[other_sheet])
            except GeometryFailure as e:
                assert e.record['code']=='FinitePanelWitnessFailure';rejects.append(dict(name='nonincident_source_panel',result=e.record))
            else:raise AssertionError('nonincident panel')
            actual=curve_endpoint_enclosure(a,view,ref,p,[Q[u],Q[v]])
            fake=copy.deepcopy(actual);fake['witness_exact_m']=[str(2*x-y) for x,y in zip(Q[u],Q[v])]
            with patch.object(boundary,'curve_endpoint_enclosure',return_value=fake):
                reject('fabricated_out_of_segment_witness',failed,'FinitePanelWitnessFailure')
            understated=copy.deepcopy(actual);understated['endpoint_bound_m']=0.
            with patch.object(boundary,'curve_endpoint_enclosure',return_value=understated):
                reject('understated_witness_displacement',failed,'NumericalEnclosureFailure')
            original=boundary.curve_endpoint_enclosure
            def unavailable(*args):return None if args[2]==ref else original(*args)
            with patch.object(boundary,'curve_endpoint_enclosure',side_effect=unavailable):
                unresolved=audit_directory(out,a,c,source_arrays=failed)
            assert not unresolved['geometry_requirements_passed']
            assert sum(r['operation']=='finite_panel_witness' for r in unresolved['unresolved'])==2
            results['unavailable_witness']=unresolved['unresolved']
            save('REJECTIONS.json',rejects)
            results['adversarial_rejections']=len(rejects)
        save('SUMMARY.json',dict(phase=args.phase,geometry=g.identity,topology=a.packet['topology_identity'],source=source,
            results=results,
            seconds=time.perf_counter()-start,checker_peak_bytes=psutil.Process().memory_info().peak_wset,
            host_peak_bytes=psutil.Process(owner.transport.process.pid).memory_info().peak_wset,
            implementation_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),V4/'tools/polygon_boundary_contract.py']}))


if __name__=='__main__':main()
