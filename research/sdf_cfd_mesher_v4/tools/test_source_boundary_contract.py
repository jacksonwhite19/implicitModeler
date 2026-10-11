"""Actual qualifier boundary, retained seed and adversarial copies; no meshing."""
import argparse,copy,json,sys,time,struct,tempfile
from pathlib import Path
import numpy as np
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from surface_binding import bind_canonical
from polygon_boundary_contract import SourceBoundaryContract,identity,exact,rational
from audit_adaptive_surface import audit_directory
from replay_surface_dispatch import verify_kernel_sources
from canonical_polygon_authority import validate_topology,sha
from production_surface_authority import publish_surface


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=False);start=time.perf_counter()
    def save(name,obj):(out/name).write_text(json.dumps(obj,indent=2,allow_nan=False,default=lambda v:v.tolist())+'\n')
    source,direct=verify_kernel_sources();case=next(c for c in direct['cases'] if c['name']=='plain_wing')
    retained=V4/'results/canonical_polygon_connection'
    events=[]
    def rejected(name,code,fn):
        t=time.perf_counter()
        try:fn()
        except (GeometryFailure,ValueError) as error:
            record=error.record if isinstance(error,GeometryFailure) else dict(code=str(error))
            assert record['code']==code,(name,record)
            row=dict(name=name,expected_code=code,result=record,seconds=time.perf_counter()-t)
            events.append(row);save('TESTS.json',events);print(json.dumps(dict(test=name,code=code)),flush=True)
            return
        raise AssertionError('Expected rejection: '+name)
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
        g=owner.load_rhai(case['script'],case['metres_per_model_unit'])
        t=time.perf_counter();binding=bind_canonical(g);load_binding_seconds=time.perf_counter()-t
        a=binding.authority;c=binding.qualification
        packet=g.stored_topology();assert packet==json.loads((retained/'STORED_TOPOLOGY.json').read_text())
        assert all(struct.pack('d',v)==struct.pack('d',w) for v,w in zip(g.query(case['points_m']),case['direct_m']))
        old=json.loads((V4/'results/canonical_affine_consistency/final/CHECK.json').read_text())
        assert c.record['conversion']['source_to_materialized_affine_L1_bound_m']==old['conversion']['source_to_materialized_affine_L1_bound_m']
        assert max(b['normal_angle_upper_rad'] for b in c.record['branch_bounds'])==old['maximum_branch_normal_angle_bound_rad']
        with np.load(retained/'labelled_seed/surface/A/seed/SEED.npz') as z:
            x=z['points']*a.scale;f=z['faces'].copy();fr=z['refs'].copy()
        ce=np.array([(*edge,r) for r,edge in a.unit.curve_vertices.items()],int)
        cr=np.array([a.unit.corner_ref[name] for name in a.unit.point_names],int)
        arrays=dict(x=x,faces=f,face_refs=fr,curve_edges=ce,corner_refs=cr)
        def qualify(data=arrays,contract=c):return audit_directory(out,a,contract,source_arrays=data)
        t=time.perf_counter();valid=qualify();valid_seconds=time.perf_counter()-t
        save('VALID_SEED.json',valid);save('CONTRACT.json',c.record)
        assert valid['triangles_checked']==len(f)==792 and valid['finite_domain_failures']==0
        assert valid['acceptance']=='not_evaluated' and not valid['passed'] and not valid['geometry_requirements_passed']
        assert valid['residual_gate']['passed']
        print(json.dumps(dict(valid_triangles=792,unresolved=len(valid['unresolved']),seconds=valid_seconds)),flush=True)
        # Verify the file-backed post-adaptation entry also enforces the same
        # boundary. These are tiny diagnostic copies, not generated mesh outputs.
        with tempfile.TemporaryDirectory(prefix='v4-boundary-') as d:
            np.savez(Path(d)/'SURFACE.npz',coordinates=x,triangles=f)
            np.savez(Path(d)/'REFERENCES.npz',face_refs=fr,curve_edges=ce,corner_refs=cr)
            file_result=audit_directory(d,a,c)
            assert file_result==valid
        cap=next(r for r,n in a.unit.sheet_names.items() if '/cap/positive' in n)
        fi=int(np.flatnonzero(fr==cap)[0]);vertices=f[fi]
        bad=copy.deepcopy(arrays);bad['x'][vertices,0]+=2*a.scale
        rejected('translated_cap','FiniteSheetDomainFailure',lambda:qualify(bad))
        bad=copy.deepcopy(arrays);bad['face_refs'][fi]=next(r for r,n in a.unit.sheet_names.items() if '/cap/negative' in n)
        rejected('wrong_owning_sheet','SurfaceTopologyFailure',lambda:qualify(bad))
        centre=np.array([a.unit.brep.points[k] for k in a.unit.brep.loops[a.unit.sheet_names[cap]]]).mean(0)*a.scale
        bad=copy.deepcopy(arrays);q=centre.copy();q[0]=max(x[:,0])+a.scale
        dv=x[vertices[1]]-x[vertices[0]]
        bad['x'][vertices]=[q,2*centre-q+dv,centre-dv]
        partial_centroid=bad['x'][vertices].mean(0)
        loop=[rational(np.array(a.unit.brep.points[k])*a.scale) for k in a.unit.brep.loops[a.unit.sheet_names[cap]]]
        N=exact.cross(exact.sub(loop[1],loop[0]),exact.sub(loop[2],loop[0]))
        halfspaces=[(exact.cross(N,exact.sub(loop[(i+1)%len(loop)],p)),p) for i,p in enumerate(loop)]
        assert exact.clip_polygon([rational(partial_centroid)],halfspaces)
        rejected('partly_outside_centroid_inside','FiniteSheetDomainFailure',lambda:qualify(bad))
        # One ULP off a cap: finite domains remain valid, but a numerical angle
        # premise must not be inferred from small residuals or from the seed pass.
        nonparallel=copy.deepcopy(arrays);p=x[vertices].mean(0);p[1]=np.nextafter(p[1],np.inf)
        j=len(x);nonparallel['x']=np.vstack([x,p]);nonparallel['corner_refs']=np.append(cr,0)
        u,v,w=vertices
        nonparallel['faces']=np.vstack([np.delete(f,fi,axis=0),[u,v,j],[v,w,j],[w,u,j]])
        nonparallel['face_refs']=np.append(np.delete(fr,fi),[cap,cap,cap])
        unsupported=qualify(nonparallel)
        assert len(unsupported['unresolved'])==0 and not unsupported['geometry_requirements_passed']
        save('NONPARALLEL.json',dict(triangles_checked=unsupported['triangles_checked'],
            changed_vertex_m=p,unresolved=unsupported['unresolved'],residual_gate=unsupported['residual_gate'],
            finite_domain_failures=unsupported['finite_domain_failures'],acceptance=unsupported['acceptance']))
        bad=copy.deepcopy(arrays);bad['curve_edges'][[0,1],2]=bad['curve_edges'][[1,0],2]
        rejected('wrong_curve_incidence','SourceCurveIncidenceFailure',lambda:qualify(bad))
        bad=copy.deepcopy(arrays);bad['corner_refs'][[0,1]]=bad['corner_refs'][[1,0]]
        rejected('wrong_corner_identity','SourceCornerIdentityFailure',lambda:qualify(bad))
        bad=copy.deepcopy(arrays);bad['curve_edges']=bad['curve_edges'][1:]
        rejected('missing_curve_coverage','QualificationReferenceCoverageFailure',lambda:qualify(bad))
        bad=copy.deepcopy(arrays);keep=bad['face_refs']!=min(a.sheet_names)
        bad['faces']=bad['faces'][keep];bad['face_refs']=bad['face_refs'][keep]
        rejected('missing_sheet_coverage','QualificationReferenceCoverageFailure',lambda:qualify(bad))
        for field,value in [('geometry_identity','wrong'),('topology_identity','wrong'),('physical_unit_rational','1'),
                            ('kernel_source_sha256','wrong')]:
            bad=copy.deepcopy(c);bad.record[field]=value
            rejected('altered_'+field,'SourceBoundaryEvidenceMismatch',lambda:qualify(contract=bad))
        bad=copy.deepcopy(c);bad.record['conversion']['source_to_materialized_affine_L1_bound_m']['upper_binary64']=0.
        rejected('altered_conversion_bound','SourceBoundaryEvidenceMismatch',lambda:qualify(contract=bad))
        bad=copy.deepcopy(c);first=next(iter(bad.record['source_dependencies']));bad.record['source_dependencies'][first]='wrong'
        rejected('stale_source_dependency','SourceBoundaryEvidenceMismatch',lambda:qualify(contract=bad))
        rejected('missing_contract','SourceBoundaryContractRequired',lambda:qualify(contract=None))
        invalid=copy.deepcopy(packet);invalid['data']['sections'][1]['twist_bits']=1065353216
        rejected('unsupported_twist','UniformExtrusionPremiseFailed',lambda:validate_topology(invalid,g.identity))
        rejected('missing_accuracy','AccuracyRequirementsMissing',c.validate)
        rejected('invalid_accuracy','InvalidAccuracyRequirements',lambda:SourceBoundaryContract.automatic(a,{'position_m':float('nan'),'normal_rad':1.}))
        # Missing limits cannot be turned into a publishable success.
        with tempfile.TemporaryDirectory(prefix='v4-publish-reject-') as d:
            (Path(d)/'AUDIT.json').write_text(json.dumps(valid))
            rejected('seed_publication','CannotPublishUnqualifiedSurface',lambda:publish_surface(d,{},'unused',dict(passed=True)))
        admission=binding.inspect();assert not admission['executable']
        assert {r['operation'] for r in admission['issues']}=={'metric','sizing_policy','caller_accuracy_limits'}
        repeated=SourceBoundaryContract.automatic(a);assert repeated.record==c.record
        host_peak=psutil.Process(owner.transport.process.pid).memory_info().peak_wset
        summary=dict(classification='V4StoredPolygonSourceBoundaryContractConnectedSeedAcceptanceNotEvaluated',
            geometry_identity=g.identity,topology_identity=packet['topology_identity'],source=source,
            valid_seed=dict(vertices=len(x),faces=len(f),edges=len({tuple(sorted((int(p),int(q)))) for face in f for p,q in zip(face,np.roll(face,-1))}),finite_domain_failures=0,
                position_bound_max_m=valid['position_bound_max_m'],normal_bound_max_rad=valid['normal_bound_max_rad'],
                unresolved=valid['unresolved'],residual_gate=valid['residual_gate'],acceptance=valid['acceptance'],
                scalar_count=valid['scalar_diagnostics']['count'],scalar_max_m=valid['scalar_diagnostics']['maximum_abs_m']),
            file_backed_qualifier_equal=True,repeated_contract_equal=True,retained_conversion_bounds_equal=True,
            scalar_reference_bit_equal_count=len(case['points_m']),partly_outside_case=dict(centroid_m=partial_centroid,source_cap_centroid_m=centre,centroid_membership_exact=True),
            nonparallel_unresolved_faces=len(unsupported['unresolved']),
            admission_issues=[{k:v for k,v in i.items() if k!='curves'} for i in admission['issues']],
            seconds=dict(binding=load_binding_seconds,valid_seed=valid_seconds,total=time.perf_counter()-start),
            memory=dict(checker_peak_bytes=psutil.Process().memory_info().peak_wset,host_peak_bytes=host_peak),
            tests_expected_rejection=len(events),production_surface=False)
        save('SUMMARY.json',summary)
    sphere=next(c for c in direct['cases'] if c['name']=='sphere')
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
        other=bind_canonical(owner.load_rhai(sphere['script'],sphere['metres_per_model_unit']))
        rejected('unsupported_representation','UnsupportedSourceBoundaryRepresentation',lambda:SourceBoundaryContract.automatic(other.authority))
    summary['tests_expected_rejection']=len(events);save('SUMMARY.json',summary)
    print(json.dumps(summary,default=lambda v:v.tolist()),flush=True)


if __name__=='__main__':main()
