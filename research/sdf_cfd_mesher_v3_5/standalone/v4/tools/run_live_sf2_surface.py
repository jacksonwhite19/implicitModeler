"""One authorized live-spline run. Existing executors, fail-fast observation only."""
import argparse, collections, contextlib, dataclasses, json, math, os, re
from pathlib import Path
import shutil, subprocess, sys, time, traceback
from unittest.mock import patch
import numpy as np
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1]; ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from run_two_geometry_surface_integration import save,size,inventory
from historical_capsule_surface import OUT as SEALED,V3,sha,wsl,array_hash
from replay_surface_dispatch import verify_kernel_sources
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from surface_binding import bind_canonical
from sf2_surface_policy import bind_sf2_policy,load_case,CASE,ROOT as SF2
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from test_sf2_coupled_policy import retained_reader,exact_majorant
from shape_aware_metric import apply_shape_metric
import surface_execution as execution
import audit_adaptive_surface as audit

CASE_HASH='993e40e646b40c915f76500fce308159fb8d4cd8752aa3c8c6f448e197427f93'


def read(path):
    """Evidence is UTF-8 regardless of the Windows process code page."""
    return json.loads(Path(path).read_text(encoding='utf-8'))


def resource_plan():
    paths=[SF2/'SF2/A/wing/surface/A',SEALED/'fresh',V4/'results/wing_qualification_resume/run']
    rows=[dict(path=str(p.relative_to(ROOT)),bytes=size(p)) for p in paths]
    budget=2*max(r['bytes'] for r in rows)
    return dict(comparables=rows,output_limit_bytes=budget,minimum_free_bytes=budget,
        basis='Existing two-times measured comparable output reserve, including full qualification reports; one realization only.',
        native_stage_timeout_seconds=120,spline_chart_call_timeout_seconds=120,
        python_wall_limit='No existing overall Python deadline; monitor progress and storage.',
        memory='Measure worker and child peaks; no invented memory forecast or scientific limit.',monitoring_seconds=1)


def stop(code,stage,**evidence):
    raise GeometryFailure(code,stage,evidence)


def verify_written(binding,directory,vertices,faces,edges):
    a=binding.authority;c=binding.metric_connection;unpack=retained_reader(a)
    _,_,_,actual=unpack(directory/'metric.bin')
    old=SF2/'SF2/A/wing/surface/A/v3mmg1_A.bin'
    hv,hf,he,hm=unpack(old)
    curves=[set() for _ in vertices]
    for edge in edges:
        for i in edge.vertices:curves[i].add(edge.curve_ref)
    checks=collections.Counter();maximum_difference=0.
    for i,v in enumerate(vertices):
        m,row=c.evaluate(v.point,v.sheet_ref,curves=curves[i],trace=True)
        difference=float(np.max(abs(m[np.triu_indices(3)]-actual[i][np.triu_indices(3)])))
        maximum_difference=max(maximum_difference,difference)
        if difference!=0:stop('WrittenMetricMismatch','tensor_serialization',vertex=i,difference=difference)
        row['shape']=apply_shape_metric(row['prism'],row['principal'],c.shape)[0]
        for name in ('geometry','feature','geometry_feature','prism','shape'):
            term=row[name]
            if term is None:continue
            term=np.triu(term)+np.triu(term,1).T
            if not exact_majorant(actual[i],term):stop('MetricContributionLost','tensor_serialization',vertex=i,term=name)
            checks[name]+=1
        if not np.array_equal(m,row['shape']/(c.length_scale*c.length_scale)):
            stop('FamilyScalingMismatch','tensor_serialization',vertex=i)
    same_points=np.array_equal([v.point for v in vertices],[v.point for v in hv])
    same_faces=np.array_equal([f.vertices for f in faces],[f.vertices for f in hf])
    same_labels=([f.sheet_ref for f in faces]==[f.sheet_ref for f in hf] and
        [(e.vertices,e.curve_ref) for e in edges]==[(e.vertices,e.curve_ref) for e in he])
    record=dict(vertices=len(vertices),triangles=len(faces),feature_edges=len(edges),
        exact_directional_checks=dict(checks),maximum_written_difference=maximum_difference,
        family_scale_applied_once=True,metric_override=False,clipping=False,
        historical_points_equal=same_points,historical_faces_equal=same_faces,historical_labels_equal=same_labels,
        historical_tensors_equal=np.array_equal(actual,hm) if same_points and same_labels else None,
        written_sha256=sha(directory/'metric.bin'),historical_sha256=sha(old),
        complete_contract_bytes_equal=(directory/'metric.bin').read_bytes()==old.read_bytes(),
        eig_min=float(np.linalg.eigvalsh(actual).min()),eig_max=float(np.linalg.eigvalsh(actual).max()))
    save(directory.parent/'WRITTEN_METRIC_VERIFICATION.json',record)
    if same_points and same_faces and same_labels and not record['historical_tensors_equal']:
        stop('MatchedInputHistoricalMetricMismatch','tensor_serialization',record=record)
    return record


@contextlib.contextmanager
def observe_qualification(out,binding):
    """Stop on existing predicates; no replacement certificate or tolerance."""
    original_arrays=audit.audit_arrays;original_face=audit.certify_face
    original_feature=audit.feature_bound;original_native=audit.run_native
    rows=[];features=[];counts=collections.Counter();methods=collections.Counter()
    def finite_or_text(value):
        return value if value is None or math.isfinite(value) else str(value)
    def persist():
        save(out/'QUALIFICATION_PROGRESS.json',dict(face_counts=dict(counts),methods=dict(methods),
            faces_completed=len(rows),features_completed=len(features),
            maximum_position_bound=finite_or_text(max((r[0] for r in rows),default=None)),
            maximum_normal_bound=finite_or_text(max((r[1] for r in rows),default=None))))
        np.savez_compressed(out/'PARTIAL_CERTIFICATES.npz',face_bounds=np.array(rows),feature_bounds=np.array(features))
    def arrays(*args,**kwargs):
        result,data=original_arrays(*args,**kwargs);save(out/'ARRAY_AUDIT.json',result)
        if not result['topology']['passed']:stop('SurfaceTopologyFailed','array_audit',topology=result['topology'])
        if result['normal']['inverted_triangles']:
            x,f,a=args[:3];fr=kwargs['face_refs']
            bad=[i for i,(ref,p,n) in enumerate(zip(fr,data['centre'],data['normal'])) if n@a.exact_normal(ref,p)<=0]
            save(out/'INVERTED_FACES.json',dict(indices=bad,triangles=np.asarray(f)[bad],coordinates=np.asarray(x)[np.asarray(f)[bad]]))
            stop('SourceRelativeInversion','array_audit',count=len(bad),entities='INVERTED_FACES.json')
        if result['sdf_residual']['max']>binding.qualification.projection_tolerance_m:
            stop('OwningResidualGateFailed','array_audit',maximum=result['sdf_residual']['max'],limit=binding.qualification.projection_tolerance_m)
        return result,data
    def face(a,ref,triangle):
        value=original_face(a,ref,triangle);status,position,normal,method=value
        rows.append((position,normal));counts[status]+=1;methods[method]+=1
        if status!='Certified' or position>binding.qualification.position_limit_m or normal>binding.qualification.normal_limit_rad:
            safe=[status,finite_or_text(position),finite_or_text(normal),method]
            persist();save(out/'FIRST_FACE_FAILURE.json',dict(index=len(rows)-1,ref=ref,triangle=triangle,result=safe))
            stop('ContinuousFaceRequirementFailed','continuous_geometry',face=len(rows)-1,result=safe)
        if len(rows)%2000==0:persist()
        return value
    def feature(a,p,q,ref):
        value=original_feature(a,p,q,ref);features.append(value)
        if not math.isfinite(value) or value>binding.qualification.position_limit_m:
            persist();save(out/'FIRST_FEATURE_FAILURE.json',dict(index=len(features)-1,ref=ref,points=[p,q],bound=str(value)))
            stop('ContinuousFeatureRequirementFailed','continuous_features',edge=len(features)-1,bound=str(value))
        return value
    def native(command,stem,*args,**kwargs):
        value=original_native(command,stem,*args,**kwargs)
        if Path(stem).name=='intersections':
            r=json.loads(value[1]);save(out/'INTERSECTIONS.json',r)
            if r['forbidden'] or r['unresolved']:stop('SurfaceIntersectionRequirementFailed','intersections',result=r)
        return value
    with patch.object(audit,'audit_arrays',arrays),patch.object(audit,'certify_face',face),patch.object(audit,'feature_bound',feature),patch.object(audit,'run_native',native):
        try:yield
        finally:persist()


def worker(out):
    start=time.perf_counter();active='preflight'
    try:
        if sha(CASE)!=CASE_HASH:stop('CaseIdentityMismatch','preflight')
        case=load_case();plan=read(out/'RESOURCE_PLAN.json')
        if any(os.environ.get(n) is not None for n in ('V3_MMGS_NOMOVE','SDF_LENGTH_SCALE')):stop('ConflictingEnvironment','preflight')
        source,_=verify_kernel_sources()
        expected=read(V4/'results/sf2_coupled_policy/EVIDENCE.json')['identities']
        if source['source_sha256']!=expected['kernel']:stop('KernelSourceMismatch','preflight')
        dep=read(SF2/'MMGS_DEPENDENCY_QUALIFICATION.json');trace=read(SEALED/'IMPLEMENTATION_TRACE.json')
        from canonical_spline_authority import DLL,LOCKS
        chart=V3/'singular_sector_charts/sector_cdt';host=ROOT/'target/debug/canonical_kernel_host.exe'
        binaries={chart:read(V4/'results/canonical_polygon_connection/SEED_CONNECTION.json')['cgal_sha256'],
            V3/'adaptive_surface_architecture/bin/mmgs_metric_contract_runner':dep['adapter_sha256'],
            SF2/'dependency/install/lib/libmmgs.so.5.8.0':dep['library_sha256'],
            SEALED/'runtime/intersections':trace['native_intersections_sha256'],DLL:LOCKS[DLL]}
        if not dep['pass_gate']:stop('MMGSDependencyUnqualified','preflight')
        for p,h in binaries.items():
            if sha(p)!=h:stop('BinaryIdentityMismatch','preflight',path=str(p))
            if p==DLL:continue
            result=subprocess.run(['wsl','-e','env','LD_LIBRARY_PATH='+wsl(SF2/'dependency/install/lib'),'ldd',wsl(p)],capture_output=True,text=True,timeout=30)
            if result.returncode or 'not found' in result.stdout:stop('NativeDependencyUnavailable','preflight',path=str(p),result=result.stdout)
        binaries[host]=sha(host)
        with CanonicalKernel([str(host)],timeout=60) as owner:
            script=(V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8')
            g=owner.load_rhai(script,1.);b=bind_sf2_policy(bind_canonical(g));b.storage_budget_bytes=plan['output_limit_bytes']
            entry=ProductionAdaptiveSdfSurfaceMesher();admission=entry.admission(b)
            if not admission['executable']:stop('AdmissionFailed','preflight',issues=admission['issues'])
            if g.identity!=expected['canonical'] or b.authority.spline_packet['spline_identity']!=expected['live_spline']:stop('GeometryIdentityMismatch','preflight')
            save(out/'PREFLIGHT.json',dict(passed=True,source=source,binaries={str(p.relative_to(ROOT)):h for p,h in binaries.items()},
                admission=admission,case_sha256=CASE_HASH,script=script,units=1.,
                authorization='Current user authorizes one fresh surface and independent qualification using unchanged source-locked case; supersedes configuration-only execution scope, not its numeric policy.',
                policy=case['policy'],qualified_prism_request=False,replay=False,publication=False,
                runner_identity=sha(__file__),output=str(out/'fresh')))
            original=execution.write_surface_contract
            def writer(binding,directory,vertices,faces,edges):
                result=original(binding,directory,vertices,faces,edges)
                verify_written(binding,Path(directory),vertices,faces,edges)
                print('Actual written metric verification passed before MMGS',flush=True)
                return result
            active='production_surface'
            with patch.object(execution,'write_surface_contract',writer),observe_qualification(out,b):
                proof=entry.generate(b,out/'fresh')
            save(out/'RESULT.json',dict(classification='V4LiveSmoothSF2AdaptiveSurfaceQualifiedForBoundScope',passed=True,
                generation=proof,audit=read(out/'fresh/AUDIT.json'),seconds=time.perf_counter()-start,
                peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
                host_peak_working_set_bytes=psutil.Process(owner.transport.process.pid).memory_info().peak_wset,
                no_replay=True,no_publication=True,no_prisms=True))
        return 0
    except Exception as error:
        save(out/'STOP.json',dict(classification='V4LiveSmoothSF2SurfaceRunStopped',passed=False,active=active,
            error=str(error),structured=getattr(error,'record',None),traceback=traceback.format_exc(),
            seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
            no_retry=True,no_replay=True,no_publication=True,no_prisms=True))
        print(traceback.format_exc(),flush=True);return 1


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--worker',action='store_true');args=p.parse_args();out=args.output.resolve()
    if not out.is_relative_to((V4/'results/live_sf2_surface').resolve()):raise ValueError('TaskOwnedOutputRequired')
    if args.worker:return worker(out)
    if out.exists():raise ValueError('OutputMustBeNew; no retry')
    plan=resource_plan()
    if shutil.disk_usage(V4).free<plan['minimum_free_bytes']:raise ValueError('InsufficientOperationalReserve')
    out.mkdir(parents=True);save(out/'RESOURCE_PLAN.json',plan);samples=[];start=time.perf_counter();breach=None
    with (out/'execution.log').open('w',encoding='utf-8',newline='\n') as log:
        child=subprocess.Popen([sys.executable,'-B',__file__,'--worker','--output',str(out)],stdout=log,stderr=subprocess.STDOUT)
        while child.poll() is None:
            used=size(out);free=shutil.disk_usage(out).free
            try:peak=psutil.Process(child.pid).memory_info().peak_wset
            except psutil.NoSuchProcess:peak=None
            row=dict(seconds=time.perf_counter()-start,bytes=used,free_bytes=free,peak_working_set_bytes=peak);samples.append(row)
            if used>plan['output_limit_bytes'] or free<plan['minimum_free_bytes']:
                breach=row
                subprocess.run(['wsl','-e','pkill','-TERM','-f',re.escape(wsl(out))],timeout=15)
                proc=psutil.Process(child.pid)
                for descendant in proc.children(recursive=True):
                    try:descendant.terminate()
                    except psutil.NoSuchProcess:pass
                child.terminate();child.wait(timeout=15)
                save(out/'STOP.json',dict(error='OperationalStorageLimit',measurement=row,no_retry=True));break
            time.sleep(plan['monitoring_seconds'])
    save(out/'MONITOR.json',dict(exit_code=child.returncode,seconds=time.perf_counter()-start,resource_breach=breach,samples=samples))
    save(out/'MANIFEST.json',inventory(out));print(json.dumps(dict(exit_code=child.returncode,output=str(out),bytes=size(out))))
    return child.returncode


if __name__=='__main__':raise SystemExit('Historical case driver retired. Use research/sdf_cfd_mesher_v4/mesh.py.')
