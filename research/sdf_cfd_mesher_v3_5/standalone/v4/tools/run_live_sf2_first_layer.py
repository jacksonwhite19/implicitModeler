"""One bound first layer, existing predicates, no correction or future layers."""
import argparse,collections,json,math,shutil,subprocess,sys,time,traceback
from pathlib import Path
import numpy as np
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from standalone_case import read
from historical_capsule_surface import save,sha
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
from surface_binding import bind_canonical
from sf2_surface_policy import bind_sf2_policy
from replay_surface_dispatch import verify_kernel_sources
from live_spline_first_layer import SourceSheetFirstLayerProvider,SF2,CORNER,BRANCH
from production_surface_authority import load_production_wall
from production_prism_sizing import inspect_first_layer_inputs,read_request
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from production_first_layer_diagnostics import stats,triangle
from fractions import Fraction

OUT=V4/'results/live_sf2_first_layer'
SURFACE=V4/'results/live_sf2_surface/replay_20260929/authority'
REQUEST=OUT/'REQUEST.json'
SURFACE_ID='29ae61b5ca81cda49121badcedafd0b574f4fc95bbfecb9973f4f85f5052efb9'
STANDALONE_INPUTS=None

def size(path):
    return sum(p.stat().st_size for p in Path(path).rglob('*') if p.is_file())

def configure_inputs(output, surface, request, surface_identity):
    """Explicit fresh case paths; the existing loader and preflight own trust."""
    global OUT,SURFACE,REQUEST,SURFACE_ID
    output=Path(output).resolve();surface=Path(surface).resolve();request=Path(request).resolve()
    # Explicit inputs may live in a caller-owned run directory. No fallback
    # input is inferred there; the existing authority/request loader owns trust.
    if not request.is_relative_to(output):raise ValueError('FirstLayerRequestOutsideCase')
    if not isinstance(surface_identity,str) or len(surface_identity)!=64:raise ValueError('SurfaceIdentityRequired')
    OUT,SURFACE,REQUEST,SURFACE_ID=output,surface,request,surface_identity

def preflight():
    if STANDALONE_INPUTS is not None:
        from standalone_tool import first_preflight
        return first_preflight(STANDALONE_INPUTS,SURFACE,REQUEST,SURFACE_ID)
    x,f,s=load_production_wall(SURFACE)
    if s['surface_identity']!=SURFACE_ID:raise ValueError('SealedSurfaceMismatch')
    impl=read(SURFACE/'IMPLEMENTATION.json');replay=read(SURFACE/'REPLAY.json')
    if identity(impl)!=s['implementation_identity'] or identity(replay)!=s['determinism_identity']:raise ValueError('SurfaceSidecarMismatch')
    for name,h in impl['dependencies'].items():
        if sha(ROOT/name)!=h:raise ValueError('SourceDependencyMismatch:'+name)
    for name,h in impl['binaries'].items():
        if sha(ROOT/name)!=h:raise ValueError('BinaryDependencyMismatch:'+name)
    source,_=verify_kernel_sources()
    admission=inspect_first_layer_inputs(SURFACE,REQUEST);req=read_request(REQUEST,s)
    if len(req['increments'])!=1 or float(req['increments'][0]).hex()!='0x1.5d3b8a168f007p-13':raise ValueError('AuthorizedHeightMismatch')
    from sdf_cfd_mesher_v4.certificate.efficient import SOURCE,ROOT as CROOT
    binary=CROOT/'runtime/bin'/('proof_'+sha(SOURCE)+'.dll')
    if not binary.exists():raise ValueError('ExistingCertificateBinaryRequired')
    dependency_paths=[CORNER,BRANCH,SF2/'first_row.py',Path(__file__),V4/'tools/live_spline_first_layer.py']
    for n in ['production_prism_sizing.py','production_first_layer_run.py','run_geometry_family.py','generalize_local_remesh.py','local_termination_topology.py','first_layer_metric_state.py','final_shell_incidence.py','cavity_volume_audit.py','native_mixed_face_certificate.py','reuse_mixed_certificate.py']:
        dependency_paths.append(V4/'tools'/n)
    comparable=V4/'results/production_adaptive_capsule_first_layer/A'
    measured=size(comparable);ratio=len(f)/33786
    # Operational allowance only: two candidate/replay realizations, the known
    # case-count ratio, and the existing two-times measured-output reserve.
    budget=math.ceil(2*2*measured*ratio)
    return dict(passed=True,source=source,surface=s,admission=admission,request_identity=identity(req['declaration']),
        request_sha256=sha(REQUEST),height_hex=float(req['increments'][0]).hex(),
        dependencies={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in dependency_paths},
        certificate_binary={str(binary.relative_to(ROOT)).replace('\\','/'):sha(binary)},
        resources=dict(comparable=str(comparable.relative_to(ROOT)),comparable_bytes=measured,prism_count_ratio=ratio,
            output_limit_bytes=budget,minimum_free_bytes=budget,basis='Two realizations times measured per-run capsule bytes times cell-count ratio times existing 2x reserve; operational allowance, not a forecast or scientific threshold.',
            python_deadline=None,memory_limit=None,monitor_seconds=1),vertices=len(x),prisms=len(f))

def diagnostics(p):
    q=p.check['quality'];refs=p.refs['face_refs'];rows={};regional={};failures=collections.Counter(v['kind'] for v in q['failures'])
    for key,source,gate,upper in [('volume','cells',0,False),('determinant','cells',.0011,False),('aspect','cells',1000,True),('weight','faces',.055,False),('skewness','faces',3.63636,True),('symmetric_volume_ratio','faces',.01,False)]:
        records=q[source];values=np.array([r[key] for r in records]);bad=values>gate if upper else (values<=gate if key=='volume' else values<gate)
        idx=int(values.argmax() if upper else values.argmin())
        rows[key]=dict(distribution=stats(values),gate=gate,violations=int(bad.sum()),worst=records[idx])
        for ref,name in p.source.sheet_names.items():
            mask=np.array([refs[r['column']]==ref if source=='cells' else any(refs[v%1000000]==ref for v in r['owners']) for r in records])
            regional.setdefault(name,{})[key]=dict(distribution=stats(values[mask]),violations=int(bad[mask].sum()),count=int(mask.sum()))
    jac=np.array([float(Fraction(r['jacobian'])) for r in q['exact']]);rows['exact_jacobian']=dict(distribution=stats(jac),violations=sum(r['jacobian_sign']<=0 for r in q['exact']),worst=q['exact'][int(jac.argmin())])
    features=set(map(int,p.refs['curve_edges'][:,:2].ravel()));corners=set(map(int,np.flatnonzero(p.refs['corner_refs'])))
    feature_regions={}
    for name,ids in [('incident_feature_columns',features),('incident_corner_columns',corners)]:
        mask=np.array([any(int(v) in ids for v in t) for t in p.tri])
        feature_regions[name]=({k:stats(np.array([r[k] for r in q['cells']])[mask]) for k in ('volume','determinant','aspect')}
            if mask.any() else dict(scope='No source entities of this kind; no regional samples'))
        feature_regions[name]['columns']=int(mask.sum())
    for ref,name in p.source.sheet_names.items():
        regional[name]['exact_jacobian']=dict(distribution=stats(jac[refs==ref]),violations=sum(r['jacobian_sign']<=0 for r,yes in zip(q['exact'],refs==ref) if yes))
    top=triangle(p.high,p.tri);base=triangle(p.x,p.tri)
    area_dot=np.einsum('ij,ij->i',np.cross(p.x[p.tri[:,1]]-p.x[p.tri[:,0]],p.x[p.tri[:,2]]-p.x[p.tri[:,0]]),np.cross(p.high[p.tri[:,1]]-p.high[p.tri[:,0]],p.high[p.tri[:,2]]-p.high[p.tri[:,0]]))
    return dict(hard_gates=rows,regions=regional,feature_regions=feature_regions,failures_by_kind=dict(failures),
        front_triangles={k:stats(v) for k,v in top.items()},wall_triangles={k:stats(v) for k,v in base.items()},front_reversed=int(sum(area_dot<=0)),
        normal_height=p.assigned,quality_passed=q['passed'],ownership=p.check['ownership'],vertex_links=p.check['vertex_links'],
        sigma_G='Not measured; no gate',region_face_scope='Pair metrics use actual lateral neighbours; each region includes pairs incident to that region. Counts may overlap.')

def worker(label,commit=False,*,commit_action=None,geometry_session=None,_rolling_schedule=None,_rolling_completion=None,_rolling_completed=False,_rolling_transition=None,_rolling_diagnostics=False):
    if STANDALONE_INPUTS is not None:
        return standalone_worker(label,commit,commit_action=commit_action,geometry_session=geometry_session,_rolling_schedule=_rolling_schedule,_rolling_completion=_rolling_completion,_rolling_completed=_rolling_completed,_rolling_transition=_rolling_transition,_rolling_diagnostics=_rolling_diagnostics)
    if _rolling_diagnostics:raise ValueError('StandaloneUpstreamDiagnosticsRequired')
    if _rolling_transition is not None:raise ValueError('StandaloneProspectiveTransitionRequired')
    if geometry_session is not None:raise ValueError('StandaloneGeometrySessionRequired')
    start=time.perf_counter();pre=preflight();save(OUT/'PREFLIGHT.json',pre)
    if shutil.disk_usage(OUT).free<pre['resources']['minimum_free_bytes']:raise ValueError('OperationalFreeReserve')
    if STANDALONE_INPUTS is None:
        host=ROOT/'target/debug/canonical_kernel_host.exe'
    else:
        from owned_runtime import BIN
        host=BIN/'canonical_kernel_host.exe'
    with CanonicalKernel([str(host)],timeout=60) as owner:
        if STANDALONE_INPUTS is None:
            obj=owner.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
            b=bind_sf2_policy(bind_canonical(obj))
        else:
            from standalone_case import load,bind
            case,_,_=load(STANDALONE_INPUTS)
            obj=owner.load_rhai(case['geometry']['rhai'],case['geometry']['metres_per_model_unit'])
            b=bind(bind_canonical(obj),STANDALONE_INPUTS)
        p=SourceSheetFirstLayerProvider(OUT/label,SURFACE,REQUEST,b)
        save(p.folder/'INPUT.json',dict(surface=pre['surface']['surface_identity'],geometry=obj.identity,request=pre['request_identity'],height=p.h1,source=pre['source']))
        passed=p.generate_candidate()
        # Reporting never changes candidate geometry or qualified predicates.
        diagnostic=diagnostics(p);save(p.folder/'DIAGNOSTICS.json',diagnostic)
        p.verify_wall()
        result=dict(passed=passed,wall_preserved=True,prisms=len(p.tri),wall_vertices=len(p.x),front_vertices=len(p.high),
            front_coordinate_hash=array_hash(p.high),front_topology_hash=array_hash(p.tri),
            prism_connectivity_hash=array_hash(np.array([c['vertices'] for c in p.cells],dtype=np.int64)),
            ownership_hash=identity([(c['id'],c['column']) for c in p.cells]),quality_hash=sha(p.folder/'QUALITY.json'),
            certificate_hash=p.check.get('certificate',{}).get('proof_hash'),times=p.times,seconds=time.perf_counter()-start,
            peak_working_set_bytes=psutil.Process().memory_info().peak_wset,committed=None)
        save(p.folder/'RESULT.json',result)
        if not passed:raise ValueError('RequiredFirstLayerQualificationFailed')
        if commit:
            if commit_action is None:
                state=p.phase('commit_checkpoint',p.commit_candidate);result['committed']=scientific_hash(state)
            else:
                result['committed']=p.phase('commit_checkpoint',lambda:commit_action(p,result,pre))
            result['seconds']=time.perf_counter()-start;result['peak_working_set_bytes']=psutil.Process().memory_info().peak_wset
            save(p.folder/'RESULT.json',result)
        return result

def standalone_worker(label,commit=False,*,commit_action=None,geometry_session=None,_rolling_schedule=None,_rolling_completion=None,_rolling_completed=False,_rolling_transition=None,_rolling_diagnostics=False):
    """Same stage owners; geometry capabilities select only their input provider."""
    from standalone_tool import geometry
    from standalone_geometry import is_scalar
    from production_first_layer_run import ProductionFirstLayerProvider
    start=time.perf_counter();pre=preflight();save(OUT/'PREFLIGHT.json',pre)
    if shutil.disk_usage(OUT).free<pre['resources']['minimum_free_bytes']:raise ValueError('OperationalFreeReserve')
    host,b=geometry(STANDALONE_INPUTS) if geometry_session is None else geometry_session
    try:
        if _rolling_transition is not None and is_scalar(b):raise ValueError('RollingGeneralSourceTetTransitionRequired')
        if _rolling_diagnostics and is_scalar(b):raise ValueError('SourceSheetUpstreamDiagnosticsRequired')
        p=(ProductionFirstLayerProvider(OUT/label,surface=SURFACE,request=REQUEST,binding=b)
           if is_scalar(b) else SourceSheetFirstLayerProvider(OUT/label,SURFACE,REQUEST,b,_rolling_schedule=_rolling_schedule,_rolling_completion=_rolling_completion,_rolling_completed=_rolling_completed,_rolling_transition=_rolling_transition,_rolling_diagnostics=_rolling_diagnostics))
        save(p.folder/'INPUT.json',dict(surface=pre['surface']['surface_identity'],geometry=b.geometry_identity,
            request=pre['request_identity'],height=p.h1,source=pre['source']))
        passed=p.generate_candidate();save(p.folder/'DIAGNOSTICS.json',diagnostics(p));p.verify_wall()
        result=dict(passed=passed,wall_preserved=True,prisms=len(p.tri),wall_vertices=len(p.x),front_vertices=len(p.high),
            front_coordinate_hash=array_hash(p.high),front_topology_hash=array_hash(p.tri),
            prism_connectivity_hash=array_hash(np.array([c['vertices'] for c in p.cells],dtype=np.int64)),
            ownership_hash=identity([(c['id'],c['column']) for c in p.cells]),quality_hash=sha(p.folder/'QUALITY.json'),
            certificate_hash=p.check.get('certificate',{}).get('proof_hash'),times=p.times,
            seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset,committed=None)
        save(p.folder/'RESULT.json',result)
        if not passed:raise ValueError('RequiredFirstLayerQualificationFailed')
        if commit:
            if commit_action is None:raise ValueError('StandaloneProductionCommitOwnerRequired')
            result['committed']=p.phase('commit_checkpoint',lambda:commit_action(p,result,pre))
            result['seconds']=time.perf_counter()-start
            result['peak_working_set_bytes']=psutil.Process().memory_info().peak_wset
            save(p.folder/'RESULT.json',result)
        return result
    finally:
        if geometry_session is None:host.close()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--worker',choices=['A','B']);parser.add_argument('--commit',action='store_true');parser.add_argument('--preflight',action='store_true')
    parser.add_argument('--output',type=Path);parser.add_argument('--surface',type=Path);parser.add_argument('--request',type=Path);parser.add_argument('--surface-identity')
    args=parser.parse_args()
    if any(v is not None for v in (args.output,args.surface,args.request,args.surface_identity)):
        if any(v is None for v in (args.output,args.surface,args.request,args.surface_identity)):parser.error('Complete first-layer case inputs required')
        configure_inputs(args.output,args.surface,args.request,args.surface_identity)
    if args.preflight:
        pre=preflight();save(OUT/'PREFLIGHT.json',pre);print(json.dumps(pre['admission'],indent=2));return 0
    if args.worker:
        try:worker(args.worker,args.commit);return 0
        except Exception as e:
            save(OUT/args.worker/'STOP.json',dict(error_type=type(e).__name__,error=str(e),traceback=traceback.format_exc(),no_retry=True));raise
    raise ValueError('ExplicitWorkerRequired')

if __name__=='__main__':raise SystemExit('Historical case driver retired. Use research/sdf_cfd_mesher_v4/mesh.py.')
