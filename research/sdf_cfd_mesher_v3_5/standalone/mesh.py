"""Single experimental V3.5 CLI: V4 surface -> V3 stack -> V4 outer stages."""
from pathlib import Path
from contextlib import ExitStack
import argparse,json,os,subprocess,sys,time,traceback,hashlib,stat
import numpy as np
ROOT=Path(__file__).resolve().parent
from planner import Planner,Request,jsonable
from plan_evidence import plan_and_capture
from geometry import write_case,surface,carrier_selection
from native import construct
from cli_controls import surface_configuration,surface_policy_configuration

def save(path,value):
    Path(path).write_text(json.dumps(value,indent=2,default=jsonable,allow_nan=False)+'\n')

def source_independence_guard():
    original='c:/users/jackson/desktop/02_projects/09b_implicit_cad_claude/'
    native_original='/mnt/c/users/jackson/desktop/02_projects/09b_implicit_cad_claude/'
    forbidden=[root+p for root in (original,native_original) for p in ('research/sdf_cfd_mesher_v3/','research/sdf_cfd_mesher_v4/','research/finite_cli_runs/','src/','shaders/')]
    def check(value):
        if not isinstance(value,(str,bytes,os.PathLike)):return
        path=os.fsdecode(value).replace('\\','/').lower()
        if any(path.startswith(prefix) for prefix in forbidden):raise PermissionError('OriginalSourceAccessForbidden:'+path)
    for module in tuple(sys.modules.values()):check(getattr(module,'__file__',None))
    def audit(event,args):
        if event=='open':check(args[0])
        if event=='subprocess.Popen':
            check(args[0])
            command=args[1] if isinstance(args[1],(list,tuple)) else (args[1],)
            for value in command:
                normalized=os.fsdecode(value).replace('\\','/').lower()
                if any(prefix in normalized for prefix in forbidden):raise PermissionError('OriginalSourceSubprocessForbidden')
    sys.addaudithook(audit)

def output_bytes(folder):
    total=0
    for path in folder.rglob('*'):
        try:entry=path.stat()
        except FileNotFoundError:continue  # Atomic checkpoint rename/removal.
        if stat.S_ISREG(entry.st_mode):total+=entry.st_size
    return total


def supervise(args):
    import psutil
    from audit_completion import SupervisorHandoff,REQUEST,requires_handoff
    handoff=None
    output_limit=getattr(args,'output_limit_bytes',None)
    if output_limit is None:output_limit=4_000_000_000
    if args.output.exists():raise ValueError('FreshOutputRequired')
    logs=ROOT/'run_logs';logs.mkdir(exist_ok=True)
    key=hashlib.sha256(str(args.output.resolve()).encode()).hexdigest()[:12]
    start=time.perf_counter();reason=None;peak=0;largest=0;monitor_error=None
    command=[sys.executable,'-B',str(Path(__file__).resolve()),*sys.argv[1:],'--worker']
    from source_custody import held
    with held(args.output.resolve(),logs/(key+'.SOURCES.json')) as sources,(logs/(key+'.stdout')).open('w') as stdout,(logs/(key+'.stderr')).open('w') as stderr:
        process=subprocess.Popen(command,stdout=stdout,stderr=stderr)
        last_disk=0
        def kill_owned():
            try:children=psutil.Process(process.pid).children(recursive=True)
            except psutil.Error:children=[]
            for child in reversed(children):
                try:child.kill()
                except psutil.Error:pass
            if process.poll() is None:process.kill()
        try:
            while process.poll() is None:
                if handoff is None and (args.output/REQUEST).exists():
                    handoff=SupervisorHandoff(args.output,process,verification=getattr(args,'verification','audit'))
                elapsed=time.perf_counter()-start
                if elapsed>args.timeout_seconds:
                    reason='TimeLimit';kill_owned();break
                try:
                    parent=psutil.Process(process.pid);tree=[parent]+parent.children(recursive=True)
                    memory=sum(p.memory_info().rss for p in tree if p.is_running());peak=max(peak,memory)
                    if memory>args.memory_limit_gib*2**30:reason='WindowsProcessTreeRSSLimit'
                    if elapsed-last_disk>5 and args.output.exists():
                        largest=output_bytes(args.output);last_disk=elapsed
                        if largest>output_limit:reason='OutputLimit'
                    if reason:
                        kill_owned()
                        break
                except psutil.Error:pass
                time.sleep(.25)
        except Exception as error:
            reason='SupervisorFailure'
            monitor_error=dict(type=type(error).__name__,details=str(error))
            kill_owned()
        code=process.wait()
    if sources['passed'] is not True:reason='SourceClosureChanged';code=1
    if code!=0 and reason is None:reason='AuditFinalizedWorkerFailed' if handoff is not None else 'WorkerFailed'
    if reason is None and code==0 and requires_handoff(args.output) and handoff is None:
        reason='AuditFinalizationHandoffMissing';code=1
    record=dict(command=command,exit_code=code,stop_reason=reason,monitor_error=monitor_error,source_identity=sources['identity'],source_custody_passed=sources['passed'],seconds=time.perf_counter()-start,peak_sampled_windows_tree_rss_bytes=peak,last_sampled_output_bytes=largest,output_limit_bytes=output_limit,timeout_seconds=args.timeout_seconds,memory_limit_gib=args.memory_limit_gib,linux_memory='RLIMIT_AS per Linux process, inherited by children; not aggregate RSS')
    # Persist closure evidence before the final atomic acceptance publication.
    save(logs/(key+'.json'),record)
    publication_failed=False
    if handoff is not None and reason is None and code==0:
        try:handoff.publish(sources,record,logs/(key+'.SOURCES.json'),logs/(key+'.json'))
        except Exception as error:
            reason='AuditFinalizationPublicationFailed';code=1
            record.update(exit_code=code,stop_reason=reason,monitor_error=dict(type=type(error).__name__,details=str(error)))
            publication_failed=True
    # Release artifact handles before the existing failure/revocation writer.
    if handoff is not None:handoff.close()
    if publication_failed:save(logs/(key+'.json'),record)
    print(json.dumps(record),flush=True)
    if code and args.output.exists() and not (args.output/'RESULT.json').exists():
        save(args.output/'RESULT.json',dict(passed=False,full_mesh_complete=False,failed_stage='configuration_or_startup',supervisor=record,stderr=str(logs/(key+'.stderr'))))
    if reason and args.output.exists():
        # Revoke acceptance before any fallible cleanup/import operation.
        result=args.output/'RESULT.json'
        if result.exists():result.rename(args.output/'RESULT_BEFORE_SUPERVISOR_STOP.json')
        record.update(full_mesh_complete=False,passed=False,cleanup_status='pending',cleanup=[])
        save(result,record);save(args.output/'SUPERVISOR_STOP.json',record)
        cleanup=[]
        try:
            from build_native import linux
            from standalone_background import stop_backend
            for scope in args.output.rglob('*.linux_scope.json'):
                try:
                    name=scope.name.removesuffix('.linux_scope.json')
                    command=['wsl','-d','Ubuntu-22.04','--','python3',linux(ROOT/'cancel_native.py'),linux(scope),linux(scope.parent),linux(ROOT/'native_bin'/name)]
                    outcome=subprocess.run(command,capture_output=True,text=True,timeout=15)
                    cleanup.append(dict(scope=str(scope),exit_code=outcome.returncode,stderr=outcome.stderr))
                except Exception as error:cleanup.append(dict(scope=str(scope),error=type(error).__name__,details=str(error)))
            try:stop_backend(args.output)
            except Exception as error:cleanup.append(dict(background_cleanup_error=type(error).__name__,details=str(error)))
        except Exception as error:cleanup.append(dict(cleanup_setup_error=type(error).__name__,details=str(error)))
        record.update(cleanup_status='completed_with_errors' if any('error' in row or 'background_cleanup_error' in row or 'cleanup_setup_error' in row or row.get('exit_code',0) for row in cleanup) else 'completed',cleanup=cleanup)
        save(result,record);save(args.output/'SUPERVISOR_STOP.json',record)
    return code if code else (1 if reason else 0)

def execute(args):
    import standalone_tool as cli
    from production_surface_authority import load_production_wall
    from live_spline_first_layer import SourceSheetFirstLayerProvider
    from production_prism_sizing import read_request
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from sdf_cfd_mesher_v4.common.authority import identity,array_hash
    from immutable_state_access import guard
    from v35_shell import prepare,enroll
    out=args.output.resolve()
    if out.exists():raise ValueError('FreshOutputRequired')
    out.mkdir(parents=True);started=time.perf_counter();stage='configuration';times={}
    request=Request(args.prism_first_height,args.prism_growth,args.prism_layers)
    case=out/'CASE.json'
    settings=write_case(args.case,args.geometry,case,request)
    settings['geometry']['metres_per_model_unit']=args.metres_per_model_unit
    if args.surface_size is not None:settings,_=surface_configuration(settings,args.surface_size)
    # Preserve explicit V4 size-led semantics and disclose resolved allowances.
    if args.surface_policy is not None:
        settings,_,policy=surface_policy_configuration(settings,args.surface_policy,args.surface_size,args.surface_normal_angle_deg,args.surface_position_error)
        save(out/'SURFACE_POLICY.json',policy)
    else:
        from surface_requirements195 import configure
        settings,accuracy=configure(settings,args.surface_normal_angle_deg,args.surface_position_error)
        if accuracy is not None:save(out/'SURFACE_ACCURACY.json',accuracy)
    if args.background is not None:settings['background']=json.loads(args.background.read_text())
    if getattr(args,'output_limit_bytes',None) is not None:settings['resources']['output_limit_bytes']=args.output_limit_bytes
    if args.stop_after in ('transition','complete') and 'background' not in settings:raise ValueError('ExplicitV4BackgroundPolicyRequired')
    direct=settings.get('background',{}).get('schema')=='V4DirectFillEngineeringPolicy/1'
    if direct and args.stop_after=='transition':raise ValueError('DirectFillHasNoSeparateTransitionTarget')
    from verification_policy import selection
    verification=selection(getattr(args,'verification','audit'),settings,args.stop_after,getattr(args,'verification_origin','explicit_api'))
    settings['verification']=verification
    save(case,settings)
    construction=carrier_selection(getattr(args,'surface_carrier',None),origin='explicit_cli')
    save(out/'SURFACE_CONSTRUCTION.json',construction)
    def mark(name,start):
        times[name]=time.perf_counter()-start
        save(out/'PROGRESS.json',dict(stage=name,times=times,seconds=time.perf_counter()-started,full_mesh_complete=False))
    try:
        stage='surface';t=time.perf_counter()
        with surface(case,out/'upstream',carrier_mode=getattr(args,'surface_carrier',None),carrier_origin='explicit_cli') as (wall,record),ExitStack() as lifetime:
            mark(stage,t)
            if args.stop_after=='surface':
                save(out/'RESULT.json',dict(passed=True,completed_stage='surface',full_mesh_complete=False,surface=record,surface_construction=construction,times=times));return 0
            stage='first_front';t=time.perf_counter()
            first=out/'first';first.mkdir()
            cli.first_request(first,wall,case)
            host,binding=cli.geometry(case);lifetime.callback(host.close)
            provider=SourceSheetFirstLayerProvider(first/'source',wall,first/'REQUEST.json',binding)
            with provider.source.prism_query_scope(),provider.source.verified_operation('trajectory'):
                provider._generate_trajectory()
            req=read_request(provider.request,provider.wall_authority)
            def owner_for(schedule):
                with provider.source.prism_query_scope(),provider.source.verified_operation('trajectory'):
                    return AcceptedTrajectoryOwner.from_inputs(provider.source,provider.x,provider.high,provider.tri,provider.refs['face_refs'],schedule,parent='V35CompleteHorizonBeforeConstruction',surface=provider.wall_authority['surface_identity'],request=req,case_identity=identity(settings))
            nominal_owner=owner_for(request.nominal);mark(stage,t)
            from source_handoff import live_handoff
            with live_handoff(provider.source,provider.h1,verification['mode']) as handoff:
                stage='v3_complete_plan';t=time.perf_counter()
                planner=Planner(request)
                plan=plan_and_capture(planner,nominal_owner.state,out,face_refs=provider.refs['face_refs'],
                    provenance=dict(case_sha256=hashlib.sha256(case.read_bytes()).hexdigest(),
                        source_identity=provider.wall_authority['surface_identity']))
                from collision_policy import construct as collision_construct,Policy,update_plan,segment_guard
                source_capabilities=dict(live_source=True,geometry=provider.source.geometry_identity,
                    assurance=provider.wall_authority['assurance'],clearance_bound=None,
                    field_is_exact_distance=False,segment_predicates='unchanged_source_qualify_segments')
                proposal,collision=collision_construct(planner,nominal_owner.state,plan['schedule'],out/'collision',
                    policy=Policy(args.collision_max_candidates,args.collision_max_seconds),
                    source_guard=segment_guard(provider.source,provider.h1,handoff=handoff),
                    source_capabilities=source_capabilities,source_handoff=handoff)
                if not collision['proposal_collision_passed']:raise ValueError('V35CollisionConstructionRefusal:'+collision['outcome'])
                plan=update_plan(planner,nominal_owner.state,plan,proposal,collision)
                if direct:
                    # Start from the existing complete collision-valid schedule,
                    # exactly as in retained proposal validation. No native cells
                    # or V4 admission occur before the final cap/global checks.
                    from cap_planning import select as select_cap,cap_report
                    cap_proposal,cap_plan=select_cap(planner,nominal_owner.state,proposal,out/'cap_plan')
                    if not cap_plan['numerical_proposal_passed']:raise ValueError('V35CapPlanningRefusal:'+cap_plan['outcome'])
                    if not np.array_equal(cap_proposal,proposal):
                        proposal,cap_collision=collision_construct(planner,nominal_owner.state,cap_proposal,out/'cap_collision',
                            policy=Policy(args.collision_max_candidates,args.collision_max_seconds),
                            source_guard=segment_guard(provider.source,provider.h1,handoff=handoff),source_capabilities=source_capabilities,source_handoff=handoff,
                            candidate_guard=lambda q:cap_report(nominal_owner.state,q)['passed'])
                        if not cap_collision['proposal_collision_passed']:raise ValueError('V35CapCollisionConstructionRefusal:'+cap_collision['outcome'])
                        plan=update_plan(planner,nominal_owner.state,plan,proposal,cap_collision)
                    if not cap_report(nominal_owner.state,proposal)['passed']:raise ValueError('V35CollisionCapContractRefusal')
                    plan['cap_planning']=cap_plan
                nominal_owner.state.save(out/'collision_plan',proposal)
                owner=owner_for(plan['schedule'])
                if owner.positions().tobytes()!=plan['positions'].tobytes():raise ValueError('V3V4PlannedArithmeticMismatch')
                owner.state.save(out/'plan',plan['schedule']);mark(stage,t)
                stage='v4_prospective_shell_checks';t=time.perf_counter()
                prepared=prepare(provider,owner,out/'shell',source_handoff=handoff,verification_mode=verification['mode']);mark(stage,t)
            plan['source_global_proof']='passed_before_native_construction'
            save(out/'PRISM_PLAN.json',{k:v for k,v in plan.items() if k not in ('positions','delta','schedule')})
            stage='v3_native_construction';t=time.perf_counter()
            native,native_report=construct(plan['positions'],provider.tri,out/'native',trace_files=args.trace_native_files)
            # Hold actual readback operands throughout enrollment and background.
            for p in sorted(Path(native['folder']).iterdir()):
                if p.is_file():lifetime.enter_context(guard(p))
            lifetime.enter_context(guard(out/'native/constant/polyMesh/boundary'))
            lifetime.enter_context(guard(out/'native/SEALED_PLAN.npz'))
            lifetime.enter_context(guard(out/'native/SEALED_PLAN.json'))
            mark(stage,t)
            stage='v4_shell_admission';t=time.perf_counter()
            run=enroll(prepared,native);mark(stage,t)
            from rolling_fast_engine import write_mesh
            write_mesh(run,out)
            if args.stop_after=='prisms':
                save(out/'RESULT.json',dict(passed=True,completed_stage='prisms',full_mesh_complete=False,requested=plan['requested'],achieved=plan['achieved'],native=native_report,shell=run.live_acceptance.mesh,times=times,seconds=time.perf_counter()-started));return 0
            stage='v4_transition_background';t=time.perf_counter()
            if direct:
                stage='v4_direct_fill'
                from direct_connection import execute as connect
                result=connect(run,case,out/'background',trace_files=args.trace_native_files,_finalization_scope=lifetime)
                mark('v4_direct_fill',t)
                root_result=dict(passed=True,completed_stage='complete',requested=plan['requested'],achieved=plan['achieved'],native=native_report,background=result,export=result['export'],times=times,seconds=time.perf_counter()-started,CFD_qualified=False)
                if verification['mode']=='audit':
                    if result.get('full_mesh_complete') is not True:raise ValueError('V4DirectCompleteRefusal')
                    root_result['full_mesh_complete']=True;descriptor=result['audit_finalization']
                else:
                    from verification_policy import production_result,ROOT_RESULT,profile
                    production_result(result);descriptor=result['artifact_finalization']
                    root_result.pop('passed')
                    root_result.update(schema=ROOT_RESULT,geometry_complete=True,producer_checks_passed=True,artifact_fidelity_passed=True,
                        independent_audit_status='not_run',acceptance_status='pending_supervisor',verification_policy='production',acceptance_profile=profile(),audit='not_run')
                save(out/'RESULT.json',root_result)
                from audit_completion import worker_handoff
                worker_handoff(out,descriptor,time.monotonic()+max(0,args.timeout_seconds-(time.perf_counter()-started)),lifetime)
                return 0
            import standalone_background as background
            expected=background.policy_backend(settings['background'])
            trace_token=background._trace_native_files.set(args.trace_native_files)
            try:result=background.execute(run,case,out/'background',expected,_transition_only=args.stop_after=='transition')
            finally:background._trace_native_files.reset(trace_token)
            mark(stage,t)
            if result.get('passed') is not True:raise ValueError('V4OuterStageRefusal')
            complete=False;export_result=None
            if args.stop_after=='complete':
                stage='openfoam_export';t=time.perf_counter()
                from export_mesh import export
                export_result=export(out/'background/COMPLETE_MESH.npz',out/'export',trace_files=args.trace_native_files);mark(stage,t);complete=True
            save(out/'RESULT.json',dict(passed=True,completed_stage=args.stop_after,full_mesh_complete=complete,requested=plan['requested'],achieved=plan['achieved'],native=native_report,background=result,export=export_result,times=times,seconds=time.perf_counter()-started,CFD_qualified=False))
            return 0
    except BaseException as failure:
        save(out/'RESULT.json',dict(passed=False,completed_stage=None,failed_stage=stage,full_mesh_complete=False,surface_construction=construction,error=str(failure),traceback=traceback.format_exc(),times=times,seconds=time.perf_counter()-started))
        raise

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',type=Path,required=True,help='Explicit V4 surface/seed/proof/resource settings')
    p.add_argument('--geometry',type=Path,required=True,help='Stock Rhai source file')
    p.add_argument('--metres-per-model-unit',type=float,required=True)
    p.add_argument('--prism-first-height',type=float,required=True)
    p.add_argument('--prism-growth',type=float,required=True)
    p.add_argument('--prism-layers',type=int,required=True)
    p.add_argument('--surface-size',type=float)
    p.add_argument('--surface-policy',choices=('size-led',))
    p.add_argument('--surface-carrier',choices=('adaptive','nodal'),help='Construction mode only; omitted retains the V3.5 adaptive default. Explicit nodal uses the existing V4 carrier-optional path; all final acceptance remains required.')
    p.add_argument('--surface-normal-angle-deg',type=float)
    p.add_argument('--surface-position-error',type=float)
    p.add_argument('--background',type=Path,help='Explicit V4 direct-fill policy; historical finite-row policies remain selectable')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stop-after',choices=('surface','prisms','transition','complete'),default='complete')
    p.add_argument('--verification',choices=('audit','production'),default='audit',help='Audit retains full cold replay. Production supports complete direct workflow only and records separate audit not_run.')
    p.add_argument('--timeout-seconds',type=float,default=3600,help='Whole worker resource bound, not a runtime target')
    p.add_argument('--memory-limit-gib',type=float,default=16,help='Sampled Windows tree RSS and Linux per-process virtual-address-space limit; not aggregate Linux RSS')
    p.add_argument('--output-limit-bytes',type=int,help='Explicit positive operational output ceiling for supervisor and CASE; omitted preserves the existing supervisor 4 GB and template resource policy')
    p.add_argument('--collision-max-candidates',type=int,default=96,help='Finite existing late-taper candidate budget; exhaustion refuses')
    p.add_argument('--collision-max-seconds',type=float,default=900,help='Collision search admission deadline between atomic checks; whole-worker timeout remains binding')
    p.add_argument('--trace-native-files',action='store_true',help='Trace native file access for standalone dependency verification; requires installed strace')
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args()
    args.verification_origin='explicit_cli' if any(arg=='--verification' or arg.startswith('--verification=') for arg in sys.argv[1:]) else 'v35_audit_default'
    from collision_policy import Policy
    Policy(args.collision_max_candidates,args.collision_max_seconds)
    if not np.isfinite([args.timeout_seconds,args.memory_limit_gib]).all() or args.timeout_seconds<=0 or args.memory_limit_gib<=0:p.error('Finite positive resource bounds required')
    if args.output_limit_bytes is not None and args.output_limit_bytes<=0:p.error('Positive output byte limit required')
    if not args.worker:return supervise(args)
    os.environ['V35_LINUX_AS_BYTES']=str(int(args.memory_limit_gib*2**30))
    source_independence_guard()
    return execute(args)

if __name__=='__main__':raise SystemExit(main())
