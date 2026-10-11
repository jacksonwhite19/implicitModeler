"""Initial-stage case plumbing for the shared current-front range tool.

Existing surface, first-layer and enrollment owners execute their own gates.
There is no prism loop here: continuation uses the authoritative warm executor.
The retained SF2 coordinates are comparison fixtures, never generation inputs.
"""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import json, subprocess, sys, time, traceback
import numpy as np
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity,array_hash

def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))

def verify_code(held,directory):
    """Same guarded reviewed-source association as the existing range tool."""
    import immutable_state_access as access
    directory=Path(directory).resolve()
    for name in ('CODE_CONNECTION.json','REVIEW_REQUEST_3.json','REVIEW.json','check.py'):
        held.enter_context(access.guard(directory/name))
    record=read(directory/'CODE_CONNECTION.json');c=record['content']
    q=read(directory/'REVIEW_REQUEST_3.json');r=read(directory/'REVIEW.json')
    if (c.get('schema')!='V4SharedPrismToolCodeReview/1' or identity(c)!=record['identity'] or r.get('passed') is not True or not r.get('reviewer')
        or c['request_sha256']!=file_hash(directory/'REVIEW_REQUEST_3.json')
        or r['request_sha256']!=c['request_sha256']
        or c['review_sha256']!=file_hash(directory/'REVIEW.json')
        or c['checker_sha256']!=file_hash(directory/'check.py')
        or any(c[k]!=v for k,v in q.items())):raise ValueError('IndependentBootstrapReviewRequired')
    for p,h in {**c['sources'],**c['binaries'],**c['inputs']}.items():
        held.enter_context(access.guard(p))
        if file_hash(p)!=h:raise ValueError('ChangedReviewedBootstrapDependency',p)
    # An arbitrary approved record cannot omit the executed bootstrap wrapper.
    if c['sources'].get(str(Path(__file__).resolve()))!=file_hash(__file__):raise ValueError('BootstrapOutsideReviewedClosure')
    return record['identity']

def validate_range(case_path,through_layer,authorization,profile):
    from run_live_sf2_surface import CASE,CASE_HASH
    from production_prism_stack import ExecutionProfile
    if Path(case_path).resolve()!=CASE.resolve() or file_hash(case_path)!=CASE_HASH:raise ValueError('UnsupportedFreshCasePolicy')
    if type(through_layer) is not int or not 1<=through_layer<=20:raise ValueError('InvalidRequestedLayerRange')
    if not isinstance(authorization,str) or not authorization.strip():raise ValueError('ExplicitRangeRequestProvenanceRequired')
    ExecutionProfile(profile)
    return dict(from_layer=0,through_layer=through_layer,authorization=authorization,profile=profile,
                case_path=str(Path(case_path).resolve()),case_sha256=CASE_HASH)

def first_request(out,surface,grant):
    """Existing request schema; bind full normal thickness to the NEW authority."""
    from production_prism_sizing import read_request
    from production_surface_authority import load_production_wall
    _,_,s=load_production_wall(surface)
    template=V4/'results/live_sf2_first_layer/REQUEST.json'
    # Validate the old approval and every original source-document hash BEFORE
    # rebinding paths/authority. Do not legitimize changed provenance by hashing it.
    old_seal=read(template.parent/'AUTHORITY.json')
    if file_hash(template)!=old_seal['request_sha256']:raise ValueError('ChangedApprovedFirstRequest')
    original=read_request(template,dict(surface_identity=old_seal['scientific']['surface'],geometry_identity=old_seal['geometry']))
    req=deepcopy(original['declaration']);req['surface_identity']=s['surface_identity']
    if req['geometry_identity']!=s['geometry_identity']:raise ValueError('FreshRequestGeometryMismatch')
    evidence=out/'AUTHORIZATION.json'
    durable_json(evidence,dict(schema='V4FreshRangeInitializationAuthorization/1',request=grant,
        scope='Fresh surface and exactly one initial full-normal layer; continuation is separately bound from this same user range.',
        first_full_normal_height_hex=req['increments_m_hex'][0],template=str(template),template_sha256=file_hash(template)))
    sources=[evidence]
    # Preserve the authorized numeric package's scientific provenance, not old
    # request-directory-relative path spelling or an old execution authorization.
    sources.extend((template.parent/r['path']).resolve() for r in req['source_documents'][1:])
    req['source_documents']=[dict(path=str(p.resolve()),sha256=file_hash(p)) for p in sources]
    durable_json(out/'REQUEST.json',req)
    return read_request(out/'REQUEST.json',s)

def continuation_record(case,first_path):
    """Rebind recovered provenance only after exact fresh source comparisons.

    AcceptedTrajectoryOwner still verifies every source, wall/X1 mapping and the
    live source-normal producer. No historical positions become generation data.
    """
    from sealed_front_connection import load_case
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY
    wall,tri,surface,req=load_case(case)
    state,selected=existing_state_class().load(STUDY/'planner/selected_state')
    mapping=exact_vertex_map(state.wall,wall)
    with np.load(first_path,allow_pickle=False) as z:
        first=z['front'];refs=z['face_refs']
    if first.tobytes()!=state.first[mapping].tobytes():raise ValueError('FreshFirstFrontSourceMismatch')
    old_path=V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json'
    record=deepcopy(read(old_path))
    if array_hash(mapping)!=record['current_to_historical_vertex_map']:raise ValueError('FreshSourceMappingMismatch')
    record.update(surface=surface['surface_identity'],committed_parent=case['committed'],
        original_request_file_sha256=req['request_file_sha256'],original_request_identity=identity(req['declaration']),
        original_schedule_identity=req['schedule_identity'],fresh_origin=dict(recovered_record=str(old_path),
            recovered_record_sha256=file_hash(old_path),case=identity(case),candidate_sha256=file_hash(first_path)))
    # Keep the original recovered record and its source evidence immutable.
    extra=[Path(case['request']),Path(case['front_qualification']),Path(first_path)]
    record['sources'] += [dict(path=p.relative_to(ROOT).as_posix(),sha256=file_hash(p)) for p in extra]
    return record,wall,tri,first,refs,mapping,selected

def initial_authorization(owner,through_layer,grant):
    from accepted_production_contract import validate_authorization
    rows=[]
    for layer in range(2,through_layer+1):
        row=owner.schedule.row_record(layer)
        rows.append(dict(layer=layer,quantity='path_length_m',binary64_hex=row['uniform_hex'],
                         value_m=float.fromhex(row['uniform_hex'])) if row['scope']=='global' else row)
    auth=dict(schema='V4WingContinuationUserAuthorization/1',user_authorized=True,
        diagnostic_contract=identity(owner.record),initial_committed_parent=owner.parent,
        original_request_sha256=owner.record['original_request_file_sha256'],original_request_identity=owner.record['original_request_identity'],
        trajectory_owner=owner.source_identity,recovered_schedule=owner.schedule.sha256,
        geometry=owner.geometry_identity,surface=owner.record['surface'],units=owner.units,
        first_full_normal_height_m=owner.state.first_height,authorized_layers=list(range(2,through_layer+1)),
        increments=rows,authorization=grant['authorization'],range_request=grant,
        production_admission_passed=False)
    validate_authorization(auth,identity(auth),owner)
    return auth

def bind_initial_front(out,first_out,surface,grant):
    """Enroll once. Later layers reuse persistent indexed access incrementally."""
    from sealed_front_connection import enroll_native,load_case,inspect_front,build_access_indices
    from composable_state import CommittedState
    from production_prism_stack import session
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from accepted_production_contract import bind_production
    seal=read(first_out/'AUTHORITY.json');result=read(first_out/'A/RESULT.json')
    production_initial=(first_out/'INITIAL_ENROLLMENT.json').exists()
    if production_initial:
        enrolled=read(first_out/'INITIAL_ENROLLMENT.json');case=enrolled['case']
        if (grant['profile']!='production' or seal.get('extended_replay_performed') is not False
            or case['committed']!=result['committed'] or case['committed']!=seal['committed_front_identity']
            or Path(case['request']).resolve()!=(first_out/'REQUEST.json').resolve()
            or Path(case['surface']).resolve()!=surface.resolve()
            or file_hash(first_out/'PRODUCTION_CHECKS.json')!=seal['production_checks_sha256']):
            raise ValueError('ProductionInitialEnrollmentMismatch')
        load_case(case)
        durable_json(out/'CASE.json',case)
        reference=enrolled['reference']
    else:
        case,reference=legacy_initial_enrollment(out,first_out,surface,seal,result)
    record,wall,tri,first,refs,mapping,selected=continuation_record(case,first_out/'A/CANDIDATE.npz')
    durable_json(out/'DIAGNOSTIC_REQUEST.json',record)
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
        obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
        authority=bind_sf2_policy(bind_canonical(obj)).authority
        owner=AcceptedTrajectoryOwner(authority,record,identity(record),wall,first,tri,refs,mapping,selected,parent=case['committed'])
        auth=initial_authorization(owner,grant['through_layer'],grant)
        durable_json(out/'AUTHORIZATION.json',auth)
        with session(grant['profile']),CommittedState(reference) as base:
            if base.scientific!=case['committed']:raise ValueError('InitialEnrolledStateMismatch')
            adapter,inspected=bind_production(base,case,owner,auth,identity(auth))
            target=adapter.schedule.targets[1]
            prediction=adapter.geometry_binding.predict(adapter.front.coordinates,target)
            evidence=adapter.geometry_binding.validate_target(prediction.coordinates,target)
            if evidence.get('admitted') is not True:raise ValueError('FreshTrajectoryPremiseRejected')
            indices=build_access_indices(base,inspected,out/'indices')
            durable_json(out/'INDICES.json',indices)
            durable_json(out/'PREFLIGHT.json',dict(passed=True,scientific=base.scientific,
                reference=reference,owner=owner.source_identity,schedule=owner.schedule.sha256,trajectory=evidence))
    parent=dict(passed=True,layer=1,scientific=case['committed'],reference=reference)
    return dict(schema='V4AuthorizedContinuationExecution/1',layers=auth['authorized_layers'],resume=False,
        parent=parent,authorization_record=auth,authorization=identity(auth),execution_profile=grant['profile'],
        compare_inside_publisher=grant['profile']=='qualification',
        fresh_inputs={n:dict(path=str(p.resolve()),sha256=file_hash(p)) for n,p in
            [('case',out/'CASE.json'),('diagnostic',out/'DIAGNOSTIC_REQUEST.json'),
             ('candidate',first_out/'A/CANDIDATE.npz'),('indices',out/'INDICES.json')]})

def legacy_initial_enrollment(out,first_out,surface,seal,result):
    from sealed_front_connection import enroll_native
    files=[first_out/n for n in ('REQUEST.json','AUTHORITY.json','IMPLEMENTATION.json','REPLAY.json','A/RESULT.json','A/COVERAGE_RECHECK.json')]
    files += [(first_out/row['path']).resolve() for row in read(first_out/'REQUEST.json')['source_documents']]
    files += [surface/n for n in ('SURFACE_AUTHORITY.json','SURFACE.npz','REFERENCES.npz','IMPLEMENTATION.json','REPLAY.json')]
    case=dict(units='m',surface=str(surface),surface_identity=seal['scientific']['surface'],geometry_identity=seal['geometry'],
        request=str(first_out/'REQUEST.json'),request_sha256=file_hash(first_out/'REQUEST.json'),
        checkpoint=str(first_out/'A/checkpoints'),committed=result['committed'],layer=1,
        front_coordinates=result['front_coordinate_hash'],front_qualification=str(first_out/'AUTHORITY.json'),
        sources={str(p.resolve()):file_hash(p) for p in files})
    durable_json(out/'CASE.json',case)
    reference=enroll_native(case,out/'enrollment')
    return case,reference

def ingest_initial(held,surface,first_out,case_path):
    """Ingest an existing sealed wall/X1. No regeneration or fabricated pass."""
    import immutable_state_access as access
    from production_surface_authority import load_production_wall
    from production_prism_sizing import read_request
    from verify_live_sf2_first_layer import FIELDS
    surface=Path(surface).resolve();first_out=Path(first_out).resolve()
    if not all(p.is_relative_to((V4/'results').resolve()) for p in (surface,first_out)):
        raise ValueError('IngestOutsideV4Results')
    paths=[surface/n for n in ('SURFACE_AUTHORITY.json','IMPLEMENTATION.json','REPLAY.json','CASE.json','REFERENCES.npz')]
    paths += [first_out/n for n in ('AUTHORITY.json','IMPLEMENTATION.json','REPLAY.json','REQUEST.json',
                                  'A/RESULT.json','A/COVERAGE_RECHECK.json','A/CANDIDATE.npz','PREFLIGHT_A.json')]
    for p in paths:held.enter_context(access.guard(p))
    if file_hash(surface/'CASE.json')!=file_hash(case_path):raise ValueError('IngestCaseMismatch')
    _,_,wall=load_production_wall(surface)
    for name in wall['files']:held.enter_context(access.guard(surface/name))
    # Recheck under the complete wall payload guards.
    _,_,wall=load_production_wall(surface)
    seal=read(first_out/'AUTHORITY.json');science=seal['scientific']
    if (seal.get('first_layer_qualified') is not True or identity(science)!=seal['first_layer_identity']
        or science['committed']!=seal['committed_front_identity'] or science['surface']!=wall['surface_identity']
        or seal['geometry']!=wall['geometry_identity']):raise ValueError('IngestFirstLayerAuthorityMismatch')
    for name,key in [('IMPLEMENTATION.json','implementation_sha256'),('REPLAY.json','replay_sha256'),('REQUEST.json','request_sha256')]:
        if file_hash(first_out/name)!=seal[key]:raise ValueError('IngestFirstLayerEvidenceMismatch:'+name)
    if file_hash(surface/'IMPLEMENTATION.json')!=seal['surface_implementation_sha256']:
        raise ValueError('IngestSurfaceImplementationMismatch')
    request=read_request(first_out/'REQUEST.json',wall);hold_request(held,first_out,request)
    if (len(request['increments'])!=1 or identity(request['declaration'])!=science['request']):
        raise ValueError('IngestFirstRequestMismatch')
    result=read(first_out/'A/RESULT.json');replay=read(first_out/'REPLAY.json')
    if (result.get('passed') is not True or replay.get('exact') is not True
        or any(result[k]!=science[k] or replay['fields'][k]!=science[k] for k in FIELDS)
        or file_hash(first_out/'A/CANDIDATE.npz')!=replay['files']['CANDIDATE.npz']):
        raise ValueError('IngestFirstLayerReplayMismatch')
    coverage=read(first_out/'A/COVERAGE_RECHECK.json')
    if any(coverage[k]!=0 for k in ('forbidden','unresolved','missing','duplicate')):
        raise ValueError('IngestFirstLayerCoverageFailure')
    return dict(surface=str(surface),first_layer=str(first_out),surface_identity=wall['surface_identity'],
                first_layer_identity=seal['first_layer_identity'],native_state=seal['committed_front_identity'],
                files={str(p):file_hash(p) for p in paths},new_surface=False,new_first_layer=False)


def run(output,case,through_layer,*,authorization,profile='production',review,surface=None,first_layer=None,new_first_layer=False,resume_initial=None):
    """One request: fresh surface -> qualified X1 -> existing optimized range."""
    output=Path(output).resolve()
    if not output.is_relative_to((V4/'results/live_sf2_surface').resolve()):raise ValueError('FreshOutputOutsideSupportedSurfaceRoot')
    if output.exists():raise ValueError('PreserveFreshRangeOutput')
    grant=validate_range(case,through_layer,authorization,profile)
    if new_first_layer:
        if surface is None or first_layer is not None:raise ValueError('SealedSurfaceAndNewFirstLayerRequired')
    elif (surface is None)!=(first_layer is None):raise ValueError('CompleteSealedIngestInputsRequired')
    if resume_initial is not None and (not new_first_layer or profile!='production'):
        raise ValueError('ProductionInitialResumeRequiresSealedSurface')
    if first_layer is not None:
        if through_layer<=1:raise ValueError('IngestContinuationRangeRequired')
        grant['from_layer']=1
    start=time.perf_counter();stage='reviewed_code';last=None
    with ExitStack() as held:
        code=verify_code(held,review)
        from run_live_sf2_surface import resource_plan
        if resource_plan()!=read(Path(review)/'CODE_CONNECTION.json')['content']['initialization_resources']['surface']:
            raise ValueError('ChangedReviewedSurfaceResourcePolicy')
        output.mkdir(parents=True,exist_ok=False)
        import immutable_state_access as access
        try:
            if first_layer is not None:
                stage='sealed_initial_ingestion'
                ingestion=ingest_initial(held,surface,first_layer,case)
                grant['ingested_initial']=ingestion
                durable_json(output/'INGESTION.json',ingestion)
            durable_json(output/'REQUEST.json',dict(grant,code_identity=code))
            held.enter_context(access.guard(output/'REQUEST.json'))
            # The existing fresh-process runner independently qualifies its new
            # surface and compares to the retained reference before publication.
            if surface is None:
                stage='fresh_surface'
                cmd=[sys.executable,'-B',str(V4/'tools/replay_live_sf2_surface.py'),'--output',str(output/'surface')]
                durable_json(output/'SURFACE_INVOCATION.json',dict(command=cmd,retained_mesh_is_generation_input=False))
                if subprocess.call(cmd,cwd=ROOT):raise ValueError('FreshSurfaceStageFailed')
                surface=output/'surface/authority';last=0
            elif new_first_layer:
                stage='sealed_surface_ingestion';surface=Path(surface).resolve()
                durable_json(output/'SURFACE_INGESTION.json',ingest_surface(held,surface,case));last=0
            if first_layer is None:
                stage='first_layer';first_out=output/'first_layer';first_out.mkdir()
                if resume_initial is None:
                    request_record=first_request(first_out,surface,grant)
                else:
                    import shutil
                    source=Path(resume_initial).resolve()
                    held.enter_context(access.guard(source.parent/'REQUEST.json'))
                    prior=read(source.parent/'REQUEST.json')
                    if any(prior[k]!=grant[k] for k in ('from_layer','through_layer','profile','case_sha256','authorization')):
                        raise ValueError('ChangedRetainedInitialRange')
                    for name in ('REQUEST.json','INITIAL_QUALIFICATION.json','PRODUCTION_CHECKS.json','IMPLEMENTATION.json','A/STOP.json','A/RAW_QUALITY.json'):
                        held.enter_context(access.guard(source/name))
                    # Existing request approval and provenance remain byte-exact.
                    shutil.copyfile(source/'REQUEST.json',first_out/'REQUEST.json')
                    held.enter_context(access.guard(first_out/'REQUEST.json'))
                    if file_hash(first_out/'REQUEST.json')!=file_hash(source/'REQUEST.json'):
                        raise ValueError('CopiedRetainedInitialRequestChanged')
                    from production_prism_sizing import read_request
                    from production_surface_authority import load_production_wall
                    request_record=read_request(first_out/'REQUEST.json',load_production_wall(surface)[2])
                hold_request(held,first_out,request_record)
                from production_surface_authority import load_production_wall
                _,_,s=load_production_wall(surface)
                for label in (('A',) if profile=='production' else ('A','B')):
                    import run_live_sf2_first_layer as initial
                    initial.configure_inputs(first_out,surface,first_out/'REQUEST.json',s['surface_identity'])
                    if initial.preflight()['resources']!=read(Path(review)/'CODE_CONNECTION.json')['content']['initialization_resources']['first']:
                        raise ValueError('ChangedReviewedFirstLayerResourcePolicy')
                    cmd=[sys.executable,'-B',str(V4/'tools/execute_live_sf2_first_layer.py'),label,
                        '--output',str(first_out),'--surface',str(surface),'--request',str(first_out/'REQUEST.json'),
                        '--surface-identity',s['surface_identity'],'--profile',profile]
                    if resume_initial is not None:cmd += ['--resume-initial',str(Path(resume_initial).resolve())]
                    durable_json(first_out/(label+'.INVOCATION.json'),dict(command=cmd))
                    if subprocess.call(cmd,cwd=ROOT):raise ValueError('FreshFirstLayerStageFailed:'+label)
                    held.enter_context(access.guard(first_out/('PREFLIGHT_'+label+'.json')))
                    if read(first_out/('PREFLIGHT_'+label+'.json'))['resources']!=read(Path(review)/'CODE_CONNECTION.json')['content']['initialization_resources']['first']:
                        raise ValueError('ChangedReviewedFirstLayerResourcePolicy')
                    import run_live_sf2_first_layer as initial
                    initial.configure_inputs(first_out,surface,first_out/'REQUEST.json',s['surface_identity'])
                    import verify_live_sf2_first_layer as verify
                    verify.OUT,verify.SURFACE,verify.REQUEST=first_out,surface,first_out/'REQUEST.json'
                    if profile=='qualification':verify.coverage(label)
                stage='first_layer_receipt'
                if profile=='qualification':verify.replay_and_index()
                elif not read(first_out/'INITIAL_ENROLLMENT.json')['independent_readback_reused']:
                    raise ValueError('MissingProductionInitialReadback')
                last=1
            else:
                surface=Path(surface).resolve();first_out=Path(first_layer).resolve();last=1
            if through_layer>1:
                stage='initial_enrollment';init=output/'initial';init.mkdir()
                control=bind_initial_front(init,first_out,surface,grant)
                from production_prism_stack import execution_resources
                resources=read(first_out/'PREFLIGHT_A.json')['resources']
                approved_resources=read(Path(review)/'CODE_CONNECTION.json')['content']['initialization_resources']['first']
                if resources!=approved_resources:raise ValueError('ChangedReviewedFirstLayerResourcePolicy')
                control['reviewed_resources']=approved_resources
                control['resource_policy']=dict(total_storage_policy='monitor_without_output_cap',output_limit_bytes=None,
                    minimum_free_bytes=resources['minimum_free_bytes'])
                control['resource_origin']=dict(path=str((first_out/'PREFLIGHT_A.json').resolve()),sha256=file_hash(first_out/'PREFLIGHT_A.json'))
                execution_resources(resources,control['resource_policy'])
                durable_json(output/'CONTROL.json',control)
                # Hold explicit input guards across the shared loop, not just at
                # JSON decoding. No old recovery/access anchors are injected.
                import immutable_state_access as access
                for row in control['fresh_inputs'].values():held.enter_context(access.guard(row['path']))
                stage='optimized_continuation'
                import finish_retained_l5 as monitor
                monitor.OUT=output/'monitor';monitor.OUT.mkdir()
                command=[sys.executable,'-B',str(V4/'tools/run_current_front.py'),'fresh-worker',
                    '--output',str(output),'--code-review',str(Path(review).resolve()),
                    '--control-sha256',file_hash(output/'CONTROL.json')]
                if monitor.monitor(worker_command=command,enforce_output_cap=False,storage_roots=[output],
                    resources=execution_resources(resources,control['resource_policy'])):
                    raise ValueError('FreshContinuationStageFailed')
                last=read(output/'run/RESULT.json')['last_qualified_layer']
            durable_json(output/'RESULT.json',dict(passed=True,last_qualified_layer=last,seconds=time.perf_counter()-start,
                code_identity=code,range_request=grant,no_core=True,no_cfd=True))
            return 0
        except Exception as error:
            if (output/'run/LAST_QUALIFIED.json').exists():last=read(output/'run/LAST_QUALIFIED.json')['layer']
            durable_json(output/'STOP.json',dict(stage=stage,error=str(error),traceback=traceback.format_exc(),
                last_qualified_layer=last,seconds=time.perf_counter()-start,no_retry=True))
            raise

def continue_worker(output,review,control_sha256):
    """Fresh-process boundary: guarded current code and exact initialization pins."""
    import immutable_state_access as access
    import run_authorized_continuation as runner
    output=Path(output).resolve()
    if not output.is_relative_to((V4/'results/live_sf2_surface').resolve()):raise ValueError('FreshOutputOutsideSupportedSurfaceRoot')
    with ExitStack() as held:
        code=verify_code(held,review)
        for name in ('CONTROL.json','REQUEST.json'):held.enter_context(access.guard(output/name))
        if file_hash(output/'CONTROL.json')!=control_sha256:raise ValueError('ChangedFreshContinuationControl')
        grant=read(output/'REQUEST.json');control=read(output/'CONTROL.json');auth=control['authorization_record']
        if (grant['code_identity']!=code or auth.get('range_request')!={k:v for k,v in grant.items() if k!='code_identity'}
            or auth['authorized_layers']!=list(range(2,grant['through_layer']+1))
            or control['execution_profile']!=grant['profile']):raise ValueError('UnboundFreshRangeExecution')
        for row in control['fresh_inputs'].values():held.enter_context(access.guard(row['path']))
        held.enter_context(access.guard(control['resource_origin']['path']))
        if control['reviewed_resources']!=read(Path(review)/'CODE_CONNECTION.json')['content']['initialization_resources']['first']:
            raise ValueError('ChangedReviewedFirstLayerResourcePolicy')
        runner.continuation_inputs(control)
        runner.resource_origin(control)
        runner.OUT=output/'run';runner.OUT.mkdir(exist_ok=False)
        runner.execute(continuation=control)

def hold_request(held,directory,bound):
    """Protect exact generated request and every validated source document."""
    import immutable_state_access as access
    request=Path(directory)/'REQUEST.json'
    held.enter_context(access.guard(request))
    if file_hash(request)!=bound['request_file_sha256']:raise ValueError('ChangedGeneratedFirstRequest')
    for row in bound['declaration']['source_documents']:
        p=(request.parent/row['path']).resolve()
        held.enter_context(access.guard(p))
        if file_hash(p)!=row['sha256']:raise ValueError('ChangedGeneratedFirstRequestSource')

def ingest_surface(held,surface,case_path):
    """Existing sealed wall only; fresh first-front generation follows."""
    import immutable_state_access as access
    from production_surface_authority import load_production_wall
    if not surface.is_relative_to((V4/'results').resolve()):raise ValueError('IngestOutsideV4Results')
    for name in ('SURFACE_AUTHORITY.json','IMPLEMENTATION.json','REPLAY.json','CASE.json','REFERENCES.npz'):
        held.enter_context(access.guard(surface/name))
    if file_hash(surface/'CASE.json')!=file_hash(case_path):raise ValueError('IngestCaseMismatch')
    _,_,wall=load_production_wall(surface)
    for name in wall['files']:held.enter_context(access.guard(surface/name))
    _,_,wall=load_production_wall(surface)
    return dict(surface=str(surface),surface_identity=wall['surface_identity'],new_surface=False,new_first_layer=True)
