"""Current authenticated front configuration for the shared production executor.

No geometry, meshing, proof, recovery or publication algorithm lives here.
"""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import json
import sys
import time
V4=Path(__file__).resolve().parents[1]; ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
OUT=V4/'results/current_front_l9'; ACCESS=V4/'results/storage_cleanup_mesh_readiness'
CODE_REVIEW=V4/'results/fast_new_first'
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity
from record_receipt_resume import disk
from run_successor_benchmark import execution_sources,execution_binaries,retained_execution_inputs
from capsule_reuse_dispatch import PRODUCTION_CONTROLS


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))


def configure():
    if (OUT/'CONTROL.json').exists():raise ValueError('PreserveCurrentFrontControl')
    previous=read(V4/'results/wing_continuation_layers6to8/AUTHORIZATION.json')
    parent=dict(read(ACCESS/'REOPEN.json'),layer=8)
    old=read(V4/'results/wing_l9_preflight/AUTHORIZATION.json')
    auth={k:deepcopy(v) for k,v in old.items() if k not in ('predecessor','predecessor_identity')}
    auth.update(schema='V4CurrentFrontContinuationAuthorization/1',approved_parent=parent,
        history=previous,history_identity=identity(previous),
        authorization='User: reconnect most updated authorization/access to qualified L8; resume authorized L9 only. Old grants and admissions retain their identities.')
    recovery=read(ACCESS/'RECOVERY.json')['record']; indices=ACCESS/'INDEX_ACCESS.json'
    control=dict(read(V4/'results/wing_l9_preflight/CONTROL.json'),resume=False,parent=parent,
        authorization_record=auth,authorization=identity(auth),
        current_indices=dict(path=str(indices),sha256=file_hash(indices),identity=identity(read(indices))))
    control['recovered_access']=dict(directory=str(ACCESS/'access'),identity=identity(recovery),reference=recovery['reference'])
    durable_json(OUT/'AUTHORIZATION.json',auth);durable_json(OUT/'CONTROL.json',control)
    print('Authorized current front',identity(auth),parent['scientific'],auth['increments'],flush=True)


def configure_range(parent_path, history_path, from_layer, through_layer, *, authorization,
                    profile='production', template_path=None):
    """One requested range; the existing executor owns the sequential loop.

    This case wrapper uses the already bound SF2 owner. It never loads historical
    front coordinates as generation inputs. Preflight reconstructs the live owner
    and authenticates the actual current parent before admission can run.
    """
    from production_prism_stack import ExecutionProfile
    from accepted_production_contract import range_authorization
    from accepted_trajectory_contract import AcceptedSchedule
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY,PRIOR
    from sealed_front_connection import load_case
    import immutable_state_access as access
    if type(from_layer) is not int or type(through_layer) is not int:raise ValueError('InvalidRequestedLayerRange')
    profile=ExecutionProfile(profile)
    if OUT.exists():raise ValueError('PreserveCurrentFrontRangeOutput')
    template_path=template_path or V4/'results/current_front_l9/CONTROL.json'
    diagnostic_path=V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json'
    with ExitStack() as held:
        for p in (parent_path,history_path,template_path,diagnostic_path):held.enter_context(access.guard(p))
        parent=read(parent_path)
        # REOPEN is a pointer; actual authentication is mandatory in preflight
        # and again in the executor, not inferred from this passed flag.
        if 'layer' in parent and parent['layer']!=from_layer:raise ValueError('WrongRequestedStartingLayer')
        parent=dict(parent,layer=from_layer)
        history=read(history_path);template=read(template_path)
        case=read(PRIOR/'CASES.json')['wing'];wall,_,_,_=load_case(case)
        state,selected=existing_state_class().load(STUDY/'planner/selected_state')
        schedule=AcceptedSchedule(read(diagnostic_path),selected,exact_vertex_map(state.wall,wall))
        auth=range_authorization(history,parent,through_layer,schedule,authorization=authorization)
        control=dict(schema='V4AuthorizedContinuationExecution/1',layers=auth['authorized_layers'],resume=False,
            parent=parent,authorization_record=auth,authorization=identity(auth),
            execution_profile=profile.value,compare_inside_publisher=(profile.value=='qualification'),
            range_request=dict(from_layer=from_layer,through_layer=through_layer,authorization=authorization,profile=profile.value))
        # Explicit reuse of established recovery and resource policy. Do not copy
        # an old front's current_indices into a different committed parent.
        for name in ('recovered_access','resource_policy'):
            if name in template:control[name]=deepcopy(template[name])
        original_baseline=read(V4/'results/current_front_l10_preflight/BASELINE.json')
        baseline=dict(protected={p:file_hash(p) for p in original_baseline['protected']},
            binaries=execution_binaries(),
            inputs={str(Path(p).resolve()):file_hash(p) for p in (parent_path,history_path,template_path,diagnostic_path)},
            request=dict(from_layer=from_layer,through_layer=through_layer,authorization=authorization,profile=profile.value),
            disk=disk())
        OUT.mkdir(parents=True,exist_ok=False)
        durable_json(OUT/'BASELINE.json',baseline)
        durable_json(OUT/'AUTHORIZATION.json',auth);durable_json(OUT/'CONTROL.json',control)
    return control


def reader_scope(held):
    from native_reader_source_access import qualified_reader_access
    approved=read(ACCESS/'RECOVERY_REVIEW.json')
    return held.enter_context(qualified_reader_access(ACCESS/'SOURCE_CONNECTION.json',approved['source_connection_identity'],
        ACCESS/'SOURCE_REVIEW.json',approved['source_review_sha256']))


def preflight():
    import immutable_state_access as access
    with ExitStack() as held:
        for name in ('BASELINE.json','CONTROL.json','AUTHORIZATION.json'):
            held.enter_context(access.guard(OUT/name))
        return guarded_preflight()


def guarded_preflight():
    import check_wing_l9_preflight as check
    import run_authorized_continuation as runner
    from unittest.mock import patch
    # Reuse the prior complete live geometry/parent/target check; add the actual
    # renewed index session to its production binding. No forecast/cells here.
    attempt=OUT/('preflight_'+str(len(list(OUT.glob('preflight_*')))+1));attempt.mkdir(exist_ok=False)
    b=read(OUT/'BASELINE.json')
    runtime=execution_sources([Path(__file__),V4/'tools/run_authorized_continuation.py'])
    control=read(OUT/'CONTROL.json')
    b['runtime']=runtime
    b['execution_control_identity']=identity(control)
    b['authorization_identity']=control['authorization']
    owned={str(V4/'tools'/n) for n in ('accepted_production_contract.py','run_authorized_continuation.py')}
    changes={p:dict(before=h,after=file_hash(p)) for p,h in b['protected'].items() if p in owned}
    b['protected']={p:h for p,h in b['protected'].items() if p not in owned}
    inputs=[ACCESS/n for n in ('RECOVERY.json','REOPEN.json','SOURCE_CONNECTION.json','SOURCE_REVIEW.json','RECOVERY_REVIEW.json','INDEX_ACCESS.json')]
    b.setdefault('inputs',{}).update({str(p):file_hash(p) for p in inputs})
    b['inputs'].update({p:c['after'] for p,c in changes.items()})
    b['inputs'].update(runtime)
    b['intentional_source_changes']=changes
    durable_json(attempt/'BASELINE.json',b)
    for name in ('CONTROL.json','AUTHORIZATION.json'):durable_json(attempt/name,read(OUT/name))
    original_bind=__import__('accepted_production_contract').bind_production
    def indexed_bind(base,case,owner,auth,pin):
        adapter,inspected=original_bind(base,case,owner,auth,pin)
        with runner.committed_index_session(base,adapter,control) as (numeric,pairs):
            durable_json(attempt/'INDEX_CHECK.json',dict(passed=True,scientific=base.scientific,reference=base.reference,
                authorization=pin,control_identity=identity(control),runtime_identity=identity(runtime),
                current_indices=control.get('current_indices'),numeric_faces=len(numeric),pairs_open=True))
        return adapter,inspected
    with patch.object(check,'OUT',attempt),patch('accepted_production_contract.bind_production',indexed_bind):
        code=check.check()
    durable_json(OUT/'PREFLIGHT_POINTER.json',dict(path=str(attempt/'CHECK.json'),sha256=file_hash(attempt/'CHECK.json')))
    return code


def request():
    path=OUT/('REVIEW_REQUEST_'+str(len(list(OUT.glob('REVIEW_REQUEST*.json')))+1)+'.json')
    attempt=Path(read(OUT/'PREFLIGHT_POINTER.json')['path']).parent
    runtime=execution_sources([Path(__file__),V4/'tools/run_authorized_continuation.py'])
    inputs=retained_execution_inputs({str(ACCESS/n):file_hash(ACCESS/n) for n in (
        'RECOVERY.json','REOPEN.json','SOURCE_CONNECTION.json','SOURCE_REVIEW.json','RECOVERY_REVIEW.json','INDEX_ACCESS.json')})
    inputs.update(read(OUT/'BASELINE.json')['inputs'])
    code={}
    if read(OUT/'CONTROL.json').get('range_request') is not None:
        code=dict(code_connection_sha256=file_hash(CODE_REVIEW/'CODE_CONNECTION.json'))
    durable_json(path,dict(runtime=runtime,inputs=inputs,binary=execution_binaries(),**code,
        baseline_sha256=file_hash(OUT/'BASELINE.json'),control_sha256=file_hash(OUT/'CONTROL.json'),
        authorization_sha256=file_hash(OUT/'AUTHORIZATION.json'),flags=PRODUCTION_CONTROLS,
        preflight_pointer_sha256=file_hash(OUT/'PREFLIGHT_POINTER.json'),
        preflight_sha256=file_hash(attempt/'CHECK.json'),index_check_sha256=file_hash(attempt/'INDEX_CHECK.json'),
        preflight_baseline_sha256=file_hash(attempt/'BASELINE.json'),preflight_control_sha256=file_hash(attempt/'CONTROL.json')))
    durable_json(OUT/'REVIEW_POINTER.json',dict(path=str(path),sha256=file_hash(path)))


def validate_preflight_binding(control,request,baseline,result,indices,attempt_control):
    """A hash-pinned success for another grant/front is not this prerequisite."""
    parent=control['parent'];phases=result.get('phases',{})
    if (attempt_control!=control or baseline.get('runtime')!=request['runtime']
        or baseline.get('execution_control_identity')!=identity(control)
        or baseline.get('authorization_identity')!=control['authorization']
        or result.get('passed') is not True or result.get('stage')!='trajectory_premises'
        or phases.get('geometry_schedule',{}).get('authorization')!=control['authorization']
        or phases.get('parent',{}).get('reference')!=parent['reference']
        or phases.get('parent',{}).get('scientific')!=parent['scientific']
        or phases.get('parent',{}).get('layer')!=parent['layer']
        or phases.get('binding',{}).get('increment',{}).get('layer')!=control['layers'][0]
        or phases.get('trajectory',{}).get('admitted') is not True
        or indices.get('passed') is not True or indices.get('scientific')!=parent['scientific']
        or indices.get('reference')!=parent['reference'] or indices.get('authorization')!=control['authorization']
        or indices.get('control_identity')!=identity(control) or indices.get('runtime_identity')!=identity(request['runtime'])
        or indices.get('current_indices')!=control.get('current_indices')):
        raise ValueError('WrongCurrentFrontPreflightBinding')


def verified(held):
    import immutable_state_access as access
    for name in ('REVIEW_POINTER.json','BASELINE.json','CONTROL.json','AUTHORIZATION.json','PREFLIGHT_POINTER.json'):
        held.enter_context(access.guard(OUT/name))
    pointer=read(OUT/'REVIEW_POINTER.json');held.enter_context(access.guard(pointer['path']))
    if file_hash(pointer['path'])!=pointer['sha256']:raise ValueError('ChangedCurrentFrontReviewRequest')
    q=read(pointer['path'])
    control=read(OUT/'CONTROL.json')
    if 'code_connection_sha256' in q:
        verify_range_code(held,q,control)
    else:
        if control.get('range_request') is not None:raise ValueError('MissingReviewedRangeCode')
        held.enter_context(access.guard(OUT/'REVIEW.json'));review=read(OUT/'REVIEW.json')
        if (review.get('passed') is not True or not review.get('reviewer')
            or review['request_sha256']!=pointer['sha256'] or review['runtime']!=q['runtime']):
            raise ValueError('UnreviewedCurrentFrontExecution')
    preflight=read(OUT/'PREFLIGHT_POINTER.json');attempt=Path(preflight['path']).parent
    for name in ('CHECK.json','INDEX_CHECK.json','BASELINE.json','CONTROL.json'):held.enter_context(access.guard(attempt/name))
    if (q['baseline_sha256']!=file_hash(OUT/'BASELINE.json') or q['control_sha256']!=file_hash(OUT/'CONTROL.json')
        or q['authorization_sha256']!=file_hash(OUT/'AUTHORIZATION.json') or q['flags']!=PRODUCTION_CONTROLS
        or q['preflight_pointer_sha256']!=file_hash(OUT/'PREFLIGHT_POINTER.json')
        or q['preflight_sha256']!=file_hash(preflight['path']) or preflight['sha256']!=q['preflight_sha256']
        or q['index_check_sha256']!=file_hash(attempt/'INDEX_CHECK.json')
        or q['preflight_baseline_sha256']!=file_hash(attempt/'BASELINE.json')
        or q['preflight_control_sha256']!=file_hash(attempt/'CONTROL.json')):
        raise ValueError('UnreviewedCurrentFrontExecution')
    validate_preflight_binding(read(OUT/'CONTROL.json'),q,read(attempt/'BASELINE.json'),read(attempt/'CHECK.json'),
        read(attempt/'INDEX_CHECK.json'),read(attempt/'CONTROL.json'))
    for p,h in {**q['runtime'],**q['inputs'],**q['binary']}.items():
        held.enter_context(access.guard(p))
        if file_hash(p)!=h:raise ValueError('ChangedCurrentFrontExecutionDependency',p)
    reader_scope(held)
    from current_front_dependencies import install_reader
    install_reader()
    return read(OUT/'CONTROL.json')


def validate_range_code_binding(content,request,control):
    """Reviewed software may serve new data requests; it never approves a case.

    Every runtime/binary must still be in that exact reviewed closure. The live
    case retains separate guarded input pins, preflight and production admission.
    """
    from production_prism_stack import continuation_scope,ExecutionProfile
    layers=continuation_scope(control,control['authorization_record'],control['authorization'])
    r=control.get('range_request');auth=control['authorization_record']
    if (not isinstance(r,dict) or type(r.get('from_layer')) is not int
        or type(r.get('through_layer')) is not int or r['from_layer']!=control['parent']['layer']
        or r['through_layer']!=layers[-1] or r['authorization']!=auth['authorization']
        or r['profile']!=control.get('execution_profile')
        or control.get('compare_inside_publisher')!=(r['profile']=='qualification')):
        raise ValueError('WrongReviewedRangeRequest')
    ExecutionProfile(r['profile'])
    if (not request['runtime'] or not request['binary']
        or any(content['sources'].get(p)!=h for p,h in request['runtime'].items())
        or request['binary']!=content.get('continuation_binaries',content['binaries'])):
        raise ValueError('ChangedReviewedRangeCode')


def verify_range_code(held,request,control):
    """Reuse the guarded review/closure association, not a fabricated case review."""
    import immutable_state_access as access
    held.enter_context(access.guard(CODE_REVIEW/'CODE_CONNECTION.json'))
    if read(CODE_REVIEW/'CODE_CONNECTION.json')['content'].get('schema')=='V4SharedPrismToolCodeReview/1':
        from front_bootstrap import verify_code
        verify_code(held,CODE_REVIEW)
        if request['code_connection_sha256']!=file_hash(CODE_REVIEW/'CODE_CONNECTION.json'):raise ValueError('ChangedRangeCodeConnection')
        validate_range_code_binding(read(CODE_REVIEW/'CODE_CONNECTION.json')['content'],request,control)
        return
    for name in ('CODE_CONNECTION.json','REVIEW_REQUEST_2.json','REVIEW.json','check.py'):
        held.enter_context(access.guard(CODE_REVIEW/name))
    record=read(CODE_REVIEW/'CODE_CONNECTION.json');content=record['content']
    q=read(CODE_REVIEW/'REVIEW_REQUEST_2.json');r=read(CODE_REVIEW/'REVIEW.json')
    if (identity(content)!=record['identity'] or request['code_connection_sha256']!=file_hash(CODE_REVIEW/'CODE_CONNECTION.json')
        or r.get('passed') is not True or not r.get('reviewer')
        or r.get('request_sha256')!=content['request_sha256']
        or content['request_sha256']!=file_hash(CODE_REVIEW/'REVIEW_REQUEST_2.json')
        or content['review_sha256']!=file_hash(CODE_REVIEW/'REVIEW.json')
        or content['checker_sha256']!=q['checker_sha256']
        or q['checker_sha256']!=file_hash(CODE_REVIEW/'check.py')
        or any(content[k]!=q[k] for k in ('sources','binaries','inputs','prior_code_authority','intentional_prior_closure_delta','scope'))):
        raise ValueError('UnreviewedRangeCodeConnection')
    validate_range_code_binding(content,request,control)
    for path,pin in {**content['sources'],**content['binaries'],**content['inputs']}.items():
        held.enter_context(access.guard(path))
        if file_hash(path)!=pin:raise ValueError('ChangedReviewedRangeDependency',path)


def run_range(parent,history,from_layer,through_layer,*,authorization,profile='production'):
    """One user range, no per-layer approval or alternate execution workflow."""
    configure_range(parent,history,from_layer,through_layer,authorization=authorization,profile=profile)
    code=preflight()
    if code:return code
    request()
    return monitor()


def worker():
    import run_authorized_continuation as runner
    with ExitStack() as held:
        c=verified(held)
        runner.OUT=OUT/'run';runner.OUT.mkdir(exist_ok=False)
        runner.execute(continuation=c)


def monitor():
    import finish_retained_l5 as m
    with ExitStack() as held:verified(held)
    m.OUT=OUT/'monitor';m.OUT.mkdir(exist_ok=False)
    return m.monitor(worker_command=[sys.executable,'-B',__file__,'--code-review',str(CODE_REVIEW.resolve()),'worker','--output',str(OUT.resolve())],
        enforce_output_cap=False,extra_storage_roots=[OUT/'run'])


if __name__=='__main__':
    raise SystemExit('Historical driver retired. Use research/sdf_cfd_mesher_v4/mesh.py with an explicit case and optional restart.')
    import argparse
    parser=argparse.ArgumentParser(description='One current-front range request; existing optimized sequential execution.')
    parser.add_argument('mode',choices=['configure','configure-range','run-range','run-fresh','run-ingest','run-surface','fresh-worker','preflight','request','worker','monitor'])
    parser.add_argument('--output',type=Path)
    parser.add_argument('--parent',type=Path,help='Existing guarded REOPEN.json pointer')
    parser.add_argument('--history',type=Path,help='Existing immutable authorization record')
    parser.add_argument('--from-layer',type=int);parser.add_argument('--through-layer',type=int)
    parser.add_argument('--authorization',help='Exact user range request provenance, recorded once')
    parser.add_argument('--case',type=Path,help='Approved initial surface/first-height policy')
    parser.add_argument('--surface',type=Path,help='Existing sealed surface for ingestion; not an unqualified mesh')
    parser.add_argument('--first-layer',type=Path,help='Matching sealed qualified L1; ingest mode starts at L1')
    parser.add_argument('--resume-initial',type=Path,help='Passed retained initial candidate stopped at routine publication plumbing; new output required')
    parser.add_argument('--code-review',type=Path,help='Guarded independently reviewed initialization code connection')
    parser.add_argument('--control-sha256',help='Exact parent-worker initialization control pin')
    parser.add_argument('--profile',choices=['production','qualification'],default='production')
    args=parser.parse_args();mode=args.mode
    if args.code_review is not None:CODE_REVIEW=args.code_review.resolve()
    if mode=='fresh-worker':
        if any(v is None for v in (args.output,args.code_review,args.control_sha256)):parser.error('complete guarded fresh worker inputs required')
        from front_bootstrap import continue_worker
        continue_worker(args.output,args.code_review,args.control_sha256);sys.exit(0)
    if mode in ('run-fresh','run-ingest','run-surface'):
        if any(v is None for v in (args.output,args.case,args.through_layer,args.authorization,args.code_review)):
            parser.error('fresh mode requires output, case, through-layer, authorization and code-review')
        ingest=mode=='run-ingest'
        new_first=mode=='run-surface'
        if ingest and any(v is None for v in (args.surface,args.first_layer)):
            parser.error('ingest mode requires sealed surface and matching qualified first-layer')
        if new_first and (args.surface is None or args.first_layer is not None):
            parser.error('run-surface requires sealed surface; always generates a new L1')
        if not ingest and not new_first and any(v is not None for v in (args.surface,args.first_layer)):
            parser.error('fresh mode cannot silently ingest retained initial geometry')
        if args.from_layer not in (None,1 if ingest else 0):parser.error('incorrect requested initial level')
        from front_bootstrap import run
        sys.exit(run(args.output,args.case,args.through_layer,authorization=args.authorization,
                     profile=args.profile,review=args.code_review,
                     surface=args.surface if ingest or new_first else None,first_layer=args.first_layer if ingest else None,
                     new_first_layer=new_first,resume_initial=args.resume_initial))
    if args.output is not None:
        OUT=args.output.resolve()
        if not OUT.is_relative_to(V4/'results'):raise ValueError('OutputOutsideV4Results')
    if mode in ('configure-range','run-range'):
        if any(v is None for v in (args.output,args.parent,args.history,args.from_layer,args.through_layer,args.authorization)):
            parser.error('range mode requires output, parent, history, from-layer, through-layer and authorization')
        if mode=='run-range':
            sys.exit(run_range(args.parent,args.history,args.from_layer,args.through_layer,
                authorization=args.authorization,profile=args.profile))
        c=configure_range(args.parent,args.history,args.from_layer,args.through_layer,
            authorization=args.authorization,profile=args.profile)
        print('Range configured',c['layers'],'authorization',c['authorization'],flush=True)
        sys.exit(0)
    if mode in ('preflight','monitor'):sys.exit(globals()[mode]())
    elif mode in ('configure','request','worker'):globals()[mode]()
    else:raise ValueError('UnsupportedCurrentFrontMode')
