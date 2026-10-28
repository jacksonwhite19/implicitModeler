"""Read-only retained production replay into a fresh, separate output.

Trusted inputs are the expected VERIFICATION hash and explicit verifier runtime
identity. This audits retained geometry, never grants a live source capability.
"""
from pathlib import Path
from contextlib import ExitStack
import argparse,hashlib,json,math,time
import geometry
from artifact_finalization import hold_manifest,safe_path
from immutable_state_access import guard
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import file_hash
from verification_policy import MANIFEST,OFFLINE,profile

def verifier_identity(output):
    from source_custody import selected,ROOT,sha
    rows=[dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),bytes=p.stat().st_size) for p in selected(Path(output).resolve())]
    return hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def authenticate(folder,expected,scope):
    root=safe_path(folder,'VERIFICATION.json');scope.enter_context(guard(root))
    if file_hash(root)!=expected:raise ValueError('ChangedProductionExpectedRoot')
    value=json.loads(root.read_text())
    if (value.get('schema')!=MANIFEST or value.get('status')!='production_accepted' or value.get('verification_policy')!='production'
        or value.get('acceptance_profile')!=profile() or value.get('audit')!='not_run' or value.get('independent_semantic_replay')!='not_run'
        or value.get('production_bypass') is not False or value.get('acceptance_status')!='production_accepted'
        or value.get('independent_audit_status')!='not_run' or value.get('producer_checks_passed') is not True
        or value.get('artifact_fidelity_passed') is not True or value.get('geometry_complete') is not True
        or value.get('producer_artifact_binding')!='passed' or value.get('source_custody')!='closed_passed'
        or value.get('worker_exit_code')!=0 or value.get('supervisor_stop_reason') is not None):raise ValueError('UnsupportedProductionAcceptance')
    from audit_completion import PRODUCTION_EXTRA,validate_production_completion,REQUEST
    files=value.get('completion_files',{})
    if set(files)!=set(PRODUCTION_EXTRA):raise ValueError('IncompleteProductionBundle')
    for name,pin in {**files,REQUEST:value['handoff_sha256'],**value.get('closure_files',{})}.items():
        path=safe_path(folder,name);scope.enter_context(guard(path))
        if file_hash(path)!=pin:raise ValueError('ChangedProductionCompletionArtifact',name)
    if set(value.get('closure_files',{}))!={'closure/SOURCES.json','closure/SUPERVISOR.json'}:raise ValueError('IncompleteProductionClosure')
    read=lambda p:json.loads((folder/p).read_text())
    request=read(REQUEST)
    if (request.get('schema')!='V35ProductionGuardHandoff/1' or request.get('verification_policy')!='production'
        or request.get('files')!=files or request.get('finalization')!=value['finalization']):raise ValueError('ChangedProductionHandoff')
    sources=read('closure/SOURCES.json');supervisor=read('closure/SUPERVISOR.json')
    if (sources.get('state')!='closed' or sources.get('passed') is not True or sources.get('changed') or sources.get('added')
        or sources.get('identity')!=value.get('source_identity') or supervisor.get('source_identity')!=sources['identity']
        or supervisor.get('exit_code')!=0 or supervisor.get('stop_reason') is not None or supervisor.get('monitor_error') is not None):raise ValueError('InvalidOriginalSourceClosure')
    finalized=hold_manifest(folder/'background',value['finalization'],scope)
    if value.get('execution_record')!=finalized.get('execution_record') or value.get('producer_implementation_identity')!=value.get('source_identity'):
        raise ValueError('ChangedProductionExecutionRecord')
    root_result=read('RESULT.json');direct=read('background/RESULT.json')
    validate_production_completion(folder,root_result,direct,value['finalization'])
    from source_evidence import record as source_evidence_record
    if value.get('source_evidence')!=source_evidence_record(folder):raise ValueError('ChangedOriginalSourceEvidenceRecord')
    return value

def audit(folder,expected,verifier,output,seconds=10800):
    folder=Path(folder).resolve();output=Path(output).absolute()
    # Resolve parents before creation and reject any overlap/reparse redirect.
    if output.resolve().is_relative_to(folder) or folder.is_relative_to(output.resolve()):raise ValueError('OfflineOutputMustBeSeparate')
    if output.exists():raise ValueError('FreshOfflineAuditOutputRequired')
    if not math.isfinite(seconds) or seconds<=0:raise ValueError('PositiveOfflineAuditDeadlineRequired')
    output.mkdir(parents=True,exist_ok=False);started=time.monotonic();result=None
    from source_custody import held
    try:
        with held(output,output/'VERIFIER_SOURCES.json') as custody,ExitStack() as scope:
            if custody['identity']!=verifier:raise ValueError('UnsupportedVerifierIdentity')
            root=authenticate(folder,expected,scope)
            from direct_readback import _verify
            replay=_verify(folder/'background',root['finalization']['manifest_sha256'],max(.001,seconds-(time.monotonic()-started)),production=True,output=output/'replay')
            if replay.get('passed') is not True or replay.get('all_twelve_semantically_replayed') is not True:raise ValueError('OfflineSemanticReplayFailed')
            # Recheck the retained root while still guarded; no source writes.
            authenticate(folder,expected,scope)
        if custody.get('passed') is not True:raise ValueError('OfflineVerifierSourceClosureFailed')
        result=dict(schema=OFFLINE,status='audit_passed',passed=True,source_bundle_root=expected,verifier_identity=verifier,
            producer_source_identity=root['source_identity'],fresh_source_authority=False,remeshed=False,search_rerun=False,
            source_evidence=root['source_evidence'],
            replay=replay,seconds=time.monotonic()-started,CFD_qualified=False)
    except BaseException as error:
        durable_json(output/'AUDIT_RESULT.json',dict(schema=OFFLINE,status='audit_failed',passed=False,source_bundle_root=expected,
            verifier_identity=verifier,fresh_source_authority=False,error=repr(error),seconds=time.monotonic()-started))
        raise
    durable_json(output/'AUDIT_RESULT.json',result)
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--print-verifier-identity',action='store_true')
    p.add_argument('--bundle',type=Path);p.add_argument('--expected-root');p.add_argument('--verifier-identity');p.add_argument('--output',type=Path)
    p.add_argument('--timeout-seconds',type=float,default=10800)
    args=p.parse_args()
    from mesh import source_independence_guard
    source_independence_guard()
    if args.print_verifier_identity:
        print(verifier_identity(Path(__file__).parent/'results'));return 0
    if any(getattr(args,k) is None for k in ('bundle','expected_root','verifier_identity','output')):p.error('bundle, expected-root, verifier-identity and fresh output are required')
    audit(args.bundle,args.expected_root,args.verifier_identity,args.output,args.timeout_seconds)
    return 0

if __name__=='__main__':raise SystemExit(main())
