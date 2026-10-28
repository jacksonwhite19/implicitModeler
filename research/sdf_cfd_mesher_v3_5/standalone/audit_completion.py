"""Versioned verification guard handoff from worker to supervisor.

Request/ack files are fresh per output. The worker retains its bundle guards
until the supervisor holds the same expected roots. Only the supervisor writes
VERIFICATION.json after worker exit and clean source/resource closure.
"""
from contextlib import ExitStack
from pathlib import Path
import json,os,time
from immutable_state_access import guard
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import file_hash,identity
from artifact_finalization import hold_manifest,safe_path

REQUEST='FINALIZATION_HANDOFF.json';ACK='FINALIZATION_ACK.json'
SCHEMA='V35AuditGuardHandoff/1'
EXTRA=('CASE.json','RESULT.json','background/RESULT.json','background/READBACK.json')
from source_evidence import FILES as SOURCE_EVIDENCE_FILES
PRODUCTION_EXTRA=EXTRA[:-1]+SOURCE_EVIDENCE_FILES

def handoff_contract(descriptor):
    from verification_policy import FINALIZER,profile
    if descriptor.get('verification_policy')=='production':
        if descriptor.get('schema')!=FINALIZER or descriptor.get('acceptance_profile')!=profile():raise ValueError('UnknownProductionFinalizer')
        return 'V35ProductionGuardHandoff/1',PRODUCTION_EXTRA
    if descriptor.get('verification_policy')!='audit':raise ValueError('UnknownVerificationMode')
    return SCHEMA,EXTRA

def read(path):return json.loads(Path(path).read_text(encoding='utf8'))

def worker_handoff(folder,descriptor,deadline,scope):
    folder=Path(folder).resolve()
    if (folder/REQUEST).exists() or (folder/ACK).exists():raise ValueError('FreshAuditHandoffRequired')
    pins={}
    schema,extra=handoff_contract(descriptor)
    for name in extra:
        path=safe_path(folder,name);scope.enter_context(guard(path));pins[name]=file_hash(path)
    request=dict(schema=schema,verification_policy=descriptor['verification_policy'],worker_pid=os.getpid(),supervisor_pid=os.getppid(),
        finalization=descriptor,files=pins)
    durable_json(folder/REQUEST,request);scope.enter_context(guard(folder/REQUEST));expected=file_hash(folder/REQUEST)
    while not (folder/ACK).exists():
        if time.monotonic()>=deadline:raise TimeoutError('AuditGuardHandoffDeadline')
        time.sleep(.05)
    scope.enter_context(guard(folder/ACK));ack=read(folder/ACK)
    if ack!=dict(schema=schema,request_sha256=expected,supervisor_pid=os.getppid()):
        raise ValueError('InvalidAuditGuardAcknowledgement')

class SupervisorHandoff:
    def __init__(self,folder,worker,*,verification='audit'):
        self.folder=Path(folder).resolve();self.scope=ExitStack();self.worker=worker
        try:
            if worker.poll() is not None:raise ValueError('AuditHandoffWorkerAlreadyExited')
            path=safe_path(self.folder,REQUEST);self.scope.enter_context(guard(path));self.request_pin=file_hash(path);r=read(path)
            schema,extra=handoff_contract(r.get('finalization',{}))
            if r.get('schema')!=schema or r.get('verification_policy')!=verification or r.get('worker_pid')!=worker.pid or r.get('supervisor_pid')!=os.getpid() or set(r.get('files',{}))!=set(extra):
                raise ValueError('UnsupportedAuditGuardHandoff')
            self.request=r
            self.finalization=hold_manifest(self.folder/'background',r['finalization'],self.scope)
            for name,pin in r['files'].items():
                path=safe_path(self.folder,name);self.scope.enter_context(guard(path))
                if file_hash(path)!=pin:raise ValueError('AuditHandoffChangedArtifact',name)
            root=read(self.folder/'RESULT.json');direct=read(self.folder/'background/RESULT.json')
            if root.get('background')!=direct:
                raise ValueError('AuditHandoffIncompleteRoot')
            if verification=='production':
                validate_production_completion(self.folder,root,direct,r['finalization'])
            else:
                audit=read(self.folder/'background/READBACK.json')
                if root.get('passed') is not True or root.get('full_mesh_complete') is not True or direct.get('schema')!='V35V4DirectComplete/1' or direct.get('passed') is not True or direct.get('full_mesh_complete') is not True or direct.get('provisional') is not False or direct.get('audit_finalization')!=r['finalization']:
                    raise ValueError('AuditHandoffIncompleteDirect')
                if audit.get('schema')!='V35DirectIndependentReadback/1' or audit.get('passed') is not True or audit.get('all_twelve_semantically_replayed') is not True or audit.get('manifest_sha256')!=r['finalization']['manifest_sha256']:
                    raise ValueError('AuditHandoffColdReplayRequired')
                if direct.get('readback_sha256')!=r['files']['background/READBACK.json']:raise ValueError('AuditHandoffReadbackBinding')
            if direct.get('manifest_sha256')!=r['finalization']['manifest_sha256']:
                raise ValueError('AuditHandoffReadbackBinding')
            # A successful overlap is mandatory. An exited worker or an ACK
            # already present cannot be repaired by acquiring locks afterward.
            if worker.poll() is not None or (self.folder/ACK).exists():raise ValueError('AuditHandoffOverlapRequired')
            durable_json(self.folder/ACK,dict(schema=schema,request_sha256=self.request_pin,supervisor_pid=os.getpid()))
            self.scope.enter_context(guard(self.folder/ACK))
        except BaseException:
            self.scope.close();raise

    def publish(self,sources,supervisor,source_record,supervisor_record):
        if self.worker.poll()!=0 or supervisor.get('exit_code')!=0 or supervisor.get('stop_reason') is not None or supervisor.get('monitor_error') is not None:
            raise ValueError('AuditCompletionUnsuccessfulSupervisor')
        if sources.get('state')!='closed' or sources.get('passed') is not True or sources.get('changed') or sources.get('added') or sources.get('identity')!=supervisor.get('source_identity'):
            raise ValueError('AuditCompletionUnsuccessfulSourceClosure')
        if file_hash(self.folder/REQUEST)!=self.request_pin:raise ValueError('AuditCompletionChangedHandoff')
        closure={}
        for name,path,expected in [('source',source_record,sources),('supervisor',supervisor_record,supervisor)]:
            self.scope.enter_context(guard(path))
            if read(path)!=expected:raise ValueError('AuditCompletionChangedClosure',name)
            closure[name]=dict(path=str(Path(path).resolve()),sha256=file_hash(path))
        if (self.folder/'VERIFICATION.json').exists():raise ValueError('FreshVerificationPublicationRequired')
        value=dict(schema='V35VerificationManifest/1',status='audit_accepted',verification_policy='audit',production_bypass=False,
            independent_semantic_replay='passed',producer_artifact_binding='passed',finalization=self.request['finalization'],
            completion_files=self.request['files'],handoff_sha256=self.request_pin,source_identity=sources['identity'],
            source_custody='closed_passed',worker_exit_code=0,supervisor_stop_reason=None,closure_records=closure,
            assurance='Existing audited engineering path only; no production selection, CFD, continuous geometry or global exact-Q claim')
        if self.request['verification_policy']=='production':
            from verification_policy import MANIFEST,profile
            from source_evidence import record as source_evidence_record
            # Self-contained copies retain original-run custody evidence for
            # later read-only replay; paths outside the bundle are not needed.
            directory=self.folder/'closure';directory.mkdir(exist_ok=False)
            copied={}
            for name,path in [('SOURCES.json',source_record),('SUPERVISOR.json',supervisor_record)]:
                target=directory/name;target.write_bytes(Path(path).read_bytes());self.scope.enter_context(guard(target))
                copied['closure/'+name]=file_hash(target)
            value.update(schema=MANIFEST,status='production_accepted',verification_policy='production',acceptance_profile=profile(),
                independent_semantic_replay='not_run',audit='not_run',independent_audit_status='not_run',acceptance_status='production_accepted',
                geometry_complete=True,producer_checks_passed=True,artifact_fidelity_passed=True,closure_files=copied,
                execution_record=self.finalization['execution_record'],producer_implementation_identity=sources['identity'],
                source_evidence=source_evidence_record(self.folder),
                assurance='Producer engineering checks and finalization only; separate audit not_run; no CFD or fresh offline source authority')
        durable_json(self.folder/'VERIFICATION.json',value)
        return value

    def close(self):self.scope.close()

def requires_handoff(folder):
    path=Path(folder)/'RESULT.json'
    if not path.is_file():return False
    value=read(path)
    from verification_policy import ROOT_RESULT
    return (value.get('full_mesh_complete') is True and value.get('background',{}).get('schema')=='V35V4DirectComplete/1') or value.get('schema')==ROOT_RESULT


def validate_production_completion(folder,root,direct,descriptor):
    from verification_policy import production_result,ROOT_RESULT,profile,require_selection
    production_result(direct,descriptor)
    case=read(Path(folder)/'CASE.json');require_selection(case.get('verification'),case,'production')
    from source_evidence import validate_completion
    validate_completion(folder,case,direct['parent'])
    if (root.get('schema')!=ROOT_RESULT or root.get('geometry_complete') is not True or root.get('producer_checks_passed') is not True
        or root.get('artifact_fidelity_passed') is not True or root.get('independent_audit_status')!='not_run'
        or root.get('acceptance_status')!='pending_supervisor' or 'passed' in root or root.get('verification_policy')!='production'
        or root.get('acceptance_profile')!=profile() or root.get('audit')!='not_run' or 'full_mesh_complete' in root
        or root.get('background')!=direct or (Path(folder)/'background/READBACK.json').exists()):raise ValueError('InvalidProductionCompletion')
    from sdf_cfd_mesher_v4.common.authority import identity
    if (direct.get('parent',{}).get('case_sha256')!=file_hash(Path(folder)/'CASE.json')
        or case.get('background')!=read(Path(folder)/'background/POLICY.json')
        or direct['parent'].get('policy_identity')!=identity(case['background'])):raise ValueError('ProductionCompletionPolicyMismatch')
    provisional=read(Path(folder)/'background/PROVISIONAL.json')
    expected=dict(provisional,acceptance_status='pending_supervisor',provisional=False,artifact_finalization=descriptor,
        manifest_sha256=descriptor['manifest_sha256'],mesh_sha256=file_hash(Path(folder)/'background/COMPLETE_MESH.npz'))
    for key in ('seconds','stage_seconds'):expected[key]=direct.get(key)
    if direct!=expected or root.get('export')!=direct.get('export'):raise ValueError('ProductionCompletionChangedProducerRoot')
