"""One audit-only guard handoff from the owned worker to its supervisor.

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

def read(path):return json.loads(Path(path).read_text(encoding='utf8'))

def worker_handoff(folder,descriptor,deadline,scope):
    folder=Path(folder).resolve()
    if (folder/REQUEST).exists() or (folder/ACK).exists():raise ValueError('FreshAuditHandoffRequired')
    pins={}
    for name in EXTRA:
        path=safe_path(folder,name);scope.enter_context(guard(path));pins[name]=file_hash(path)
    request=dict(schema=SCHEMA,verification_policy='audit',worker_pid=os.getpid(),supervisor_pid=os.getppid(),
        finalization=descriptor,files=pins)
    durable_json(folder/REQUEST,request);scope.enter_context(guard(folder/REQUEST));expected=file_hash(folder/REQUEST)
    while not (folder/ACK).exists():
        if time.monotonic()>=deadline:raise TimeoutError('AuditGuardHandoffDeadline')
        time.sleep(.05)
    scope.enter_context(guard(folder/ACK));ack=read(folder/ACK)
    if ack!=dict(schema=SCHEMA,request_sha256=expected,supervisor_pid=os.getppid()):
        raise ValueError('InvalidAuditGuardAcknowledgement')

class SupervisorHandoff:
    def __init__(self,folder,worker):
        self.folder=Path(folder).resolve();self.scope=ExitStack();self.worker=worker
        try:
            if worker.poll() is not None:raise ValueError('AuditHandoffWorkerAlreadyExited')
            path=safe_path(self.folder,REQUEST);self.scope.enter_context(guard(path));self.request_pin=file_hash(path);r=read(path)
            if r.get('schema')!=SCHEMA or r.get('verification_policy')!='audit' or r.get('worker_pid')!=worker.pid or r.get('supervisor_pid')!=os.getpid() or set(r.get('files',{}))!=set(EXTRA):
                raise ValueError('UnsupportedAuditGuardHandoff')
            self.request=r
            hold_manifest(self.folder/'background',r['finalization'],self.scope)
            for name,pin in r['files'].items():
                path=safe_path(self.folder,name);self.scope.enter_context(guard(path))
                if file_hash(path)!=pin:raise ValueError('AuditHandoffChangedArtifact',name)
            root=read(self.folder/'RESULT.json');direct=read(self.folder/'background/RESULT.json');audit=read(self.folder/'background/READBACK.json')
            if root.get('passed') is not True or root.get('full_mesh_complete') is not True or root.get('background')!=direct:
                raise ValueError('AuditHandoffIncompleteRoot')
            if direct.get('schema')!='V35V4DirectComplete/1' or direct.get('passed') is not True or direct.get('full_mesh_complete') is not True or direct.get('provisional') is not False or direct.get('audit_finalization')!=r['finalization']:
                raise ValueError('AuditHandoffIncompleteDirect')
            if audit.get('schema')!='V35DirectIndependentReadback/1' or audit.get('passed') is not True or audit.get('all_twelve_semantically_replayed') is not True or audit.get('manifest_sha256')!=r['finalization']['manifest_sha256']:
                raise ValueError('AuditHandoffColdReplayRequired')
            if direct.get('readback_sha256')!=r['files']['background/READBACK.json'] or direct.get('manifest_sha256')!=r['finalization']['manifest_sha256']:
                raise ValueError('AuditHandoffReadbackBinding')
            # A successful overlap is mandatory. An exited worker or an ACK
            # already present cannot be repaired by acquiring locks afterward.
            if worker.poll() is not None or (self.folder/ACK).exists():raise ValueError('AuditHandoffOverlapRequired')
            durable_json(self.folder/ACK,dict(schema=SCHEMA,request_sha256=self.request_pin,supervisor_pid=os.getpid()))
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
        durable_json(self.folder/'VERIFICATION.json',value)
        return value

    def close(self):self.scope.close()

def requires_handoff(folder):
    path=Path(folder)/'RESULT.json'
    if not path.is_file():return False
    value=read(path)
    return value.get('full_mesh_complete') is True and value.get('background',{}).get('schema')=='V35V4DirectComplete/1'
