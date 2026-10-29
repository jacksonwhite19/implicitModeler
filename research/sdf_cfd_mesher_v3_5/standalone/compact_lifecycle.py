"""Explicit new-run scratch ownership. Historical export has no cleanup API."""
from pathlib import Path
from contextlib import ExitStack
import datetime,json,os,subprocess,sys,time,uuid,shutil,hashlib
import psutil
from compact_paths import checked,hold_parents,inventory,file_id,Handle,allowed_parent
from compact_delivery import export,verify,sha,write,read,require


def receipt_path(delivery):
    delivery=Path(delivery)
    return delivery.with_name(delivery.name+'.cleanup-authorization.json')


def atomic_receipt(path,value):
    """Fresh immutable operational receipt; not part of the compact manifest."""
    path=checked(path,exists=False)
    require(not path.exists(),'FreshCleanupReceiptRequired')
    payload=(json.dumps(value,indent=2,allow_nan=False)+'\n').encode('utf-8')
    pending=path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
    with pending.open('xb') as f:
        f.write(payload);f.flush();os.fsync(f.fileno())
    with Handle(pending,delete=True) as h:h.rename_flushed(path,payload)
    return hashlib.sha256(payload).hexdigest()


class Scratch:
    def __init__(self,parent,*,required_audit=False):
        self.scope=ExitStack();self.parent=hold_parents(allowed_parent(parent),self.scope)
        self.root=self.parent/('.v35-compact-scratch-'+uuid.uuid4().hex)
        self.root.mkdir(exist_ok=False);self.initial_id=file_id(self.root);self.parent_id=file_id(self.parent)
        self.marker=dict(schema='V35NewCompactScratchOwner/1',nonce=uuid.uuid4().hex,
            root=str(self.root),file_identity=list(self.initial_id),pid=os.getpid(),birth=psutil.Process().create_time())
        write(self.root/'OWNER.json',self.marker)
        self.handle=Handle(self.root);self.scope.enter_context(self.handle)
        require(file_id(self.root)==self.initial_id,'ChangedNewScratchIdentity')
        self.observed={};self.authorized_here=True;self.required_audit=required_audit;self.accepted=None;self.cleanup_receipt=None
    def close(self):self.scope.close()
    def retire_native(self):
        # Only this live, fresh ownership capability permits recorded-group retirement.
        require(self.authorized_here and file_id(checked(self.root))==self.initial_id
            and read(self.root/'OWNER.json')==self.marker,'ChangedNewScratchOwnership')
        # The legacy exact-folder backend may exist without a scope receipt.
        from source_process_owner import launch
        code="from source_process_owner import worker_start_gate;worker_start_gate();from pathlib import Path;import sys,geometry;from standalone_background import stop_backend;stop_backend(Path(sys.argv[1]))"
        command=[sys.executable,'-B','-c',code,str(self.root/'bundle')]
        with launch(command,contained=True,popen=subprocess.Popen,cwd=Path(__file__).parent,
                stdout=subprocess.PIPE,stderr=subprocess.PIPE) as proc:
            try:
                stdout,stderr=proc.communicate(timeout=30)
                require(proc.returncode==0,'CompactExactBackendClosureFailed')
            except BaseException:
                proc._v35_source_job.retire();proc.wait();raise
        from build_native import linux
        command=['wsl','-d','Ubuntu-22.04','--','python3','-B',linux(Path(__file__).with_name('compact_native_close.py')),
            linux(self.root),'failure',self.marker['nonce']]
        done=subprocess.run(command,capture_output=True,text=True,timeout=30)
        value=json.loads(done.stdout)
        require(done.returncode==0 and value.get('passed') is True,'CompactNativeRetirementUnresolved')
        return value
    def authorize_cleanup(self,delivery,manifest,expected,*,audit_path=None,audit_expected=None):
        from compact_delivery import authenticate_source,audit_record
        require(not self.required_audit or (audit_path is not None and audit_expected is not None),'RequestedScratchAuditMissing')
        value=verify(delivery,manifest);source=self.root/'bundle'
        require(value['source_verification_sha256']==expected and read(Path(delivery)/'PROVENANCE.json')['source_bundle']==str(source),'UnrelatedScratchDelivery')
        with ExitStack() as scope:
            hold_parents(source,scope);producer=authenticate_source(source,expected,scope)
            audited=audit_record(audit_path,audit_expected,expected,scope)
        self.accepted=dict(delivery=str(checked(delivery)),manifest=manifest,source=expected,audit_path=audit_path,audit_expected=audit_expected,
            producer_status=producer['status'],producer_source_identity=producer['source_identity'],
            audit_status=audited['status'] if audited else 'not_requested',
            audit_source_root=audited['source_bundle_root'] if audited else None)
    def cleanup(self,delivery,manifest):
        with ExitStack() as scope:
            verify(delivery,manifest,retained_scope=scope)
            return self._cleanup(delivery,manifest,scope)
    def _cleanup(self,delivery,manifest,scope):
        require(self.accepted is not None and self.accepted['delivery']==str(checked(delivery)) and self.accepted['manifest']==manifest,'ScratchSuccessCutpointMissing')
        require(self.authorized_here and self.root.parent==self.parent and self.root!=self.parent
            and self.root.is_relative_to(self.parent) and self.root.name.startswith('.v35-compact-scratch-'),'NotNewOwnedScratch')
        checked(self.root);require(file_id(self.parent)==self.parent_id and file_id(self.root)==self.initial_id and read(self.root/'OWNER.json')==self.marker
            and self.marker['pid']==os.getpid() and self.marker['birth']==psutil.Process().create_time(),'ChangedNewScratchOwnership')
        delivery=checked(delivery);require(not delivery.is_relative_to(self.root),'DeliveryInsideScratch')
        require(verify(delivery,manifest)['source_verification_sha256']==self.accepted['source']
            and sha(self.root/'bundle/VERIFICATION.json')==self.accepted['source'],'ChangedScratchAcceptedRoot')
        # Every observed Windows descendant must be gone; unknown birth refuses.
        for pid,birth in self.observed.items():
            try:
                require(psutil.Process(pid).create_time()!=birth,'ScratchWorkerStillActive')
            except psutil.NoSuchProcess:pass
        # Upgrade only after success. Identity is rechecked on the pinned handle
        # before any deletion, so replacement in this interval cannot be adopted.
        self.handle.close();self.handle=Handle(self.root,delete=True);self.scope.enter_context(self.handle)
        require(file_id(self.root)==self.initial_id and read(self.root/'OWNER.json')==self.marker,'ReplacedScratchBeforeCleanup')
        paths=inventory(self.root)  # Reject all reparse points before first deletion.
        # Always inspect the exact owned root, including legacy zero-scope work.
        # Inspect mode cannot signal; unknown/live argv, cwd or PGID refuses.
        from build_native import linux
        done=subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3','-B',
            linux(Path(__file__).with_name('compact_native_close.py')),linux(self.root),'inspect',self.marker['nonce']],
            capture_output=True,text=True,timeout=30)
        require(done.returncode==0,'ScratchNativeClosureUnknownOrActive:'+done.stdout[-4000:]+done.stderr[-4000:])
        native=json.loads(done.stdout)
        require(native.get('passed') is True,'ScratchNativeClosureUnknownOrActive')
        # Persist the actual verified cutpoint before the first deletion. No
        # external ACK or scientific archive is needed or accepted as authority.
        paths=inventory(self.root)
        staged_bytes=sum(p.stat().st_size for p in paths)
        receipt=dict(schema='V35CompactCleanupAuthorization/1',status='precleanup_complete',
            prepared_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            producer=dict(verification_sha256=self.accepted['source'],status=self.accepted['producer_status'],source_identity=self.accepted['producer_source_identity']),
            immediate_audit=dict(requested=self.required_audit,status=self.accepted['audit_status'],receipt_sha256=self.accepted['audit_expected'],source_verification_sha256=self.accepted['audit_source_root']),
            compact=dict(path=str(delivery),manifest_sha256=manifest,status='solver_delivery_complete'),
            staging=dict(owner=self.marker,parent=str(self.parent),parent_identity=list(self.parent_id)),
            windows_census=dict(passed=True,observed_pid_birth_pairs=[dict(pid=pid,birth=birth) for pid,birth in sorted(self.observed.items())],all_observed_absent=True,scope='Observed owned Windows processes; each stage also closes its Windows Job'),
            native_census=native,staged_logical_bytes_before_cleanup=staged_bytes,staged_files_before_cleanup=len(paths),
            full_offline_replay_available_after_cleanup=False,
            removed_data='Full producer/replay operands and proof/native logs in owned scratch; compact reports are completed-check records only')
        path=receipt_path(delivery);require(not path.is_relative_to(self.root),'CleanupReceiptInsideScratch')
        pin=atomic_receipt(path,receipt)
        scope.enter_context(Handle(path))
        require(sha(path)==pin and read(path)==receipt,'ChangedCleanupAuthorizationReceipt')
        self.cleanup_receipt=dict(path=str(path),sha256=pin)
        # DELETE is set on the same no-reparse, no-replacement Windows handle.
        # No path-based recursive delete, rmtree, shell deletion or loaded marker.
        def remove_children(folder):
            for child in sorted(folder.iterdir()):
                checked(child)
                with Handle(child,delete=True) as h:
                    if child.is_dir():remove_children(child)
                    h.dispose()
        remove_children(self.root);self.handle.dispose();self.handle.close()
        require(not self.root.exists(),'ScratchCleanupIncomplete')
        self.authorized_here=False
        return dict(status='cleanup_complete',removed_root=str(self.root),authorization_receipt=self.cleanup_receipt,
            staged_logical_bytes_before_cleanup=staged_bytes,staged_files_before_cleanup=len(paths),remaining_scratch_logical_bytes=0)


def bounded(command,scratch,args,destination,deadline,label):
    """Own every child stage in a Windows Job, not a bare Popen context."""
    from source_process_owner import launch
    resource_check(scratch,args,destination,deadline)
    out=scratch.root/(label+'.stdout');err=scratch.root/(label+'.stderr')
    with out.open('x') as stdout,err.open('x') as stderr:
        with launch(command,contained=True,popen=subprocess.Popen,stdout=stdout,stderr=stderr) as proc:
            try:
                birth=psutil.Process(proc.pid).create_time()
                while proc.poll() is None:
                    try:
                        parent=psutil.Process(proc.pid)
                        require(parent.create_time()==birth,'CompactChildBirthChanged')
                        processes=[parent]+parent.children(recursive=True);rss=psutil.Process().memory_info().rss
                        for p in processes:
                            try:scratch.observed[p.pid]=p.create_time();rss+=p.memory_info().rss
                            except psutil.NoSuchProcess:pass
                    except psutil.NoSuchProcess:
                        if proc.poll() is not None:break
                        raise
                    resource_check(scratch,args,destination,deadline,rss)
                    time.sleep(.25)
                code=proc.wait()
                stdout.flush();stderr.flush()
                resource_check(scratch,args,destination,deadline)
                require(code==0,'CompactChildStageFailed:'+label)
            except BaseException:
                proc._v35_source_job.retire();proc.wait();raise


def resource_check(scratch,args,destination,deadline,rss=None):
    require(time.monotonic()<deadline,'CompactWholeInvocationTimeLimit')
    rss=psutil.Process().memory_info().rss if rss is None else rss
    require(rss<=args.memory_limit_gib*2**30,'CompactWindowsTreeRSSLimit')
    total=sum(live_bytes(root) for root in (scratch.root,destination) if root.exists())
    require(total<=(args.output_limit_bytes or 4_000_000_000),'CompactInvocationOutputLimit')
    require(shutil.disk_usage(scratch.root).free>=1_000_000_000,'CompactMinimumFreeDisk')


def live_bytes(root):
    """Checkpoint rename races are ordinary; reparse redirects are refusals."""
    total=0
    for directory,dirs,files in os.walk(root,followlinks=False):
        for name in list(dirs):
            try:checked(Path(directory)/name)
            except FileNotFoundError:dirs.remove(name)
        for name in files:
            try:total+=checked(Path(directory)/name).stat().st_size
            except FileNotFoundError:pass
    return total


def child_arguments(argv,output):
    result=[];i=0
    valued={'--retention','--output'};flags={'--cleanup-scratch','--immediate-audit'}
    while i<len(argv):
        item=argv[i]
        if item in valued:
            require(i+1<len(argv),'MissingRetentionArgument');i+=2;continue
        if any(item.startswith(k+'=') for k in valued) or item in flags:i+=1;continue
        require(item!='--worker','CompactWorkerCannotOwnLifecycle')
        result.append(item);i+=1
    return result+['--output',str(output),'--retention','full-evidence']


def run(args,argv):
    destination=checked(args.output,exists=False)
    allowed_parent(destination.parent)
    require(not destination.exists(),'FreshCompactDestinationRequired')
    require(args.stop_after=='complete','CompactRequiresCompleteMesh')
    require(not args.immediate_audit or args.verification=='production','ImmediateAuditRequiresProduction')
    status=destination.parent/(destination.name+'.retention.json')
    checked(status,exists=False);require(not status.exists(),'FreshRetentionStatusRequired')
    if args.cleanup_scratch:
        reserved=checked(receipt_path(destination),exists=False);require(not reserved.exists(),'FreshCleanupReceiptRequired')
    scratch=Scratch(destination.parent,required_audit=args.immediate_audit);deadline=time.monotonic()+args.timeout_seconds
    record=dict(schema='V35CompactLifecycle/1',status='running',output=str(destination),scratch=str(scratch.root),
        retention='compact-delivery',cleanup_requested=args.cleanup_scratch,verification=args.verification,
        immediate_audit_requested=args.immediate_audit,scientific_status='not_accepted',separate_audit='not_run',compact_publication='not_run',cleanup='not_run')
    write(status,record)
    def save():
        # Only this newly created sidecar is replaced; no bundle is edited.
        checked(status);tmp=status.with_name(status.name+'.'+uuid.uuid4().hex+'.tmp');write(tmp,record);os.replace(tmp,status)
    try:
        command=[sys.executable,'-B',str(Path(__file__).with_name('mesh.py')),*child_arguments(argv,scratch.root/'bundle')]
        bounded(command,scratch,args,destination,deadline,'mesh')
        bundle=scratch.root/'bundle';expected=sha(bundle/'VERIFICATION.json')
        from compact_delivery import authenticate_source
        from compact_paths import hold_tree
        with ExitStack() as scope:
            hold_tree(bundle,scope);accepted=authenticate_source(bundle,expected,scope)
        record.update(scientific_status='accepted',source_root=expected,full_mesh_acceptance=accepted['status'])
        save()
        audit_path=audit_pin=None
        if args.immediate_audit:
            import offline_audit
            target=scratch.root/'offline_audit';record['separate_audit']='running';save()
            bounded([sys.executable,'-B',str(Path(__file__).with_name('offline_audit.py')),'--bundle',str(bundle),'--expected-root',expected,
                '--verifier-identity',offline_audit.verifier_identity(target),'--output',str(target),'--timeout-seconds',str(max(.001,deadline-time.monotonic()))],scratch,args,destination,deadline,'audit')
            audit_path=target/'AUDIT_RESULT.json';audit_pin=sha(audit_path)
            from compact_delivery import audit_record
            with ExitStack() as scope:audit_record(audit_path,audit_pin,expected,scope)
            record['separate_audit']='passed';save()
        command=[sys.executable,'-B',str(Path(__file__).with_name('compact_delivery.py')),'export','--bundle',str(bundle),'--expected-root',expected,'--output',str(destination)]
        if audit_path is not None:command+=['--audit-result',str(audit_path),'--expected-audit-root',audit_pin]
        record['compact_publication']='running';save()
        bounded(command,scratch,args,destination,deadline,'export')
        delivered=read(scratch.root/'export.stdout');delivered['original_mesh_invocation']=[sys.executable,'mesh.py',*argv]
        record.update(scientific_status='accepted',source_root=expected,compact_publication='verified',delivery=delivered,status='delivery_complete')
        save()
        if args.cleanup_scratch:
            try:
                scratch.authorize_cleanup(destination,delivered['manifest_sha256'],expected,audit_path=audit_path,audit_expected=audit_pin)
                record['cleanup']=scratch.cleanup(destination,delivered['manifest_sha256'])
            except BaseException as error:
                record['cleanup']=dict(status='cleanup_failed_or_partial',error=repr(error),remaining_root=str(scratch.root),authorization_receipt=scratch.cleanup_receipt)
                record['status']='delivery_complete_cleanup_incomplete'
        else:record['cleanup']=dict(status='full_scratch_retained',root=str(scratch.root))
        save();return 0 if record['status']=='delivery_complete' else 2
    except BaseException as error:
        record.update(status='failed_scratch_preserved' if scratch.root.exists() else 'failed_after_scratch_removed',error=repr(error))
        if record['separate_audit']=='running':record['separate_audit']='failed_or_incomplete'
        if record['compact_publication']=='running':record['compact_publication']='failed_or_incomplete'
        try:record['failure_native_closure']=scratch.retire_native()
        except BaseException as close_error:record['failure_native_closure']=dict(status='unresolved',error=repr(close_error))
        save();raise
    finally:scratch.close()
