"""Storage/lifecycle tests, not SDF/mesh qualification.

Small real serialized/native-export and child guard/publication fixtures reuse
the existing declared synthetic twelve-proof/source premises. No auth boolean
is mocked to pass; faults act on the actual accepted root and copied bytes.
"""
from pathlib import Path
from contextlib import ExitStack
import json,os,shutil,subprocess,sys,tempfile,unittest
from unittest.mock import patch
HERE=Path(__file__).resolve().parents[1];sys.path[:0]=[str(HERE),str(Path(__file__).parent)]
import geometry
import compact_delivery as delivery
from compact_lifecycle import Scratch,child_arguments
from compact_paths import checked,Handle,member
import test_verification_policy as base
from path_preserving_deduplication import durable_json as save


class CompactDelivery(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.addCleanup(self.tmp.cleanup)
    def fixture(self,destination=None):
        owner=base.VerificationPolicy();owner.setUp();self.addCleanup(owner.doCleanups)
        def create():
            root=destination or owner.root/'bundle';root.mkdir()
            joined=base.controls.export_control()[1]
            counts=dict(points=len(joined['points']),prism_cells=len(joined['prefix_ids']),core_cells=len(joined['bulk_tets']))
            d,args=base.production_fixture(root,case_extra=dict(geometry=dict(rhai='// synthetic storage fixture\nsphere(1);\n',metres_per_model_unit=.001)),
                result_extra=dict(**counts,conditioning=dict(outcome='AlreadyAboveFloor'),repair=dict(classification='UNQUALIFIED',changed_vertices=[])),
                root_extra=dict(requested=dict(first_height_m=.00002,layers=1,growth=1.3),achieved=dict(compressed_vertices=0)),
                repair_report=dict(classification='UNQUALIFIED',claim_scope='Synthetic source-float only; no exact-Q population claim',scientific_acceptance=False))
            return root,d,args
        owner.fixture=create
        root,handoff,args=owner.acquire()
        sources,supervisor,source,record=args
        supervisor.update(command=['synthetic-storage-fixture'],source_custody_passed=True)
        save(record,supervisor);value=handoff.publish(*args);handoff.close()
        return root,delivery.sha(root/'VERIFICATION.json')
    def test_real_export_and_full_recursive_native_payload(self):
        root,pin=self.fixture();out=self.root/'compact';before=delivery.sha(root/'VERIFICATION.json')
        result=delivery.export(root,pin,out);value=delivery.verify(out,result['manifest_sha256'])
        expected={p.relative_to(root/'background/export/openfoam_case/constant/polyMesh').as_posix() for p in delivery.inventory(root/'background/export/openfoam_case/constant/polyMesh')}
        self.assertEqual(set(value['solver_mesh_files']),expected)
        for n in expected:self.assertEqual((out/'polyMesh'/n).read_bytes(),(root/'background/export/openfoam_case/constant/polyMesh'/n).read_bytes())
        self.assertIn('sets/fluid',expected);self.assertIn('cellZones',expected)
        self.assertEqual(delivery.read(out/'CHECKS.json')['repair_assurance']['classification'],'UNQUALIFIED')
        self.assertFalse(value['full_offline_replay_available']);self.assertEqual(before,delivery.sha(root/'VERIFICATION.json'))
    def test_wrong_outer_and_inner_roots_and_changed_summary_refuse(self):
        root,pin=self.fixture()
        with self.assertRaises(ValueError):delivery.export(root,'0'*64,self.root/'wrong')
        for name in ['background/MANIFEST.json','RESULT.json']:
            p=root/name;before=p.read_bytes();p.write_bytes(before+b' ')
            with self.assertRaises(ValueError):delivery.export(root,pin,self.root/'bad')
            p.write_bytes(before)
        self.assertFalse((self.root/'bad').exists())
    def test_unpinned_poly_mesh_extra_and_missing_zone_refuse(self):
        root,pin=self.fixture();poly=root/'background/export/openfoam_case/constant/polyMesh'
        (poly/'extra').write_text('unbound')
        with self.assertRaises(ValueError):delivery.export(root,pin,self.root/'bad')
        (poly/'extra').unlink();p=poly/'cellZones';old=p.read_bytes();p.unlink()
        with self.assertRaises((ValueError,FileNotFoundError)):delivery.export(root,pin,self.root/'bad')
        p.write_bytes(old)
    def test_fresh_destination_and_overlap(self):
        root,pin=self.fixture();out=self.root/'existing';out.mkdir();(out/'keep').write_text('user')
        with self.assertRaises(ValueError):delivery.export(root,pin,out)
        with self.assertRaises(ValueError):delivery.export(root,pin,root/'child')
        self.assertEqual((out/'keep').read_text(),'user')
    def test_partial_copy_and_commit_failure_preserve_source(self):
        root,pin=self.fixture()
        for target in ['copy_payload','publish']:
            out=self.root/target
            with patch.object(delivery,target,side_effect=OSError('injected interruption')):
                with self.assertRaises(OSError):delivery.export(root,pin,out)
            self.assertFalse((out/'COMPACT.json').exists());self.assertEqual(delivery.sha(root/'VERIFICATION.json'),pin)
            with self.assertRaises((ValueError,FileNotFoundError)):delivery.verify(out)
    def test_integrity_tamper_extra_unknown_schema_and_missing_metadata(self):
        root,pin=self.fixture();out=self.root/'compact';result=delivery.export(root,pin,out)
        p=out/'CHECKS.json';old=p.read_bytes();p.write_bytes(old+b' ')
        with self.assertRaises(ValueError):delivery.verify(out,result['manifest_sha256'])
        p.write_bytes(old);(out/'extra').write_text('x')
        with self.assertRaises(ValueError):delivery.verify(out,result['manifest_sha256'])
        (out/'extra').unlink();v=delivery.read(out/'COMPACT.json');v['schema']='unknown';save(out/'COMPACT.json',v)
        with self.assertRaises(ValueError):delivery.verify(out)
    def test_offline_explicitly_refuses_compact_without_creating_output(self):
        from offline_audit import authenticate,audit
        root,pin=self.fixture();out=self.root/'compact';result=delivery.export(root,pin,out)
        with ExitStack() as scope:
            with self.assertRaisesRegex(ValueError,'NoFullOfflineReplay'):authenticate(out,result['manifest_sha256'],scope)
        with self.assertRaisesRegex(ValueError,'NoFullOfflineReplay'):audit(out,result['manifest_sha256'],'not-used',self.root/'audit')
        self.assertFalse((self.root/'audit').exists())
    def test_reparse_and_traversal_refusal(self):
        with self.assertRaises(ValueError):checked(self.root/'..'/'escape')
        for n in ['../x','/x','a/../x','C:/x','a\\x','NUL','a.','a ']:
            with self.assertRaises(ValueError):member(self.root,n)
        target=self.root/'target';target.mkdir();link=self.root/'junction'
        result=subprocess.run(['cmd','/c','mklink','/J',str(link),str(target)],capture_output=True)
        self.assertEqual(result.returncode,0)
        with self.assertRaises(ValueError):checked(link)
        os.rmdir(link)
    def test_handle_blocks_replacement(self):
        p=self.root/'locked';p.mkdir()
        with Handle(p):
            with self.assertRaises(OSError):p.rename(self.root/'moved')
    def test_new_scratch_cleanup_keeps_verified_delivery(self):
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        root,pin=self.fixture(scratch.root/'bundle');out=self.root/'compact';result=delivery.export(root,pin,out)
        scratch.authorize_cleanup(out,result['manifest_sha256'],pin)
        (scratch.root/'nested').mkdir();(scratch.root/'nested'/'x').write_text('owned temporary')
        path=scratch.root;r=scratch.cleanup(out,result['manifest_sha256'])
        self.assertEqual(r['status'],'cleanup_complete');self.assertFalse(path.exists())
        delivery.verify(out,result['manifest_sha256']);self.assertFalse(root.exists())
    def test_cleanup_missing_publication_mutated_marker_and_active_process_refuse(self):
        import psutil
        scratch=Scratch(self.root);self.addCleanup(scratch.close);(scratch.root/'keep').write_text('diagnostic')
        root,pin=self.fixture(scratch.root/'bundle');out=self.root/'compact';result=delivery.export(root,pin,out)
        with self.assertRaises(ValueError):scratch.cleanup(out,result['manifest_sha256'])
        scratch.authorize_cleanup(out,result['manifest_sha256'],pin)
        with self.assertRaises((ValueError,FileNotFoundError)):scratch.cleanup(self.root/'absent','0'*64)
        (scratch.root/'OWNER.json').write_text(json.dumps(dict(scratch.marker,nonce='wrong')))
        with self.assertRaises(ValueError):scratch.cleanup(out,result['manifest_sha256'])
        (scratch.root/'OWNER.json').write_text(json.dumps(scratch.marker));scratch.observed[os.getpid()]=psutil.Process().create_time()
        with self.assertRaisesRegex(ValueError,'StillActive'):scratch.cleanup(out,result['manifest_sha256'])
        self.assertTrue((scratch.root/'keep').exists());delivery.verify(out,result['manifest_sha256'])
    def test_unrelated_delivery_and_missing_requested_audit_cannot_authorize_cleanup(self):
        root,pin=self.fixture();out=self.root/'compact';result=delivery.export(root,pin,out)
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        with self.assertRaisesRegex(ValueError,'Unrelated'):scratch.authorize_cleanup(out,result['manifest_sha256'],pin)
        requested=Scratch(self.root,required_audit=True);self.addCleanup(requested.close)
        with self.assertRaisesRegex(ValueError,'AuditMissing'):requested.authorize_cleanup(out,result['manifest_sha256'],pin)
        self.assertTrue(root.exists());self.assertTrue(scratch.root.exists())
    def test_mid_copy_source_addition_and_stage_injection_refuse(self):
        root,pin=self.fixture();copy=delivery.copy_payload;poly=root/'background/export/openfoam_case/constant/polyMesh'
        done=False
        def added(source,target):
            nonlocal done
            copy(source,target)
            if not done:(poly/'unbound-extra').write_text('extra');done=True
        with patch.object(delivery,'copy_payload',side_effect=added):
            with self.assertRaises(ValueError):delivery.export(root,pin,self.root/'added')
        self.assertFalse((self.root/'added/COMPACT.json').exists());(poly/'unbound-extra').unlink()
        done=False
        def injected(source,target):
            nonlocal done
            copy(source,target)
            if not done:(self.root/'injected'/'unbound').write_text('extra');done=True
        with patch.object(delivery,'copy_payload',side_effect=injected):
            with self.assertRaises(ValueError):delivery.export(root,pin,self.root/'injected')
        self.assertFalse((self.root/'injected/COMPACT.json').exists())
    def test_flush_failure_never_commits_success_marker(self):
        from compact_paths import publish
        out=self.root/'incomplete';out.mkdir()
        with patch('compact_paths.os.fsync',side_effect=OSError('injected flush failure')):
            with self.assertRaises(OSError):publish(out,dict(schema=delivery.SCHEMA,status='solver_delivery_complete'))
        self.assertFalse((out/'COMPACT.json').exists())
    def test_delivery_guards_cover_native_census_and_cleanup_failure(self):
        from types import SimpleNamespace
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        root,pin=self.fixture(scratch.root/'bundle');out=self.root/'compact';result=delivery.export(root,pin,out)
        scratch.authorize_cleanup(out,result['manifest_sha256'],pin)
        # Zero scope receipts must still invoke read-only exact-root census.
        denied=[]
        def census(*args,**kwargs):
            self.assertIn('inspect',args[0])
            try:(out/'polyMesh/points').write_text('tampered')
            except PermissionError:denied.append(True)
            return SimpleNamespace(returncode=1,stdout="",stderr="injected native census refusal")
        with patch('compact_lifecycle.subprocess.run',side_effect=census):
            with self.assertRaisesRegex(ValueError,'NativeClosure'):scratch.cleanup(out,result['manifest_sha256'])
        self.assertEqual(denied,[True]);self.assertTrue(root.exists());delivery.verify(out,result['manifest_sha256'])
    def test_bounded_child_time_limit_retires_job_and_preserves_scratch(self):
        from compact_lifecycle import bounded
        from types import SimpleNamespace
        import time,psutil
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        command=[sys.executable,'-B','-c',f"import sys;sys.path.insert(0,{str(HERE)!r});from source_process_owner import worker_start_gate;worker_start_gate();import time;time.sleep(30)"]
        args=SimpleNamespace(memory_limit_gib=16,output_limit_bytes=100000000)
        with self.assertRaisesRegex(ValueError,'TimeLimit'):bounded(command,scratch,args,self.root/'dest',time.monotonic()+.5,'timeout')
        self.assertTrue(scratch.root.exists())
        for pid,birth in scratch.observed.items():
            try:self.assertNotEqual(psutil.Process(pid).create_time(),birth)
            except psutil.NoSuchProcess:pass
    def test_audit_and_export_children_gate_before_imports(self):
        for name in ['offline_audit.py','compact_delivery.py']:
            env=dict(os.environ,V35_SOURCE_START_EVENT='Local\\MissingCompactTestEvent',V35_SOURCE_SUPERVISOR_PID=str(os.getpid()))
            r=subprocess.run([sys.executable,'-B',str(HERE/name),'--help'],env=env,capture_output=True,text=True,timeout=10)
            self.assertNotEqual(r.returncode,0);self.assertIn('SourceSupervisorStartGateMissing',r.stderr)
    def test_retained_audit_authenticates_external_closure_and_refuses_missing(self):
        import test_audit_completion as audit
        audit.AuditCompletion.setUpClass();owner=audit.AuditCompletion();owner.setUp();self.addCleanup(owner.doCleanups)
        root,handoff=owner.acquire();sources,supervisor,sp,rp=owner.closure(root)
        supervisor.update(source_custody_passed=True,command=['synthetic-audit-fixture']);save(rp,supervisor)
        handoff.publish(sources,supervisor,sp,rp);handoff.close();pin=delivery.sha(root/'VERIFICATION.json')
        with ExitStack() as scope:self.assertEqual(delivery.authenticate_source(root,pin,scope)['status'],'audit_accepted')
        rp.unlink()
        with ExitStack() as scope:
            with self.assertRaises(FileNotFoundError):delivery.authenticate_source(root,pin,scope)
    def test_scratch_root_replacement_and_nested_reparse_refuse(self):
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        with self.assertRaises(OSError):scratch.root.rename(self.root/'replaced')
        target=self.root/'outside';target.mkdir();(target/'historical').write_text('keep')
        link=scratch.root/'junction';r=subprocess.run(['cmd','/c','mklink','/J',str(link),str(target)],capture_output=True);self.assertEqual(r.returncode,0)
        with self.assertRaises(ValueError):delivery.inventory(scratch.root)
        self.assertEqual((target/'historical').read_text(),'keep');os.rmdir(link)
    def test_generated_metadata_mutation_before_guard_refuses(self):
        root,pin=self.fixture();original=delivery.write
        def mutate(path,value):
            expected=original(path,value)
            if Path(path).name=='CHECKS.json':Path(path).write_bytes(Path(path).read_bytes()+b' ')
            return expected
        out=self.root/'mutated'
        with patch.object(delivery,'write',side_effect=mutate):
            with self.assertRaisesRegex(ValueError,'GeneratedCompactMetadata'):delivery.export(root,pin,out)
        self.assertFalse((out/'COMPACT.json').exists());self.assertEqual(delivery.sha(root/'VERIFICATION.json'),pin)
    def test_manifest_mutation_after_commit_cannot_become_its_own_pin(self):
        root,pin=self.fixture();original=delivery.publish;out=self.root/'changed-manifest'
        def mutate(folder,value):
            expected=original(folder,value)
            path=folder/'COMPACT.json';path.write_bytes(path.read_bytes()+b' ')
            return expected
        with patch.object(delivery,'publish',side_effect=mutate):
            with self.assertRaisesRegex(ValueError,'ChangedCompactDeliveryRoot'):delivery.export(root,pin,out)
        self.assertEqual(delivery.sha(root/'VERIFICATION.json'),pin)
    def test_commit_rename_exact_utf16_names_all_alignment_lengths(self):
        from compact_paths import publish
        for name in ['a','ab','abc','abcd','long-output-folder','unicode-\u03bb-\U0001f642']:
            folder=self.root/name;folder.mkdir()
            value=dict(schema='alignment-control',name=name)
            pin=publish(folder,value)
            self.assertEqual({p.name for p in folder.iterdir()},{'COMPACT.json'})
            self.assertEqual(delivery.read(folder/'COMPACT.json'),value)
            self.assertEqual(delivery.sha(folder/'COMPACT.json'),pin)
            with self.assertRaises(OSError):publish(folder,value)
    def test_runtime_parent_refused_before_creation(self):
        from compact_lifecycle import allowed_parent
        with self.assertRaisesRegex(ValueError,'InsideRuntime'):allowed_parent(HERE/'tests')
        self.assertEqual(allowed_parent(self.root),self.root)
        root,pin=self.fixture()
        with self.assertRaisesRegex(ValueError,'InsideRuntime'):delivery.export(root,pin,HERE/'tests'/'never-create-compact')
    def test_separate_audit_valid_wrong_root_and_failed(self):
        # Declared synthetic semantic premises; authentic receipt/closed-source binding.
        target=self.root/'audit';target.mkdir();root='a'*64
        save(target/'VERIFIER_SOURCES.json',dict(state='closed',passed=True,changed=[],added=[],identity='synthetic-verifier'))
        value=dict(schema='V35RetainedDirectAudit/1',status='audit_passed',passed=True,source_bundle_root=root,
            fresh_source_authority=False,remeshed=False,search_rerun=False,verifier_identity='synthetic-verifier',
            replay=dict(all_twelve_semantically_replayed=True,proofs=12))
        path=target/'AUDIT_RESULT.json';save(path,value)
        with ExitStack() as scope:self.assertTrue(delivery.audit_record(path,delivery.sha(path),root,scope)['passed'])
        with ExitStack() as scope:
            with self.assertRaisesRegex(ValueError,'InvalidCompactSeparateAudit'):delivery.audit_record(path,delivery.sha(path),'b'*64,scope)
        value['passed']=False;save(path,value)
        with ExitStack() as scope:
            with self.assertRaisesRegex(ValueError,'InvalidCompactSeparateAudit'):delivery.audit_record(path,delivery.sha(path),root,scope)
    def test_prelaunch_and_closed_stage_resource_limits(self):
        from compact_lifecycle import bounded
        from types import SimpleNamespace
        import time
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        args=SimpleNamespace(memory_limit_gib=16,output_limit_bytes=1000)
        with patch('source_process_owner.launch') as launch:
            with self.assertRaisesRegex(ValueError,'TimeLimit'):bounded([],scratch,args,self.root/'dest',time.monotonic()-1,'expired')
            launch.assert_not_called()
        (scratch.root/'overflow').write_bytes(b'x'*2000)
        with patch('source_process_owner.launch') as launch:
            with self.assertRaisesRegex(ValueError,'OutputLimit'):bounded([],scratch,args,self.root/'dest',time.monotonic()+30,'preoverflow')
            launch.assert_not_called()
        (scratch.root/'overflow').unlink()
        command=[sys.executable,'-B','-c',f"import sys;sys.path.insert(0,{str(HERE)!r});from source_process_owner import worker_start_gate;worker_start_gate();print('x'*2000)"]
        with self.assertRaisesRegex(ValueError,'OutputLimit'):bounded(command,scratch,args,self.root/'dest',time.monotonic()+30,'finaloverflow')
    def test_accepted_mesh_export_failure_keeps_science_status(self):
        from compact_lifecycle import run
        from types import SimpleNamespace
        destination=self.root/'delivery';args=SimpleNamespace(output=destination,stop_after='complete',immediate_audit=False,
            verification='production',timeout_seconds=60,cleanup_scratch=False)
        # Stage dispatch is mocked; acceptance itself reads a real serialized accepted fixture.
        def stage(command,scratch,args,destination,deadline,label):
            if label=='mesh':self.fixture(scratch.root/'bundle')
            else:raise OSError('injected storage failure')
        with patch('compact_lifecycle.bounded',side_effect=stage),patch.object(Scratch,'retire_native',return_value=dict(status='synthetic_closed')):
            with self.assertRaisesRegex(OSError,'storage failure'):run(args,[])
        value=delivery.read(self.root/'delivery.retention.json')
        self.assertEqual(value['scientific_status'],'accepted');self.assertEqual(value['compact_publication'],'failed_or_incomplete')
        self.assertTrue(Path(value['scratch']).exists());self.assertIn('failure_native_closure',value)
    def test_native_retirement_requires_live_marker_and_exact_scope(self):
        from types import SimpleNamespace
        scratch=Scratch(self.root);self.addCleanup(scratch.close)
        def closed(command,**kwargs):
            self.assertIn(str(scratch.marker['nonce']),command)
            self.assertTrue(any(scratch.root.name in a for a in command))
            return SimpleNamespace(returncode=0,stdout=json.dumps(dict(passed=True,remaining=[])))
        with patch('compact_lifecycle.subprocess.run',side_effect=closed):self.assertTrue(scratch.retire_native()['passed'])
        with patch('compact_lifecycle.subprocess.run',return_value=SimpleNamespace(returncode=1,stdout=json.dumps(dict(passed=False,remaining=[dict(argv=['legacy-backend'])])))):
            with self.assertRaisesRegex(ValueError,'NativeRetirementUnresolved'):scratch.retire_native()
        (scratch.root/'OWNER.json').write_text('{}')
        with patch('compact_lifecycle.subprocess.run') as launch:
            with self.assertRaisesRegex(ValueError,'Ownership'):scratch.retire_native()
            launch.assert_not_called()
    def test_cli_retention_arguments_independent_and_legacy_default(self):
        argv=['--output=dest','--retention','compact-delivery','--verification','production','--source-workers','4','--cleanup-scratch','--immediate-audit']
        child=child_arguments(argv,self.root/'bundle')
        self.assertEqual(child,['--verification','production','--source-workers','4','--output',str(self.root/'bundle'),'--retention','full-evidence'])
        import mesh
        args=['mesh.py','--case','x','--geometry','y','--metres-per-model-unit','.001','--prism-first-height','.00002','--prism-growth','1.3','--prism-layers','5','--output','z']
        with patch.object(sys,'argv',args),patch.object(mesh,'supervise',return_value=17) as f:
            self.assertEqual(mesh.main(),17);self.assertEqual(f.call_args.args[0].retention,'full-evidence');self.assertEqual(f.call_args.args[0].source_workers,1)

if __name__=='__main__':unittest.main()
