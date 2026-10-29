"""Real process/guard handoff and publication failures, no meshing.

The helper stands in for a completed worker only to test the closure protocol;
its synthetic cold receipt is not scientific qualification or a bypass CLI.
"""
from pathlib import Path
from contextlib import ExitStack
from types import SimpleNamespace
import json,os,shutil,subprocess,sys,tempfile,time,unittest,uuid
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(Path(__file__).parent)]
import geometry
import test_artifact_finalization as controls
from audit_completion import SupervisorHandoff,REQUEST,ACK
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import file_hash

HELPER='''from pathlib import Path
from contextlib import ExitStack
import sys,time
sys.path.insert(0,sys.argv[1])
import geometry
from audit_completion import worker_handoff,read
from artifact_finalization import hold_manifest
folder=Path(sys.argv[2]);mode=sys.argv[3]
if mode=='timeout':time.sleep(30)
if mode=='exit_before_handoff':raise SystemExit(1)
if mode=='source_added':Path(sys.argv[4]).write_text('{"test":"source_added"}')
descriptor=read(folder/'background/RESULT.json')['audit_finalization']
with ExitStack() as scope:
 hold_manifest(folder/'background',descriptor,scope)
 worker_handoff(folder,descriptor,time.monotonic()+20,scope)
 if mode=='exit_after_ack':raise SystemExit(1)
'''

class AuditCompletion(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.base,_,_,cls.descriptor,_=controls.export_control()

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.addCleanup(self.tmp.cleanup)
        self.helper=self.root/'worker.py';self.helper.write_text(HELPER)

    def prepare(self,folder):
        folder=Path(folder);shutil.copytree(self.base,folder/'background')
        durable_json(folder/'CASE.json',dict(test='protocol only, not a geometry acceptance test'))
        audit=dict(schema='V35DirectIndependentReadback/1',passed=True,all_twelve_semantically_replayed=True,manifest_sha256=self.descriptor['manifest_sha256'])
        durable_json(folder/'background/READBACK.json',audit)
        direct=dict(schema='V35V4DirectComplete/1',passed=True,full_mesh_complete=True,provisional=False,audit_finalization=self.descriptor,
            manifest_sha256=self.descriptor['manifest_sha256'],readback_sha256=file_hash(folder/'background/READBACK.json'))
        durable_json(folder/'background/RESULT.json',direct)
        durable_json(folder/'RESULT.json',dict(passed=True,full_mesh_complete=True,background=direct))

    def acquire(self):
        folder=self.root/'case';self.prepare(folder)
        proc=subprocess.Popen([sys.executable,'-B',str(self.helper),str(ROOT),str(folder),'normal'])
        self.addCleanup(lambda:proc.kill() if proc.poll() is None else None)
        deadline=time.monotonic()+15
        while not (folder/REQUEST).exists():
            if proc.poll() is not None or time.monotonic()>deadline:self.fail('helper did not publish handoff')
            time.sleep(.02)
        handoff=SupervisorHandoff(folder,proc);self.addCleanup(handoff.close)
        self.assertEqual(proc.wait(timeout=15),0)
        return folder,handoff

    def closure(self,folder):
        sources=dict(identity='test-runtime',state='closed',passed=True,changed=[],added=[])
        supervisor=dict(exit_code=0,stop_reason=None,monitor_error=None,source_identity=sources['identity'])
        source=self.root/'source.json';record=self.root/'supervisor.json'
        durable_json(source,sources);durable_json(record,supervisor)
        return sources,supervisor,source,record

    def test_live_overlapping_guards_and_atomic_audit_only_manifest(self):
        folder,handoff=self.acquire()
        # Worker is gone; parent ownership must still deny replacement.
        with self.assertRaises(PermissionError):(folder/'background/COMPLETE_MESH.npz').write_bytes(b'late')
        result=handoff.publish(*self.closure(folder))
        self.assertEqual(result['status'],'audit_accepted');self.assertFalse(result['production_bypass'])
        self.assertEqual(result['independent_semantic_replay'],'passed')
        self.assertEqual(json.loads((folder/'VERIFICATION.json').read_text()),result)

    def test_unsuccessful_source_and_resource_closure_cannot_publish(self):
        folder,handoff=self.acquire();sources,supervisor,source,record=self.closure(folder)
        for bad in (dict(sources,passed=False),dict(sources,state='held'),dict(sources,added=['extra.py'])):
            with self.assertRaisesRegex(ValueError,'UnsuccessfulSourceClosure'):handoff.publish(bad,supervisor,source,record)
        for bad in (dict(supervisor,exit_code=1),dict(supervisor,stop_reason='TimeLimit'),dict(supervisor,monitor_error={'error':'test'})):
            with self.assertRaisesRegex(ValueError,'UnsuccessfulSupervisor'):handoff.publish(sources,bad,source,record)
        self.assertFalse((folder/'VERIFICATION.json').exists())

    def test_failed_atomic_publication_leaves_no_accepted_manifest(self):
        folder,handoff=self.acquire();args=self.closure(folder)
        with patch('audit_completion.durable_json',side_effect=OSError('before atomic acceptance commit')):
            with self.assertRaises(OSError):handoff.publish(*args)
        self.assertFalse((folder/'VERIFICATION.json').exists())

    def test_exited_worker_cannot_repair_missing_overlap(self):
        folder=self.root/'case';self.prepare(folder)
        proc=subprocess.Popen([sys.executable,'-c','pass']);proc.wait()
        with self.assertRaisesRegex(ValueError,'WorkerAlreadyExited'):SupervisorHandoff(folder,proc)

    def supervise_fixture(self,mode,seconds=20):
        import mesh
        folder=self.root/'supervised';added=ROOT/('phase1a_added_source_'+uuid.uuid4().hex+'.json')
        args=SimpleNamespace(output=folder,timeout_seconds=seconds,memory_limit_gib=16,output_limit_bytes=1000000000)
        original=subprocess.Popen
        def launch(_command,**kwargs):
            self.prepare(folder)
            return original([sys.executable,'-B',str(self.helper),str(ROOT),str(folder),mode,str(added)],**kwargs)
        try:
            with patch('mesh.subprocess',SimpleNamespace(Popen=launch,run=subprocess.run)):code=mesh.supervise(args)
            return code,json.loads((folder/'RESULT.json').read_text()),(folder/'VERIFICATION.json').exists()
        finally:
            if added.exists():added.unlink()  # This test's single new file only.

    def test_real_supervisor_source_addition_revokes_without_publication(self):
        code,result,published=self.supervise_fixture('source_added')
        self.assertNotEqual(code,0);self.assertFalse(result['passed']);self.assertFalse(published)
        self.assertEqual(result['stop_reason'],'SourceClosureChanged')

    def test_real_supervisor_timeout_revokes_without_publication(self):
        code,result,published=self.supervise_fixture('timeout',3)
        self.assertNotEqual(code,0);self.assertFalse(result['passed']);self.assertFalse(published)
        self.assertEqual(result['stop_reason'],'TimeLimit')

    def test_real_supervisor_nonzero_exit_after_ack_revokes_legacy_success(self):
        code,result,published=self.supervise_fixture('exit_after_ack')
        self.assertNotEqual(code,0);self.assertFalse(result['passed']);self.assertFalse(result['full_mesh_complete']);self.assertFalse(published)
        self.assertEqual(result['stop_reason'],'AuditFinalizedWorkerFailed')

    def test_real_supervisor_nonzero_exit_before_handoff_revokes_legacy_success(self):
        code,result,published=self.supervise_fixture('exit_before_handoff')
        self.assertNotEqual(code,0);self.assertFalse(result['passed']);self.assertFalse(result['full_mesh_complete']);self.assertFalse(published)
        self.assertEqual(result['stop_reason'],'WorkerFailed')

    def test_real_supervisor_clean_closure_publishes_audit_only(self):
        code,result,published=self.supervise_fixture('normal')
        self.assertEqual(code,0);self.assertTrue(result['passed']);self.assertTrue(published)

if __name__=='__main__':unittest.main()
