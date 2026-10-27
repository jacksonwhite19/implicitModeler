"""Supervisor fault injections; no actual worker or candidate is launched."""
from pathlib import Path
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import patch
import tempfile,unittest,json,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import retained_core as cli
import source_custody,psutil

class FakeProcess:
    pid=987654321
    def __init__(self,code=None):self.code=code;self.waited=False
    def poll(self):return self.code
    def kill(self):self.code=-1
    def wait(self):self.waited=True;return self.code

class RetainedSupervisor(unittest.TestCase):
    def control(self,kind):
      with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp);args=SimpleNamespace(input=root/'input',output=root/'out',timeout_seconds=60.,memory_limit_gib=16.)
        state=dict(identity='control',state='held' if kind=='unclosed' else 'closed',passed=kind!='custody')
        @contextmanager
        def held(*a,**kw):yield state
        proc=FakeProcess(None if kind=='monitor' else (1 if kind=='worker' else 0))
        def start(*a,**kw):
            args.output.mkdir();(args.output/'RESULT.json').write_text(json.dumps(dict(retained_bulk_conditioning_passed=True,retained_joined_diagnostics_passed=True)))
            if kind=='cleanup':(args.output/'native.linux_scope.json').write_text('{}')
            return proc
        def command(*a,**kw):
            if kind=='cleanup':return SimpleNamespace(returncode=1)
            proc.kill()
        with patch.object(cli,'ROOT',root),patch.object(source_custody,'held',held),patch.object(cli.subprocess,'Popen',start),patch.object(cli.subprocess,'run',side_effect=command),patch.object(psutil,'Process',side_effect=psutil.AccessDenied):
            code=cli.supervise(args)
        result=json.loads((args.output/'RESULT.json').read_text())
        self.assertNotEqual(code,0);self.assertTrue(proc.waited);self.assertIsNotNone(proc.poll())
        self.assertFalse(result['retained_bulk_conditioning_passed']);self.assertFalse(result['retained_joined_diagnostics_passed'])
        self.assertTrue(result['authoritative_supervisor_failure']);self.assertTrue((args.output/'RESULT_BEFORE_SUPERVISOR_STOP.json').exists())
        return result
    def test_monitor_error_kills_waits_and_revokes(self):self.assertEqual(self.control('monitor')['reason'],'MonitorFailure')
    def test_source_custody_failure_revokes(self):self.assertEqual(self.control('custody')['reason'],'SourceCustodyFailure')
    def test_unclosed_custody_revokes(self):self.assertEqual(self.control('unclosed')['reason'],'SourceCustodyFailure')
    def test_worker_failure_revokes_provisional_success(self):self.assertEqual(self.control('worker')['reason'],'WorkerFailure')
    def test_native_cleanup_failure_revokes_success(self):self.assertEqual(self.control('cleanup')['reason'],'NativeCleanupFailure')

if __name__=='__main__':unittest.main()
