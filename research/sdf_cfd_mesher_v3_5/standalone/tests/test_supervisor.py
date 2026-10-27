from pathlib import Path
import sys,unittest,tempfile,json,subprocess,contextlib,importlib.util
from types import SimpleNamespace
from unittest.mock import patch
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import mesh as module
import source_custody,standalone_background
class FailClosed(unittest.TestCase):
 def test_cleanup_errors_cannot_retain_acceptance(self):
  for failure in (subprocess.TimeoutExpired(['scoped_cancel'],15),OSError('injected launch failure')):
   with self.subTest(failure=type(failure).__name__),tempfile.TemporaryDirectory(dir=HERE/'results') as tmp:
    root=Path(tmp);output=root/'out';args=SimpleNamespace(output=output,timeout_seconds=10,memory_limit_gib=16)
    @contextlib.contextmanager
    def custody(*args):yield dict(identity='fault_injection',passed=False)
    class Worker:
     def __init__(self,*args,**kwargs):
      output.mkdir();(output/'RESULT.json').write_text(json.dumps(dict(passed=True,full_mesh_complete=True)))
      for name in ('first','second'):(output/(name+'.linux_scope.json')).write_text('{}')
     def poll(self):return 0
     def wait(self):return 0
    calls=[]
    def cancel(command,**kwargs):
     state=json.loads((output/'RESULT.json').read_text());self.assertFalse(state['passed']);self.assertFalse(state['full_mesh_complete']);self.assertEqual(state['cleanup_status'],'pending')
     calls.append(command)
     if len(calls)==1:raise failure
     return SimpleNamespace(returncode=0,stderr='')
    with patch.object(module,'ROOT',root),patch.object(module.sys,'argv',['mesh.py']),patch.object(source_custody,'held',custody),patch.object(module.subprocess,'Popen',Worker),patch.object(module.subprocess,'run',cancel),patch.object(standalone_background,'stop_backend') as backend:
     self.assertNotEqual(module.supervise(args),0)
    self.assertEqual(len(calls),2);backend.assert_called_once_with(output)
    final=json.loads((output/'RESULT.json').read_text());self.assertFalse(final['passed']);self.assertFalse(final['full_mesh_complete']);self.assertEqual(final['cleanup_status'],'completed_with_errors');self.assertEqual(final['cleanup'][0]['error'],type(failure).__name__)
    self.assertTrue(json.loads((output/'RESULT_BEFORE_SUPERVISOR_STOP.json').read_text())['passed'])

 def test_output_scan_ignores_disappeared_checkpoint(self):
  import stat
  def missing():raise FileNotFoundError('atomic checkpoint rename')
  folder=SimpleNamespace(rglob=lambda pattern:iter([SimpleNamespace(stat=missing),SimpleNamespace(stat=lambda:SimpleNamespace(st_mode=stat.S_IFREG,st_size=7)),SimpleNamespace(stat=lambda:SimpleNamespace(st_mode=stat.S_IFDIR,st_size=400))]))
  self.assertEqual(module.output_bytes(folder),7)

 def test_unexpected_monitor_failure_stops_worker_and_refuses(self):
  with tempfile.TemporaryDirectory(dir=HERE/'results') as tmp:
   root=Path(tmp);output=root/'out';args=SimpleNamespace(output=output,timeout_seconds=100,memory_limit_gib=16)
   @contextlib.contextmanager
   def custody(*args):yield dict(identity='fault_injection',passed=True)
   workers=[]
   class Worker:
    pid=42
    def __init__(self,*args,**kwargs):
     self.killed=False;workers.append(self);output.mkdir();(output/'RESULT.json').write_text(json.dumps(dict(passed=True,full_mesh_complete=True)))
    def poll(self):return 1 if self.killed else None
    def kill(self):self.killed=True
    def wait(self):return 1
   parent=SimpleNamespace(children=lambda recursive:[],memory_info=lambda:SimpleNamespace(rss=1),is_running=lambda:True)
   import psutil
   with patch.object(module,'ROOT',root),patch.object(module.sys,'argv',['mesh.py']),patch.object(source_custody,'held',custody),patch.object(module.subprocess,'Popen',Worker),patch.object(psutil,'Process',lambda pid:parent),patch.object(module.time,'perf_counter',side_effect=[0,6,7]),patch.object(module,'output_bytes',side_effect=PermissionError('injected monitor error')),patch.object(standalone_background,'stop_backend') as backend:
    self.assertNotEqual(module.supervise(args),0)
   self.assertTrue(workers[0].killed);backend.assert_called_once_with(output)
   final=json.loads((output/'RESULT.json').read_text());self.assertFalse(final['passed']);self.assertEqual(final['stop_reason'],'SupervisorFailure');self.assertEqual(final['monitor_error']['type'],'PermissionError')

if __name__=='__main__':unittest.main()
