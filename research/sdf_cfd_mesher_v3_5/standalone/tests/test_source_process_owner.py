"""Real Windows Job cancellation and hard-supervisor-death controls."""
from pathlib import Path
import json,os,subprocess,sys,tempfile,time,unittest
import psutil
from types import SimpleNamespace
from unittest.mock import patch
S=Path(__file__).resolve().parents[1];sys.path.insert(0,str(S))
from source_process_owner import launch

CHILD='''from source_process_owner import worker_start_gate
worker_start_gate()
import subprocess,sys,time,json,psutil
from pathlib import Path
p=subprocess.Popen([sys.executable,'-B','-c','import time;time.sleep(60)'])
path=Path(sys.argv[1]);temporary=path.with_suffix('.pending');temporary.write_text(json.dumps([(x.pid,x.create_time()) for x in [psutil.Process(),psutil.Process(p.pid)]]));temporary.replace(path)
time.sleep(60)
'''

SOURCE_CHILD='''from source_process_owner import worker_start_gate
worker_start_gate()
from pathlib import Path
import os,sys,time,json,psutil
import geometry,standalone_tool as cli
from planner import Request
from source_parallel import Pool
path=Path(sys.argv[1]);case=path.with_suffix('.CASE.json')
root=Path.cwd();geometry.write_case(root/'fixtures/v4_case_template.json',root/'fixtures/sphere.rhai',case,Request(2e-5,1.3,5))
token=cli._surface_assurance.set('numerical')
try:host,binding=cli.geometry(case)
finally:cli._surface_assurance.reset(token)
pool=Pool(binding.authority,2e-5,4,time.perf_counter()+60)
if sys.argv[2]=='pool_parent_exit':host.close()
parent=psutil.Process();rows=[(p.pid,p.create_time()) for p in [parent]+parent.children(recursive=True)]
temporary=path.with_suffix('.pending');temporary.write_text(json.dumps(rows));temporary.replace(path)
if sys.argv[2]=='pool_parent_exit':os._exit(17)
time.sleep(60)
'''

def ready(path,process):
    deadline=time.monotonic()+15
    while not path.exists():
        if process.poll() is not None:raise AssertionError(('GateWorkerExited',process.returncode))
        if time.monotonic()>deadline:raise TimeoutError('ControlStartup')
        time.sleep(.02)
    return json.loads(path.read_text())

def remaining(rows):
    values=[]
    for pid,birth in rows:
        try:
            p=psutil.Process(pid)
            if p.create_time()==birth and p.is_running():values.append(pid)
        except psutil.NoSuchProcess:pass
    return values

class ProcessOwnerTests(unittest.TestCase):
    def test_parallel_monitor_access_denied_fails_and_retires_owned_job(self):
        import mesh,runtime_environment
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);helper=root/'gated.py'
            helper.write_text('import sys\nsys.path.insert(0,'+repr(str(S))+')\nfrom source_process_owner import worker_start_gate\nworker_start_gate()\nimport time\ntime.sleep(60)\n')
            args=SimpleNamespace(output=root/'run',timeout_seconds=20,memory_limit_gib=8,output_limit_bytes=128*2**20,source_workers=4,verification='production')
            def denied(pid):raise psutil.AccessDenied(pid)
            with patch.object(runtime_environment,'preflight',return_value=dict(passed=True)),patch.object(mesh,'__file__',str(helper)),patch.object(mesh.sys,'argv',['mesh.py']),patch.object(psutil,'Process',side_effect=denied):
                self.assertEqual(mesh.supervise(args),125)
            import hashlib
            key=hashlib.sha256(str(args.output.resolve()).encode()).hexdigest()[:12]
            result=json.loads((S/'run_logs'/(key+'.json')).read_text())
            self.assertEqual(result['stop_reason'],'SupervisorFailure')
            self.assertEqual(result['monitor_error']['type'],'AccessDenied')
    def test_actual_source_pool_parent_death_closes_worker_hosts(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'pids.json';rows=[]
            # Outer containment also bounds the test if the inner ownership fails.
            with launch([sys.executable,'-B','-c',SOURCE_CHILD,str(path),'pool_parent_exit'],contained=True,cwd=S) as p:
                rows=ready(path,p);self.assertGreaterEqual(len(rows),9)
                self.assertEqual(p.wait(timeout=20),17)
                deadline=time.monotonic()+5
                while remaining(rows) and time.monotonic()<deadline:time.sleep(.02)
                self.assertFalse(remaining(rows))
    def test_interrupt_and_system_exit_retire_worker_and_descendants(self):
        for exception in (KeyboardInterrupt,SystemExit):
            with self.subTest(exception=exception),tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'pids.json';rows=[]
                with self.assertRaises(exception):
                    with launch([sys.executable,'-B','-c',CHILD,str(path)],contained=True,cwd=S) as p:
                        rows=ready(path,p);raise exception()
                self.assertFalse(remaining(rows))
    def test_outer_supervisor_hard_exit_closes_noninherited_job(self):
        for child in (CHILD,SOURCE_CHILD):
          with self.subTest(actual_source_pool=child is SOURCE_CHILD),tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'pids.json'
            supervisor='''from source_process_owner import launch
from pathlib import Path
import os,sys,time
with launch([sys.executable,'-B','-c',sys.argv[2],sys.argv[1],'wait'],contained=True) as p:
 deadline=time.monotonic()+15
 while not Path(sys.argv[1]).exists():
  if time.monotonic()>deadline:raise TimeoutError('ControlStartup')
  time.sleep(.02)
 os._exit(17)
'''
            p=subprocess.Popen([sys.executable,'-B','-c',supervisor,str(path),child],cwd=S)
            rows=[]
            try:
                rows=ready(path,p);self.assertEqual(p.wait(timeout=20),17)
                deadline=time.monotonic()+5
                while remaining(rows) and time.monotonic()<deadline:time.sleep(.02)
                self.assertFalse(remaining(rows))
            finally:
                if p.poll() is None:p.kill();p.wait(timeout=5)
                for pid,birth in rows:
                    try:
                        q=psutil.Process(pid)
                        if q.create_time()==birth:q.kill()
                    except psutil.NoSuchProcess:pass

if __name__=='__main__':unittest.main()
