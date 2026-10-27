"""Operational quota controls; no actual worker or geometry query is run."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import contextlib,hashlib,io,json,sys,tempfile,unittest
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
import mesh as cli
import source_custody,standalone_background,psutil

class OutputBudget(unittest.TestCase):
    def test_supervisor_preserves_default_and_enforces_explicit_budget(self):
        for limit,size,stopped in ((None,5_000_000_000,True),(16_000_000_000,5_000_000_000,False),(16_000_000_000,17_000_000_000,True)):
            with self.subTest(limit=limit,size=size),tempfile.TemporaryDirectory(dir=HERE/'results') as tmp:
                root=Path(tmp);out=root/'out'
                args=SimpleNamespace(output=out,timeout_seconds=100,memory_limit_gib=16,output_limit_bytes=limit)
                @contextlib.contextmanager
                def held(*args):yield dict(identity='quota_control',passed=True)
                class Worker:
                    pid=42
                    def __init__(self,*args,**kwargs):
                        self.killed=False;self.polls=0;out.mkdir()
                        (out/'RESULT.json').write_text(json.dumps(dict(passed=True,full_mesh_complete=True)))
                    def poll(self):
                        self.polls+=1
                        return None if self.polls==1 and not self.killed else (1 if self.killed else 0)
                    def kill(self):self.killed=True
                    def wait(self):return 1 if self.killed else 0
                parent=SimpleNamespace(children=lambda recursive:[],memory_info=lambda:SimpleNamespace(rss=1),is_running=lambda:True)
                with patch.object(cli,'ROOT',root),patch.object(cli.sys,'argv',['mesh.py']),patch.object(source_custody,'held',held),patch.object(cli.subprocess,'Popen',Worker),patch.object(psutil,'Process',lambda pid:parent),patch.object(cli.time,'perf_counter',side_effect=[0,6,7]),patch.object(cli.time,'sleep'),patch.object(cli,'output_bytes',return_value=size),patch.object(standalone_background,'stop_backend'):
                    code=cli.supervise(args)
                key=hashlib.sha256(str(out.resolve()).encode()).hexdigest()[:12]
                receipt=json.loads((root/'run_logs'/(key+'.json')).read_text())
                self.assertEqual(receipt['output_limit_bytes'],4_000_000_000 if limit is None else limit)
                self.assertEqual(code!=0,stopped)
                self.assertEqual(receipt['stop_reason'],'OutputLimit' if stopped else None)
                result=json.loads((out/'RESULT.json').read_text())
                self.assertEqual(result['full_mesh_complete'],not stopped)

    def test_actual_case_configuration_gets_same_explicit_budget(self):
        for limit in (None,16_000_000_000):
            with self.subTest(limit=limit),tempfile.TemporaryDirectory(dir=HERE/'results') as tmp:
                out=Path(tmp)/'out'
                args=SimpleNamespace(output=out,case=HERE/'fixtures/v4_case_template.json',geometry=HERE/'fixtures/sphere.rhai',
                    prism_first_height=2e-5,prism_growth=1.3,prism_layers=5,metres_per_model_unit=.001,
                    surface_size=.0002,surface_policy='size-led',surface_normal_angle_deg=None,surface_position_error=None,
                    background=HERE/'fixtures/background_direct_v4.json',output_limit_bytes=limit,stop_after='complete')
                class BeforeGeometry(Exception):pass
                def stop_surface(*args,**kwargs):raise BeforeGeometry()
                with patch.object(cli,'surface',stop_surface),self.assertRaises(BeforeGeometry):cli.execute(args)
                case=json.loads((out/'CASE.json').read_text())
                self.assertEqual(case['resources']['output_limit_bytes'],4_000_000_000 if limit is None else limit)
                self.assertEqual(case['surface']['position_limit_m'],5e-5)
                self.assertEqual(case['general_sdf']['seed']['maximum_queries'],4_000_000)

    def test_cli_accepts_positive_integer_and_refuses_invalid_budgets(self):
        base=['mesh.py','--case','template.json','--geometry','shape.rhai','--metres-per-model-unit','.001',
            '--prism-first-height','.00002','--prism-growth','1.3','--prism-layers','5','--output','fresh']
        for value in ('0','-1','1.5'):
            with self.subTest(value=value),patch.object(sys,'argv',base+['--output-limit-bytes',value]),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as failure:
                cli.main()
            self.assertEqual(failure.exception.code,2)
        with patch.object(sys,'argv',base+['--output-limit-bytes','16000000000']),patch.object(cli,'supervise',return_value=0) as supervised:
            self.assertEqual(cli.main(),0)
        self.assertEqual(supervised.call_args.args[0].output_limit_bytes,16_000_000_000)

if __name__=='__main__':unittest.main()
