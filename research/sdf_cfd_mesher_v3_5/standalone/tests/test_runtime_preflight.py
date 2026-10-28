"""Windows supervisor ordering/custody controls; never launches geometry."""
from pathlib import Path
from contextlib import redirect_stdout
from types import SimpleNamespace
import io,json,os,subprocess,sys,tempfile,unittest
from unittest.mock import MagicMock,patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import geometry,mesh,runtime_environment

@unittest.skipUnless(os.name=='nt','Actual Windows source custody')
class PreflightOrder(unittest.TestCase):
    def test_refusal_and_exception_precede_worker_and_geometry_under_custody(self):
        for failure in ('mismatch','exception'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);args=SimpleNamespace(output=root/'fresh',verification='production',timeout_seconds=60,memory_limit_gib=16,output_limit_bytes=1000000000)
                guarded=[]
                def refuse(*a,**kw):
                    for p in [ROOT/'runtime_environment.py',ROOT/'v4/runtime/isolated_brotli/libbrotlidec.so.1']:
                        with self.assertRaises(PermissionError):
                            with p.open('r+b'):pass
                        guarded.append(str(p))
                    if failure=='exception':raise ValueError('changed library')
                    return dict(schema='V35ExternalRuntimePreflight/1',passed=False,mismatches=[{'synthetic_changed_library':True}])
                launch=MagicMock()
                with patch.object(mesh,'ROOT',root),patch.object(runtime_environment,'preflight',side_effect=refuse),patch.object(mesh,'subprocess',SimpleNamespace(Popen=launch,run=subprocess.run)),patch.object(mesh,'execute') as geometry_worker,redirect_stdout(io.StringIO()):
                    code=mesh.supervise(args)
                self.assertEqual(code,1);launch.assert_not_called();geometry_worker.assert_not_called()
                self.assertEqual(len(guarded),2)
                value=json.loads((args.output/'RESULT.json').read_text());self.assertEqual(value['stop_reason'],'ExternalRuntimePreflightRefusal');self.assertFalse(value['worker_launched']);self.assertTrue(value['source_custody_passed'])
                self.assertFalse((args.output/'VERIFICATION.json').exists());self.assertFalse((args.output/'upstream').exists())
                sources=list((root/'run_logs').glob('*.SOURCES.json'));self.assertEqual(len(sources),1);self.assertTrue(json.loads(sources[0].read_text())['passed'])

if __name__=='__main__':unittest.main()
