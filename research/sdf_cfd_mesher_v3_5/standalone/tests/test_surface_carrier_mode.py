"""Construction-mode routing only; no source evaluation or meshing."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import contextlib,io,json,sys,tempfile,unittest
HERE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE))
import geometry,mesh
import standalone_tool as v4
from production_surface_authority import fastprod_surface_scope_active

class SurfaceCarrierMode(unittest.TestCase):
    def test_modes_default_same_stage_and_provenance(self):
        for requested,expected,origin in ((None,True,'v3_5_backward_compatible_default'),('adaptive',True,'explicit_api'),('nodal',False,'explicit_api')):
            with self.subTest(requested=requested),tempfile.TemporaryDirectory() as tmp:
                out=Path(tmp)/'upstream';case=Path(tmp)/'CASE.json';case.write_text('{}')
                before=(v4._carrier_assessment.get(),v4.surface_assurance())
                def stage(output,path,*,profile):
                    self.assertEqual(output,out);self.assertEqual(path,case);self.assertEqual(profile,'fastprod')
                    self.assertEqual(v4._carrier_assessment.get(),expected);self.assertEqual(v4.surface_assurance(),'numerical')
                    self.assertTrue(fastprod_surface_scope_active(out/'surface/A/fresh'))
                    return out/'surface/A/fresh',{'test_only':True}
                with patch.object(v4,'surface_stage',side_effect=stage) as called:
                    with geometry.surface(case,out,carrier_mode=requested) as result:
                        self.assertTrue(result[1]['test_only']);self.assertEqual(v4._carrier_assessment.get(),expected)
                called.assert_called_once()
                self.assertEqual(before,(v4._carrier_assessment.get(),v4.surface_assurance()))
                receipt=json.loads((out/'SURFACE_CONSTRUCTION.json').read_text())
                self.assertEqual(receipt['origin'],origin);self.assertEqual(receipt['requested'],requested)
                self.assertEqual(receipt['effective'],'adaptive' if expected else 'nodal')

    def test_nested_and_all_exception_boundaries_restore_context(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);before=(v4._carrier_assessment.get(),v4.surface_assurance())
            def stage(output,path,*,profile):return output/'surface/A/fresh',{}
            with patch.object(v4,'surface_stage',side_effect=stage):
                with geometry.surface(root/'case',root/'outer',carrier_mode='nodal'):
                    self.assertFalse(v4._carrier_assessment.get())
                    with self.assertRaisesRegex(ValueError,'body refusal'):
                        with geometry.surface(root/'case',root/'inner',carrier_mode='adaptive'):
                            self.assertTrue(v4._carrier_assessment.get());raise ValueError('body refusal')
                    self.assertFalse(v4._carrier_assessment.get());self.assertEqual(v4.surface_assurance(),'numerical')
            self.assertEqual(before,(v4._carrier_assessment.get(),v4.surface_assurance()))
            with patch.object(v4,'surface_stage',side_effect=ValueError('final qualification refusal')):
                with self.assertRaisesRegex(ValueError,'final qualification refusal'):
                    with geometry.surface(root/'case',root/'refused',carrier_mode='nodal'):self.fail('refusal must not yield')
            self.assertEqual(before,(v4._carrier_assessment.get(),v4.surface_assurance()))

    def test_cli_parsing_and_worker_forwarding(self):
        base=['mesh.py','--case','template.json','--geometry','shape.rhai','--metres-per-model-unit','.001',
            '--prism-first-height','.00002','--prism-growth','1.3','--prism-layers','5','--output','fresh']
        for requested in (None,'adaptive','nodal'):
            argv=base+([] if requested is None else ['--surface-carrier',requested])
            with patch.object(sys,'argv',argv),patch.object(mesh,'supervise',return_value=0) as supervisor:
                self.assertEqual(mesh.main(),0)
            self.assertEqual(supervisor.call_args.args[0].surface_carrier,requested)
        with patch.object(sys,'argv',base+['--surface-carrier','unknown']),contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):mesh.main()

    def test_execute_provenance_same_case_and_final_refusal(self):
        cases=[]
        for requested in (None,'adaptive','nodal'):
            with self.subTest(requested=requested),tempfile.TemporaryDirectory(dir=HERE/'results') as tmp:
                out=Path(tmp)/'out'
                args=SimpleNamespace(output=out,case=HERE/'fixtures/passage_historical_case.json',geometry=HERE/'fixtures/passage.rhai',
                    prism_first_height=2e-5,prism_growth=1.3,prism_layers=5,metres_per_model_unit=.001,
                    surface_size=.0002,surface_policy='size-led',surface_normal_angle_deg=None,surface_position_error=None,
                    background=HERE/'fixtures/background_direct_v4.json',output_limit_bytes=16_000_000_000,stop_after='surface',surface_carrier=requested)
                def stage(output,path,*,profile):
                    self.assertEqual(profile,'fastprod');self.assertEqual(v4._carrier_assessment.get(),requested!='nodal')
                    raise ValueError('final qualification refusal')
                with patch.object(v4,'surface_stage',side_effect=stage),self.assertRaisesRegex(ValueError,'final qualification refusal'):mesh.execute(args)
                cases.append((out/'CASE.json').read_bytes())
                root=json.loads((out/'SURFACE_CONSTRUCTION.json').read_text());upstream=json.loads((out/'upstream/SURFACE_CONSTRUCTION.json').read_text())
                self.assertEqual(root,upstream);self.assertEqual(root['requested'],requested)
                self.assertEqual(root['origin'],'v3_5_backward_compatible_default' if requested is None else 'explicit_cli')
                result=json.loads((out/'RESULT.json').read_text())
                self.assertFalse(result['passed']);self.assertFalse(result['full_mesh_complete']);self.assertEqual(result['failed_stage'],'surface')
                self.assertEqual(result['surface_construction'],root);self.assertFalse(list(out.rglob('SURFACE_AUTHORITY.json')))
                self.assertFalse((out/'first').exists())
        self.assertEqual(cases[0],cases[1]);self.assertEqual(cases[0],cases[2])

if __name__=='__main__':unittest.main()
