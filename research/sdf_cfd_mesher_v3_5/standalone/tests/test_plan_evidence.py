import sys,json,tempfile,hashlib,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planner import Planner,Request
from plan_evidence import plan_and_capture
from test_components import closed_control


class RefusalCapture(unittest.TestCase):
    def test_real_refusal_retains_input_proposal_and_exact_operands(self):
        p=Planner(Request(1e-5,1.05,5));s=closed_control(p)
        refs=np.arange(len(s.faces),dtype='<i4')+100
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)
            with self.assertRaisesRegex(ValueError,'V3CompleteStackNumericalRefusal'):
                plan_and_capture(p,s,out,face_refs=refs,compress=False)
            folder=out/'plan_evidence';summary=json.loads((folder/'SUMMARY.json').read_text())
            self.assertFalse(summary['numerical_feasible']);self.assertFalse(summary['mesh_accepted'])
            self.assertGreater(len(summary['failed_cells']),0)
            saved,h=p.state_type.load(folder/'input');proposal,h2=p.state_type.load(folder/'proposal')
            self.assertEqual(saved.normals.tobytes(),s.normals.tobytes())
            self.assertEqual(saved.vectors.tobytes(),s.vectors.tobytes())
            self.assertEqual(saved.positions(h).tobytes(),proposal.positions(h2).tobytes())
            row=np.load(folder/'row_05.npz');x=proposal.positions(h2)
            prisms=np.concatenate([x[-2][s.faces],x[-1][s.faces]],axis=1)
            det=p.stack_audit.__globals__['determinant']
            np.testing.assert_array_equal(row['B_determinant'],det(prisms,True,False))
            np.testing.assert_array_equal(row['C_determinant'],det(prisms,True,True))
            np.testing.assert_array_equal(row['source_face_ref'],refs)
            np.testing.assert_array_equal(row['cell_id'],4*len(s.faces)+np.arange(len(s.faces)))
            row.close()
            for name,digest in json.loads((folder/'MANIFEST.json').read_text()).items():
                self.assertEqual(hashlib.sha256((folder/name).read_bytes()).hexdigest(),digest)
            self.assertFalse((out/'native').exists())

    def test_success_keeps_admission_pending(self):
        p=Planner(Request(.000166527069,1.163359674,5));s=closed_control(p)
        with tempfile.TemporaryDirectory() as tmp:
            result=plan_and_capture(p,s,Path(tmp))
            self.assertTrue(result['numerical_feasible']);self.assertFalse(result['mesh_accepted'])
            self.assertEqual(result['source_global_proof'],'pending')


if __name__=='__main__':unittest.main()
