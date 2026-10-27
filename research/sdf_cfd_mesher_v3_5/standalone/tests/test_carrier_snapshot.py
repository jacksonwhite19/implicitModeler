import sys,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import adaptive_sdf_carrier as carrier
from procedural_authorities import SeedVertex,SeedFace
from carrier_snapshot import write_terminal,read_terminal,require_supported_handoff

class Sphere:
    def exact_normal(self,ref,p):return p/np.linalg.norm(p)
    def project_sheet(self,ref,p):return p/np.linalg.norm(p)
    def implicit_value(self,ref,p):return np.linalg.norm(p)-1
    def competing_component_sheet_refs(self,ref):return []

class CarrierSnapshot(unittest.TestCase):
    def test_normal_success_and_guard_refusal_exact_snapshot_without_queries(self):
        for refused in (False,True):
            with self.subTest(refused=refused),tempfile.TemporaryDirectory() as directory:
                x=np.array([[1.,-0.,0.],[1.,1e-8,0.],[1.,0.,1e-8]])
                vertices=[SeedVertex(p,1,0,0) for p in x];faces=[SeedFace((0,1,2),1)];calls=[]
                def metric(p,ref):
                    calls.append(p.tobytes())
                    return np.eye(3)*(100 if refused and not any(p.tobytes()==v.point.tobytes() for v in vertices) else 1)
                def capture(payload):
                    count=len(calls)
                    result=write_terminal(Path(directory)/'CARRIER_TERMINAL.npz',payload,dict(source_identity='held-test',geometry_identity='sphere-test'))
                    self.assertEqual(count,len(calls));return result
                v,f,e,receipt=carrier.refine_metric_carriers(Sphere(),vertices,faces,[],metric,carrier.CarrierOraclePolicy(),
                    derivative_diagnostics=False,terminal_capture=capture)
                restored,meta=read_terminal(Path(directory)/'CARRIER_TERMINAL.npz')
                self.assertEqual(restored['coordinates'].tobytes(),x.tobytes())
                np.testing.assert_array_equal(restored['triangles'],[[0,1,2]])
                np.testing.assert_array_equal(restored['base_nodal_metrics'],restored['represented_nodal_metrics'])
                self.assertEqual(meta['carrier_status']['materialization_authorized'],not refused)
                self.assertEqual(receipt['materialization_authorized'],not refused)
                self.assertFalse(meta['acceptance_authority']);self.assertTrue(receipt['terminal_snapshot']['readback_verified'])
                metadata=Path(directory)/'CARRIER_TERMINAL.json';saved=metadata.read_bytes();changed=json.loads(saved)
                changed['context']['source_identity']='different-source';metadata.write_text(json.dumps(changed),encoding='utf-8')
                with self.assertRaisesRegex(ValueError,'CarrierSnapshotMetadataHash'):
                    read_terminal(Path(directory)/'CARRIER_TERMINAL.npz',expected_metadata_sha256=receipt['terminal_snapshot']['metadata_sha256'])
                metadata.write_bytes(saved)
                path=Path(directory)/'CARRIER_TERMINAL.npz';raw=bytearray(path.read_bytes());raw[-1]^=1;path.write_bytes(raw)
                with self.assertRaisesRegex(ValueError,'CarrierSnapshotFileHash'):read_terminal(path)

    def test_rollback_snapshot_matches_assessed_generation_and_preserves_promoted_field(self):
        x=np.array([[1.,0,0],[-1.,0,0],[0,1.,0],[0,-1.,0],[0,0,1.],[0,0,-1.]])
        vertices=[SeedVertex(p,1,0,0) for p in x]
        faces=[SeedFace(row,1) for row in [(0,2,4),(2,1,4),(1,3,4),(3,0,4),(2,0,5),(1,2,5),(3,1,5),(0,3,5)]]
        def assess(a,v,face,metric,*args,**kw):
            for i in face.vertices:metric(v[i].point,face.sheet_ref)
            return dict(inadequate=face.vertices==(0,2,4),minimum_edge_m=1.,maximum_metric_edge_ratio=1.,maximum_normal_interpolation_defect_deg=2.)
        def promote(a,v,*args):return [2*np.eye(3) for _ in v],dict(active_faces=1,changed_vertices=len(v))
        captured=[]
        with patch.object(carrier,'assess_carrier_face',side_effect=assess),patch.object(carrier,'promote_face_probe_metrics',side_effect=promote),patch.object(carrier,'validate_split',side_effect=ValueError('InjectedFailure')):
            _,_,_,receipt=carrier.refine_metric_carriers(Sphere(),vertices,faces,[],lambda p,r:np.eye(3),carrier.CarrierOraclePolicy(),
                interpolation_compatible_transition=True,derivative_diagnostics=False,terminal_capture=lambda p:captured.append(p))
        self.assertFalse(receipt['materialization_authorized']);payload=captured[0]
        np.testing.assert_array_equal(payload['arrays']['coordinates'],x)
        np.testing.assert_array_equal(payload['arrays']['triangles'],[f.vertices for f in faces])
        np.testing.assert_array_equal(payload['arrays']['depths'],np.zeros(8))
        np.testing.assert_array_equal(payload['arrays']['base_nodal_metrics'],np.tile(np.eye(3),(6,1,1)))
        np.testing.assert_array_equal(payload['arrays']['represented_nodal_metrics'],np.tile(2*np.eye(3),(6,1,1)))

    def test_promoted_consumer_mismatch_refuses_without_changing_ordinary_path(self):
        require_supported_handoff(dict(interpolation_compatible_transition=False))
        require_supported_handoff(dict(enabled=False))
        with self.assertRaisesRegex(ValueError,'CarrierPromotedMetricConsumerNotBound'):
            require_supported_handoff(dict(interpolation_compatible_transition=True,materialization_authorized=True))

if __name__=='__main__':unittest.main()
