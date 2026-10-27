"""Focused request, binding and fail-closed control checks; no mesh run."""
import unittest
from unittest.mock import patch
import numpy as np
import run_live_sf2_first_layer as run
from live_spline_first_layer import SourceSheetFirstLayerProvider,existing_compatibility

class Checks(unittest.TestCase):
    def test_bound_request(self):
        p=run.preflight()
        self.assertTrue(p['admission']['generation_authorized'])
        self.assertEqual(p['admission']['requested_layer_count'],1)
        self.assertEqual(p['height_hex'],'0x1.5d3b8a168f007p-13')
        self.assertEqual(p['surface']['surface_identity'],run.SURFACE_ID)

    def test_incident_height_not_normalized(self):
        n=np.array([[1.,0,0],[0,1.,0]])
        r=existing_compatibility()(n);v=np.array(r['displacement_per_height'])
        self.assertTrue(np.array_equal(n@v,np.ones(2)))
        self.assertGreater(np.linalg.norm(v),1.)
        self.assertEqual(r['classification'],'SharedFiniteCornerTrajectoryFeasible')

    def test_incompatible_normals(self):
        r=existing_compatibility()(np.array([[1.,0,0],[-1.,0,0]]))
        self.assertEqual(r['classification'],'SharedFiniteCornerTrajectoryInfeasible')

    def test_scope_and_failed_commit(self):
        p=SourceSheetFirstLayerProvider.__new__(SourceSheetFirstLayerProvider)
        with self.assertRaisesRegex(ValueError,'ExactlyOneLayerScope'):p.propose(None)
        with self.assertRaisesRegex(ValueError,'NoLaterFrontConstruction'):p.geometry({},extra=1)
        p.check={'passed':False};p.assigned={'admitted':True}
        with self.assertRaisesRegex(ValueError,'FailedFirstLayerCommit'):p.commit_candidate()

    def test_unsupported_binding(self):
        from types import SimpleNamespace
        with self.assertRaisesRegex(ValueError,'SupportedLiveSplineAuthorityRequired'):
            SourceSheetFirstLayerProvider(None,None,None,SimpleNamespace(authority=object()))

    def test_no_volume_after_failed_certificate(self):
        # Orchestration-only negative; existing certificate math is not mocked
        # in the scientific run. No test data enter that run.
        import tempfile
        from pathlib import Path
        p=SourceSheetFirstLayerProvider.__new__(SourceSheetFirstLayerProvider)
        p.pack=lambda *_:{};p.x=np.zeros((0,3));p.tri=[];p.times={};p.previous_certificate_state=None
        with tempfile.TemporaryDirectory() as tmp:
            p.folder=Path(tmp)
            with patch('live_spline_first_layer.g.s.construct',return_value=([],[],[])),patch('live_spline_first_layer.g.t.ownership',return_value=({},{})),patch('reuse_mixed_certificate.certify',return_value={'passed':False,'proofs':[]}),patch('live_spline_first_layer.g.volume.audit') as volume:
                self.assertFalse(p.certify_candidate([],[],{})['passed'])
                volume.assert_not_called()
                self.assertTrue((p.folder/'FIRST_FAILURE.json').exists())

    def test_failed_or_mismatched_replay_cannot_seal(self):
        import verify_live_sf2_first_layer as verify
        good={k:'same' for k in verify.FIELDS};good['passed']=True
        for bad in [dict(good,passed=False),dict(good,front_coordinate_hash='different')]:
            with patch.object(verify,'read',side_effect=[good,bad]),patch.object(verify,'save') as write:
                with self.assertRaises(ValueError):verify.replay_and_index()
                write.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
