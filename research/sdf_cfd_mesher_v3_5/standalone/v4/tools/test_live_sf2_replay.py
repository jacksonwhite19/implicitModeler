"""Bounded replay/publication checks; no generation or qualification rerun."""
import tempfile,unittest
from pathlib import Path
import numpy as np
from replay_live_sf2_surface import preflight,compare_arrays,scientific,REFERENCE,save,publish_surface

class ReplayTests(unittest.TestCase):
    def test_reference_preflight(self):
        p=preflight();self.assertTrue(p['passed']);self.assertEqual(p['reference_files_checked'],45)
    def test_existing_scientific_payload_self_comparison(self):
        r=compare_arrays(REFERENCE/'fresh/REFERENCES.npz',REFERENCE/'fresh/REFERENCES.npz')
        self.assertTrue(r['passed']);self.assertEqual(len(r['arrays']),4)
    def test_changed_bits_and_missing_arrays_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p,q=Path(d)/'a.npz',Path(d)/'b.npz'
            np.savez(p,a=np.array([0.,1.]));np.savez(q,a=np.array([-0.,1.]))
            self.assertFalse(compare_arrays(p,q)['passed'])
            np.savez(q,a=np.array([0.,np.nextafter(1.,2.)]))
            self.assertFalse(compare_arrays(p,q)['passed'])
            np.savez(q,b=np.array([0.,1.]))
            self.assertFalse(compare_arrays(p,q)['passed'])
    def test_only_named_metadata_excluded(self):
        a={'seconds':1,'timing':{'command':'old'},'bounds':{'position':1.}}
        b={'seconds':2,'timing':{'command':'new'},'bounds':{'position':1.}}
        self.assertEqual(scientific(a),scientific(b))
        b['bounds']['position']=2.;self.assertNotEqual(scientific(a),scientific(b))
    def test_existing_publisher_rejects_failed_audit_and_replay(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);save(p/'AUDIT.json',{'passed':False})
            with self.assertRaisesRegex(ValueError,'CannotPublishUnqualifiedSurface'):
                publish_surface(p,{},'unused',{'passed':True})
            save(p/'AUDIT.json',{'passed':True})
            with self.assertRaisesRegex(ValueError,'CannotPublishUnqualifiedSurface'):
                publish_surface(p,{},'unused',{'passed':False})
            self.assertFalse((p/'SURFACE_AUTHORITY.json').exists())

if __name__=='__main__':unittest.main(verbosity=2)
