"""Reporting/admission plumbing controls, not geometric qualification evidence."""
import sys,copy,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import generalize_local_remesh as g
import prism_hotpath_v2 as hot
from accepted_trajectory_contract import _planned_quality

class PlannedQualityVerdict(unittest.TestCase):
    def evaluate(self,*,stale=False,old_det=0.,new_det=.1,ownership=True,links=True,incidence=True,other_failure=False,completed=True,cell_id=1000000):
        cell=dict(id=cell_id,volume=1.,determinant=old_det,aspect=1.)
        failures=([dict(kind='cell_quality',**cell)] if old_det<.0011 else [])
        if other_failure:failures.append(dict(kind='face_weight',weight=0.))
        checked=dict(passed=stale,quality=dict(passed=not failures,cells=[cell],failures=failures),
            ownership=dict(passed=ownership),vertex_links=dict(passed=links))
        final=dict(passed=incidence,cells=[dict(id=cell_id,determinant=new_det)])
        wall=np.zeros((3,3));faces=np.array([[0,1,2]]);positions=np.zeros((3,3,3))
        # Patch only upstream return values to isolate recomputation and AND
        # propagation. Real numerical/collision controls remain separate.
        with patch.object(g,'validation',return_value=([],[],[],checked,{})), \
             patch.object(g.t,'ownership',return_value=({},{})), \
             patch.object(g.t,'geometry',lambda *args:None), \
             patch.object(hot,'completed_incidence',return_value=final):
            return _planned_quality(wall,faces,positions,completed_front=completed)[4]
    def test_stale_false_recomputed_after_completed_role_pass(self):
        r=self.evaluate();self.assertTrue(r['passed']);self.assertTrue(r['quality']['passed'])
        self.assertEqual(r['quality']['cells'][0]['determinant'],.1)
    def test_stale_true_recomputed_after_completed_role_failure(self):
        r=self.evaluate(stale=True,old_det=.1,new_det=0.)
        self.assertFalse(r['passed']);self.assertFalse(r['quality']['passed'])
    def test_every_required_component_failure_propagates(self):
        for key in ('ownership','links','incidence'):
            with self.subTest(component=key):self.assertFalse(self.evaluate(stale=True,old_det=.1,**{key:False})['passed'])
    def test_noncell_failure_is_retained(self):
        r=self.evaluate(stale=True,old_det=.1,other_failure=True)
        self.assertFalse(r['passed']);self.assertEqual(r['quality']['failures'][0]['kind'],'face_weight')
    def test_incomplete_front_keeps_existing_later_role(self):
        r=self.evaluate(stale=True,old_det=0.,new_det=.1,completed=False,cell_id=2000000)
        self.assertFalse(r['passed']);self.assertEqual(r['quality']['cells'][0]['determinant'],0.)

if __name__=='__main__':unittest.main()
