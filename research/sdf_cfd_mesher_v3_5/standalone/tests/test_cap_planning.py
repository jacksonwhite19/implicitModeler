"""Causal closed geometry controls; source/CFD qualification is not inferred."""
import sys,tempfile,unittest,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
from planner import Planner,Request
from test_components import closed_control
from cap_planning import Policy,select,cap_report,bounds,family
from collision_policy import topology,exact_check,construct,Policy as CollisionPolicy


def sheared(p,tangent=1.5):
    a=closed_control(p);wall=a.wall*.1;first=wall+(a.first-a.wall)
    vectors=a.vectors.copy();vectors[:,8,1]+=tangent
    return p.state_type(wall,first,a.faces,a.vertex_order,a.normals,a.incidents,vectors,a.first_height,a.scalar_quantity)


class CapPlanning(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.p=Planner(Request(2e-5,1.3,5))
    def h(self,s):return np.broadcast_to(self.p.request.nominal,(len(s.wall),5)).copy()
    def run_select(self,s,h,**kwargs):
        with tempfile.TemporaryDirectory() as tmp:return select(self.p,s,h,Path(tmp)/'cap',**kwargs)
    def test_causal_closed_cap_repair_and_unchanged_prism_gates(self):
        s=sheared(self.p);h=self.h(s);before=cap_report(s,h)
        self.assertFalse(before['passed']);self.assertTrue(self.p.audit(s,h)['numerical_feasible'])
        q,r=self.run_select(s,h)
        self.assertEqual(r['outcome'],'NumericalCapProposal');self.assertTrue(cap_report(s,q)['passed'])
        self.assertTrue(self.p.audit(s,q)['numerical_feasible'])
        np.testing.assert_array_equal(s.positions(h)[1],s.positions(q)[1])
        np.testing.assert_array_equal(q[:,0],h[:,0]);self.assertEqual(q.shape,h.shape)
        low,high=bounds(self.p,h);self.assertTrue(np.all((q>=low)&(q<=high)))
        faces,_=topology(s.faces,5,len(h));cert=exact_check(s.positions(q).reshape(-1,3),faces,range(len(faces)))
        self.assertTrue(cert['passed'])
        self.assertEqual(r['source_global_proof'],'pending');self.assertFalse(r['mesh_accepted'])
    def test_safe_noop_is_byte_identical(self):
        s=closed_control(self.p);h=self.h(s);q,r=self.run_select(s,h)
        self.assertEqual(r['outcome'],'CompatibleCapNoop');self.assertEqual(r['history'],[])
        self.assertEqual(h.tobytes(),q.tobytes())
    def test_all_later_rows_available_and_finite_bounds(self):
        s=sheared(self.p);h=self.h(s);low,high=bounds(self.p,h)
        proposals=list(family(self.p,s,h,cap_report(s,h)['failed_faces']))
        self.assertEqual(len(proposals),585)
        self.assertTrue(any(np.any(q[:,1]!=h[:,1]) for q,_ in proposals))
        self.assertTrue(any(np.any(q[:,2]!=h[:,2]) for q,_ in proposals))
        for q,_ in proposals:
            np.testing.assert_array_equal(q[:,0],h[:,0]);self.assertTrue(np.all((q>=low)&(q<=high)))
    def test_real_other_gate_refuses_cap_only_success(self):
        # Aggressive fixed tangent requires a very thin suffix for the cap;
        # full unchanged prism predicates are still binding.
        s=sheared(self.p,8.);h=self.h(s);q,r=self.run_select(s,h,policy=Policy(max_audits=1))
        self.assertFalse(r['numerical_proposal_passed']);np.testing.assert_array_equal(q,h)
        self.assertGreater(r['cap_compatible_count'],0);self.assertEqual(len(r['history']),1)
        self.assertFalse(r['history'][0]['audit']['numerical_feasible'])
        self.assertEqual(r['history'][0]['outcome'],'OtherNumericalGateRefusal')
        self.assertEqual(r['outcome'],'CapNumericalBudgetExhausted')
    def test_finite_cap_family_exhaustion(self):
        s=sheared(self.p,1000.);h=self.h(s);q,r=self.run_select(s,h)
        self.assertEqual(r['outcome'],'CapFamilyExhausted');self.assertEqual(r['cap_compatible_count'],0)
        self.assertFalse(r['numerical_proposal_passed']);np.testing.assert_array_equal(q,h)
    def test_time_budget_refuses_without_claiming_family_infeasible(self):
        s=sheared(self.p);h=self.h(s);q,r=self.run_select(s,h,policy=Policy(max_seconds=1e-10))
        self.assertEqual(r['outcome'],'CapScreenBudgetExhausted');self.assertFalse(r['numerical_proposal_passed'])
        np.testing.assert_array_equal(q,h)
    def test_collision_feedback_cannot_erase_cap_contract(self):
        # Real collision controls, deliberate contract rejection, no mocked
        # collision predicates or numerical gates.
        from test_collision_policy import opposing
        s=opposing(self.p);h=self.h(s)
        with tempfile.TemporaryDirectory() as tmp:
            q,r=construct(self.p,s,h,Path(tmp)/'collision',policy=CollisionPolicy(max_candidates=2),
                candidate_guard=lambda q:np.array_equal(q,h))
        self.assertFalse(r['proposal_collision_passed']);np.testing.assert_array_equal(q,h)
        self.assertTrue(all(row['outcome']=='CandidateContractRefusal' for row in r['history']))
    def test_invalid_budgets(self):
        for kw in ({'max_audits':False},{'max_audits':0},{'max_seconds':float('nan')}):
            with self.assertRaises(ValueError):Policy(**kw)
    def test_collapsed_cap_is_a_recordable_rejection(self):
        # Artificial collapsed operand tests diagnostics, not a valid shell.
        class Collapsed:
            faces=np.array([[0,1,2]])
            def positions(self,h):return np.zeros((2,3,3))
        r=cap_report(Collapsed(),None)
        self.assertFalse(r['passed']);self.assertEqual(r['invalid_count'],1)
        json.dumps(r,allow_nan=False)

if __name__=='__main__':unittest.main()
