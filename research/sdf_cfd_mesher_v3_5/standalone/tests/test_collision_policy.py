"""Real mixed-face geometry controls. No mocked collision-removal evidence."""
import sys,tempfile,unittest,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
from planner import Planner,Request
from test_components import closed_control
from collision_policy import construct,Policy,topology,exact_check,WitnessLedger,digest,state_identity,preview,feedback_family,segment_guard,SourceSegmentRefusal

def opposing(planner,gap=.00025):
    s=closed_control(planner);wall=s.wall*.2;first=wall+planner.request.first_height*s.vectors[0]
    offset=np.array([.0002+gap/2,0,0]);n=len(wall)
    return planner.state_type(np.concatenate((wall-offset,wall+offset)),np.concatenate((first-offset,first+offset)),
        np.concatenate((s.faces,s.faces+n)),[str(i) for i in range(2*n)],s.normals,list(s.incidents)*2,
        np.concatenate((s.vectors,s.vectors),axis=1),planner.request.first_height,s.scalar_quantity)

class CollisionPolicy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.planner=Planner(Request(2e-5,1.3,5))
    def run_control(self,s,policy=Policy(max_candidates=8,max_seconds=120)):
        h=np.broadcast_to(self.planner.request.nominal,(len(s.wall),5)).copy()
        with tempfile.TemporaryDirectory() as tmp:
            q,r=construct(self.planner,s,h,Path(tmp)/'test',policy=policy)
        return h,q,r
    def test_real_closed_positive_and_preview_miss_fallback(self):
        s=opposing(self.planner);h=np.broadcast_to(self.planner.request.nominal,(len(s.wall),5)).copy()
        self.assertTrue(self.planner.audit(s,h)['numerical_feasible'])
        # One anticipatory seed deliberately leaves real collision columns unflagged.
        h,q,r=self.run_control(s,Policy(max_candidates=8,max_seconds=120,preview_seed_columns=1))
        self.assertEqual(r['outcome'],'CollisionCorrected');self.assertGreater(len(r['baseline']['failures']),0)
        self.assertTrue(r['final_exact']['passed']);self.assertTrue(self.planner.audit(s,q)['numerical_feasible'])
        self.assertFalse(np.array_equal(h,q));np.testing.assert_array_equal(h[:,:3],q[:,:3])
        np.testing.assert_array_equal(s.positions(h)[1],s.positions(q)[1])
        self.assertTrue(np.all(q<=h));self.assertEqual(q.shape,h.shape)
        forecast=preview(s,h,Policy(preview_seed_columns=1))
        witness_cols={c for w in r['witnesses'] for f in w['faces'] for _,c in f['source_columns_and_layers']}
        self.assertTrue(witness_cols-set(forecast['seed_columns']))
        self.assertFalse(forecast['guaranteed_prevention']);self.assertTrue(forecast['risks'])
        self.assertEqual(len({v['proposal'] for v in r['history']}),len(r['history']))
    def test_safe_noop_and_legal_adjacency(self):
        s=opposing(self.planner,gap=.001);h,q,r=self.run_control(s)
        self.assertEqual(r['outcome'],'BaselineCollisionFree');self.assertEqual(r['history'],[])
        np.testing.assert_array_equal(h,q);self.assertTrue(r['baseline']['passed'])
    def test_immutable_prefix_exhaustion(self):
        s=opposing(self.planner,gap=.00010);h,q,r=self.run_control(s,Policy(max_candidates=2,max_seconds=120))
        self.assertFalse(r['proposal_collision_passed']);self.assertEqual(r['outcome'],'CollisionBudgetExhausted')
        self.assertEqual(len(r['history']),2);self.assertTrue(all(v['outcome']=='UnchangedKnownForbidden' for v in r['history']))
        np.testing.assert_array_equal(q,h)
    def test_changed_vs_stationary_outsider_and_witness_identity(self):
        # Two disjoint triangles. Move one through an unrelated stationary face.
        points=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,1],[.8,.2,1],[.2,.8,1]],dtype=float)
        faces=[(0,1,2),(3,4,5)];self.assertTrue(exact_check(points,faces,[0,1])['passed'])
        q=points.copy();q[3:,2]=[-1,1,1]
        bad=exact_check(q,faces,[1]);self.assertFalse(bad['passed'])
        s=opposing(self.planner);ledger=WitnessLedger(faces,[[[1,0]],[[2,1]]],s)
        ledger.add(bad,q,'proposal-A');self.assertTrue(ledger.unchanged_forbidden(q))
        q2=q.copy();q2[3,0]+=.01;self.assertFalse(ledger.unchanged_forbidden(q2))
        ledger.add(exact_check(q2,faces,[1]),q2,'proposal-B')
        self.assertEqual(len(ledger.rows),2);self.assertEqual(ledger.rows[-1]['proposal'],'proposal-B')
        self.assertEqual(len(ledger.columns()),2)
        old=state_identity(s);s.vertex_order=tuple(reversed(s.vertex_order));self.assertNotEqual(old,state_identity(s))
    def test_shared_vertex_dependencies_and_new_intersections(self):
        # Shared vertices do not exempt crossing interiors from the real predicate.
        p=np.array([[0,0,0],[1,0,0],[0,1,0],[.7,.2,0],[.2,.7,0]],float)
        self.assertFalse(exact_check(p,[(0,1,2),(0,3,4)],[1])['passed'])
        # Local guard cannot erase a forbidden pair among stationary outsiders.
        p=np.concatenate((p,np.array([[3,0,0],[4,0,0],[3,1,0]])))
        f=[(0,1,2),(0,3,4),(5,6,7)]
        self.assertTrue(exact_check(p,f,[2])['passed']);self.assertFalse(exact_check(p,f,range(3))['passed'])
    def test_invalid_budgets(self):
        for kw in ({'max_candidates':0},{'max_seconds':float('nan')},{'preview_pairs':False}):
            with self.assertRaises(ValueError):Policy(**kw)
    def test_new_real_witness_expands_finite_family_support(self):
        s=opposing(self.planner)
        p=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,-1],[.8,.2,1],[.2,.8,1]],float)
        p=np.concatenate((p,p+np.array([10,0,0])))
        f=[(0,1,2),(3,4,5),(6,7,8),(9,10,11)]
        ledger=WitnessLedger(f,[[[5,i]] for i in range(4)],s)
        ledger.add(exact_check(p,f,[0,1]),p,'A');self.assertEqual(ledger.columns(),[0,1])
        h=np.broadcast_to(self.planner.request.nominal,(len(s.wall),5)).copy()
        family=feedback_family(self.planner,s,h,ledger,dict(suggested_core_scale=.5,seed_columns=[]))
        _,a=next(family);self.assertEqual(a['seed_columns'],[0,1])
        ledger.add(exact_check(p,f,[2,3]),p,'B')
        _,b=next(family);self.assertEqual(b['seed_columns'],[0,1,2,3])
    def test_semantic_mutation_fails_closed(self):
        s=opposing(self.planner);h=np.broadcast_to(self.planner.request.nominal,(len(s.wall),5)).copy()
        def changed_source(x):
            s.vertex_order=tuple(reversed(s.vertex_order));return dict(passed=True)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'CollisionStateChanged'):
                construct(self.planner,s,h,Path(tmp)/'test',source_guard=changed_source)
    def test_real_generic_source_exception_and_candidate_refusal_flow(self):
        import standalone_tool as cli
        root=Path(__file__).resolve().parents[1]
        token=cli._surface_assurance.set('numerical');host=None
        try:
            host,binding=cli.geometry(root/'fixtures/sphere_case.json')
            check=segment_guard(binding.authority,None)
            with self.assertRaises(SourceSegmentRefusal) as caught:
                check(np.array([[[.0011,0,0]],[[.0009,0,0]]]))
            detail=caught.exception.detail
            self.assertEqual(detail['args'],['SourceOutwardSegmentUnresolved'])
            self.assertEqual(detail['diagnostic']['attempt']['reason'],'endpoint_positive_unresolved')
            # This tests flow using the real captured source refusal, not source
            # qualification of the separate closed collision-control geometry.
            calls=[]
            def eligibility(x):
                calls.append(1)
                if len(calls)>1:raise SourceSegmentRefusal(detail)
                return dict(scope='flow control only')
            s=opposing(self.planner);h=np.broadcast_to(self.planner.request.nominal,(len(s.wall),5)).copy()
            with tempfile.TemporaryDirectory() as tmp:
                _,r=construct(self.planner,s,h,Path(tmp)/'flow',policy=Policy(max_candidates=2,max_seconds=120),source_guard=eligibility)
            self.assertEqual(r['history'][0]['outcome'],'SourceSegmentRefusal')
            self.assertEqual(r['history'][0]['source_refusal'],detail)
            self.assertEqual(len(r['history']),2);self.assertFalse(r['proposal_collision_passed'])
        finally:
            if host is not None:host.close()
            cli._surface_assurance.reset(token)
    def test_authority_and_resource_exceptions_stay_fatal(self):
        class Failure:
            def __init__(self,reason):self.reason=reason
            def qualify_segments(self,*a,**kw):raise ValueError(self.reason)
        for reason in ('ChangedSourceSegmentInputs','OperationalResourceLimit','ProtectedGeometryQueries'):
            with self.assertRaisesRegex(ValueError,reason):
                segment_guard(Failure(reason),None)(np.zeros((2,1,3)))

if __name__=='__main__':unittest.main()
