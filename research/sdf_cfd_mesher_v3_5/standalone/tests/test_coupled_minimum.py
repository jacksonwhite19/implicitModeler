"""Policy-2 physical-domain/scaling controls; no retained full-row qualification."""
import sys,json,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));import geometry
from coupled_displacement_minimum import box,solve,TAU
from coupled_transition_minimum import seed_vertex,configuration,MinimumCompletion,propose
from coupled_transition_row import configuration as v1,propose as dispatch
from tet_layer_row import RowConflict,_descending,signed_six
from tet_layer_planning import complete_rows
from test_coupled_transition import folded

def tick():pass
tick.remaining_seconds=lambda:10.
class MinimumPolicy(unittest.TestCase):
    def fixture(self):
        d=json.loads((Path(__file__).parent/'fixtures/policy1_seed_local_control.json').read_text())
        for k in ['current','original','bounds','normal','lower','upper','offset','known_local_displacement']:d[k]=np.array(d[k])
        return d
    def seed(self,d,factor=1.):
        return seed_vertex(d['current'],d['original'],d['radius'],d['bounds'],d['normal'],d['lower'],d['upper'],d['offset'],d['inc'],d['cap'],d['scale']*factor,30,tick)
    def test_known_witness_is_in_original_domain_and_new_reserve(self):
        d=self.fixture();lo,hi,res=box(d['current'],d['original'],d['radius'],d['bounds']);u=d['known_local_displacement'];point=d['current']+u
        self.assertTrue(np.all(point>=lo) and np.all(point<=hi));self.assertTrue(np.any(abs(u)>d['scale']))
        self.assertTrue(np.all(d['normal']@u>res));h=d['normal'][d['cap']]@u+d['offset'][d['cap']]
        self.assertTrue(np.all(h>=d['lower'][d['cap']]+res));self.assertTrue(np.all(h<=d['upper'][d['cap']]-res))
        new,r=self.seed(d);self.assertIsNotNone(new,r);self.assertTrue(np.any(abs(new-d['current'])>d['scale']))
    def test_variable_rescaling_preserves_physical_domain_and_objective(self):
        d=self.fixture();results=[self.seed(d,f) for f in [.0625,1.,16.]]
        for new,r in results:
            self.assertIsNotNone(new,r);self.assertEqual(r['physical_plane_residual_budget_m'],TAU*d['radius'])
            self.assertTrue(np.all(r['physical_constraint_residual_m']>=-TAU*d['radius']))
        self.assertLess(max(r['L1_m'] for _,r in results)-min(r['L1_m'] for _,r in results),16*TAU*d['radius'])
        for _,r in results[1:]:
            np.testing.assert_array_equal(r['coordinate_lower_m'],results[0][1]['coordinate_lower_m']);np.testing.assert_array_equal(r['physical_b_m'],results[0][1]['physical_b_m'])
    def test_minimum_movement_does_not_maximize_domain_margin(self):
        p=np.zeros(3);normal=np.array([[0.,0.,1.]]);L=np.array([.1]);U=np.array([.5]);off=np.zeros(1)
        new,r=seed_vertex(p,p,1.,None,normal,L,U,off,[0],[0],.01,30,tick)
        self.assertIsNotNone(new);self.assertAlmostEqual(new[0],0.);self.assertAlmostEqual(new[1],0.);self.assertLess(new[2],.10000001)
        self.assertGreater(new[2],.1);self.assertLess(r['L1_m'],.11)
    def test_real_coplanar_geometry_refuses_without_weakening(self):
        import local_termination_topology as t
        from background_mixed_geometry import batch_geometry
        p=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[1.,1.,0.]]);f=np.array([[0,1,3],[0,3,2]]);wall=p.copy();wall[:,2]-=[1e-6,1.,1e-6,1e-6]
        pts=np.r_[wall,p];cells=[t.cell(np.r_[face,face+4],i,1) for i,face in enumerate(f)];g=batch_geometry(pts,cells)
        o=np.array([g[c['id']]['centre'] for c in cells]);v=np.array([g[c['id']]['volume'] for c in cells])
        with self.assertRaises(RowConflict) as e:propose(p,f,o,v,dict(rows=1,maxiter=30,seconds=10.,maximum_distance_m=10.),policy=configuration())
        self.assertEqual(e.exception.reason,'CoupledMinimumSeedRefusal')
    def test_completion_minimizes_from_current_not_folded_seed(self):
        p=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]);f=np.array([[0,1,2]]);goal=p+np.array([[3.,3.,.1],[0.,0.,.1],[0.,0.,.1]])
        normal=np.array([[0.,0.,1.]]);L=np.array([.01]);U=np.array([1.]);records=[]
        callback=MinimumCompletion(p,p,10.,None,np.array([.01]),[[0],[0],[0]],normal,L,U,dict(maxiter=30),tick,lambda e,r:records.append(r))
        new,trace=_descending(p,f,goal,p,10.,None,normal,p[f].mean(1),L,U,30,tick,_minimum_solver=callback)
        self.assertTrue(np.all(signed_six(p,new,f)>0));self.assertTrue(records)
        for record in records:np.testing.assert_array_equal(record['current_m'],p[record['vertex']])
        self.assertLess(np.abs(new[0]-p[0]).sum(),.1)
    def test_solver_tolerance_cannot_promote_nonpositive_endpoint(self):
        fit=SimpleNamespace(success=True,status=0,message='mock optimal endpoint',nit=1,x=np.zeros(6))
        p=np.zeros(3);lo,hi,res=box(p,p,1.,None)
        with patch('coupled_displacement_minimum.linprog',return_value=fit):
            new,r=solve(p,lo,hi,1.,.01,np.array([[0.,0.,-1.]]),np.array([-res]),30,tick,reserve=res)
        self.assertIsNone(new);self.assertTrue(r['numerical_endpoint_refused'])
    def test_policy1_identity_and_coordinates_remain_unchanged(self):
        self.assertEqual(v1()['identity'],'7254868bd99f3889af36ba104237805bc2c2e5e6c5cb6bf6c6a72c8776fc830b')
        p,f,pts,cells,o,v=folded();c=dict(rows=1,maxiter=30,seconds=10.,maximum_distance_m=1.)
        with patch('coupled_transition_minimum.propose',side_effect=AssertionError('policy1 reached policy2')):a=dispatch(p,f,o,v,c,policy=v1())
        self.assertEqual(a['diagnostics']['method'],'V35CoupledSharedDisplacement/1')
    def test_legacy_success_bypasses_policy2(self):
        p,f,pts,cells,o,v=folded();c=dict(rows=1,maxiter=30,seconds=10.,maximum_distance_m=1.,ftol=1e-10)
        a=complete_rows(p,f,o,v,c)
        with patch('coupled_transition_minimum.propose',side_effect=AssertionError('legacy success reached policy2')):b=complete_rows(p,f,o,v,c,_proposal_policy=configuration())
        self.assertEqual(a[0].tobytes(),b[0].tobytes())
    def test_empty_reserve_invalid_scaling_and_errors_refuse(self):
        p=np.zeros(3);lo,hi,res=box(p,p,1.,None)
        with self.assertRaises(RowConflict):solve(p,lo,hi,1.,np.nan,np.array([[0.,0.,1.]]),np.array([1.]),30,tick,reserve=res)
        bad=configuration();bad['primal_dual_tolerance']=1e-7
        with self.assertRaises(ValueError):dispatch(*folded()[:2],folded()[4],folded()[5],dict(rows=1,maxiter=30,seconds=10.,maximum_distance_m=1.),policy=bad)
        def fail():raise TimeoutError('outer deadline')
        fail.remaining_seconds=lambda:0.
        with self.assertRaises(TimeoutError):solve(p,lo,hi,1.,1.,np.array([[0.,0.,1.]]),np.array([1.]),30,fail,reserve=res)
        new,r=solve(p,lo,hi,1.,1.,np.array([[0.,0.,-1.],[0.,0.,1.]]),np.array([-res,0.]),30,tick,reserve=res)
        self.assertIsNone(new)

if __name__=='__main__':unittest.main()
