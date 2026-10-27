"""Real closed fixed-boundary cavity controls; no simulated geometry verdicts."""
from pathlib import Path
import sys,tempfile,unittest,copy
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));import geometry
from core_connectivity import repair,Policy,BoundedTopology,stage_commit
from background_topology_repair import orient,boundary
from background_materialization import build
from finite_support.transition_front_targets import tetrahedron_ratio
import local_termination_topology as topology
from sdf_cfd_mesher_v4.common.authority import identity

def bipyramid(thin=.03,outsider=False):
    x=np.array([[0,0,0],[1,0,0],[.5,3**.5/2,0],[.5,3**.5/6,thin],[.5,3**.5/6,-.8]],dtype='<f8')
    cells=build(np.asarray(orient(x,[[0,1,2,3],[0,2,1,4]]),dtype='<i8'),0)
    # A real stationary halo gives every cavity face its actual outside owner.
    # An isolated 3-tet bipyramid has two uncoupled boundary faces per tet and
    # correctly fails Foundation's rank-three cell requirement.
    exterior=boundary(cells)[1];target_volume=(.8+thin)*3**.5/36
    for face in exterior:
        p=x[list(face)];n=np.cross(p[1]-p[0],p[2]-p[0]);area=np.linalg.norm(n)/2;n/=2*area
        height=1e-5 if outsider and set(face)=={0,1,3} else 3*target_volume/area
        vertex=len(x);x=np.vstack((x,p.mean(0)+height*n))
        cells+=build(np.asarray(orient(x,[[*face,vertex]]),dtype='<i8'),len(cells))
    far=boundary(cells)[1];owners,incidence=topology.ownership(cells,far,[])
    assert incidence['passed']
    return x,cells,owners,far

class Connectivity(unittest.TestCase):
    def run_repair(self,x,cells,owners,far,ids=(0,1),**kw):
        with tempfile.TemporaryDirectory() as tmp:
            return repair(x,cells,owners,ids,far,Path(tmp)/'repair',minimum=.25,**kw)
    def test_fixed_boundary_positive_transaction(self):
        x,c,o,f=bipyramid();original=(x.tobytes(),identity(c),identity({str(k):v for k,v in o.items()}))
        s,r=self.run_repair(x,c,o,f)
        self.assertTrue(r['construction_passed']);self.assertTrue(r['all_bulk_above_floor'])
        self.assertEqual(len(r['commits']),1);self.assertEqual(r['commits'][0]['operation'],'2to3')
        self.assertEqual(len(s.core_ids),3);self.assertGreater(r['final_minimum'],.67)
        self.assertTrue(r['commits'][0]['changed_vs_all']['independent_coverage_passed'])
        self.assertEqual(x.tobytes(),s.points.tobytes())
        self.assertEqual(original,(x.tobytes(),identity(c),identity({str(k):v for k,v in o.items()})))
        self.assertEqual(boundary(c)[0],boundary(list(s.byid.values()))[0])
    def test_noop_keeps_original_connectivity(self):
        x,c,o,f=bipyramid(thin=.5);s,r=self.run_repair(x,c,o,f)
        self.assertEqual(r['outcome'],'AlreadyAboveFloor');self.assertEqual(r['history'],[])
        self.assertEqual(identity(c),identity(list(s.byid.values())))
    def test_single_fixed_bad_tet_refuses_without_edit(self):
        x,c,o,f=bipyramid();c=c[:1];f=boundary(c)[1];o,_=topology.ownership(c,f,[])
        s,r=self.run_repair(x,c,o,f,ids=(0,))
        self.assertFalse(r['construction_passed']);self.assertEqual(r['commits'],[])
        self.assertEqual(identity(c),identity(list(s.byid.values())))
        self.assertEqual(r['outcome'],'FiniteConnectivityFamilyRefusal')
    def test_actual_stationary_outsider_blocks_new_quality_failure(self):
        x,c,o,f=bipyramid(outsider=True);s,r=self.run_repair(x,c,o,f)
        self.assertFalse(r['construction_passed']);self.assertEqual(r['commits'],[])
        self.assertTrue(any(a['outcome']=='ActualNeighbourQualityRefusal' for a in r['history']))
        self.assertEqual(identity(c),identity(list(s.byid.values())))
    def test_nonlocal_stationary_conflict_is_rejected_after_local_qa(self):
        # Deliberately invalid initial embedding for a negative guard control.
        # A disconnected stationary tetrahedron crosses a new internal face.
        x,c,o,f=bipyramid();centre=x[[3,4,0]].mean(0);n=len(x)
        x=np.vstack((x,centre+.025*np.array([[1,1,1],[-1,-1,1],[-1,1,-1],[1,-1,-1]])))
        c+=build(np.asarray(orient(x,[list(range(n,n+4))]),dtype='<i8'),len(c))
        f=boundary(c)[1];o,_=topology.ownership(c,f,[])
        s,r=self.run_repair(x,c,o,f)
        self.assertFalse(r['construction_passed']);self.assertEqual(r['commits'],[])
        rows=[a for a in r['history'] if a['outcome']=='ChangedVsAllCollisionRefusal']
        self.assertTrue(rows);self.assertTrue(rows[0]['quality']['passed'])
        self.assertTrue(rows[0]['before_geometry']['passed']);self.assertTrue(rows[0]['after_geometry']['passed'])
        self.assertFalse(rows[0]['changed_vs_all']['passed'])
        self.assertEqual(identity(c),identity(list(s.byid.values())))
    def test_changed_boundary_duplicate_and_degenerate_refuse_without_mutation(self):
        x,c,o,f=bipyramid();s=BoundedTopology(x,c,o,[0,1],f,policy=Policy());stamp=identity(c)
        base=dict(operation='control',remove=[0,1],insert=[(3,4,0,1),(3,4,1,2)])
        with self.assertRaisesRegex(ValueError,'ChangedOrientedCavityBoundary'):s.view(base)
        p=dict(base,insert=[(3,4,0,1),(3,4,1,2),(3,4,2,0),(3,4,0,1)])
        with self.assertRaisesRegex(ValueError,'DuplicateReplacement'):s.view(p)
        with self.assertRaisesRegex(ValueError,'DegenerateReplacement'):s.view(dict(base,insert=[(0,0,1,2)]))
        self.assertEqual(stamp,identity(list(s.byid.values())))
    def test_candidate_budget_and_identity_failure_are_not_acceptance(self):
        x,c,o,f=bipyramid(outsider=True);s,r=self.run_repair(x,c,o,f,policy=Policy(max_candidates=1))
        self.assertFalse(r['construction_passed']);self.assertEqual(r['outcome'],'ConnectivityCandidateBudget')
        def changed():raise ValueError('AuthenticationFailure')
        with self.assertRaisesRegex(ValueError,'AuthenticationFailure'):self.run_repair(x,c,o,f,check=changed)
        self.assertEqual(identity(c),identity(list(s.byid.values())))
    def test_noop_time_budget_still_vetoes_success(self):
        x,c,o,f=bipyramid(thin=.5);s,r=self.run_repair(x,c,o,f,policy=Policy(seconds=1e-10))
        self.assertTrue(r['all_bulk_above_floor']);self.assertFalse(r['construction_passed'])
        self.assertEqual(r['outcome'],'ConnectivityTimeBudget')
    def test_staged_commit_keeps_parent_state_untouched(self):
        x,c,o,f=bipyramid();s=BoundedTopology(x,c,o,[0,1],f,policy=Policy())
        p=next(p for p in s.inventory(0) if p['operation']=='2to3');v=s.view(p)
        new=stage_commit(s,v);self.assertEqual(len(s.core_ids),2);self.assertEqual(len(new.core_ids),3)
        self.assertEqual(identity(c),identity(list(s.byid.values())))
    def test_invalid_budgets(self):
        for kw in ({'max_candidates':False},{'max_commits':0},{'max_edge_cells':9},{'max_cavity_cells':41},{'seconds':float('nan')}):
            with self.assertRaises(ValueError):Policy(**kw)

if __name__=='__main__':unittest.main()
