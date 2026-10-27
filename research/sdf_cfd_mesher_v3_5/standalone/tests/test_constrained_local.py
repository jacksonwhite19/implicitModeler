"""Real moved/inserted interior controls and exact fixed-boundary refusals."""
from pathlib import Path
import sys,tempfile,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));import geometry
from background_topology_repair import orient,boundary
from background_materialization import build
import local_termination_topology as topology
from constrained_local import Policy,CavityTopology,map_candidate,admit,cavity_ball
from sdf_cfd_mesher_v4.common.authority import identity

def octahedron():
    x=np.asarray([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1],[.85,0,0]],dtype='<f8')
    triangles=[[a,b,c] for a in (0,1) for b in (2,3) for c in (4,5)]
    cells=build(np.asarray(orient(x,[[6,*f] for f in triangles]),dtype='<i8'),0)
    faces=boundary(cells)[1]
    # Stationary halo closes actual-neighbour context and avoids isolated
    # multi-boundary-face rank defects in Foundation's cell-quality contract.
    for face in faces:
        p=x[list(face)];n=np.cross(p[1]-p[0],p[2]-p[0]);n/=np.linalg.norm(n)
        v=len(x);x=np.vstack((x,p.mean(0)+n*.6));cells+=build(np.asarray(orient(x,[[*face,v]]),dtype='<i8'),len(cells))
    far=boundary(cells)[1];owners,incidence=topology.ownership(cells,far,[]);assert incidence['passed']
    s=CavityTopology(x,cells,owners,range(8),far)
    return s,cells[:8],faces

def candidate(s,insert=False):
    x=s.points[:7].copy();x[6]=0;rows=np.asarray([s.byid[i]['vertices'] for i in range(8)],dtype='<i8')
    if insert:
        p=x[rows[0]].mean(0);x=np.vstack((x,p));c=build(np.asarray(orient(x,rows[:1]),dtype='<i8'),0)[0]
        rows=np.vstack((rows[1:],np.asarray(orient(x,[[7,*f] for f in c['faces']]),dtype='<i8')))
    return x,rows

class Constrained(unittest.TestCase):
    def test_complete_moved_interior_transaction(self):
        s,old,faces=octahedron();x,t=candidate(s);before=(s.points.tobytes(),identity(list(s.byid.values())))
        with tempfile.TemporaryDirectory() as tmp:new,r=admit(s,old,faces,x,t,Path(tmp)/'txn',.25,Policy(),lambda:None)
        self.assertIsNotNone(new);self.assertEqual(r['outcome'],'Committed');self.assertEqual(r['point_mapping']['moved_vertices'],[6]);self.assertEqual(len(new.points),len(s.points))
        self.assertTrue(r['changed_vs_all']['independent_coverage_passed']);self.assertTrue(r['before_geometry']['passed']);self.assertTrue(r['after_geometry']['passed'])
        self.assertEqual(before,(s.points.tobytes(),identity(list(s.byid.values()))));self.assertEqual(boundary(list(s.byid.values()))[0],boundary(list(new.byid.values()))[0])
    def test_inserted_point_complete_transaction(self):
        s,old,faces=octahedron();x,t=candidate(s,True)
        with tempfile.TemporaryDirectory() as tmp:new,r=admit(s,old,faces,x,t,Path(tmp)/'txn',.25,Policy(),lambda:None)
        self.assertIsNotNone(new);self.assertEqual(len(new.points),len(s.points)+1);self.assertEqual(r['point_mapping']['inserted_vertices'],[len(s.points)])
        self.assertTrue(r['point_mapping']['strict_inclusion']['passed']);self.assertEqual(r['outcome'],'Committed')
    def test_boundary_coordinate_refuses_without_mutation(self):
        s,old,faces=octahedron();x,t=candidate(s);before=s.points.tobytes();x[0,0]+=.001
        with self.assertRaisesRegex(ValueError,'ChangedFixedCavityCoordinate'):map_candidate(s,old,faces,x,t,Policy())
        self.assertEqual(s.points.tobytes(),before)
    def test_outside_new_site_refuses(self):
        s,old,faces=octahedron();x,t=candidate(s);x[6]=[5,5,5]
        with self.assertRaisesRegex(ValueError,'NewPointNotStrictlyInOldCavity'):map_candidate(s,old,faces,x,t,Policy())
    def test_duplicate_and_unused_points_refuse(self):
        s,old,faces=octahedron();x,t=candidate(s)
        for xx in (np.vstack((x,x[0])),np.vstack((x,[.1,.1,.1]))):
            with self.assertRaisesRegex(ValueError,'DuplicateOrUnusedConstrainedPoint'):map_candidate(s,old,faces,xx,t,Policy())
    def test_stranded_old_interior_slot_refuses(self):
        s,old,faces=octahedron();x=s.points[:6];ring=[2,4,3,5]
        rows=np.asarray([[0,1,ring[i],ring[(i+1)%4]] for i in range(4)])
        with self.assertRaisesRegex(ValueError,'StrandedOriginalInteriorPoint'):map_candidate(s,old,faces,x,rows,Policy())
    def test_incomplete_movable_star_refuses(self):
        s,old,faces=octahedron();x,t=candidate(s);n=len(s.points)
        s.points=np.vstack((s.points,[[4,0,0],[4,1,0],[4,0,1]]));s.byid[100]=build(np.asarray([[6,n,n+1,n+2]]),100)[0]
        with self.assertRaisesRegex(ValueError,'IncompleteMovableVertexStar'):map_candidate(s,old,faces,x,t,Policy())
    def test_boundary_facet_change_refuses(self):
        s,old,faces=octahedron();x,t=candidate(s);xx,tt,_=map_candidate(s,old,faces,x,t[:-1],Policy())
        view=CavityTopology(xx,list(s.byid.values()),s.owners,s.core_ids,s.far)
        with self.assertRaisesRegex(ValueError,'ChangedOrientedCavityBoundary'):view.view(dict(remove=list(range(8)),insert=tt))
    def test_nonpassing_complete_candidate_never_commits(self):
        s,old,faces=octahedron();x=s.points[:7].copy();t=np.asarray([c['vertices'] for c in old]);before=s.points.tobytes()
        with tempfile.TemporaryDirectory() as tmp:new,r=admit(s,old,faces,x,t,Path(tmp)/'txn',.25,Policy(),lambda:None)
        self.assertIsNone(new);self.assertEqual(r['outcome'],'BulkShapeRefusal');self.assertEqual(s.points.tobytes(),before)
    def test_complete_cavity_size_bound_is_not_truncation(self):
        s,old,faces=octahedron();self.assertIsNone(cavity_ball(s,0,4,Policy(max_cavity_cells=2)))
    def test_native_intermediate_rejects_bad_boundary_and_duplicates(self):
        from constrained_local_worker import intermediate
        s,old,faces=octahedron();x,t=candidate(s);bf=np.asarray(faces)
        good=intermediate(x,t,x[:6],bf);self.assertEqual(len(good[0]),8)
        with self.assertRaisesRegex(ValueError,'DuplicateNativeTetrahedra'):intermediate(x,np.vstack((t,t[0])),x[:6],bf)
        with self.assertRaisesRegex(ValueError,'ChangedNativeOrientedBoundary'):intermediate(x,t[:-1],x[:6],bf)
    def test_invalid_or_expanded_budgets_refuse(self):
        for args in ({'rings':(4,1)},{'max_cavity_cells':257},{'point_rules':('unknown',)},{'seconds':float('nan')},{'seconds':7201},{'native_seconds':361}):
            with self.assertRaises(ValueError):Policy(**args)
    def test_commit_budget_does_not_claim_family_exhaustion(self):
        from test_core_connectivity import bipyramid
        from constrained_local import repair
        from sdf_cfd_mesher_v4.common.authority import file_hash
        x,a,_,_=bipyramid();y,b,_,_=bipyramid();nv=len(x);nc=len(a);y=y+[10,0,0]
        b=build(np.asarray([[v+nv for v in c['vertices']] for c in b],dtype='<i8'),nc)
        xx=np.vstack((x,y));cells=a+b;far=boundary(cells)[1];owners,_=topology.ownership(cells,far,[])
        s=CavityTopology(xx,cells,owners,[0,1,nc,nc+1],far)
        def proposer(s,old,faces,seed,rule,folder,policy,check):
            folder.mkdir();vs=np.asarray(sorted({v for c in old for v in c['vertices']}))
            vertices=[3,4,0,1,2] if seed<nc else [v+nv for v in (3,4,0,1,2)]
            u,v,a,b,c=vertices;rows=np.searchsorted(vs,np.asarray([[u,v,a,b],[u,v,b,c],[u,v,c,a]]))
            path=folder/'optimized.npz';np.savez_compressed(path,points=s.points[vs],tetrahedra=rows)
            return dict(stages=[dict(stage='optimized',intermediate_passed=True,sha256=file_hash(path))])
        with tempfile.TemporaryDirectory() as tmp:new,r=repair(s,Path(tmp)/'run',minimum=.25,policy=Policy(max_commits=1),proposer=proposer)
        self.assertEqual(len(r['commits']),1);self.assertEqual(r['outcome'],'ConstrainedCommitBudget');self.assertFalse(r['family_exhausted']);self.assertEqual(len(r['final_bad_cells']),1)

if __name__=='__main__':unittest.main()
