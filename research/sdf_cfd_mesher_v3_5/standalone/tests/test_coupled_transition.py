"""Real owner geometry and unchanged QA controls; open patch is not qualification."""
import sys,unittest,json
from pathlib import Path
from collections import defaultdict
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));import geometry
import local_termination_topology as t
from background_mixed_geometry import batch_geometry
from background_materialization import build
from tet_layer_row import propose_row,RowConflict,product,signed_six,_descending
from tet_layer_adapter import quality
from tet_layer_planning import complete_rows
from coupled_transition_row import propose,configuration,validate_policy,validate_endpoint
from transition_construction_evidence import plain

def folded():
    theta=np.radians(100.);r=5.75
    p=np.array([[0.,0.,0.],[1.,0.,0.],[-r*np.cos(theta),r*np.sin(theta),0.],[0.,0.,1.]])
    f=np.array([[0,1,3],[0,3,2]])
    u=np.array([-np.sin(theta),-1-np.cos(theta),0.]);u*=.2/np.linalg.norm(u)
    points=np.r_[p-u,p];cells=[t.cell(np.r_[face,face+4],i,1) for i,face in enumerate(f)]
    g=batch_geometry(points,cells);centres=np.array([g[c['id']]['centre'] for c in cells]);vol=np.array([g[c['id']]['volume'] for c in cells])
    return p,f,points,cells,centres,vol

class CoupledRow(unittest.TestCase):
    def setUp(self):
        self.c=dict(rows=1,maxiter=30,seconds=30.,maximum_distance_m=1.,ftol=1e-10)
        self.p,self.f,self.points,self.cells,self.owners,self.vol=folded();self.events=[];self.models=[]
        def check():pass
        check.repair_diagnostics=lambda e,p:self.events.append((e,p))
        check.coupled_vertex_trace=lambda e,p:self.models.append((e,p))
        self.check=check
    def call(self,**kwargs):
        return propose(self.p,self.f,self.owners,self.vol,self.c,policy=configuration(),check=self.check,**kwargs)
    def test_actual_prism_fixed_ray_conflict_free_vector_and_full_patch_QA(self):
        with self.assertRaises(RowConflict) as error:propose_row(self.p,self.f,self.owners,self.vol,self.c)
        self.assertEqual(error.exception.reason,'SharedActualOwnerIntervalConflict')
        before=self.p.tobytes();r=self.call();self.assertEqual(self.p.tobytes(),before)
        self.assertTrue(np.all(signed_six(self.p,r['front'],self.f)>0))
        pts=np.r_[self.points,r['front']];adds=build(product(self.f,4)+4,2000000);cells=self.cells+adds;owners=defaultdict(list)
        for cell in cells:
            for face in cell['faces']:owners[tuple(sorted(face))].append((cell['id'],face))
        outside=[rows[0][1] for rows in owners.values() if len(rows)==1]
        # All actual neighbours checked; prospective outside mask supplies NO
        # fictitious neighbour metrics. This open patch is not a complete mesh.
        q=quality(pts,cells,owners,{c['id'] for c in cells},outside)
        self.assertTrue(q['passed'],q);self.assertGreaterEqual(q['summary']['minimum_determinant'],.0011)
        self.assertEqual(q['summary']['exact_nonpositive_cells'],0)
        self.assertEqual(len({v for row in product(self.f,4) for v in row if v>=4}),4)
        self.assertEqual(len(self.models),4);self.assertTrue(all('A_ub' in x for _,x in self.models))
        json.dumps(plain(self.events),allow_nan=False)
    def test_real_coplanar_actual_owner_slabs_are_incompatible(self):
        p=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[1.,1.,0.]]);f=np.array([[0,1,3],[0,3,2]])
        wall=p.copy();wall[:,2]-=np.array([1e-6,1.,1e-6,1e-6]);points=np.r_[wall,p]
        cells=[t.cell(np.r_[face,face+4],i,1) for i,face in enumerate(f)];g=batch_geometry(points,cells)
        o=np.array([g[c['id']]['centre'] for c in cells]);v=np.array([g[c['id']]['volume'] for c in cells])
        with self.assertRaises(RowConflict) as e:propose(p,f,o,v,dict(self.c,maximum_distance_m=10.),policy=configuration(),check=self.check)
        self.assertEqual(e.exception.reason,'CoupledSeedSearchRefusal');self.assertEqual(e.exception.details['status'],2)
        self.assertIn('coupled_seed_refusal',[e for e,_ in self.events])
    def test_positive_initial_derivatives_do_not_hide_folded_endpoint(self):
        p=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]);f=np.array([[0,1,2]]);goal=p+np.array([[3.,3.,.1],[0.,0.,.1],[0.,0.,.1]])
        self.assertTrue(np.all((goal-p)[:,2]>0));self.assertTrue(np.any(signed_six(p,goal,f)<=0))
        def tick():pass
        tick.remaining_seconds=lambda:30.
        new,records=_descending(p,f,goal,p,10.,None,np.array([[0.,0.,1.]]),p[f].mean(1),np.array([.01]),np.array([1.]),30,tick)
        self.assertTrue(np.all(signed_six(p,new,f)>0));self.assertTrue(any(x['used_lp'] for x in records))
    def test_legacy_success_is_byte_preserved(self):
        p=np.array([[1.,1.,1.],[-1.,-1.,1.],[-1.,1.,-1.],[1.,-1.,-1.]]);f=np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])
        pts=np.r_[.99*p,p];cells=[t.cell(np.r_[face,face+4],i,1) for i,face in enumerate(f)];g=batch_geometry(pts,cells)
        o=np.array([g[c['id']]['centre'] for c in cells]);v=np.array([g[c['id']]['volume'] for c in cells]);c=dict(self.c,maximum_distance_m=10.)
        a=complete_rows(p,f,o,v,c)
        with patch('coupled_transition_row.propose',side_effect=AssertionError('legacy success reached new policy')):b=complete_rows(p,f,o,v,c,_proposal_policy=configuration())
        self.assertEqual(a[0].tobytes(),b[0].tobytes())
    def test_explicit_route_and_shared_endpoint(self):
        # Routing control only: real vector and actual QA are tested above.
        with patch('tet_layer_planning.kernel',side_effect=RowConflict('CapKernelProposalConflict')):
            r=complete_rows(self.p,self.f,self.owners,self.vol,self.c,_proposal_policy=configuration())
        self.assertEqual(r[0].shape,(2,4,3));self.assertEqual(r[1]['rows'][0]['method'],'V35CoupledSharedDisplacement/1')
    def test_post_rounding_validator_rejects_changed_endpoint(self):
        r=self.call();bad=r['front'].copy();bad[3]=self.p[3]
        with self.assertRaises(RowConflict) as e:validate_endpoint(self.p,bad,self.f,self.owners,self.vol,self.c,self.p,0,check=self.check)
        self.assertEqual(e.exception.reason,'CoupledPostRoundingEndpointRefusal')
    def test_post_rounding_guard_precedes_row_finish(self):
        r=self.call();bad=r['front'].copy();bad[3]=self.p[3]
        with patch('tet_layer_planning.kernel',side_effect=RowConflict('CapKernelProposalConflict')),patch('front_owner_endpoint.close_weight',return_value=(bad,{})),self.assertRaises(RowConflict) as e:
            complete_rows(self.p,self.f,self.owners,self.vol,self.c,_proposal_policy=configuration(),row_finish=lambda *a:(_ for _ in ()).throw(AssertionError('bad rounded endpoint advanced')))
        self.assertEqual(e.exception.reason,'CoupledPostRoundingEndpointRefusal')
    def test_identity_nonfinite_and_malformed_refuse(self):
        bad=configuration();bad['candidates_per_row']=2
        with self.assertRaises(ValueError):propose(self.p,self.f,self.owners,self.vol,self.c,policy=bad)
        for key in ['coordinates','owner','volume','face_type','duplicate']:
            p=self.p.copy();f=self.f.copy();o=self.owners.copy();v=self.vol.copy()
            if key=='coordinates':p[0,0]=np.nan
            if key=='owner':o[0,0]=np.inf
            if key=='volume':v[0]=0
            if key=='face_type':f=f.astype(float)+.1
            if key=='duplicate':f=np.r_[f,f[:1]];o=np.r_[o,o[:1]];v=np.r_[v,v[:1]]
            with self.subTest(key=key),self.assertRaises(RowConflict):propose(p,f,o,v,self.c,policy=configuration())
    def test_authority_storage_and_solver_errors_propagate(self):
        for kind in ['authority','storage','solver']:
            def fail(*a,**k):raise OSError(kind)
            check=self.check
            if kind=='authority':check=fail
            if kind=='storage':check.repair_diagnostics=fail
            context=patch('scipy.optimize.linprog',side_effect=fail) if kind=='solver' else patch('builtins.id',wraps=id)
            with context,self.assertRaises(OSError):propose(self.p,self.f,self.owners,self.vol,self.c,policy=configuration(),check=check)
            if kind=='storage':self.check.repair_diagnostics=lambda e,p:None

if __name__=='__main__':unittest.main()
