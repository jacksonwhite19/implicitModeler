"""Instrumentation controls only; no scientific geometry qualification claim."""
import sys,json,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
from tet_layer_row import propose_row,RowConflict
from tet_layer_planning import kernel,complete_rows
from transition_construction_evidence import emit,plain

class DiagnosticRecords(unittest.TestCase):
    def setUp(self):
        self.p=np.array([[1.,1.,1.],[-1.,-1.,1.],[-1.,1.,-1.],[1.,-1.,-1.]])
        self.f=np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])
        tri=self.p[self.f];s=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);self.n=s/np.linalg.norm(s,axis=1)[:,None]
        self.c=tri.mean(1);self.area=np.linalg.norm(s,axis=1)/2
        self.controls=dict(rows=1,maxiter=30,seconds=10,maximum_distance_m=10.)
        self.events=[]
        def check():pass
        check.repair_diagnostics=lambda event,payload:self.events.append((event,payload))
        check.transition_diagnostic_context=dict(vertex_ids=['v0','v1','v2','v3'])
        self.check=check
    def test_kernel_mean_success_record(self):
        p,r=kernel(self.p,self.f,self.controls,self.check)
        self.assertEqual(self.events[-1][1]['outcome'],'success');self.assertTrue(np.array_equal(p,[0.,0.,0.]))
        json.dumps(self.events,allow_nan=False)
    def test_lp_failure_status_message_and_operands(self):
        fit=SimpleNamespace(success=False,status=1,message='Iteration limit reached',nit=30,x=None)
        with patch('scipy.optimize.linprog',return_value=fit),self.assertRaises(RowConflict) as e:
            kernel(self.p,self.f[:,::-1],self.controls,self.check)
        self.assertEqual(e.exception.reason,'CapKernelProposalConflict')
        r=self.events[-1][1];self.assertEqual(r['solver']['status'],1);self.assertIn('plane_bound_normalized',r)
        self.assertEqual(r['context']['vertex_ids'][3],'v3');json.dumps(self.events,allow_nan=False)
    def test_real_shared_interval_failure_record(self):
        dp=np.array([.01,.00001,1.,.00001]);owners=self.c-dp[:,None]*self.n;vol=self.area*dp/3
        with self.assertRaises(RowConflict) as e:propose_row(self.p,self.f,owners,vol,self.controls,check=self.check)
        self.assertEqual(e.exception.reason,'SharedActualOwnerIntervalConflict')
        r=self.events[-1][1];self.assertEqual(r['outcome'],'failure');self.assertTrue(r['witnesses'])
        for w in r['witnesses']:self.assertLess(w['margin_m'],0.)
        self.assertEqual(r['actual_owner_centres'],owners.tolist());json.dumps(self.events,allow_nan=False)
    def test_real_success_matches_without_sink(self):
        owners=self.c-.01*self.n;vol=self.area*.01/3
        a=propose_row(self.p,self.f,owners,vol,self.controls,check=self.check)
        b=propose_row(self.p,self.f,owners,vol,self.controls)
        self.assertEqual(a['front'].tobytes(),b['front'].tobytes())
        self.assertIn('actual_owner_row_success',[e for e,_ in self.events]);json.dumps(self.events,allow_nan=False)
    def test_fallback_failure_keeps_both_reasons(self):
        with patch('tet_layer_planning.kernel',side_effect=RowConflict('CapKernelProposalConflict',status=1)),patch('tet_layer_planning.propose_row',side_effect=RowConflict('SharedActualOwnerIntervalConflict')),self.assertRaises(RowConflict) as e:
            complete_rows(self.p,self.f,self.c-.01*self.n,self.area*.01/3,self.controls,check=self.check)
        self.assertEqual(e.exception.reason,'SharedActualOwnerIntervalConflict')
        self.assertEqual(e.exception.details['current_front_kernel']['details']['status'],1)
        self.assertEqual(e.exception.details['radial_fallback']['reason'],'NoCapKernelProposal')
        self.assertIn('transition_initialization_operands',[e for e,_ in self.events]);json.dumps(self.events,allow_nan=False)
    def test_sink_failure_is_not_swallowed(self):
        def fail(*a):raise OSError('diagnostic storage unavailable')
        self.check.repair_diagnostics=fail
        with self.assertRaises(OSError):kernel(self.p,self.f,self.controls,self.check)
    def test_nonfinite_encoding_is_strict_json(self):
        r=plain(np.array([np.inf,-np.inf,np.nan]));self.assertEqual(r[0],{'float64_hex':'inf'});json.dumps(r,allow_nan=False)

if __name__=='__main__':unittest.main()
