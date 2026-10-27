import sys,unittest,copy,tempfile,json
from pathlib import Path
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planner import Planner,Request
from plan_evidence import plan_and_capture
from test_components import closed_control


class TerminalBRepair(unittest.TestCase):
    def setup_control(self):
        p=Planner(Request(.000166527069,1.163359674,5));s=closed_control(p)
        real=p.audit
        def residual(state,h):
            result=copy.deepcopy(real(state,h));result['numerical_feasible']=False
            result['complete']['all_B_numerical_pass']=False
            result['complete']['rows'][-1]['failures_B']['determinant']=2
            return result
        values=np.ones(len(s.faces));values[[1,3]]=0
        p.role_determinant=Mock(return_value=values)
        return p,s,real,residual

    def test_passing_schedule_is_unchanged_without_fallback(self):
        p=Planner(Request(.000166527069,1.163359674,5));s=closed_control(p)
        p.base._select_independent_late_candidate=Mock(side_effect=AssertionError('unexpected fallback'))
        result=p.plan(s)
        np.testing.assert_array_equal(result['schedule'],np.broadcast_to(p.request.nominal,(14,5)))
        self.assertTrue(result['numerical_feasible']);self.assertNotIn('selection_repair',result)

    def test_terminal_B_trigger_uses_existing_fallback_once_and_captures_both(self):
        p,s,real,residual=self.setup_control();calls=[]
        def audit(state,h):
            calls.append(h.copy());return residual(state,h) if len(calls)==1 else real(state,h)
        p.audit=audit
        def fallback(*args):
            self.assertEqual(args[0],[1,3]);self.assertEqual(args[-2],4);self.assertIs(args[-1],s)
            h=args[4].copy();h[:,3:]*=.99
            return {'pass':True,'proposal':h,'attempts':[{'start':'nominal'}]}
        p.base._select_independent_late_candidate=Mock(side_effect=fallback)
        with tempfile.TemporaryDirectory() as tmp:
            result=plan_and_capture(p,s,Path(tmp))
            self.assertTrue(result['numerical_feasible']);self.assertFalse(result['mesh_accepted'])
            self.assertEqual(result['selection_repair']['outcome'],'TerminalBRepairPassed')
            self.assertEqual(p.base._select_independent_late_candidate.call_count,1)
            self.assertEqual(p.role_determinant.call_args.args[1:],(True,False))
            np.testing.assert_array_equal(result['schedule'][:,:3],calls[0][:,:3])
            _,baseline=p.state_type.load(Path(tmp)/'plan_evidence/baseline_proposal')
            np.testing.assert_array_equal(baseline,calls[0])
            public=json.loads((Path(tmp)/'PRISM_PLAN.json').read_text())
            self.assertNotIn('baseline_schedule',public['selection_repair'])

    def test_fallback_refusal_remains_refusal(self):
        p,s,real,residual=self.setup_control();p.audit=residual
        p.base._select_independent_late_candidate=Mock(return_value={'pass':False,'reason':'IndependentLateSearchUnresolved'})
        result=p.plan(s)
        self.assertFalse(result['numerical_feasible'])
        self.assertEqual(result['selection_repair']['outcome'],'TerminalBRepairUnresolved')
        self.assertEqual(p.base._select_independent_late_candidate.call_count,1)

    def test_fallback_pass_cannot_override_full_audit_refusal(self):
        p,s,real,residual=self.setup_control();p.audit=residual
        h=np.broadcast_to(p.request.nominal,(len(s.wall),5)).copy()
        p.base._select_independent_late_candidate=Mock(return_value={'pass':True,'proposal':h})
        result=p.plan(s)
        self.assertFalse(result['numerical_feasible'])
        self.assertEqual(result['selection_repair']['outcome'],'TerminalBRepairFullAuditRefusal')

    def test_C_refusal_and_no_compression_do_not_trigger(self):
        p,s,real,residual=self.setup_control()
        def failC(state,h):
            result=residual(state,h);result['complete']['all_C_numerical_pass']=False;return result
        p.audit=failC;p.base._select_independent_late_candidate=Mock(side_effect=AssertionError('unexpected fallback'))
        self.assertFalse(p.plan(s)['numerical_feasible'])
        p.audit=residual
        self.assertFalse(p.plan(s,compress=False)['numerical_feasible'])
        self.assertEqual(p.base._select_independent_late_candidate.call_count,0)

    def test_other_B_failures_do_not_trigger(self):
        for layer,key in [(2,'determinant'),(4,'volume')]:
            p,s,real,residual=self.setup_control()
            def other(state,h):
                result=residual(state,h);result['complete']['rows'][layer]['failures_B'][key]=1;return result
            p.audit=other;p.base._select_independent_late_candidate=Mock(side_effect=AssertionError('unexpected fallback'))
            self.assertFalse(p.plan(s)['numerical_feasible'])
            self.assertEqual(p.base._select_independent_late_candidate.call_count,0)

    def test_fallback_cannot_change_frozen_rows_or_increase(self):
        for layer,factor in [(2,.9),(4,1.01)]:
            p,s,real,residual=self.setup_control();p.audit=residual
            h=np.broadcast_to(p.request.nominal,(len(s.wall),5)).copy();h[0,layer]*=factor
            p.base._select_independent_late_candidate=Mock(return_value={'pass':True,'proposal':h})
            with self.assertRaisesRegex(ValueError,'TerminalBRepairScheduleContractViolation'):p.plan(s)


if __name__=='__main__':unittest.main()
