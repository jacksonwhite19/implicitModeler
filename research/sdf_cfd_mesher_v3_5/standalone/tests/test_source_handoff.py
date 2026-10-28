"""Small live canonical source controls; no historical receipt grants a pass."""
import sys,unittest,json,threading,ctypes,tempfile,subprocess
from pathlib import Path
from unittest.mock import patch
import numpy as np
S=Path(__file__).resolve().parents[1];sys.path.insert(0,str(S))
import geometry,standalone_tool as cli
import collision_policy as collision
from source_handoff import live_handoff,consume

class HandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from planner import Request
        cls.tmp=tempfile.TemporaryDirectory();cls.case=Path(cls.tmp.name)/'CASE.json'
        request=Request(2e-5,1.3,5)
        geometry.write_case(S/'fixtures/v4_case_template.json',S/'fixtures/sphere.rhai',cls.case,request)
        token=cli._surface_assurance.set('numerical')
        try:cls.host,cls.binding=cli.geometry(cls.case)
        finally:cli._surface_assurance.reset(token)
        cls.source=cls.binding.authority
        cls.x=np.asarray([np.eye(3)*(.001+h) for h in np.r_[0,np.cumsum(request.nominal)]]);cls.h=2e-5
    @classmethod
    def tearDownClass(cls):cls.host.close();cls.tmp.cleanup()
    def ready(self,owner,x=None):
        x=self.x if x is None else x
        collision.segment_guard(self.source,self.h,handoff=owner)(x)
        self.assertTrue(owner.accept(x))
    def test_live_same_inputs_exact_output_and_single_consume(self):
        expected=collision._segment_population(self.source,self.h,self.x,True)[1]
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner);actual=consume(owner,self.source,self.h,self.x,'production')
            self.assertEqual(actual,expected);self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
            self.assertEqual(owner.events()[-1]['segments'],15)
            actual[0]['passed']=False;self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
    def test_changed_endpoint_trajectory_population_falls_back(self):
        for change in ('endpoint','earlier_trajectory','order','subset','signedzero'):
            with self.subTest(change=change),live_handoff(self.source,self.h,'production') as owner:
                self.ready(owner);x=self.x.copy()
                if change=='endpoint':x[-1,0,0]=np.nextafter(x[-1,0,0],np.inf)
                if change=='earlier_trajectory':x[2,0,0]=np.nextafter(x[2,0,0],np.inf)
                if change=='order':x=x[:,::-1].copy()
                if change=='subset':x=x[:,:2].copy()
                if change=='signedzero':x[0,0,1]=-0.0
                self.assertIsNone(consume(owner,self.source,self.h,x,'production'))
    def test_mode_height_source_and_budget_changes(self):
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner);self.assertIsNone(consume(owner,self.source,self.h,self.x,'audit'))
        with live_handoff(self.source,self.h,'audit') as owner:
            collision.segment_guard(self.source,self.h,handoff=owner)(self.x)
            self.assertFalse(owner.accept(self.x));self.assertIsNone(consume(owner,self.source,self.h,self.x,'audit'))
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner);self.assertIsNone(consume(owner,self.source,self.h*2,self.x,'production'))
        token=cli._surface_assurance.set('numerical')
        try:host,binding=cli.geometry(self.case)
        finally:cli._surface_assurance.reset(token)
        try:
            with live_handoff(self.source,self.h,'production') as owner:
                self.ready(owner);self.assertIsNone(consume(owner,binding.authority,self.h,self.x,'production'))
        finally:host.close()
        scope=self.source.queries._scope;old=scope._budgets
        try:
            with live_handoff(self.source,self.h,'production') as owner:
                self.ready(owner);object.__setattr__(scope,'_budgets',(old[0],old[1],old[2]-1))
                self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
        finally:object.__setattr__(scope,'_budgets',old)
    def test_changed_fp_and_numpy_context(self):
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner)
            with np.errstate(over='ignore'):self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
        read=ctypes.CDLL('ucrtbase.dll')._controlfp_s;word=ctypes.c_uint();read(ctypes.byref(word),0,0);old=word.value
        try:
            with live_handoff(self.source,self.h,'production') as owner:
                self.ready(owner);read(ctypes.byref(word),0x100,0x300)
                self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
        finally:read(ctypes.byref(word),old,0x300)
    def test_incomplete_proofs_and_failed_source_clear_evidence(self):
        real=collision._segment_population
        def incomplete(*a,**k):
            summary,proofs=real(*a,**k);return summary,proofs[:-1]
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner)
            with patch.object(collision,'_segment_population',incomplete),self.assertRaisesRegex(ValueError,'IncompleteSourceHandoffPopulation'):
                collision.segment_guard(self.source,self.h,handoff=owner)(self.x)
            self.assertFalse(owner.accept(self.x));self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
        bad=self.x.copy();bad[1]=bad[0]-(bad[1]-bad[0])
        before=None
        try:collision.segment_guard(self.source,self.h)(bad)
        except BaseException as failure:before=(type(failure),failure.args,getattr(failure,'detail',None))
        self.assertIsNotNone(before)
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner)
            try:collision.segment_guard(self.source,self.h,handoff=owner)(bad)
            except BaseException as failure:self.assertEqual((type(failure),failure.args,getattr(failure,'detail',None)),before)
            else:self.fail('Expected identical refusal')
            self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
    def test_not_accepted_and_expired_and_rollback(self):
        with live_handoff(self.source,self.h,'production') as owner:
            collision.segment_guard(self.source,self.h,handoff=owner)(self.x)
            self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
            self.ready(owner);owner.discard('independent_collision_refusal')
            self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
        with self.assertRaisesRegex(ValueError,'ExpiredOrDetached'):consume(owner,self.source,self.h,self.x,'production')
        with self.assertRaisesRegex(RuntimeError,'planning_failed'):
            with live_handoff(self.source,self.h,'production') as rolled:
                self.ready(rolled);raise RuntimeError('planning_failed')
        with self.assertRaisesRegex(ValueError,'ExpiredOrDetached'):consume(rolled,self.source,self.h,self.x,'production')
    def test_private_snapshot_and_custom_evaluator(self):
        x=self.x.copy()
        with live_handoff(self.source,self.h,'production') as owner:
            self.ready(owner,x);x[:]=0
            self.assertIsNotNone(consume(owner,self.source,self.h,self.x,'production'))
            with self.assertRaisesRegex(ValueError,'ProtectedSourceHandoff'):owner.accepted=[]
        from general_sdf_authority import GeneralSdfAuthority
        original=GeneralSdfAuthority.qualify_segments;calls=[]
        def custom(source,*a,**k):calls.append(1);return original(source,*a,**k)
        with patch.object(GeneralSdfAuthority,'qualify_segments',custom),live_handoff(self.source,self.h,'production') as owner:
            collision.segment_guard(self.source,self.h,handoff=owner)(self.x)
            self.assertEqual(len(calls),5);self.assertFalse(owner.accept(self.x))
    def test_outer_source_modes_recompute(self):
        from contextlib import ExitStack
        from sdf_cfd_mesher_v4.sdf.regional import exact_fallback
        for mode in ('query','prism','verified_exact'):
            with self.subTest(mode=mode),live_handoff(self.source,self.h,'production') as owner:
                self.ready(owner)
                with ExitStack() as scopes:
                    if mode=='query':scopes.enter_context(self.source.query_scope())
                    else:scopes.enter_context(self.source.prism_query_scope())
                    if mode=='verified_exact':scopes.enter_context(self.source.verified_operation('segments'))
                    def control():
                        self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
                        # The original qualifier remains usable in this mode.
                        original=collision._segment_population(self.source,self.h,self.x,True)[1]
                        self.assertEqual(len(original),5)
                    if mode=='verified_exact':exact_fallback(control)
                    else:control()
    def test_numpy_equal_but_different_callback_identity(self):
        class Handler:
            def __call__(self,*args):pass
            def __eq__(self,other):return True
        old=np.geterrcall();one=Handler();two=Handler()
        try:
            np.seterrcall(one)
            with live_handoff(self.source,self.h,'production') as owner:
                self.ready(owner);np.seterrcall(two)
                self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
        finally:np.seterrcall(old)
    def test_custom_installed_before_consumer_import_is_not_stock(self):
        code="""import geometry
from general_sdf_authority import GeneralSdfAuthority
from sdf_cfd_mesher_v4.sdf.operations import ProtectedQueries
old=GeneralSdfAuthority.qualify_segments
def custom(*a,**k):return old(*a,**k)
GeneralSdfAuthority.qualify_segments=custom
oldout=ProtectedQueries.outward_segment
def customout(*a,**k):return oldout(*a,**k)
ProtectedQueries.outward_segment=customout
import source_handoff
assert source_handoff._METHODS['qualify_segments'] is old
assert source_handoff._METHODS['qualify_segments'] is not custom
assert source_handoff._OUTWARD is oldout
assert source_handoff._OUTWARD is not customout
"""
        run=subprocess.run([sys.executable,'-B','-c',code],cwd=S,capture_output=True,text=True,timeout=30)
        self.assertEqual(run.returncode,0,run.stderr)

if __name__=='__main__':unittest.main()
