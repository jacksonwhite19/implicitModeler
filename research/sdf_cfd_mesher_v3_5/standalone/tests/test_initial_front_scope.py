"""Execute the actual initial mesh block; stock live query guards stay in use.

Small provider/owner seams isolate scope lifetime and exception boundaries.
These fixtures do not issue a mesh or replace any production predicate.
"""
from pathlib import Path
from contextlib import ExitStack
from contextvars import copy_context
from types import SimpleNamespace
import sys,tempfile,textwrap,unittest,threading
from unittest.mock import patch
import numpy as np
S=Path(__file__).resolve().parents[1];sys.path.insert(0,str(S))
import geometry,standalone_tool as cli,general_sdf_authority as owner
from source_handoff import _context

class InitialScope(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from planner import Request
        cls.tmp=tempfile.TemporaryDirectory();cls.case=Path(cls.tmp.name)/'CASE.json'
        geometry.write_case(S/'fixtures/v4_case_template.json',S/'fixtures/sphere.rhai',cls.case,Request(2e-5,1.3,5))
        token=cli._surface_assurance.set('numerical')
        try:cls.host,cls.binding=cli.geometry(cls.case)
        finally:cli._surface_assurance.reset(token)
        cls.source=cls.binding.authority;cls.ref=cls.source._root_ref
        cls.point=np.array([.001,0.,0.]);cls.expected=cls.source.exact_normal(cls.ref,cls.point)
        text=(S/'mesh.py').read_text();start=text.index("            first=out/'first';first.mkdir()")
        end=text.index('            from source_handoff import live_handoff',start)
        cls.block=compile(textwrap.dedent(text[start:end]),str(S/'mesh.py'),'exec')
    @classmethod
    def tearDownClass(cls):cls.host.close();cls.tmp.cleanup()
    def clean(self):
        self.assertIsNone(owner._surface_queries.get());self.assertIsNone(owner._prism_queries.get());self.assertIsNone(owner._verified.get())
        self.assertIsNotNone(_context(self.source))
    def execute(self,*,generation_failure=False,owner_failure=False,mutate_return=False,between=None):
        source=self.source;ref=self.ref;point=self.point;observations={}
        class Provider:
            def __init__(this,*args):
                this.source=source;this.x=np.eye(3)*.001;this.high=this.x.copy();this.tri=np.array([[0,1,2]])
                this.refs={'face_refs':np.array([ref])};this.request=None;this.wall_authority={'surface_identity':'fixture'}
            def _generate_trajectory(this):
                value=source.exact_normal(ref,point)
                if mutate_return:value[:]=123
                if generation_failure:raise RuntimeError('generation fixture refusal')
        class Accepted:
            @staticmethod
            def from_inputs(*args,**kwargs):
                value=source.exact_normal(ref,point);observations['normal']=value
                observations['query']=owner._surface_queries.get()[2]()
                if owner_failure:raise RuntimeError('owner fixture refusal')
                return SimpleNamespace(value=value)
        def read_request(*args):
            if between:between()
            return {}
        with tempfile.TemporaryDirectory() as temp,ExitStack() as life:
            ns=dict(out=Path(temp),wall=None,case=self.case,
                cli=SimpleNamespace(first_request=lambda *args:None,geometry=lambda *args:(SimpleNamespace(close=lambda:None),self.binding)),
                lifetime=life,settings={},request=SimpleNamespace(nominal=np.array([2e-5])),
                SourceSheetFirstLayerProvider=Provider,read_request=read_request,AcceptedTrajectoryOwner=Accepted,
                identity=lambda value:'fixture',stage='first_front',t=0,mark=lambda *args:None)
            exec(self.block,ns)
            observations['initial_query']=observations['query']
            observations['initial_normal']=observations['normal'].copy()
            observations['later_owner']=ns['owner_for'](np.array([2e-5]))
            observations['later_query']=observations['query']
        return observations
    def test_initial_normal_shared_and_later_owner_is_cold(self):
        result=self.execute()
        self.assertEqual(result['initial_query']['counts']['normal_computations'],1)
        self.assertEqual(result['initial_query']['counts']['normal_hits'],1)
        self.assertEqual(result['later_query']['counts']['normal_computations'],1)
        self.assertEqual(result['later_query']['counts'].get('normal_hits',0),0)
        self.assertEqual(result['normal'].tobytes(),self.expected.tobytes());self.clean()
    def test_returned_array_mutation_cannot_change_cached_normal(self):
        result=self.execute(mutate_return=True)
        self.assertEqual(result['initial_normal'].tobytes(),self.expected.tobytes())
        self.assertEqual(result['normal'].tobytes(),self.expected.tobytes());self.clean()
    def test_generation_exception_resets_all_source_contexts(self):
        with self.assertRaisesRegex(RuntimeError,'generation fixture refusal'):self.execute(generation_failure=True)
        self.clean()
    def test_owner_exception_resets_all_source_contexts(self):
        with self.assertRaisesRegex(RuntimeError,'owner fixture refusal'):self.execute(owner_failure=True)
        self.clean()
    def test_changed_fp_mode_refuses_before_nominal_cached_value(self):
        old=np.geterr()
        try:
            with self.assertRaisesRegex(ValueError,'ChangedGeneralQueryArithmeticContext'):
                self.execute(between=lambda:np.seterr(divide='raise'))
        finally:np.seterr(**old)
        self.clean()
    def test_changed_method_refuses_before_nominal_cached_value(self):
        patcher=patch.object(type(self.source),'exact_normal',lambda *args:self.expected.copy())
        try:
            with self.assertRaisesRegex(ValueError,'StockPrismQueryMethodsRequired|ChangedPrismQueryMethods'):
                self.execute(between=patcher.start)
        finally:patcher.stop()
        self.clean()
    def test_changed_coordinate_recomputes_instead_of_returning_cached_value(self):
        old=self.point.copy()
        try:
            result=self.execute(between=lambda:self.point.__setitem__(slice(None),[0.,.001,0.]))
            self.assertEqual(result['initial_query']['counts']['normal_computations'],2)
            self.assertEqual(result['initial_query']['counts'].get('normal_hits',0),0)
            self.assertEqual(result['initial_normal'].tobytes(),np.array([0.,1.,0.]).tobytes())
        finally:self.point[:]=old
        self.clean()
    def test_copied_context_cannot_read_cache_from_another_thread(self):
        failures=[];results=[]
        def between():
            context=copy_context()
            def call():
                try:results.append(self.source.exact_normal(self.ref,self.point))
                except BaseException as error:failures.append(error)
            thread=threading.Thread(target=lambda:context.run(call));thread.start();thread.join()
        self.execute(between=between)
        self.assertFalse(results);self.assertEqual(len(failures),1);self.assertIsInstance(failures[0],ValueError);self.clean()

if __name__=='__main__':unittest.main(verbosity=2)
