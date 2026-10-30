"""Original full volume records, dispatch/refusal and private-input custody."""
from pathlib import Path
import sys,threading,unittest
from fractions import Fraction
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import prospective_volume as producer
import cavity_volume_audit as reference
import prism_degree as batch
import generalize_local_remesh as g
import projected_boundary_index as index
from sdf_cfd_mesher_v4.certificate import aabb_index as tree


def operands():
    wall=np.array([[-1.,-1.,-1.],[1.,1.,-1.],[1.,-1.,1.],[-1.,1.,1.]])
    faces=[]
    for missing in range(4):
        f=[i for i in range(4) if i!=missing]
        if np.dot(np.cross(wall[f[1]]-wall[f[0]],wall[f[2]]-wall[f[0]]),wall[f].mean(axis=0))<0:f.reverse()
        faces.append(tuple(f))
    p,c,o=g.s.construct(wall,np.asarray(faces),[2*wall],np.ones(4,dtype=int))
    return p,c,o,faces


def outcome(call):
    try:return ('result',call())
    except (ValueError,TypeError,IndexError,RuntimeError,ZeroDivisionError) as error:return ('error',type(error),error.args)


class ProspectiveVolume(unittest.TestCase):
    def test_closed_all_prism_complete_record_and_single_serial_call(self):
        args=operands();expected=reference.audit(*args);original=reference.audit
        self.assertTrue(expected['exact_volume_balance']);self.assertTrue(expected['positive_cells']);self.assertTrue(expected['interior_degree']['passed'])
        with patch.object(batch,'serial',wraps=batch.serial) as backend,producer.for_mode('production'):
            actual=reference.audit(*args);backend.assert_called_once()
            self.assertEqual(len(backend.call_args.args[2]),len(args[1]))
        self.assertIs(reference.audit,original);self.assertEqual(actual,expected)

    def test_audit_unknown_mode_and_explicit_callbacks(self):
        args=operands();original=reference.audit;sentinel={'passed':False,'values':[],'unresolved':[],'callback':True}
        def custom(*a,**kw):return sentinel
        with patch.object(batch,'serial',side_effect=AssertionError('backend')):
            with producer.for_mode('audit'):self.assertIs(reference.audit,original)
            with self.assertRaisesRegex(ValueError,'UnknownVerificationMode'):producer.for_mode('unknown')
            expected=original(*args,_degree=custom)
            with producer.production():self.assertEqual(reference.audit(*args,_degree=custom),expected)
        with patch.object(batch,'serial',wraps=batch.serial) as backend,producer.production():
            self.assertEqual(reference.audit(*args,_degree=None),original(*args));backend.assert_called_once()

    def test_custom_audit_or_degree_before_entry_and_forged_names(self):
        args=operands();old=reference.audit
        def custom(*a,**kw):return {'passed':False,'values':[],'unresolved':[],'custom':True}
        for name in ('audit','degree'):
            with self.subTest(name=name),patch.object(reference,name,custom),patch.object(batch,'serial',side_effect=AssertionError('backend')):
                expected=reference.audit(*args)
                with producer.production():self.assertEqual(reference.audit(*args),expected)
        custom.__name__='degree';custom.__module__=reference.__name__
        with patch.object(reference,'degree',custom),patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
            self.assertIs(reference.audit,old)

    def test_changed_original_projected_constructor_keeps_original_refusal(self):
        args=operands()
        for cls in (index.ProjectedBoundaryIndex,tree.AABBIndex):
            with patch.object(cls,'__init__',side_effect=RuntimeError('original index')):
                expected=outcome(lambda:reference.audit(*args))
                with patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
                    self.assertEqual(outcome(lambda:reference.audit(*args)),expected)

    def test_changed_defaults_keep_original_selection(self):
        args=operands();old=reference.audit.__kwdefaults__
        def custom(*a):return {'passed':False,'values':[],'unresolved':[],'default':True}
        try:
            reference.audit.__kwdefaults__={'_degree':custom};expected=reference.audit(*args)
            with patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
                self.assertEqual(reference.audit(*args),expected)
        finally:reference.audit.__kwdefaults__=old
        old=tree.AABBIndex.__init__.__defaults__
        try:
            tree.AABBIndex.__init__.__defaults__=(1,);expected=reference.audit(*args)
            with patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
                self.assertEqual(reference.audit(*args),expected)
        finally:tree.AABBIndex.__init__.__defaults__=old

    def test_early_custom_index_class_hooks_keep_original(self):
        args=operands()
        for cls in (index.ProjectedBoundaryIndex,tree.AABBIndex):
            for hook in ('__new__','__getattribute__'):
                # Never mutate allocation slots on a shared original class.
                # CPython can retain slot behavior after deleting __new__.
                namespace={k:v for k,v in vars(cls).items() if k not in ('__dict__','__weakref__')}
                disposable=type(cls.__name__,(object,),namespace)
                def refused(*a,**kw):raise RuntimeError('original hook')
                setattr(disposable,hook,staticmethod(refused) if hook=='__new__' else refused)
                if cls is index.ProjectedBoundaryIndex:
                    patches=(patch.object(index,'ProjectedBoundaryIndex',disposable),patch.object(reference,'ProjectedBoundaryIndex',disposable))
                else:patches=(patch.object(tree,'AABBIndex',disposable),patch.object(index,'AABBIndex',disposable))
                with patches[0],patches[1]:
                    expected=outcome(lambda:reference.audit(*args))
                    with patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
                        self.assertEqual(outcome(lambda:reference.audit(*args)),expected)
                # The subsequent default oracle must still construct normally.
                self.assertTrue(reference.audit(*args)['interior_degree']['passed'])

    def test_custom_binding_before_adapter_import_is_not_blessed(self):
        import importlib.util
        args=operands()
        for name in ('audit','degree'):
            def custom(*a,**kw):return {'passed':False,'values':[],'unresolved':[],'early':True}
            with patch.object(reference,name,custom),patch.object(batch,'serial',side_effect=AssertionError('backend')):
                spec=importlib.util.spec_from_file_location('early_prospective_fixture',producer.__file__)
                early=importlib.util.module_from_spec(spec);spec.loader.exec_module(early)
                expected=reference.audit(*args)
                with early.production():self.assertEqual(reference.audit(*args),expected)

    def test_unsupported_representations_and_invalid_arguments_keep_original(self):
        args=operands();p,c,o,w=args
        invalid=p.copy();invalid[0,0]=np.nan
        controls=[(p.astype('f4'),c,o,w),(p[::-1],c,o,w),(invalid,c,o,w),(p,c,tuple(o),w),(p,c,o,tuple(w)),(p.tolist(),c,o,w)]
        for values in controls:
            expected=outcome(lambda:reference.audit(*values))
            with self.subTest(kind=str(type(values[0]))),patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
                self.assertEqual(outcome(lambda:reference.audit(*values)),expected)
        for call in [lambda:reference.audit(),lambda:reference.audit(*args,unknown=True),lambda:reference.audit(*args,None)]:
            expected=outcome(call)
            with patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():self.assertEqual(outcome(call),expected)

    def test_cross_thread_and_keyword_call_fall_back(self):
        args=operands();expected=reference.audit(*args);values=[]
        with patch.object(batch,'serial',side_effect=AssertionError('backend')),producer.production():
            thread=threading.Thread(target=lambda:values.append(reference.audit(*args)));thread.start();thread.join()
            self.assertEqual(reference.audit(points=args[0],cells=args[1],outer=args[2],wall=args[3]),expected)
        self.assertEqual(values,[expected])

    def test_backend_and_body_failures_propagate_once_and_restore(self):
        args=operands();original=reference.audit
        with patch.object(batch,'serial',side_effect=RuntimeError('backend')) as backend:
            with self.assertRaisesRegex(RuntimeError,'backend'),producer.production():reference.audit(*args)
            backend.assert_called_once()
        self.assertIs(reference.audit,original)
        with self.assertRaisesRegex(RuntimeError,'body'),producer.production():raise RuntimeError('body')
        self.assertIs(reference.audit,original)

    def test_private_points_witnesses_and_mutation_refusal(self):
        for target in ('points','cells','outer','cell_count','private_witness'):
            args=operands();original=batch.serial
            def changed(p,b,s):
                self.assertFalse(p.flags.writeable);self.assertFalse(np.shares_memory(p,args[0]));self.assertIsInstance(s,tuple)
                result=original(p,b,s)
                if target=='points':args[0][0,0]+=1
                elif target=='cells':args[1][0]['vertices'][0]=1
                elif target=='outer':args[2][0]=tuple(reversed(args[2][0]))
                elif target=='cell_count':
                    # The backend cannot alias the auditor's witness objects.
                    self.assertTrue(all(type(q) is Fraction for row in s for q in row))
                    args[1].pop()
                else:s[0][0]._numerator+=1
                return result
            with self.subTest(target=target),patch.object(batch,'serial',side_effect=changed):
                with self.assertRaisesRegex(ValueError,'ChangedProspectiveVolumeInputs'),producer.production():reference.audit(*args)

    def test_binding_and_function_code_changes_refuse_and_restore(self):
        args=operands();original=reference.audit;degree=reference.degree
        with self.assertRaisesRegex(ValueError,'ChangedProspectiveVolumeBinding'),producer.production():reference.audit=lambda *a,**kw:None
        self.assertIs(reference.audit,original)
        old_code=degree.__code__
        def change_code(*a):degree.__code__=(lambda *a:None).__code__;return None
        with patch.object(batch,'serial',side_effect=change_code):
            try:
                with self.assertRaisesRegex(ValueError,'ChangedProspectiveVolumeBinding'),producer.production():reference.audit(*args)
            finally:degree.__code__=old_code
        self.assertIs(reference.audit,original)
        with patch.object(batch,'serial',side_effect=lambda *a: setattr(reference,'degree',lambda *a:None)):
            try:
                with self.assertRaisesRegex(ValueError,'ChangedProspectiveVolumeBinding'),producer.production():reference.audit(*args)
            finally:reference.degree=degree
        self.assertIs(reference.audit,original)


if __name__=='__main__':unittest.main()
