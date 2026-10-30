"""Producer batching preserves the old certifier's proofs and boundaries."""
from pathlib import Path
import sys,threading,unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import producer_collision as producer
import native_mixed_face_certificate as reference
import optimized_mixed_certificate as batch


def operands():
    return np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,-1.,0.]]),[(0,1,2),(1,0,3)]


def normalized(value):return {k:v for k,v in value.items() if k!='proofs'}


def stream(value):return b''.join(block+b'\n' for block in value['proofs'].encoded_chunks(b'\n'))


def outcome(call):
    try:
        value=call()
        return ('result',normalized(value),stream(value))
    except (ValueError,RuntimeError,TypeError,IndexError) as error:
        return ('error',type(error),error.args)


class ProducerCollision(unittest.TestCase):
    def test_complete_population_exact_proofs_and_same_workers(self):
        points,faces=operands();active=list(range(len(faces)))
        expected=reference.certify(points,faces,active,workers=4);original=reference.certify
        with patch.object(batch,'certify',wraps=batch.certify) as backend,producer.for_mode('production'):
            actual=reference.certify(points,faces,active,workers=4)
            backend.assert_called_once()
            self.assertEqual(backend.call_args.kwargs,dict(workers=4,calculate_digest=True))
        self.assertIs(reference.certify,original)
        self.assertEqual(actual.keys(),expected.keys())
        self.assertEqual(normalized(actual),normalized(expected));self.assertEqual(stream(actual),stream(expected))

    def test_partial_duplicate_and_noncanonical_active_keep_original(self):
        points,faces=operands()
        for active in [[],[0],[0,0],[1,0],range(2),[False,1],[np.int64(0),np.int64(1)]]:
            expected=outcome(lambda:reference.certify(points,faces,active,workers=4))
            with self.subTest(active=active),patch.object(batch,'certify',side_effect=AssertionError('batch reached')),producer.complete_batch():
                self.assertEqual(outcome(lambda:reference.certify(points,faces,active,workers=4)),expected)

    def test_unsupported_inputs_and_invalid_worker_precedence(self):
        points,faces=operands();active=[0,1]
        invalid=points.copy();invalid[0,0]=np.nan
        controls=[(points.astype('f4'),faces,4),(points[::-1],faces,4),(invalid,faces,4),
            (points,tuple(faces),4),(points,[list(f) for f in faces],4),
            (points,[(0,0,1),(1,0,3)],4),(points,faces,True),(points,faces,np.int64(4)),
            (points,faces,25),(points,None,0),(None,faces,0)]
        for x,f,workers in controls:
            expected=outcome(lambda:reference.certify(x,f,active,workers=workers))
            with self.subTest(workers=workers),patch.object(batch,'certify',side_effect=AssertionError('batch reached')),producer.complete_batch():
                self.assertEqual(outcome(lambda:reference.certify(x,f,active,workers=workers)),expected)

    def test_custom_callback_gets_original_arguments(self):
        points,faces=operands();active=[0,1];sentinel=object()
        with patch.object(reference,'certify',return_value=sentinel) as original,patch.object(batch,'certify',side_effect=AssertionError('batch reached')),producer.complete_batch():
            self.assertIs(reference.certify(points,faces,active,workers=4),sentinel)
            self.assertIs(original.call_args.args[0],points);self.assertIs(original.call_args.args[1],faces)
            self.assertIs(original.call_args.args[2],active);self.assertEqual(original.call_args.kwargs,dict(workers=4))

    def test_audit_is_original_and_unknown_mode_refuses(self):
        original=reference.certify
        with producer.for_mode('audit'):self.assertIs(reference.certify,original)
        with self.assertRaisesRegex(ValueError,'UnknownVerificationMode'):producer.for_mode('unknown')

    def test_original_constructor_failure_precedes_batch(self):
        points,faces=operands();failure=RuntimeError('original filter gate')
        with patch.object(reference,'Filter',side_effect=failure),patch.object(batch,'certify',side_effect=AssertionError('batch reached')),producer.complete_batch():
            with self.assertRaises(RuntimeError) as caught:reference.certify(points,faces,[0,1],workers=4)
            self.assertIs(caught.exception,failure)

    def test_backend_failure_is_not_retried_and_binding_restores(self):
        points,faces=operands();original=reference.certify;failure=RuntimeError('batch refusal')
        with patch.object(batch,'certify',side_effect=failure) as backend:
            with self.assertRaises(RuntimeError) as caught,producer.complete_batch():reference.certify(points,faces,[0,1],workers=4)
            self.assertIs(caught.exception,failure);backend.assert_called_once()
        self.assertIs(reference.certify,original)

    def test_private_operands_and_mutated_inputs_refuse(self):
        for target in ('points','faces','active'):
            points,faces=operands();active=[0,1];original=reference.certify
            def changed(private,polygons,selected,**kwargs):
                self.assertFalse(private.flags.writeable)
                self.assertIsNot(private,points);self.assertIsInstance(polygons,tuple)
                if target=='points':points[0,0]=1.
                elif target=='faces':faces[0]=(0,2,1)
                else:active.reverse()
                return dict(optimization={})
            with self.subTest(target=target),patch.object(batch,'certify',side_effect=changed):
                with self.assertRaisesRegex(ValueError,'ChangedProducerCollisionInputs'),producer.complete_batch():
                    reference.certify(points,faces,active,workers=4)
            self.assertIs(reference.certify,original)

    def test_cross_thread_dispatch_uses_original(self):
        points,faces=operands();expected=outcome(lambda:reference.certify(points,faces,[0,1],workers=4));actual=[]
        with patch.object(batch,'certify',side_effect=AssertionError('batch reached')),producer.complete_batch():
            thread=threading.Thread(target=lambda:actual.append(outcome(lambda:reference.certify(points,faces,[0,1],workers=4))))
            thread.start();thread.join(timeout=30);self.assertFalse(thread.is_alive())
        self.assertEqual(actual,[expected])

    def test_rebinding_refuses_and_restores_original(self):
        original=reference.certify
        with self.assertRaisesRegex(ValueError,'ChangedProducerCollisionBinding'),producer.complete_batch():
            reference.certify=lambda *a,**k:None
        self.assertIs(reference.certify,original)


if __name__=='__main__':unittest.main()
