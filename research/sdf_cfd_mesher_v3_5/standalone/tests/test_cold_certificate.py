"""Cold batching must preserve complete ordered proofs and owner boundaries."""
from pathlib import Path
import sys,unittest,ctypes,os
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import reuse_mixed_certificate as reuse
import native_mixed_face_certificate as reference
import optimized_mixed_certificate as batch
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
from sdf_cfd_mesher_v4.common.authority import file_hash

def state():return dict(authority=dict(certificate_filter=file_hash(reference.SOURCE)))
def triangles():
    return np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[2.,0.,0.],[3.,0.,0.],[2.,1.,0.]]),[(0,1,2),(3,4,5)]
def normalized(result):return {k:v for k,v in result.items() if k not in ('proofs','optimization')}
def stream(result):return b''.join(block+b'\n' for block in result['proofs'].encoded_chunks(b'\n'))
def outcome(operation):
    try:
        result=operation();return ('result',normalized(result),stream(result))
    except (ValueError,RuntimeError,TypeError,IndexError) as error:return ('error',type(error),error.args)

class ColdCertificate(unittest.TestCase):
    def assert_equivalent(self,points,faces):
        expected=reference.certify(points,faces,range(len(faces)),workers=4)
        actual=reuse.certify(points,faces,state(),None)
        self.assertEqual(actual.keys(),expected.keys())
        self.assertEqual(normalized(actual),normalized(expected))
        self.assertEqual(stream(actual),stream(expected))
        return actual

    def test_complete_proofs_contacts_rotations_and_forbidden_crossings(self):
        controls=[triangles(),
            (np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,-1.,0.]]),[(0,1,2),(1,0,3)]),
            (np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[-1.,0.,0.]]),[(0,1,2),(0,3,4)]),
            (np.array([[0.,0.,0.],[2.,0.,0.],[0.,2.,0.],[.5,.5,-1.],[.5,.5,1.],[1.,.5,0.]]),[(0,1,2),(3,4,5)]),
            (np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,.1],[0.,1.,0.],[2.,0.,0.],[2.,1.,.1]]),[(0,1,2,3),(1,4,5,2)]),
            (np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.],[.5,-1.,1.]]),[(0,1,2,3),(1,0,4)])]
        failures=0
        for points,faces in controls:
            with self.subTest(faces=faces):failures+=not self.assert_equivalent(points,faces)['passed']
        self.assertGreater(failures,0)

    def test_boundary_scales_keep_exact_failure_rows(self):
        points=np.array([[0.,0.,0.],[2.,0.,0.],[0.,2.,0.],[.5,.5,-1.],[.5,.5,1.],[1.,.5,0.]])
        faces=[(0,1,2),(3,4,5)]
        for scale in (np.ldexp(1.,-1070),1e-100,1e100):
            with self.subTest(scale=scale):self.assert_equivalent(points*scale,faces)

    def test_custom_filtered_callback_keeps_original_arguments(self):
        points,faces=triangles();sentinel=object()
        with patch.object(reference,'certify',return_value=sentinel) as custom,patch.object(batch,'certify',side_effect=AssertionError('batch reached')):
            self.assertIs(reuse.certify(points,faces,state(),None),sentinel)
            self.assertIs(custom.call_args.args[0],points);self.assertIs(custom.call_args.args[1],faces)

    def test_unfiltered_callback_keeps_original_dispatch(self):
        points,faces=triangles();sentinel=object()
        with patch.object(reuse.g.s.cert,'certify',return_value=sentinel) as custom,patch.object(batch,'certify',side_effect=AssertionError('batch reached')):
            self.assertIs(reuse.certify(points,faces,dict(authority={}),None),sentinel)
            custom.assert_called_once()

    def test_unsupported_containers_and_invalid_inputs_keep_old_outcomes(self):
        points,faces=triangles()
        class Array(np.ndarray):pass
        invalid=points.copy();invalid[0,0]=np.nan
        controls=[(points.astype('f4'),faces),(points.view(Array),faces),(points[::-1],faces),
            (points,tuple(faces)),(points,[list(f) for f in faces]),(points,[]),
            (invalid,faces),(points,[(0,0,1)]),(points,[(0,1,99)]),
            (points,[(0,1,2,3,4)]),(points,[(False,1,2)])]
        for x,f in controls:
            with self.subTest(shape=x.shape,faces=f):
                expected=outcome(lambda:reference.certify(x,f,range(len(f)),workers=4))
                with patch.object(batch,'certify',side_effect=AssertionError('batch reached')):
                    self.assertEqual(outcome(lambda:reuse.certify(x,f,state(),None)),expected)

    def test_changed_filter_refuses_before_backend(self):
        points,faces=triangles()
        with patch.object(batch,'certify',side_effect=AssertionError('batch reached')):
            with self.assertRaisesRegex(ValueError,'ChangedCertificateFilter'):
                reuse.certify(points,faces,dict(authority=dict(certificate_filter='wrong')),None)

    def test_incremental_path_does_not_enter_cold_backend(self):
        points,faces=triangles()
        with patch.object(batch,'certify',side_effect=AssertionError('cold batch reached')):
            with self.assertRaisesRegex(ValueError,'ChangedCertificateImplementation'):
                reuse.certify(points,faces,state(),dict(authority=dict(certificate='wrong')))

    def test_backend_refusal_propagates_without_retry(self):
        points,faces=triangles();failure=RuntimeError('deliberate optimized refusal')
        with patch.object(batch,'certify',side_effect=failure) as backend:
            with self.assertRaises(RuntimeError) as raised:reuse.certify(points,faces,state(),None)
            self.assertIs(raised.exception,failure);backend.assert_called_once()

    def test_private_operands_and_mutation_refusals(self):
        for mutation in ('points','faces','shape'):
            points,faces=triangles()
            def compute(private,polygons,active,**kwargs):
                self.assertFalse(private.flags.writeable);self.assertEqual(type(polygons),tuple)
                self.assertEqual(list(active),list(range(len(faces))))
                self.assertEqual(kwargs,dict(workers=4,calculate_digest=True))
                with self.assertRaises(ValueError):private.flags.writeable=True
                if mutation=='points':points[0,0]+=1
                elif mutation=='faces':faces[0]=(0,2,1)
                else:points.shape=(1,-1)
                return dict(optimization={})
            with self.subTest(mutation=mutation),patch.object(batch,'certify',side_effect=compute):
                with self.assertRaisesRegex(ValueError,'ChangedColdCertificateInputs'):
                    reuse.certify(points,faces,state(),None)

    def test_changed_geometry_has_fresh_pair_population(self):
        points,faces=triangles();first=self.assert_equivalent(points,faces)
        points[3:,0]-=2
        second=self.assert_equivalent(points,faces)
        self.assertNotEqual(first['proof_hash'],second['proof_hash'])
        self.assertNotEqual(first['required_pairs'],second['required_pairs'])

    @unittest.skipUnless(os.name=='nt','Qualified runtime is Windows-only')
    def test_original_gradual_underflow_refusal_is_retained(self):
        points,faces=triangles();crt=ctypes.CDLL('ucrtbase.dll');control=crt._controlfp_s
        control.argtypes=(ctypes.POINTER(ctypes.c_uint),ctypes.c_uint,ctypes.c_uint);control.restype=ctypes.c_int
        word=ctypes.c_uint();self.assertEqual(control(ctypes.byref(word),0,0),0);saved=word.value
        try:
            self.assertEqual(control(ctypes.byref(word),0x1000000,0x3000000),0)
            expected=outcome(lambda:reference.certify(points,faces,range(len(faces)),workers=4))
            actual=outcome(lambda:reuse.certify(points,faces,state(),None))
            self.assertEqual(actual,expected)
            self.assertEqual(actual,('error',RuntimeError,('GradualUnderflowRequired',)))
        finally:
            control(ctypes.byref(word),saved,0x3000000)

    def test_native_broadphase_is_complete_inclusive_and_ordered(self):
        # Bounds-only controls: touching, one-ULP gaps, subnormals and huge extents.
        tiny=np.nextafter(0.,1.)
        boxes=[((0.,0.,0.),(1.,1.,1.)),((1.,0.,0.),(2.,1.,1.)),
            ((np.nextafter(2.,np.inf),0.,0.),(3.,1.,1.)),
            ((-tiny,-tiny,-tiny),(tiny,tiny,tiny)),((-1e300,-1e300,-1e300),(1e300,1e300,1e300))]
        q=np.array([[[a[0],a[1],a[2]],[b[0],a[1],a[2]],[b[0],b[1],b[2]],[a[0],b[1],b[2]]] for a,b in boxes])
        native=batch.Native();index=AABBIndex(q.min(1),q.max(1))
        for active in ([],[0],[1,3],list(range(len(q)))):
            with self.subTest(active=active):
                expected=np.asarray(list(index.pairs(active)),dtype=np.uint32).reshape(-1,2)
                actual=native.pairs(q,active,4)
                self.assertEqual(actual.tobytes(),expected.tobytes())
        self.assertEqual(native.pairs(np.empty((0,4,3)),[],4).shape,(0,2))

if __name__=='__main__':unittest.main()
