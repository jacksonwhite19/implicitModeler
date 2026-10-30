"""Exact degree equivalence, invalidation and independent-reader controls."""
from pathlib import Path
import sys,unittest,copy
from fractions import Fraction as F
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import cavity_volume_audit as oracle
import prism_degree
import producer_filling as producer
import tet_layer_adapter as adapter
from background_materialization import build
from background_topology_repair import boundary

def tetra():
    return np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]),build(np.array([[0,1,2,3]],dtype='<i8'),0)

def centroid(points):return tuple(sum(F(float(p[j])) for p in points)/len(points) for j in range(3))

class ProducerFilling(unittest.TestCase):
    def compare_degree(self,points,faces,samples):
        a={};b={};expected=oracle.degree(points,faces,samples,stats=a)
        actual=prism_degree.serial(points,faces,samples,stats=b)
        self.assertEqual(actual,expected);self.assertEqual(b,a)
        return actual

    def test_exact_degree_inside_outside_touching_and_retry(self):
        p,c=tetra();faces=c[0]['faces']
        samples=[centroid(p),(F(2),F(2),F(2)),tuple(map(F,p[0])),(F(1,3),F(1,3),F(0))]
        result=self.compare_degree(p,faces,samples)
        self.assertEqual(result['values'][0]['degree'],1)
        self.assertEqual(result['values'][1]['degree'],0)
        self.assertFalse(result['passed']);self.assertTrue(result['unresolved'])

    def test_binary_scales_subnormal_and_rational_witnesses(self):
        p,c=tetra()
        for scale in (np.ldexp(1.,-1070),np.ldexp(1.,-100),1.,np.ldexp(1.,100),np.ldexp(1.,500)):
            with self.subTest(scale=scale):
                q=p*scale
                self.compare_degree(q,c[0]['faces'],[centroid(q),tuple(F(float(x))*F(1,3) for x in q[1]+q[2]+q[3])])

    def test_orientation_and_face_order_preserved(self):
        p,c=tetra();faces=c[0]['faces'];samples=[centroid(p)]
        for fs in (faces,list(reversed(faces)),[f[::-1] for f in faces]):self.compare_degree(p,fs,samples)

    def test_degenerate_projected_triangles_keep_unresolved_population(self):
        p=np.array([[0.,0.,0.],[1.,1.,2.],[2.,2.,4.]])
        result=self.compare_degree(p,[(0,1,2)],[(F(1),F(1),F(2)),(F(0),F(0),F(0))])
        self.assertFalse(result['passed']);self.assertEqual(result['unresolved'],[0,1])

    def test_empty_sample_population(self):
        p,c=tetra();self.compare_degree(p,c[0]['faces'],[])

    def test_deterministic_transforms_match_every_degree_and_candidate_count(self):
        rng=np.random.default_rng(7612);p,c=tetra()
        for unused in range(24):
            matrix=rng.integers(-5,6,(3,3)).astype(float)
            q=p@matrix+rng.integers(-8,9,3)
            samples=[centroid(q),tuple(F(float(v)) for v in q[0])]
            self.compare_degree(q,c[0]['faces'],samples)

    def test_complete_filling_dict_and_input_bytes_match(self):
        p,c=tetra();before=p.tobytes();old=copy.deepcopy(c);state=dict(points=p,cells=c)
        expected=adapter.filling(state,None);actual=producer.filling(state)
        self.assertEqual(actual,expected)
        self.assertTrue(actual['positive_cells'] and actual['exact_volume_balance'] and actual['interior_degree']['passed'])
        self.assertEqual(p.tobytes(),before);self.assertEqual(c,old)

    def test_no_cross_call_geometry_or_witness_reuse(self):
        p,c=tetra();first=producer.filling(dict(points=p,cells=c))
        p[1,0]=2.
        second=producer.filling(dict(points=p,cells=c));expected=adapter.filling(dict(points=p,cells=c),None)
        self.assertEqual(second,expected);self.assertNotEqual(second['cell_volume'],first['cell_volume'])

    def test_default_reader_uses_original_rational_engine_and_dynamic_patch(self):
        p,c=tetra();state=dict(points=p,cells=c)
        original=oracle.degree
        with patch.object(prism_degree,'serial',side_effect=AssertionError('ProducerReachedByReader')):
            with patch.object(oracle,'degree',wraps=original) as called:
                result=adapter.filling(state,None);called.assert_called_once()
            self.assertTrue(result['interior_degree']['passed'])

    def test_producer_native_refusal_propagates_without_fallback(self):
        p,c=tetra()
        with patch.object(prism_degree,'serial',side_effect=ValueError('InjectedNativeRefusal')):
            with self.assertRaisesRegex(ValueError,'InjectedNativeRefusal'):producer.filling(dict(points=p,cells=c))

    def test_invalid_point_and_incidence_refusals_match_original(self):
        p,c=tetra()
        controls=[]
        for value in (np.nan,np.inf):
            q=p.copy();q[0,0]=value;controls.append(dict(points=q,cells=c))
        controls.append(dict(points=p,cells=c+c))
        def outcome(call):
            try:return ('ok',call())
            except Exception as e:return (type(e).__name__,e.args)
        for state in controls:
            self.assertEqual(outcome(lambda:producer.filling(state)),outcome(lambda:adapter.filling(state,None)))

    def test_explicit_producer_dispatch_and_audit_default(self):
        self.assertIs(producer.for_mode('production'),producer.filling)
        self.assertIsNone(producer.for_mode('audit'))
        with self.assertRaisesRegex(ValueError,'UnknownVerificationMode'):producer.for_mode('unknown')

if __name__=='__main__':unittest.main()
