"""Ordered pair population, native batch boundaries and refusal controls."""
from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
from native_mixed_face_certificate import _pair_batches

class PairBatches(unittest.TestCase):
    def compare(self,index,active):
        expected=np.asarray(list(index.pairs(active)),dtype='u4').reshape(-1,2)
        blocks=list(_pair_batches(index,set(active)))
        actual=np.concatenate(blocks) if blocks else np.empty((0,2),dtype='u4')
        np.testing.assert_array_equal(actual,expected)
        self.assertEqual([len(b) for b in blocks],
                         [min(50000,len(expected)-i) for i in range(0,len(expected),50000)])
        # Retained blocks must survive subsequent iterator advancement.
        for b in blocks:self.assertEqual(b.dtype,np.dtype('u4'))

    def test_full_dense_crosses_native_batch_boundary(self):
        self.compare(AABBIndex(np.zeros((400,3)),np.ones((400,3))),range(400))

    def test_partial_empty_duplicate_mixed_signed_and_unsigned(self):
        rng=np.random.default_rng(724);lo=rng.integers(-4,4,(99,3)).astype(float)
        index=AABBIndex(lo,lo+2)
        for active in ([],[0],[0,0,98],[np.uint64(1),np.int64(97)],range(99),range(0,99,3)):
            with self.subTest(active=list(active)):self.compare(index,active)

    def test_inclusive_contact_and_no_pairs(self):
        for n in (0,1,2):
            lo=np.arange(n*3,dtype=float).reshape(n,3)
            self.compare(AABBIndex(lo,lo+3),range(n))

    def test_invalid_active_refusals_match_original(self):
        index=AABBIndex(np.zeros((2,3)),np.ones((2,3)))
        for active in ({False,True},{0.,1.},{-1},{2},{'bad'}):
            def outcome(call):
                try:return ('ok',list(call()))
                except Exception as e:return (type(e).__name__,e.args)
            with self.subTest(active=active):
                self.assertEqual(outcome(lambda:index.pairs(active)),outcome(lambda:_pair_batches(index,active)))

    def test_pair_index_overflow_refused_before_native_cast(self):
        class Index:
            lo=[]
            def pair_blocks(self,active):yield np.array([[0,2**32]],dtype='u8')
        with self.assertRaisesRegex(ValueError,'CertificatePairIndexOverflow'):
            list(_pair_batches(Index(),set()))

if __name__=='__main__':unittest.main()
