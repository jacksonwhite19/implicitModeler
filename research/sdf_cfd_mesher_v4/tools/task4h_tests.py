from task4h_common import *
import unittest
from sdf_cfd_mesher_v4.certificate.future_proofs import FutureSpotProofs,verify_bound_record
from sdf_cfd_mesher_v4.certificate.future_selection import SortedBounds
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import repaired_kernel as kernel
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.repaired_runtime import CertificateSession
from sdf_cfd_mesher_v4.forecast.forensics import exact_wedge as old
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_wedge,exact_signs

class ExactDyadicTests(unittest.TestCase):
    def check(self,p):
        a=old(p);self.assertEqual(a,exact_wedge(p));self.assertEqual(exact_signs(p),{k:a[k] for k in ('jacobian_sign','volume_sign')})
    def test_healthy(self):self.check([(0,0,0),(1,0,0),(0,1,0),(0,0,1),(1,0,1),(0,1,1)])
    def test_zero(self):self.check([(0,0,0),(1,0,0),(0,1,0)]*2)
    def test_negative(self):self.check([(0,0,0),(1,0,0),(0,1,0),(0,0,-1),(1,0,-1),(0,1,-1)])
    def test_subnormal_positive(self):self.check([(0,0,0),(1,0,0),(0,1,0),(0,0,2**-1074),(1,0,2**-1074),(0,1,2**-1074)])
    def test_deterministic_deformations(self):
        for k in range(201):
            p=np.array([(0,0,0),(1,0,0),(0,1,0),(0,0,1),(1,0,1),(0,1,1)],float)
            for i in range(6):
                for j in range(3):p[i,j]+=((k*(i+3)*(j+5))%47-23)/32
            self.check(p)
    def test_translated_scaled(self):
        p=np.array([(0,0,0),(1,0,0),(0,1,0),(0,0,1),(1,0,1),(0,1,1)],float)
        for e in (-400,-30,0,30,400):self.check((p+.375)*2.**e)
    def test_nonfinite_rejected(self):
        with self.assertRaises((ValueError,OverflowError)):exact_wedge(np.full((6,3),np.inf))
    def test_repeat(self):
        p=np.arange(18,dtype=float).reshape(6,3)/17
        self.assertEqual(exact_wedge(p),exact_wedge(p.copy()))

class FutureProofTests(unittest.TestCase):
    def setUp(self):
        self.q=np.array([[(0,0,0),(1,0,0),(0,1,0),(1,1,0)],[(0,0,1),(1,0,1),(0,1,1),(1,1,1)]],float)
        self.labels=[[(0,i) for i in range(4)],[(1,i) for i in range(4)]]
    def frozen(self,q,labels):
        p=[kernel.Patch.binary(x,tuple(v)) for x,v in zip(q,labels)]
        return CertificateSession().pair(*p)
    def test_bounds_equivalence(self):
        r=FutureSpotProofs().future_pair(self.q,self.labels)
        self.assertEqual(r['classification'],self.frozen(self.q,self.labels)['classification']);self.assertTrue(verify_bound_record(self.q,self.labels,r))
    def test_exact_hit_and_mutation_isolation(self):
        e=FutureSpotProofs();r=e.future_pair(self.q,self.labels);r['classification']='corrupt'
        self.assertEqual(e.future_pair(self.q.copy(),self.labels)['classification'],'CertifiedSeparated');self.assertEqual(e.stats['hits'],1)
    def test_coordinate_invalidation(self):
        e=FutureSpotProofs();e.future_pair(self.q,self.labels);q=self.q.copy();q[1,0,2]=np.nextafter(1.,2.)
        e.future_pair(q,self.labels);self.assertEqual(e.stats['misses'],2)
    def test_label_invalidation(self):
        e=FutureSpotProofs();e.future_pair(self.q,self.labels);ids=[self.labels[0],[(2,i) for i in range(4)]]
        e.future_pair(self.q,ids);self.assertEqual(e.stats['misses'],2)
    def test_corrupt_bound_fails(self):
        r=FutureSpotProofs().future_pair(self.q,self.labels);r['proof']['witness']+=1
        with self.assertRaises(ValueError):verify_bound_record(self.q,self.labels,r)
    def test_forbidden_crossing(self):
        self.q[1]=[(.5,0,-1),(.5,1,-1),(.5,0,1),(.5,1,1)]
        old=self.frozen(self.q,self.labels);new=FutureSpotProofs().future_pair(self.q,self.labels)
        self.assertEqual(old,new);self.assertEqual(new['classification'],'CertifiedForbiddenIntersection')
    def test_invalid_patch(self):
        self.q[0]=0;r=FutureSpotProofs().future_pair(self.q,self.labels);self.assertEqual(r['classification'],'InvalidSidePatch')
    def test_cache_eviction_and_rollback(self):
        e=FutureSpotProofs(capacity=1);r=e.future_pair(self.q,self.labels);q=self.q.copy();q[1,:,2]+=1;e.future_pair(q,self.labels)
        self.assertEqual(e.future_pair(self.q,self.labels),r);self.assertEqual(e.stats['misses'],3)

class SelectionTests(unittest.TestCase):
    def test_complete_predicate_equivalence(self):
        x=np.arange(3000,dtype=float).reshape(1000,3);lo=((x*17)%193)/37;hi=lo+(x%7)/41
        index=SortedBounds(lo,hi);pad=128*np.finfo(float).eps
        for i in range(1000):
            expected=np.flatnonzero(np.all(hi>=lo[i]-pad,axis=1)&np.all(lo<=hi[i]+pad,axis=1))
            actual,tested=index.query(lo[i],hi[i],pad);self.assertTrue(np.array_equal(actual,expected))
    def test_touching_and_signed_zero(self):
        lo=np.array([[0,0,0],[-0.,0,0],[1,1,1],[np.nextafter(1.,2.),1,1]]);hi=lo+1
        index=SortedBounds(lo,hi)
        for pad in (0.,128*np.finfo(float).eps):
            for a,b in zip(lo,hi):
                actual,_=index.query(a,b,pad);expected=np.flatnonzero(np.all(hi>=a-pad,axis=1)&np.all(lo<=b+pad,axis=1));self.assertTrue(np.array_equal(actual,expected))

if __name__=='__main__':unittest.main()
