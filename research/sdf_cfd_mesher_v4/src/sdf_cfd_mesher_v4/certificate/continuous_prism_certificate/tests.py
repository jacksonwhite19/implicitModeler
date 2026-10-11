"""Focused integration tests; never imported by production runtime."""
import hashlib
import json
import unittest
import tempfile
import struct
from pathlib import Path
import numpy as np
from . import kernel
from .runtime import CertificateSession, certify_shell, certify_local, canonical_bytes
from .guard import affected_guard

def patch(points, start=0, ids=None, dimension=2):
    return kernel.Patch.binary(points, tuple(range(start, start+4)) if ids is None else ids, dimension)

class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.a = patch([(0,0,0),(1,0,0),(0,1,0),(1,1,0)])
    def test_unordered_completeness(self):
        lo=np.array([[0,0,0],[.5,0,0],[2,0,0]]);hi=lo+1
        reference=kernel.enumerator.brute_force(lo,hi)
        for order in [np.arange(3),np.arange(3)[::-1]]:
            for capacity in [1,2,8]:
                got,_=kernel.enumerator.bvh_pairs(lo[order],hi[order],ids=order,leaf_capacity=capacity,reverse_work=True,swap_children=True)
                self.assertEqual(got,reference)
    def test_legal_edge(self):
        b=patch([(1,0,0),(2,0,0),(1,1,0),(2,1,0)],ids=(1,4,3,5))
        self.assertEqual(kernel.certificate.certify(self.a,b)['classification'],'CertifiedAllowedIncidentContact')
    def test_legal_vertex(self):
        b=patch([(1,1,0),(2,1,0),(1,2,0),(2,2,0)],ids=(3,4,5,6))
        self.assertEqual(kernel.certificate.certify(self.a,b)['classification'],'CertifiedAllowedIncidentContact')
    def test_planar_overlap(self):
        b=patch([(.1,.1,0),(1.1,.1,0),(.1,1.1,0),(1.1,1.1,0)],4)
        self.assertEqual(kernel.certificate.certify(self.a,b)['classification'],'CertifiedForbiddenIntersection')
    def test_exact_separation(self):
        b=patch([(0,0,1),(1,0,1),(0,1,1),(1,1,1)],4)
        self.assertEqual(kernel.certificate.certify(self.a,b)['classification'],'CertifiedSeparated')
    def test_coincident_rank3(self):
        a=patch([(0,0,0),(1,0,0),(0,1,0),(1,1,1)])
        b=patch([(0,0,0),(1,0,0),(0,1,0),(1,1,1)],4)
        self.assertEqual(kernel.certificate.certify(a,b)['classification'],'CertifiedForbiddenIntersection')
    def test_noncoincident_rank3(self):
        a=patch([(0,0,0),(1,0,0),(0,1,0),(1,1,1)])
        b=patch([(0,0,-1/32),(1,0,-1/32),(0,1,3/32),(1,1,35/32)],4)
        self.assertEqual(kernel.certificate.certify(a,b)['classification'],'CertifiedForbiddenIntersection')
    def test_cache_cold_warm_mutation_component_rollback(self):
        base=np.array([(0,0,1),(1,0,1),(0,1,1),(1,1,1)],dtype=float)
        states=[base,base.copy(),base+np.array([0,0,1]),base+np.array([2,0,0]),base]
        cache=CertificateSession(8)
        for points in states:
            b=patch(points,4)
            kernel.clear_kernel_caches()
            expected=CertificateSession().pair(self.a,b)
            self.assertEqual(canonical_bytes(expected),canonical_bytes(cache.pair(self.a,b)))
        self.assertEqual(cache.hits,2);self.assertEqual(cache.misses,3)
    def test_guard_includes_shared_lineages_and_remote_overlap(self):
        wall=np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0],[.2,.2,0],[.8,.2,0],[.2,.8,0],[9,9,0],[10,9,0],[9,10,0]],dtype=float)
        x=np.stack((wall,wall+[0,0,1]));f=np.array([[0,1,2],[1,3,2],[4,5,6],[7,8,9]])
        g=affected_guard(x,f,[0],x.copy())
        self.assertEqual(g['columns'].tolist(),[0,1,2])
    def test_deterministic_records(self):
        x=np.array([[[0,0,0],[1,0,0],[0,1,0]],[[0,0,1],[1,0,1],[0,1,1]]],dtype=float)
        streams=[]
        for _ in range(2):
            out=[];certify_shell(x,np.array([[0,1,2]]),sink=out.append);streams.append(b''.join(out))
        self.assertEqual(*streams)
    def test_strict_input_admission(self):
        x=np.zeros((2,3,3),dtype=float);f=np.array([[0,1,2]])
        for xx,ff,vv in [(x,f+.1,None),(x[:1],f,None),(x.astype(np.float32),f,None),(x,f,np.array([0.,1.,2.]))]:
            with self.assertRaises(ValueError):certify_shell(xx,ff,vertices=vv)
        with self.assertRaises(ValueError):affected_guard(x,f,[.5])
    def test_fixed_gateway_and_retained_call_chain(self):
        from .integration import ContinuousLayerAdapter
        x=np.array([[[0,0,0],[1,0,0],[0,1,0]],[[0,0,1],[1,0,1],[0,1,1]]],dtype='<f8')
        f=np.array([[0,1,2]],dtype='<u8');calls=[]
        def native(tool,path,tag):
            calls.append(tool);return {'pass':True,'tool':tool}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'side.bin';p.write_bytes(b'V3SIDE1\0'+struct.pack('<QQ',3,1)+x.tobytes()+f.tobytes())
            r=ContinuousLayerAdapter(native).certify(p,contact_input=p,front_input=p,tag='fixture')
            self.assertTrue(r['pass'])
        self.assertEqual(calls,['full_airfoil_side_cap_audit','complete_one_layer_collision_audit','surface_topology_audit'])
    def test_local_guard_certificate_matches_complete_reference(self):
        x=np.array([[[0,0,0],[1,0,0],[0,1,0]],[[0,0,1],[1,0,1],[0,1,1]]],dtype=float)
        f=np.array([[0,1,2]]);out=[]
        r=certify_local(x,f,[0],reference=x.copy(),sink=out.append)
        full=[];certify_shell(x,f,state='local',sink=full.append)
        self.assertTrue(r['pass']);self.assertEqual(out,full)
    def test_cache_component_mutation_and_disconnected_authority(self):
        base=np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0]],dtype=float)
        states=[]
        original=[base,base+[0,0,1],base+[4,0,0],base+[4,0,1]]
        states.append(original)
        states.append([original[0],base,original[2],original[3]])
        states.append([base+[2,0,0],base+[2,0,0],original[2],original[3]])
        states.append(original)
        session=CertificateSession(64)
        for state in states:
            patches=[patch(points,4*i) for i,points in enumerate(state)]
            for i,j in [(0,1),(0,2),(1,2),(2,3)]:
                expected=CertificateSession().pair(patches[i],patches[j])
                actual=session.pair(patches[i],patches[j])
                self.assertEqual(canonical_bytes(actual),canonical_bytes(expected))
        self.assertGreaterEqual(session.hits,6)

if __name__=='__main__':unittest.main()
