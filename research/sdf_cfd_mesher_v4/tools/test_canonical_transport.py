"""Focused protocol checks without meshing or mutable production state."""
import json
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[3]
V4=ROOT/'research/sdf_cfd_mesher_v4'
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel, GeometryFailure, EnclosingRegion, require_capabilities
from sdf_cfd_mesher_v4.sdf.cgal import CgalInvocation
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.owner=CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')])
    def tearDown(self): self.owner.close()

    def test_malformed_json_and_recovery(self):
        self.owner.transport.write('{invalid')
        error=json.loads(self.owner.transport.read())
        self.assertEqual(error['error']['code'],'InvalidJson')
        self.assertEqual(self.owner.load_rhai('sphere(1.0)',1).query([[0,0,0]]),[-1.])

    def test_unsupported_operations(self):
        with self.assertRaises(GeometryFailure) as ctx:self.owner.request(op='project',handle=1,point=[0,0,0])
        self.assertEqual(ctx.exception.record['code'],'InvalidRequest')
        g=self.owner.load_rhai('sphere(1.0)',1)
        for stage in ['carrier_refinement','tensor_sizing','mmgs_adaptation','exact_projection','independent_qualification']:
            with self.assertRaises(GeometryFailure) as ctx:require_capabilities(g,stage)
            self.assertEqual(ctx.exception.record['geometry_identity'],g.identity)
            self.assertEqual(ctx.exception.record['stage'],stage)
            self.assertTrue(ctx.exception.record['required'])

    def test_close_and_lifetime(self):
        g=self.owner.load_rhai('sphere(1.0)',1)
        self.owner.close()
        with self.assertRaises(GeometryFailure) as ctx:g.query([[0,0,0]])
        self.assertEqual(ctx.exception.record['code'],'HostClosed')

    def test_no_parameter_dictionary_fallback(self):
        with self.assertRaises(GeometryFailure) as ctx:
            ProductionAdaptiveSdfSurfaceMesher().admission({'radius':.04,'half_length':.04},EnclosingRegion((0,0,0),1,'test'))
        self.assertEqual(ctx.exception.record['code'],'CanonicalHandleRequired')

    def test_enclosure_and_criteria_are_caller_owned(self):
        region=EnclosingRegion((1,2,3),4,'test')
        call=CgalInvocation(('backend',),'--probe','points',.001,repetitions=10)
        self.assertEqual(call.command(region),['backend','--probe','points','1','2','3','4','0.001','10'])
        with self.assertRaises(ValueError):CgalInvocation(('backend',),'--extract','out',.001).command(region)

    def test_unit_identity(self):
        first=self.owner.load_rhai('sphere(1.0)',1)
        with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as other:
            second=other.load_rhai('sphere(1.0)',.001)
            self.assertNotEqual(first.identity,second.identity)
            self.assertEqual(second.query([[0,0,0]]),[-.001])

    def test_native_descriptor_binding(self):
        g=self.owner.load_rhai('sphere(1.0)',1)
        g.description['capabilities']['projection']='supported'
        with self.assertRaises(GeometryFailure) as ctx:
            ProductionAdaptiveSdfSurfaceMesher().admission(g,EnclosingRegion((0,0,0),2,'test'))
        self.assertEqual(ctx.exception.record['code'],'GeometryDescriptionMismatch')


if __name__=='__main__':unittest.main(verbosity=2)
