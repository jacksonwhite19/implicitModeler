"""Representation checks and scale contracts; no geometry or meshing algorithm."""
import copy, json, sys, unittest, subprocess
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel,GeometryFailure
from canonical_polygon_authority import validate_topology
from surface_binding import bind_canonical

class PolygonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cases=json.loads((V4/'results/generic_canonical_bridge/DIRECT.json').read_text())['cases']
        c=next(c for c in cases if c['name']=='plain_wing')
        cls.owner=CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')])
        cls.geometry=cls.owner.load_rhai(c['script'],c['metres_per_model_unit'])
        cls.binding=bind_canonical(cls.geometry);cls.a=cls.binding.authority;cls.packet=cls.geometry.stored_topology()

    @classmethod
    def tearDownClass(cls):cls.owner.close()

    def test_affine_differential_scale_all_branches(self):
        for ref in self.a.sheet_names:
            p=self.a.unit.points[0]*self.a.scale
            g,h=self.a.gradient_hessian(ref,p)
            self.assertTrue(np.array_equal(g,self.a.unit.exact_normal(ref,p/self.a.scale)))
            self.assertTrue(np.array_equal(h,np.zeros((3,3))))
            self.assertEqual(self.a.curvature_upper(ref,p),0.)
            u,v,k0,k1,n=self.a._normal_map_principal_data(ref,p)
            self.assertEqual((k0,k1),(0.,0.))
            self.assertTrue(np.array_equal(n,g))

    def test_nonconvex_source_rejected_not_reoriented(self):
        p=copy.deepcopy(self.packet)
        for s in p['data']['sections']:
            s['point_bits'][2],s['point_bits'][3]=s['point_bits'][3],s['point_bits'][2]
        with self.assertRaisesRegex(GeometryFailure,'ConvexOrientationPremiseFailed'):
            validate_topology(p,self.geometry.identity)

    def test_collapsed_span_rejected(self):
        p=copy.deepcopy(self.packet);p['data']['sections'][1]['position_bits']=p['data']['sections'][0]['position_bits']
        with self.assertRaisesRegex(GeometryFailure,'UniformExtrusionPremiseFailed'):
            validate_topology(p,self.geometry.identity)

    def test_invalid_scale_rejected(self):
        for scale in (None,0.,float('inf')):
            p=copy.deepcopy(self.packet);p['metres_per_model_unit']=scale
            with self.assertRaisesRegex(GeometryFailure,'InvalidUnitScale'):
                validate_topology(p,self.geometry.identity)

    def test_ownership_distinct_from_diagnostic_sectors(self):
        self.assertEqual(len(self.a.sheet_names),201)
        self.assertEqual(len(self.a.curve_names),597)
        self.assertEqual(len(self.a.corner_names),398)
        self.assertEqual(len(self.binding.query_discovery['data']['inventory']['model']['sectors']),5)
        sets=[set(getattr(self.a,k)) for k in ('sheet_names','curve_names','corner_names')]
        self.assertFalse(sets[0]&sets[1] or sets[0]&sets[2] or sets[1]&sets[2])
        self.assertTrue(all(s.startswith('stored-polygon/'+self.packet['topology_identity']+'/') for s in self.a.sheet_names.values()))

    def test_automatic_source_contract_and_absent_accuracy_sizing(self):
        report=self.binding.inspect()
        self.assertEqual({r['operation'] for r in report['issues']},
            {'metric','sizing_policy','caller_accuracy_limits'})
        self.assertTrue(report['source_boundary_status']['conversion_evidence_verified'])
        self.assertFalse(report['executable'])
        self.assertEqual(report['admission_layers']['affine_branch_operations'],'available')
        self.assertIsNone(self.binding.sizing_policy)

    def test_no_historical_metric_inheritance(self):
        self.assertFalse(hasattr(self.a,'metric'))

    def test_existing_queries_equal_direct_kernel_oracle(self):
        from test_canonical_query_connection import same,si_feature
        inputs=V4/'results/generic_canonical_bridge/DIRECT.json'
        retained=V4/'results/automatic_field_ingestion/QUERY_RESULTS.json'
        cases=json.loads(inputs.read_text())['cases']
        for name in ('plain_wing','sphere'):
            case=next(c for c in cases if c['name']==name)
            oracle=json.loads(subprocess.check_output([str(ROOT/'target/debug/query_oracle.exe'),str(inputs),name,str(retained)],text=True,timeout=30))
            with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')]) as owner:
                g=owner.load_rhai(case['script'],case['metres_per_model_unit']);a=bind_canonical(g).authority
                same(a.scalar_value(oracle['points_m']).tolist(),oracle['scalar'])
                projection=a.numerical_project(oracle['points_m'])['data']['results']
                for actual,expected in zip(projection,oracle['projected']):same({k:actual[k] for k in expected},expected)
                same([r['normal_direction'] for r in a.numerical_normal(oracle['points_m'])['data']['results']],oracle['normal'])
                same(a.feature_inventory()['data']['model'],oracle['inventory'])
                requests=[dict(feature_id=r['feature_id'],parameter=r['parameter']) for r in oracle['evaluations']]
                for actual,expected in zip(a.feature_evaluate(requests)['data']['results'],oracle['evaluations']):
                    same(actual['model'],expected['model']);same(actual['si'],si_feature(expected['model'],case['metres_per_model_unit']))
                same(a.feature_query(oracle['feature_points_m'])['data']['model']['results'],oracle['batch']['results'])
                print(json.dumps(dict(query_transport=name,scalar_projection_normal_count=len(oracle['points_m']),
                    feature_evaluations=len(requests),classifications=len(oracle['feature_points_m']),all_bits_equal=True)))

if __name__=='__main__':unittest.main(verbosity=2)
