"""Focused boundary tests. Real pipeline replay is a separate mandatory check."""
from pathlib import Path
import dataclasses
import tempfile
import unittest
from unittest.mock import patch
import sys
import numpy as np
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from historical_capsule_surface import CapsuleGeometry,capsule_binding,SeedVertex,SeedFace
from procedural_authorities import SeedEdge
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from surface_binding import QualificationContract
from surface_execution import validate_references,validate_output_references
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.binding=capsule_binding(CapsuleGeometry(dict(primitive='capsule',radius=.04,half_length=.04)))
        self.entry=ProductionAdaptiveSdfSurfaceMesher()

    def test_supported_callables_and_precision(self):
        r=self.entry.admission(self.binding)
        self.assertTrue(r['executable'],r)
        self.assertEqual(r['units'],'m')
        self.assertEqual(r['precision'],'binary64 analytic Authority')
        self.assertIn('project_sheet',r['operations'])

    def test_dispatch_calls_existing_shared_executor(self):
        with patch('surface_execution.execute_surface',return_value='receipt') as call:
            self.assertEqual(self.entry.generate(self.binding,'not-created'),'receipt')
            call.assert_called_once_with(self.binding,'not-created')

    def test_identity_rejected(self):
        self.binding.geometry_identity='wrong'
        with self.assertRaisesRegex(GeometryFailure,'GeometryIdentityMismatch'):self.entry.admission(self.binding)

    def test_units_rejected(self):
        self.binding.units='mm'
        with self.assertRaisesRegex(GeometryFailure,'UnitPrecisionContractRequired'):self.entry.admission(self.binding)

    def test_operation_descriptor_rejected(self):
        self.binding.operation_provenance['project_sheet']['sha256']='wrong'
        with self.assertRaisesRegex(GeometryFailure,'OperationBindingMismatch'):self.entry.admission(self.binding)

    def test_reference_descriptor_rejected(self):
        self.binding.references['sheet_names']={5:'invented'}
        with self.assertRaisesRegex(GeometryFailure,'ReferenceBindingMismatch'):self.entry.admission(self.binding)

    def test_seed_unknown_sheet_curve_corner_rejected(self):
        v=[SeedVertex(np.zeros(3),1,0,0)]
        for faces,edges in [([SeedFace((0,0,0),9)],[]),([SeedFace((0,0,0),1)],[SeedEdge((0,0),8,True)])]:
            with self.assertRaisesRegex(GeometryFailure,'ReferenceCoverageFailure'):
                validate_references(self.binding,v,faces,edges,'test')
        with self.assertRaisesRegex(GeometryFailure,'LostSourceSheetIdentity'):validate_output_references(self.binding,[2],[],set())

    def test_missing_seed_specific_failure_no_output(self):
        self.binding.seed_provider=None
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'surface'
            with self.assertRaises(GeometryFailure) as ctx:self.entry.generate(self.binding,p)
            self.assertFalse(p.exists())
            self.assertEqual(ctx.exception.record['operation'],'labelled_seed_provider')

    def test_qualification_contract_rejects_changed_gate(self):
        self.binding.qualification=dataclasses.replace(self.binding.qualification,projection_tolerance_m=1.)
        with self.assertRaisesRegex(ValueError,'UnqualifiedContinuousOrProjectionPolicy'):self.entry.admission(self.binding)

    def test_historical_class_delegates_explicitly(self):
        from historical_capsule_surface import ProductionAdaptiveSdfSurfaceMesher as Historical
        with patch.object(ProductionAdaptiveSdfSurfaceMesher,'generate',return_value='receipt') as call:
            self.assertEqual(Historical().generate(self.binding.authority,'unused'),'receipt')
            self.assertEqual(call.call_args.args[0].geometry_identity,self.binding.geometry_identity)


if __name__=='__main__':unittest.main(verbosity=2)
