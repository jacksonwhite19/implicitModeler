"""Small regression of unchanged capsule writer arithmetic; no extraction."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
sys.dont_write_bytecode=True
from historical_capsule_surface import OUT,CapsuleGeometry,capsule_binding,metric,provider,ShapeMetricPolicy,SeedVertex,SeedFace
from surface_execution import write_surface_contract


class MetricRegressions(unittest.TestCase):
    def test_legacy_capsule_writer_bytes_unchanged(self):
        a=CapsuleGeometry(json.loads((OUT/'GEOMETRY.json').read_text())['definition'])
        b=capsule_binding(a)
        with np.load(OUT/'fresh/SURFACE.npz') as z:
            p=z['coordinates'][z['triangles'][0]].copy()
        v=[SeedVertex(x,1,0,0) for x in p];f=[SeedFace((0,1,2),1)]
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            original=provider.write_contract
            m=[metric(a,x,b.sizing_policy,sheet_ref=1) for x in p]
            original(root/'before.bin',v,f,[],a,1.,shape_policy=ShapeMetricPolicy(),metric_override=m)
            write_surface_contract(b,root,v,f,[])
            self.assertEqual((root/'before.bin').read_bytes(),(root/'metric.bin').read_bytes())
            self.assertIs(provider.write_contract,original)


if __name__=='__main__':unittest.main(verbosity=2)
