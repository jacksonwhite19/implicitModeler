import sys,unittest,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import geometry
import adaptive_sdf_carrier as carrier
from carrier_split_checks import refinement_edges,validate_split
from procedural_authorities import SeedVertex,SeedFace

def vertices(x):return [SeedVertex(np.array(p,dtype=float),1,0,0) for p in x]
class Sphere:
    def exact_normal(self,ref,p):return p/np.linalg.norm(p)
    def project_sheet(self,ref,p):return p/np.linalg.norm(p)
    def implicit_value(self,ref,p):return np.linalg.norm(p)-1
    def competing_component_sheet_refs(self,ref):return []

class CarrierShortEdge(unittest.TestCase):
    def test_long_direction_split_preserves_short_feature(self):
        v=vertices([[0,0,0],[1e-4,0,0],[0,1e-8,0]]);f=SeedFace((0,1,2),1)
        saved=np.array([p.point.copy() for p in v])
        selected=refinement_edges(v,f,carrier.CarrierOraclePolicy())
        self.assertEqual(selected,[(1,2)])
        children=carrier._split_face(f,{selected[0]:3})
        self.assertEqual(len(children),2)
        self.assertTrue(any({0,2}.issubset(set(c.vertices)) for c in children))
        np.testing.assert_array_equal(saved,np.array([p.point for p in v]))
    def test_guard_and_ordinary_red_split_unchanged(self):
        p=carrier.CarrierOraclePolicy()
        self.assertEqual(refinement_edges(vertices([[0,0,0],[1e-8,0,0],[0,1e-8,0]]),SeedFace((0,1,2),1),p),[])
        v=vertices([[0,0,0],[1,0,0],[0,1,0]])
        self.assertEqual(refinement_edges(v,SeedFace((0,1,2),1),p),[(0,1),(1,2),(0,2)])
        self.assertEqual(len(carrier._split_face(SeedFace((0,1,2),1),{(0,1):3,(1,2):4,(0,2):5})),4)
    def test_conforming_sphere_topology_and_bad_projection(self):
        a=Sphere();v=vertices([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])
        rows=[(0,2,4),(2,1,4),(1,3,4),(3,0,4),(2,0,5),(1,2,5),(3,1,5),(0,3,5)]
        faces=[SeedFace(row,1) for row in rows];new=v+[SeedVertex(a.project_sheet(1,(v[0].point+v[2].point)/2),1,0,0)]
        children=[];changed=[]
        for face in faces:
            c=carrier._split_face(face,{(0,2):6})
            if len(c)>1:changed.extend(range(len(children),len(children)+len(c)))
            children.extend(c)
        receipt=validate_split(a,v,faces,new,children,changed)
        self.assertTrue(receipt['after']['passed']);self.assertEqual(receipt['after']['euler'],2)
        self.assertEqual(len(children),10)
        bad=new[:-1]+[SeedVertex(new[-1].point*1.01,1,0,0)]
        with self.assertRaisesRegex(ValueError,'CarrierSplitProjectionFailure'):validate_split(a,v,faces,bad,children,changed)
        inverted=list(children);inverted[0]=SeedFace(tuple(reversed(inverted[0].vertices)),1)
        with self.assertRaisesRegex(ValueError,'CarrierSplitTopologyFailure'):validate_split(a,v,faces,new,inverted,changed)
    def test_durable_guard_refusal_geometry_and_predicates(self):
        v=vertices([[1,0,0],[1,1e-8,0],[1,0,1e-8]]);face=SeedFace((0,1,2),1);evidence=[]
        # A spatially varying target inside this face exceeds its nodal target.
        points=[p.point.copy() for p in v]
        def metric(p,ref):return np.eye(3)*(1 if any(np.array_equal(p,q) for q in points) else 100)
        _,_,_,receipt=carrier.refine_metric_carriers(Sphere(),v,[face],[],metric,carrier.CarrierOraclePolicy(),
            derivative_diagnostics=False,evidence=evidence.append)
        self.assertFalse(receipt['materialization_authorized'])
        row=receipt['face_decisions'][0]
        self.assertEqual(row['reason'],'NoEligibleEdge');self.assertFalse(row['refinement_eligible'])
        self.assertTrue(row['predicates']['metric_inadequate']);self.assertEqual(row['vertex_ids'],[0,1,2])
        self.assertEqual(row['coordinates_m'],[p.tolist() for p in points])
        self.assertEqual(evidence[-1]['face_decisions'],receipt['face_decisions'])
        json.dumps(receipt,allow_nan=False)
    def test_assessment_cache_reuses_only_unchanged_faces(self):
        v=vertices([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])
        rows=[(0,2,4),(2,1,4),(1,3,4),(3,0,4),(2,0,5),(1,2,5),(3,1,5),(0,3,5)]
        faces=[SeedFace(row,1) for row in rows];calls=[]
        def assess(a,v,face,*args,**kw):
            calls.append(face.vertices)
            return dict(inadequate=face.vertices==rows[0],minimum_edge_m=1.,maximum_metric_edge_ratio=1.,
                        maximum_normal_interpolation_defect_deg=2. if face.vertices==rows[0] else 0.)
        with patch.object(carrier,'assess_carrier_face',side_effect=assess):
            _,_,_,receipt=carrier.refine_metric_carriers(Sphere(),v,faces,[],lambda p,r:np.eye(3),carrier.CarrierOraclePolicy(),derivative_diagnostics=False)
        self.assertTrue(receipt['materialization_authorized'])
        self.assertGreater(receipt['assessment_instrumentation']['unchanged_face_cache_hits'],0)
        self.assertEqual(len(calls),len(set(calls)))
        nodal=[np.eye(3) for _ in v]
        calls.clear()
        def promote(a,points,*args):return [np.eye(3) for _ in points],dict(active_faces=0,changed_vertices=0)
        with patch.object(carrier,'assess_carrier_face',side_effect=assess),patch.object(carrier,'promote_face_probe_metrics',side_effect=promote):
            _,_,_,receipt=carrier.refine_metric_carriers(Sphere(),v,faces,[],lambda p,r:np.eye(3),carrier.CarrierOraclePolicy(),
                interpolation_compatible_transition=True,derivative_diagnostics=False)
        self.assertTrue(receipt['materialization_authorized'])
        self.assertEqual(receipt['assessment_instrumentation']['unchanged_face_cache_hits'],0)
        self.assertGreater(len(calls),len(set(calls)))
        calls.clear();original=np.array([p.point.copy() for p in v]);evidence=[]
        with patch.object(carrier,'assess_carrier_face',side_effect=assess),patch.object(carrier,'validate_split',side_effect=ValueError('InjectedProjectionFailure')):
            result_v,result_f,_,receipt=carrier.refine_metric_carriers(Sphere(),v,faces,[],lambda p,r:np.eye(3),carrier.CarrierOraclePolicy(),
                derivative_diagnostics=False,evidence=evidence.append)
        self.assertFalse(receipt['materialization_authorized'])
        self.assertEqual(receipt['construction_refusal']['code'],'InjectedProjectionFailure')
        np.testing.assert_array_equal(np.array([p.point for p in result_v]),original)
        self.assertEqual([f.vertices for f in result_f],rows)
        self.assertTrue(evidence[-1]['face_decisions'])

if __name__=='__main__':unittest.main()
