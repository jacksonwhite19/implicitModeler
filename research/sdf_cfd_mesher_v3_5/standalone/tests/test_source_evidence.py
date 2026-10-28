"""Retention/binding controls with declared synthetic source premises."""
from pathlib import Path
from contextlib import ExitStack
import copy,json,sys,tempfile,unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import geometry,source_evidence as evidence
from source_handoff import VERSION
from sdf_cfd_mesher_v4.common.authority import array_hash,identity

class SourceEvidence(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.folder=Path(self.temp.name)
        self.positions=np.asarray([[[0.,0.,0.]],[[0.,0.,1.]],[[0.,0.,2.]]]);self.h1=1.
        self.rows=[dict(passed=True,segments=1,segment_proofs=[{'synthetic_source_precondition':True}],geometry_identity='source',
            start=array_hash(self.positions[k]),end=array_hash(self.positions[k+1]),first_source_normal_height_m=1. if k==0 else None,
            exact_field_progress=True,positive_endpoints=True) for k in range(2)]
        self.segments=dict(passed=True,layers=self.rows)
        self.evaluation=dict(schema=VERSION,mode='production',method='live_final_population_authenticated',independent_source_recomputation=False,layers=2,segments=2)

    def validate(self,evaluation=None,segments=None,positions=None,proof_identity=None):
        evidence.validate(self.evaluation if evaluation is None else evaluation,self.segments if segments is None else segments,
            self.positions if positions is None else positions,'source',self.h1,identity(self.rows) if proof_identity is None else proof_identity)

    def test_both_effective_methods_and_held_exact_producer_values(self):
        self.validate()
        self.validate(dict(self.evaluation,method='recomputed',independent_source_recomputation=True))
        with ExitStack() as scope:
            evidence.retain(self.folder,self.evaluation,self.segments,self.positions,'source',self.h1,scope)
            for name in ('SOURCE_EVALUATION.json','SOURCE_SEGMENTS.json'):
                with self.assertRaises(PermissionError):(self.folder/name).write_text('{}')
        with self.assertRaisesRegex(ValueError,'LifetimeRequired'):
            evidence.retain(self.folder,self.evaluation,self.segments,self.positions,'source',self.h1,None)

    def test_producer_write_substitution_refused(self):
        original=evidence.durable_json
        def substitute(path,value):original(path,dict(value,method='substituted'))
        with ExitStack() as scope,patch.object(evidence,'durable_json',side_effect=substitute),self.assertRaisesRegex(ValueError,'ChangedProducer'):
            evidence.retain(self.folder,self.evaluation,self.segments,self.positions,'source',self.h1,scope)

    def test_mode_method_count_and_recomputation_mutations_refused(self):
        for key,value in [('schema','unknown'),('mode','audit'),('method','historical_cache'),('independent_source_recomputation',True),('layers',1),('segments',1)]:
            with self.subTest(key=key),self.assertRaises(ValueError):self.validate(dict(self.evaluation,**{key:value}))

    def test_complete_population_binding_and_actual_operands(self):
        for key,value in [('start','changed'),('end','changed'),('geometry_identity','other'),('first_source_normal_height_m',2.),('segments',0),('segment_proofs',[]),('positive_endpoints',False)]:
            rows=copy.deepcopy(self.rows);rows[0][key]=value
            # Even if an attacker substitutes the declared proof identity,
            # actual source/positions/H1 and full-population checks still fail.
            with self.subTest(key=key),self.assertRaises(ValueError):self.validate(segments=dict(passed=True,layers=rows),proof_identity=identity(rows))
        with self.assertRaises(ValueError):self.validate(proof_identity='other')
        with self.assertRaises(ValueError):self.validate(segments=dict(passed=True,layers=self.rows[:1]))
        positions=self.positions.copy();positions[1,0,0]=.1
        with self.assertRaises(ValueError):self.validate(positions=positions)

if __name__=='__main__':unittest.main()
