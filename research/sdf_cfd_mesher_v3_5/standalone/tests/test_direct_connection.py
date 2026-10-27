import sys,json,tempfile,unittest,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import geometry
import numpy as np
from direct_connection import policy,_construction


class DirectConnection(unittest.TestCase):
    def test_policy_has_no_finite_band_and_retains_v4_contract(self):
        from standalone_background import validate_policy,assurance_mode
        from finite_research.direct_contract import method,validate_policy as validate
        p=policy([[-.008,.008]]*3,.26236426446749106,.001)
        np.testing.assert_array_equal(validate_policy(p),[[-.008,.008]]*3)
        self.assertEqual(assurance_mode(p),'backend_trusted')
        self.assertNotIn('tet_transition',p)
        self.assertFalse(method()['transition'])
        self.assertEqual(p['acceptance_contract_version'],'DirectFillEngineering/1')
        for change in ({'acceptance_contract_identity':'0'*64},{'backend_method':'HXT4DirectSizing095/1'},{'quality_targets':{}}):
            with self.assertRaises(ValueError):validate(dict(p,**change))

    def test_worker_rejects_input_changes_before_native_execution(self):
        import direct_worker
        from sdf_cfd_mesher_v4.common.authority import file_hash
        with tempfile.TemporaryDirectory() as raw:
            folder=Path(raw);(folder/'CAP.npz').write_bytes(b'original')
            request=dict(files={'CAP.npz':file_hash(folder/'CAP.npz')})
            (folder/'INPUT.json').write_text(json.dumps(request));pin=file_hash(folder/'INPUT.json')
            (folder/'CAP.npz').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'ChangedDirectWorkerInput'):direct_worker.execute(folder,pin)
            with self.assertRaisesRegex(ValueError,'ChangedDirectWorkerRequest'):direct_worker.execute(folder,'0'*64)

    def test_worker_uses_portable_evidence_writer(self):
        import direct_worker
        with tempfile.TemporaryDirectory() as raw:
            path=Path(raw)/'evidence.json';value=dict(passed=True,meshing_invoked=False)
            direct_worker.save(path,value)
            self.assertEqual(json.loads(path.read_text()),value)
        self.assertEqual(direct_worker.save.__module__,'finite_research.direct_evidence_modules.variant')

    def test_prefix_preservation_detects_mutation(self):
        construction=_construction()
        cells=[dict(id=1,kind='prism',vertices=[0,1,2,3,4,5])]
        owners={(0,1,2):[(1,(0,2,1))]}
        check=lambda:None
        digest=construction.inherited_digest(cells,owners,check)
        self.assertTrue(construction.verify_inherited_join(dict(cells=cells,owners=owners),cells,owners,digest,check)['passed'])
        changed=copy.deepcopy(cells);changed[0]['vertices'][0]=6
        with self.assertRaisesRegex(ValueError,'ChangedInheritedCellJoin'):
            construction.verify_inherited_join(dict(cells=changed,owners=owners),cells,owners,digest,check)

    def test_independent_reader_rejects_artifact_mutation(self):
        from direct_readback import verify
        from sdf_cfd_mesher_v4.common.authority import file_hash
        with tempfile.TemporaryDirectory() as raw:
            folder=Path(raw);(folder/'payload.json').write_text('{}')
            (folder/'MANIFEST.json').write_text(json.dumps(dict(files={'payload.json':file_hash(folder/'payload.json')})))
            pin=file_hash(folder/'MANIFEST.json');(folder/'payload.json').write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError,'ChangedDirectArtifact'):verify(folder,pin)
            with self.assertRaisesRegex(ValueError,'ChangedDirectManifest'):verify(folder,'0'*64)

    def test_shared_native_budget_cannot_be_reset_per_stage(self):
        from direct_connection import NativeBudget
        now=[0.];budget=NativeBudget(10,clock=lambda:now[0])
        first=budget.clock();now[0]=4.;budget.charge(first)
        self.assertEqual(budget.remaining(),6.)
        second=budget.clock();now[0]=9.5;budget.charge(second)
        with self.assertRaisesRegex(TimeoutError,'SharedNativeBudget'):budget.remaining()
        last=budget.clock();now[0]=11.
        with self.assertRaisesRegex(TimeoutError,'SharedNativeBudgetOverrun'):budget.charge(last)

    def test_streaming_hash_compatibility(self):
        import io,hashlib
        from direct_compat import file_digest
        for data in (b'',b'abc',b'0123456789'*300000):
            self.assertEqual(file_digest(io.BytesIO(data),'sha256').hexdigest(),hashlib.sha256(data).hexdigest())

    def test_existing_reader_rejects_substituted_parent_arrays(self):
        from finite_research.direct_readback import joined
        from sdf_cfd_mesher_v4.common.authority import array_hash
        with tempfile.TemporaryDirectory() as raw:
            folder=Path(raw)
            arrays=dict(points=np.arange(21,dtype='<f8').reshape(7,3),prisms=np.array([[0,1,2,3,4,5]],dtype='<i8'),
                prism_ids=np.array([1],dtype='<i8'),background_tets=np.array([[0,1,2,6]],dtype='<i8'),tetrahedron_ids=np.array([2],dtype='<i8'),
                wall=np.array([[0,2,1]],dtype='<i8'),cap=np.array([[3,4,5]],dtype='<i8'),farfield=np.array([[0,1,6]],dtype='<i8'))
            np.savez(folder/'JOINED_EVIDENCE.npz',**arrays)
            parent=dict(prefix_points=6,array_hashes={k:array_hash(v) for k,v in arrays.items()})
            parent['array_hashes']['prisms']='0'*64
            with self.assertRaisesRegex(ValueError,'independent parent prisms'):joined(folder,parent,{},lambda:None)

    def test_collision_reader_binds_actual_face_population(self):
        from finite_research.direct_readback import collision
        from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash,identity
        with tempfile.TemporaryDirectory() as raw:
            folder=Path(raw);audit=folder/'audit';directory=audit/'collision_witnesses';directory.mkdir(parents=True)
            points=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],dtype='<f8')
            np.savez(directory/'FACE_POPULATION.npz',offsets=np.array([0,3],dtype='<i8'),vertices=np.array([0,1,2],dtype='<i8'),active=np.array([0],dtype='<i8'))
            # Hash-valid objects reach the population comparison, which must
            # reject changed winding before attempting proof decoding.
            (directory/'proof_object').write_bytes(b'not decoded on this rejection path')
            obj=dict(file='proof_object',sha256=file_hash(directory/'proof_object'))
            witness=dict(points_identity=array_hash(points),face_population_sha256=file_hash(directory/'FACE_POPULATION.npz'),proof_hash='x',segment={k:obj for k in ('faces','pairs','codes','payloads')})
            (directory/'WITNESSES.json').write_text(json.dumps(witness))
            receipt=dict(witness_directory=directory.name,witnesses_sha256=file_hash(directory/'WITNESSES.json'),witness_identity=identity(witness),proof_hash='x')
            (audit/'COLLISION.json').write_text(json.dumps(receipt))
            with self.assertRaisesRegex(ValueError,'collision face population'):
                collision(folder,'collision',points,[(0,2,1)],lambda:None)

    def test_cap_compatibility_reuses_existing_minimum(self):
        from direct_connection import cap_compatibility
        from finite_support.transition_front_targets import triangle_ratio,population_report
        selected=policy([[-.008,.008]]*3,.26236426446749106,.001)
        tri=np.array([[0,1,2]],dtype='<i8')
        for points in (np.array([[0.,0.,0.],[1.,0.,0.],[.5,.8660254037844386,0.]],dtype='<f8'),
                       np.array([[0.,0.,0.],[1.,0.,0.],[.5,.001,0.]],dtype='<f8')):
            expected=population_report(triangle_ratio(points,tri),selected['quality_targets']['outer_triangles'])
            actual=cap_compatibility(points,tri,selected)
            self.assertEqual(actual['passed'],expected['minimum_passed'])
            self.assertEqual(actual['population']['minimum'],expected['minimum'])
        zero=np.zeros((3,3),dtype='<f8')
        with self.assertRaisesRegex(ValueError,'NonpositiveOrNonfinite'):cap_compatibility(zero,tri,selected)


if __name__=='__main__':unittest.main()
