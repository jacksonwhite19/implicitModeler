"""Real serialized binding faults; one tiny native export, no mesh campaign.

The small synthetic producer receipts model the trusted precondition only.
These tests do not qualify their tiny shell against the twelve mesh claims.
Actual finalizer decoding, guards, export correspondence and cold failure run.
"""
from pathlib import Path
from contextlib import ExitStack
import copy,json,shutil,sys,tempfile,unittest,uuid
from functools import lru_cache
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import geometry
from artifact_finalization import AuditedBundle,expected_arrays,hold_manifest,run_independent_audit
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
from path_preserving_deduplication import durable_json
from finite_research.direct_evidence_modules import joined_audit
from background_materialization import build
from local_termination_topology import ownership
from export_mesh import export

def fixture(folder):
    folder=Path(folder);folder.mkdir(parents=True)
    inner=np.array([[0.,0.,0.],[.001,0.,0.],[0.,.001,0.],[0.,0.,.001]],dtype='<f8')
    outer=inner.mean(axis=0)+2*(inner-inner.mean(axis=0));points=np.vstack((inner,outer))
    faces=np.asarray(build(np.array([[0,1,2,3]],dtype='<i8'),1)[0]['faces'],dtype='<i8')
    patterns=((0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5));cells=[]
    for i,f in enumerate(faces):
        row=np.r_[f,f+4].tolist();cells.append(dict(id=i+1,kind='prism',vertices=row,faces=[tuple(row[j] for j in p) for p in patterns]))
    tets=np.empty((0,4),dtype='<i8');wall=faces[:,::-1];far=faces+4
    owners,qa=ownership(cells,list(map(tuple,far)),list(map(tuple,wall)));assert qa['passed']
    joined=dict(points=points,bulk_tets=tets,prefix_front=far,farfield=far,wall=wall,bounds=np.array([[-.01,.01]]*3),
        cells=cells,owners=owners,prefix_ids=[1,2,3,4],prefix_points=8,prefix_digest='test-prefix',provenance=dict(test_precondition_only=True))
    np.savez(folder/'JOINED_EVIDENCE.npz',**expected_arrays(joined))
    durable_json(folder/'JOINED_RECONSTRUCTION.json',dict(cell_metadata=[{k:v for k,v in c.items() if k not in ('id','vertices')} for c in cells],provenance=joined['provenance']))
    audit=folder/'audit';audit.mkdir();stamp=joined_audit.geometry_identity(joined);receipts={}
    for name in joined_audit.REQUIRED:
        value=dict(geometry_identity=stamp,wall_contract=joined_audit.SCHEMA,synthetic_producer_precondition=True)
        path=audit/(name.upper()+'.json');durable_json(path,value);receipts[name]=dict(identity=identity(value),sha256=file_hash(path))
    proof=dict(whole_joined_geometry_passed=True,geometry_identity=stamp,proof_receipts=receipts)
    durable_json(folder/'PROVISIONAL.json',dict(test_precondition_only=True))
    return joined,proof

@lru_cache(maxsize=1)
def export_control():
    base=ROOT/'results'/('phase1a_native_export_control_'+uuid.uuid4().hex[:8])
    joined,proof=fixture(base)
    with ExitStack() as scope:
        bundle=AuditedBundle(base,joined,proof,scope,lambda:None);pin=bundle.write_complete(joined)
        # Actual gmshToFoam / contractAudit / checkMesh on four fixed prisms.
        import export_mesh
        original=export_mesh.execute;denied=[]
        def native_boundary(name,case,*args,**kwargs):
            result=original(name,case,*args,**kwargs)
            if name=='contractAudit':
                path=case/'constant/polyMesh/points'
                try:path.write_bytes(b'mutate after native audit but before checkMesh')
                except PermissionError:denied.append(True)
                else:raise AssertionError('polyMesh was writable between native readers')
            return result
        with patch('export_mesh.execute',side_effect=native_boundary):
            report=export(base/'COMPLETE_MESH.npz',base/'export',_custody=scope,_expected_archive_sha256=pin)
        assert denied==[True]
        bundle.accept_export(report);descriptor=bundle.seal(joined,proof)
    durable_json(base.parent/(base.name+'_BOUNDARY_GUARD_TEST.json'),dict(passed=True,mutation_after_native_audit_denied=True,
        native_export=str(base/'export/EXPORT_RESULT.json'),scope='Four-prism conversion/readback fault test; not new scientific qualification'))
    return base,joined,proof,descriptor,report

class ArtifactFinalization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.base,cls.joined,cls.proof,cls.descriptor,cls.exported=export_control()

    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.folder=Path(self.tmp.name)/'bundle'
        shutil.copytree(self.base,self.folder)
        (self.folder/'MANIFEST.json').unlink();(self.folder/'FINALIZATION.json').unlink()
        self.scope=ExitStack();self.addCleanup(self.tmp.cleanup);self.addCleanup(self.scope.close)

    def bundle(self):return AuditedBundle(self.folder,self.joined,self.proof,self.scope,lambda:None)

    def test_valid_serialized_native_binding_and_guards(self):
        b=self.bundle();b.bind_complete();b.accept_export(copy.deepcopy(self.exported));d=b.seal(self.joined,self.proof)
        receipt=hold_manifest(self.folder,d,self.scope)
        self.assertEqual(receipt['geometry_identity'],self.proof['geometry_identity'])
        self.assertFalse(receipt['production_bypass'])
        for path in ('COMPLETE_MESH.npz','JOINED_RECONSTRUCTION.json','export/foundation_audit/points.f64'):
            with self.assertRaises(PermissionError):(self.folder/path).write_bytes(b'late replacement')

    def test_structurally_valid_different_mesh_is_not_accepted_by_new_hash(self):
        p=self.folder/'COMPLETE_MESH.npz'
        with np.load(p) as a:values={k:a[k] for k in a.files}
        values['points']=values['points'].copy();values['points'][1,0]*=1.1
        np.savez(p,**values)
        # The NPZ is valid, and hashing it succeeds; it is not audited state S.
        self.assertEqual(len(file_hash(p)),64)
        with self.assertRaisesRegex(ValueError,'FinalizationArrayMismatch'):self.bundle().bind_complete()

    def test_truncated_corrupt_and_dtype_changed_archives(self):
        p=self.folder/'COMPLETE_MESH.npz';original=p.read_bytes()
        for content in (original[:40],b'not a zip archive'):
            p.write_bytes(content)
            with self.assertRaises(Exception):self.bundle().bind_complete()
            self.scope.close();self.scope=ExitStack();self.addCleanup(self.scope.close)
        p.write_bytes(original)
        with np.load(p) as a:values={k:a[k] for k in a.files}
        values['tetrahedra']=values['tetrahedra'].astype('<i4');np.savez(p,**values)
        with self.assertRaises(ValueError):self.bundle().bind_complete()

    def test_metadata_owner_and_expected_proof_faults(self):
        p=self.folder/'JOINED_RECONSTRUCTION.json';m=json.loads(p.read_text());m['cell_metadata'][0]['faces'][0].reverse();durable_json(p,m)
        with self.assertRaisesRegex(ValueError,'FinalizationMetadataMismatch'):self.bundle()
        self.scope.close();self.scope=ExitStack();self.addCleanup(self.scope.close)
        shutil.copyfile(self.base/'JOINED_RECONSTRUCTION.json',p)
        wrong=copy.deepcopy(self.joined);wrong['owners']={}
        forged=copy.deepcopy(self.proof);forged['geometry_identity']=joined_audit.geometry_identity(wrong)
        # Even a self-consistent new stamp cannot make wrong ownership valid.
        with self.assertRaisesRegex(ValueError,'FinalizationOwnerMismatch'):
            AuditedBundle(self.folder,wrong,forged,self.scope,lambda:None)

    def test_stale_expected_stamp_refuses(self):
        proof=copy.deepcopy(self.proof);proof['geometry_identity']='0'*64
        with self.assertRaisesRegex(ValueError,'FinalizationUnboundProducerProof'):
            AuditedBundle(self.folder,self.joined,proof,self.scope,lambda:None)

    def test_boundary_export_and_native_mapping_mismatch(self):
        report=copy.deepcopy(self.exported);report['source_archive_sha256']='0'*64
        durable_json(self.folder/'export/EXPORT_RESULT.json',report)
        b=self.bundle();b.bind_complete()
        with self.assertRaisesRegex(ValueError,'FinalizationExportInputMismatch'):b.accept_export(report)

    def test_actual_native_boundary_corruption_fails_pinned_inventory(self):
        path=self.folder/'export/openfoam_case/constant/polyMesh/boundary'
        path.write_bytes(path.read_bytes().replace(b'farfield',b'badpatch'))
        b=self.bundle();b.bind_complete()
        with self.assertRaisesRegex(ValueError,'FinalizationNativeExportMismatch'):b.accept_export(self.exported)

    def test_stale_manifest_and_publication_failure_have_no_success(self):
        b=self.bundle();b.bind_complete();b.accept_export(self.exported)
        with patch('artifact_finalization.durable_json',side_effect=OSError('injected before atomic publication')):
            with self.assertRaises(OSError):b.seal(self.joined,self.proof)
        self.assertFalse((self.folder/'FINALIZATION.json').exists());self.assertFalse((self.folder/'MANIFEST.json').exists())
        d=b.seal(self.joined,self.proof);d=dict(d,manifest_sha256='0'*64)
        with self.assertRaisesRegex(ValueError,'FinalizationChangedExpectedRoot'):hold_manifest(self.folder,d,self.scope)

    def test_real_cold_reader_failure_is_mandatory(self):
        b=self.bundle();b.bind_complete();b.accept_export(self.exported);d=b.seal(self.joined,self.proof)
        # The synthetic producer precondition is deliberately not full legacy
        # evidence. Invoke the real reader: it must fail, never bypass to pass.
        with self.assertRaisesRegex(ValueError,'IndependentDirectReadbackRefusal'):
            run_independent_audit(self.folder,d,lambda:20.)
        self.assertFalse((self.folder/'RESULT.json').exists())
        self.assertTrue((self.folder/'READBACK.stderr').stat().st_size)

if __name__=='__main__':unittest.main()
