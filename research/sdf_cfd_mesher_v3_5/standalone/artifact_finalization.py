"""Audit-only binding of saved operands to the current producer-checked state.

No geometry predicate is replaced here. The existing cold audit remains required.
The caller owns ``scope`` through the worker/supervisor guard handoff.
"""
from pathlib import Path
import json,subprocess,sys
import numpy as np
from immutable_state_access import guard
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash,identity
from finite_research.direct_evidence_modules import driver,joined_audit

SCHEMA='V35AuditedArtifactFinalization/1'
COMPLETE={'points':'points','prisms':'prisms','tetrahedra':'background_tets','wall':'wall','farfield':'farfield'}

def safe_path(folder,name):
    folder=Path(folder).resolve();raw=folder/name;path=raw.resolve()
    if Path(name).is_absolute() or Path(name).drive or '..' in Path(name).parts or not path.is_relative_to(folder) or path==folder:
        raise ValueError('FinalizationPathEscape',name)
    for part in (raw,*raw.parents):
        if part==folder:break
        if part.is_symlink() or part.is_junction():raise ValueError('FinalizationReparsePoint',name)
    if not path.is_file():raise ValueError('FinalizationFileMissing',name)
    return path

def signature(array):
    return dict(dtype=array.dtype.str,shape=list(array.shape),sha256=array_hash(array))

def compare_archive(path,expected,check):
    fields={k:v['dtype'] for k,v in expected.items()}
    actual=driver.arrays(path,fields,check)
    if {k:signature(v) for k,v in actual.items()}!=expected:raise ValueError('FinalizationArrayMismatch',str(path))

def expected_arrays(joined):
    cells=joined['cells'];byid={c['id']:c for c in cells};prefix=joined['prefix_ids'];prefix_set=set(prefix)
    if len(byid)!=len(cells) or len(prefix_set)!=len(prefix):raise ValueError('FinalizationCellIdentities')
    old=[byid[cid] for cid in prefix];core=[c for c in cells if c['id'] not in prefix_set]
    if not np.array_equal(np.asarray([c['vertices'] for c in core],dtype='<i8').reshape(-1,4),joined['bulk_tets']):
        raise ValueError('FinalizationCoreOrdering')
    return dict(points=joined['points'],background_tets=joined['bulk_tets'],farfield=joined['farfield'],
        prisms=np.asarray([c['vertices'] for c in old],dtype='<i8').reshape(-1,6),
        prism_ids=np.asarray(prefix,dtype='<i8'),tetrahedron_ids=np.asarray([c['id'] for c in core],dtype='<i8'),
        wall=joined['wall'],cap=joined['prefix_front'])

class AuditedBundle:
    def __init__(self,folder,joined,proof,scope,check):
        self.folder=Path(folder).resolve();self.scope=scope;self.check=check;self.held=set()
        self.stamp=joined_audit.geometry_identity(joined)
        if proof.get('whole_joined_geometry_passed') is not True or proof.get('geometry_identity')!=self.stamp:
            raise ValueError('FinalizationUnboundProducerProof')
        if set(proof.get('proof_receipts',{}))!=set(joined_audit.REQUIRED):raise ValueError('FinalizationProofPopulation')
        arrays=expected_arrays(joined)
        self.arrays={name:signature(value) for name,value in arrays.items()}
        self.complete={name:self.arrays[key] for name,key in COMPLETE.items()}
        metadata=dict(cell_metadata=[{k:v for k,v in c.items() if k not in ('id','vertices')} for c in joined['cells']],provenance=joined['provenance'])
        self.metadata_identity=identity(metadata)
        self.owners_identity=identity(repr(sorted(joined['owners'].items())))
        self.proof_identity=identity(proof)
        self.hold_existing()
        compare_archive(self.folder/'JOINED_EVIDENCE.npz',self.arrays,check)
        if identity(driver.read(self.folder/'JOINED_RECONSTRUCTION.json'))!=self.metadata_identity:
            raise ValueError('FinalizationMetadataMismatch')
        # Rebuild ownership from serialized canonical metadata/arrays. This is
        # an identity/conformity bridge, not a substitute for the twelve checks.
        from local_termination_topology import ownership
        actual,incidence=ownership(joined['cells'],list(map(tuple,arrays['farfield'])),list(map(tuple,arrays['wall'])))
        if not incidence['passed'] or identity(repr(sorted(actual.items())))!=self.owners_identity:
            raise ValueError('FinalizationOwnerMismatch')
        joined_audit.readback(self.folder/'audit',proof['proof_receipts'],self.stamp)

    def hold(self,name):
        if name not in self.held:
            path=safe_path(self.folder,name);self.scope.enter_context(guard(path));self.held.add(name)
        return safe_path(self.folder,name)

    def hold_existing(self):
        for path in sorted(self.folder.rglob('*')):
            if path.is_file() and path.name!='PROGRESS.json':self.hold(path.relative_to(self.folder).as_posix())

    def write_complete(self,joined):
        if joined_audit.geometry_identity(joined)!=self.stamp:raise ValueError('FinalizationChangedAuditedState')
        path=self.folder/'COMPLETE_MESH.npz'
        if path.exists():raise ValueError('FreshFinalizationArchiveRequired')
        arrays=expected_arrays(joined)
        np.savez(path,**{name:arrays[key] for name,key in COMPLETE.items()})
        return self.bind_complete()

    def bind_complete(self):
        """Compare an existing archive to expected producer operands, read-only."""
        path=self.hold('COMPLETE_MESH.npz');compare_archive(path,self.complete,self.check)
        self.archive_sha256=file_hash(path)
        return self.archive_sha256

    def accept_export(self,exported):
        self.hold_existing()
        if driver.read(self.folder/'export/EXPORT_RESULT.json')!=exported:raise ValueError('FinalizationExportReportMismatch')
        flags=('passed','conversion_fidelity','point_bijection_exact','cell_bijection_exact','all_face_incidence_and_winding_exact')
        if not all(exported.get(k) is True for k in flags) or exported.get('checkmesh',{}).get('passed') is not True:
            raise ValueError('FinalizationExportFidelity')
        if exported.get('source_archive_sha256')!=self.archive_sha256:raise ValueError('FinalizationExportInputMismatch')
        if file_hash(self.folder/'export/OPENFOAM_MAPPING.npz')!=exported['mapping_sha256']:
            raise ValueError('FinalizationExportMappingMismatch')
        pins=exported.get('native_files')
        if not pins or not any(n.startswith('foundation_audit/') for n in pins) or not any(n.startswith('openfoam_case/constant/polyMesh/') for n in pins):
            raise ValueError('FinalizationNativeExportInventory')
        for name,pin in pins.items():
            if file_hash(safe_path(self.folder/'export',name))!=pin:raise ValueError('FinalizationNativeExportMismatch',name)
        compare_archive(self.folder/'COMPLETE_MESH.npz',self.complete,self.check)
        self.export_identity=identity(exported)

    def seal(self,joined,proof):
        if joined_audit.geometry_identity(joined)!=self.stamp or identity(proof)!=self.proof_identity:
            raise ValueError('FinalizationChangedProducerRoot')
        self.hold_existing();self.check()
        files={name:file_hash(safe_path(self.folder,name)) for name in sorted(self.held)}
        if {'MANIFEST.json','FINALIZATION.json'} & set(files):raise ValueError('FreshFinalizationManifestRequired')
        receipt=dict(schema=SCHEMA,verification_policy='audit',production_bypass=False,geometry_identity=self.stamp,
            arrays=self.arrays,complete_arrays=self.complete,metadata_identity=self.metadata_identity,
            owners_identity=self.owners_identity,proof_identity=self.proof_identity,export_identity=self.export_identity,
            payload_identity=identity(files),scope='Producer-state/serialized/native-output binding; full independent audit still mandatory')
        durable_json(self.folder/'FINALIZATION.json',receipt);self.hold('FINALIZATION.json')
        files['FINALIZATION.json']=file_hash(self.folder/'FINALIZATION.json')
        durable_json(self.folder/'MANIFEST.json',dict(schema='V35DirectArtifactManifest/1',files=files,geometry_identity=self.stamp))
        self.hold('MANIFEST.json')
        self.descriptor=dict(schema=SCHEMA,verification_policy='audit',manifest_sha256=file_hash(self.folder/'MANIFEST.json'),
            finalization_sha256=files['FINALIZATION.json'],geometry_identity=self.stamp)
        return self.descriptor

def hold_manifest(folder,descriptor,scope):
    folder=Path(folder).resolve()
    if descriptor.get('schema')!=SCHEMA or descriptor.get('verification_policy')!='audit':raise ValueError('UnsupportedAuditFinalization')
    for name,pin in [('MANIFEST.json',descriptor['manifest_sha256']),('FINALIZATION.json',descriptor['finalization_sha256'])]:
        path=safe_path(folder,name);scope.enter_context(guard(path))
        if file_hash(path)!=pin:raise ValueError('FinalizationChangedExpectedRoot',name)
    manifest=driver.read(folder/'MANIFEST.json');receipt=driver.read(folder/'FINALIZATION.json')
    if manifest.get('schema')!='V35DirectArtifactManifest/1' or receipt.get('schema')!=SCHEMA or receipt.get('verification_policy')!='audit' or receipt.get('production_bypass') is not False:
        raise ValueError('UnsupportedFinalizationManifest')
    if not receipt['geometry_identity']==manifest['geometry_identity']==descriptor['geometry_identity']:
        raise ValueError('FinalizationGeometryRootMismatch')
    files=manifest['files'];payload={k:v for k,v in files.items() if k!='FINALIZATION.json'}
    if files.get('FINALIZATION.json')!=descriptor['finalization_sha256'] or identity(payload)!=receipt['payload_identity']:
        raise ValueError('FinalizationInventoryRootMismatch')
    for name,pin in files.items():
        path=safe_path(folder,name);scope.enter_context(guard(path))
        if file_hash(path)!=pin:raise ValueError('FinalizationChangedArtifact',name)
    return receipt

def run_independent_audit(folder,descriptor,remaining):
    """The only completion behavior: always launch the existing full reader."""
    folder=Path(folder);seconds=remaining()
    with (folder/'READBACK.stdout').open('w') as out,(folder/'READBACK.stderr').open('w') as err:
        done=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('direct_readback.py')),str(folder),descriptor['manifest_sha256'],str(seconds)],
            stdout=out,stderr=err,timeout=max(1,seconds))
    if done.returncode:raise ValueError('IndependentDirectReadbackRefusal',done.returncode)
    receipt=driver.read(folder/'READBACK.json')
    if receipt.get('passed') is not True or receipt.get('manifest_sha256')!=descriptor['manifest_sha256'] or receipt.get('all_twelve_semantically_replayed') is not True:
        raise ValueError('IndependentDirectReadbackRefusal')
    return receipt
