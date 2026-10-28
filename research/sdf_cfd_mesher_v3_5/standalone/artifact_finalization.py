"""Binding of saved operands to the current producer-checked state.

No geometry predicate is replaced here. Explicit profiles control cold replay.
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
    def __init__(self,folder,joined,proof,scope,check,*,verification='audit'):
        if verification not in ('audit','production'):raise ValueError('UnknownVerificationMode')
        self.verification=verification;self.production_bindings=None
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

    def bind_production(self,joined,proof,selected,parent,binding,shape,exported,result,handoff,case_path):
        """Replace cold-only identity bridges using expected live producer roots.

        No geometric predicate, search or live authority is reconstructed here.
        Caller has just revalidated repair, live parent origin and pinned case.
        """
        from verification_policy import require_selection,profile,DIRECT,execution_record
        from finite_research.direct_fill_contract import binding as contract,acceptance
        from finite_research.direct_contract import method
        from finite_research.direct_readback import core_metadata
        arrays=expected_arrays(joined);n=len(arrays['prisms']);cells=joined['cells']
        case=driver.read(case_path);choice=require_selection(case.get('verification'),case,'production')
        if choice['stop_after']!='complete' or case['background']!=selected:raise ValueError('ProductionCasePolicyMismatch')
        if self.verification!='production' or joined_audit.geometry_identity(joined)!=self.stamp or identity(proof)!=self.proof_identity:
            raise ValueError('ProductionChangedAuditedState')
        if (binding.get('case_sha256')!=file_hash(case_path) or binding.get('policy_identity')!=identity(selected)
            or joined['provenance'].get('binding')!=binding or joined['provenance'].get('live_parent') is not True
            or joined['provenance'].get('generation_invocations')!=1 or joined['provenance'].get('construction_invocations')!=1):
            raise ValueError('ProductionParentProvenanceMismatch')
        if (parent.get('binding')!=binding or parent.get('prefix_points')!=joined['prefix_points']
            or parent.get('prefix_digest')!=joined['prefix_digest'] or parent.get('cells_identity')!=identity(cells[:n])
            or [c['id'] for c in cells[:n]]!=joined['prefix_ids']):raise ValueError('ProductionParentMetadataMismatch')
        expected={name:array_hash(arrays[name]) for name in ('prisms','prism_ids','wall','cap')}
        expected['points']=array_hash(arrays['points'][:joined['prefix_points']])
        if parent.get('array_hashes')!=expected:raise ValueError('ProductionPrefixArrayMismatch')
        rebuilt=core_metadata(arrays['background_tets'],arrays['tetrahedron_ids'],conditioned='conditioning' in joined['provenance'],first_id=int(max(arrays['prism_ids']))+1)
        if rebuilt!=cells[n:]:raise ValueError('ProductionNoncanonicalCoreMetadata')
        expected_handoff=dict(binding=binding,method_identity=identity(method()),geometry_identity=self.stamp,proof=proof,shape=shape,
            shape_applicability=acceptance(shape,selected),acceptance_contract=contract(selected))
        if handoff!=expected_handoff:raise ValueError('ProductionHandoffMismatch')
        if (result.get('schema')!=DIRECT or result.get('verification_policy')!='production' or result.get('acceptance_profile')!=profile()
            or result.get('audit')!='not_run' or result.get('provisional') is not True or result.get('acceptance_status')!='provisional'
            or result.get('producer_checks_passed') is not True or result.get('geometry_complete') is not True
            or result.get('artifact_fidelity_passed') is not True or result.get('independent_audit_status')!='not_run'
            or 'full_mesh_complete' in result or 'passed' in result
            or result.get('producer_shape_contract')!=contract(selected) or result.get('parent')!=binding
            or result.get('geometry_identity')!=self.stamp or result.get('proof')!=proof or result.get('shape')!=shape
            or result.get('export')!=exported or identity(exported)!=self.export_identity):raise ValueError('ProductionProvisionalMismatch')
        roots={'POLICY.json':selected,'PARENT.json':binding,'PARENT_EXPECTATIONS.json':parent,'PROVISIONAL.json':result,'EVIDENCE_HANDOFF.json':handoff}
        self.hold_existing()
        for name,value in roots.items():
            if driver.read(self.hold(name))!=value:raise ValueError('ProductionSavedRootMismatch',name)
        self.production_bindings={name:identity(value) for name,value in roots.items()}
        self.execution_record=execution_record(choice,proof)

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
        from verification_policy import FINALIZER,profile
        production=self.verification=='production'
        if production and self.production_bindings is None:raise ValueError('ProductionIdentityBridgeRequired')
        if production:
            for name,pin in self.production_bindings.items():
                if identity(driver.read(self.folder/name))!=pin:raise ValueError('ProductionChangedRoot',name)
        receipt=dict(schema=FINALIZER if production else SCHEMA,verification_policy=self.verification,production_bypass=False,geometry_identity=self.stamp,
            arrays=self.arrays,complete_arrays=self.complete,metadata_identity=self.metadata_identity,
            owners_identity=self.owners_identity,proof_identity=self.proof_identity,export_identity=self.export_identity,
            payload_identity=identity(files),scope='Producer-state/serialized/native-output binding')
        if production:receipt.update(acceptance_profile=profile(),producer_bindings=self.production_bindings,audit='not_run',execution_record=self.execution_record)
        durable_json(self.folder/'FINALIZATION.json',receipt);self.hold('FINALIZATION.json')
        files['FINALIZATION.json']=file_hash(self.folder/'FINALIZATION.json')
        durable_json(self.folder/'MANIFEST.json',dict(schema='V35ProductionArtifactManifest/1' if production else 'V35DirectArtifactManifest/1',files=files,geometry_identity=self.stamp))
        self.hold('MANIFEST.json')
        self.descriptor=dict(schema=receipt['schema'],verification_policy=self.verification,manifest_sha256=file_hash(self.folder/'MANIFEST.json'),
            finalization_sha256=files['FINALIZATION.json'],geometry_identity=self.stamp)
        if production:self.descriptor['acceptance_profile']=profile()
        return self.descriptor

def hold_manifest(folder,descriptor,scope):
    folder=Path(folder).resolve()
    from verification_policy import FINALIZER,profile
    production=descriptor.get('verification_policy')=='production'
    schema=FINALIZER if production else SCHEMA
    if descriptor.get('schema')!=schema or descriptor.get('verification_policy') not in ('audit','production') or (production and descriptor.get('acceptance_profile')!=profile()):raise ValueError('UnsupportedAuditFinalization')
    for name,pin in [('MANIFEST.json',descriptor['manifest_sha256']),('FINALIZATION.json',descriptor['finalization_sha256'])]:
        path=safe_path(folder,name);scope.enter_context(guard(path))
        if file_hash(path)!=pin:raise ValueError('FinalizationChangedExpectedRoot',name)
    manifest=driver.read(folder/'MANIFEST.json');receipt=driver.read(folder/'FINALIZATION.json')
    if (manifest.get('schema')!=('V35ProductionArtifactManifest/1' if production else 'V35DirectArtifactManifest/1') or receipt.get('schema')!=schema
        or receipt.get('verification_policy')!=descriptor['verification_policy'] or receipt.get('production_bypass') is not False
        or (production and (receipt.get('acceptance_profile')!=profile() or receipt.get('audit')!='not_run' or not receipt.get('producer_bindings')))):
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
