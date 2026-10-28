"""Separate process readback of completed V3.5/V4 direct-fill evidence."""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import geometry  # activates only the copied local V4 modules
import numpy as np
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash


def verify(folder,expected,seconds=10800):
    return _verify(Path(folder),expected,seconds)


def _verify(folder,expected,seconds,*,production=False,output=None):
    from finite_research.direct_evidence_modules import joined_audit,driver
    from finite_research.direct_fill_contract import require_binding,require_minima
    from finite_research.direct_contract import validate_policy
    from finite_research import direct_readback as existing_readback
    from direct_connection import _construction
    import local_termination_topology as topology
    if file_hash(folder/'MANIFEST.json')!=expected:raise ValueError('ChangedDirectManifest')
    manifest=driver.read(folder/'MANIFEST.json')
    deadline=time.monotonic()+seconds
    def check():
        if time.monotonic()>=deadline:raise TimeoutError('V35DirectReadbackDeadline')
        if file_hash(folder/'MANIFEST.json')!=expected:raise ValueError('ChangedDirectManifest')
    for name,pin in manifest['files'].items():
        path=(folder/name).resolve()
        if not path.is_relative_to(folder.resolve()) or file_hash(path)!=pin:raise ValueError('ChangedDirectArtifact',name)
    if manifest.get('schema')!=('V35ProductionArtifactManifest/1' if production else 'V35DirectArtifactManifest/1'):raise ValueError('UnsupportedDirectManifest')
    result=driver.read(folder/'PROVISIONAL.json');selected=validate_policy(driver.read(folder/'POLICY.json'))
    if result.get('provisional') is not True:raise ValueError('ExpectedProvisionalDirectResult')
    if production:
        from verification_policy import DIRECT,profile
        if (result.get('schema')!=DIRECT or result.get('acceptance_profile')!=profile() or result.get('verification_policy')!='production'
            or result.get('producer_checks_passed') is not True or result.get('geometry_complete') is not True
            or result.get('artifact_fidelity_passed') is not True or result.get('acceptance_status')!='provisional'
            or result.get('independent_audit_status')!='not_run' or result.get('audit')!='not_run'
            or 'full_mesh_complete' in result or 'passed' in result):raise ValueError('UnsupportedProductionEvidence')
        shape_contract=result['producer_shape_contract']
    else:
        if result.get('schema')!='V35V4DirectComplete/1' or result.get('passed') is not True or result.get('full_mesh_complete') is not False:raise ValueError('ExpectedProvisionalDirectResult')
        shape_contract=result['acceptance_contract']
    require_binding(shape_contract,selected);require_minima(result['shape'],selected)
    parent=driver.read(folder/'PARENT_EXPECTATIONS.json')
    if parent['binding']!=driver.read(folder/'PARENT.json') or parent['binding']!=result['parent']:raise ValueError('ChangedDirectParentBinding')
    import standalone_background,tet_layer_adapter
    helpers=dict(finite_stage_background=standalone_background,finite_stage_adapter=tet_layer_adapter)
    from contextlib import nullcontext
    from background_conditioning import replay_output_scope
    with replay_output_scope(Path(output).parent/'conditioning') if production else nullcontext():
        arrays,joined,replayed=existing_readback.semantics(folder,parent,selected,helpers,_construction(),check,Path(output) if output is not None else folder/'readback_audit')
    if joined['provenance']['live_parent'] is not True or joined['provenance']['generation_invocations']!=1:raise ValueError('FreshDirectGenerationAndParentRequired')
    with np.load(folder/'COMPLETE_MESH.npz',allow_pickle=False) as data:
        for name,key in dict(points='points',prisms='prisms',tetrahedra='background_tets',wall='wall',farfield='farfield').items():driver.same(data[name],arrays[key],'complete archive '+name)
    if joined_audit.geometry_identity(joined)!=manifest['geometry_identity'] or manifest['geometry_identity']!=result['geometry_identity']:raise ValueError('ChangedDirectReadbackGeometry')
    proof=result['proof']
    if not proof['whole_joined_geometry_passed'] or set(proof['proof_receipts'])!=set(joined_audit.REQUIRED):raise ValueError('IncompleteDirectProofSet')
    joined_audit.readback(folder/'audit',proof['proof_receipts'],result['geometry_identity'])
    exported=driver.read(folder/'export/EXPORT_RESULT.json')
    if exported!=result['export'] or exported['passed'] is not True or exported['conversion_fidelity'] is not True or exported['checkmesh']['passed'] is not True:raise ValueError('InvalidDirectNativeExport')
    for name,pin in manifest['files'].items():
        if file_hash(folder/name)!=pin:raise ValueError('ChangedDirectReadbackArtifact',name)
    return dict(schema='V35ProductionEvidenceSemanticReplay/1' if production else 'V35DirectIndependentReadback/1',passed=True,manifest_sha256=expected,geometry_identity=result['geometry_identity'],
        prisms=len(arrays['prisms']),tetrahedra=len(arrays['background_tets']),points=len(arrays['points']),proofs=len(proof['proof_receipts']),
        repair_replayed=True,conditioning_replayed='conditioning' in joined['provenance'],all_twelve_semantically_replayed=replayed['whole_joined_geometry_passed'],actual_shape_populations_recomputed=True,CFD_qualified=False)


if __name__=='__main__':
    from mesh import source_independence_guard
    source_independence_guard()
    from path_preserving_deduplication import durable_json
    folder=Path(sys.argv[1]).resolve()
    durable_json(folder/'READBACK.json',verify(folder,sys.argv[2],float(sys.argv[3])))
