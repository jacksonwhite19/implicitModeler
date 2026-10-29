"""Copy an authenticated accepted mesh to a distinct, non-replay delivery.

Existing-run export never removes or rewrites the source. Trusted hashes must
come from the successful supervisor/audit publication, not arbitrary files.
"""
from source_process_owner import worker_start_gate
worker_start_gate()
from pathlib import Path
from contextlib import ExitStack
import argparse,hashlib,json,os,shutil,uuid
import geometry
from compact_paths import checked,member,hold_tree,hold_parents,inventory,publish,Handle,file_id,allowed_parent
from immutable_state_access import guard
from artifact_finalization import hold_manifest

SCHEMA='V35CompactSolverDelivery/1'
POLY='export/openfoam_case/constant/polyMesh/'
REQUIRED={'points','faces','owner','neighbour','boundary'}


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(path,value):
    return write_bytes(path,(json.dumps(value,indent=2,allow_nan=False)+'\n').encode('utf-8'))

def write_bytes(path,payload):
    with Path(path).open('xb') as f:
        f.write(payload);f.flush();os.fsync(f.fileno())
    return hashlib.sha256(payload).hexdigest()
def require(ok,message):
    if not ok:raise ValueError(message)


def copy_payload(source,target):
    with source.open('rb') as src,target.open('xb') as dst:
        shutil.copyfileobj(src,dst);dst.flush();os.fsync(dst.fileno())


def authenticate_source(folder,expected,scope):
    """Production uses the unchanged reader; audit mirrors published gates."""
    root=member(folder,'VERIFICATION.json');scope.enter_context(guard(root))
    require(sha(root)==expected,'ChangedCompactSourceRoot')
    value=read(root)
    if value.get('verification_policy')=='production':
        from offline_audit import authenticate
        return authenticate(folder,expected,scope)
    require(value.get('schema')=='V35VerificationManifest/1' and value.get('status')=='audit_accepted'
        and value.get('verification_policy')=='audit' and value.get('production_bypass') is False
        and value.get('independent_semantic_replay')=='passed' and value.get('producer_artifact_binding')=='passed'
        and value.get('source_custody')=='closed_passed' and value.get('worker_exit_code')==0
        and value.get('supervisor_stop_reason') is None,'UnsupportedCompactAuditAcceptance')
    from audit_completion import EXTRA,REQUEST,SCHEMA as HANDOFF
    pins=value.get('completion_files',{});require(set(pins)==set(EXTRA),'IncompleteCompactAuditCompletion')
    for name,pin in {**pins,REQUEST:value['handoff_sha256']}.items():
        p=member(folder,name);scope.enter_context(guard(p));require(sha(p)==pin,'ChangedCompactAuditArtifact')
    request=read(folder/REQUEST)
    require(request.get('schema')==HANDOFF and request.get('verification_policy')=='audit'
        and request.get('files')==pins and request.get('finalization')==value.get('finalization'),'ChangedCompactAuditHandoff')
    closure=value.get('closure_records',{});require(set(closure)=={'source','supervisor'},'IncompleteCompactAuditClosure')
    records={}
    for key,pin in closure.items():
        p=checked(pin['path']);require(p.is_absolute() and p.is_file(),'MissingCompactAuditClosure')
        hold_parents(p.parent,scope);scope.enter_context(guard(p));require(sha(p)==pin['sha256'],'ChangedCompactAuditClosure')
        records[key]=read(p)
    sources=records['source'];supervisor=records['supervisor']
    require(sources.get('state')=='closed' and sources.get('passed') is True and not sources.get('changed') and not sources.get('added')
        and sources.get('identity')==value.get('source_identity')==supervisor.get('source_identity')
        and supervisor.get('source_custody_passed') is True and supervisor.get('exit_code')==0
        and supervisor.get('stop_reason') is None and supervisor.get('monitor_error') is None,'UnclosedCompactAuditSource')
    hold_manifest(folder/'background',value['finalization'],scope)
    root_result=read(folder/'RESULT.json');direct=read(folder/'background/RESULT.json');cold=read(folder/'background/READBACK.json')
    require(root_result.get('passed') is True and root_result.get('full_mesh_complete') is True and root_result.get('background')==direct
        and direct.get('schema')=='V35V4DirectComplete/1' and direct.get('passed') is True and direct.get('full_mesh_complete') is True
        and direct.get('provisional') is False and direct.get('audit_finalization')==value['finalization']
        and direct.get('manifest_sha256')==value['finalization']['manifest_sha256'],'IncompleteCompactAuditRoot')
    require(cold.get('schema')=='V35DirectIndependentReadback/1' and cold.get('passed') is True
        and cold.get('all_twelve_semantically_replayed') is True and cold.get('manifest_sha256')==value['finalization']['manifest_sha256']
        and direct.get('readback_sha256')==pins['background/READBACK.json'],'IncompleteCompactColdReplay')
    return value


def audit_record(path,expected,source_root,scope):
    require(bool(path)==bool(expected),'AuditReceiptAndTrustedHashRequiredTogether')
    if path is None:return None
    path=checked(path);hold_parents(path.parent,scope);scope.enter_context(guard(path))
    require(sha(path)==expected,'ChangedCompactAuditReceipt')
    value=read(path);p=member(path.parent,'VERIFIER_SOURCES.json');scope.enter_context(guard(p));sources=read(p)
    require(value.get('schema')=='V35RetainedDirectAudit/1' and value.get('status')=='audit_passed' and value.get('passed') is True
        and value.get('source_bundle_root')==source_root and value.get('fresh_source_authority') is False
        and value.get('remeshed') is False and value.get('search_rerun') is False
        and value.get('replay',{}).get('all_twelve_semantically_replayed') is True and value['replay'].get('proofs')==12
        and sources.get('state')=='closed' and sources.get('passed') is True and not sources.get('changed') and not sources.get('added')
        and sources.get('identity')==value.get('verifier_identity'),'InvalidCompactSeparateAudit')
    return value


def summary(root,result,audit,repair_report):
    direct=result['background'];shape={}
    for name,value in direct['shape'].items():shape[name]={k:v for k,v in value.items() if k!='selected_ids'}
    repair=direct['repair']
    return dict(schema='V35CompactChecksPerformed/1',records_only=True,replay_proofs=False,
        scientific_status=root['status'],verification_policy=root['verification_policy'],runtime_identity=root['source_identity'],
        acceptance_profile=root.get('acceptance_profile'),geometry_identity=direct['geometry_identity'],
        counts=dict(points=direct['points'],prisms=direct['prism_cells'],tetrahedra=direct['core_cells']),
        requested=result['requested'],achieved={k:v for k,v in result['achieved'].items() if k!='path_totals_m'},shape=shape,
        mesh_quality=direct['export']['checkmesh'],conditioning=direct['conditioning'],repair=repair,repair_assurance=repair_report,
        producer_checks=direct['proof']['proof_receipts'],separate_audit=audit,
        limits=['Shared-apex classification and source-float versus exact changed-star limits remain as recorded in repair.',
                'No global exact-Q or continuous whole-source/SDF-fluid proof. Nominal surface size is a request.',
                'Solver mesh only: fields, boundary conditions and solver dictionaries are not supplied or CFD-qualified.'],
        CFD_qualified=False,full_offline_replay_available=False)


def verify(folder,expected=None,*,retained_scope=None):
    folder=checked(folder)
    with ExitStack() as scope:
        if retained_scope is not None:scope=retained_scope
        hold_tree(folder,scope)
        path=member(folder,'COMPACT.json');scope.enter_context(guard(path))
        if expected is not None:require(sha(path)==expected,'ChangedCompactDeliveryRoot')
        value=read(path)
        require(value.get('schema')==SCHEMA and value.get('status')=='solver_delivery_complete'
            and value.get('full_offline_replay_available') is False and value.get('reports_are_replay_proofs') is False,'UnsupportedCompactDelivery')
        files=value.get('files');require(isinstance(files,dict) and bool(files),'IncompleteCompactInventory')
        require(len({n.casefold() for n in files})==len(files),'CompactCaseCollision')
        actual={p.relative_to(folder).as_posix() for p in inventory(folder)}
        require(actual==set(files)|{'COMPACT.json'},'ChangedCompactFilePopulation')
        for name,pin in files.items():
            p=member(folder,name);scope.enter_context(guard(p))
            require(p.is_file() and type(pin.get('bytes')) is int and p.stat().st_size==pin['bytes'] and sha(p)==pin.get('sha256'),'ChangedCompactPayload')
        mesh={n.removeprefix('polyMesh/') for n in files if n.startswith('polyMesh/')}
        require(REQUIRED<=mesh and mesh==set(value.get('solver_mesh_files',[])),'IncompleteCompactSolverMesh')
        require({'CASE.json','geometry.rhai','CHECKS.json','PROVENANCE.json','DELIVERY_OWNER.json','CONSUMER.json'}<=set(files),'IncompleteCompactMetadata')
        for key in ('source_verification_sha256','source_background_manifest_sha256','source_native_export_sha256'):
            p=value.get(key);require(isinstance(p,str) and len(p)==64 and all(c in '0123456789abcdef' for c in p),'MissingCompactTrustedRoots')
        require(read(folder/'CHECKS.json').get('full_offline_replay_available') is False,'CompactReplayCapabilityConflict')
        return value


def export(folder,expected,destination,*,audit_path=None,audit_expected=None,provenance=None):
    folder=checked(folder);destination=checked(destination,exists=False)
    allowed_parent(destination.parent)
    require(not destination.exists(),'FreshCompactDestinationRequired')
    require(not destination.is_relative_to(folder) and not folder.is_relative_to(destination),'CompactSourceDestinationOverlap')
    # Fresh reserved directory remains provisional until the commit marker.
    stage=destination
    with ExitStack() as scope:
        from source_custody import selected,ROOT
        from sdf_cfd_mesher_v4.common.authority import identity
        current=selected(destination)
        for p in current:scope.enter_context(guard(p))
        rows=[dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size,sha256=sha(p)) for p in current]
        consumer=dict(schema='V35CompactConsumerSource/1',identity=identity(rows),files=rows,custody='held_through_publication',
            scope='Consumer source bytes only; producer evidence remains separately authenticated. No numerical or portability qualification.')
        hold_tree(folder,scope);hold_parents(destination.parent,scope)
        root=authenticate_source(folder,expected,scope)
        inner=read(folder/'background/MANIFEST.json');result=read(folder/'RESULT.json');direct=result['background']
        audit=audit_record(audit_path,audit_expected,expected,scope)
        exported=direct['export'];export_receipt=member(folder,'background/export/EXPORT_RESULT.json')
        require(exported==read(export_receipt) and exported.get('passed') is True and exported.get('conversion_fidelity') is True
            and exported.get('point_bijection_exact') is True and exported.get('cell_bijection_exact') is True
            and exported.get('all_face_incidence_and_winding_exact') is True and exported.get('checkmesh',{}).get('literal_mesh_ok') is True,'UnacceptedCompactNativeExport')
        require(inner['files'].get('export/EXPORT_RESULT.json')==sha(export_receipt),'UnboundCompactNativeExport')
        meshroot=member(folder,'background/'+POLY.rstrip('/'))
        actual={p.relative_to(meshroot).as_posix() for p in inventory(meshroot)}
        pinned={name[len(POLY):]:pin for name,pin in inner['files'].items() if name.startswith(POLY)}
        require(actual==set(pinned) and REQUIRED<=actual,'UnpinnedOrIncompletePolyMeshPopulation')
        for name,pin in pinned.items():
            require(exported['native_files'].get('openfoam_case/constant/polyMesh/'+name)==pin,'DetachedCompactNativeMeshPin')
        source_population={p.relative_to(folder).as_posix() for p in inventory(folder)}
        stage.mkdir(exist_ok=False);created_id=file_id(stage)
        # A strict initial handle covers creation of the pinned nonempty anchor.
        with Handle(stage):
            owner_pin=write(stage/'DELIVERY_OWNER.json',dict(schema='V35FreshCompactDirectory/1',nonce=uuid.uuid4().hex))
            scope.enter_context(guard(stage/'DELIVERY_OWNER.json'))
            require(sha(stage/'DELIVERY_OWNER.json')==owner_pin,'ChangedGeneratedCompactMetadata')
        hold_tree(stage,scope,strict_root=False)
        require(file_id(stage)==created_id,'ReplacedCompactStage')
        copies={**{'polyMesh/'+n:meshroot/n for n in sorted(actual)},'CASE.json':folder/'CASE.json'}
        expected_names=set(copies)|{'geometry.rhai','CHECKS.json','PROVENANCE.json','DELIVERY_OWNER.json','CONSUMER.json'}
        for name,source in copies.items():
            target=stage/name;target.parent.mkdir(parents=True,exist_ok=True);hold_parents(target.parent,scope)
            if target.parent!=stage:scope.enter_context(Handle(target.parent))
            checked(target,exists=False);copy_payload(source,target)
            scope.enter_context(guard(target))
            expected_pin=pinned[name[len('polyMesh/'):]] if name.startswith('polyMesh/') else root['completion_files']['CASE.json']
            require(sha(target)==expected_pin==sha(source),'ChangedCompactCopy')
        case=read(folder/'CASE.json');require(isinstance(case.get('geometry',{}).get('rhai'),str),'MissingCompactInlineGeometry')
        repair_path=member(folder,'background/SHARED_APEX_REPAIR.json')
        require(inner['files'].get('SHARED_APEX_REPAIR.json')==sha(repair_path),'UnboundCompactRepairAssurance')
        generated={}
        generated['geometry.rhai']=write_bytes(stage/'geometry.rhai',case['geometry']['rhai'].encode('utf-8'))
        generated['CHECKS.json']=write(stage/'CHECKS.json',summary(root,result,audit,read(repair_path)))
        generated['CONSUMER.json']=write(stage/'CONSUMER.json',consumer)
        generated['PROVENANCE.json']=write(stage/'PROVENANCE.json',dict(schema='V35CompactProvenance/1',source_bundle=str(folder),
            source_verification_sha256=expected,source_background_manifest_sha256=root['finalization']['manifest_sha256'],
            source_native_export_sha256=sha(export_receipt),geometry_units=case['geometry']['metres_per_model_unit'],
            source_cli=read(folder/'closure/SUPERVISOR.json')['command'] if root['verification_policy']=='production' else read(root['closure_records']['supervisor']['path'])['command'],
            compact_invocation=provenance,copy_method='byte_copy',existing_source_modified=False))
        for name,pin in generated.items():
            scope.enter_context(guard(stage/name))
            require(sha(stage/name)==pin,'ChangedGeneratedCompactMetadata')
        require({p.relative_to(stage).as_posix() for p in inventory(stage)}==expected_names,'UnexpectedCompactStagePopulation')
        files={p.relative_to(stage).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in inventory(stage)}
        value=dict(schema=SCHEMA,status='solver_delivery_complete',scientific_status=root['status'],
            source_verification_sha256=expected,source_background_manifest_sha256=root['finalization']['manifest_sha256'],
            source_native_export_sha256=sha(export_receipt),full_offline_replay_available=False,reports_are_replay_proofs=False,
            solver_mesh_files=sorted(actual),files=files,source_cleanup='not_performed_by_exporter',CFD_qualified=False)
        # Source population and trusted solver pins must still be identical.
        require({p.relative_to(folder).as_posix() for p in inventory(folder)}==source_population,'ChangedCompactSourcePopulation')
        require({p.relative_to(meshroot).as_posix() for p in inventory(meshroot)}==actual,'ChangedCompactSolverPopulation')
        require(all(sha(member(meshroot,n))==pin for n,pin in pinned.items()),'ChangedCompactSolverPins')
        require(sha(folder/'VERIFICATION.json')==expected,'ChangedCompactSourceBeforePublish')
        require(selected(destination)==current,'ChangedCompactConsumerPopulation')
        pin=publish(stage,value)
        verify(destination,pin)
    return dict(status='solver_delivery_complete',output=str(destination),manifest_sha256=pin,
        retained_logical_bytes=sum(p.stat().st_size for p in inventory(destination)),source_modified=False,full_offline_replay_available=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    q=sub.add_parser('export');q.add_argument('--bundle',type=Path,required=True);q.add_argument('--expected-root',required=True);q.add_argument('--output',type=Path,required=True)
    q.add_argument('--audit-result',type=Path);q.add_argument('--expected-audit-root')
    q=sub.add_parser('check');q.add_argument('--bundle',type=Path,required=True);q.add_argument('--expected-root',required=True)
    a=p.parse_args()
    result=export(a.bundle,a.expected_root,a.output,audit_path=a.audit_result,audit_expected=a.expected_audit_root,provenance=dict(command=__import__('sys').argv)) if a.action=='export' else verify(a.bundle,a.expected_root)
    print(json.dumps(result,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
