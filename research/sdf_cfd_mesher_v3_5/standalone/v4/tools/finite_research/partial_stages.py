"""Requested stage stops in the existing owned pipeline; no scientific issuer."""
from pathlib import Path
import json,os

STAGES=('seed','surface','prisms','complete')
DOWNSTREAM=('farfield_preparation','terminal_handoff','background','gmsh_bulk','full_publication','cold_full_consumer')

def choice(value):
    selected=value.get('stop_after','complete')
    if type(selected) is not str or selected not in STAGES:raise ValueError('InvalidRequestedStage')
    return selected

def read(path):return json.loads(Path(path).read_text(encoding='utf8'))

def sha(path):
    from hashlib import file_digest
    with Path(path).open('rb') as stream:return file_digest(stream,'sha256').hexdigest()

def seed_stage(case_path,output,through_layer):
    import standalone_tool as cli
    from standalone_case import load,copy_inputs
    from finite_research.prism_connection import require
    from general_sdf_binding import stock_seed
    from general_sdf_authority import GeneralSdfAuthority
    from path_preserving_deduplication import durable_json
    case,increments,pin=load(case_path)
    selection=require(case,increments,through_layer)
    if choice(selection)!='seed':raise ValueError('UnboundSeedStop')
    output=Path(output);output.mkdir(parents=True)
    copy_inputs(case_path,output/'CASE.json')
    folder=output/'seed';folder.mkdir()
    host,binding=cli.geometry(output/'CASE.json')
    try:
        if type(binding.authority) is not GeneralSdfAuthority or binding.seed_route!='CanonicalMarchingCubesSeed/1':
            raise ValueError('RequiredSourceAuthorizedStockSeedUnavailable')
        x,f,timing,spacing=stock_seed(binding.authority,case['general_sdf']['seed'],folder)
        cli.check_resources(output,case['resources'])
    finally:host.close()
    durable_json(output/'RESULT.json',dict(passed=True,proposal_only=True,last_qualified_layer=0,
        vertices=len(x),triangles=len(f),spacing_m=spacing,accepted_wall=False,
        source_coverage='NOT_PROVED',unresolved_regions='NOT_CLASSIFIED',
        construction_scope='Dense native metadata-box proposal; no global discovery or topology completeness claim',
        published_scientific_authority=False,restart_supported=False))

def complete(output,case_path,stop):
    from standalone_case import load
    from finite_research.prism_connection import require,_selection
    from owned_runtime import V4
    from path_preserving_deduplication import durable_json
    output=Path(output).resolve();case,increments,pin=load(case_path)
    selected=require(case,increments,len(increments))
    if stop=='complete' or choice(selected)!=stop:raise ValueError('UnboundPartialStageCompletion')
    result=read(output/'RESULT.json')
    if result.get('passed') is not True or (output/'STOP.json').exists():raise ValueError('FailedStageCannotComplete')
    if stop=='seed':
        if result.get('proposal_only') is not True or result.get('accepted_wall') is not False:raise ValueError('SeedProposalRequired')
        count=0
    elif stop=='surface':
        # Existing surface_stage already read back the wall under the live engine scope.
        # The retained receipt describes that completed check; it cannot reopen authority.
        if result.get('surface_readback_verified') is not True or result.get('surface_authority')!=read(output/'surface/A/fresh/SURFACE_AUTHORITY.json'):
            raise ValueError('MissingLiveSurfaceReadback')
        count=0
    else:
        count=result.get('last_qualified_layer')
        current=read(output/'CURRENT.json')
        if type(count) is not int or count!=len(increments) or current.get('layer')!=count or current.get('schema')!='V4FastProdMeshOnly/1':
            raise ValueError('RequestedPrismCountNotAccepted')
        if current.get('sha256')!=sha(output/'MESH.npz'):raise ValueError('ChangedAcceptedShellExport')
    for name in ('background','transition','DIRECT_SCOPE_EXIT.json','DIRECT_PUBLICATION_PREPARED.json','DIRECT_PUBLICATION_COMMITTED.json'):
        if (output/name).exists():raise ValueError('UnexpectedDownstreamStage',name)
    # The coordinator owns these mutable process observations through worker exit.
    # They remain retained separately; they are not mesh or acceptance artifacts.
    telemetry={'MONITOR.json','MONITOR.json.pending','PROCESS.json'}
    artifacts={p.relative_to(output).as_posix():sha(p) for p in sorted(output.rglob('*'))
        if p.relative_to(output).as_posix() not in telemetry and p.is_file()}
    receipt=dict(schema='V4RequestedStageCompletion/1',passed=True,requested_stop_after=stop,completed_stage=stop,
        pid=os.getpid(),requested_prism_layers=len(increments),accepted_prism_layers=count,
        case_sha256=pin,effective_case_identity=selected['upstream_effective_identity'],
        geometry_identity=selected['upstream_geometry_identity'],prism_schedule_identity=selected['prism_schedule_identity'],
        stage_request_sha256=_selection.get()[1],source_release_sha256=sha(V4/'runtime/RELEASE.json'),
        artifacts=artifacts,full_mesh_complete=False,solver_ready=False,normal_run_qualified=False,
        scientific_acceptance=False,published_scientific_authority=False,restart_supported=False,
        proposal_only=stop=='seed',accepted_wall=stop!='seed',
        downstream={name:'NOT_RUN_BY_REQUEST' for name in DOWNSTREAM})
    durable_json(output/'STAGE_COMPLETION.json',receipt)
    return validate(output,stop,pin,len(increments),receipt['source_release_sha256'],receipt['stage_request_sha256'])

def validate(root,stop,case_sha,count,release,request_sha):
    root=Path(root).resolve();value=read(root/'STAGE_COMPLETION.json')
    if (stop=='complete' or choice(dict(stop_after=stop))!=stop or value.get('schema')!='V4RequestedStageCompletion/1'
        or value.get('passed') is not True or value.get('requested_stop_after')!=stop or value.get('completed_stage')!=stop
        or value.get('case_sha256')!=case_sha or value.get('requested_prism_layers')!=count
        or type(value.get('accepted_prism_layers')) is not int or value['accepted_prism_layers']!=(count if stop=='prisms' else 0)
        or value.get('source_release_sha256')!=release or value.get('stage_request_sha256')!=request_sha
        or any(value.get(k) is not False for k in ('full_mesh_complete','solver_ready','normal_run_qualified','scientific_acceptance','published_scientific_authority','restart_supported'))
        or value.get('proposal_only') is not (stop=='seed') or value.get('accepted_wall') is not (stop!='seed')
        or value.get('downstream')!={name:'NOT_RUN_BY_REQUEST' for name in DOWNSTREAM}):
        raise ValueError('ContradictoryRequestedStageCompletion')
    if (root/'STOP.json').exists():raise ValueError('OutstandingPartialProducerFailure')
    result=read(root/'RESULT.json')
    if result.get('passed') is not True:raise ValueError('FailedPartialProducer')
    if stop=='prisms' and result.get('last_qualified_layer')!=count:raise ValueError('FewerThanRequestedLayers')
    if stop=='surface' and (result.get('surface_readback_verified') is not True or result.get('surface_authority')!=read(root/'surface/A/fresh/SURFACE_AUTHORITY.json')):
        raise ValueError('MissingLiveSurfaceReadback')
    if stop=='seed' and (result.get('proposal_only') is not True or result.get('accepted_wall') is not False):raise ValueError('SeedAcceptanceClaim')
    if not isinstance(value.get('artifacts'),dict) or not value['artifacts']:raise ValueError('MissingPartialArtifacts')
    required={'CASE.json','RESULT.json'} | ({'MESH.npz','CURRENT.json'} if stop=='prisms' else {'seed/seed.off','seed/SEED_EXTRACTION.json','seed/SEED_POINT_IDENTITY.json'} if stop=='seed' else {'surface/A/fresh/SURFACE_AUTHORITY.json','surface/A/fresh/SURFACE.npz','surface/A/fresh/AUDIT.json'})
    if not required<=set(value['artifacts']):raise ValueError('MissingRequiredPartialArtifacts')
    for name,digest in value['artifacts'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or sha(path)!=digest:raise ValueError('ChangedPartialArtifact',name)
    for name in ('background','transition','DIRECT_SCOPE_EXIT.json','DIRECT_PUBLICATION_PREPARED.json','DIRECT_PUBLICATION_COMMITTED.json'):
        if (root/name).exists():raise ValueError('UnexpectedDownstreamStage',name)
    if stop=='surface':
        from surface_requirements195 import verify_numerical_evidence
        verify_numerical_evidence(read(root/'CASE.json'),read(root/'surface/A/fresh/AUDIT.json'),
            geometry_identity=read(root/'surface/A/fresh/SURFACE_AUTHORITY.json')['geometry_identity'])
    if stop=='prisms':
        from owned_source_segments import enabled,verify_archive
        from owned_runtime import V4
        if enabled() and release==sha(V4/'runtime/RELEASE.json'):
            proof=verify_archive(root,release,mesh_required=True)
            archive=read(root/'INITIAL_SUMMARY.json').get('owned_source_segments',{})
            if (archive.get('path')!='initial/SOURCE_SEGMENT_EVIDENCE'
                or archive.get('archive_sha256')!=sha(root/'initial/SOURCE_SEGMENT_EVIDENCE/ARCHIVE.json')
                or archive.get('chain_sha256')!=proof['receipt_summary']['chain_sha256']):raise ValueError('DetachedInitialSegmentArchive')
    return value

def external(captured,context,root,manifest,code):
    # Caller has checked the actual terminal Windows handle and process/source completion.
    if code!=0 or captured.get('root')!=str(root) or captured.get('publication_state')!='NOT_RUN_BY_REQUEST':raise ValueError('InvalidPartialExternalOutcome')
    packet=Path(root).parent.parent
    planned=read(packet/'LIVE_REQUEST.json');stage=read(packet/'STAGE_REQUEST.json')
    for p in (packet/'LIVE_REQUEST.json',packet/'STAGE_REQUEST.json'):
        if manifest.get(str(p))!=sha(p):raise ValueError('UnboundExternalPartialRequest')
    if planned.get('publication_enabled') is not False or planned['stop_after']!=stage['stop_after']:raise ValueError('PartialPublicationClaim')
    for name,digest in captured['retirement_files'].items():
        if sha(name)!=digest:raise ValueError('ChangedExternalPartialRetirement')
    if captured.get('stage_completion_sha256')!=sha(root/'STAGE_COMPLETION.json'):raise ValueError('ChangedExternalStageCompletion')
    value=validate(root,planned['stop_after'],planned['case_sha256'],stage['upstream_flags']['through_layer'],planned['source_release_sha256'],sha(packet/'STAGE_REQUEST.json'))
    if captured.get('requested_stop_after')!=value['requested_stop_after'] or captured.get('full_mesh_complete') is not False:raise ValueError('ChangedExternalStageOutcome')
    return value
